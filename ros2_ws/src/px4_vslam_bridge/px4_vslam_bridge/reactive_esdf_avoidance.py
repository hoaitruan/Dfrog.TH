#!/usr/bin/env python3
"""
Phase 6 MILESTONE: reactive obstacle avoidance driven by nvblox's real ESDF.

Architecture: a fast 20Hz loop keeps streaming OffboardControlMode +
TrajectorySetpoint toward a "micro-waypoint" (required -- PX4 exits
Offboard if setpoints stop). A slower (~3Hz) reactive-planning step fires
an ASYNC get_esdf_and_gradient query (never blocking -- see Phase 5's
project-memory notes on why a blocking call here would deadlock nvblox's
executor) around the drone's current position, and its completion
callback recomputes the micro-waypoint via a classic potential-field
combination: attract toward the goal, repel from the single closest
known-distance voxel in the queried region.

Frame handling: PX4's own NED local position (/fmu/out/vehicle_local_position_v1,
GPS-based -- EKF2_EV_CTRL=0 as of this phase, see the airframe file's
comment on why cuVSLAM's vision isn't fed into EKF2 for flight control
here) is converted to ENU via frame_transforms for both the goal-tracking
math and the nvblox query frame_id="map". "map" is exactly PX4's own NED
origin re-expressed in ENU (ground_truth_tf.py's dynamic map->base_link_gt
transform comes from this same vehicle_odometry topic), so this ENU
conversion and nvblox's "map" frame are already the same frame by
construction, not an approximation. The final micro-waypoint is converted
back to NED immediately before publishing.
"""

import math
import os
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import Point, Vector3
from nvblox_msgs.srv import EsdfAndGradients
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)

from px4_vslam_bridge.frame_transforms import ned_to_enu_position, enu_to_ned_position

# Climb straight up first (proven stable in Phases 1/3/4), THEN switch to
# reactive goal-seeking -- keeps the takeoff phase decoupled from
# avoidance, matching how every earlier phase validated flight behavior
# in isolation before adding the next piece.
CLIMB_TARGET_NED = (0.0, 0.0, -5.0)
CLIMB_ARRIVAL_TOLERANCE_M = 0.3

# Goal in ENU (odom-ish) frame: well past the pillar ring (pillars sit on
# a 2.5m-radius ring around the origin, see vio_test.sdf) so a straight
# line to it passes directly through pillar_01 at (2.5, 0, ~5) --
# guaranteeing the avoidance logic actually gets exercised, not just a
# clear-path flight.
GOAL_ENU = (7.0, 0.0, 5.0)
GOAL_ARRIVAL_TOLERANCE_M = 0.5

# Potential-field tuning. Repulsion uses 1/d^2 (classic), clamped to a
# max magnitude so a very-close voxel can't demand an unbounded step.
QUERY_AABB_HALF_SIZE_M = 1.5  # 3m cube centered on the drone
ATTRACTION_GAIN = 1.0
REPULSION_GAIN = 2.5
REPULSION_INFLUENCE_RADIUS_M = 2.0  # ignore obstacles farther than this
MAX_STEP_M = 0.35  # per reactive-planning tick (~3Hz -> ~1 m/s cap)
ESDF_UNKNOWN_THRESHOLD = 500.0  # sentinel values are +-1000
REACTIVE_PLAN_PERIOD_S = 0.3


class ReactiveEsdfAvoidance(Node):
    def __init__(self) -> None:
        super().__init__("reactive_esdf_avoidance")

        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.offboard_pub = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", pub_qos)
        self.traj_pub = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", pub_qos)
        self.cmd_pub = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", pub_qos)

        self.create_subscription(
            VehicleLocalPosition, "/fmu/out/vehicle_local_position_v1", self._pos_cb, sub_qos
        )
        self.create_subscription(
            VehicleStatus, "/fmu/out/vehicle_status_v4", self._status_cb, sub_qos
        )

        self.esdf_client = self.create_client(EsdfAndGradients, "/nvblox_node/get_esdf_and_gradient")

        self._local_position = VehicleLocalPosition()
        self._vehicle_status = VehicleStatus()
        self._pos_received = False
        self._status_received = False
        self._tick = 0
        self._tick_enter = 0
        self._confirmed_armed = False
        self._climbed = False
        self._goal_reached = False

        # Micro-waypoint in NED, published every fast-loop tick. Starts
        # as None (hold current position) until the first reactive plan
        # or climb target is available.
        self._micro_waypoint_ned = None
        self._esdf_query_in_flight = False
        self._last_closest_obstacle_dist = None

        auto_start = os.getenv("OFFBOARD_AUTO_START", "").lower() in {"1", "true", "yes", "y"}
        self._start_allowed = auto_start
        if not auto_start:
            threading.Thread(target=self._wait_for_enter, daemon=True).start()

        self._fast_timer = self.create_timer(0.05, self._fast_control_loop)  # 20 Hz
        self._reactive_timer = self.create_timer(REACTIVE_PLAN_PERIOD_S, self._reactive_plan_step)

        self.get_logger().info(
            f"reactive_esdf_avoidance ready -- climb to {CLIMB_TARGET_NED} NED, "
            f"then reactively fly to goal {GOAL_ENU} ENU"
        )
        self.get_logger().info(">>> Press ENTER to arm and fly (or set OFFBOARD_AUTO_START=1) <<<")

    def _wait_for_enter(self) -> None:
        try:
            input()
        except EOFError:
            return
        self._start_allowed = True
        self.get_logger().info("ENTER pressed -- arming and taking off!")

    def _pos_cb(self, msg: VehicleLocalPosition) -> None:
        self._local_position = msg
        self._pos_received = True

    def _status_cb(self, msg: VehicleStatus) -> None:
        self._vehicle_status = msg
        self._status_received = True

    # -- Fast loop: streams setpoints, runs the arm/offboard state machine --

    def _fast_control_loop(self) -> None:
        self._tick += 1
        self.offboard_pub.publish(self._offboard_mode_msg())

        if not self._pos_received:
            if self._tick % 40 == 0:
                self.get_logger().warn(
                    f"[tick {self._tick}] No data on /fmu/out/vehicle_local_position_v1.",
                )
            self._publish_hold_at_current()
            return

        if not self._start_allowed:
            self._publish_hold_at_current()
            return

        self._tick_enter += 1
        if self._tick_enter < 50:  # 2.5s pre-stream before mode switch
            self._publish_hold_at_current()
            return

        nav_state = self._vehicle_status.nav_state
        arming_state = self._vehicle_status.arming_state
        OFFBOARD_MODE, ARMED = 14, 2

        if nav_state != OFFBOARD_MODE:
            self._set_offboard_mode()
            self._publish_hold_at_current()
            if self._tick_enter % 20 == 0:
                self.get_logger().info(f"Requesting OFFBOARD... nav_state={nav_state}")
            return

        if arming_state != ARMED:
            self._arm()
            self._publish_hold_at_current()
            if self._tick_enter % 20 == 0:
                self.get_logger().info(f"Requesting ARM... arming_state={arming_state}")
            return

        if not self._confirmed_armed:
            self._confirmed_armed = True
            self.get_logger().info("OFFBOARD + ARMED confirmed -- climbing.")

        if not self._climbed:
            x, y, z = CLIMB_TARGET_NED
            self._publish_trajectory_setpoint(x, y, z, yaw=0.0)
            dx = x - self._local_position.x
            dy = y - self._local_position.y
            dz = z - self._local_position.z
            dist = math.sqrt(dx * dx + dy * dy + dz * dz)
            if dist < CLIMB_ARRIVAL_TOLERANCE_M:
                self._climbed = True
                self._micro_waypoint_ned = CLIMB_TARGET_NED
                self.get_logger().info(f"Climb complete (dist={dist:.2f}m) -- starting reactive avoidance.")
            if self._tick % 20 == 0:
                self.get_logger().info(
                    f"[CLIMBING] pos_ned=({self._local_position.x:.2f},{self._local_position.y:.2f},"
                    f"{self._local_position.z:.2f}) dist={dist:.2f}m"
                )
            return

        # Reactive phase: stream whatever micro-waypoint the reactive
        # planner last computed (falls back to holding current position
        # if a plan hasn't landed yet, e.g. first tick after climb).
        if self._micro_waypoint_ned is None:
            self._publish_hold_at_current()
            return

        x, y, z = self._micro_waypoint_ned
        self._publish_trajectory_setpoint(x, y, z, yaw=0.0)

        cur_enu = ned_to_enu_position(
            self._local_position.x, self._local_position.y, self._local_position.z
        )
        goal_dist = math.sqrt(sum((g - c) ** 2 for g, c in zip(GOAL_ENU, cur_enu)))
        if goal_dist < GOAL_ARRIVAL_TOLERANCE_M and not self._goal_reached:
            self._goal_reached = True
            self.get_logger().info(f"GOAL REACHED (dist={goal_dist:.2f}m) -- holding.")

        if self._tick % 20 == 0:
            status = "AT GOAL" if self._goal_reached else "REACTIVE NAV"
            obs = (
                f"{self._last_closest_obstacle_dist:.2f}m"
                if self._last_closest_obstacle_dist is not None
                else "none seen"
            )
            self.get_logger().info(
                f"[{status}] pos_enu=({cur_enu[0]:.2f},{cur_enu[1]:.2f},{cur_enu[2]:.2f}) "
                f"goal_dist={goal_dist:.2f}m nearest_obstacle={obs}"
            )

    def _publish_hold_at_current(self) -> None:
        if not self._pos_received:
            self._publish_trajectory_setpoint(math.nan, math.nan, math.nan, math.nan)
            return
        self._publish_trajectory_setpoint(
            self._local_position.x, self._local_position.y, self._local_position.z, 0.0
        )

    # -- Reactive planning: async ESDF query + potential-field step --

    def _reactive_plan_step(self) -> None:
        if not (self._climbed and self._confirmed_armed) or self._goal_reached:
            return
        if self._esdf_query_in_flight:
            return
        if not self.esdf_client.service_is_ready():
            return

        cur_enu = ned_to_enu_position(
            self._local_position.x, self._local_position.y, self._local_position.z
        )

        req = EsdfAndGradients.Request()
        req.update_esdf = True
        req.visualize_esdf = False
        req.use_aabb = True
        req.frame_id = "map"
        h = QUERY_AABB_HALF_SIZE_M
        req.aabb_min_m = Point(x=cur_enu[0] - h, y=cur_enu[1] - h, z=cur_enu[2] - h)
        req.aabb_size_m = Vector3(x=2 * h, y=2 * h, z=2 * h)

        self._esdf_query_in_flight = True
        future = self.esdf_client.call_async(req)
        future.add_done_callback(lambda f: self._on_esdf_response(f, cur_enu))

    def _on_esdf_response(self, future, cur_enu) -> None:
        self._esdf_query_in_flight = False
        try:
            res = future.result()
        except Exception as exc:  # noqa: BLE001 -- log and skip this cycle
            self.get_logger().warn(f"ESDF query failed: {exc}", throttle_duration_sec=2.0)
            return

        if res is None or not res.success:
            return

        closest_pos_enu, closest_dist = self._find_closest_obstacle(res, cur_enu)
        self._last_closest_obstacle_dist = closest_dist

        to_goal = np.array(GOAL_ENU) - np.array(cur_enu)
        goal_dist = np.linalg.norm(to_goal)
        attractive = np.zeros(3) if goal_dist < 1e-6 else (to_goal / goal_dist) * ATTRACTION_GAIN

        repulsive = np.zeros(3)
        if closest_pos_enu is not None and closest_dist < REPULSION_INFLUENCE_RADIUS_M:
            away = np.array(cur_enu) - np.array(closest_pos_enu)
            away_norm = np.linalg.norm(away)
            if away_norm > 1e-6:
                # Clamp the effective distance so a voxel center coincident
                # with (or inside) the drone can't blow up the 1/d^2 term.
                d = max(closest_dist, 0.15)
                repulsive = (away / away_norm) * (REPULSION_GAIN / (d * d))

        resultant = attractive + repulsive
        resultant_norm = np.linalg.norm(resultant)
        if resultant_norm < 1e-6:
            return
        step = (resultant / resultant_norm) * min(MAX_STEP_M, resultant_norm * MAX_STEP_M)

        next_enu = np.array(cur_enu) + step
        next_ned = enu_to_ned_position(next_enu[0], next_enu[1], next_enu[2])
        self._micro_waypoint_ned = next_ned

    @staticmethod
    def _find_closest_obstacle(res: EsdfAndGradients.Response, cur_enu):
        dims = res.esdf_and_gradients.layout.dim
        if len(dims) != 3:
            return None, None
        size_x, size_y, size_z = dims[0].size, dims[1].size, dims[2].size
        if size_x * size_y * size_z == 0:
            return None, None

        data = np.array(res.esdf_and_gradients.data, dtype=np.float32).reshape(size_x, size_y, size_z)
        known_mask = np.abs(data) < ESDF_UNKNOWN_THRESHOLD
        if not np.any(known_mask):
            return None, None

        # Distance to nearest *surface* -- want the smallest absolute
        # value (closest to zero-crossing), not the smallest signed value
        # (which would chase deeply-negative inside-obstacle voxels that
        # aren't necessarily the nearest surface point).
        abs_data = np.where(known_mask, np.abs(data), np.inf)
        flat_idx = int(np.argmin(abs_data))
        i, j, k = np.unravel_index(flat_idx, (size_x, size_y, size_z))
        dist = float(data[i, j, k])

        voxel_size = res.voxel_size_m
        origin = res.origin_m
        pos_enu = (
            origin.x + (i + 0.5) * voxel_size,
            origin.y + (j + 0.5) * voxel_size,
            origin.z + (k + 0.5) * voxel_size,
        )
        return pos_enu, abs(dist)

    # -- Offboard/arm boilerplate (identical pattern to minimal_offboard.py) --

    def _offboard_mode_msg(self) -> OffboardControlMode:
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        return msg

    def _publish_trajectory_setpoint(self, x: float, y: float, z: float, yaw: float) -> None:
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = [float(x), float(y), float(z)]
        msg.velocity = [math.nan, math.nan, math.nan]
        msg.yaw = float(yaw)
        self.traj_pub.publish(msg)

    def _arm(self) -> None:
        self._send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)

    def _set_offboard_mode(self) -> None:
        self._send_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)

    def _send_vehicle_command(self, command: int, **kwargs) -> None:
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        for k, v in kwargs.items():
            setattr(msg, k, float(v))
        self.cmd_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ReactiveEsdfAvoidance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
