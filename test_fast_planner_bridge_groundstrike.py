#!/usr/bin/env python3
"""
Cutoff-hardening verification, item 3: deliberately drive the vehicle's
COMMANDED position below the new hard_killswitch.py lower-altitude floor
(ALTITUDE_LOWER_LIMIT_M = -0.5m NED-altitude, i.e. z > 0.5 NED) and confirm
the killswitch fires (SIGKILL bridge -> force-disarm) instead of letting
the vehicle actually strike the ground.

Reuses the exact proven arm/offboard/climb boilerplate from
fast_planner_bridge.py (home captured at confirmed-arm time, relative
climb target, 20Hz OffboardControlMode heartbeat, NaN-vel/acc-safe
position-only setpoints). After a genuine climb to 5m and a stabilize
hold, it explicitly commands a NED z target 2.0m BELOW home (i.e.
commanded altitude -2.0m, well past the -0.5m floor and past the real
-1.68m excursion from the receding-horizon flight test) and just holds
there -- unlike fast_planner_bridge.py's real flight relay, there is no
plan-following here, only a deliberate bad setpoint, so the killswitch's
own kill path is the only thing standing between this command and an
actual ground strike. Isolated pure-SITL (SIH) use only -- no Gazebo, no
perception stack.
"""

import math
import signal
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)

CLIMB_OFFSET_NED = (0.0, 0.0, -5.0)       # genuine 5m climb, matches proven-safe regime
CLIMB_ARRIVAL_TOLERANCE_M = 0.3
STABILIZE_TICKS = 60                       # 3s @ 20Hz hold at altitude before the bad command
GROUND_STRIKE_OFFSET_NED = (0.0, 0.0, 2.0)  # 2m BELOW home -- past the -0.5m floor


class TestGroundStrike(Node):
    def __init__(self) -> None:
        super().__init__("test_ground_strike")

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

        self.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position_v1", self._pos_cb, sub_qos)
        self.create_subscription(VehicleStatus, "/fmu/out/vehicle_status_v4", self._status_cb, sub_qos)

        self._local_position = VehicleLocalPosition()
        self._vehicle_status = VehicleStatus()
        self._pos_received = False
        self._tick = 0
        self._tick_enter = 0
        self._confirmed_armed = False
        self._climbed = False
        self._stabilize_count = 0
        self._struck = False
        self._home_ned = None

        self._shutdown_requested = False
        signal.signal(signal.SIGINT, self._handle_shutdown_signal)
        signal.signal(signal.SIGTERM, self._handle_shutdown_signal)

        self._timer = self.create_timer(0.05, self._control_loop)  # 20 Hz
        self.get_logger().info(
            "test_ground_strike ready -- will arm, climb 5m, stabilize, then "
            f"deliberately command NED z = home_z + {GROUND_STRIKE_OFFSET_NED[2]}m "
            "(2m below home) to exercise hard_killswitch.py's new lower-altitude bound."
        )

    def _handle_shutdown_signal(self, signum, frame) -> None:
        self._shutdown_requested = True
        self.get_logger().warn(f"Signal {signum} received -- stopping setpoint stream, exiting.")

    def _pos_cb(self, msg: VehicleLocalPosition) -> None:
        self._local_position = msg
        self._pos_received = True

    def _status_cb(self, msg: VehicleStatus) -> None:
        self._vehicle_status = msg

    def _climb_target_ned(self) -> tuple:
        hx, hy, hz = self._home_ned
        ox, oy, oz = CLIMB_OFFSET_NED
        return (hx + ox, hy + oy, hz + oz)

    def _strike_target_ned(self) -> tuple:
        hx, hy, hz = self._home_ned
        ox, oy, oz = GROUND_STRIKE_OFFSET_NED
        return (hx + ox, hy + oy, hz + oz)

    def _control_loop(self) -> None:
        self._tick += 1

        if self._shutdown_requested:
            return

        if not self._pos_received:
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
            self._send_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
            self._publish_hold_at_current()
            if self._tick_enter % 20 == 0:
                self.get_logger().info(f"Requesting OFFBOARD... nav_state={nav_state}")
            return

        if arming_state != ARMED:
            self._send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
            self._publish_hold_at_current()
            if self._tick_enter % 20 == 0:
                self.get_logger().info(f"Requesting ARM... arming_state={arming_state}")
            return

        if not self._confirmed_armed:
            self._confirmed_armed = True
            self._home_ned = (self._local_position.x, self._local_position.y, self._local_position.z)
            self.get_logger().info(
                f"OFFBOARD + ARMED confirmed -- home_ned={self._home_ned}, "
                f"climbing to {self._climb_target_ned()}."
            )

        if not self._climbed:
            x, y, z = self._climb_target_ned()
            self._publish_trajectory_setpoint(x, y, z)
            dx = x - self._local_position.x
            dy = y - self._local_position.y
            dz = z - self._local_position.z
            dist = math.sqrt(dx * dx + dy * dy + dz * dz)
            if dist < CLIMB_ARRIVAL_TOLERANCE_M:
                self._climbed = True
                self.get_logger().info(f"Climb complete (dist={dist:.2f}m) -- stabilizing before ground-strike command.")
            if self._tick % 20 == 0:
                self.get_logger().info(
                    f"[CLIMBING] pos_ned=({self._local_position.x:.2f},{self._local_position.y:.2f},"
                    f"{self._local_position.z:.2f}) dist={dist:.2f}m"
                )
            return

        if self._stabilize_count < STABILIZE_TICKS:
            self._stabilize_count += 1
            x, y, z = self._climb_target_ned()
            self._publish_trajectory_setpoint(x, y, z)
            return

        if not self._struck:
            self._struck = True
            self.get_logger().warn(
                f"COMMANDING GROUND-STRIKE SETPOINT NOW: target_ned={self._strike_target_ned()} "
                "(expect hard_killswitch.py to fire: kill bridge process, force-disarm)"
            )

        x, y, z = self._strike_target_ned()
        self._publish_trajectory_setpoint(x, y, z)
        if self._tick % 10 == 0:
            self.get_logger().info(
                f"[GROUND-STRIKE HOLD] pos_ned=({self._local_position.x:.2f},{self._local_position.y:.2f},"
                f"{self._local_position.z:.2f}) target_ned=({x:.2f},{y:.2f},{z:.2f}) "
                f"arming_state={arming_state}"
            )

    def _publish_hold_at_current(self) -> None:
        if not self._pos_received:
            self._publish_trajectory_setpoint(math.nan, math.nan, math.nan)
            return
        self._publish_trajectory_setpoint(self._local_position.x, self._local_position.y, self._local_position.z)

    def _offboard_mode_msg(self) -> OffboardControlMode:
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        return msg

    def _publish_trajectory_setpoint(self, x: float, y: float, z: float) -> None:
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = [float(x), float(y), float(z)]
        msg.velocity = [math.nan] * 3
        msg.acceleration = [math.nan] * 3
        msg.yaw = 0.0
        msg.yawspeed = math.nan
        self.offboard_pub.publish(self._offboard_mode_msg())
        self.traj_pub.publish(msg)

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
    node = TestGroundStrike()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
