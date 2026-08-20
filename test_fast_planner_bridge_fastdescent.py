#!/usr/bin/env python3
"""
Descent-rate guard verification, item 4: deliberately command a fast,
direct downward VELOCITY setpoint (not a position step) and confirm
hard_killswitch.py's new DESCENT_RATE_LIMIT_MPS guard fires while the
vehicle is still comfortably above the ALTITUDE_LOWER_LIMIT_M (-0.5m)
floor -- catching a fast dive earlier than the position floor could.

Why velocity, not position: the position-only step-command pattern
(test_fast_planner_bridge_groundstrike.py) only produced ~1.3-1.6 m/s
descent via PX4's own position-error-driven acceleration -- gentler than
the ~2.5-3.0 m/s real runaway this guard exists to catch, and it crossed
1.5 m/s only after already dropping below the 1.0m gate altitude,
so the position floor fired first (confirmed empirically). Commanding
DESCENT_TARGET_VZ_MPS directly as a velocity setpoint (position fields
NaN) removes that indirection and reliably produces a known, fast,
deterministic descent rate from the very first tick after climb+stabilize
-- a fair, direct test of the guard.

Isolated pure-SITL (SIH) use only -- no Gazebo, no perception stack.
"""

import math
import signal

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
STABILIZE_TICKS = 60                       # 3s @ 20Hz hold at altitude before the dive
DESCENT_TARGET_VZ_MPS = 3.0                # NED: positive = down. Above the 1.5 m/s guard,
                                            # below what a real vehicle can physically achieve.


class TestFastDescent(Node):
    def __init__(self) -> None:
        super().__init__("test_fast_descent")

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
        self._diving = False
        self._home_ned = None

        self._shutdown_requested = False
        signal.signal(signal.SIGINT, self._handle_shutdown_signal)
        signal.signal(signal.SIGTERM, self._handle_shutdown_signal)

        self._timer = self.create_timer(0.05, self._control_loop)  # 20 Hz
        self.get_logger().info(
            "test_fast_descent ready -- will arm, climb 5m, stabilize, then "
            f"command a direct vz={DESCENT_TARGET_VZ_MPS} m/s downward velocity setpoint "
            "to exercise hard_killswitch.py's new descent-rate guard."
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
            self._publish_position_setpoint(x, y, z)
            dx = x - self._local_position.x
            dy = y - self._local_position.y
            dz = z - self._local_position.z
            dist = math.sqrt(dx * dx + dy * dy + dz * dz)
            if dist < CLIMB_ARRIVAL_TOLERANCE_M:
                self._climbed = True
                self.get_logger().info(f"Climb complete (dist={dist:.2f}m) -- stabilizing before fast-descent command.")
            if self._tick % 20 == 0:
                self.get_logger().info(
                    f"[CLIMBING] pos_ned=({self._local_position.x:.2f},{self._local_position.y:.2f},"
                    f"{self._local_position.z:.2f}) dist={dist:.2f}m"
                )
            return

        if self._stabilize_count < STABILIZE_TICKS:
            self._stabilize_count += 1
            x, y, z = self._climb_target_ned()
            self._publish_position_setpoint(x, y, z)
            return

        if not self._diving:
            self._diving = True
            self.get_logger().warn(
                f"COMMANDING FAST-DESCENT VELOCITY SETPOINT NOW: vz={DESCENT_TARGET_VZ_MPS} m/s "
                "(expect hard_killswitch.py's descent-rate guard to fire before the -0.5m floor)"
            )

        self._publish_velocity_setpoint(0.0, 0.0, DESCENT_TARGET_VZ_MPS)
        if self._tick % 4 == 0:  # ~5Hz logging -- fine enough to see the dive profile
            self.get_logger().info(
                f"[FAST-DESCENT] pos_ned=({self._local_position.x:.2f},{self._local_position.y:.2f},"
                f"{self._local_position.z:.2f}) vz_actual={self._local_position.vz:.2f} "
                f"vz_cmd={DESCENT_TARGET_VZ_MPS} arming_state={arming_state}"
            )

    def _publish_hold_at_current(self) -> None:
        if not self._pos_received:
            self._publish_position_setpoint(math.nan, math.nan, math.nan)
            return
        self._publish_position_setpoint(self._local_position.x, self._local_position.y, self._local_position.z)

    def _offboard_mode_msg(self, velocity: bool) -> OffboardControlMode:
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = not velocity
        msg.velocity = velocity
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        return msg

    def _publish_position_setpoint(self, x: float, y: float, z: float) -> None:
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = [float(x), float(y), float(z)]
        msg.velocity = [math.nan] * 3
        msg.acceleration = [math.nan] * 3
        msg.yaw = 0.0
        msg.yawspeed = math.nan
        self.offboard_pub.publish(self._offboard_mode_msg(velocity=False))
        self.traj_pub.publish(msg)

    def _publish_velocity_setpoint(self, vx: float, vy: float, vz: float) -> None:
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = [math.nan] * 3
        msg.velocity = [float(vx), float(vy), float(vz)]
        msg.acceleration = [math.nan] * 3
        msg.yaw = 0.0
        msg.yawspeed = math.nan
        self.offboard_pub.publish(self._offboard_mode_msg(velocity=True))
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
    node = TestFastDescent()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
