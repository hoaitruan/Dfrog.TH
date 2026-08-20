#!/usr/bin/env python3
"""
Phase 1 gate: arm, switch to Offboard, climb to a fixed point, hold.

Uses PX4's own (simulated) state estimate -- no cuVSLAM, no EKF2 external
vision yet (that's Phase 4). Same proven arm/offboard/heartbeat pattern as
~/Drone/ros2_ws/src/drone/controller/pid_takeoff.py and
~/Drone/realtimeavoid's offboard_trajectory_follower.py, just position
control this time instead of velocity-only.
"""

import math
import os
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

# Target: hold 5m above the arming point (NED, negative = up).
TARGET_NED = (0.0, 0.0, -5.0)
ARRIVAL_TOLERANCE_M = 0.3


class MinimalOffboard(Node):
    def __init__(self) -> None:
        super().__init__("minimal_offboard")

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

        self._local_position = VehicleLocalPosition()
        self._vehicle_status = VehicleStatus()
        self._pos_received = False
        self._status_received = False
        self._tick = 0
        self._tick_enter = 0
        self._confirmed_armed = False
        self._target_reached = False

        auto_start = os.getenv("OFFBOARD_AUTO_START", "").lower() in {"1", "true", "yes", "y"}
        self._start_allowed = auto_start
        if not auto_start:
            threading.Thread(target=self._wait_for_enter, daemon=True).start()

        self._timer = self.create_timer(0.05, self._control_loop)  # 20 Hz

        self.get_logger().info(f"minimal_offboard ready -- target NED {TARGET_NED}")
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

    def _control_loop(self) -> None:
        self._tick += 1
        self.offboard_pub.publish(self._offboard_mode_msg())

        if not self._pos_received:
            if self._tick % 40 == 0:
                self.get_logger().warn(
                    f"[tick {self._tick}] No data on /fmu/out/vehicle_local_position_v1 -- "
                    "verify MicroXRCEAgent is running and PX4 has connected to it."
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
            self.get_logger().info("OFFBOARD + ARMED confirmed -- flying to target")

        x, y, z = TARGET_NED
        self._publish_trajectory_setpoint(x, y, z, yaw=0.0)

        dx = x - self._local_position.x
        dy = y - self._local_position.y
        dz = z - self._local_position.z
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)
        if dist < ARRIVAL_TOLERANCE_M and not self._target_reached:
            self._target_reached = True
            self.get_logger().info(f"Target reached (dist={dist:.2f}m) -- holding.")

        if self._tick % 20 == 0:
            status = "HOLDING" if self._target_reached else "EN ROUTE"
            self.get_logger().info(
                f"[{status}] pos_ned=({self._local_position.x:.2f},{self._local_position.y:.2f},"
                f"{self._local_position.z:.2f}) target={TARGET_NED} dist={dist:.2f}m"
            )

    def _publish_hold_at_current(self) -> None:
        if not self._pos_received:
            self._publish_trajectory_setpoint(math.nan, math.nan, math.nan, math.nan)
            return
        self._publish_trajectory_setpoint(
            self._local_position.x, self._local_position.y, self._local_position.z, 0.0
        )

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
    node = MinimalOffboard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
