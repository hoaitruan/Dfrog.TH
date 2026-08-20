#!/usr/bin/env python3
"""
Sends PX4's force-disarm command (VEHICLE_CMD_COMPONENT_ARM_DISARM,
param1=0.0 disarm, param2=21196.0 force-magic) so the vehicle can be
disarmed while still airborne/hovering, since this project has no
autoland/RTL step -- a normal disarm request is refused by PX4 while
flying. Used to end a supervised test flight cleanly after the planned
trajectory completes and holds.
"""
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleCommand

PUB_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


def main() -> None:
    rclpy.init()
    node = Node("force_disarm")
    pub = node.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", PUB_QOS)
    time.sleep(1.0)  # let the publisher match before sending

    msg = VehicleCommand()
    msg.timestamp = int(node.get_clock().now().nanoseconds / 1000)
    msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
    msg.param1 = 0.0
    msg.param2 = 21196.0
    msg.target_system = 1
    msg.target_component = 1
    msg.source_system = 1
    msg.source_component = 1
    msg.from_external = True

    for _ in range(3):
        pub.publish(msg)
        time.sleep(0.2)

    node.get_logger().info("force-disarm command sent")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
