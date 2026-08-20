#!/usr/bin/env python3
"""
Guaranteed subscriber for nvblox's lazy-published ESDF pointcloud.

nvblox only computes/publishes /nvblox_node/static_esdf_pointcloud if it
sees a matched subscriber at publish time. The flight_20260812_202601 bag
recorded this topic with Count: 0 despite being in record.sh's topic list --
ros2 bag record's own subscription either registered too late or, more
likely, used a QoS (RELIABLE by default) that never matched nvblox's
publisher (nvblox point cloud publishers commonly use BEST_EFFORT sensor
QoS -- the same class of silent QoS-mismatch bug already hit twice in this
project: fast_planner_bridge.py's /fmu/out subs, health_check.py's
PX4_OUT_QOS). Run this alongside record.sh so a real, QoS-compatible
subscriber exists for the whole recording, independent of whatever QoS
ros2 bag record negotiates on its own.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2

ESDF_TOPIC = "/nvblox_node/static_esdf_pointcloud"

ESDF_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)


def main() -> None:
    rclpy.init()
    node = Node("esdf_keepalive")
    count = 0

    def cb(msg: PointCloud2) -> None:
        nonlocal count
        count += 1
        if count == 1 or count % 20 == 0:
            node.get_logger().info(f"esdf_keepalive: received msg #{count} on {ESDF_TOPIC}")

    node.create_subscription(PointCloud2, ESDF_TOPIC, cb, ESDF_QOS)
    node.get_logger().info(f"esdf_keepalive: subscribed to {ESDF_TOPIC}, holding to trigger nvblox lazy-publish")
    rclpy.spin(node)


if __name__ == "__main__":
    main()
