#!/usr/bin/env python3
"""
Continuous position/clearance logger for the obstacle-avoidance flight.

Started BEFORE arming, left running through touchdown, so the entire
flight -- including the pillar-transit window -- is captured at full
odometry rate. Fixes the previous flight's gap, where periodic
`sleep 1; ros2 topic echo --once` polling started ~30s after trigger and
missed the transit entirely.

Subscribes /ground_truth/odom directly (no polling), so every message is
processed, not sampled. For each message, computes the clamped-rectangle
distance from the drone's (x,y) to pillar_03's footprint (world pose
(0,2.5,5), 0.4x0.4x10m box -> x in [-0.2,0.2], y in [2.3,2.7], vio_test
.sdf:226-229) and appends one CSV row: timestamp, x, y, z, distance.
"""
import csv
import sys
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

PILLAR_X_MIN, PILLAR_X_MAX = -0.2, 0.2
PILLAR_Y_MIN, PILLAR_Y_MAX = 2.3, 2.7


def clamped_distance(x: float, y: float) -> float:
    cx = min(max(x, PILLAR_X_MIN), PILLAR_X_MAX)
    cy = min(max(y, PILLAR_Y_MIN), PILLAR_Y_MAX)
    return ((x - cx) ** 2 + (y - cy) ** 2) ** 0.5


class ClearanceLogger(Node):
    def __init__(self, csv_path: str) -> None:
        super().__init__("clearance_logger")
        self._f = open(csv_path, "w", newline="")
        self._writer = csv.writer(self._f)
        self._writer.writerow(["t_wall", "t_msg", "x", "y", "z", "dist_to_pillar"])
        self._count = 0
        self.create_subscription(Odometry, "/ground_truth/odom", self._cb, 50)
        self.get_logger().info(f"clearance_logger writing to {csv_path}")

    def _cb(self, msg: Odometry) -> None:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        d = clamped_distance(x, y)
        t_msg = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._writer.writerow([f"{time.time():.6f}", f"{t_msg:.6f}", f"{x:.4f}", f"{y:.4f}", f"{z:.4f}", f"{d:.4f}"])
        self._count += 1
        if self._count % 100 == 0:
            self._f.flush()
            self.get_logger().info(f"logged {self._count} messages, latest dist_to_pillar={d:.3f}")


def main() -> None:
    csv_path = sys.argv[1] if len(sys.argv) > 1 else "/tmp/clearance_log.csv"
    rclpy.init()
    node = ClearanceLogger(csv_path)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._f.flush()
        node._f.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
