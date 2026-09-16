#!/usr/bin/env python3
"""
H3: continuous d_min(t) logger, Gazebo GROUND TRUTH (not the map).

Obstacle geometry copied verbatim from tools/route2_probe/analyze_r2_2.py
(extracted from vio_test.sdf, already verified there) -- same source of
truth, not re-derived. Logs a continuous distribution, not a binary
crash/no-crash flag, per this probe's own instruction.

Usage: dmin_logger.py --out <csv>
"""
import argparse
import csv
import math
import sys

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

OBSTACLES = [
    ("marker_01", 4, 0, 1, "box", (0.5, 0.5, 2)),
    ("marker_02", -4, 3, 1.5, "cyl", (0.4, 3)),
    ("marker_03", 0, 5, 2, "box", (0.6, 0.6, 4)),
    ("marker_04", -3, -4, 2.5, "box", (0.5, 0.5, 5)),
    ("marker_05", 5, -3, 3, "cyl", (0.4, 6)),
    ("marker_06", 2, -6, 3.5, "box", (0.5, 0.5, 7)),
    ("marker_07", -5, -2, 4, "box", (0.6, 0.6, 8)),
    ("marker_08", -2, 6, 4.5, "cyl", (0.4, 9)),
    ("marker_09", 6, 2, 1, "box", (0.5, 0.5, 2)),
    ("marker_10", -6, 1, 1.5, "cyl", (0.35, 3)),
    ("marker_11", 1, -3, 2, "box", (0.4, 0.4, 4)),
    ("marker_12", -1, 4, 2.5, "box", (0.5, 0.5, 5)),
    ("marker_13", 3, 4, 1, "cyl", (0.4, 2)),
    ("marker_14", -3, -1, 1.5, "box", (0.5, 0.5, 3)),
    ("marker_15", 0, -5, 2, "box", (0.5, 0.5, 4)),
    ("marker_16", 4, -1, 2.5, "cyl", (0.4, 5)),
    ("pillar_01", 2.5, 0, 5, "box", (0.4, 0.4, 10)),
    ("pillar_02", 1.77, 1.77, 5, "box", (0.4, 0.4, 10)),
    ("pillar_03", 0, 2.5, 5, "box", (0.4, 0.4, 10)),
    ("pillar_04", -1.77, 1.77, 5, "box", (0.4, 0.4, 10)),
    ("pillar_05", -2.5, 0, 5, "box", (0.4, 0.4, 10)),
    ("pillar_06", -1.77, -1.77, 5, "box", (0.4, 0.4, 10)),
    ("pillar_07", 0, -2.5, 5, "box", (0.4, 0.4, 10)),
    ("pillar_08", 1.77, -1.77, 5, "box", (0.4, 0.4, 10)),
]


def point_to_box_dist(px, py, pz, cx, cy, cz, sx, sy, sz):
    hx, hy, hz = sx / 2, sy / 2, sz / 2
    dx = max(abs(px - cx) - hx, 0.0)
    dy = max(abs(py - cy) - hy, 0.0)
    dz = max(abs(pz - cz) - hz, 0.0)
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def point_to_cyl_dist(px, py, pz, cx, cy, cz, radius, length):
    dz = max(abs(pz - cz) - length / 2, 0.0)
    dr = max(math.sqrt((px - cx) ** 2 + (py - cy) ** 2) - radius, 0.0)
    return math.sqrt(dr * dr + dz * dz)


def min_clearance(px, py, pz):
    best = None
    for _, cx, cy, cz, kind, dims in OBSTACLES:
        d = (
            point_to_box_dist(px, py, pz, cx, cy, cz, *dims)
            if kind == "box"
            else point_to_cyl_dist(px, py, pz, cx, cy, cz, *dims)
        )
        if best is None or d < best:
            best = d
    return best


class DminLogger(Node):
    def __init__(self, out_path: str) -> None:
        super().__init__("dmin_logger")
        self.csv_file = open(out_path, "w", newline="")
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(["t", "x", "y", "z", "d_min"])
        self.create_subscription(Odometry, "/ground_truth/odom", self._cb, 10)

    def _cb(self, msg: Odometry) -> None:
        t = self.get_clock().now().nanoseconds / 1e9
        p = msg.pose.pose.position
        d = min_clearance(p.x, p.y, p.z)
        self.writer.writerow([t, p.x, p.y, p.z, f"{d:.4f}"])
        self.csv_file.flush()


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    args = ap.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    rclpy.init()
    node = DminLogger(args.out)
    try:
        rclpy.spin(node)
    finally:
        node.csv_file.close()


if __name__ == "__main__":
    main()
