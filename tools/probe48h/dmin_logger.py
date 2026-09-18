#!/usr/bin/env python3
"""
H3: continuous d_min(t) logger, Gazebo GROUND TRUTH (not the map).

probe-48h gate fix (docs/probe48h/GATE12.md A2): now sources position from
gz-sim's SceneBroadcaster (/world/$GZ_WORLD_NAME/dynamic_pose/info,
tf2_msgs/TFMessage), filtered by child_frame_id==GZ_MODEL_FRAME -- true
physics-engine pose, no PX4/EKF2 involved. Previously subscribed
/ground_truth/odom, which run.sh's own comment says explicitly is
EKF2-derived (GPS+baro+IMU fused), not literal simulator truth despite the
name. Same source/filter position_logger.py already uses, confirmed live
in this project's history (see GATE12.md A2) -- not a new, untested path.

Obstacle geometry copied verbatim from tools/route2_probe/analyze_r2_2.py
(extracted from vio_test.sdf, already verified there) -- same source of
truth, not re-derived. Logs a continuous distribution, not a binary
crash/no-crash flag, per this probe's own instruction.

Usage: dmin_logger.py --out <csv> [--gz-world vio_test] [--gz-model x500_depth_stereo_0]
"""
import argparse
import csv
import math
import sys

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage

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
    def __init__(self, out_path: str, gz_world: str, gz_model: str) -> None:
        super().__init__("dmin_logger")
        self.gz_model = gz_model
        self.csv_file = open(out_path, "w", newline="")
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(["t", "x", "y", "z", "d_min"])
        self.create_subscription(
            TFMessage, f"/world/{gz_world}/dynamic_pose/info", self._cb, 50
        )

    def _cb(self, msg: TFMessage) -> None:
        for tr in msg.transforms:
            if tr.child_frame_id != self.gz_model:
                continue
            t = self.get_clock().now().nanoseconds / 1e9
            p = tr.transform.translation
            d = min_clearance(p.x, p.y, p.z)
            self.writer.writerow([t, p.x, p.y, p.z, f"{d:.4f}"])
            self.csv_file.flush()
            return


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--gz-world", default="vio_test")
    ap.add_argument("--gz-model", default="x500_depth_stereo_0")
    args = ap.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    rclpy.init()
    node = DminLogger(args.out, args.gz_world, args.gz_model)
    try:
        rclpy.spin(node)
    finally:
        node.csv_file.close()


if __name__ == "__main__":
    main()
