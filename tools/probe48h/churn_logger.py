#!/usr/bin/env python3
"""
H4: Map-Churn logger.

Phase-0 finding: /nvblox_node/static_esdf_pointcloud never publishes in
this project's esdf_mode=3d config (nvblox_node.cpp:787's only publish
call site is inside a k2D-only branch -- confirmed dead code, same
diagnosis esdf_recorder.py already made). So this polls
/nvblox_node/get_esdf_and_gradient on a timer, same pattern as
esdf_recorder.py, instead of subscribing a topic.

Churn(t) = mean_{v in Vr} |d_t(v) - d_{t-1}(v)|, Vr = voxels within
radius r of the drone's CURRENT (time t) ground-truth position that were
also present in the t-1 grid. Voxels are matched by rounded (x,y,z)
center across polls (voxel_size is constant within one run).

Usage: churn_logger.py --out <csv> [--radius 2.0] [--period 1.0]
"""
import argparse
import csv
import math
import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Vector3
from nav_msgs.msg import Odometry
from nvblox_msgs.srv import EsdfAndGradients

UNOBSERVED_SENTINEL = -999.0
AABB_MARGIN = 1.0  # extra margin beyond radius so the sphere isn't AABB-clipped


class ChurnLogger(Node):
    def __init__(self, out_path: str, radius: float, period: float) -> None:
        super().__init__("churn_logger")
        self.radius = radius
        self.cli = self.create_client(EsdfAndGradients, "/nvblox_node/get_esdf_and_gradient")
        self.create_subscription(Odometry, "/ground_truth/odom", self._gt_cb, 10)
        self.pos = None
        self.prev_grid = None  # {(x,y,z): distance}
        self.in_flight = False
        self.csv_file = open(out_path, "w", newline="")
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(["t", "n_voxels_compared", "churn", "n_total_voxels"])
        self.csv_file.flush()
        self.timer = self.create_timer(period, self.tick)

    def _gt_cb(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self.pos = (p.x, p.y, p.z)

    def tick(self) -> None:
        if self.in_flight or self.pos is None or not self.cli.service_is_ready():
            return
        px, py, pz = self.pos
        half = self.radius + AABB_MARGIN
        req = EsdfAndGradients.Request()
        req.update_esdf = True
        req.visualize_esdf = False
        req.use_aabb = True
        req.frame_id = "map"
        req.aabb_min_m = Point(x=px - half, y=py - half, z=pz - half)
        req.aabb_size_m = Vector3(x=2 * half, y=2 * half, z=2 * half)
        self.in_flight = True
        self._req_pos = self.pos
        self._req_t = self.get_clock().now()
        future = self.cli.call_async(req)
        future.add_done_callback(self._on_response)

    def _on_response(self, future) -> None:
        self.in_flight = False
        resp = future.result()
        if resp is None or not resp.success:
            return
        vs = resp.voxel_size_m
        dims = resp.esdf_and_gradients.layout.dim
        if vs <= 0.0 or len(dims) != 3:
            return
        nx, ny, nz = dims[0].size, dims[1].size, dims[2].size
        data = resp.esdf_and_gradients.data
        if nx <= 0 or ny <= 0 or nz <= 0 or len(data) != nx * ny * nz:
            return
        ox, oy, oz = resp.origin_m.x, resp.origin_m.y, resp.origin_m.z

        grid = {}
        for ix in range(nx):
            base_x = ix * ny * nz
            x = ox + vs * (ix + 0.5)
            for iy in range(ny):
                base = base_x + iy * nz
                y = oy + vs * (iy + 0.5)
                for iz in range(nz):
                    d = data[base + iz]
                    if d <= UNOBSERVED_SENTINEL:
                        continue
                    z = oz + vs * (iz + 0.5)
                    grid[(round(x, 3), round(y, 3), round(z, 3))] = d

        churn = None
        n_compared = 0
        if self.prev_grid is not None:
            cx, cy, cz = self._req_pos
            diffs = []
            for key, d_new in grid.items():
                d_old = self.prev_grid.get(key)
                if d_old is None:
                    continue
                x, y, z = key
                if math.sqrt((x - cx) ** 2 + (y - cy) ** 2 + (z - cz) ** 2) <= self.radius:
                    diffs.append(abs(d_new - d_old))
            n_compared = len(diffs)
            if diffs:
                churn = sum(diffs) / len(diffs)

        self.writer.writerow([
            self._req_t.nanoseconds / 1e9,
            n_compared,
            f"{churn:.6f}" if churn is not None else "",
            len(grid),
        ])
        self.csv_file.flush()
        self.prev_grid = grid


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--radius", type=float, default=2.0)
    ap.add_argument("--period", type=float, default=1.0)
    args = ap.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    rclpy.init()
    node = ChurnLogger(args.out, args.radius, args.period)
    try:
        rclpy.spin(node)
    finally:
        node.csv_file.close()


if __name__ == "__main__":
    main()
