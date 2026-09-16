#!/usr/bin/env python3
"""
H4: Control-Point Drift logger.

Subscribes /planning/bspline (quadrotor_msgs/Bspline, verified real and
subscribable -- kino_replan_fsm.cpp:90/755/943, traj_server.cpp:34). Every
message here is counted as one "replan event" -- this includes BOTH real
kinodynamic replans (kino_replan_fsm.cpp:943) and watchdog-hold republishes
(kino_replan_fsm.cpp:755); the message alone doesn't disambiguate the two,
so replan_count in the CSV is an upper bound on real replans, noted here
rather than hidden.

Between consecutive messages (k-1, k), ΔP is computed over the FUTURE
(not-yet-flown) segment: for trajectory k-1, "future" = control points at
or after the knot index corresponding to elapsed time (now - start_time).
Trajectory k always starts fresh at "now", so its own not-yet-flown segment
is its full control-point list. Both slices are then truncated to the
shorter length and compared pairwise by position in the slice (an
approximation -- exact per-control-point correspondence across replans
with different knot vectors isn't well-defined; this is the closest
reading of "Q_i^(k) vs Q_i^(k-1) over the future segment" that's
computable from the message alone).

Usage: plan_drift_logger.py --out <csv>
"""
import argparse
import bisect
import csv
import math
import sys

import rclpy
from rclpy.node import Node
from quadrotor_msgs.msg import Bspline


class PlanDriftLogger(Node):
    def __init__(self, out_path: str) -> None:
        super().__init__("plan_drift_logger")
        self.prev = None  # (start_time_s, knots, pos_pts)
        self.replan_count = 0
        self.csv_file = open(out_path, "w", newline="")
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(["t", "replan_count", "traj_id", "n_matched", "delta_p"])
        self.csv_file.flush()
        self.create_subscription(Bspline, "/planning/bspline", self._cb, 10)

    def _cb(self, msg: Bspline) -> None:
        now_s = self.get_clock().now().nanoseconds / 1e9
        self.replan_count += 1
        pos_pts = [(p.x, p.y, p.z) for p in msg.pos_pts]

        delta_p = ""
        n_matched = 0
        if self.prev is not None:
            prev_start, prev_knots, prev_pts = self.prev
            elapsed = now_s - prev_start
            idx = bisect.bisect_left(prev_knots, elapsed) if prev_knots else 0
            idx = min(idx, len(prev_pts))
            future_prev = prev_pts[idx:]
            future_new = pos_pts  # k always starts fresh at "now"
            n = min(len(future_prev), len(future_new))
            if n > 0:
                dists = [
                    math.dist(future_prev[i], future_new[i]) for i in range(n)
                ]
                delta_p = f"{sum(dists) / n:.6f}"
                n_matched = n

        self.writer.writerow([now_s, self.replan_count, msg.traj_id, n_matched, delta_p])
        self.csv_file.flush()

        start_s = msg.start_time.sec + msg.start_time.nanosec / 1e9
        self.prev = (start_s, list(msg.knots), pos_pts)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    args = ap.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    rclpy.init()
    node = PlanDriftLogger(args.out)
    try:
        rclpy.spin(node)
    finally:
        node.csv_file.close()


if __name__ == "__main__":
    main()
