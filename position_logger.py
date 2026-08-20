#!/usr/bin/env python3
"""
Continuous position logger for receding-horizon milestone-1's first
flight test. Started BEFORE arming, left running through touchdown.

Subscribes /ground_truth/odom directly (no polling, every message
processed) and appends (t_wall, t_msg, x, y, z) to a CSV. Correlated
after the flight against fast_planner_node's own DEBUG_FRONTIER log
lines (already emitted every replan cycle, kino_replan_fsm.cpp) by
timestamp, to see how frontier_pt_ advances relative to where the drone
actually is over time.

Lỗi 3 Part C: also subscribes the raw gz-sim pose bridged by run.sh
(/world/vio_test/dynamic_pose/info, tf2_msgs/msg/TFMessage -- gz-sim's
SceneBroadcaster, not the dedicated PosePublisher plugin originally added
to x500_depth_stereo/model.sdf, which advertises but never actually
publishes in practice; see run.sh's bridge-line comment) and logs it
alongside the EKF2-derived /ground_truth/odom columns in the SAME row, so
the next flight can answer whether EKF2 tracks the real simulated
airframe or drifts from it. TFMessage carries one transform per entity in
the world (child_frame_id names it) -- this filters for
"x500_depth_stereo_0" (the model root), ignoring the other entities
(base_link, sensors, etc.) also present in the same message. No header/
timestamp survives that filtering (TransformStamped's own header.stamp is
gz sim-time, but int32 sec/nanosec only, same resolution concern as
everywhere else in this project -- simplest to just stamp receipt time
here), so gz_t_wall is stamped at local receipt time, same as t_wall
already is for the primary odom column -- directly comparable, not a
lesser-quality timestamp. gz_* columns are blank until the first raw-pose
message arrives (the bridge and this logger start independently; there
is no ordering guarantee between them).
"""
import csv
import sys
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage

GZ_MODEL_FRAME = "x500_depth_stereo_0"


class PositionLogger(Node):
    def __init__(self, csv_path: str) -> None:
        super().__init__("position_logger")
        self._f = open(csv_path, "w", newline="")
        self._writer = csv.writer(self._f)
        self._writer.writerow(
            ["t_wall", "t_msg", "x", "y", "z", "gz_t_wall", "gz_x", "gz_y", "gz_z"]
        )
        self._count = 0
        self._latest_gz_pose = None  # (t_wall, x, y, z) or None until first message
        self.create_subscription(Odometry, "/ground_truth/odom", self._cb, 50)
        self.create_subscription(
            TFMessage, "/world/vio_test/dynamic_pose/info", self._gz_pose_cb, 50
        )
        self.get_logger().info(f"position_logger writing to {csv_path}")

    def _gz_pose_cb(self, msg: TFMessage) -> None:
        for t in msg.transforms:
            if t.child_frame_id == GZ_MODEL_FRAME:
                p = t.transform.translation
                self._latest_gz_pose = (time.time(), p.x, p.y, p.z)
                return

    def _cb(self, msg: Odometry) -> None:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        t_msg = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if self._latest_gz_pose is not None:
            gz_t_wall, gz_x, gz_y, gz_z = self._latest_gz_pose
            gz_row = [f"{gz_t_wall:.6f}", f"{gz_x:.4f}", f"{gz_y:.4f}", f"{gz_z:.4f}"]
        else:
            gz_row = ["", "", "", ""]

        self._writer.writerow(
            [f"{time.time():.6f}", f"{t_msg:.6f}", f"{x:.4f}", f"{y:.4f}", f"{z:.4f}"] + gz_row
        )
        self._count += 1
        if self._count % 100 == 0:
            self._f.flush()
            gz_note = "no raw-gz pose yet" if self._latest_gz_pose is None else "raw-gz pose flowing"
            self.get_logger().info(
                f"logged {self._count} messages, latest pos=({x:.2f},{y:.2f},{z:.2f}), {gz_note}"
            )


def main() -> None:
    csv_path = sys.argv[1] if len(sys.argv) > 1 else "/tmp/position_log.csv"
    rclpy.init()
    node = PositionLogger(csv_path)
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
