#!/usr/bin/env python3
"""
Phase 3 gate check: compare cuVSLAM's odometry against PX4's ground truth.

cuVSLAM has no global reference -- it starts at (0,0,0) wherever tracking
first initializes, not PX4's home position. So this aligns by subtracting
off each trajectory's own first sample (assumes near-level start, which is
true for arming on the ground) rather than doing a full SE(3) registration
-- adequate for a coarse "is it tracking sanely, not diverging" check.

G2 (feasibility-gate) extension: optional per-message CSV logging, keyed
by an experiment id, for tools/feasibility_gate/analyze_sweep.py. Off by
default (`csv_path` param empty) so this stays a drop-in for anyone still
using it the original Phase-3 way. Yaw is compared as change-from-own-start
rather than absolute yaw, for the same reason position is origin-relative:
cuVSLAM's yaw origin is arbitrary (see visual_odometry_bridge.py's
docstring), so raw yaw values from the two sources aren't comparable, only
their drift since each one's own first sample is.
"""

import csv
import math
import os
import time

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2

from px4_vslam_bridge.frame_transforms import yaw_from_quat

try:
    from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
    HAVE_VSLAM_STATUS_MSG = True
except ImportError:
    HAVE_VSLAM_STATUS_MSG = False

# cuVSLAM's default output topic (isaac_ros_visual_slam).
VSLAM_ODOM_TOPIC = "/visual_slam/tracking/odometry"
VSLAM_STATUS_TOPIC = "/visual_slam/status"
VSLAM_FEATURES_TOPIC = "/visual_slam/vis/observations_cloud"

CSV_FIELDS = [
    "t_wall", "t_sim", "exp_id",
    "gt_x", "gt_y", "gt_z", "gt_dyaw_deg",
    "vslam_x", "vslam_y", "vslam_z", "vslam_dyaw_deg",
    "pos_error_m", "yaw_error_deg",
    "vo_state", "feature_count", "msg_age_s",
]


def wrap_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class VslamCompare(Node):
    def __init__(self) -> None:
        super().__init__("vslam_compare")

        self.declare_parameter("csv_path", "")
        self.declare_parameter("exp_id", "")
        csv_path = self.get_parameter("csv_path").value
        self._exp_id = self.get_parameter("exp_id").value

        self._gt_origin = None      # (x, y, z, yaw)
        self._vslam_origin = None   # (x, y, z, yaw)
        self._gt_pos = None
        self._vslam_pos = None
        self._max_error = 0.0
        self._samples = 0

        self._latest_gt_full = None       # (x, y, z, yaw)
        self._vo_state = None
        self._feature_count = None

        self._csv_writer = None
        self._csv_file = None
        if csv_path:
            os.makedirs(os.path.dirname(csv_path) or ".", exist_ok=True)
            self._csv_file = open(csv_path, "w", newline="")
            self._csv_writer = csv.DictWriter(self._csv_file, fieldnames=CSV_FIELDS)
            self._csv_writer.writeheader()
            self.get_logger().info(f"vslam_compare: logging per-message CSV to {csv_path} (exp_id={self._exp_id!r})")

        self.create_subscription(Odometry, "/ground_truth/odom", self._gt_cb, 10)
        self.create_subscription(Odometry, VSLAM_ODOM_TOPIC, self._vslam_cb, 10)
        self.create_subscription(PointCloud2, VSLAM_FEATURES_TOPIC, self._features_cb, 10)
        if HAVE_VSLAM_STATUS_MSG:
            self.create_subscription(VisualSlamStatus, VSLAM_STATUS_TOPIC, self._status_cb, 10)
        else:
            self.get_logger().warn(
                f"isaac_ros_visual_slam_interfaces not importable -- {VSLAM_STATUS_TOPIC} "
                "(vo_state) will be logged as empty."
            )
        self.create_timer(1.0, self._report)

        self.get_logger().info(f"vslam_compare ready -- watching {VSLAM_ODOM_TOPIC} vs /ground_truth/odom")

    def destroy_node(self) -> bool:
        if self._csv_file is not None:
            self._csv_file.close()
        return super().destroy_node()

    def _gt_cb(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        yaw = yaw_from_quat(o.w, o.x, o.y, o.z)
        pos = (p.x, p.y, p.z)
        if self._gt_origin is None:
            self._gt_origin = (p.x, p.y, p.z, yaw)
            self.get_logger().info(f"Ground truth origin set: {pos}")
        self._gt_pos = tuple(a - b for a, b in zip(pos, self._gt_origin[:3]))
        self._latest_gt_full = (p.x, p.y, p.z, yaw)

    def _vslam_cb(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        yaw = yaw_from_quat(o.w, o.x, o.y, o.z)
        pos = (p.x, p.y, p.z)
        if self._vslam_origin is None:
            self._vslam_origin = (p.x, p.y, p.z, yaw)
            self.get_logger().info(f"cuVSLAM origin set: {pos} -- tracking initialized")
        self._vslam_pos = tuple(a - b for a, b in zip(pos, self._vslam_origin[:3]))

        if self._csv_writer is not None and self._gt_origin is not None and self._latest_gt_full is not None:
            self._log_row(msg, p, yaw)

    def _features_cb(self, msg: PointCloud2) -> None:
        self._feature_count = msg.width

    def _status_cb(self, msg) -> None:
        self._vo_state = int(msg.vo_state)

    def _log_row(self, msg: Odometry, p, vslam_yaw: float) -> None:
        gx, gy, gz, gyaw0 = self._gt_origin
        vx0, vy0, vz0, vyaw0 = self._vslam_origin
        gtx, gty, gtz, gtyaw = self._latest_gt_full

        gt_dyaw = wrap_angle(gtyaw - gyaw0)
        vslam_dyaw = wrap_angle(vslam_yaw - vyaw0)
        yaw_error = abs(wrap_angle(vslam_dyaw - gt_dyaw))

        gt_rel = (gtx - gx, gty - gy, gtz - gz)
        vslam_rel = self._vslam_pos
        pos_error = math.sqrt(sum((a - b) ** 2 for a, b in zip(gt_rel, vslam_rel)))

        stamp_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        msg_age_s = self.get_clock().now().nanoseconds * 1e-9 - stamp_s

        self._csv_writer.writerow({
            "t_wall": f"{time.time():.6f}",
            "t_sim": f"{stamp_s:.6f}",
            "exp_id": self._exp_id,
            "gt_x": gt_rel[0], "gt_y": gt_rel[1], "gt_z": gt_rel[2],
            "gt_dyaw_deg": math.degrees(gt_dyaw),
            "vslam_x": vslam_rel[0], "vslam_y": vslam_rel[1], "vslam_z": vslam_rel[2],
            "vslam_dyaw_deg": math.degrees(vslam_dyaw),
            "pos_error_m": pos_error,
            "yaw_error_deg": math.degrees(yaw_error),
            "vo_state": self._vo_state if self._vo_state is not None else "",
            "feature_count": self._feature_count if self._feature_count is not None else "",
            "msg_age_s": msg_age_s,
        })
        self._csv_file.flush()

    def _report(self) -> None:
        if self._gt_pos is None:
            self.get_logger().warn("No /ground_truth/odom data yet.", throttle_duration_sec=5.0)
            return
        if self._vslam_pos is None:
            self.get_logger().warn(
                f"No {VSLAM_ODOM_TOPIC} data yet -- cuVSLAM not tracking (or not launched).",
                throttle_duration_sec=5.0,
            )
            return

        err = math.sqrt(sum((a - b) ** 2 for a, b in zip(self._gt_pos, self._vslam_pos)))
        self._max_error = max(self._max_error, err)
        self._samples += 1

        self.get_logger().info(
            f"gt={tuple(round(v, 2) for v in self._gt_pos)} "
            f"vslam={tuple(round(v, 2) for v in self._vslam_pos)} "
            f"error={err:.3f}m max_error={self._max_error:.3f}m (n={self._samples})"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VslamCompare()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
