#!/usr/bin/env python3
"""
Phase 3 gate check: compare cuVSLAM's odometry against PX4's ground truth.

cuVSLAM has no global reference -- it starts at (0,0,0) wherever tracking
first initializes, not PX4's home position. So this aligns by subtracting
off each trajectory's own first sample (assumes near-level start, which is
true for arming on the ground) rather than doing a full SE(3) registration
-- adequate for a coarse "is it tracking sanely, not diverging" check.
"""

import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry

# cuVSLAM's default output topic (isaac_ros_visual_slam).
VSLAM_ODOM_TOPIC = "/visual_slam/tracking/odometry"


class VslamCompare(Node):
    def __init__(self) -> None:
        super().__init__("vslam_compare")

        self._gt_origin = None
        self._vslam_origin = None
        self._gt_pos = None
        self._vslam_pos = None
        self._max_error = 0.0
        self._samples = 0

        self.create_subscription(Odometry, "/ground_truth/odom", self._gt_cb, 10)
        self.create_subscription(Odometry, VSLAM_ODOM_TOPIC, self._vslam_cb, 10)
        self.create_timer(1.0, self._report)

        self.get_logger().info(f"vslam_compare ready -- watching {VSLAM_ODOM_TOPIC} vs /ground_truth/odom")

    def _gt_cb(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        pos = (p.x, p.y, p.z)
        if self._gt_origin is None:
            self._gt_origin = pos
            self.get_logger().info(f"Ground truth origin set: {pos}")
        self._gt_pos = tuple(a - b for a, b in zip(pos, self._gt_origin))

    def _vslam_cb(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        pos = (p.x, p.y, p.z)
        if self._vslam_origin is None:
            self._vslam_origin = pos
            self.get_logger().info(f"cuVSLAM origin set: {pos} -- tracking initialized")
        self._vslam_pos = tuple(a - b for a, b in zip(pos, self._vslam_origin))

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
