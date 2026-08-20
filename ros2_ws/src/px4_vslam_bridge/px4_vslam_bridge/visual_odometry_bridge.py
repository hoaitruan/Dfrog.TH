#!/usr/bin/env python3
"""
Phase 4: cuVSLAM -> PX4 EKF2 external vision bridge.

Republishes cuVSLAM's /visual_slam/tracking/odometry (nav_msgs/Odometry,
frame_id="odom" ENU with an arbitrary yaw origin, child_frame_id="base_link"
FLU body) as px4_msgs/VehicleOdometry on /fmu/in/vehicle_visual_odometry,
which PX4's EKF2 external-vision fusion consumes directly (no MAVLink
round-trip needed since we're already native ROS 2 / uXRCE-DDS).

pose_frame is POSE_FRAME_FRD, not POSE_FRAME_NED: cuVSLAM's "odom" frame
starts wherever tracking first initializes with an arbitrary yaw, not
aligned to True North, which is exactly what FRD (vs NED) means in this
message's own field docs.

cuVSLAM does not populate pose/twist covariance (confirmed all-zero on the
live topic) -- trusting that literally would tell EKF2 the vision estimate
is perfect, causing overconfident fusion. Using fixed conservative
defaults instead, sized to Phase 3's actually-measured ~0.2-0.25m
steady-state tracking error against ground truth.

Velocity is left NaN (invalid/unavailable) rather than converting
cuVSLAM's also-all-zero twist field -- EKF2 falls back to deriving
velocity from its own state propagation when only position+attitude
aiding is provided, which avoids feeding a velocity estimate we have no
real confidence in.

Outlier rejection (added after a Phase 6 flight test): the stationary
hover that validated Phase 3/4 didn't stress cuVSLAM tracking the way
lateral cruising toward a nearby obstacle does. During that flight test,
cuVSLAM's raw tracking output drifted to a wildly wrong position (~55m
from the real, Gazebo-ground-truth-confirmed location, while EKF2_EV_CTRL
was still actively fusing it) -- corrupting PX4's fused position estimate
badly enough to cause a genuinely dangerous runaway flight, stopped only
by killing the controlling node. Since cuVSLAM's own reported vo_state
stays "Success" straight through drift events like this (confirmed in
Phase 3's debugging), that field can't be used as a trustworthy gate --
so instead this bridge tracks the last known-good position and computes
the implied speed of each new update; anything implying a physically
implausible speed for this vehicle gets dropped (not forwarded to EKF2
at all, not held-and-repeated either, so EKF2 falls back on GPS+other
aiding sources for that cycle rather than fusing either a bad jump or a
stale repeat).
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry

from px4_vslam_bridge.frame_transforms import enu_to_ned_position, ros_to_px4_orientation

# cuVSLAM reports zero covariance (not "small", literally unpopulated) --
# see module docstring. sqrt(0.05) ~= 0.22m, matching Phase 3's observed
# steady-state error magnitude.
POSITION_VARIANCE = 0.05
# ~3deg stddev.
ORIENTATION_VARIANCE = 0.0025

# Generous headroom above anything this vehicle actually commands in this
# project (climb ~1-2 m/s, reactive-nav step cap ~1.2 m/s) -- wide enough
# to never reject real motion, tight enough to catch a multi-meter
# tracking-glitch jump within one ~45ms cuVSLAM frame interval.
MAX_PLAUSIBLE_SPEED_MPS = 8.0
MIN_DT_S = 1e-3  # guards against a division blowup on a near-zero dt

NAN = float("nan")


class VisualOdometryBridge(Node):
    def __init__(self) -> None:
        super().__init__("visual_odometry_bridge")

        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.pub = self.create_publisher(
            VehicleOdometry, "/fmu/in/vehicle_visual_odometry", pub_qos
        )
        self.create_subscription(Odometry, "/visual_slam/tracking/odometry", self._cb, 10)

        self._count = 0
        self._rejected_count = 0
        self._last_good_enu = None
        self._last_good_stamp_s = None
        self.get_logger().info(
            "visual_odometry_bridge ready: /visual_slam/tracking/odometry -> "
            "/fmu/in/vehicle_visual_odometry"
        )

    def _cb(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation

        cur_enu = (p.x, p.y, p.z)
        stamp_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if self._last_good_enu is not None:
            dt = max(stamp_s - self._last_good_stamp_s, MIN_DT_S)
            dist = math.sqrt(sum((a - b) ** 2 for a, b in zip(cur_enu, self._last_good_enu)))
            implied_speed = dist / dt
            if implied_speed > MAX_PLAUSIBLE_SPEED_MPS:
                self._rejected_count += 1
                self.get_logger().warn(
                    f"Rejected cuVSLAM update: implied speed {implied_speed:.1f} m/s "
                    f"(jumped {dist:.2f}m in {dt*1000:.0f}ms) -- holding last known-good "
                    f"position, not forwarding to EKF2. ({self._rejected_count} rejected total)",
                    throttle_duration_sec=1.0,
                )
                return

        self._last_good_enu = cur_enu
        self._last_good_stamp_s = stamp_s

        x, y, z = enu_to_ned_position(p.x, p.y, p.z)
        qw, qx, qy, qz = ros_to_px4_orientation((o.w, o.x, o.y, o.z))

        out = VehicleOdometry()
        out.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        out.timestamp_sample = out.timestamp

        out.pose_frame = VehicleOdometry.POSE_FRAME_FRD
        out.position = [float(x), float(y), float(z)]
        out.q = [float(qw), float(qx), float(qy), float(qz)]

        out.velocity_frame = VehicleOdometry.VELOCITY_FRAME_UNKNOWN
        out.velocity = [NAN, NAN, NAN]
        out.angular_velocity = [NAN, NAN, NAN]

        out.position_variance = [POSITION_VARIANCE] * 3
        out.orientation_variance = [ORIENTATION_VARIANCE] * 3
        out.velocity_variance = [NAN] * 3

        out.reset_counter = 0
        out.quality = 0

        self.pub.publish(out)

        self._count += 1
        if self._count % 100 == 0:
            self.get_logger().info(
                f"published {self._count} vehicle_visual_odometry samples "
                f"(last NED pos=({x:.2f},{y:.2f},{z:.2f}))",
                throttle_duration_sec=5.0,
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VisualOdometryBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
