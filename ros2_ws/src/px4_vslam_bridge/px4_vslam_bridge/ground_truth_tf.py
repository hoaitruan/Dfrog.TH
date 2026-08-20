#!/usr/bin/env python3
"""
Phase 3: ground-truth TF from PX4's own (sim) state estimate.

This is NOT what cuVSLAM uses as input -- it's the reference we compare
cuVSLAM's own odometry output against, to verify VIO tracking quality
before wiring it into EKF2 (Phase 4). Publishes:

  - TF: map -> base_link_gt        (dynamic, from vehicle_odometry)
  - TF: base_link -> stereo_imu_link          (static, model mount offset)
  - TF: stereo_imu_link -> stereo_imu_link_{left,right}_optical
        (static, +/-25mm baseline + REP103 optical rotation)
  - TF: base_link_gt -> camera_link_gt -> camera_link_optical_gt
        (static, depth camera mount offset + REP103 optical rotation)

Left/right get DISTINCT optical frames (not one shared frame) because
cuVSLAM (isaac_ros_visual_slam) reads the physical stereo baseline from TF
between camera_optical_frames, not from camera_info's P[3] Tx term -- see
StereoIMU/model.sdf's header comment for the full story.

The dynamic ground-truth pose publishes to "base_link_gt", NOT "base_link"
-- cuVSLAM's own VisualSlamNode (base_frame="base_link") publishes its own
"odom -> base_link" TF, and having this node ALSO claim a parent for
"base_link" (as "map -> base_link") gave that frame two competing
parents, corrupting the shared TF buffer (visible as repeated
"TF_OLD_DATA ignoring data from the past for frame base_link" warnings in
the composable node container's log) and interfering with cuVSLAM's own
tracking-frame lookups.

The depth camera's static chain hangs off "base_link_gt", NOT "base_link"
(unlike the stereo/cuVSLAM chain above) -- as of Phase 6, cuVSLAM's "odom"
frame is no longer trusted for flight control (see the airframe file's
EKF2_EV_CTRL=0 comment: it drifted badly under GPU contention with nvblox
running concurrently), so nvblox now sources its camera pose from PX4's
own reliable ground-truth chain instead, keeping its ESDF spatially
consistent with reality regardless of whatever cuVSLAM is doing on the
side. See OakD-Lite/model.sdf's depth sensor gz_frame_id comment.

  - nav_msgs/Odometry on /ground_truth/odom (ENU), for numeric comparison
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

from px4_vslam_bridge.frame_transforms import ned_to_enu_position, px4_to_ros_orientation

# x500_depth_stereo model.sdf: <joint name="StereoIMUJoint">
#   <pose relative_to="base_link"> .12 .03 .242 0 0 0
STEREO_IMU_OFFSET_XYZ = (0.12, 0.03, 0.242)

# StereoIMU/model.sdf: left camera <pose>0 0.025 0 0 0 0</pose>,
# right camera <pose>0 -0.025 0 0 0 0</pose> relative to stereo_imu_link.
LEFT_CAM_OFFSET_XYZ = (0.0, 0.025, 0.0)
RIGHT_CAM_OFFSET_XYZ = (0.0, -0.025, 0.0)

# x500_depth/model.sdf: <joint name="CameraJoint">
#   <pose relative_to="base_link"> .12 .03 .242 0 0 0
# Same offset as StereoIMU -- co-located mount point, see
# x500_depth_stereo/model.sdf's header comment (one physical D435i doing
# both jobs on real hardware; two co-located sim sensor rigs here).
CAMERA_LINK_OFFSET_XYZ = (0.12, 0.03, 0.242)

# REP 103 optical frame: Z forward, X right, Y down, relative to an FLU
# link (X forward, Y left, Z up). Same constant as wall_avoid's
# odom_tf_publisher.py -- fixed quaternion (w, x, y, z).
OPTICAL_Q = (0.5, -0.5, 0.5, -0.5)


class GroundTruthTf(Node):
    def __init__(self) -> None:
        super().__init__("ground_truth_tf")

        sub_px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, "/ground_truth/odom", 10)

        self.create_subscription(
            VehicleOdometry, "/fmu/out/vehicle_odometry", self._odom_callback, sub_px4_qos
        )

        self._publish_static_transforms()
        self.get_logger().info(
            "ground_truth_tf ready: map -> base_link_gt (dynamic); "
            "base_link -> stereo_imu_link -> stereo_imu_link_{left,right}_optical (static); "
            "base_link_gt -> camera_link_gt -> camera_link_optical_gt (static)"
        )

    def _publish_static_transforms(self) -> None:
        now = self.get_clock().now().to_msg()

        base_to_stereo = TransformStamped()
        base_to_stereo.header.stamp = now
        base_to_stereo.header.frame_id = "base_link"
        base_to_stereo.child_frame_id = "stereo_imu_link"
        base_to_stereo.transform.translation.x = STEREO_IMU_OFFSET_XYZ[0]
        base_to_stereo.transform.translation.y = STEREO_IMU_OFFSET_XYZ[1]
        base_to_stereo.transform.translation.z = STEREO_IMU_OFFSET_XYZ[2]
        base_to_stereo.transform.rotation.w = 1.0

        stereo_to_left_optical = TransformStamped()
        stereo_to_left_optical.header.stamp = now
        stereo_to_left_optical.header.frame_id = "stereo_imu_link"
        stereo_to_left_optical.child_frame_id = "stereo_imu_link_left_optical"
        stereo_to_left_optical.transform.translation.x = LEFT_CAM_OFFSET_XYZ[0]
        stereo_to_left_optical.transform.translation.y = LEFT_CAM_OFFSET_XYZ[1]
        stereo_to_left_optical.transform.translation.z = LEFT_CAM_OFFSET_XYZ[2]
        stereo_to_left_optical.transform.rotation.w = OPTICAL_Q[0]
        stereo_to_left_optical.transform.rotation.x = OPTICAL_Q[1]
        stereo_to_left_optical.transform.rotation.y = OPTICAL_Q[2]
        stereo_to_left_optical.transform.rotation.z = OPTICAL_Q[3]

        stereo_to_right_optical = TransformStamped()
        stereo_to_right_optical.header.stamp = now
        stereo_to_right_optical.header.frame_id = "stereo_imu_link"
        stereo_to_right_optical.child_frame_id = "stereo_imu_link_right_optical"
        stereo_to_right_optical.transform.translation.x = RIGHT_CAM_OFFSET_XYZ[0]
        stereo_to_right_optical.transform.translation.y = RIGHT_CAM_OFFSET_XYZ[1]
        stereo_to_right_optical.transform.translation.z = RIGHT_CAM_OFFSET_XYZ[2]
        stereo_to_right_optical.transform.rotation.w = OPTICAL_Q[0]
        stereo_to_right_optical.transform.rotation.x = OPTICAL_Q[1]
        stereo_to_right_optical.transform.rotation.y = OPTICAL_Q[2]
        stereo_to_right_optical.transform.rotation.z = OPTICAL_Q[3]

        base_to_camera = TransformStamped()
        base_to_camera.header.stamp = now
        base_to_camera.header.frame_id = "base_link_gt"
        base_to_camera.child_frame_id = "camera_link_gt"
        base_to_camera.transform.translation.x = CAMERA_LINK_OFFSET_XYZ[0]
        base_to_camera.transform.translation.y = CAMERA_LINK_OFFSET_XYZ[1]
        base_to_camera.transform.translation.z = CAMERA_LINK_OFFSET_XYZ[2]
        base_to_camera.transform.rotation.w = 1.0

        camera_to_optical = TransformStamped()
        camera_to_optical.header.stamp = now
        camera_to_optical.header.frame_id = "camera_link_gt"
        camera_to_optical.child_frame_id = "camera_link_optical_gt"
        camera_to_optical.transform.rotation.w = OPTICAL_Q[0]
        camera_to_optical.transform.rotation.x = OPTICAL_Q[1]
        camera_to_optical.transform.rotation.y = OPTICAL_Q[2]
        camera_to_optical.transform.rotation.z = OPTICAL_Q[3]

        self.static_tf_broadcaster.sendTransform(
            [
                base_to_stereo,
                stereo_to_left_optical,
                stereo_to_right_optical,
                base_to_camera,
                camera_to_optical,
            ]
        )

    def _odom_callback(self, msg: VehicleOdometry) -> None:
        x, y, z = ned_to_enu_position(msg.position[0], msg.position[1], msg.position[2])
        qw, qx, qy, qz = px4_to_ros_orientation((msg.q[0], msg.q[1], msg.q[2], msg.q[3]))

        stamp = self.get_clock().now().to_msg()

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = "map"
        t.child_frame_id = "base_link_gt"
        t.transform.translation.x = float(x)
        t.transform.translation.y = float(y)
        t.transform.translation.z = float(z)
        t.transform.rotation.w = float(qw)
        t.transform.rotation.x = float(qx)
        t.transform.rotation.y = float(qy)
        t.transform.rotation.z = float(qz)
        self.tf_broadcaster.sendTransform(t)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "map"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = float(x)
        odom.pose.pose.position.y = float(y)
        odom.pose.pose.position.z = float(z)
        odom.pose.pose.orientation.w = float(qw)
        odom.pose.pose.orientation.x = float(qx)
        odom.pose.pose.orientation.y = float(qy)
        odom.pose.pose.orientation.z = float(qz)
        self.odom_pub.publish(odom)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GroundTruthTf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
