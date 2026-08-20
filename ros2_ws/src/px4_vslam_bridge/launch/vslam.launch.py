#!/usr/bin/env python3
"""
Phase 3: cuVSLAM (isaac_ros_visual_slam) against the StereoIMU sim rig.

Adapted from isaac_ros_visual_slam's own core launch fragment
(isaac_ros_visual_slam_core.launch.py), with our actual bridged topic
names substituted and IMU fusion enabled. Our sim cameras have zero lens
distortion (D=[0,0,0,0,0]), so raw == rectified here -- the
ImageFormatConverter stage only needs to do encoding conversion (raw L8 ->
mono8), not undistortion.

Noise params computed from StereoIMU's configured Gazebo sensor noise
(see PX4-Autopilot/Tools/simulation/gz/models/StereoIMU/model.sdf):
noise_density (rad/s or m/s^2 per sqrt(Hz)) = per-sample gaussian stddev /
sqrt(update_rate). Random walk terms aren't modeled by our sensor (no bias
drift configured), so those use small conservative MEMS-typical defaults
-- not critical for this coarse tracking-quality check.
"""

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480

# 0.0008726646 rad/s stddev @ 200Hz
GYRO_NOISE_DENSITY = 0.0008726646 / (200.0 ** 0.5)
GYRO_RANDOM_WALK = 0.0000019393
# 0.00637 m/s^2 stddev @ 200Hz
ACCEL_NOISE_DENSITY = 0.00637 / (200.0 ** 0.5)
ACCEL_RANDOM_WALK = 0.0003


def generate_launch_description() -> launch.LaunchDescription:
    image_format_left = ComposableNode(
        package="isaac_ros_image_proc",
        plugin="nvidia::isaac_ros::image_proc::ImageFormatConverterNode",
        name="image_format_node_left",
        parameters=[{
            "encoding_desired": "mono8",
            "image_width": IMAGE_WIDTH,
            "image_height": IMAGE_HEIGHT,
        }],
        remappings=[
            ("image_raw", "/stereo/left/image"),
            ("image", "/stereo/left/image_mono8"),
        ],
    )
    image_format_right = ComposableNode(
        package="isaac_ros_image_proc",
        plugin="nvidia::isaac_ros::image_proc::ImageFormatConverterNode",
        name="image_format_node_right",
        parameters=[{
            "encoding_desired": "mono8",
            "image_width": IMAGE_WIDTH,
            "image_height": IMAGE_HEIGHT,
        }],
        remappings=[
            ("image_raw", "/stereo/right/image"),
            ("image", "/stereo/right/image_mono8"),
        ],
    )

    visual_slam_node = ComposableNode(
        name="visual_slam_node",
        package="isaac_ros_visual_slam",
        plugin="nvidia::isaac_ros::visual_slam::VisualSlamNode",
        parameters=[{
            "use_sim_time": True,
            "enable_image_denoising": False,
            "rectified_images": True,
            "enable_slam_visualization": True,
            "enable_landmarks_view": True,
            "enable_observations_view": True,
            # DISTINCT frames per NVIDIA's own multi-camera example
            # (isaac_ros_visual_slam_realsense.launch.py uses
            # camera_infra1_optical_frame/camera_infra2_optical_frame) --
            # cuVSLAM derives the stereo baseline from TF between these two
            # frames, not from camera_info's P[3] Tx term. A shared frame
            # here told cuVSLAM the cameras were co-located (zero
            # baseline), which is why tracking was frozen with zero
            # observations despite healthy images and correct camera_info.
            "camera_optical_frames": [
                "stereo_imu_link_left_optical",
                "stereo_imu_link_right_optical",
            ],
            "base_frame": "base_link",
            "num_cameras": 2,
            # Defaults to 1 if unset -- force classic 2-camera stereo
            # matching (not independent-multicamera mode) for our
            # overlapping-FOV stereo rig.
            "multicam_mode": 0,
            "enable_imu_fusion": True,
            "imu_frame": "stereo_imu_link",
            "gyro_noise_density": GYRO_NOISE_DENSITY,
            "gyro_random_walk": GYRO_RANDOM_WALK,
            "accel_noise_density": ACCEL_NOISE_DENSITY,
            "accel_random_walk": ACCEL_RANDOM_WALK,
            "calibration_frequency": 200.0,
            # Configured for 30Hz (33.3ms nominal), but this dev machine
            # under load doesn't hold real-time-factor=1.0, so actual
            # frame interval runs ~36-40ms. The default 34.0ms threshold
            # was tight enough that EVERY frame logged "Delta... above
            # threshold" -- worth ruling out as an actual frame-rejection
            # cause (not just log noise) before looking elsewhere.
            "image_jitter_threshold_ms": 100.0,
        }],
        remappings=[
            ("visual_slam/image_0", "/stereo/left/image_mono8"),
            ("visual_slam/camera_info_0", "/stereo/left/camera_info"),
            ("visual_slam/image_1", "/stereo/right/image_mono8"),
            ("visual_slam/camera_info_1", "/stereo/right/camera_info"),
            ("visual_slam/imu", "/stereo/imu"),
        ],
    )

    container = ComposableNodeContainer(
        name="visual_slam_launch_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[image_format_left, image_format_right, visual_slam_node],
        output="screen",
    )

    return launch.LaunchDescription([container])
