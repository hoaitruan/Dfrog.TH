#!/usr/bin/env python3
"""
Phase 5: nvblox ESDF mapping from the OakD-Lite depth camera.

nvblox's own example launch files (nvblox_examples_bringup) only expose a
fixed set of vendor camera profiles (isaac_sim/realsense/zed) via their
`camera` argument, none of which match our custom Gazebo depth rig's
topic names -- so this instantiates nvblox_ros's ComposableNode directly,
reusing its stock nvblox_base.yaml for all the TSDF/ESDF integration
tuning and only overriding what our rig needs.

use_tf_transforms defaults to true in nvblox_base.yaml, so nvblox places
depth points using TF rather than a separate pose topic -- it looks up
the depth image's own frame_id (camera_link_optical_gt, see
OakD-Lite/model.sdf and ground_truth_tf.py) against global_frame ("map").

global_frame is PX4's own ground-truth frame, NOT cuVSLAM's "odom" frame
-- a Phase 6 flight test found cuVSLAM tracking drifting badly under GPU
contention with nvblox's own concurrent CUDA context, corrupting anything
downstream that trusted its "odom" frame (see the airframe file's
EKF2_EV_CTRL=0 comment). nvblox's ESDF needs to stay spatially consistent
with where the drone actually is regardless of what cuVSLAM is doing on
the side, so it now sources camera pose from ground_truth_tf.py's
base_link_gt-rooted chain (fed by PX4's own /fmu/out/vehicle_odometry,
GPS-based since EKF2_EV_CTRL=0) instead.

use_color/use_lidar are turned off: our rig has no bridged color feed for
the depth camera (OakD-Lite's IMX214 sensor is unused here) and no lidar
-- ESDF/collision geometry only needs depth.

esdf_mode is forced to "3d", overriding nvblox_base.yaml's "2d" default.
The 2D mode's static_map_slice only reports a fixed ~0-1m *absolute*
height band above map_clearing_frame's ground plane (sane for a
ground/wheeled robot whose camera is near the floor) -- our drone hovers
and looks at obstacles around its own flight altitude (~5m in testing),
so that band never gets any real observations even though TSDF blocks
are genuinely being allocated from real depth data (confirmed via
mapper.cpp's GPU hash growth log messages during a flight test). A
drone needs the full 3D ESDF, not a ground-level slice.

feasibility-gate addition: use_sim_time=True, matching visual_slam_node
(vslam.launch.py) and ground_truth_tf (run.sh's use_sim_time:=true).
Previously unset here -- nvblox_node's own timer/TF lookups fell back to
wall-clock, which the project's own sim-time discipline (see the Phase 6
"use_sim_time" incident in ground_truth_tf.py's history) already
established as a real, silent-failure-mode risk for any node doing
timestamp-sensitive work against a Gazebo-bridged clock. Fixed here as a
measurement-correctness change only; does not touch EKF2_EV_CTRL,
airframe, or estimator params.

feasibility-gate addition: integrate_depth_rate_hz/update_esdf_rate_hz are
exposed as launch arguments (defaulting to nvblox_base.yaml's own 40.0/
10.0, so omitting them changes nothing) for
tools/feasibility_gate/run_gate.sh's `--workload nvblox` contention knob.
This MUST be a launch-time argument, not a runtime `ros2 param set`:
node_params.cpp's initParam() reads every nvblox param exactly once via
declare_parameter() at construction and assigns it into a plain struct
member (nvblox_node.cpp:990's params_.integrate_depth_rate_hz) -- there is
no add_on_set_parameters_callback anywhere in nvblox_ros (checked), so a
live `ros2 param set` on a running node changes the parameter server's
stored value only, silently, with zero effect on actual behavior.
Changing the rate for real means relaunching this container with the new
value, which is what run_gate.sh does (kill + relaunch, not param set).
"""

import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode, ParameterValue

NVBLOX_BASE_CONFIG = "/opt/ros/humble/share/nvblox_examples_bringup/config/nvblox/nvblox_base.yaml"

# nvblox_base.yaml's own defaults -- kept here only so omitting the launch
# arguments reproduces exactly the previous, un-overridden behavior.
DEFAULT_INTEGRATE_DEPTH_RATE_HZ = "40.0"
DEFAULT_UPDATE_ESDF_RATE_HZ = "10.0"


def generate_launch_description() -> launch.LaunchDescription:
    integrate_depth_rate_hz_arg = DeclareLaunchArgument(
        "integrate_depth_rate_hz", default_value=DEFAULT_INTEGRATE_DEPTH_RATE_HZ,
        description="feasibility-gate contention knob -- see module docstring.",
    )
    update_esdf_rate_hz_arg = DeclareLaunchArgument(
        "update_esdf_rate_hz", default_value=DEFAULT_UPDATE_ESDF_RATE_HZ,
        description="feasibility-gate contention knob -- see module docstring.",
    )

    nvblox_node = ComposableNode(
        name="nvblox_node",
        package="nvblox_ros",
        plugin="nvblox::NvbloxNode",
        parameters=[
            NVBLOX_BASE_CONFIG,
            {
                "use_sim_time": True,
                "num_cameras": 1,
                "use_color": False,
                "use_lidar": False,
                "use_depth": True,
                "use_tf_transforms": True,
                "global_frame": "map",
                "pose_frame": "camera_link_optical_gt",
                "map_clearing_frame_id": "base_link_gt",
                # Disabled (<=0, see nvblox_base.yaml's own comment on
                # tick-rate params): its periodic "base_link" TF lookup
                # failed once at startup (before cuVSLAM's first odom->
                # base_link message existed) and its log line literally
                # says "Layer pointclouds not published" as a direct
                # consequence -- static_map_slice/static_esdf_pointcloud
                # stayed permanently empty afterward even though TSDF
                # integration kept running fine (confirmed via mapper.cpp's
                # GPU hash growth log messages). Not essential for Phase 5.
                # Both re-enabled for RViz visualization (mesh/ESDF
                # slice/pointcloud topics) -- originally disabled because
                # their periodic TF-lookup-driven timer callback
                # deadlocked the node's *single-threaded* executor
                # entirely (see the container-executable comment above).
                # Now that the container runs component_container_mt, and
                # the TF chain is correctly rooted at base_link_gt with
                # sim_time fixed (Phase 6/7), testing whether these work
                # again -- the on-demand get_esdf_and_gradient service
                # remains the interface Phase 6's reactive planner
                # actually depends on, this is purely for visualization.
                "clear_map_outside_radius_rate_hz": 1.0,
                "publish_layer_rate_hz": 2.0,
                "esdf_mode": "3d",
                "integrate_depth_rate_hz": ParameterValue(
                    LaunchConfiguration("integrate_depth_rate_hz"), value_type=float
                ),
                "update_esdf_rate_hz": ParameterValue(
                    LaunchConfiguration("update_esdf_rate_hz"), value_type=float
                ),
            },
        ],
        remappings=[
            ("camera_0/depth/image", "/depth_camera"),
            ("camera_0/depth/camera_info", "/camera_info"),
        ],
    )

    container = ComposableNodeContainer(
        name="nvblox_container",
        namespace="",
        package="rclcpp_components",
        # MUST be multi-threaded -- confirmed via gdb backtrace
        # (`sudo gdb -p <pid> -batch -ex 'thread apply all bt'` on a
        # hung get_esdf_and_gradient call) that this is a real,
        # unconditional deadlock in nvblox_ros's own design, not a config
        # issue: the service handler (getEsdfAndGradientService) runs on
        # the executor thread, queues a task, and calls
        # waitForTaskCompletion() -- blocking that SAME thread. The task
        # can only be drained by tick()'s processServiceRequestTaskQueue(),
        # which is scheduled on the identical thread under a
        # single-threaded executor, so it can never run while the service
        # handler is blocked waiting for it. This is unconditional --
        # happens on every call regardless of AABB size, TF availability,
        # or update_esdf, and reproduces identically after rebuilding
        # nvblox_ros from source (github.com/NVIDIA-ISAAC-ROS/isaac_ros_
        # nvblox release-3.2), so it's not an apt-packaging bug either.
        # An earlier attempt at component_container_mt failed to even
        # load the composable node (no "Load Library" line, node never
        # appeared) -- see nvblox.launch.py's git history for that
        # attempt; unclear why it failed, retrying since gdb now proves
        # mt is the only viable fix short of patching nvblox_ros itself.
        executable="component_container_mt",
        composable_node_descriptions=[nvblox_node],
        output="screen",
    )

    return launch.LaunchDescription([
        integrate_depth_rate_hz_arg,
        update_esdf_rate_hz_arg,
        container,
    ])
