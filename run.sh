#!/usr/bin/env bash
# Task 5: single-script boot of the full non-arming pipeline, ending in a
# health check that refuses to let a flight-capable node start unless it
# passes. Codifies the boot sequence that was, until now, only ever typed
# by hand across many sessions -- including once forgetting the
# parameter_bridge step entirely (see README's "Operational gotchas").
#
# Must be run as admin inside the container:
#   docker exec -u admin -it isaac_ros_dev_persistent bash /workspaces/isaac_ros-dev/run.sh
# (docker exec defaults to root, which silently receives zero DDS data --
# see README. This script refuses to proceed if not run as admin.)
#
# Brings up: MicroXRCEAgent -> PX4 SITL+Gazebo -> ros_gz parameter_bridge
# -> ground_truth_tf -> cuVSLAM -> nvblox -> fast_planner_node/traj_server,
# then runs health_check.py. Does NOT start fast_planner_bridge or
# fast_planner_trigger -- those are the only two nodes that actually arm
# the vehicle, and stay a deliberate, separate, manual step after this
# script reports READY.

set -eo pipefail
# ROS2's own setup.bash scripts reference unset variables internally --
# set -u (nounset) is deliberately left off for that reason (confirmed by
# record.sh hitting exactly this while testing).

if [[ "$(whoami)" != "admin" ]]; then
  echo "run.sh: must run as admin (got '$(whoami)'). Re-run via:" >&2
  echo "  docker exec -u admin -it isaac_ros_dev_persistent bash $0" >&2
  exit 1
fi

WS=/workspaces/isaac_ros-dev
LOG_DIR="$WS/ros2_ws/run_logs"
mkdir -p "$LOG_DIR"

source /opt/ros/humble/setup.bash
source "$WS/ros2_ws/install/setup.bash"

echo "=== [1/7] MicroXRCEAgent ==="
"$WS/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent" udp4 -p 8888 \
  > "$LOG_DIR/agent.log" 2>&1 &
sleep 2

echo "=== [2/7] PX4 SITL + Gazebo (gz_x500_depth_stereo_vio_test) ==="
(
  cd "$WS/PX4-Autopilot"
  exec make px4_sitl gz_x500_depth_stereo_vio_test \
    < <(tail -f /dev/null)
) > "$LOG_DIR/px4.log" 2>&1 &
echo "    waiting for PX4 to boot..."
sleep 15

echo "=== [3/7] ros_gz camera/depth/clock bridge ==="
# /world/vio_test/dynamic_pose/info (Lỗi 3 Part C): the raw gz-sim physics
# pose of every non-static entity, independent of PX4/EKF2 entirely, for
# comparison against /ground_truth/odom (which IS EKF2-derived, see
# ground_truth_tf.py's header). Published by gz-sim's SceneBroadcaster
# system, which PX4's own server.config already loads unconditionally --
# no extra plugin needed, confirmed live via `gz topic -e` during bring-up
# (2026-08-16). Originally tried a dedicated PosePublisher plugin on
# x500_depth_stereo/model.sdf instead (still present there) -- that
# topic advertises but never actually publishes in practice (root cause
# not yet diagnosed, possibly an <include merge='true'> plugin-scoping
# gap), so this SceneBroadcaster topic is the one actually in use.
# Bridged as tf2_msgs/msg/TFMessage, not geometry_msgs/msg/PoseArray --
# ros_gz_bridge's Pose_V conversion preserves the per-entity name into
# child_frame_id for TFMessage (PoseArray would lose it, leaving a bare,
# ambiguously-ordered array). position_logger.py filters for
# child_frame_id=="x500_depth_stereo_0" (the model root, confirmed via
# `gz topic -e` to be first in the vector, but filtered by name, not
# position, so ordering doesn't matter).
ros2 run ros_gz_bridge parameter_bridge \
  /stereo/left/image@sensor_msgs/msg/Image[gz.msgs.Image \
  /stereo/right/image@sensor_msgs/msg/Image[gz.msgs.Image \
  /stereo/left/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo \
  /stereo/right/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo \
  /stereo/imu@sensor_msgs/msg/Imu[gz.msgs.IMU \
  /depth_camera@sensor_msgs/msg/Image[gz.msgs.Image \
  /depth_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked \
  /camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo \
  /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock \
  /world/vio_test/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V \
  > "$LOG_DIR/bridge.log" 2>&1 &
sleep 3

echo "=== [4/7] ground_truth_tf (use_sim_time:=true) ==="
ros2 run px4_vslam_bridge ground_truth_tf --ros-args -p use_sim_time:=true \
  > "$LOG_DIR/ground_truth_tf.log" 2>&1 &
sleep 2

echo "=== [5/7] cuVSLAM ==="
ros2 launch px4_vslam_bridge vslam.launch.py \
  > "$LOG_DIR/vslam.log" 2>&1 &
sleep 5

echo "=== [6/7] nvblox ==="
ros2 launch px4_vslam_bridge nvblox.launch.py \
  > "$LOG_DIR/nvblox.log" 2>&1 &
sleep 5

echo "=== [7/7] fast_planner_node + traj_server (NOT the arming bridge) ==="
ros2 launch plan_manage fast_planner_px4.launch.py \
  > "$LOG_DIR/fast_planner.log" 2>&1 &
echo "    letting the map populate before health check..."
sleep 20

echo ""
echo "=== Health check ==="
python3 "$WS/health_check.py"
HEALTH_STATUS=$?

echo ""
if [[ $HEALTH_STATUS -eq 0 ]]; then
  echo "READY FOR SUPERVISED FLIGHT."
  echo "fast_planner_bridge and fast_planner_trigger are NOT started by this"
  echo "script -- start them manually, deliberately, with force-disarm ready:"
  echo "  ros2 run px4_vslam_bridge fast_planner_bridge   # arms + climbs"
  echo "  ros2 run px4_vslam_bridge fast_planner_trigger  # starts the flight"
else
  echo "NOT READY -- see the failure above. Fix it before starting anything"
  echo "that can arm the vehicle."
  exit 1
fi
