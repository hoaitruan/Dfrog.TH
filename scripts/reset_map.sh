#!/usr/bin/env bash
# Clears the current cuVSLAM map/trajectory and starts a fresh tracking session,
# without restarting the camera/pipeline. Run run_cuvslam_mapping.sh first.

set -euo pipefail
CONTAINER=isaac_ros_dev-x86_64-container

docker exec "$CONTAINER" bash -c "
  source /opt/ros/humble/setup.bash
  ros2 service call /visual_slam/reset isaac_ros_visual_slam_interfaces/srv/Reset '{}'
"
