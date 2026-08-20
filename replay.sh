#!/usr/bin/env bash
# Offline replay of a flight rosbag for smooth free-look review in RViz2.
# Deliberately does NOT touch the container or bring up the live pipeline --
# it only stops heavy GPU consumers (if any happen to be running) to give
# RViz2 the full GPU, then plays the bag + RViz2 alone.
#
# Must be run as admin inside the container:
#   docker exec -u admin -it isaac_ros_dev_persistent bash /workspaces/isaac_ros-dev/replay.sh [BAG_DIR]
# (docker exec defaults to root, which silently receives zero DDS data --
# same incident as run.sh/record.sh.)
#
# BAG_DIR defaults to the most recently recorded bag under
# ros2_ws/rosbags/. Known limitation of the current flight bag
# (flight_20260812_202601): frame_id split -- /ground_truth/odom is
# frame_id=map (matches verification.rviz's Fixed Frame, renders fine with
# zero TF), but /planning/travel_traj and /planning/position_cmd_vis are
# frame_id=world, which cannot resolve to map without a TF edge this bag
# doesn't have. Only one of {ground truth path, planner trajectory markers}
# renders at a time in this specific bag -- see README/report for detail.
# Also: /nvblox_node/static_esdf_pointcloud recorded with 0 messages in that
# bag (nvblox's lazy-publish never fired), so the ESDF display will be
# empty for that bag regardless of RViz config.

set -eo pipefail

if [[ "$(whoami)" != "admin" ]]; then
  echo "replay.sh: must run as admin (got '$(whoami)')." >&2
  exit 1
fi

WS=/workspaces/isaac_ros-dev
ROSBAGS_DIR="$WS/ros2_ws/rosbags"

BAG_DIR="${1:-}"
if [[ -z "$BAG_DIR" ]]; then
  BAG_DIR="$(ls -dt "$ROSBAGS_DIR"/flight_*/ 2>/dev/null | head -n1)"
  if [[ -z "$BAG_DIR" ]]; then
    echo "replay.sh: no bag given and none found under $ROSBAGS_DIR" >&2
    exit 1
  fi
fi
BAG_DIR="${BAG_DIR%/}"
echo "Replaying bag: $BAG_DIR"

source /opt/ros/humble/setup.bash
source "$WS/ros2_ws/install/setup.bash"

echo "=== Freeing the GPU: stopping any heavy pipeline processes (idempotent) ==="
# All best-effort -- fine if none of these are running (e.g. container was
# just started bare for replay and nothing heavy came up at all).
pkill -f "px4_sitl_default/bin/px4"        2>/dev/null || true
pkill -f "gz sim"                          2>/dev/null || true
pkill -f "ruby.*gz"                        2>/dev/null || true
pkill -f "visual_slam_launch_container"    2>/dev/null || true
pkill -f "nvblox_container"                2>/dev/null || true
pkill -f "ros_gz_bridge"                   2>/dev/null || true
pkill -f "ground_truth_tf"                 2>/dev/null || true
pkill -f "fast_planner_node"               2>/dev/null || true
pkill -f "traj_server"                     2>/dev/null || true
pkill -f "MicroXRCEAgent"                  2>/dev/null || true
sleep 1

cleanup() {
  echo ""
  echo "=== Stopping bag playback ==="
  if [[ -n "${PLAY_PID:-}" ]] && kill -0 "$PLAY_PID" 2>/dev/null; then
    kill "$PLAY_PID" 2>/dev/null || true
    wait "$PLAY_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

echo "=== Starting bag playback (--clock --loop) ==="
ros2 bag play "$BAG_DIR" --clock --loop &
PLAY_PID=$!
sleep 2

echo "=== Launching RViz2 (verification.rviz, use_sim_time:=true) ==="
rviz2 -d "$WS/ros2_ws/verification.rviz" --ros-args -p use_sim_time:=true

# When RViz2 exits (user closes it), cleanup() stops bag playback.
