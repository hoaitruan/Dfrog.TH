#!/usr/bin/env bash
# Task 6: rosbag the supervised flight. The prior "successful avoidance
# flight" could not be substantiated from any surviving log -- PX4's own
# ulog silently failed to capture it (the container died mid-session
# before the logger flushed). Trust this rosbag, not the ulog.
#
# Must be run as admin inside the container:
#   docker exec -u admin -it isaac_ros_dev_persistent bash /workspaces/isaac_ros-dev/record.sh
#
# Start this, wait for the "SAFE TO ARM/TRIGGER" line below, THEN arm/
# trigger. Ctrl-C to stop after the flight settles.
#
# Post-flight_20260812_202601 fixes:
#  - /tf and /tf_static are now recorded. Without them the previous bag
#    could not resolve /planning/travel_traj and /planning/position_cmd_vis
#    (frame_id=world) against /ground_truth/odom (frame_id=map) under any
#    single RViz Fixed Frame -- confirmed during the offline-replay audit.
#    (Phase A of the world/map investigation then proved world==map by
#    construction -- /odom_world is a straight remap of /ground_truth/odom
#    with zero transform code anywhere in Fast-Planner -- so a static
#    identity transform_publisher map->world is the correct, permanent
#    fix, not a workaround. It must be running before this script starts
#    recording, same as the rest of the pipeline.)
#
#  - ESDF RECORDING: esdf_keepalive.py (a plain topic subscriber on
#    /nvblox_node/static_esdf_pointcloud) turned out to be built on a wrong
#    diagnosis. Traced through nvblox_ros source directly (not guessed):
#    nvblox.launch.py forces esdf_mode: "3d" (Fast-Planner needs full 3D
#    ESDF, not a 2D slice); nvblox_node.cpp gates the ENTIRE subscriber-
#    driven publish path for static_esdf_pointcloud_publisher_ -- the
#    get_subscription_count() check included -- behind
#    `if (esdf_mode == EsdfMode::k2D)`; that publisher's ->publish() call
#    has exactly one call site in the whole nvblox_ros source, inside that
#    same k2D-only block. In our required 3D mode this is dead code -- no
#    subscriber, no QoS profile, no wait time will ever make it publish.
#    (The other trigger, visualize_esdf=true on the ESDF service, doesn't
#    publish ESDF either -- LayerPublisher only streams mesh/tsdf/color/
#    freespace/dynamic-occupancy layers, no ESDF layer at all.)
#    The only live path to ESDF data in 3D mode is the
#    /nvblox_node/get_esdf_and_gradient service response itself -- the
#    same one Fast-Planner's sdf_map.cpp and health_check.py already use.
#    esdf_recorder.py (replaces esdf_keepalive.py) calls that service on a
#    timer and republishes the response as a normal PointCloud2 on
#    /esdf_recorder/pointcloud, which is what's now recorded instead of
#    the permanently-empty /nvblox_node/static_esdf_pointcloud. No QoS
#    override needed -- esdf_recorder.py's own publisher uses default
#    QoS, which ros2 bag record's default reader already matches; the
#    earlier qos_overrides.yaml assumed a reliability mismatch that
#    doesn't exist (nvblox's publisher is actually RELIABLE, confirmed via
#    `ros2 topic info -v`) and is unused now.
#
#  - DISCOVERY RACE FIX: the flight_20260812_202601 bag's /planning/
#    position_cmd_vis and /planning/travel_traj were recorded with data
#    only from the frozen post-trajectory hover point -- the entire ~6-8s
#    active trajectory phase was missed. Root cause: record.sh launched
#    `ros2 bag record` and returned immediately, so the operator triggered
#    the flight before ros2 bag record's DDS subscriptions had finished
#    matching traj_server's publishers -- messages published before match
#    completes are simply never delivered, with no error.
#    First fix attempt polled `ros2 topic info <topic>` in a loop -- this
#    itself turned out broken: each `ros2 topic info` CLI invocation pays
#    real rclpy-node-startup + graph-discovery overhead (~2-5s observed),
#    so three of them per poll iteration blew the 15s budget even though
#    rosbag2_recorder's OWN log already showed "All requested topics are
#    subscribed" within ~130ms of starting. Fixed for real below: watch
#    `ros2 bag record`'s own stdout (redirected to BAG_LOG) for that exact
#    line -- it's emitted directly by rosbag2_recorder once every
#    requested topic (not just a hand-picked subset) has a matched
#    subscriber, and a `grep` on a local file has none of the CLI
#    startup cost that broke the topic-info-polling approach.

set -eo pipefail

if [[ "$(whoami)" != "admin" ]]; then
  echo "record.sh: must run as admin (got '$(whoami)')." >&2
  exit 1
fi

# ROS2's own setup.bash scripts reference unset variables internally --
# set -u (nounset) fights that, so it's deliberately left off, unlike
# run.sh which doesn't source anything upstream before its own checks.
source /opt/ros/humble/setup.bash
source /workspaces/isaac_ros-dev/ros2_ws/install/setup.bash

BAG_DIR="/workspaces/isaac_ros-dev/ros2_ws/rosbags/flight_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$(dirname "$BAG_DIR")"

echo "Checking for map->world static transform..."
# tf2_echo streams forever, so `timeout` always kills it (exit 124) even
# when the transform resolves fine -- check the actual output for a
# successful lookup instead of the exit code.
TF_CHECK_OUTPUT=$(timeout 3 ros2 run tf2_ros tf2_echo map world 2>&1 || true)
if ! echo "$TF_CHECK_OUTPUT" | grep -q "Translation:"; then
  echo "record.sh: no map->world transform found. Start it first:" >&2
  echo "  ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 --frame-id map --child-frame-id world" >&2
  exit 1
fi
echo "map->world transform confirmed."

echo "Starting esdf_recorder (calls the ESDF service on a timer, republishes as PointCloud2)..."
python3 /workspaces/isaac_ros-dev/esdf_recorder.py > /tmp/esdf_recorder.log 2>&1 &
RECORDER_PID=$!
sleep 2

cleanup() {
  echo ""
  echo "Stopping esdf_recorder..."
  kill "$RECORDER_PID" 2>/dev/null || true
  wait "$RECORDER_PID" 2>/dev/null || true
  if [[ -n "${TAIL_PID:-}" ]]; then
    kill "$TAIL_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

echo "Recording to: $BAG_DIR"
echo "Topics: TF, ground truth path, ESDF (via esdf_recorder bridge), planner trajectory/commands, PX4 status"

BAG_LOG="/tmp/bag_record_output.log"
ros2 bag record -o "$BAG_DIR" \
  /tf \
  /tf_static \
  /ground_truth/odom \
  /esdf_recorder/pointcloud \
  /planning/travel_traj \
  /planning_vis/trajectory \
  /planning/position_cmd_vis \
  /planning/pos_cmd \
  /fmu/out/vehicle_local_position_v1 \
  /fmu/out/vehicle_status_v4 \
  /fmu/out/failsafe_flags > "$BAG_LOG" 2>&1 &
BAG_PID=$!

# Discovery-race fix: block here until rosbag2_recorder's own log confirms
# every requested topic has a matched subscriber, so the operator doesn't
# trigger the flight into a recorder that hasn't finished DDS discovery.
echo "Waiting for rosbag2_recorder to confirm all topics subscribed..."
ATTEMPTS=0
MAX_ATTEMPTS=100  # 100 * 0.2s = 20s
MATCHED=false
while (( ATTEMPTS < MAX_ATTEMPTS )); do
  if grep -q "All requested topics are subscribed" "$BAG_LOG" 2>/dev/null; then
    MATCHED=true
    break
  fi
  sleep 0.2
  ATTEMPTS=$((ATTEMPTS + 1))
done

tail -n +1 "$BAG_LOG"

if $MATCHED; then
  echo ""
  echo "=== SAFE TO ARM/TRIGGER NOW -- recorder is matched and capturing live. ==="
  echo ""
  tail -f "$BAG_LOG" &
  TAIL_PID=$!
else
  echo "record.sh: recorder did NOT confirm all topics subscribed within 20s -- NOT safe to trigger yet." >&2
  echo "Check that fast_planner_node/traj_server are actually running." >&2
fi

wait "$BAG_PID"
