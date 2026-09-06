#!/usr/bin/env bash
# Route 2 CLOSED-LOOP probe runner. Sibling of
# tools/feasibility_gate/run_gate.sh, NOT a modification of it -- that
# script is relied on by every open-loop feasibility-gate experiment and
# stays untouched. This one is EKF2_EV_CTRL-aware by design: it assumes
# the base pipeline was started with PX4_SYS_AUTOSTART=4900 (the
# EKF2_EV_CTRL=11 airframe) against the obstacle-free
# gz_x500_depth_stereo_vio_test_r2_obstaclefree world (see run.sh's
# GZ_TARGET/GZ_WORLD_NAME/PX4_SYS_AUTOSTART and
# flight_test_log.html's Route 2 setup entry).
#
# Adds on top of the base pipeline: visual_odometry_bridge (republishes
# cuVSLAM as VehicleOdometry -- NOT started by run.sh, since it would be
# dead weight for every EKF2_EV_CTRL=0 experiment), r2_killswitch.py
# (geofence + est-vs-ground-truth divergence abort, see that file), the
# contention workload, a topic-scoped rosbag, and the flight trigger.
#
# Hard constraint this script respects: never changes EKF2_EV_CTRL
# itself -- that's fixed by which airframe (SYS_AUTOSTART) the base
# pipeline was launched with, before this script runs.
#
# Usage:
#   bash run_r2_probe.sh --contention <none|low|medium|high|extreme> \
#                         --workload <synthetic|nvblox> \
#                         [--duration 30] --exp-id <id>
#
# Outputs (results/route2_probe/<exp-id>/):
#   rosbag/ (estimated pose, ground truth, EV innovation/accept-reject,
#   cuVSLAM status/features, arming state), gpu_log.csv, and
#   runaway_event.json IF r2_killswitch triggered (that's data, not an
#   error -- STAGE analysis reads for its presence).

set -eo pipefail

CHILD_PIDS=()
CLEANED_UP=0

if [[ "$(whoami)" != "admin" ]]; then
  echo "run_r2_probe.sh: must run as admin (got '$(whoami)')." >&2
  exit 1
fi

WS=/workspaces/isaac_ros-dev
CONTENTION=""
WORKLOAD="synthetic"
DURATION=30
EXP_ID=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --contention) CONTENTION="$2"; shift 2 ;;
    --workload) WORKLOAD="$2"; shift 2 ;;
    --duration) DURATION="$2"; shift 2 ;;
    --exp-id) EXP_ID="$2"; shift 2 ;;
    *) echo "run_r2_probe.sh: unknown argument: $1" >&2; exit 1 ;;
  esac
done

case "$CONTENTION" in none|low|medium|high|extreme) ;; *)
  echo "run_r2_probe.sh: --contention must be one of none|low|medium|high|extreme (got '$CONTENTION')" >&2; exit 1 ;;
esac
case "$WORKLOAD" in nvblox|synthetic) ;; *)
  echo "run_r2_probe.sh: --workload must be one of nvblox|synthetic (got '$WORKLOAD')" >&2; exit 1 ;;
esac
if [[ -z "$EXP_ID" ]]; then
  EXP_ID="r2_${CONTENTION}_${WORKLOAD}_$(date +%Y%m%d_%H%M%S)"
fi

OUT_DIR="$WS/results/route2_probe/$EXP_ID"
mkdir -p "$OUT_DIR"
echo "run_r2_probe.sh: exp_id=$EXP_ID -> $OUT_DIR"

source /opt/ros/humble/setup.bash
source "$WS/ros2_ws/install/setup.bash"

echo "=== Checking base pipeline is up ==="
# Retry, don't fail on the first miss: run.sh's own health check verifies
# generic /fmu/* topics and process liveness, but never specifically
# waits on /visual_slam/tracking/odometry -- confirmed live (twice) that
# health check can print PASS while cuVSLAM's component container is
# still mid-initialization (CUVSLAM_CreateTracker alone measured ~2s;
# under load, the whole vslam_launch_container startup can occasionally
# still be settling by the time this script's very first check runs
# immediately after run.sh returns). Same retry shape as the nvblox
# service-wait below, not a new pattern.
PIPELINE_UP=0
for i in $(seq 1 20); do
  if ros2 topic list 2>/dev/null | grep -q "^/visual_slam/tracking/odometry$"; then
    PIPELINE_UP=1
    break
  fi
  sleep 1
done
if [[ "$PIPELINE_UP" -ne 1 ]]; then
  echo "run_r2_probe.sh: /visual_slam/tracking/odometry not found after 20s -- is run.sh's pipeline up?" >&2
  exit 1
fi
echo "  OK"

declare -A NVBLOX_RATE_HZ=( [none]=40.0 [low]=80.0 [medium]=160.0 [high]=320.0 [extreme]=640.0 )

# Identical to run_gate.sh's kill_group -- see that file's own comment
# for why pgrep -g (group membership), not kill -0 on the tracked leader
# PID alone, is required here.
kill_group() {
  local pid="$1"
  pgrep -g "$pid" > /dev/null 2>&1 || return 0
  kill -TERM "-$pid" 2>/dev/null || true
  for i in $(seq 1 10); do
    pgrep -g "$pid" > /dev/null 2>&1 || return 0
    sleep 0.3
  done
  kill -KILL "-$pid" 2>/dev/null || true
  sleep 0.3
  wait "$pid" 2>/dev/null || true
}

cleanup() {
  [[ "$CLEANED_UP" -eq 1 ]] && return 0
  CLEANED_UP=1
  echo ""
  echo "run_r2_probe.sh: cleaning up (${#CHILD_PIDS[@]} tracked process group(s))..."
  for pid in "${CHILD_PIDS[@]}"; do
    kill_group "$pid"
  done
  pkill -f "fast_planner_bridge" 2>/dev/null || true
  pkill -f "fast_planner_trigger" 2>/dev/null || true
  pkill -f "vslam_compare" 2>/dev/null || true
  pkill -f "gpu_stressor.py" 2>/dev/null || true
  pkill -f "gpu_sampler.py" 2>/dev/null || true
  pkill -f "r2_killswitch.py" 2>/dev/null || true
  pkill -f "visual_odometry_bridge" 2>/dev/null || true

  if [[ "$WORKLOAD" == "nvblox" && "$CONTENTION" != "none" ]]; then
    echo "  restoring nvblox to default rate..."
    pkill -f "nvblox.launch.py" 2>/dev/null || true
    sleep 2
    setsid ros2 launch px4_vslam_bridge nvblox.launch.py \
      > "$OUT_DIR/nvblox_restore.log" 2>&1 < /dev/null &
    sleep 5
  fi

  # Unconditional, confirmed-disarm cleanup (same as run_gate.sh, per
  # §49's fix) -- the ONLY guarantee against "run ends, vehicle stays
  # armed" that matters even more here since EKF2_EV_CTRL is nonzero.
  echo "  force-disarming (idempotent, safe even if never armed; blocks until confirmed)..."
  if ! timeout 15 python3 "$WS/force_disarm.py" > "$OUT_DIR/force_disarm_cleanup.log" 2>&1; then
    echo "  WARNING: force-disarm NOT CONFIRMED within timeout -- see $OUT_DIR/force_disarm_cleanup.log" >&2
  fi

  if [[ -f "$OUT_DIR/runaway_event.json" ]]; then
    echo "  RUNAWAY CAPTURED this run -- see $OUT_DIR/runaway_event.json (this is data, not a script failure)"
  fi
}
trap cleanup EXIT INT TERM ERR

if [[ "$WORKLOAD" == "nvblox" ]]; then
  RATE="${NVBLOX_RATE_HZ[$CONTENTION]}"
  echo "=== workload=nvblox: relaunching nvblox_container at integrate_depth_rate_hz=$RATE ==="
  pkill -f "nvblox.launch.py" 2>/dev/null || true
  sleep 2
  setsid ros2 launch px4_vslam_bridge nvblox.launch.py \
    integrate_depth_rate_hz:="$RATE" \
    > "$OUT_DIR/nvblox.log" 2>&1 < /dev/null &
  CHILD_PIDS+=("$!")
  for i in $(seq 1 20); do
    ros2 service list 2>/dev/null | grep -q "^/nvblox_node/get_esdf_and_gradient$" && break
    sleep 1
  done
elif [[ "$WORKLOAD" == "synthetic" ]]; then
  echo "=== workload=synthetic: starting gpu_stressor.py --intensity $CONTENTION ==="
  if [[ "$CONTENTION" != "none" ]]; then
    setsid python3 "$WS/tools/feasibility_gate/gpu_stressor.py" \
      --intensity "$CONTENTION" --duration $((DURATION + 15)) \
      > "$OUT_DIR/gpu_stressor.log" 2>&1 < /dev/null &
    CHILD_PIDS+=("$!")
  fi
fi

echo "=== Starting gpu_sampler.py ==="
setsid python3 "$WS/tools/feasibility_gate/gpu_sampler.py" \
  --out "$OUT_DIR/gpu_log.csv" --interval 0.2 --duration $((DURATION + 15)) \
  > "$OUT_DIR/gpu_sampler.log" 2>&1 < /dev/null &
CHILD_PIDS+=("$!")

echo "=== Starting visual_odometry_bridge (cuVSLAM -> EKF2 EV input) ==="
setsid ros2 run px4_vslam_bridge visual_odometry_bridge \
  > "$OUT_DIR/visual_odometry_bridge.log" 2>&1 < /dev/null &
CHILD_PIDS+=("$!")
sleep 2

echo "=== Starting r2_killswitch.py (geofence + divergence abort watchdog) ==="
setsid python3 "$WS/tools/route2_probe/r2_killswitch.py" \
  --breach-file "$OUT_DIR/runaway_event.json" \
  > "$OUT_DIR/r2_killswitch.log" 2>&1 < /dev/null &
CHILD_PIDS+=("$!")
sleep 1

echo "=== Starting dedicated vslam_compare (CSV logging) ==="
setsid ros2 run px4_vslam_bridge vslam_compare --ros-args \
  -p csv_path:="$OUT_DIR/vslam_compare.csv" -p exp_id:="$EXP_ID" \
  > "$OUT_DIR/vslam_compare_node.log" 2>&1 < /dev/null &
CHILD_PIDS+=("$!")
sleep 2

echo "=== Starting rosbag record ==="
BAG_LOG="$OUT_DIR/bag_record.log"
# Recorded topic set for the closed-loop probe: estimated (fused) pose,
# ground truth, cuVSLAM's own odometry/status/features, the 3 EV
# aid-source topics (innovation/covariance/accept-reject -- newly
# whitelisted in dds_topics.yaml, see Route 2 setup entry), and arming
# state (also used live by r2_killswitch/force_disarm, bag copy is for
# post-hoc confirmation).
setsid ros2 bag record -o "$OUT_DIR/rosbag" \
  /fmu/out/vehicle_local_position_v1 \
  /ground_truth/odom \
  /visual_slam/tracking/odometry \
  /visual_slam/status \
  /visual_slam/vis/observations_cloud \
  /fmu/out/estimator_aid_src_ev_pos \
  /fmu/out/estimator_aid_src_ev_hgt \
  /fmu/out/estimator_aid_src_ev_yaw \
  /fmu/out/vehicle_status_v4 \
  > "$BAG_LOG" 2>&1 < /dev/null &
CHILD_PIDS+=("$!")
echo "  waiting for rosbag2_recorder to confirm topic subscriptions..."
for i in $(seq 1 100); do
  grep -q "All requested topics are subscribed" "$BAG_LOG" 2>/dev/null && break
  sleep 0.2
done

echo "=== Arming + climbing (fast_planner_bridge) ==="
setsid env OFFBOARD_AUTO_START=1 ros2 run px4_vslam_bridge fast_planner_bridge \
  > "$OUT_DIR/fast_planner_bridge.log" 2>&1 < /dev/null &
CHILD_PIDS+=("$!")
sleep 8
echo "=== Triggering flight (fast_planner_trigger) ==="
# NOT `timeout 15 ros2 run ... || true` (run_gate.sh's foreground
# pattern, inherited here at first) -- confirmed live this can leave a
# real stray: `ros2 run`'s wrapper is timeout's direct child, but the
# actual trigger entry-point is a separate grandchild in the same
# process group. When the trigger hangs (empty log, no "Trigger
# published" -- happened once in ~113 runs, cause not otherwise
# diagnosed) and 15s elapses, timeout's SIGTERM reaches the wrapper,
# the wrapper dies, and the still-hung grandchild gets reparented to
# PID 1 and keeps running -- exactly the same "ros2 run wrapper vs.
# grandchild" class of bug this project already hardened
# fast_planner_bridge/run_gate.sh's other children against (see
# kill_group()'s own comment). Fixed the same way: setsid + track PID +
# kill_group() on timeout, not a bare `timeout` on the wrapper alone.
setsid ros2 run px4_vslam_bridge fast_planner_trigger \
  > "$OUT_DIR/fast_planner_trigger.log" 2>&1 < /dev/null &
TRIGGER_PID=$!
TRIGGER_DEADLINE=$((SECONDS + 15))
while [[ $SECONDS -lt $TRIGGER_DEADLINE ]]; do
  pgrep -g "$TRIGGER_PID" > /dev/null 2>&1 || break
  sleep 0.2
done
if pgrep -g "$TRIGGER_PID" > /dev/null 2>&1; then
  echo "  fast_planner_trigger still running after 15s -- killing its process group" >&2
  kill_group "$TRIGGER_PID"
else
  wait "$TRIGGER_PID" 2>/dev/null || true
fi
sleep "$DURATION"

echo ""
echo "run_r2_probe.sh: scenario $EXP_ID done. Outputs in $OUT_DIR"
echo "  - vslam_compare.csv, gpu_log.csv, rosbag/, runaway_event.json (if triggered)"
