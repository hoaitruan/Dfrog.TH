#!/usr/bin/env bash
# Route 2, STAGE R2-2: the decisive "55m regime" reproduction attempt.
# Sibling of run_r2_probe.sh (R2-0/R2-1), NOT a modification of it --
# that script stays as the record of those stages. Differences from it,
# all deliberate:
#
#   - Uses reactive_esdf_avoidance (Phase 6's ACTUAL original-incident
#     control node -- see README.md's "Custom nodes" table and Phase 6
#     history), not fast_planner_bridge/fast_planner_trigger. It is
#     self-contained: its own offboard/arm/climb state machine, then a
#     20Hz setpoint loop driven by a ~3Hz potential-field step computed
#     from nvblox's real ESDF (attract to a fixed goal past the pillar
#     ring, repel from the nearest known obstacle voxel) -- the actual
#     coupled cuVSLAM->EKF->control AND nvblox->avoidance->control stack
#     this stage exists to test, not Fast-Planner's separate kinodynamic
#     planner used in every other stage/gate in this project. NOT
#     modified (flight-control code) -- used exactly as committed.
#   - Runs against the OBSTACLE world (vio_test.sdf, default GZ_TARGET/
#     GZ_WORLD_NAME, rich texture, real markers/pillars) -- not R2-0/R2-1's
#     obstacle-free world. reactive_esdf_avoidance's own goal, GOAL_ENU =
#     (7,0,5), is deliberately set past pillar_01 at (2.5,0,~5) so a
#     straight line to it is blocked -- this IS the original "cruise
#     near obstacles" trajectory.
#   - Contention is REAL nvblox load (integrate_depth_rate_hz), not the
#     synthetic PyTorch stressor -- L0/L1 differ only in this rate; L2
#     adds the synthetic extreme stressor on top of L1's rate, the
#     worst-case combined condition.
#
# Still: airframe 4900 (EKF2_EV_CTRL=11) assumed already running,
# visual_odometry_bridge, r2_killswitch.py, confirmed-disarm cleanup,
# same EV aid-source topic recording -- all unchanged from run_r2_probe.sh.

set -eo pipefail

CHILD_PIDS=()
CLEANED_UP=0

if [[ "$(whoami)" != "admin" ]]; then
  echo "run_r2_2_probe.sh: must run as admin (got '$(whoami)')." >&2
  exit 1
fi

WS=/workspaces/isaac_ros-dev
CONDITION=""
DURATION=45
EXP_ID=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --condition) CONDITION="$2"; shift 2 ;;
    --duration) DURATION="$2"; shift 2 ;;
    --exp-id) EXP_ID="$2"; shift 2 ;;
    *) echo "run_r2_2_probe.sh: unknown argument: $1" >&2; exit 1 ;;
  esac
done

case "$CONDITION" in L0|L1|L2) ;; *)
  echo "run_r2_2_probe.sh: --condition must be one of L0|L1|L2 (got '$CONDITION')" >&2; exit 1 ;;
esac
if [[ -z "$EXP_ID" ]]; then
  EXP_ID="r2_2_${CONDITION}_$(date +%Y%m%d_%H%M%S)"
fi

# L0 = nvblox light (the "none" preset rate from run_gate.sh's existing
# scheme -- there is no true "nvblox off" condition here since
# reactive_esdf_avoidance NEEDS nvblox's ESDF for navigation; "light" is
# nvblox's own lowest-load operating rate, not its absence).
# L1 = nvblox heavy ("extreme" preset rate).
# L2 = L1's rate + synthetic gpu_stressor.py --intensity extreme on top
# -- the explicit worst-case combined condition.
declare -A NVBLOX_RATE_HZ=( [L0]=40.0 [L1]=640.0 [L2]=640.0 )

OUT_DIR="$WS/results/route2_probe/$EXP_ID"
mkdir -p "$OUT_DIR"
echo "run_r2_2_probe.sh: exp_id=$EXP_ID condition=$CONDITION -> $OUT_DIR"

source /opt/ros/humble/setup.bash
source "$WS/ros2_ws/install/setup.bash"

echo "=== Checking base pipeline is up ==="
PIPELINE_UP=0
for i in $(seq 1 20); do
  if ros2 topic list 2>/dev/null | grep -q "^/visual_slam/tracking/odometry$"; then
    PIPELINE_UP=1
    break
  fi
  sleep 1
done
if [[ "$PIPELINE_UP" -ne 1 ]]; then
  echo "run_r2_2_probe.sh: /visual_slam/tracking/odometry not found after 20s -- is run.sh's pipeline up?" >&2
  exit 1
fi
echo "  OK"

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
  echo "run_r2_2_probe.sh: cleaning up (${#CHILD_PIDS[@]} tracked process group(s))..."
  for pid in "${CHILD_PIDS[@]}"; do
    kill_group "$pid"
  done
  pkill -f "reactive_esdf_avoidance" 2>/dev/null || true
  pkill -f "vslam_compare" 2>/dev/null || true
  pkill -f "gpu_stressor.py" 2>/dev/null || true
  pkill -f "gpu_sampler.py" 2>/dev/null || true
  pkill -f "r2_killswitch.py" 2>/dev/null || true
  pkill -f "visual_odometry_bridge" 2>/dev/null || true

  echo "  restoring nvblox to default rate..."
  pkill -f "nvblox.launch.py" 2>/dev/null || true
  sleep 2
  setsid ros2 launch px4_vslam_bridge nvblox.launch.py \
    > "$OUT_DIR/nvblox_restore.log" 2>&1 < /dev/null &
  sleep 5

  echo "  force-disarming (idempotent, safe even if never armed; blocks until confirmed)..."
  if ! timeout 15 python3 "$WS/force_disarm.py" > "$OUT_DIR/force_disarm_cleanup.log" 2>&1; then
    echo "  WARNING: force-disarm NOT CONFIRMED within timeout -- see $OUT_DIR/force_disarm_cleanup.log" >&2
  fi

  if [[ -f "$OUT_DIR/runaway_event.json" ]]; then
    echo "  RUNAWAY CAPTURED this run -- see $OUT_DIR/runaway_event.json (this is data, not a script failure)"
  fi
}
trap cleanup EXIT INT TERM ERR

RATE="${NVBLOX_RATE_HZ[$CONDITION]}"
echo "=== Relaunching nvblox_container at integrate_depth_rate_hz=$RATE (condition=$CONDITION) ==="
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

if [[ "$CONDITION" == "L2" ]]; then
  echo "=== L2: also starting gpu_stressor.py --intensity extreme (worst-case combined load) ==="
  setsid python3 "$WS/tools/feasibility_gate/gpu_stressor.py" \
    --intensity extreme --duration $((DURATION + 15)) \
    > "$OUT_DIR/gpu_stressor.log" 2>&1 < /dev/null &
  CHILD_PIDS+=("$!")
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
# Same topic set as run_r2_probe.sh -- estimated (fused) pose, ground
# truth (used post-hoc for both divergence AND obstacle-clearance/
# collision computation against vio_test.sdf's known, fixed obstacle
# geometry -- no live collision sensor needed), cuVSLAM's own odometry/
# status/features, the 3 EV aid-source topics, arming state.
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

echo "=== Starting reactive_esdf_avoidance (arms, climbs, then flies the cruise-near-obstacle goal) ==="
setsid env OFFBOARD_AUTO_START=1 ros2 run px4_vslam_bridge reactive_esdf_avoidance \
  > "$OUT_DIR/reactive_esdf_avoidance.log" 2>&1 < /dev/null &
CHILD_PIDS+=("$!")
sleep "$DURATION"

echo ""
echo "run_r2_2_probe.sh: scenario $EXP_ID done. Outputs in $OUT_DIR"
echo "  - vslam_compare.csv, gpu_log.csv, rosbag/, runaway_event.json (if triggered)"
