#!/usr/bin/env bash
# G2: single-scenario runner for the open-loop feasibility gate.
#
# ASSUMES run.sh's base pipeline is ALREADY up and has passed
# health_check.py: MicroXRCEAgent, PX4 SITL+Gazebo, ros_gz bridge,
# ground_truth_tf, cuVSLAM, nvblox, fast_planner_node/traj_server. This
# script does not start or stop that pipeline -- it only layers on: the
# contention workload for ONE scenario, a dedicated CSV-logging
# vslam_compare instance, a topic-scoped rosbag, and (optionally) triggers
# the SAME already-validated, unmodified fast_planner flight path
# (fast_planner_bridge + fast_planner_trigger) used throughout Milestone 1.
#
# Hard constraints this script respects (do not change):
#   - Never touches EKF2_EV_CTRL (stays 0 the whole time -- nothing here
#     arms external vision fusion).
#   - Does not modify flight-control/estimator/airframe code. The flight
#     trigger step below calls existing, unmodified binaries.
#
# SCOPE CORRECTION (2026-09, from the task owner): this measures raw
# cuVSLAM pose error vs /ground_truth/odom across contention levels. It
# does NOT reproduce the original ~55m closed-loop runaway -- that
# required EKF2_EV_CTRL nonzero, which this gate never sets. See
# triage_incident.py's docstring for the full reasoning.
#
# TEXTURE IS NOT SELECTED BY THIS SCRIPT. It's a property of which Gazebo
# world PX4 SITL was launched against (run.sh currently hardcodes
# `gz_x500_depth_stereo_vio_test`, i.e. texture-rich vio_test.sdf) BEFORE
# this script runs. A texture-poor world variant + matching PX4 make
# target do not exist yet -- that's a G3 prep deliverable, not built here.
# --texture below only labels the experiment id/metadata so
# analyze_sweep.py can group by it later -- get it right by hand to match
# whatever world is actually running.
#
# CONTENTION KNOB, workload=nvblox: nvblox_ros reads integrate_depth_rate_hz
# exactly ONCE at node construction (verified against node_params.cpp /
# nvblox_node.cpp -- no add_on_set_parameters_callback exists in
# nvblox_ros). `ros2 param set` on a running node is a silent no-op. The
# ONLY way to actually change it is to kill nvblox_container and relaunch
# nvblox.launch.py with the new value, which is what this script does.
# This means nvblox's ESDF map is empty again after each relaunch (fine --
# this gate doesn't depend on ESDF content, only on nvblox's GPU activity
# as a load source) and takes a few seconds to reinitialize.
#
# Usage:
#   bash run_gate.sh --contention <none|low|medium|high|extreme> \
#                     --workload <nvblox|synthetic> \
#                     --texture <rich|poor> \
#                     [--duration 60] [--exp-id <id>] [--no-flight]
#
#   --no-flight: skip the fast_planner_bridge/trigger arm+flight step --
#   just run the contention workload + logging while the vehicle sits on
#   the ground. Useful for a first dry run of the instrumentation itself
#   before committing to a real flight for every one of a ~50-run sweep.
#
# Must be run as admin inside isaac_ros_dev_persistent, with the base
# pipeline already up:
#   docker exec -u admin -it isaac_ros_dev_persistent \
#     bash /workspaces/isaac_ros-dev/tools/feasibility_gate/run_gate.sh ...

set -eo pipefail

if [[ "$(whoami)" != "admin" ]]; then
  echo "run_gate.sh: must run as admin (got '$(whoami)')." >&2
  exit 1
fi

WS=/workspaces/isaac_ros-dev
CONTENTION=""
WORKLOAD=""
TEXTURE=""
DURATION=60
EXP_ID=""
NO_FLIGHT=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --contention) CONTENTION="$2"; shift 2 ;;
    --workload) WORKLOAD="$2"; shift 2 ;;
    --texture) TEXTURE="$2"; shift 2 ;;
    --duration) DURATION="$2"; shift 2 ;;
    --exp-id) EXP_ID="$2"; shift 2 ;;
    --no-flight) NO_FLIGHT=1; shift 1 ;;
    *) echo "run_gate.sh: unknown argument: $1" >&2; exit 1 ;;
  esac
done

case "$CONTENTION" in none|low|medium|high|extreme) ;; *)
  echo "run_gate.sh: --contention must be one of none|low|medium|high|extreme (got '$CONTENTION')" >&2; exit 1 ;;
esac
case "$WORKLOAD" in nvblox|synthetic) ;; *)
  echo "run_gate.sh: --workload must be one of nvblox|synthetic (got '$WORKLOAD')" >&2; exit 1 ;;
esac
case "$TEXTURE" in rich|poor) ;; *)
  echo "run_gate.sh: --texture must be one of rich|poor (got '$TEXTURE')" >&2; exit 1 ;;
esac

if [[ -z "$EXP_ID" ]]; then
  EXP_ID="${CONTENTION}_${WORKLOAD}_${TEXTURE}_$(date +%Y%m%d_%H%M%S)"
fi

OUT_DIR="$WS/results/feasibility_gate/$EXP_ID"
mkdir -p "$OUT_DIR"
echo "run_gate.sh: exp_id=$EXP_ID -> $OUT_DIR"

source /opt/ros/humble/setup.bash
source "$WS/ros2_ws/install/setup.bash"

# --- Precondition: base pipeline already up -------------------------------
echo "=== Checking base pipeline is up ==="
if ! ros2 topic list 2>/dev/null | grep -q "^/visual_slam/tracking/odometry$"; then
  echo "run_gate.sh: /visual_slam/tracking/odometry not found -- is run.sh's" >&2
  echo "  pipeline running and past health_check.py? Start it first:" >&2
  echo "  docker exec -u admin -it isaac_ros_dev_persistent bash $WS/run.sh" >&2
  exit 1
fi
if ! ros2 service list 2>/dev/null | grep -q "^/nvblox_node/get_esdf_and_gradient$"; then
  echo "run_gate.sh: /nvblox_node/get_esdf_and_gradient service not found." >&2
  exit 1
fi
echo "  OK"

# --- Contention knob -------------------------------------------------------
declare -A NVBLOX_RATE_HZ=( [none]=40.0 [low]=80.0 [medium]=160.0 [high]=320.0 [extreme]=640.0 )
GPU_STRESSOR_PID=""

cleanup() {
  echo ""
  echo "run_gate.sh: cleaning up..."
  [[ -n "$GPU_STRESSOR_PID" ]] && kill "$GPU_STRESSOR_PID" 2>/dev/null || true
  [[ -n "${GPU_SAMPLER_PID:-}" ]] && kill "$GPU_SAMPLER_PID" 2>/dev/null || true
  [[ -n "${VSLAM_COMPARE_PID:-}" ]] && kill "$VSLAM_COMPARE_PID" 2>/dev/null || true
  [[ -n "${BAG_PID:-}" ]] && kill "$BAG_PID" 2>/dev/null || true
  if [[ "$WORKLOAD" == "nvblox" && "$CONTENTION" != "none" ]]; then
    echo "  restoring nvblox to default rate..."
    pkill -f "nvblox.launch.py" 2>/dev/null || true
    sleep 2
    ros2 launch px4_vslam_bridge nvblox.launch.py > "$OUT_DIR/nvblox_restore.log" 2>&1 &
    sleep 5
  fi
}
trap cleanup EXIT INT TERM

if [[ "$WORKLOAD" == "nvblox" ]]; then
  RATE="${NVBLOX_RATE_HZ[$CONTENTION]}"
  echo "=== workload=nvblox: relaunching nvblox_container at integrate_depth_rate_hz=$RATE ==="
  pkill -f "nvblox.launch.py" 2>/dev/null || true
  sleep 2
  ros2 launch px4_vslam_bridge nvblox.launch.py \
    integrate_depth_rate_hz:="$RATE" \
    > "$OUT_DIR/nvblox.log" 2>&1 &
  echo "  waiting for nvblox to come back up..."
  for i in $(seq 1 20); do
    if ros2 service list 2>/dev/null | grep -q "^/nvblox_node/get_esdf_and_gradient$"; then
      break
    fi
    sleep 1
  done
  if ! ros2 service list 2>/dev/null | grep -q "^/nvblox_node/get_esdf_and_gradient$"; then
    echo "run_gate.sh: nvblox did not come back up after relaunch -- see $OUT_DIR/nvblox.log" >&2
    exit 1
  fi
  echo "  OK"
elif [[ "$WORKLOAD" == "synthetic" ]]; then
  echo "=== workload=synthetic: starting gpu_stressor.py --intensity $CONTENTION ==="
  if [[ "$CONTENTION" != "none" ]]; then
    python3 "$WS/tools/feasibility_gate/gpu_stressor.py" \
      --intensity "$CONTENTION" --duration $((DURATION + 15)) \
      > "$OUT_DIR/gpu_stressor.log" 2>&1 &
    GPU_STRESSOR_PID=$!
  fi
fi

# --- Logging ----------------------------------------------------------------
echo "=== Starting gpu_sampler.py ==="
python3 "$WS/tools/feasibility_gate/gpu_sampler.py" \
  --out "$OUT_DIR/gpu_log.csv" --interval 0.2 --duration $((DURATION + 15)) \
  > "$OUT_DIR/gpu_sampler.log" 2>&1 &
GPU_SAMPLER_PID=$!

echo "=== Starting dedicated vslam_compare (CSV logging) ==="
ros2 run px4_vslam_bridge vslam_compare --ros-args \
  -p csv_path:="$OUT_DIR/vslam_compare.csv" -p exp_id:="$EXP_ID" \
  > "$OUT_DIR/vslam_compare_node.log" 2>&1 &
VSLAM_COMPARE_PID=$!
sleep 2

echo "=== Starting rosbag record ==="
BAG_LOG="$OUT_DIR/bag_record.log"
ros2 bag record -o "$OUT_DIR/rosbag" \
  /tf /tf_static /clock \
  /ground_truth/odom \
  /visual_slam/tracking/odometry \
  /visual_slam/status \
  /visual_slam/vis/observations_cloud \
  /fmu/out/estimator_status_flags \
  > "$BAG_LOG" 2>&1 &
BAG_PID=$!
echo "  waiting for rosbag2_recorder to confirm topic subscriptions..."
for i in $(seq 1 100); do
  grep -q "All requested topics are subscribed" "$BAG_LOG" 2>/dev/null && break
  sleep 0.2
done

# --- Flight trigger -----------------------------------------------------
if [[ "$NO_FLIGHT" -eq 1 ]]; then
  echo "=== --no-flight: skipping arm/climb/trigger, just logging for ${DURATION}s ==="
  sleep "$DURATION"
else
  echo "=== Arming + climbing (fast_planner_bridge) ==="
  echo "    Operator: watch Gazebo / ground truth per the project's established"
  echo "    safe-testing protocol. hard_killswitch.py / force_disarm.py should"
  echo "    be available in another shell before proceeding."
  ros2 run px4_vslam_bridge fast_planner_bridge > "$OUT_DIR/fast_planner_bridge.log" 2>&1 &
  FPB_PID=$!
  sleep 8
  echo "=== Triggering flight (fast_planner_trigger) ==="
  timeout "$DURATION" ros2 run px4_vslam_bridge fast_planner_trigger \
    > "$OUT_DIR/fast_planner_trigger.log" 2>&1 || true
  kill "$FPB_PID" 2>/dev/null || true
fi

echo ""
echo "run_gate.sh: scenario $EXP_ID done. Outputs in $OUT_DIR"
echo "  - vslam_compare.csv, gpu_log.csv, rosbag/"
echo "Run triage on it with:"
echo "  python3 $WS/tools/feasibility_gate/triage_incident.py \\"
echo "    --bag $OUT_DIR/rosbag --gpu-log $OUT_DIR/gpu_log.csv --out-dir $OUT_DIR/triage"
