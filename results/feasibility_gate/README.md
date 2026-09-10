# Feasibility-gate result data

Result data for the open-loop GPU-contention-vs-cuVSLAM-tracking-accuracy
feasibility gate (branch `feasibility-gate`; harness and analysis code in
`tools/feasibility_gate/`). Findings are written up in
`results/feasibility_gate/REPORT.md` (provisional draft) and, as the
durable copy, in `flight_test_log.html` §42-54 and `docs/research_log.md`.

Brought into version control 2026-09-10 (previously entirely gitignored
per `CLAUDE.md`'s general "regenerable data, not source" convention) so
the study is reproducible from the repo, not only from summary numbers in
the log. See `docs/reconcile/RECONCILE_SUMMARY.md` for why and what was
left out.

## What's committed vs. what stays local

**Committed** (~75MB, ~1,970 files): per run, everything except the
rosbag — `bag_record.log`, `fast_planner_bridge.log`,
`fast_planner_trigger.log`, `force_disarm_cleanup.log`, `gpu_log.csv`
(GPU utilization/memory sampled by `gpu_sampler.py`), `gpu_sampler.log`,
`gpu_stressor.log`, `vslam_compare.csv` (a live per-tick cuVSLAM-vs-ground-
truth comparison logged during the flight by a dedicated `vslam_compare`
node instance — a companion live log, distinct from the bag-derived metric
below), `vslam_compare_node.log`. Plus two top-level files: `REPORT.md`
and `mini_dose_screen.png`.

**Not committed, stays local-only**: each run's `rosbag/` subdirectory
(`rosbag_0.db3` + `metadata.yaml`). **11.4GB total**, individual bags up to
881MB (`taskc_interrupt_test*`) — far past anything that belongs in git,
and excluded from history entirely (`.gitignore`:
`/results/feasibility_gate/*/rosbag/`). These live only on the machine
that generated them, under this same `results/feasibility_gate/<run>/`
path. **`analyze_sweep.py` (the tool that produced every number in
`REPORT.md`) reads the rosbag directly, not `vslam_compare.csv`** — so
re-deriving the exact published RPE-1s statistics from history requires
those local bags; they are not reproducible from the committed CSVs alone.
What committing the CSVs/logs does provide: the live-logged companion
metric for every run, full harness code + presets + seeds, and the means
to regenerate an equivalent sweep from scratch (below) — not to
reprocess these specific historical bags without them.

## Run directories

| Prefix | Purpose | Runs |
|---|---|---|
| `g3_1_baseline_*` | Pre-gate baseline variance characterization | 2 |
| `mini_dose_{none,low,high}_r{1-5}` | n=5/level pilot screen, §45/§46 | 15 |
| `gate_sweep_{none,low,medium,high,extreme}_r{01-20}` | Full rich-texture sweep (G3.3), §49 | 100 |
| `gate_sweep_poor_{none,low,medium,high,extreme}_r{01-20}` | Full poor-texture sweep (G3.4), §51 | 100 |
| `frame_check_r{1,2}`, `taskc_interrupt_test*`, `verify_*` | One-off instrumentation/regression checks during harness development | 7 |

## How runs were generated

Each run is one invocation of `tools/feasibility_gate/run_gate.sh
--contention <none|low|medium|high|extreme> --workload synthetic --texture
<rich|poor> --exp-id <run-id>`, against an already-running base pipeline
(PX4 SITL + Gazebo + cuVSLAM + nvblox + Fast-Planner, per the main
README). `--workload synthetic` drives `gpu_stressor.py` (PyTorch matmuls
across N concurrent CUDA streams — uncalibrated intensity presets,
characterized via `nsys` kernel-duration profiling in §47/§48, not just
`nvidia-smi` utilization, which was shown there not to separate load
levels reliably). Texture is a property of which Gazebo world PX4 was
launched against beforehand (`vio_test.sdf` = rich, texture blocks
stripped for `vio_test_poor.sdf` = poor, built in §50) — `run_gate.sh`
does not select it, only labels the run's metadata with it.

**Seeding / randomization**: rich sweep seed `20260906`, poor sweep seed
`20260907` (independent). Both sweeps: run order randomized, not blocked
by level; fresh PX4/Gazebo restart before every run; confirmed-disarm
cleanup between runs (§49's `force_disarm.py` fix). Metric is full-window
RPE-1s throughout (endpoint error and windowed drift-rate slope were tried
and rejected as worse — §46-48).

## Reproducing the analysis

Historical bags aren't in the repo (above), so this regenerates an
equivalent sweep rather than reprocessing the committed 2026-09 runs bag-
for-bag:

```
# inside isaac_ros_dev_persistent, base pipeline already up and healthy
source /opt/ros/humble/setup.bash
source /workspaces/isaac_ros-dev/ros2_ws/install/setup.bash

for level in none low medium high extreme; do
  for r in $(seq -w 1 20); do
    bash tools/feasibility_gate/run_gate.sh --contention "$level" \
      --workload synthetic --texture rich \
      --exp-id "gate_sweep_${level}_r${r}"
  done
done

python3 tools/feasibility_gate/analyze_sweep.py \
  --results-dir results/feasibility_gate --prefix gate_sweep \
  --n-boot 20000
```

Swap `--texture poor` (world `vio_test_poor.sdf`) and `--prefix
gate_sweep_poor` for the poor-texture arm. `analyze_sweep.py`'s own
docstring and `--help` cover the remaining flags (`--out-json`, bag
reading via `triage_incident.py`'s `read_bag`, level-matching by substring
against the run's `--exp-id`).
