# Route 2 closed-loop probe result data

Result data for the closed-loop GPU-contention probe (branch
`r2-closedloop-probe`; harness in `tools/route2_probe/`). First and only
place in this project where `EKF2_EV_CTRL` is nonzero — confined to a new,
isolated airframe (`4900`) on this branch; the shared `4022` airframe every
other experiment uses stayed at `0` throughout. Findings are written up in
`REPORT.md` (provisional draft) and, as the durable copy, in
`flight_test_log.html` §52-53 and `docs/research_log.md` §1.5.

Brought into version control 2026-09-11, same convention as
`results/feasibility_gate/README.md`: rosbags stay local (see below); every
other per-run file is committed so the study is inspectable/reproducible
from the repo, not only from summary numbers in the log.

## What's committed vs. what stays local

**Committed** (~58MB, 2,073 files): per run, everything except the
rosbag — `bag_record.log`, `fast_planner_bridge.log`,
`fast_planner_trigger.log`, `force_disarm_cleanup.log`, `gpu_log.csv`,
`gpu_sampler.log`, `gpu_stressor.log`, `r2_killswitch.log`,
`visual_odometry_bridge.log`, `vslam_compare.csv`,
`vslam_compare_node.log`. Plus the top-level `REPORT.md`.

**Not committed, stays local-only**: each run's `rosbag/` subdirectory. No
individual file here actually exceeds 50MB (unlike `results/
feasibility_gate/`, whose largest bags run to 881MB) — these runs use the
same 8.4MB-trimmed recording convention established in §48. They're kept
out anyway for consistency with `results/feasibility_gate/` and because
**`analyze_r2_2.py` reads each run's rosbag directly** (via
`triage_incident.py`'s `read_bag`, same as `analyze_sweep.py`), **not**
`vslam_compare.csv` — so re-deriving the exact published statistics from
history still requires those local bags, same caveat as
`results/feasibility_gate/README.md`.

A Git LFS setup was tried and reverted for this project's simulation data
generally (2026-09-11): at measured upload speed, the combined rosbag
volume across this directory + `results/feasibility_gate/` +
`ros2_ws/rosbags/` (~14GB) would take ~10 hours to push and exceed
GitHub's free 1GB/month LFS bandwidth quota. Not pursued.

## Run directories

| Prefix | Stage | Purpose | Runs |
|---|---|---|---|
| `r2_none_r{01-08}` (of the 48 `r2_none_*`) | R2-0 | Closed-loop, no contention, obstacle-free | 8 |
| `r2_{none,medium,high}_r{...}` (remaining 40/level) | R2-1 | Synthetic-stressor contention probe, obstacle-free, powered (n=40/level) | 120 |
| `r2_2_L{0,1,2}_r{01-20}` | R2-2 | Real nvblox contention + real obstacle world + original incident's own control node (`reactive_esdf_avoidance`) | 60 |
| `r2_manual_test_1`, `r2_2_manual_test_1` | — | One-off instrumentation checks during harness development | 2 |

R2-0 and R2-1 share the `r2_none`/`r2_medium`/`r2_high` naming — R2-0 is
the `none`-level runs recorded before R2-1's own powered `none` arm was
added; both are included in the "48 `r2_none_*`" count and in the "128
closed-loop runs" figure quoted in the log and research log.

## How runs were generated

`tools/route2_probe/run_r2_probe.sh` (R2-0/R2-1) and
`tools/route2_probe/run_r2_2_probe.sh` (R2-2, a sibling script, not a
modification — R2-1's script is left as the record of those stages).
Airframe `4900` (`EKF2_EV_CTRL=11`, position + yaw, no velocity) assumed
already running, `visual_odometry_bridge` + `r2_killswitch.py` +
confirmed-disarm cleanup layered on top, same EV aid-source topic
recording throughout.

- **R2-0/R2-1**: obstacle-free world, synthetic PyTorch stressor
  (`none`/`medium`/`high`), Fast-Planner-driven climb-and-hover.
- **R2-2**: real obstacle world (`vio_test.sdf`, rich texture, real
  markers/pillars), `reactive_esdf_avoidance.py` — Phase 6's actual
  original-incident control node, not Fast-Planner — with its own goal
  set past `pillar_01` so a straight line is blocked (the original
  "cruise near obstacles" trajectory). Contention is **real nvblox load**
  via `integrate_depth_rate_hz`, not synthetic: L0=40Hz (light),
  L1=640Hz (heavy), L2=640Hz + synthetic extreme stressor layered on top
  (worst case). Seed `20260907`, randomized order, n=20/level.

## Reproducing the analysis

Historical bags aren't in the repo (above), so this regenerates an
equivalent probe rather than reprocessing the committed 2026-09 runs
bag-for-bag. Requires airframe `4900` set up per
`flight_test_log.html` §52 (`EKF2_EV_CTRL=11`, isolated from the shared
`4022` airframe) and the base pipeline healthy:

```
source /opt/ros/humble/setup.bash
source /workspaces/isaac_ros-dev/ros2_ws/install/setup.bash

for level in L0 L1 L2; do
  for r in $(seq -w 1 20); do
    bash tools/route2_probe/run_r2_2_probe.sh --level "$level" \
      --exp-id "r2_2_${level}_r${r}"
  done
done

python3 tools/route2_probe/analyze_r2_2.py \
  --results-dir results/route2_probe --prefix r2_2
```

`analyze_r2_2.py`'s own docstring/`--help` covers the remaining flags. See
`tools/route2_probe/run_r2_probe.sh` for the R2-0/R2-1 invocation
(`--contention none|medium|high`, obstacle-free world).
