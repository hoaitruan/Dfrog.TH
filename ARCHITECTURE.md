# Architecture Decision: Path A (Isaac-centric)

## Decision

This project commits to **Path A**: cuVSLAM is the pose source, nvblox's ESDF
is the obstacle map, and the planner consumes nvblox's ESDF. This directly
addresses the two shortcuts a mission audit found in place:

1. Control flew on GPS, not cuVSLAM (`EKF2_EV_CTRL 0`).
2. The flying planner (Fast-Planner) built its own depth-based SDF map and
   never read nvblox's ESDF (`grep -rln nvblox fast_planner/` returned zero
   hits) — nvblox ran in parallel, producing data nobody consumed.

Path A rejects both shortcuts as the end state. It is reached in two
milestones, run separately and never collapsed into one change.

## Milestone 1 — the map half (in progress)

Rewire Fast-Planner to consume nvblox's ESDF instead of building its own
from raw depth. Fix the known foot-guns (absolute climb target, missing
`use_sim_time`, unenforced `-u admin`). Add real orchestration and a health
check. Prove obstacle avoidance end-to-end with a rosbag-verified flight.

**Pose stays on GPS (`EKF2_EV_CTRL` stays `0`) for the entire duration of
this milestone. This is deliberate, not a leftover shortcut being kept
around out of convenience.** The map/planner rewiring is a large, real
architecture change; isolating it from the pose source means a failure
during verification has exactly one possible cause (the new map wiring),
not two entangled ones (map wiring + pose source, either of which could be
responsible). GPS is the known-stable pose source from every prior test in
this project — using it here is what makes Milestone 1's evidence trustworthy
in isolation.

## Milestone 2 — the pose half (not started, not in scope here)

Re-enable `EKF2_EV_CTRL`, root-cause the cuVSLAM 55m GPU-contention drift
under concurrent nvblox load (previously only worked around, never fixed —
see the audit), and validate vision-only flight. This is the harder problem
and is partly a hardware/GPU-scheduling concern that may not even fully
resolve in this sim environment (untested whether Jetson's GPU contention
behaves the same as this laptop's).

Any change to `EKF2_EV_CTRL`, GPU scheduling/priority, or CUDA MPS
configuration belongs to Milestone 2, not Milestone 1. If a Milestone-1 task
appears to require touching those, that's a signal to stop and re-scope,
not to proceed.

## Definition of done, per milestone

- **Milestone 1 is done** when a supervised flight, flown on GPS pose,
  produces a rosbag + RViz screenshot showing: the nvblox ESDF obstacle
  display populated, the planner's trajectory markers, and the
  ground-truth flight path visibly curving around a known obstacle — with
  `grep -rln nvblox fast_planner/` returning real hits and the old
  depth-based buildup path inactive.
- **Milestone 2 is done** when the same flight succeeds with
  `EKF2_EV_CTRL` nonzero and cuVSLAM actually driving control, without the
  GPU-contention drift recurring.

Only when both are done does the original mission statement hold without
caveats.
