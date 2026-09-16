# Smoke test — 2026-09-16 (not part of the H3/H4 run count)

One manual flight, default config (`vio_test.sdf`, goal `(0,8,5)`), to
catch bugs in `tools/probe48h/*.py` before committing to the real ~30-run
Phase 2 batch. Findings below; **no H3/H4 data was collected here** — this
run doesn't count toward Arm 0/1/2 or clean/corrupted n.

## Bugs found and fixed

1. **`fast_planner_bridge` needs `OFFBOARD_AUTO_START=1`** to arm
   non-interactively (otherwise it blocks on an interactive ENTER press
   and silently never arms). Not a bug in our scripts — an operational
   requirement already established by `tools/feasibility_gate/run_gate.sh`
   (`setsid env OFFBOARD_AUTO_START=1 ... < /dev/null &`), just missed on
   the first launch attempt here. The Phase 2 batch harness must use the
   same invocation.
2. **All four `tools/probe48h/*.py` scripts crash on `--ros-args -p
   use_sim_time:=true`** — none of them stripped ROS-specific CLI args
   before their own `argparse`, a standard rclpy Python gotcha. Fixed in
   all four: `args = ap.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])`.
   `plan_drift_logger.py` was also missing `import sys` outright (caught
   by the same fix attempt, unrelated typo).
3. **This one mattered for correctness, not just crashing:**
   `plan_drift_logger.py` compares `msg.start_time` (sim time, since
   `fast_planner_node`/`traj_server` both run with `use_sim_time: true`,
   `fast_planner_px4.launch.py:47,62`) against its own
   `self.get_clock().now()`. Without `use_sim_time:=true` on the logger
   itself, that's wall-clock epoch time minus a small sim-time value —
   a huge, meaningless "elapsed," which made `n_matched` silently 0 on
   every single row (77 rows, all zero, in the first attempt). This is
   exactly the class of bug this project's own history warns about
   (`ground_truth_tf`, `nvblox_node`'s prior `use_sim_time` incidents) —
   fixed by passing `--ros-args -p use_sim_time:=true` at invocation
   (config-only, no further code change beyond fix #2 above, which is
   what let the flag actually reach rclpy instead of crashing argparse).

## What's confirmed working, with real numbers

- **`dmin_logger.py`**: 42,346 rows over the full flight. Sane values
  throughout — `2.207m` at spawn, `2.437m` at the goal near `(0,8,5)`,
  consistent with the known pillar-ring geometry. No crashes.
- **`churn_logger.py`**: real, non-fabricated churn values while the
  drone was grounded and idle (`~0.001-0.12`, small, as expected for a
  mostly-static local map) and while flying (`~0.02-0.07`, both sampled
  above). Voxel-matching by rounded center across polls works — over
  16,000 matched voxels per tick within the r=2m sphere.
- **`fast_planner_bridge`**, once launched correctly: armed, climbed,
  followed Fast-Planner to the goal, held stably. 77 replan events
  observed while active (climb + converge on goal).

## Not fully closed — flagged, not hidden

- **`plan_drift_logger.py`'s fix was applied AFTER the flight had already
  settled at the goal** (Fast-Planner stops publishing `/planning/bspline`
  once stable and no watchdog condition is triggered — confirmed in code,
  `kino_replan_fsm.cpp:635`, the hold-watchdog only fires on real tracking
  divergence, not periodically). So `n_matched`/`delta_p` were never
  empirically observed as nonzero post-fix in this session, only
  pre-fix (where they were incorrectly always 0/empty, now understood
  and fixed). The fix itself is a standard, well-understood ROS 2
  mechanism (`use_sim_time` making `get_clock().now()` track `/clock`),
  not a novel or risky change — but per this probe's own rules, an
  unverified fix shouldn't be called "done" outright. **Action: watch
  this specifically on Phase 2's first real run** (which will have
  sustained active replanning by construction, since Arm 1/2 and the
  H3 corrupted condition are designed to keep perturbing the plan).
- **`depth_fault_injector.py` was not exercised live** in this smoke
  test (not needed to validate H4's tooling; H3's corrupted-condition
  runs will be its first live test).
- **Found during cleanup, not the scripts themselves: a real safety
  race.** Calling `force_disarm.py` while `fast_planner_bridge` is still
  running does NOT hold — the bridge re-requests ARM on every 20Hz tick
  until it believes it's armed (documented in this project's own
  `flight_test_log.html` §`e-killpath`, the original 26m-runaway root
  cause). First disarm attempt here was silently undone by the bridge
  within about a second; caught by re-checking arming state after (not
  just trusting `force_disarm.py`'s own "CONFIRMED" message), fixed by
  killing `fast_planner_bridge` first, then disarming, which held.
  **The Phase 2 batch harness must kill the bridge before disarming, in
  that order** — this is exactly what `run_gate.sh`/`run_r2_probe.sh`
  already do; a from-scratch batch script for this probe needs the same
  ordering, not a fresh mistake to repeat.

## Cleanup

Vehicle confirmed disarmed and held (re-checked after bridge kill, not
just trusted the first report). Full pipeline (PX4, Gazebo, ros_gz bridge,
cuVSLAM, nvblox, Fast-Planner, all three probe48h loggers) torn down.
Container left running for Phase 2. No H3/H4 data files were kept from
this run (smoke-test CSVs stayed under `/tmp/smoke_csv/` inside the
container, not copied into the repo — they're not real probe data).
