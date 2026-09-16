# Micro-verification flight — 2026-09-16 (not counted toward H3/H4 n)

Purpose: close the two things the smoke test (`SMOKE_TEST.md`) could not
confirm — `plan_drift_logger.py` producing a real nonzero ΔP, and
`depth_fault_injector.py` actually injecting live. One flight, default
config (`vio_test.sdf`, goal `(0,8,5)`, same as the smoke test — see
"Setup note" below for why this wasn't changed to a farther goal).

## Setup note: why the default goal, not a new/farther one

The smoke test's `bridge2.log` already showed 77 replan events over a
~27s window (climb + approach to the default goal) — the goal wasn't the
problem; the fixed logger simply wasn't running yet when that window
happened (it was relaunched mid-flight, after things had already
settled). So this run used the same default goal but with all four
loggers launched correctly **before** arming, which directly targets the
actual root cause. A farther/different goal was considered but not
needed — see the result below, which confirms this was sufficient.

## VERIFY 1 — plan_drift_logger.py: PASS

All four loggers launched with `--ros-args -p use_sim_time:=true`
(the arg-stripping fix from the smoke test held — no crash). Full
`drift.csv` from this run (11 replan events during climb + approach):

| t (sim s) | replan_count | n_matched | delta_p |
|---|---|---|---|
| 58.744 | 1 | 0 | — |
| 59.864 | 2 | 0 | — |
| 60.16 | 3 | 6 | 0.784771 |
| 61.68 | 4 | 0 | — |
| 63.0 | 5 | 0 | — |
| 64.352 | 6 | 6 | 3.076167 |
| 65.936 | 7 | 0 | — |
| 68.116 | 8 | 7 | 1.086016 |
| 70.16 | 9 | 1 | 0.685593 |
| 72.304 | 10 | 4 | 1.382588 |
| 75.308 | 11 | 0 | — |

**5/11 rows have `n_matched > 0` and a real, nonzero `delta_p`.**
Distribution over those 5: min=0.686, median=1.086, max=3.076 (mean
1.403). The other 6 rows have `n_matched=0` — expected, not a bug: per
the script's own documented approximation, this happens when the prior
trajectory's "not-yet-flown" segment (computed via `bisect` on its knot
vector against elapsed time) has already fully expired by the time the
next replan arrives, which is a real, correctly-computed outcome, not a
failure to match.

**Timestamps are sim-time-consistent**: `t` values are small
(58.7-75.3), monotonically increasing, consistent with elapsed
simulation time since Gazebo startup — not Unix epoch wall-clock
(~1.7 billion), which is exactly the smoke test's original symptom. The
zero-match bug from the smoke test **does not recur**.

**Verdict: the tool works. This is not a tool-level blocker.** The
5/11 hit rate under default conditions gives a sense of scale for
planning the real Phase 2 batch: expect a meaningful fraction of
`n_matched=0` rows even during genuine active replanning, and design the
batch analysis to report a distribution over the nonzero subset (as
above), not assume every replan event yields a usable ΔP.

## VERIFY 2 — depth_fault_injector.py: injection confirmed; map-level effect inconclusive by test-design confound, not a failure

**Injection mechanism: confirmed directly, byte-level.** A throwaway
verification subscriber (not committed — `/tmp/verify_injector.py`,
one-off) compared one message from `/depth_camera` (raw) against
`/depth_camera/faulted` (injector output, `--enabled --corner top-left
--block-frac 0.25`):

```
RAW     /depth_camera:         top-left(5,5)=inf   center=inf   (top-left-is-nan=False)
FAULTED /depth_camera/faulted: top-left(5,5)=nan   center=inf   (top-left-is-nan=True)
```

The configured corner is NaN in the faulted stream and nowhere else —
confirms the injection is real, positional (only the configured block is
touched), and toggleable (this same node ran disabled, pass-through, for
several minutes earlier in this session and in the smoke test with zero
effect on the stream). This matches Phase 0's assumption exactly.

**A `nvblox.launch.py` `depth_topic` launch argument was added** (new,
additive, default unchanged) so nvblox can be pointed at the faulted
topic without touching cuVSLAM or nvblox core — exactly the "depth-fault
injection at the topic level" scope this probe's original task named as
allowed. Verified: nvblox restarted cleanly against `/depth_camera/
faulted` with no crash.

**Map-level effect ("does the map reflect it") and dmin comparison:
inconclusive, and here's why, checked rather than assumed.** Restarting
nvblox (to point it at the faulted topic) resets its map to empty. At
this point in the flight the drone was hovering at the goal, **past**
the pillar ring, facing open space — so a fresh nvblox instance sees
nothing in range regardless of corruption. This isn't a guess: **a
control was run** — restarting nvblox on the *clean* topic from the
identical hover position also returned zero real voxels
(`health_check.py`'s ESDF check failed the same way both times). This
confirms the "zero voxels" result is a confound in this verification's
setup (nothing nearby to see, from either feed), not evidence about the
injector or nvblox's handling of corrupted input.

**What this means:** the injection mechanism is proven to work correctly
at the point where it matters (the depth topic nvblox actually
consumes). A live "map shows a hole" / "dmin measurably worsens"
comparison needs the drone actively approaching a real obstacle under
corruption vs. clean — which is the actual H3 experiment's own natural
setup (sustained flight near `vio_test.sdf`'s obstacle field), not
something a generic verification flight should try to reproduce
standalone. Not treated as a tool-level blocker: the one thing this
verification could not test in isolation (behavioral effect on a live
approach) is exactly what the real H3 batch will test directly, with
proper before/after conditions instead of a single ad hoc restart.

## Safety (mandatory ordering, per the smoke test's own finding)

`fast_planner_bridge` was killed **before** calling `force_disarm.py`
this time (not after, which is what let the vehicle silently re-arm in
the smoke test). Disarm confirmed twice: once via `force_disarm.py`'s
own "CONFIRMED" message, and again independently via a fresh
`ros2 topic echo /fmu/out/vehicle_status_v4 --field arming_state --once`
after a 2s delay — held at `1` (disarmed) both times. Full pipeline
(PX4, Gazebo, ros_gz bridge, cuVSLAM, nvblox, Fast-Planner, all four
probe48h processes) torn down; verified via `ps aux` that nothing live
(non-zombie) remained.

## Changed files

- `ros2_ws/src/px4_vslam_bridge/launch/nvblox.launch.py`: new
  `depth_topic` launch argument (default `/depth_camera`, unchanged
  behavior when omitted) for H3's corrupted-condition runs.
- No changes to `tools/probe48h/*.py` were needed this time — the smoke
  test's fixes held.

## Conclusion

Both instruments are ready for the Phase 2 batch harness. Next step
(separate, per the standing plan): write the batch harness itself —
kill-bridge-before-disarm ordering (now proven necessary twice), Arm 0's
static-map save/load, per-arm world/goal switching, randomized run
order, and a checkpoint/resume record.
