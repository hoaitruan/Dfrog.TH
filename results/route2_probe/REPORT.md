# Route 2 Closed-Loop Feasibility Probe — Provisional Report

**Status: DRAFT / PROVISIONAL — not a final decision. Goes to review before any next step is taken.**

Date: 2026-09-06/07 (R2-0/R2-1), 2026-09-07 (R2-2, decisive)
Branch: `r2-closedloop-probe` (never merged to `feasibility-gate` or `main`)
SIM ONLY. First time `EKF2_EV_CTRL` was enabled in this project (value
11 = horizontal position + vertical position + yaw, no velocity —
verified against this PX4 checkout's own
`src/modules/ekf2/params_external_vision.yaml` bit definitions and
against this project's own Phase 4 history, README.md: "EKF2_EV_CTRL 11
(position + yaw, no velocity — cuVSLAM's twist field is unpopulated)").
Confined entirely to a new airframe (`4900_gz_x500_depth_stereo_r2closedloop`)
selected only via an explicit `PX4_SYS_AUTOSTART=4900` override — the
shared `4022_gz_x500_depth_stereo` airframe used by every other
experiment in this repo is untouched at `EKF2_EV_CTRL=0`.

## Setup

- **World:** new `vio_test_r2_obstaclefree.sdf` — all 24 physical
  obstacle models (16 markers + 8 pillars) stripped from `vio_test.sdf`,
  keeping the textured ground plane and everything else (physics,
  lights, camera, initial state) identical. Deliberate, disclosed
  safety-over-tracking-fidelity trade-off: an obstacle-free world means a
  genuine runaway flies into open space with nothing to hit.
- **Safety harness (`r2_killswitch.py`):** extends `hard_killswitch.py`'s
  proven, incident-tested kill-bridge-then-disarm design unchanged (same
  thresholds: altitude 8m/-0.5m, horizontal 15m, descent rate 1.5m/s)
  with one new trigger — horizontal divergence between the EKF's fused
  estimate (NED) and Gazebo ground truth (ENU), threshold 10m. The
  NED↔ENU conversion was validated live before being trusted: residual
  stayed ~0.03–0.06m flat across a real flight with 3.6m lateral / 4.7m
  vertical displacement (would have grown with displacement had the axes
  been swapped or sign-flipped).
- **Recording:** estimated pose, ground truth, the 3 EV aid-source
  topics (innovation/covariance/accept-reject — newly added to PX4's
  uXRCE-DDS publication whitelist; they were not bridged to ROS2 at all
  before this probe, closed-loop or not), cuVSLAM status/features, gpu
  sampler with temperature, and a `runaway_event.json` breach record
  when the watchdog fires.
- **Confirmed-disarm cleanup** (§49's fix) used throughout; a
  resumable, seeded, randomized batch orchestrator with the same
  settle-and-retry robustness check validated for the G3.3/G3.4 sweeps.

Two infrastructure bugs found and fixed along the way (both pre-existing
latent issues, not introduced by this probe): (1) `4022_gz_x500_depth_stereo`
was never actually registered in PX4's ROMFS CMakeLists — it only worked
because of a stale, un-reproducible build artifact, discovered when a
clean rebuild (needed to add the new `4900` airframe) silently dropped
it; fixed by registering both airframes properly. (2) PX4's own
`px4-rc.gzsim` readiness check polls a gz service built from the world
**filename**, not the SDF's internal `<world name>` attribute — an
earlier version of the obstacle-free world kept those mismatched,
causing an infinite "Waiting for Gazebo world..." hang.

## STAGE R2-0 — Closed-loop baseline, no contention (8 runs) — EARLY-STOP GATE

| | value |
|---|---|
| mean divergence | 0.0296m ± 0.0216m (range 0.0114–0.0719m) |
| max divergence | 0.0350m ± 0.0204m (range 0.0180–0.0750m) |
| runaway rate | 0/8 (0%) |
| ev_pos / ev_hgt fused | 96.4% / 99.6% |
| ev_yaw fused | 0.0% (all 8 runs) |

**Result: STABLE.** Divergence is tiny and nowhere close to the 10m
abort threshold; zero runaways. Per the gate's own logic, this is not
"unstable at none," so the probe proceeded to R2-1 rather than stopping.

**Note on yaw fusion (0% throughout):** not a bug — `visual_odometry_bridge.py`'s
own docstring documents that cuVSLAM's odometry is republished as
`POSE_FRAME_FRD` (relative, arbitrary yaw origin), not `POSE_FRAME_NED`
(absolute, North-referenced). Position and height fusion (the primary
spatial estimate) worked well throughout (96–100% fused); yaw simply
never won PX4's own innovation gate under this FRD framing across all
128 runs (R2-0+R2-1 combined). This is a real, consistent, reproducible
characteristic of this specific bridge/EKF2 configuration, reported as a
finding — not tuned or investigated further here (out of scope for a
bounded feasibility probe).

## STAGE R2-1 — Contention probe, powered (none/medium/high, n=40/level, 120 runs)

Randomized order (seed `20260906`), not blocked by level. Two real
robustness-check halts during this stage (both root-caused and fixed,
batch resumed from the same manifest, no data lost or invalidated):

1. `run_r2_probe.sh`'s own precondition check (cuVSLAM odometry topic
   present) had no retry and failed twice at the identical manifest
   position — run.sh's health check verifies generic `/fmu/*` topics but
   never specifically waits on this one, so it can print PASS while
   cuVSLAM is still mid-initialization. Fixed with a 20×1s retry.
2. `fast_planner_trigger` survived its own 15s `timeout` once (empty
   log, evidently hung) — the foreground `timeout ... || true` pattern
   only reaches the `ros2 run` wrapper, not the real grandchild
   entry-point, which got reparented to PID 1. Fixed with `setsid` +
   tracked PID + `kill_group()`, matching the rest of the script.

A large zombie-process backlog (this container's PID 1 does not reap
children — a pre-existing, already-documented limitation) also required
one full container restart mid-batch (all data survived — `/workspaces`
and `/tmp` are both bind-mounted/volume-backed).

| Level | mean divergence | max divergence | runaway rate | ev_pos fused | ev_yaw fused |
|---|---|---|---|---|---|
| none | 0.0306m ± 0.0175m | 0.0346m ± 0.0180m | 0/40 (0%) | 97.2% | 0.0% |
| medium | 0.0236m ± 0.0134m | 0.0283m ± 0.0150m | 1/40 (2.5%) | 97.9% | 0.0% |
| high | 0.0285m ± 0.0147m | 0.0329m ± 0.0145m | 0/40 (0%) | 97.6% | 0.0% |

**Trend test (mean divergence ~ level, OLS):** slope = -0.00107/level,
r = -0.057, **p = 0.540**, bootstrap 95% CI on slope (-0.0046, 0.0024) —
crosses zero, no trend. **Cohen's d (none→high) = -0.132** (negligible).

**Runaway rate:** 0/40, 1/40, 0/40 for none/medium/high — a single
isolated event, at medium (not the highest-contention level), so no
monotonic or even directional pattern. The one runaway
(`medium_r106`) was an `altitude_low` breach at -0.52m (just past the
-0.5m ground-punch-through floor) — consistent with a routine landing/
settling overshoot, not necessarily an EKF divergence event; `divergence`-
kind breaches (the mechanism specific to this probe) never fired once
across all 128 runs (R2-0+R2-1).

## STAGE R2-2 — the decisive 55m-regime reproduction attempt (60 runs)

Ran to completion (the wall-clock gap disclosed above was closed in a
follow-up session). Full coupled stack: cuVSLAM→EKF→control **and**
nvblox ESDF→`reactive_esdf_avoidance`→control (Phase 6's *actual*
original-incident node — self-contained arm/climb/potential-field
avoidance, not Fast-Planner's separate kinodynamic planner used
everywhere else in this project), against the **obstacle world**
(`vio_test.sdf`, real markers/pillars — not R2-0/R2-1's obstacle-free
variant). `reactive_esdf_avoidance`'s own goal, `(7,0,5)` ENU, is
deliberately set past `pillar_01` so a straight line is blocked — this
*is* the original cruise-near-obstacle trajectory. Contention is **real
nvblox load** (`integrate_depth_rate_hz`), not a synthetic stand-in:
L0=40Hz (light), L1=640Hz (heavy), L2=640Hz + synthetic extreme
stressor on top (worst-case combined). n=20/level, randomized order
(seed `20260907`), 60 runs, zero halts (one container restart for
zombie-process buildup mid-batch, same pre-existing limitation as
R2-1 — all data survived).

Extended recording/analysis for this stage: EV aid-source message age
(`timestamp - timestamp_sample`, the timing-mechanism signal) and a
post-hoc min-obstacle-clearance/collision flag, computed from recorded
ground truth against `vio_test.sdf`'s known, fixed obstacle geometry (24
markers/pillars, exact positions and box/cylinder dimensions extracted
from the world file) — no live collision sensor needed, and
`reactive_esdf_avoidance.py` itself was never touched or queried.

| Condition | mean divergence | large-div (>1m) rate | runaway rate | collision rate | ev_pos msg age | ev_pos rejected% |
|---|---|---|---|---|---|---|
| L0 (light) | 0.0243m ± 0.0167m | 0/20 (0%) | 2/20 (10%) | 2/20 (10%) | 157.1ms | 0.17% |
| L1 (heavy) | 0.0315m ± 0.0182m | 0/20 (0%) | 4/20 (20%) | 1/20 (5%) | 155.8ms | 0.16% |
| L2 (heavy+synthetic) | 0.0314m ± 0.0141m | 0/20 (0%) | 4/20 (20%) | 0/20 (0%) | 157.6ms | 0.03% |

**Trend tests (OLS, condition coded L0=0/L1=1/L2=2, n=55-60):**
mean divergence slope=0.00337/level, r=0.163, **p=0.235**, bootstrap 95%
CI on slope (-0.00129, 0.00813) — crosses zero. EV message age
slope=0.211ms/level, **p=0.593** — flat (no timing degradation with
nvblox load). EV rejected% slope=-0.071%/level, **p=0.449** — flat, if
anything slightly negative.

**Zero large-divergence (>1m) reproductions at any level, in 60 runs —
the 55m regime did not reproduce even once.** All 10 runaway events were
`descent_rate`-kind killswitch breaches (mostly marginal, 1.50-1.55m/s
against the 1.5m/s threshold; one at 3.77m/s), **zero were
`divergence`-kind** — the mechanism this probe exists to detect never
fired. The two physical collisions (post-hoc geometric clearance = 0)
both occurred at **L0** (the lightest-contention condition), and neither
one triggered the killswitch at all (no accompanying geofence/divergence
anomaly) — both are consistent with `reactive_esdf_avoidance`'s own
documented limitation ("No planning ahead — reactive only": a pure
potential-field controller can graze an obstacle without any
estimator-visible flight-dynamics anomaly), not an EKF/estimator event,
and the direction (collisions decreasing, not increasing, with load)
runs opposite to a contention-driven hypothesis.

## DECISION R2

**NO-GO (robust).** Across all three closed-loop stages (200 total
runs: 8 baseline + 120 synthetic-stressor-powered + 60 real-nvblox-load
powered against the actual obstacle world and the actual original
control node), no meaningful divergence, no contention-linked trend in
divergence/timing/rejection-rate, and zero `divergence`-kind killswitch
triggers anywhere. R2-2 specifically closes the gap R2-0/R2-1 left open:
real nvblox contention (not synthetic), the actual incident's control
node (`reactive_esdf_avoidance`, not Fast-Planner), and the actual
obstacle-laden cruise trajectory all fail to reproduce anything
resembling the original 55m divergence, at any tested load level up to
and including the worst-case combined condition (L2). Combined with the
open-loop gate's own negative finding (§45-51: no contention-linked
cuVSLAM tracking-accuracy effect either) and R2-0/R2-1's negative
finding under a synthetic stressor, **Route 2 is closed: the original
~55m incident was not reproducible under any contention mechanism this
project tested (synthetic GPU load or real nvblox load), and is best
explained as a one-off, a mundane bug (e.g. a specific bad cuVSLAM
tracking excursion combined with the pre-fix control loop that has since
been changed), or a condition outside the range this stack was probed
across (a longer/more demanding trajectory, sustained load over a much
longer duration, or a hardware-specific timing pattern not reproducible
in this SITL setup).**

This does not mean GPU contention is irrelevant to this stack in every
conceivable configuration — only that the specific, repeated,
increasingly faithful attempts made across this whole engagement (G3.3
open-loop rich-texture, G3.4 open-loop texture-poor, R2-0/R2-1
closed-loop synthetic-stressor, R2-2 closed-loop real-nvblox-load) never
found it.

## Recommendation

1. Close Route 2. The evidence across 200 closed-loop runs plus the
   open-loop gate's own 200 runs (§49/§51) does not support a
   contention-gated divergence/runaway mechanism in this stack.
2. The two L0 collision events and the `descent_rate` breach pattern
   (present at all three levels, not contention-scaled) point to a
   separate, real limitation worth a note for any future closed-loop
   navigation work in this project: `reactive_esdf_avoidance`'s pure
   potential-field approach can graze obstacles or produce marginal
   descent-rate transients on its own, independent of GPU load — a
   navigation-algorithm finding, not an estimator one.
3. `EKF2_EV_CTRL` reverted to 0 in the default working config,
   confirmed live (no data flow on any EV aid-source topic under the
   unmodified `4022` airframe with no env overrides) — the invariant
   holds for every other experiment in this repo. The EV-on
   configuration exists only on `r2-closedloop-probe`, as the
   experiment record.

This is a draft finding for review, not a closed decision.
