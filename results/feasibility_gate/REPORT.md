# G3.3/G3.4 Gate Sweep — Provisional Report

**Status: DRAFT / PROVISIONAL — not a final decision. Goes to review before any next step is taken.**

Date: 2026-09-06
Branch: `feasibility-gate`
EKF2_EV_CTRL: 0 throughout, both arms (open-loop; this gate cannot and does
not test the original closed-loop incident's actual failure mode — see
`triage_incident.py`'s own scoping note).

## Design

Two arms, completing the gate's own criteria (G3.3 rich + G3.4 poor):

- **Rich** (G3.3): 100 runs, 5 contention levels x n=20/level, synthetic
  stressor, `g3_1_baseline_20260904_v2` scenario, original `vio_test.sdf`
  world (checker-noise albedo texture on every surface). Seed `20260906`.
- **Poor** (G3.4): same design, same scenario/trajectory/camera/initial
  state, new `vio_test_poor.sdf` world (all `<pbr><albedo_map>` texture
  blocks stripped — flat ambient/diffuse color only, everything else
  byte-identical). Independent seed `20260907`. Manipulation check
  confirms poor genuinely has fewer trackable features than rich (below).
- Both arms: run order randomized, not blocked by level; fresh PX4/Gazebo
  restart before every run; confirmed-disarm cleanup (§49 fix); 8.4MB
  trimmed recording; metric is full-window RPE-1s throughout (per
  §46-48, endpoint error and windowed drift-rate slope were tried and
  rejected as worse).
- Zero halts across either arm's 100 runs (§49's confirmed-disarm fix and
  settle-and-retry backstop held for all 200 runs total).

## STEP 1 — Manipulation check: texture-poor world

New world built by stripping every `<pbr><metal><albedo_map>...` block
from a copy of `vio_test.sdf` (25 blocks, all referencing the same
`checker_noise.png`), keeping geometry/collision/physics/lights/camera/
initial-state byte-identical. One real PX4 bug found and fixed along the
way: PX4's own readiness check polls `/world/${PX4_GZ_WORLD}/scene/info`
built from the world *filename*, not the SDF's internal `<world name>`
attribute — an earlier version of this file kept the internal name as
`vio_test` while the filename was `vio_test_poor`, causing an infinite
"Waiting for Gazebo world..." hang. Fixed by matching filename and
internal name (`vio_test_poor` throughout), same convention as the
original file.

Feature count from `/visual_slam/vis/observations_cloud`, rich (3 existing
`gate_sweep_none_r0{1,2,3}` runs) vs. one fresh poor verification flight:

| | mean | median | min | max |
|---|---|---|---|---|
| Rich | 262.6 | ~229 | 13 | 429 |
| Poor | 105.8 | 85.0 | 0 | 407 |

**Manipulation check PASSED**: poor/rich ratio 0.40x. Poor texture's own
minimum hit 0 (a real tracking-loss event) where rich's worst case never
dropped below 13.

## STEP 2/3 — Full sweep + rich-vs-poor comparison

Per-level RPE-1s (n=20/level, full-window, per-run median):

| Level | Rich mean | Rich dropout | Poor mean | Poor dropout |
|---|---|---|---|---|
| none | 0.150 | 1/20 (5%) | 0.373 | 18/20 (90%) |
| low | 0.159 | 3/20 (15%) | 0.310 | 17/20 (85%) |
| medium | 0.160 | 3/20 (15%) | 0.272 | 16/20 (80%) |
| high | 0.136 | 4/20 (20%) | 0.291 | 15/20 (75%) |
| extreme | 0.142 | 1/20 (5%) | 0.344 | 17/20 (85%) |
| **overall** | **0.149** | **12/100 (12%)** | **0.318** | **83/100 (83%)** |

**Texture main effect is large and unambiguous**: poor-texture RPE-1s is
~2.1x rich's, and poor-texture tracking-loss rate (83%) is ~7x rich's
(12%). This confirms G3.4's premise — texture availability dominates VIO
robustness in this world far more than anything tested in G3.3 — but it
is a texture effect, not a contention effect.

**Contention trend, poor texture (all 100 runs)**: slope=-0.0078/level,
r=-0.082, **p=0.415**, bootstrap 95% CI on slope (-0.027, 0.011) — crosses
zero. Cohen's d (none→extreme) = **-0.199** (small, wrong direction). Same
null conclusion as rich (p=0.663 there), just noisier.

**Clean-tracking-only subset, poor texture**: because dropout is the
*dominant* outcome at this texture level (75-90% per level), the
clean-tracking subset is tiny and its per-level size is itself
lopsided — none=2, low=3, medium=4, high=5, extreme=3 (n=17 total). This
subset's OLS trend: slope=-0.0508/level, p=0.107, and its bootstrap CI
(-0.090, -0.011) does *not* cross zero — but this result is **flagged as
unreliable, not reported as a finding**: bootstrap resampling from a
per-level n as low as 2 has too few distinct resample outcomes to
approximate real sampling variability, and a subset this small (2-5
runs/level) is highly vulnerable to single-point leverage. Even taken at
face value, the direction (higher contention → *lower* RPE) is backwards
from the hypothesis under test and more plausibly reflects survivorship
in an already-marginal tracking regime than a real accuracy effect.

**Timing pathway, poor texture**: dropout-event rate by level is 90% /
85% / 80% / 75% / 85% for none→extreme — not monotonic, and actually
*lowest* at `high`, the second-heaviest contention level. Same
non-monotonic-null pattern as rich (5/15/15/20/5%). No timing-pathway
dose-response in either arm.

## Rich-vs-poor interaction: **null in both, texture dominates**

Does contention show an effect in poor that was absent in rich? **No.**
Both arms return a non-significant, near-zero-or-negative full-sample
trend (p=0.663 rich, p=0.415 poor); neither shows a reliable
accuracy-pathway or timing-pathway dose-response. The one nominally
"significant" result (poor's clean-tracking-only subset) is built on a
sample too small and too structurally biased (survivor selection under
83% dropout) to trust, and even it points the wrong direction. The real,
large, well-powered effect in this dataset is texture itself, which was
never the variable under test in the original GPU-contention hypothesis.

## What this does and doesn't rule out

Does not rule out:
- A different, more targeted GPU workload (`--workload nvblox`, raising
  `integrate_depth_rate_hz` — not exercised in either sweep, both of
  which used the synthetic PyTorch stressor only).
- An effect only observable closed-loop (`EKF2_EV_CTRL` != 0) — both
  arms are open-loop by hard constraint, so neither can reproduce the
  original ~55m incident's actual runaway mechanism (EKF accepting a
  high-innovation measurement), only raw cuVSLAM tracking error in
  isolation.

## Recommendation (provisional)

Leaning NEGATIVE for "GPU contention measurably degrades cuVSLAM tracking
accuracy via a graded dose-response," now tested across both texture
conditions the gate specified (G3.3 rich, G3.4 poor) — completing the
gate's own criteria. This is a draft finding for review, not a closed
decision. The review should weigh whether to (a) close the feasibility
gate as negative for the contention hypothesis as tested (synthetic
stressor, RPE-1s, open-loop, this scenario, both textures), (b) test the
`--workload nvblox` arm before concluding, (c) treat the open-loop scope
itself as too narrow to answer the original motivating question and
reconsider next steps for Milestone 2 accordingly, or (d) separately note
the texture-robustness finding (unrelated to contention, but large and
real) as worth its own follow-up regardless of how the contention
question is closed.
