# H4 existence-gate verdict — 2026-09-18

Small-n existence probe (n=4, Arm L only — see Arm S blocker below), not
the batch. Goal `(9,9,5)` in `vio_test.sdf` (obstacle world), noise now on
(`docs/probe48h/GATE12.md` B1), true physics ground truth (`GATE12.md`
A2/B2). Raw CSVs for all runs kept under `results/probe48h/` in this
repo (see Files section).

## Arm S (frozen map): BLOCKED, not completed — reported, not hidden

Documented in full in `GATE12.md`. After fixing two real, verified
problems (the dead `map_clearing_radius_m` interaction, then confirming
via direct service queries that a single flythrough doesn't give TSDF
voxels along the corridor enough weight to register as "observed" —
real data existed only in a small bubble around the final hover point),
Arm S could not be completed as specified within this gate's scope. This
is a **genuine blocker for Arm S's exact recipe**, not a tuning failure,
and not something a fourth unvalidated workaround should paper over.

**Consequence: the Q1 cross-check ("Arm S churn must be ≈0, else the
freeze recipe is broken") and Q2 (Arm L ΔP vs. Arm S ΔP) cannot be
answered from this gate.** Q1's primary question does not need Arm S and
is answered below.

## Q1: Is churn(t) meaningfully > 0 in Arm L? — **YES**

Pooled across all 4 runs, split into "active" (before the flight settles)
and "full" (including the settled hover tail) — the split matters and is
explained below, not glossed over:

| | n rows | mean | median | max | % rows > 0 |
|---|---|---|---|---|---|
| **Active** (in flight) | 82 | 0.0214 | 0.0117 | 0.2113 | **80/82 (98%)** |
| **Full** (incl. settled hover) | 202 | 0.0101 | 0.0000 | 0.2113 | 93/202 (46%) |

**Why the split, and why it's not cherry-picking:** in every run, churn
was real and clearly nonzero while the drone was actively flying, then
dropped to **exactly** `0.000000` for many consecutive polls once it
settled into a stable hover at the goal (visible directly in each run's
`churn.csv`, e.g. L1 t=58.2-97.0). This is consistent with TSDF's
weighted-average integration: once a voxel's accumulated weight is high
(sustained observation from one stable viewpoint), each additional noisy
sample moves the running average less and less, converging toward a
value that stops visibly changing — not a measurement bug, a real
property of how the noise interacts with weighted averaging. The
practical implication: **meaningful churn requires ongoing fresh
observation of a region**, which is exactly the condition during active
flight and exactly why H4's premise (moving through a scene with a noisy
sensor produces map churn) is supported, not undermined, by the settled
tail going to zero.

**Per-run detail** (active-window only):

| Run | n | mean | median | max |
|---|---|---|---|---|
| L1 | 17 | 0.0207 | 0.0097 | 0.1189 |
| L2 | 27 | 0.0303 | 0.0157 | 0.1531 |
| L3 | 20 | 0.0232 | 0.0135 | 0.1243 |
| L4 | 18 | 0.0066 | 0.0031 | 0.0257 |

Consistent across all 4 runs — no run shows churn ≈0 throughout its
active window. **This is a well-supported positive answer to Q1**, not a
marginal call: 98% of active-window samples are nonzero, across every
run independently.

## Q2: Does Arm L's ΔP exceed Arm S's approach-driven floor? — **CANNOT BE ANSWERED**

Arm S is blocked (above), so there is no baseline to compare against.
Reporting Arm L's own ΔP distribution here as a reference for whenever
Arm S is completed, **not** as a substitute for the actual comparison:

- Pooled across 4 runs, 49 nonzero-match replan events: **median 1.348,
  IQR 0.805 (Q1=0.804, Q3=1.609), mean 1.340, range 0.307-3.103.**
- `n_matched` was 0 on a substantial fraction of replan events in every
  run (matches the micro-verify finding — not new, not a defect).

**What this does and doesn't show:** it shows real, replan-driven control-
point displacement of order ~0.3-3.1m happens under live, noisy nvblox.
It does **not** show that this is *caused by* churn specifically, versus
being the ordinary ΔP any approach-to-a-goal replan sequence would show
even with a perfectly static, noise-free map (that's exactly what Arm S
was supposed to isolate). Without Arm S, this number is a data point,
not evidence either way for the churn→ΔP causal link Q2 is actually
asking about.

## A safety-relevant finding outside H3/H4's scope, reported because it's too significant to omit

**2 of the first 5 total Arm L flight attempts (both discarded, not
counted in the n=4 above) diverged into large, uncommanded excursions**
(one to NED `(-8,15,...)` with a wrong internal target; one to NED
`(22,170,...)`, similarly wrong target) instead of flying the intended
path. Both were caught and safely disarmed (confirmed independently both
times, per this gate's mandatory ordering). One was traced to a stale
`/dev/shm/fastrtps_*` DDS lock file from a prior `kill -9` (fixed —
`run_probe.sh`'s teardown now cleans these up); after that fix, one
**more** excursion still occurred on the very next attempt, with no DDS
error present, then the retry after that succeeded normally.

**Not claimed as caused by the newly-added depth noise** — n=2 divergent
events is not evidence of a rate or a mechanism, and no noise-off control
was run in this exact goal/config to compare against (that comparison
was out of this gate's scope). But it's a real, repeated, uncommanded
large-excursion pattern (~2/7 total attempts, both non-DDS-explained
after the fix) that occurred exclusively in this session's runs — every
one of which used the newly-noisy depth sensor — and it is flagged here
explicitly rather than silently retried past. **If Direction E work
continues, this pattern deserves its own targeted investigation before
being dismissed as unrelated to the noise patch.**

## Verdict

**Not H4-DEAD** — Q1 is unambiguously positive; churn from sensor noise
is real and consistently observed during active flight across all 4
runs, at magnitudes (median ~0.01-0.03m per active tick) that are not
trivial relative to the depth noise stddev itself (0.02m).

**Not H4-ALIVE either** — Q2, the question that actually isolates
churn-driven plan instability from ordinary approach-driven replanning,
could not be answered because Arm S is blocked.

**Verdict: BORDERLINE — specifically, blocked on completing Arm S, not
on sample size.** The task's own three-way framing (ALIVE/DEAD/BORDERLINE-
needs-larger-n) doesn't have a clean slot for "the control arm's method
doesn't work yet" — this is that case, stated plainly. Recommended next
step, if this direction continues: solve Arm S properly (a supported
option, not attempted further here per "don't improvise a fourth
workaround" — e.g. deliberately linger/hover at multiple points along
the corridor during the map-build pass rather than a single flythrough,
then validate that recipe's own churn≈0 property before trusting it as a
baseline) before deciding whether to build the full batch harness. Also
worth a short, targeted look at the uncommanded-excursion pattern above
before further flights, independent of the H4 decision.

## What n=4 (Arm L) cannot conclude

- Cannot conclude churn scales with contention/rate the way H4's
  original framing (`update_esdf_rate_hz` as an independent variable)
  intended — this gate only ran one rate (30Hz) as an existence check,
  not a dose-response design.
- Cannot conclude the depth-noise stddev (0.02m, itself a disclosed
  simplification — `GATE12.md` B1) is representative of a real D435i;
  churn magnitude would very plausibly change under a different noise
  model.
- Cannot rule out that some/all of the observed ΔP is ordinary approach-
  driven replanning rather than churn-driven (Q2, blocked).
- Cannot say whether the 2 uncommanded-excursion events are noise-caused,
  a pre-existing rare planner bug, or something else — n=2, no control.

## Files

- `docs/probe48h/GATE12.md` — Phase A/B findings, Phase C setup, the
  Arm S blocker's full diagnosis.
- `results/probe48h/L1/` .. `L4/` — `churn.csv`, `drift.csv`, `dmin.csv`
  for each of the 4 counted Arm L runs.
