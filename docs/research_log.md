# Research log: GPU contention, scene texture, and the ~55m incident

**Status:** working document for the year-end report. Synthesizes two
planning-workspace documents (`~/Drone/plan/dfrog_comprehensive_report.md`,
`~/Drone/plan/Dfrog_research_direction_report.md`, both dated 2026-09-09,
outside this repo) with the primary record in `flight_test_log.html`
(§42-55) and `results/feasibility_gate/REPORT.md`. Every number in
**Section 1** was re-verified directly against `flight_test_log.html`
while writing this document (2026-09-10) — see
`docs/reconcile/PHASE0_FINDINGS.md` for the verification trail.
**Sections 2-4** carry forward analysis and literature claims from the
direction report; those are marked as such and are not independently
re-verified against source papers in this session.

---

## 1. Experimental findings (verified against `flight_test_log.html`)

### 1.1 The question

Does GPU contention between cuVSLAM (localization) and nvblox (dense
mapping), sharing one GPU, degrade VIO localization accuracy — and does
that propagate to navigation safety? Motivated by a single, unrecorded
~55m cuVSLAM drift observed once during Phase 6 closed-loop testing (see
Section 5 below) — an n=1 anecdote with no rosbag, not evidence on its
own.

### 1.2 Method summary

A cheap open-loop go/no-go gate was run before committing to a larger
study (`tools/feasibility_gate/`, branch `feasibility-gate`), followed by
a closed-loop probe (`tools/route2_probe/`, branch `r2-closedloop-probe`).
`EKF2_EV_CTRL` stayed at its default `0` (GPS-only control) throughout the
open-loop gate; the closed-loop probe deliberately set it nonzero (`11`)
on an isolated new airframe (`4900`), confined to `r2-closedloop-probe`,
reverted to `0` afterward and verified live — the shared `4022` airframe
every other experiment uses was never touched.

### 1.3 Methodology results (a finding in their own right)

- **Endpoint/ATE error is unusable as a treatment metric.** Five identical
  zero-contention runs produced max errors of 0.85m, 12.07m, 15.63m,
  1.37m, 18.83m — mean 9.75m, stdev 8.24m, a >20x range ratio (measured:
  22.2x) with zero difference in treatment. Switching to **RPE, 1.0s
  window** (per-run median) cut the range ratio to 3-4x (4.0x for the 1.0s
  window, 3.3x for the 1.0m arc-length window) — a real improvement, still
  not "stable."
- **A windowed drift-rate slope metric was tried and made things worse**
  (CV 1.54 vs. endpoint's 0.85) — dropped.
- **`nvidia-smi` GPU utilization cannot separate contention presets**
  (94.5% `low` vs. 95.7% `high` — a 1.01x ratio). `nsys` kernel-duration
  profiling separates all five presets cleanly and monotonically: none
  (idle, 0 kernels) / low 296.3µs / medium 2,564.1µs / high 7,852.6µs /
  extreme 19,548.5µs avg CUDA kernel duration.
- **A small-sample false positive, caught before it propagated.** At the
  n=5/level pilot, RPE-1s showed a monotonic none<low<high trend with
  Cohen's d≈1.0 (none vs. high) — but simulation-based power analysis
  showed this pilot had only 0.19-0.28 power (a coin-flip-or-worse chance
  of detecting even a real effect this size), and the required sample was
  n≈17-25/level. At the powered n=20/level, this trend **collapsed to a
  null** (below) — exactly the scenario the power analysis predicted, and
  the reason a ~400-run study was run instead of stopping at n=5.
- **A recording-size blocker was fixed, not worked around**: per-run disk
  footprint went from 530MB to 8.4MB (63x) by trimming the bag to only
  the topics the analysis pipeline reads, with zero loss of metric
  fidelity (verified: RPE recomputed identically from the reduced bag).

### 1.4 Main result: texture dominates, contention is null

Full factorial, n=20/level, randomized order, fresh restart + confirmed
disarm between every run, RPE-1s metric, 200 runs (100 rich-texture +
100 poor-texture):

| Level | Rich RPE-1s mean | Rich dropout | Poor RPE-1s mean | Poor dropout |
|---|---|---|---|---|
| none | 0.150 | 1/20 (5%) | 0.373 | 18/20 (90%) |
| low | 0.159 | 3/20 (15%) | 0.310 | 17/20 (85%) |
| medium | 0.160 | 3/20 (15%) | 0.272 | 16/20 (80%) |
| high | 0.136 | 4/20 (20%) | 0.291 | 15/20 (75%) |
| extreme | 0.142 | 1/20 (5%) | 0.344 | 17/20 (85%) |
| **overall** | **0.149** | **12/100 (12%)** | **0.318** | **83/100 (83%)** |

- **Contention trend, rich (n=100):** slope=−0.0039/level, p=**0.663**,
  bootstrap 95% CI (−0.022, 0.014) crosses zero. Cohen's d
  (none→extreme)=**−0.062** (negligible).
- **Contention trend, poor (n=100):** slope=−0.0078/level, p=**0.415**,
  bootstrap 95% CI (−0.027, 0.011) crosses zero. Cohen's d
  (none→extreme)=**−0.199** (small, wrong direction).
- **Texture main effect: LARGE.** Poor-texture RPE-1s ≈2.1x rich's;
  poor-texture tracking-loss rate (83%) ≈7x rich's (12%); manipulation
  check confirms poor genuinely has fewer trackable features (feature
  ratio poor/rich ≈0.40x).
- Poor's clean-tracking-only subset (n=17, lopsided 2-5/level) shows a
  nominally non-zero-crossing bootstrap CI, but is explicitly flagged in
  the source data as **unreliable** (too few distinct resample outcomes,
  single-point leverage) and should not be cited as a finding.

**Verdict: NEGATIVE for the contention hypothesis, well-powered.** This is
a well-powered null (adequate to detect the effect size that would have
mattered), not an underpowered "cannot tell."

### 1.5 Closed-loop probe (Route 2)

Because the open-loop null couldn't test the original incident's actual
mechanism (EKF2 accepting a high-innovation measurement — only observable
closed-loop), a three-stage closed-loop probe followed, on the isolated
`r2-closedloop-probe` branch:

| Stage | Conditions | Runs | Result |
|---|---|---|---|
| R2-0 | closed-loop, no contention | 8 | mean divergence 0.0296m ± 0.0216m, 0/8 (0%) large-divergence |
| R2-1 | synthetic none/medium/high, obstacle-free | 120 (128 total incl. R2-0) | trend p=**0.540**, d≈**−0.132** (negligible); 1 isolated runaway (medium, altitude-low breach, landing-overshoot-consistent), no divergence-kind trigger fired |
| R2-2 | **real nvblox load + real obstacle world + original incident's own control node** (`reactive_esdf_avoidance`), L0/L1/L2 | 60 | **0/60 large-divergence reproductions**; mean-divergence trend p=**0.235**; EV message-age trend p=**0.593**; EV rejected% trend p=**0.449** — all flat |

R2-2 was the decisive test: real nvblox CUDA contention (not synthetic),
the actual original control node (not Fast-Planner), and the actual
obstacle-laden cruise trajectory. All 10 runaway events across R2-2 were
`descent_rate`-kind killswitch breaches (marginal, 1.50-1.55m/s against a
1.5m/s threshold, one at 3.77m/s); **zero were `divergence`-kind** — the
exact mechanism the probe exists to detect. Across all three closed-loop
stages combined, zero divergence-kind killswitch triggers fired anywhere.

**Decision: Route 2 CLOSED, NO-GO (robust).** `EKF2_EV_CTRL` reverted to
`0` in the default config afterward, confirmed live.

### 1.6 Combined verdict

Across ~400 total automated runs — open-loop and closed-loop, synthetic
and real nvblox contention, obstacle-free and obstacle-laden worlds, two
textures — **no contention-gated phenomenon was found under any tested
condition.** Scene texture, not GPU contention, is the operative driver of
localization degradation observed in this study.

---

## 2. Positioning vs. prior work

*(Carried forward from `Dfrog_research_direction_report.md`'s literature
review. These are external literature claims, not repo data — they were
not independently re-verified against the source papers in this session.
The source document itself notes: several key papers, including the
OV²SLAM contention study and the Embry-Riddle texture thesis, were read
from abstracts/result snippets only, not in full; a systematic 20-40
paper survey for the "common practice is underpowered" claim has not
been performed; arXiv identifiers are "as surfaced during review; verify
before formal citation." Treat every claim below accordingly —
**[VERIFY before citing formally]**.)*

- **Texture → degradation is not novel except for stack-specificity.** An
  Embry-Riddle (2025) thesis and a separate downfacing-VIO paper both
  already relate feature-level metrics to VIO error across rich/poor
  textures with ground truth. What's specific to this study: the cuVSLAM
  + nvblox coupling — a thin, benchmarking-only delta.
- **Contention-ruled-out is complicated by prior art showing the opposite
  sign under different conditions.** *Analysis and Mitigation of Shared
  Resource Contention on Heterogeneous Multicore* (arXiv 2304.13110)
  reports OV²SLAM's ATE rising ≈27x under co-scheduled DNN + memory-DoS
  load via shared-DRAM contention on an integrated-GPU SoC. This is not a
  contradiction of Section 1's null (see Section 3) — but it does mean
  "nobody has shown contention degrades SLAM accuracy" is false, and the
  honest framing of this study's null is "no effect on this desktop-class,
  headroom-rich discrete GPU," not "no effect."
- **Feature count as the mediator between texture and error is not
  novel.** IV-SLAM (2020), Introspective Perception (2016), and a
  DROID-SLAM diagnosis paper all already correlate feature/optical-flow
  quality with trajectory error.
- **The methodology lessons (§1.3) are real but not, on their own, a
  paper.** The `nvidia-smi`-is-inadequate finding is established in GPU
  systems literature (ETH SoCC'25; arXiv 2501.16909). The "common practice
  is underpowered" claim would need the 20-40 paper survey noted above —
  not yet done.

---

## 3. The apparent contradiction, resolved

*(Analysis carried forward from the direction report §6 — a piece of
reasoning about how two literature results relate, not a new
experimental result of its own.)*

There is no real contradiction between this study's null and OV²SLAM's
positive contention effect — they describe **different mechanisms at
different operating points**:

- **Texture → accuracy is direct and always-on.** Fewer distinctive
  features means worse geometry for the pose solve, on any hardware —
  this is Section 1's large texture effect.
- **Contention → accuracy is indirect, via timing.** Contention doesn't
  change the computed math; it changes *when* it finishes. Under enough
  load, deadlines are missed and frames/keyframes drop, and *those drops*
  degrade accuracy — OV²SLAM's own shared-DRAM mechanism.

This study's discrete-GPU laptop (RTX 4060, 8GB) apparently never crossed
the load level where frame-drop starvation actually triggers — headroom,
so no drops, so no accuracy hit, consistent with the null in Section 1.4.
OV²SLAM's weak, unified-memory SoC did cross it. Same underlying physics,
different regime. The one genuinely unmeasured quantity this leaves open:
on a starved platform, does a feature-poor scene make the degradation
*worse than additive*, or *less*? Neither this study (no contention main
effect to interact with) nor OV²SLAM (didn't vary texture) can answer
that — it is structurally invisible to both.

---

## 4. Candidate directions and status

*(Carried forward from the direction report §4, §7 — its own synthesis
of where each direction stands after the literature review above.)*

| Direction | Pitch | Status |
|---|---|---|
| A1 | Publish the null as-is | Weak — OV²SLAM shows the "nobody checked" framing is false; workshop-tier at best |
| A2 | Texture-dominant positive reframe | Thinner than first assessed — both legs (texture, contention-null) have close prior art; only the controlled head-to-head + the false-positive lesson survive |
| A3 | Methodology / rigor paper | Narrow — the core insight is borrowed from GPU systems literature; the "common practice" prevalence claim is unproven |
| B | Jetson Orin Nano replay | Re-scoped — folds into Direction E as the second platform, below |
| C | Texture-health runtime monitor | **Drop.** All three functional pieces (health signal, lead-time predictor, closed-loop response) are separately occupied in the literature — notably SUPER (arXiv 2512.14189, 2026) for the lead-time predictor specifically |
| D/E-old | Predict/mitigate a contention-induced failure | Dormant — presupposes a contention effect this study didn't find |

### Direction E — PROPOSED, not a result

**This is a proposed reframe for future work, not something this study
measured or validated.** The idea (direction report §8): stop asking
*which factor causes the error* — Section 1 already answers that for GPU
contention specifically on this platform — and instead ask *where is the
safe/unsafe operating boundary for this coupled localization+mapping GPU
workload, and can it be predicted?* The claim, if it held up, would be
that localization degradation under compute load is a threshold
phenomenon whose location is a predictable function of platform timing
margin and scene feature load.

This would require new work not yet done: porting to a second platform
(Jetson Orin Nano), deliberately pushing mapping load until a cliff is
found (if one exists on that hardware), and building/validating an
offline predictor. **Whether this is worth pursuing depends on open
decisions listed in Section 6** — it is not committed work.

---

## 5. The 55m incident: what's known, and what isn't

**What's documented:** Phase 6 (first cruising flight test with the
reactive-avoidance control node) saw cuVSLAM tracking drift ~55m from
Gazebo-confirmed ground truth while `EKF2_EV_CTRL` was nonzero and EKF2
was fusing it, producing a genuine thrust runaway stopped by the
killswitch. GPU contention with nvblox's concurrent CUDA context was the
attribution made at the time — it was the only variable that had changed
relative to earlier, clean standalone tests.

**What the subsequent ~400-run study found:** the contention hypothesis,
tested as directly as this study could manage — open-loop (Section 1.4)
and closed-loop with the actual incident control node against the actual
obstacle world under real nvblox load (Section 1.5, R2-2) — was null at
every level tested, including the "55m regime" itself (0/60 large-
divergence reproductions).

**What is NOT known, and is not claimed anywhere in this document:** the
specific cause of the original ~55m incident. No rosbag of that event
exists — confirmed directly: none of the rosbags in this repo carry any
`/visual_slam/*` topic. **The forensic triage needed to settle
attribution (was it genuinely closed-loop and fused at that moment; did
feature tracking collapse before or after pose error grew) has not been
run.** The most this document states: contention was the original working
attribution; the subsequent factorial found contention null and texture
dominant; the specific cause of the original incident remains **pending
forensic triage**.

One relevant cross-check, not a substitute for that triage: the direction
report's proposed forensic-triage prompt (its §3.1) would answer Q1
(closed-loop/fused?) and Q2 (mechanism — feature collapse vs. smooth
divergence?) from a recorded incident bag or, failing that, from the
55m-regime reproduction data. No incident bag exists to run Q1/Q2
against directly, but R2-2 (Section 1.5) already is the 60-run
"55m-regime reproduction" data that triage step would have analyzed —
and it shows zero `divergence`-kind events, only benign control-loop
transients. This narrows what a full triage could still find, but does
not replace running it, and does not establish a cause.

---

## 6. Open items

1. **55m forensic triage — not run.** See Section 5. No cause is
   established; none should be asserted until this is done or the
   incident is formally closed as unattributable (no data survives to
   attribute it).
2. **Orin Nano access** — TODO: supply from research notes / advisor
   discussion. Determines whether Direction E's cross-platform centerpiece
   is viable or whether a laptop-only fallback envelope is the ceiling.
3. **Target venue and timeline** — TODO: supply from research notes.
   Workshop/RA-L this cycle vs. a full conference/journal cycle changes
   how much of Direction E (if pursued) is attempted before a first
   submission.
4. **Thesis vs. external publication framing** — TODO: supply from
   research notes. A rigorous null/characterization (Sections 1-3 as
   written) is sufficient for the capstone as-is; external venues would
   weight a predictor and cross-platform result (Direction E) more
   heavily.
5. **Interaction interest** — TODO: supply from research notes. Whether
   the texture×contention interaction (Section 3, the one genuinely
   unmeasured quantity) is worth measuring directly, independent of
   whether Direction E as a whole is pursued.
6. **Advisor/committee notification** — TODO: supply from research notes.
   The direction report (§11) and the comprehensive report both flag this
   as an open administrative step, not something this document can
   resolve.

---

## 7. Sources

- `flight_test_log.html` §42-55 (primary experimental record; every
  number in Section 1 above traces to a specific section there).
- `results/feasibility_gate/REPORT.md`, `results/route2_probe/REPORT.md`
  (provisional draft reports, gitignored rosbags aside — see
  `results/feasibility_gate/README.md`).
- `~/Drone/plan/dfrog_comprehensive_report.md` (2026-09-09, outside this
  repo — project retrospective).
- `~/Drone/plan/Dfrog_research_direction_report.md` (2026-09-09, outside
  this repo — literature review and Direction E proposal).
