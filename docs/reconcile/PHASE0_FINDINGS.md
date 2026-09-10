# Phase 0 — Exploration findings

Run 2026-09-10, on branch `r2-closedloop-probe` (confirmed via `git branch
--show-current` before starting; matches the standing plan's required
branch). Working tree was clean at start.

## 0a. README.md / ARCHITECTURE.md stale-attribution locations

Grep for `contention|55|not flight-safe|EKF2_EV_CTRL` (case-insensitive):

**README.md**
- L84, L103, L247-249: neutral architecture description of `EKF2_EV_CTRL`
  (which topic/service it is, current value) — not a causal claim, no
  update needed.
- **L260-270 (Phase 6 narrative)**: "First cruising flight test caused
  cuVSLAM tracking to drift ~55m from true position under GPU contention
  with nvblox's concurrent CUDA context, while EKF2 was still fusing it —
  PX4 chased the phantom error with real thrust." States contention as the
  cause outright. **Stale — needs an Update note.**
- **L361-364 ("Known limitations")**: "cuVSLAM is not flight-safe under
  concurrent nvblox GPU load on this 8GB laptop GPU — architecturally
  worked around (GPS-only control) rather than fixed." Direct safety claim
  the factorial contradicts (contention was null; nothing established
  cuVSLAM is unsafe under concurrent load on this platform). **Stale —
  needs softening per the task's specified language.**

**ARCHITECTURE.md**
- L9: "Control flew on GPS, not cuVSLAM (`EKF2_EV_CTRL 0`)." — a shortcut
  description in the mission-audit framing, not a causal claim. No update
  needed.
- L24-32: pose-stays-on-GPS rationale for Milestone 1 isolation — sound
  methodological reasoning independent of the contention question, no
  causal claim about *why* GPS was chosen beyond isolating variables. No
  update needed.
- **L36-41 (Milestone 2 definition)**: "Re-enable `EKF2_EV_CTRL`,
  root-cause the cuVSLAM 55m GPU-contention drift under concurrent nvblox
  load ... this is the harder problem and is partly a hardware/GPU-
  scheduling concern..." States the drift as a GPU-contention phenomenon
  to be root-caused as such. **Stale — needs an Update note.**
- L43-46: says any `EKF2_EV_CTRL`/GPU-scheduling change belongs to
  Milestone 2 — a scoping rule, still valid procedurally (r2-closedloop-probe
  is exactly that Milestone-2-scoped probe). No update needed, though
  worth a pointer since Milestone 2 work has in fact now happened (on this
  branch, not merged).
- **L56-58 ("Milestone 2 is done when...")**: "...without the
  GPU-contention drift recurring" — bakes in the contention framing as the
  thing to avoid recurring. **Stale — needs an Update note.**

## 0b. flight_test_log.html — verified numbers (source of truth; trust this
over any prose summary)

Read directly from the file (`grep`/`sed` against the live HTML, not from
memory or the companion planning documents). Every number below is quoted
verbatim from a specific section.

**Texture-vs-contention factorial, §49 (rich, id="m2-gate-sweep") + §51
(poor, id="m2-gate-sweep-poor"), 200 runs total (100 rich + 100 poor):**

| Level | Rich mean (RPE-1s) | Rich dropout | Poor mean (RPE-1s) | Poor dropout |
|---|---|---|---|---|
| none | 0.150 | 1/20 (5%) | 0.373 | 18/20 (90%) |
| low | 0.159 | 3/20 (15%) | 0.310 | 17/20 (85%) |
| medium | 0.160 | 3/20 (15%) | 0.272 | 16/20 (80%) |
| high | 0.136 | 4/20 (20%) | 0.291 | 15/20 (75%) |
| extreme | 0.142 | 1/20 (5%) | 0.344 | 17/20 (85%) |
| **overall** | **0.149** | **12/100 (12%)** | **0.318** | **83/100 (83%)** |

- Texture main effect: poor RPE-1s ≈2.1× rich's; poor dropout rate (83%) ≈7×
  rich's (12%). Feature-count ratio (manipulation check, rich vs. one
  fresh poor verification flight): poor/rich = 105.8/262.6 ≈ 0.40×.
- Contention trend, rich (n=100): slope=−0.0039/level, r=−0.044, **p=0.663**,
  bootstrap 95% CI (−0.022, 0.014) crosses zero. Cohen's d (none→extreme)
  = **−0.062** (negligible).
- Contention trend, poor (n=100): slope=−0.0078/level, r=−0.082, **p=0.415**,
  bootstrap 95% CI (−0.027, 0.011) crosses zero. Cohen's d (none→extreme)
  = **−0.199** (small, wrong direction).
- Poor's clean-tracking-only subset (n=17, lopsided 2-5/level) shows a
  nominally non-zero-crossing CI but is explicitly flagged in the log
  itself as unreliable (too few distinct bootstrap resample outcomes,
  single-point leverage) — **not to be cited as a finding**.

**Pilot screen, §45/§46 (id="m2-mini-dose"), n=5/level, 15 runs total —
the small-sample→full-sample reversal:**
- Baseline (zero-contention) 5 identical runs: 0.85m, 12.07m, 15.63m,
  1.37m, 18.83m — mean 9.75m, stdev 8.24m, >20× spread run-to-run with
  zero difference in treatment.
- Group means at n=5 were **non-monotonic** (none 9.75m < high 21.73m <
  low 98.12m) and heavily overlapping run-to-run — the pilot signal did
  not survive to the powered n=20/level sweep (§49/§51 above), which
  returned a clean null both times. This is the "well-powered null, not a
  pilot-scale false positive" distinction cited in §54 of the log.

**R2-2 closed-loop, real-nvblox-contention, §53 (id="route2-r2-2"), 60
runs against the real obstacle world with the original Phase 6 control
node (`reactive_esdf_avoidance`):**
- Zero large-divergence (>1m) events at any of L0/L1/L2, across 60 runs —
  the 55m regime did not reproduce.
- Mean-divergence trend: slope=0.00337/level, **p=0.235**; EV message-age
  trend: slope=0.211ms/level, **p=0.593**; EV rejected% trend: slope=
  −0.071%/level, **p=0.449**. All cross zero / flat.
- All 10 runaway events were `descent_rate`-kind killswitch breaches; zero
  were `divergence`-kind (the mechanism this probe exists to detect).
- Across all three closed-loop stages combined (R2-0+R2-1+R2-2, 200 runs):
  zero divergence-kind killswitch triggers anywhere.

**No mismatch found** between the log's actual numbers and the "texture
~2.1×, dropout ~7×, features ~0.4×, contention null" summary quoted in the
task — the summary is accurate to the source file. All figures used in
Phase 1/3 below are taken from this section, not re-derived or estimated.

**The 55m incident itself**: no verified cause exists anywhere in the log.
§43 (via grep at "Known gaps") states directly: "No incident rosbag exists
for the original ~55m drift — none of the 4 rosbags currently in the repo
carry any `/visual_slam/*` topic." The forensic triage that could establish
cause was never run. Per the task's non-negotiable rules, no definitive
cause is stated anywhere in this reconciliation.

## 0c. Feasibility-gate tooling + result data — paths, types, sizes

**Tooling — already committed on `r2-closedloop-probe`** (verified via
`git ls-files`, no action needed for Phase 2's tooling half):
```
tools/feasibility_gate/run_gate.sh
tools/feasibility_gate/analyze_sweep.py
tools/feasibility_gate/gpu_stressor.py
tools/feasibility_gate/gpu_sampler.py
tools/feasibility_gate/triage_incident.py
```
(`tools/feasibility_gate/__pycache__/` exists locally, correctly untracked
— covered by the global `__pycache__/` ignore rule.)

**Result data — currently entirely untracked**, excluded by
`.gitignore:26` (`/results/feasibility_gate/`), documented there as
"regenerable experiment data, not source" per `CLAUDE.md`'s standing
convention. Contents:
- 226 run subdirectories (`gate_sweep_*`, `mini_dose_*`, `g3_1_baseline_*`,
  `frame_check_*`, `taskc_interrupt_test*`, `verify_*`), each holding
  `bag_record.log`, `fast_planner_bridge.log`, `fast_planner_trigger.log`,
  `force_disarm_cleanup.log`, `gpu_log.csv`, `gpu_sampler.log`,
  `gpu_stressor.log`, `vslam_compare.csv`, `vslam_compare_node.log`, and a
  `rosbag/` subdirectory.
- Two top-level files: `REPORT.md` (7.2KB, the provisional gate report —
  content already cross-checked against the log in 0b) and
  `mini_dose_screen.png` (52.8KB figure).
- **Total size: 12GB.** Of that, **11.4GB is `rosbag/*.db3` files** — two
  are individually 879-881MB (`taskc_interrupt_test`,
  `taskc_interrupt_test_v2`), several more in the 500MB range
  (`mini_dose_*`). **These must not be committed** (way over the ~50MB
  cap and explicitly excluded as raw rosbags by the task's rules).
- Excluding `rosbag/` entirely: **75MB across 1,966 files**, no single
  file over 50MB (largest are two `vslam_compare.csv` at ~9MB each, from
  the `g3_1_baseline_*` pre-gate baseline-characterization runs). This
  portion is in scope to commit per Phase 2's instructions (CSVs/logs are
  not raw rosbags and none exceeds the size cap).
- No `.mcap` files found anywhere; all bag data is `.db3` (ROS 2
  `sqlite3` bag format) plus a `metadata.yaml` per run.

`results/route2_probe/` (1.4GB, same `rosbag/`-heavy structure) exists
under the same `.gitignore` rule but is **out of this task's explicit
scope** (Phase 2 names "feasibility-gate" specifically) — left untouched,
noted here for completeness and flagged in the final summary.

Nothing relevant found on the `feasibility-gate` branch beyond what's
already present in the working tree — `git show feasibility-gate:...` was
not needed since `tools/feasibility_gate/*.py` are already identical/
present on `r2-closedloop-probe` and the result data was never committed
on either branch (both branches gitignore it identically).

## 0d. Research-log draft — exists?

**No draft exists inside this repo.** Searched the working tree (`find`
for `*research*direction*`, `*research_log*`, `*comprehensive_report*`) —
no hits.

Two source documents do exist, but in the **companion planning workspace
outside this repo**, at `~/Drone/plan/` (sibling directory, not tracked by
this git repository, not on any branch here):
- `~/Drone/plan/dfrog_comprehensive_report.md` (164 lines) — full project
  retrospective.
- `~/Drone/plan/Dfrog_research_direction_report.md` (340 lines) —
  literature-review-backed direction analysis, recommending "Direction E."

These are used as **source material** for Phase 3's `docs/research_log.md`
(read directly, fact-checked line-by-line against 0b's verified numbers,
not copied verbatim) — consistent with this project's established pattern
of treating `~/Drone/plan/` as a planning workspace whose durable output
gets synthesized into the repo, not copied wholesale.

## Blockers

None. All three tasks are executable with verified data; no phase requires
inventing a number or guessing branch state.
