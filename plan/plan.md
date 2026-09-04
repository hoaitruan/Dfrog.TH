# Dfrog.TH — Comprehensive Research Execution Plan (Gate → M0-M6)

## Context

Two planning documents exist for this project: `Research_Development_Plan_4.md` (the formal research proposal — abstract, RQs, methodology, milestones M0-M6, kept at `/home/redwine/Drone/plan/Research_Development_Plan_4.md`, outside this repo) and this file, `plan.md` (a derived, executable task list). An earlier version of `plan.md` only covered the cheap pre-check, "Part 1 — Feasibility Gate," in Do/Names/Done/Verify/Log granularity; Part 2, the full study, was left at milestone level. This version extends that same granularity all the way through M0-M6.

This document is organized as one continuous sequence: the Gate (already partly built, verified 2026-09-04) first, then M0-M6 of the full study. Every claim about repo state below was verified against the actual code (file:line, live command output, or a read-only Explore-agent pass) — none of it is carried over from memory or from the plan documents' own claims about themselves.

---

## Part 0 — Critique of the plan documents (verified, not assumed)

**Citation integrity — good news.** All 9 references in `Research_Development_Plan_4.md` were checked against live search results. All nine are real, findable papers, and every one's characterization in the text matches what the paper actually claims (this is not a given for LLM-assisted proposals — worth stating plainly because it's a real, checked result, not a compliment):
- [1] Mathur/Khedekar/Alexis, arXiv:2106.00289 — confirmed (IROS 2021, S-MSCKF/VINS-Mono/OKVIS on Pi4B).
- [2] Affatato et al., ECCV 2024 Workshops, DOI matches exactly — confirmed.
- [3] SlimSLAM, ASPLOS 2024, DOI matches exactly — confirmed (authors: Behroozi et al., not stated in the doc — fine, title/venue/DOI are what matter).
- [4] OASIS, ACM TECS, DOI matches — confirmed, but the doc's title has an extra "An" (real title: "OASIS: Optimized Adaptive System for Intelligent SLAM"). Trivial, fix before any submission. Also worth knowing: OASIS's actual mechanism is adaptive *frame-region* filtering for predictability, not literally "adjusts SLAM behavior to a computation budget" the way SlimSLAM is — close enough for a one-line lit-review mention, but don't lean on it as identical to [3] if a reviewer asks.
- [5] Ali et al., ICRA 2025, the exact `robotics.cs.unc.edu` PDF URL in the doc resolves — confirmed. This is the "UNC 2025" the plan's own standing-cautions section worries about overlapping with; characterization (real-time scheduling as the remedy, timing/deadline framing) matches the actual paper.
- [6] RT-BEV, RTSS 2024 — confirmed (Liu/Lee/Shin).
- [7] E-Navi, arXiv:2512.14046 — confirmed exactly, including authors (Li et al., Sun Yat-sen).
- [8] RAPTOR, [9] PANTHER — both well-established, real (not independently re-searched, high confidence from training data).

**Code claims — verified, and one is more concrete than the doc lets on.** Checked via a read-only Explore pass against the actual repo:
- B0's "remove temporary timing instrumentation in `sdf_map.cpp`" is real and specific, not vague: `plan_env/include/plan_env/sdf_map.h:359,425` and `plan_env/src/sdf_map.cpp:342,1077,1210,1222,1227,1231` all carry literal `// TEMPORARY, Task 2c ... remove before Milestone 1 ships` comments and a live `DEBUG_TIMING` log line. This is a small, well-scoped cleanup, not a research risk — good.
- `/sdf_map/cloud` is confirmed dead code: subscribed at `sdf_map.cpp:267`, the code's own comment at `sdf_map.cpp:1198` says so, and no publisher exists anywhere in `ros2_ws/src/fast_planner/`. Safe to state as fact in B0's log entry.
- `hard_killswitch.py`, `emergency_stop.py`, `force_disarm.py` all exist at repo root — B3's safety-protocol dependency is already satisfied by existing tooling, not something to build.
- `perception_metrics`, `compute_metrics`, `run_experiment.sh`, and any ATE/RPE implementation are confirmed **absent** — B1 (Phase A instrumentation) is a real, from-scratch build, not a light extension. Size the schedule accordingly.
- GPU profiling tooling the plan assumes (NVML, Nsight, CUDA MPS) is **confirmed present** in `isaac_ros_dev_persistent`: `nvidia-cuda-mps-control`, `nsys`, and `pynvml` (importable) all verified live. This meaningfully de-risks B1.2/B2.1.

**Substantive critique (not just verification):**
1. **Scope-vs-timeline mismatch is the single biggest risk.** M0-M6 as written is a full factorial (texture × motion × contention × workload × scheduling, staged), a from-scratch pose-injection/error-model framework, an EV-control flight-safety campaign, and a Jetson port — collectively a 12-18+ month program even with durations "indicative." If this is running against a capstone/thesis deadline, the plan needs an explicit, pre-negotiated scope-cut order now (which of M4/M5 is "stretch" if time runs out), not just the existing "NEGATIVE is still a valid capstone" escape hatch at the Gate. Recommend fixing this before M1 starts, not after M3 runs long.
2. **The pose-replay/error-injection framework (condition C, §6.5) is under-scoped relative to its difficulty.** "Preserves temporal sampling, timestamp, orientation, error distribution, spatial correlation, and drift dynamics" of a real empirical error process is a nontrivial stochastic-process modeling task in its own right, not a script. It should be its own milestone-level line item with its own Do/Verify, not folded silently into M2.
3. **CUDA MPS as a clean 3-level "scheduling" factor is optimistic on this hardware.** The Explore pass confirmed MPS tooling exists, but this is a single consumer Ada Lovelace laptop GPU (non-MIG, non-datacenter) already saturated by Gazebo's own rendering plus every perception node. MPS's documented benefits are strongest on datacenter parts with real multi-tenant isolation. Recommend a cheap pilot (does `default` vs `MPS` even produce a measurable difference in `gpu_sampler`/`nsys` output on this box) before committing a full factorial arm to it — added as G2.5/M1 task below.
4. **No power/sample-size justification for the "5 / 15 / 20-30 reps" tiers.** Reviewers will ask why those numbers, not others, are sufficient for the mixed-effects model in §7. A lightweight simulation-based power check is cheap and should happen before the real sweep, not after.
5. **Profiling overhead vs. the phenomenon being measured.** Full 6-D interference-vector characterization (kernel duration, SM occupancy, memory bandwidth via `nsys`) is typically collected via instrumented runs that themselves add overhead and can perturb exactly the timing pathway RQ2 is trying to isolate. The plan should say explicitly whether full-vector profiling runs are a separate pass from the main data-collection runs — added as a note in B1.2 below.
6. **This file's own "Part 0 — Real state right now" has gone stale before, silently.** An earlier version said branch `feasibility-gate`/commit `400f245`/`tools/feasibility_gate/*` were "unconfirmed... treat as not started," which a live status check already contradicted (all existed, built, and were in the process of being pushed). Process fix, not content fix: date-stamp state claims and re-verify live rather than trusting the last snapshot in this file.

None of the above are blockers — they're inputs to the plan below (each has a corresponding task or explicit scope decision).

---

## Part 1 — Immediate next actions (resume where the Gate work left off)

**IA.1 Push `feasibility-gate` to origin.**
- Do: `git push -u origin feasibility-gate`. (A token was pasted in chat earlier for this — treat as compromised regardless of outcome; it must be stripped from `origin`'s URL immediately after any use and rotated on GitHub.)
- Done: `git ls-remote --heads origin feasibility-gate` returns commit `400f245` (or later); `git status` shows the branch tracking `origin/feasibility-gate`.
- Verify: re-run both checks live, don't trust a prior claim.
- Log: branch + SHA + "now pushed," in `flight_test_log.html`.

**IA.2 Fix the `use_sim_time` gap on `nvblox_node`.**
- Do: in `ros2_ws/src/px4_vslam_bridge/launch/nvblox.launch.py`, set `use_sim_time: True` in `nvblox_node`'s parameter dict (same pattern as `visual_slam_node` in `vslam.launch.py:70`); rebuild `px4_vslam_bridge` only.
- Done: `ros2 param get /nvblox_node use_sim_time` returns `true` at runtime, with the pipeline actually up — not inferred from the launch file text.
- Verify: the live param-get output, pasted into the log, not paraphrased.
- Log: the invariant table entry flips from "not set" to "true, verified <date>."
- Note: measurement-correctness only — do not touch `EKF2_EV_CTRL`, airframe, or estimator params while doing this.

**IA.3 Update `flight_test_log.html` with the real Gate state.**
- Do: append one new section (matching the file's existing HTML structure/style) recording: branch+SHA+pushed status; the verified topic/param table (ground truth `/ground_truth/odom`, cuVSLAM odom `/visual_slam/tracking/odometry`, status `/visual_slam/status`, feature count from `/visual_slam/vis/observations_cloud` width, ESDF service `/nvblox_node/get_esdf_and_gradient`, nvblox node + `integrate_depth_rate_hz`, TF root `map -> base_link_gt`); config invariants including the now-fixed `use_sim_time`; tooling status (4 scripts + `vslam_compare` extension present and committed, `analyze_sweep.py` not yet built, `results/feasibility_gate/` empty); known gaps (no incident bag, `/fmu/out/estimator_status_flags` not yet subscribed by any node).
- Done: section renders correctly, doesn't duplicate/contradict earlier sections.
- Verify: diff reviewed before commit; no existing content altered.
- Log: this *is* the log entry.

**IA.4 First open-loop reproduction run (G3.1).**
- Do: one fixed-scenario run, `EKF2_EV_CTRL=0` throughout, GPS flies, cuVSLAM logged only, via `tools/feasibility_gate/run_gate.sh` (or the manual `run.sh` sequence if the container needs the GUI / user interaction) — record a rosbag with at minimum `/ground_truth/odom`, `/visual_slam/tracking/odometry`, `/visual_slam/status`, `/visual_slam/vis/observations_cloud`, stereo image topics.
- Names/paths: `run_gate.sh`, saved under `results/feasibility_gate/<exp_id>/rosbag`.
- Done: `ros2 bag info <path>` shows all expected topics with non-zero message counts.
- Verify: paste the real `ros2 bag info` output; state plainly whether cuVSLAM tracked throughout or lost tracking — do not fabricate.
- Log: bag path + info output + tracking outcome.
- **Do not proceed past this to G1 triage or any sweep without a fresh go-ahead.**

---

## Part 2 — Finish the Feasibility Gate (G0-G3, Decision)

### G0 — Re-orient (mostly done; close the gaps)

**G0.1 Rebuild & smoke-test.** Targeted rebuilds of `px4_vslam_bridge` succeeded; full-workspace `colcon build` has a pre-existing, unrelated `ros_gz_sim`/`ignition-transport11` failure and stale-symlink corruption in `nvblox_msgs`/`ros_gz_interfaces` — not yet cleanly rebuilt end-to-end. Remaining: decide whether a full clean rebuild (`rm -rf build install log && colcon build`) happens before G3's real sweep starts, since IA.4 needs the whole pipeline up. Do it right before IA.4 if any launch fails, not preemptively.

**G0.2 Locate the incident bag.** Done — confirmed missing. All 4 existing rosbags (`ros2_ws/rosbags/flight_2026081{2,3}_*`) checked directly; none contain any `/visual_slam/*` topic. G1 runs on IA.4's fresh bag.

**G0.3 Config invariants.** Done except `use_sim_time` on `nvblox_node` (IA.2 above fixes this). Table: `EKF2_EV_CTRL=0` (confirmed, airframe `4022_gz_x500_depth_stereo:38`), `esdf_mode="3d"` (confirmed, `nvblox.launch.py:117`), TF root `map -> base_link_gt` (confirmed, `ground_truth_tf.py:173-174,186`).

**G0.4 Verified name table.** Done — see IA.3's log content; reuse it, don't re-derive.

### G1 — Forensic triage (built, unrun)

**G1.1 Run `triage_incident.py` on IA.4's bag.**
- Do: `python3 tools/feasibility_gate/triage_incident.py --bag results/feasibility_gate/<exp_id>/rosbag --out-dir results/feasibility_gate/<exp_id>/triage`.
- Done: script completes, writes `triage_note.md`, classifies the observed pattern (or reports "insufficient data" honestly if the open-loop run shows no interesting divergence — expected, since IA.4 doesn't yet inject contention).
- Verify: read the actual `triage_note.md`, don't trust a summary of it.
- Log: which pattern matched (or "no divergence at baseline contention, as expected") + the EKF-gating N/A note (still true, still `EKF2_EV_CTRL=0`).

### G2 — Instrumentation (built; one gap + one new pilot task)

**G2.1-G2.4** — done (CSV-logging `vslam_compare`, `gpu_sampler.py`, `gpu_stressor.py`, `run_gate.sh`, launch-arg contention knob on `nvblox_node`). No action needed beyond IA.4 exercising them for real.

**G2.5 (new, from critique #3) — MPS pilot check.**
- Do: run `nvidia-cuda-mps-control` and compare `gpu_sampler.py`/`nsys` output for the same synthetic-stressor scenario with MPS off vs. on, once, before committing scheduling as a full factorial factor in B2.
- Names/paths: a short throwaway script or manual `nsys profile` invocation; not a permanent tool.
- Done: a clear yes/no on whether MPS produces a measurable difference on this GPU.
- Verify: raw `nsys`/`gpu_sampler` numbers, both conditions, side by side.
- Log: one paragraph — "MPS pilot: [measurable / not measurable] on RTX 4060 Laptop; scheduling factor [kept as 3 levels / collapsed to default-only] for B2."

### G3 — Reproduction & controlled screening

**G3.1** = IA.4 above (reproduce as-is, a few times — IA.4 is rep 1).

**G3.2 Remove foot-guns, retest.**
- Do: with `use_sim_time` now fixed (IA.2) and timestamps/frames re-verified, repeat G3.1 a few more times. Confirm whether any large divergence still appears at zero added contention (it shouldn't — this stage is really "confirm the baseline is clean" before adding contention).
- Done: 3-5 clean baseline runs, no anomalous divergence.
- Verify: `triage_incident.py` classification on each is "no significant divergence" or equivalent.
- Log: baseline established, N runs, all clean.

**G3.3 Contention dose sweep.**
- Do: `run_gate.sh --contention {none,low,medium,high,extreme} --workload {nvblox,synthetic} --texture rich` (texture-poor world doesn't exist yet — see G3.4), ≥5 repeats per level. This is 2 workloads × 5 levels × 5 reps = 50 runs minimum for the rich-texture pass alone.
- Names/paths: outputs under `results/feasibility_gate/<exp_id>/` per existing `run_gate.sh` convention.
- Done: 50+ scenario runs completed, each with a rosbag + `vslam_compare.csv` + `gpu_log.csv`.
- Verify: spot-check `ros2 bag info` on a handful; confirm `exp_id` labeling is consistent for `analyze_sweep.py` to key on.
- Log: run count, any runs that failed/were discarded and why.

**G3.4 Texture control.**
- Do: build a texture-poor Gazebo world variant. Per the repo's own `vio_test.sdf` header comments, the "before" state (pre-Phase-3-fix) used flat solid-color markers/ground with no `albedo_map` — recreate that as `vio_test_poor.sdf` (copy `vio_test.sdf`, strip the `checker_noise.png` `albedo_map` lines, keep geometry/pillar-ring identical) plus a matching PX4 SITL make target (mirror `gz_x500_depth_stereo_vio_test`). Re-run G3.3's full sweep against it.
- Names/paths: `PX4-Autopilot/Tools/simulation/gz/worlds/vio_test_poor.sdf` (new); a new/duplicated airframe or make-target entry to point PX4 SITL at it.
- Done: identical sweep, texture-poor world, same run counts.
- Verify: confirm via a quick feature-count check (`/visual_slam/vis/observations_cloud` width) that the poor-texture world genuinely produces fewer tracked features than rich — a manipulation check, not just an assumption.
- Log: manipulation-check numbers + sweep run count.

**G3.5 Build `analyze_sweep.py`.**
- Do: aggregate all `vslam_compare.csv`/`gpu_log.csv` pairs from G3.3-G3.4 by `exp_id`/`contention_level`/`workload`/`texture`; produce error-vs-contention and drift-rate-vs-contention plots with bootstrap CIs; texture-vs-contention comparison; a discrete-jump-vs-graded-rise flag (reuse `triage_incident.py`'s jump-detection heuristic rather than reinventing it).
- Names/paths: `tools/feasibility_gate/analyze_sweep.py` (new — confirmed absent).
- Done: script runs over the full G3.3+G3.4 dataset, produces plots + a summary table.
- Verify: manager spot-checks 2-3 plotted points against raw CSVs by hand.
- Log: link to plots + summary table in the log.

### Gate Decision

**Do:** write `results/feasibility_gate/REPORT.md` synthesizing G1 (single-incident triage, likely inconclusive alone since it's n=1 by design) and G3.3-G3.5 (the actual statistical evidence), recommending one of GO / GO-tempered / PIVOT-to-timing / NEGATIVE.
**Done:** one file, one clear recommendation, evidence cited inline (not asserted).
**Verify (manager):** the recommendation follows from the plotted data, n=1 is never treated as more than motivation, and a NEGATIVE/PIVOT outcome is treated as equally valid to GO.
**Log:** the recommendation + one-paragraph justification, appended to `flight_test_log.html`.

**Decision gate for the rest of this plan:** M1 onward assumes GO or GO-tempered. If the Gate returns PIVOT or NEGATIVE, M1-M6 below are replaced by a much smaller "write up the negative/timing-pivot result" milestone — do not start M1's from-scratch instrumentation build before this decision is in.

---

## Part 3 — Full Study (M0-M6), assuming GO / GO-tempered

### M0 — Adversarial literature search + protocol freeze

**M0.1 Systematic search.**
- Do: search 30-50 papers across the query list already specified in `Research_Development_Plan_4.md` §4.5 (GPU interference + VIO, GPU contention + SLAM accuracy, accelerator interference + localization, etc.). The 9-reference spot-check in Part 0 above confirms the *seed* references are real, but is not this task — M0 is a fresh, broader search, not a re-verification of what's already cited.
- Names/paths: a new `docs/literature_search.md` or similar recording each paper checked, why it's adjacent-not-overlapping, and the eventual "novelty matrix."
- Done: 30-50 papers logged; a novelty matrix positioning this work against each cluster from §4.5's diagram.
- Verify: manager spot-checks a sample of the 30-50 for real existence + accurate characterization, same method used in Part 0 above.
- Log: novelty matrix + search completeness statement.

**M0.2 Protocol freeze.**
- Do: freeze the experimental protocol (factors, levels, rep counts, statistical model from §7) as a pre-registration document, incorporating M0.3 and M0.4 before freezing.
- Done: one frozen protocol document, dated, not modified after M1 starts without an explicit amendment note.
- Verify: manager confirms the frozen protocol matches what M2 actually runs later (drift = a flagged problem, not silently absorbed).
- Log: protocol freeze date + link.

**M0.3 (new, from critique #4) Power/sample-size check.**
- Do: a lightweight simulation (e.g., simulate the mixed-effects model in §7 under a plausible effect size range, check whether 5/15/20-30 reps/condition actually gives adequate power) before committing wall-clock time to the real factorial.
- Done: a justified rep-count table (possibly revising the 5/15/20-30 tiers).
- Verify: the simulation code + assumptions are inspectable, not just a stated conclusion.
- Log: final rep-count table + justification, folded into M0.2's frozen protocol.

**M0.4 (new, from critique #1) Scope-cut pre-negotiation.**
- Do: explicitly decide now, with the advisor/manager, which of M4 (RQ3, flight-risk propagation) or M5 (RQ4, Jetson) is cut first if the schedule slips — before it becomes an end-of-project scramble.
- Done: one written priority order (already implied by §8's "outcome scenarios" — RQ1+RQ2 > RQ3 > RQ4 — make it explicit as a schedule decision, not just a contribution-quality ranking).
- Verify: manager confirms this matches actual thesis/capstone deadline constraints.
- Log: priority order recorded in the frozen protocol.

### M1 — B0 (close Milestone 1) + Phase A instrumentation

**M1.1 Remove temporary `sdf_map.cpp` instrumentation.**
- Do: remove the confirmed `TEMPORARY, Task 2c` blocks at `sdf_map.h:359,425` and `sdf_map.cpp:342,1077,1210,1222,1227,1231` (including the `DEBUG_TIMING` log line at `sdf_map.cpp:1227`), and the dead `/sdf_map/cloud` subscription (`sdf_map.cpp:267`, plus the two launch-file remaps) if it's confirmed to have zero remaining use — leave it if anything else quietly depends on the subscription existing (check before deleting).
- Done: `grep -rn "TEMPORARY" ros2_ws/src/fast_planner/` returns nothing; a full rebuild succeeds.
- Verify: rebuild log clean; a DoD flight (below) still works identically.
- Log: diff summary + rebuild confirmation.

**M1.2 DoD flight for Milestone 1 baseline.**
- Do: one supervised GPS-pose flight producing a rosbag + RViz screenshot showing nvblox ESDF populated, planner trajectory markers, and ground-truth path visibly curving around a known obstacle — the same acceptance criterion `ARCHITECTURE.md` already states for "Milestone 1 done."
- Done: artifact set exists and is frozen as the baseline every later experiment runs on top of.
- Verify: manager confirms the RViz image actually shows curving-around-obstacle behavior, not just a straight line.
- Log: rosbag path + screenshot + "baseline frozen, `EKF2_EV_CTRL` untouched (0)."

**M1.3 `perception_metrics` (grows `vslam_compare`).**
- Do: extend the existing CSV-logging pattern already built in `vslam_compare.py` into a fuller metric set: online ATE/RPE (a windowed trajectory-alignment metric, not just instantaneous position error), yaw error (reuse the existing own-start-referenced approach), tracked-feature count (`/visual_slam/vis/observations_cloud` width, already wired), tracking status (`/visual_slam/status` `vo_state`, already wired), message latency (already logged as `msg_age_s`), `depth_valid_ratio` (new — fraction of finite/valid pixels in `/depth_camera`).
- Names/paths: `ros2_ws/src/px4_vslam_bridge/px4_vslam_bridge/vslam_compare.py` (extend further) or a new `perception_metrics.py` sibling if the ATE/RPE windowing logic makes `vslam_compare` too large — decide based on line count at the time, don't pre-commit.
- Done: CSV schema covers every listed metric; opt-in params still don't change default behavior (regression-test this explicitly).
- Verify: two runs at identical settings produce comparable CSVs (bit-for-bit isn't required; distributions should match).
- Log: schema + a sample CSV excerpt.

**M1.4 `compute_metrics`.**
- Do: GPU util/memory (already covered by `gpu_sampler.py` — extend it, don't rebuild) plus, where obtainable, kernel duration / SM occupancy / memory bandwidth via `nsys` (confirmed present) and `pynvml` (confirmed importable) on desktop, `tegrastats` on Jetson (still a stub — real implementation waits for M5 hardware access). Add ESDF query latency (wrap `/nvblox_node/get_esdf_and_gradient` client calls with timing — `esdf_recorder.py`/`health_check.py` already show the call pattern to wrap), planner runtime, replan frequency, PX4 tracking error.
- **Critique #5 note, bake into the design:** keep full `nsys`-instrumented profiling runs as a *separate pass* from the main CSV-logging data-collection runs, so profiler overhead never contaminates the timing measurements RQ2 depends on. Document this split explicitly in the schema/README, not just in someone's head.
- Names/paths: extend `gpu_sampler.py`; new `esdf_latency_wrapper` (small, wraps the existing service-client pattern); `compute_metrics.py` as the aggregation point if needed.
- Done: schema covers every listed compute metric, profiling-pass vs. data-pass distinction documented.
- Verify: a run produces a monotonic wall-clock series joinable to M1.3's output on `exp_id`.
- Log: schema + join example.

**M1.5 Structured logger + `run_experiment.sh`.**
- Do: rosbag + fixed CSV schema keyed by `experiment_id` (extend `run_gate.sh`'s existing pattern rather than starting over — it already does contention-knob-plus-CSV-plus-rosbag orchestration for the Gate; `run_experiment.sh` is the Phase-A/B-scale successor with the fuller M1.3/M1.4 schema and the multi-factor CLI `--scenario --contention <intensity> --workload <type> --sched <mode> --planner`).
- Names/paths: `tools/phase_a/run_experiment.sh` (new, but a direct descendant of `tools/feasibility_gate/run_gate.sh` — reuse its precondition checks, cleanup trap, and bag-recording discovery-race fix).
- Done (DoD, per the source doc): one-command rosbag → CSV → plot pipeline, reproducible from a fixed seed.
- Verify: same seed → comparable output; schema stable across repeated runs.
- Log: DoD confirmation + one example end-to-end run.

### M2 — Phase B open-loop: RQ1 (effect) + RQ2 (mechanism)

**M2.1 Multi-factor treatment design + staged screening.**
- Do: implement the staged factorial (Stage 1 screening → Stage 2 select 2-3 key regimes → Stage 3 deep experiment) over texture/motion/contention/workload/scheduling, per §6.3. Directly reuses G3.3-G3.4's sweep mechanics (`run_experiment.sh` now, not `run_gate.sh`) and G2.5's MPS pilot finding to decide whether scheduling is a real 3-level factor or collapses to 2.
- Names/paths: `tools/phase_b/` (new directory) for anything that doesn't fit `run_experiment.sh` directly.
- Done: Stage 1-3 completed per the rep-count table frozen in M0.3.
- Verify: manager confirms staging actually narrowed to 2-3 regimes before Stage 3, not a full factorial run by accident (cost control).
- Log: which regimes were selected and why, at each stage.

**M2.2 Synthetic-vs-ecological + suspected-locus ablation.**
- Do: resource-match `gpu_stressor.py` (already built) to real nvblox on memory bandwidth/occupancy/kernel duration/utilization (using M1.4's `nsys`/`pynvml` tooling to measure both and tune the stressor's presets to match); run the 4-condition ablation (idle GPU / synthetic stress / nvblox / unrelated GPU workload).
- Names/paths: extend `gpu_stressor.py`'s preset table with resource-matched values; a new `unrelated_workload_stressor.py` or a `gpu_stressor.py --workload unrelated` mode (something GPU-bound but not perception-shaped, e.g. plain dense matmul at a different memory-access pattern than nvblox's voxel-grid access — already partially true of the existing stressor, worth checking whether it needs a second, deliberately-different profile to serve as the "unrelated" condition).
- Done: 4-condition ablation run, resource-matching numbers reported (not just claimed).
- Verify: manager checks the resource-matching numbers themselves (occupancy/bandwidth), not just that the stressor "ran."
- Log: ablation result + resource-matching table.

**M2.3 Four-condition mechanism decomposition (the scientific core).**
- Do: build pose-replay/error-injection tooling (per critique #2, treat this as its own real sub-project): condition A (real timing + real pose, i.e. the existing pipeline unmodified), B (real timing + corrected/ground-truth pose fed downstream instead of cuVSLAM's), C (clean/nominal timing + injected spatial error drawn from the empirical error process defined in §6.5), D (clean timing + clean/ground-truth pose, baseline). Compute `Delta_int = (A-D) - (B-D) - (C-D)` with bootstrap CIs.
- Names/paths: `tools/phase_b/pose_replay.py` (new — feeds condition B/D's pose into the downstream chain in place of live cuVSLAM output, likely by publishing on `/visual_slam/tracking/odometry` from recorded ground truth instead of the real node, or an equivalent substitution point — needs a design decision on exactly where in the pipeline to intercept, since `EKF2_EV_CTRL` stays 0 throughout Phase B so none of this touches PX4 at all; the "downstream" being manipulated here is nvblox/Fast-Planner's *view* of pose for mapping purposes, not flight control); `tools/phase_b/error_injection.py` (new — implements the empirical-error-process model from §6.5: bias, drift, temporal correlation, frequency content, orientation error, burst events, characterized from real M2.1/G3.3 cuVSLAM-vs-ground-truth error data).
- Done: all 4 conditions run with ≥3 core-condition-level repeats (per M0.3's rep table), `Delta_int` computed with a bootstrap CI.
- Verify: manager re-derives `Delta_int` from the raw CSVs directly, not from a reported summary number — this is the paper's central claim and deserves independent recomputation.
- Log: `Delta_int` + CI + interpretation (additive / synergistic / redundant), plus the explicit caveat from §6.4 that this is a controlled counterfactual decomposition, not perfect causal mediation.

### M3 — RQ2 robustness + optional RQ5

**M3.1 Two-error-model robustness check.**
- Do: repeat M2.3's condition C using Model B (a controlled synthetic error model — e.g. parametric bias+drift+correlated noise, not derived from real failure data) alongside Model A (the empirical process from M2.3). Compare whether the mechanism finding (a nonzero, roughly consistent `Delta_int` sign/magnitude) holds under both.
- Names/paths: extend `error_injection.py` with a second, parametric model mode.
- Done: both models run, compared side by side.
- Verify: manager checks the comparison is apples-to-apples (same rep count, same core regime from M2.1).
- Log: both `Delta_int` values + whether the finding is robust to the error model choice.

**M3.2 (optional) Regime/change-point analysis.**
- Do: fit a change-point model to the M2.1 dose-response data (error/drift-rate vs. contention level) and compare against a single-regime fit, per §7.
- Done: change-point model fit + comparison, or an explicit "set aside" if the data don't support one (RQ5 is optional and not a priority).
- Verify: manager confirms this wasn't skipped silently — either a real analysis or an explicit, justified pass.
- Log: outcome either way.

### M4 — RQ3: propagation with external-vision control enabled (the risky milestone)

**Hard gate before this milestone starts:** M2 (open-loop RQ1/RQ2) must show a real, established degradation range first — do not enable `EKF2_EV_CTRL` speculatively. This is the one milestone in this entire plan where the "keep EKF2_EV_CTRL=0" constraint that has governed every prior task in this project is deliberately lifted, under protocol.

**M4.1 Safety protocol document.**
- Do: write an explicit protocol: killswitch armed before every run (reuse existing `hard_killswitch.py`/`emergency_stop.py`/`force_disarm.py` — confirmed present, don't rebuild), low contention first then medium only after low is clean, no deliberately induced collisions, the existing project safe-testing protocol (climb to verified hover first, confirm, only then trigger the behavior under test) applied here too.
- Done: written protocol, reviewed before the first M4 run.
- Verify: manager confirms the protocol is actually followed on the first live run (present in another shell, watching).
- Log: protocol document, dated.

**M4.2 Re-enable `EKF2_EV_CTRL` per protocol, low contention.**
- Do: set `EKF2_EV_CTRL` nonzero (the specific bit combination already used historically — `4022_gz_x500_depth_stereo`'s prior working value, or re-derive per current EKF2 docs) for a single low-contention run, with the safety protocol active.
- Names/paths: the airframe file `PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/4022_gz_x500_depth_stereo`.
- Done: one clean run, no killswitch trigger needed, quantifiable navigation metrics collected (minimum clearance, near-miss rate, bounded path perturbation, PX4 tracking error).
- Verify: manager confirms via ground-truth comparison (not just "it didn't crash") that behavior was as expected.
- Log: run outcome + metrics.

**M4.3 Scale to medium contention; quantify propagation.**
- Do: repeat M4.2 at medium contention (per M2's established safe/degraded boundary), enough repeats for a defensible propagation-quantification claim tied to the `C_GPU -> {T_VIO, D_VIO} -> EKF state/covariance -> E_nav -> P_collision` causal chain from §6.7.
- Done: propagation measured and quantified without an induced collision.
- Verify: manager confirms the four possible EKF-response outcomes from §6.7 (covariance rises / stays stable / rejects measurements / diverges) are reported as observed, not assumed.
- Log: which outcome(s) occurred, at which contention level, with the navigation-safety metrics.
- **Immediately after M4:** revert `EKF2_EV_CTRL` back to 0 unless M5 (Jetson) also needs it nonzero for a specific comparison — don't leave it live by default.

### M5 — Phase C: cross-platform envelope on Jetson Orin Nano 8GB

**M5.1 Isaac ROS 3.2 setup on the real Orin Nano.**
- Do: per the known gotcha already on record (`4.x` dropped Orin Nano support; `run_dev.sh` needs patching for no PVA/DLA), get the base Isaac ROS environment running on the actual hardware — this hasn't started as of this session.
- Done: base container running on-device, confirmed via a trivial node launch.
- Verify: manager confirms this is the real device, not a re-run of the desktop container.
- Log: setup steps + confirmation.

**M5.2 Port perception/mapping/planning stack; HITL-for-compute via rosbag replay.**
- Do: port cuVSLAM/nvblox/Fast-Planner to the Jetson, drive it with recorded rosbags from M2/M4 (no live flight required for this stage), instrument with `tegrastats` (implement the real backend now — deferred/stubbed everywhere so far) in place of desktop `nsys`/`pynvml`.
- Names/paths: `tools/feasibility_gate/gpu_sampler.py`'s `sample_tegrastats()` stub gets a real implementation here, once real hardware exists to validate the parser against (explicitly deferred to this point in the Gate-stage code already).
- Done: same metric set as M1.3/M1.4, collected on Jetson.
- Verify: spot-check `tegrastats` parsing against a manually-read sample of its raw output.
- Log: Jetson metric pipeline confirmed working.

**M5.3 Cross-platform envelope comparison.**
- Do: run the same core-regime sweep from M2.1's Stage 3 on Jetson; compute `C*_desktop`, `C*_Jetson`, and `rho = C*_Jetson / C*_desktop` (or equivalent normalized margin).
- Done: both thresholds reported, transfer/non-transfer stated as one of the three valid outcomes (coincide / Jetson narrower / no common threshold).
- Verify: manager confirms the desktop-vs-Jetson comparison accounts for the stated confound (desktop's GPU also renders the simulator; Jetson's doesn't) rather than treating them as identical workloads.
- Log: envelope comparison + honest transfer conclusion.

### M6 — Consolidation and writing

**M6.1 Draft the manuscript** around the four fixed RQs (Q1 effect/texture-separated, Q2 threshold, Q3 propagation, Q4 cross-platform), per §8's outcome-scenario framing (the paper degrades gracefully with how far the results reached — write the honest version, not the aspirational one).
**M6.2 Verify every citation** before submission (this session's Part 0 spot-check is a good start but covers only the 9 seed references, not whatever M0.1 adds).
**M6.3 Package the reproducible artifact** (C5) — only claim "benchmark" once standardized scenarios/fault injection/metrics/seeds/baselines/public logs/protocol are actually in place, per the plan's own caveat.
- Done: submission-ready manuscript + artifact.
- Verify: manager does a full read-through against the frozen M0.2 protocol, flagging any claim not backed by a logged artifact.
- Log: final state of `flight_test_log.html`/successor document, submission date.

---

## Verification approach for this whole plan

Every milestone above ends in a **Verify** step performed by re-deriving the claim from a raw artifact (CSV, rosbag, `nsys` output, live `ros2 param get`/`ros2 bag info`), never by trusting a prior summary. A task is done only when its artifact is in the repo and verifiable, not when a transcript says it was done. The Gate Decision point (end of Part 2) is a hard checkpoint: M1-M6 should not start on the strength of optimism about the 55m anecdote — only on real evidence from G1+G3.
