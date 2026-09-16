# Reconciliation summary — 2026-09-10

Autonomous run per the task's standing plan: reconcile README/ARCHITECTURE,
bring feasibility-gate tooling+data onto the working branch, add a research
log. Branch `r2-closedloop-probe` (verified before starting, per the
task's "STOP if it doesn't exist" instruction — it existed). All work
committed there in three commits; `feasibility-gate` and `main` untouched
and identical to `origin`, `main` not merged into. **Pushed to
`origin/r2-closedloop-probe`** (remote tracking branch already existed, no
force push used).

## Commits

1. `156e187` — README.md + ARCHITECTURE.md reconciliation.
2. `c533199` — `.gitignore` narrowed + `results/feasibility_gate/` result
   data (1,967 files, ~75MB).
3. `8c2accf` — `docs/research_log.md` (+ `docs/reconcile/PHASE0_FINDINGS.md`
   from the exploration phase).

## Files changed / added

- `README.md` — 2 "Update (post-study)" blockquotes added (top-of-doc
  forward pointer, Phase 6 incident narrative, "Known limitations"
  softened). Original text preserved verbatim in every case.
- `ARCHITECTURE.md` — 2 "Update (post-study)" blockquotes added (Milestone
  2 definition, Milestone 2 done-criterion). Original text preserved
  verbatim.
- `.gitignore` — `/results/feasibility_gate/` (blanket exclusion) narrowed
  to `/results/feasibility_gate/*/rosbag/` (rosbags only).
- `results/feasibility_gate/` — 1,966 pre-existing result files (logs/CSVs
  per run) + `REPORT.md` + `mini_dose_screen.png`, newly committed.
- `results/feasibility_gate/README.md` — new: what's committed vs.
  local-only, run-directory index, how runs were generated, seeding, how
  to reproduce.
- `docs/research_log.md` — new: synthesized research log for the year-end
  report.
- `docs/reconcile/PHASE0_FINDINGS.md` — new: exploration-phase findings
  and the full numeric-verification trail.
- `docs/reconcile/RECONCILE_SUMMARY.md` — this file.

## Every number verified, with source

All experimental numbers written into `README.md`, `ARCHITECTURE.md`, and
`docs/research_log.md` Section 1 were read directly from
`flight_test_log.html` during this run (not from the companion planning
documents, not from memory of an earlier session). Full trail in
`docs/reconcile/PHASE0_FINDINGS.md` §0b; highlights:

| Claim | Value | Source (flight_test_log.html) |
|---|---|---|
| Rich contention trend | p=0.663, d=−0.062 | §49 `m2-gate-sweep` |
| Poor contention trend | p=0.415, d=−0.199 | §51 `m2-gate-sweep-poor` |
| Texture main effect | RPE ≈2.1×, dropout ≈7×, features ≈0.40× | §51 |
| Pilot n=5 → false positive | d≈1.019, power 0.19-0.28 at n=5, required n=17-25 | §47 `m2-power` |
| Endpoint→RPE spread reduction | 22.2× → 3-4× (4.0×/3.3×) | §46 `m2-remetric` |
| Recording trim | 530MB → 8.4MB (63×) | §48 `m2-unblockers` |
| 5-preset nsys separation | 0/296.3/2,564.1/7,852.6/19,548.5 µs | §48 `m2-unblockers` |
| R2-0/R2-1/R2-2 | 0.0296m÷8 / p=0.540,d=−0.132÷128 / 0/60 large-div, p=0.235,0.593,0.449 | `route2-probe`, `route2-r2-2` |
| No incident rosbag exists | confirmed, no `/visual_slam/*` topics in any repo bag | §43 (via "Known gaps") |

No mismatch was found between the companion documents' summary numbers and
the log — all were confirmed accurate.

## `[VERIFY]` / `TODO` left in place

- `docs/research_log.md` §2-4 (literature positioning, OV²SLAM claims,
  candidate-direction literature review): carried from
  `Dfrog_research_direction_report.md`, explicitly flagged
  **[VERIFY before citing formally]** — these are external paper claims,
  not repo data, and the source document itself says several were read
  from abstracts only, with no systematic novelty survey done. Not
  independently re-verified against the source papers in this session.
- `docs/research_log.md` §6, items 2-6 (Orin Nano access, target/timeline,
  thesis-vs-external framing, interaction interest, advisor notification):
  marked **TODO: supply from research notes** — these are pending
  decisions for the user, not facts to derive from the repo.
- The 55m incident's cause: explicitly **not stated** anywhere (README,
  ARCHITECTURE, research log) — marked "pending forensic triage" per the
  task's hard constraint. No forensic triage was run this session (out of
  this task's scope; the direction report's own proposed triage prompt is
  quoted for reference in research_log.md §5 but not executed).

## Decisions made under ambiguity

1. **Committed a soft-reset mid-task.** The first commit accidentally
   combined the README/ARCHITECTURE changes with the already-staged
   feasibility-gate result data (a leftover `git add` from the Phase 2
   staging step). Caught immediately via `git log --stat`; fixed with
   `git reset --soft HEAD~1` (safe — nothing had been pushed yet, no data
   lost) and re-committed as two separate commits. Flagging this because
   it's the one point where the plan wasn't followed cleanly on the first
   try.
2. **Per-run CSVs/logs committed despite `CLAUDE.md`'s general "per-run
   CSVs stay gitignored" convention.** This task's Phase 2 instructions
   explicitly asked to bring "RESULT CSVs / analysis outputs / figures"
   into the branch (excluding only raw rosbags and files >~50MB) — a
   deliberate, scoped override of the general convention for this study
   specifically, not a change to the convention itself. `results/
   route2_probe/` was left under the old blanket-gitignore convention
   since Phase 2 named only feasibility-gate.
3. **`results/feasibility_gate/README.md` states plainly that
   `analyze_sweep.py` reads rosbags directly, not the committed CSVs** —
   so committing the CSVs makes the study's summary reproducible/
   inspectable, but does not let someone re-run the exact historical
   analysis without the local rosbags (11.4GB, not committed). This is a
   real limit on "reproducible," disclosed rather than glossed over.
4. **`docs/research_log.md`'s literature sections (§2-4) are presented as
   carried-forward analysis, not independently fact-checked claims** — the
   task's non-negotiable rule requires every WRITTEN quantitative claim to
   trace to a repo file; external paper findings (e.g., OV²SLAM's ≈27×
   ATE rise) cannot be verified against a file in this repo, so they are
   attributed to the source document and flagged for the reader to verify
   before formal citation, rather than either fabricating verification or
   omitting them entirely (they're load-bearing for understanding why
   Direction E was proposed).

## Blockers

None. All three tasks completed; no phase required inventing a number,
guessing branch state, or stopping short.

## Not pushed / follow-up

Everything above **was pushed** to `origin/r2-closedloop-probe`. No merge
to `main` was performed or attempted (deferred by standing decision — see
project memory `project_branch_strategy.md`). `feasibility-gate` was read
from (`git ls-files`/`git show` equivalents, though nothing was actually
needed from it — the tooling was already present on `r2-closedloop-probe`)
but never checked out, merged, rebased, or otherwise modified.
