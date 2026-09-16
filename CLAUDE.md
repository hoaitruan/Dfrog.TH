# Project conventions for Dfrog.TH

## The log is the single source of truth

`flight_test_log.html` is the single source of truth for this project. After
every experiment batch, config change, or fix:

1. Append a dated section to `flight_test_log.html`, matching its existing
   HTML structure (`<section id="...">` with a `.section-head` containing
   `<span class="idx">NN</span><h2>...</h2>`, a `.section-intro`, then
   content using the existing `callout`/`pill`/`table`/`issue` styles as
   appropriate) — never alter or renumber earlier sections. Add the
   corresponding nav entry.
2. Commit and push (branch `feasibility-gate` for gate work; never commit to
   `main`).

Raw rosbags stay gitignored (`/ros2_ws/rosbags/`,
`/results/feasibility_gate/*/rosbag/`, `/results/route2_probe/*/rosbag/`) —
several run 175MB-881MB, past GitHub's 100MB hard file limit, and a Git LFS
setup was tried and reverted 2026-09-11 (too slow, exceeds the free bandwidth
quota). Per-run CSVs/logs/reports **are** committed (as of 2026-09-10/11 —
this reverses an earlier version of this convention that gitignored them too)
so the ~400+128-run studies are inspectable from the repo, not only from
summary numbers in the log. The log remains the durable narrative record
either way: summary numbers, key findings, and the commit SHA each
change/result landed in.

A task is done only when it's in the repo and verifiable, not when a
transcript says it was done.
