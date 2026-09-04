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

Raw bags and per-run CSVs stay gitignored (`/ros2_ws/rosbags/`,
`/results/feasibility_gate/`) — regenerable experiment data, not source. The
log is what holds the durable record: summary numbers, key findings, and the
commit SHA each change/result landed in, so the record survives even though
the raw data behind it doesn't get committed.

A task is done only when it's in the repo and verifiable, not when a
transcript says it was done.
