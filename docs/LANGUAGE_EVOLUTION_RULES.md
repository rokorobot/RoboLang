# Rules of RoboLang language evolution

**Status:** PROPOSED 2026-11-17 at the owner's request; approved when the owner merges the pull request that adds it.
Binding for people, agents and loops that work on the
[language evolution strategy](LANGUAGE_EVOLUTION.md). They add to the decision rules 1–6 of the
[strategy](STRATEGY.md) and never weaken them. Only the owner changes this page; a change is recorded in the log at
the end.

## Data (atlas)

- **LE-1 One source.** [`bench/atlas.json`](../bench/atlas.json) is the only source of activities, gaps and
  priorities. [humanoid-activities.md](plans/humanoid-activities.md) is generated from it and never edited by hand;
  `tests/test_atlas_report.py` fails if it is out of date.
- **LE-2 Sourced or dropped.** Every activity cites sources that were actually opened, and its `basis` says what they
  show. A claim with no source that opens is dropped, not guessed.
- **LE-3 Strict maturity.** `concept`: not demonstrated, rests on leading signals; `research`: shown in a paper or
  lab; `demo`: shown by a vendor, no named customer doing real work; `pilot`: a named site doing real work;
  `deployment`: ongoing paid work. When in doubt, the lower stage.
- **LE-4 Falsifiable forecasts.** Every activity has a forecast that can be marked true or false from public
  information on its due date, with an honest probability. Due forecasts are marked and scored (loop C).
- **LE-5 Leading signals for the future.** A `concept` activity rests on at least two leading signals of different
  kinds. Far activities carry leading signals so their future need is evidence, not opinion.
- **LE-6 Directions.** The nine directions are fixed; adding, merging or removing one is an owner decision recorded
  in the strategy and here.

## From data to language

- **LE-7 Verified gaps only.** A gap enters the language phase (L1) only when benchmark tasks prove it: an obvious
  attempt fails with a named diagnostic, and models must decline the request today (R3).
- **LE-8 Two speeds.** Only near gaps (needed by a `pilot` or `deployment` within three years) get syntax. Far gaps
  get design sketches. Every design states which far needs it keeps possible.
- **LE-9 Scope moves only by decision.** A gap that is out of scope (`out`) is reopened only by its own decision
  record. No loop, metric or agent reopens it.
- **LE-10 One gap per iteration.** An iteration closes one gap with the smallest construct that does it, and records
  Economy. At most one language iteration is in flight (strategy rule 3).

## Acceptance and safety

- **LE-11 Pre-registered acceptance.** The tasks and bounds are fixed before any code (strategy rule 6). Automation
  never changes the quality definition, the acceptance tasks or the bounds.
- **LE-12 The promises are hard gates.** False programs and decision accuracy are never traded for coverage
  (strategy rule 2).
- **LE-13 RoboLang carries no safety function.** Safety constructs (zones, limits, proximity) are requests checked
  against the controller's envelope; collision avoidance and prediction of motion stay with the planner and the
  safety controller (safety case C1).

## Automation and records

- **LE-14 Automation proposes, the owner decides.** Loops and agents open pull requests; they never merge, never
  release, never start a paid benchmark run without the owner's confirmation. Each loop run either opens a PR or does
  nothing.
- **LE-15 Everything is recorded in GitHub.** Each loop run adds a development log entry (`--snapshot`); each research
  pass has an evidence report in `docs/evidence/`; each language change has a decision record in `spec/decisions/`.
  Work lives on `main`, reached through pull requests, so it is available from any computer.

## Log

| Date | Change |
|---|---|
| 2026-11-17 | First version, LE-1 to LE-15, proposed at the owner's request |
