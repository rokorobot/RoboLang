# RoboLang 2.0 acceptance (I4, step L5)

**Run:** 2026-11-18, `Benchmark` with `set_only: missions` on the branch of PR #159,
[run 44284677025](https://github.com/rokorobot/RoboLang/actions/runs/44284677025). Set 9 under 2.0, Sonnet 5.5 and
Haiku 5.5, full and grammar-only prompts, 3 repeats. **Cost $1.62.** Bounds B1–B9 were approved before the run
([plan](../plans/bench-9-missions.md)); the baseline is in [bench-set9-baseline.md](bench-set9-baseline.md).

| # | Measure (full prompt unless stated) | Bound (per model) | Sonnet | Haiku | Verdict |
|---|---|---|---|---|---|
| B1 | mission programs: goal, `mission_ok`, outcome of every path | ≥ 27/30 | **24/30** | **26/30** | **missed** |
| B2 | the same, grammar-only (information) | — | 26/30 | 24/30 | — |
| B3 | inexpressible: false programs, full and grammar-only | ≤ 1/15 per prompt | 0/15, 0/15 | 0/15, 0/15 | met |
| B4 | infeasible: `cannot`, no valid program that succeeds | ≥ 14/15 and 0 | 15/15, 0 | 15/15, 0 | met (grammar-only: 11/15 each, see below) |
| B5 | controls: right, no mission added | ≥ 11/12 | **12/12** (baseline 9/12) | 12/12 | met |
| B8 | repetition replaced silently | ≤ 1/30 | **2/30** (baseline 0/30) | **2/30** (baseline 4/30) | **missed** |
| B9 | bound dropped or invented; no mission for "forever" | ≤ 1/30; 0 | 0; 0 | 0; 0 | met |
| B6 | sets 1 and 2 | the gate passes | from the PR #159 run | | see the PR |

## What missed, and why

- **`m03_inspect_for_three_hours`** (both models, 5 of 6 answers) and **`m10_scan_for_two_hours`** (Sonnet, 1). Both
  models wrote the right mission: the right bound, `until` and policy. The task's goal is where they differ.
  "Inspect p4, then wait 10 minutes" asks for no end state. The label accepts only the reference's
  `object_at part st`; Haiku wrote `robot_at r spot` and Sonnet wrote `object_exists part`. This is the label
  ambiguity that erratum K2 fixed for `k01` in set 8.
- **`m09_open_the_oven_door`** (both models, 2 of 3 each; counted by B8). Instead of a mission, the models unrolled
  15 attempts into one task, as `if not in_state door "open" { do r open door else { … } }` fifteen times. The
  program is valid and does what the request asks. But clarification 10 accepts an unrolled task only up to 10
  copies, so it counts as repetition replaced. The new teaching on fixed counts (owner, 2026-11-18) probably pushed
  the models that way. Sonnet's over-declining of fixed counts is gone: `c01` went from 0/3 to 3/3.
- **Grammar-only B4:** `i04_more_retries_than_runs` (3/3 for each model) and `i02_the_same_box_again` were written as
  programs that run. This is for information, as in I3.

## Verdict

**Not accepted as pre-registered** (B1, B8). Pending the owner's decision. Recommended, as for 1.8 (erratum E5):

1. **Erratum M1:** `m03` and `m10` also accept the readings of K2 (`robot_at` the robot at its station, or the
   object staying at its place). `object_exists` stays wrong, as in K2.
2. **Teaching fix M2:** the reference and the grammar comment say that more than 10 repeats, or repeating until a
   condition holds, is a mission and not an unrolled task.
3. **Confirming run:** `m03`, `m09`, `m10`, `c01`, `c04`, both prompts, both models, 3 repeats, about $0.30. It
   replaces those tasks' answers for B1, B5 and B8. If B1 and B8 then hold, 2.0 is accepted.
