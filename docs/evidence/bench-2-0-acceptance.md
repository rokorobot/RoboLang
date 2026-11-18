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

**ACCEPTED by the owner, 2026-11-18, with one recorded exception (B1 for Sonnet).**

After errata M1 and M2, the confirming runs (full prompt:
[44286597065](https://github.com/rokorobot/RoboLang/actions/runs/44286597065); grammar-only:
[44286801836](https://github.com/rokorobot/RoboLang/actions/runs/44286801836); `m03`, `m09`, `m10`, `c01`, `c04`,
3 repeats, $0.40) replace those tasks' answers:

| # | Measure (full prompt) | Sonnet | Haiku |
|---|---|---|---|
| B1 | mission programs | 25/30 | **29/30** |
| B5 | controls | 12/12 | 12/12 |
| B8 | repetition replaced silently | **0/30** | **0/30** |

- **The exception:** Sonnet's B1 misses (25/30 against 27) are all one thing. In `m03` (3/3) and `m10` (2/3) it wrote
  the right mission (bound, `until`, policy) but gave the inspection task the goal `object_exists`, which K2 and M1
  do not accept. The request asks for no end state, so the language makes the model invent a goal. This finding
  is recorded in the atlas as the far gap **G-OBSERVE**: tasks that only observe and need no end state.
- Grammar-only B1 is for information: Sonnet 26/30, Haiku 27/30.

**Paid cost of I4:** baseline $1.10, plus about $2 for two runs lost to the bug fixed by W2; acceptance $1.62;
confirming runs $0.40.
