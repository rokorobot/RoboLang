# Benchmark set 9 "missions": baseline on RoboLang 1.9

**Run:** 2026-11-18, `Benchmark` with `set_only: missions`,
[run 44268337035](https://github.com/rokorobot/RoboLang/actions/runs/44268337035). Sonnet 5.5 and Haiku 5.5, full and
grammar-only prompts, 3 repeats, 26 tasks ([plan](../plans/bench-9-missions.md), decision
[0041](../../spec/decisions/0041-mission-layer.md)). **Cost $1.10** for the answers. The two earlier attempts
answered every request too, about $1 each, but lost their answers (erratum W1 in [errata](bench-errata-2-0.md)).
The answers of this run come from the `benchmark-answers` artifact, uploaded before scoring. They were **scored
offline** after the fix of erratum W2, because scoring had run the CI runner out of memory.

| Prompt | Model | Mission tasks declined | False programs (of which valid and SUCCEEDED) | Infeasible | Inexpressible | Controls |
|---|---|---|---|---|---|---|
| full | Sonnet 5.5 | 30/30 | 0/30 | 15/15 | 21/21 | **9/12** |
| full | Haiku 5.5 | 26/30 | **4/30** (4) | 15/15 | 21/21 | 12/12 |
| grammar | Sonnet 5.5 | 28/30 | 2/30 (1) | 15/15 | 21/21 | 12/12 |
| grammar | Haiku 5.5 | 18/30 | **12/30** (9) | 14/15 (1 `clarify`) | 19/21 (2 `clarify`) | 12/12 |

## Findings

- **Repetition is replaced silently, the B8 failure.**
  - With the full prompt, Haiku wrote 1.9 programs for `m09_open_the_oven_door` 3/3 and for
    `m08_rounds_out_of_the_wet_floor` 1/3. Each one is valid and succeeds.
  - With the grammar-only prompt, Haiku did it for 12 of 30 answers. It unrolled rounds or stacked handlers, for
    example 18 handled steps for "carry it until it is there".
- **Over-declining a fixed count.** Sonnet declined the control `c01_three_round_trips` (three round trips, one plain
  task) in 3/3 answers. This is the teaching gap that R3 already saw (`c_count_two_totes`).
- **Erratum W2, a product bug.** Answers with dozens of handled steps exposed an exponential walk over the plan's
  paths in `robolang.bridge`, which every `export-goal`, `delegate` and the scorer go through. One such valid task
  made `robolang export-goal` run for 15 s and then fail out of memory; with the fix it takes 0.16 s.
