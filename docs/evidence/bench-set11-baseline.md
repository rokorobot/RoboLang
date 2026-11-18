# Benchmark set 11 "kinds": baseline on RoboLang 2.1

**Run:** 2026-11-18, `Benchmark` with `set_only: kinds`,
[run 44405701483](https://github.com/rokorobot/RoboLang/actions/runs/44405701483). Sonnet 5.5 and Haiku 5.5, full and
grammar-only prompts, 3 repeats, 24 tasks ([plan](../plans/bench-11-kinds.md), decision
[0044](../../spec/decisions/0044-more-measured-kinds.md)). **Cost $1.21.** Not gated.

| Prompt | Model | Kind tasks declined | False programs (all valid and SUCCEEDED) | Infeasible | Inexpressible | Controls |
|---|---|---|---|---|---|---|
| full | Sonnet 5.5 | 30/30 | 0/30 | 15/15 | 15/15 | 12/12 |
| full | Haiku 5.5 | 30/30 | 0/30 | 13/15 (2 `clarify`) | 15/15 | 12/12 |
| grammar | Sonnet 5.5 | 28/30 | **2/30** | 15/15 | 15/15 | 12/12 |
| grammar | Haiku 5.5 | 26/30 | **2/30** (and 2 `clarify`) | 14/15 | 14/15 | 9/12 (`c03` declined 3/3) |

## Reading

With the full prompt nothing is silent, so 2.2 adds coverage there. With the grammar-only prompt, both models wrote
`k04_weed_and_ninety_percent_sure` as a `label` check alone. That drops "at least 90 % sure" and still succeeds:
it is the silent substitution of 1.6 (`inspect` for a reading) in a new form. B8 guards against it under 2.2.
