# Benchmark set 8 "clock": baseline on RoboLang 1.8

**Run:** 2026-11-17, `Benchmark` with `clock: true` on the I3 branch,
[run 44192000543](https://github.com/rokorobot/RoboLang/actions/runs/44192000543). Sonnet 5.5 and Haiku 5.5, full and
grammar-only prompts, 3 repeats, 24 tasks ([plan](../plans/bench-8-clock.md), decision
[0042](../../spec/decisions/0042-clock-readings.md)). **Cost $0.75.** Not gated.

| Prompt | Model | Clock tasks declined | False programs | Infeasible | Inexpressible | Controls |
|---|---|---|---|---|---|---|
| full | Sonnet 5.5 | 30/30 | 0/30 | 15/15 | 15/15 | 12/12 |
| full | Haiku 5.5 | 30/30 | 0/30 | 15/15 | 15/15 | 12/12 |
| grammar | Sonnet 5.5 | 30/30 | 0/30 | 15/15 | 15/15 | 12/12 |
| grammar | Haiku 5.5 | 29/30 (one `clarify` on `k10_decoy_11_pm`) | 0/30 | 15/15 | 15/15 | 12/12 |

## Reading

Unlike sets 6 and 7, the baseline shows **no silent failure**: no model wrote a guessed `wait for` or dropped a time
condition. RoboLang 1.9 therefore adds coverage, making requests expressible that are declined today correctly. It
does not fix a silent failure. The bounds have to guard against the reverse risk: that the new clock readings
*invite* false programs. The main candidate is "at 08:00" written as a wait that is already true when the task
starts at 10:30 (clarification K3 in 0042).
