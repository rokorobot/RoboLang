# Benchmark set 6 "regions": baseline on RoboLang 1.6

**Run:** 2026-11-17, `Benchmark` with `regions: true` on `main`,
[run 44040774000](https://github.com/rokorobot/RoboLang/actions/runs/44040774000). Sonnet 5.5 and Haiku 5.5, full and
grammar-only prompts, 3 repeats, effort medium, 22 tasks ([plan](../plans/bench-6-regions.md), decision
[0039](../../spec/decisions/0039-region-limits.md)). **Cost $0.73** (the tool estimated ~$6.5). The run is not gated.

Under 1.6 every task except the 4 controls expects `cannot / inexpressible`, because no region limit can be written.

| Prompt | Model | Decision correct | False programs | Of which valid and SUCCEEDED | Controls goal correct |
|---|---|---|---|---|---|
| full | Sonnet 5.5 | 63/66 | 3/54 | 3 | 12/12 |
| full | Haiku 5.5 | 66/66 | 0/54 | 0 | 12/12 |
| grammar | Sonnet 5.5 | 62/66 | 4/54 | 4 | 12/12 |
| grammar | Haiku 5.5 | 64/66 | 2/54 | 2 | 12/12 |

By group (full prompt, both models):

| Group | Answers | False programs |
|---|---|---|
| `region` (9 tasks, programs in 1.7) | 54 | 3 (Sonnet: `r04_swap_carts` × 3) |
| `infeasible` (4) | 24 | 0 |
| `inexpressible` (5) | 30 | 0 |
| `control` (4) | 24 | 0 wrong decisions |

## The finding: silently dropping the limit

Every false program is valid and succeeds, so no tool catches it. Each one is the region task with its limit simply
left out:
- full prompt: Sonnet `r04_swap_carts` in all 3 repeats;
- grammar-only prompt: Sonnet `r04` × 2 and `r06_field_clear_or_other_row` × 2; Haiku `r06` × 1 and
  `r09_crate_around_aisle_and_cell` × 1.

On `r04` ("Neither may enter the corridor"), Sonnet wrote two plain `move`s. Its reason was that the robots move
"directly, never touching the corridor". The model treats a `move` as a jump, while on a real robot it is a route.

This is the same kind of failure as the half-programs before 1.4 and the silent substitution before 1.6: a safety
request is lost while every check passes. It is the strongest evidence for 1.7.

## Bounds

The acceptance bounds are proposed with this baseline in hand (rule 6), in [the plan](../plans/bench-6-regions.md).
