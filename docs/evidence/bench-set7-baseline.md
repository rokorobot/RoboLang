# Benchmark set 7 "actions": baseline on RoboLang 1.7

**Run:** 2026-11-17, `Benchmark` with `actions: true` on the I2 branch,
[run 44154207584](https://github.com/rokorobot/RoboLang/actions/runs/44154207584). Sonnet 5.5 and Haiku 5.5, full and
grammar-only prompts, 3 repeats, 24 tasks ([plan](../plans/bench-7-actions.md), decision
[0040](../../spec/decisions/0040-declared-actions.md)). **Cost $0.76** (the tool estimated ~$7). Not gated.

Under 1.7 every task except the 4 controls expects `cannot / inexpressible`, because no plan statement runs a declared
action.

| Prompt | Model | Action tasks declined | False programs (all valid and SUCCEEDED) | Infeasible | Inexpressible | Controls |
|---|---|---|---|---|---|---|
| full | Sonnet 5.5 | 30/30 | 0/30 | 15/15 | 15/15 | 12/12 |
| full | Haiku 5.5 | 27/30 | **3/30** | 15/15 | 15/15 | 12/12 |
| grammar | Sonnet 5.5 | 27/30 | **3/30** | 15/15 | 15/15 | 12/12 |
| grammar | Haiku 5.5 | 29/30 | 1/30 | 15/15 | 15/15 | 12/12 |

## The finding: replacing the action silently

All 7 false programs are action tasks with the action left out:

- `a07_print_dry`: 6 of the 7 (Haiku full prompt ×2, grammar-only ×1; Sonnet grammar-only ×3);
- `a09_dock_drone`: 1 (Haiku full prompt).

They look like this:

- **`a07`, "print the layout at level_3_wall, and keep it out of wet_floor".** Haiku wrote the move and the 1.7
  region limit, and said so: *"the language has no print step, so only the move and region limit are
  represented"*. That is a knowing half-program that still passes every check.
- **`a09`, "…then dock it there".** Haiku flew the drone to the workshop and treated arriving as docking.

This is the same kind of failure as the silently dropped region limit before 1.7. Here the missing piece is the
action itself.
