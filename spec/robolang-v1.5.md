# RoboLang 1.5 (delta over 1.4)

**Status:** front end, Task IR 1.4 and simulation (run record 1.4) complete in code. The shipped language reference
and the benchmark prompts still teach 1.4: moving them to 1.5 changes benchmark labels and needs its own decision.
Decisions: [0033](decisions/0033-step-failure-handlers.md) (F1 to F8) and [0034](decisions/0034-no-goal-in-handlers.md). Grammar: [grammar-v1.5.ebnf](grammar-v1.5.ebnf).
IR: [schema 1.4](ir-schema-v1.4.md). Simulation: [section 13](simulation-v1.0.md).
Everything not stated here is exactly as in [RoboLang 1.4](robolang-v1.4.md).

## 1. Version

`robolang 1.5;` selects 1.5. A file with this marker is read with the 1.4 rules, plus this document. A 1.4 front end
rejects it with `E213`. Files with no marker or with `1.1` to `1.4` are read, validated and compiled exactly as before.
1.5 adds no reserved word.

## 2. Failure handlers

```
move r to dst else { <statement> ... }
grasp r obj else { ... }
place r obj at dst else { ... }
inspect r obj using s else { ... }
wait until <condition> timeout <duration> else { ... }
```

- A `move`, `grasp`, `place`, `inspect` or `wait until` may end with `else { ... }` instead of `;`. The block, the
  **handler**, runs only when that step fails. `wait for`, `communicate` and `fail` take no handler (`E201`).
- **The failed step changes nothing** (F1): the simulator checks a step's requirements and scripted faults before its
  effect. Time a `wait until` spent waiting has passed, and events up to its timeout have been applied.
- **A handler is a branch without a goal** (F2, [0034](decisions/0034-no-goal-in-handlers.md)): at least one
  statement (`E208`), no `goal` block (`E208`), `fail` only as its last statement (`E313`). It counts as one nesting level, as an `if` does, at most 4 in all (`E216`). Unless it
  ends in `fail`, the plan goes on with the statement after the handled step.
- **Every `wait until` has a handler** (F3): its timeout already says it can fail. A `wait until` with `;` is `E316`.
- **Retry** (F5): a handler may repeat the step, `grasp r box else { grasp r box; }`. That is one bounded retry. A
  retry "until it works" or "as many times as it takes" is still a loop and cannot be written.
- Statements inside a handler may have handlers of their own.

## 3. Goals and paths

A handled step has two sides: **`ok`** (it succeeded, nothing more runs) and **`failed`** (its handler ran). A path
is one choice at each `if` and at each handled step it reaches. Its goal is the task `goal` plus the `goal` blocks of
the `if` branches it passes through, as in 1.4. A handler adds no goal: it either recovers, so the path still reaches
the goal the task states, or cleans up and ends in `fail`. A run never succeeds on a goal other than the one asked
for (decision 0034). Every path must have a goal condition or end in `fail` (`E309`, which names a path such as
`path ok/failed`).

## 4. Holder check

The holder check (`E315`, [0032](decisions/0032-holder-check.md)) follows both sides of a handled step (F8): on the
`failed` side the step had no effect. A grasp that fails on every world is still `E315` when it has a handler: its
handler would always run, and the step would never do what it says.

## 5. Where it runs

`simulate`, `export-goal` and `delegate` accept a task with handlers ([simulation 13](simulation-v1.0.md)).
`validate --world` checks the steps of handlers as it checks the steps of branches. A task with a handler is a
branching task: `delegate` does not take it when it binds two or more robots, so no peer step has a handler (F6). The
ROS 2 backend refuses a handler with `S508` before any motion.

## 6. Diagnostics added or changed

| Code | Meaning |
|---|---|
| `E316` | A `wait until` without a failure handler in a 1.5 file (static) |
| `E201` | Also: `else` after `wait for`, `communicate` or `fail` in a 1.5 file (syntax) |
| `E208` | Also: a `goal` block in a failure handler (syntax, decision 0034) |
| `E216` | Also: a failure handler nested deeper than 4 levels, counting `if` statements and handlers (syntax) |
| `E313` | Also: `fail` that is not the last statement of a handler (static) |
| `E309` | Also: the `ok` or `failed` side of a handled step on a path with no goal (static) |

## 7. Compatibility

Nothing changes for 1.1 to 1.4 files. `E213` now lists `1.1, 1.2, 1.3, 1.4 and 1.5`.
