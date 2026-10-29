# RoboLang Task IR — schema 1.3

**Schema identifier:** `robolang.task-ir`
**Schema version:** `1.3`
**Produced from:** RoboLang language version `1.4` ([specification](robolang-v1.4.md))
**Builds on:** [Task IR schema 1.2](ir-schema-v1.2.md) and the schemas below it, unchanged.
Implemented in `src/robolang/ir.py` and read back by `src/robolang/ir_reader.py`.

Schema 1.3 is schema 1.2 plus two step kinds, `if` and `fail`. Everything not stated here is as in schemas 1.0 to 1.2.

## 1. Version

- A 1.4 source produces `"ir_schema_version": "1.3"` and `"language_version": "1.4"`.
- A 1.1, 1.2 or 1.3 source still produces schemas 1.0, 1.1 and 1.2, byte-identical to before.
- A consumer of an older schema must reject an unknown `ir_schema_version` and unknown step kinds.

## 2. Task

`goal` is always present, as in 1.2, but may be `[]`: the 1.4 task `goal` block is optional. A task with an empty `goal`
has a goal condition on every path that does not end in `fail`, from the `goal_then` and `goal_else` of its `if` steps.

## 3. Step kinds

Added to the step table of 1.0 section 3.4:

| `kind` | Other fields |
|---|---|
| `if` | `condition`: a condition object; `then`: non-empty array of steps; `else`: array of steps, empty when the source has no `else`; `goal_then`: non-empty array of conditions or `null`; `goal_else`: non-empty array of conditions or `null` |
| `fail` | `reason`: non-empty string |

Key order is `kind`, then the fields as listed. All six `if` keys are always written.

- The `condition` is evaluated once, when the step is reached. `then` and `else` are steps, so `if` nests. `else if`
  is an `else` that holds exactly one `if` step, and is not distinguishable from `else { if ... }`.
- `goal_then` and `goal_else` are the goal conditions the branch adds to the task `goal`. `goal_else` is `null` when
  `else` is empty.
- Nesting of `if` steps is at most 4 deep.
- `fail` is the last step of the `then` or `else` of an `if`, never a top-level step and never followed by a step.

## 4. Consumer duties

The duties of the earlier schemas apply. A schema 1.3 consumer also rejects: a missing `goal` key; an `if` or `fail`
that breaks the rules of section 3; a task whose `goal` is empty and that has a path with no `goal_then` or
`goal_else` and no `fail`; and, inside branches, everything it rejects at the top level of the plan. The
`ir_reader` codes are `I123` (nesting), `I214` (`fail` position) and `I209` (a path with no goal).
