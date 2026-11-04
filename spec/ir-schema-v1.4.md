# RoboLang Task IR — schema 1.4

**Schema identifier:** `robolang.task-ir`
**Schema version:** `1.4`
**Produced from:** RoboLang language version `1.5` ([specification](robolang-v1.5.md))
**Builds on:** [Task IR schema 1.3](ir-schema-v1.3.md) and the schemas below it, unchanged.
Implemented in `src/robolang/ir.py` and read back by `src/robolang/ir_reader.py`.

Schema 1.4 is schema 1.3 plus a failure handler on five step kinds (decisions
[0033](decisions/0033-step-failure-handlers.md) and [0034](decisions/0034-no-goal-in-handlers.md)). Everything not stated here is as in schemas 1.0 to 1.3.

## 1. Version

- A 1.5 source produces `"ir_schema_version": "1.4"` and `"language_version": "1.5"`.
- A 1.1 to 1.4 source still produces schemas 1.0 to 1.3, byte-identical to before.

## 2. Failure handler

A `move`, `grasp`, `place`, `inspect` or `wait_until` step may have one more key, after its own fields: `else`, a
non-empty array of steps run when the step fails. A handler has no goal, so there is no `goal_else` key. A step
without a handler has no `else` key. Every `wait_until` step has a handler. A handler counts toward the nesting
limit of 4 together with `if` steps. `fail` is allowed as the last step of a handler. Step numbering
(`ir.walk`) visits a handled step, then the steps of its handler, under the label `<step>.else.<n>`.

## 3. Consumer duties

The duties of the earlier schemas apply, and inside handlers everything rejected at the top level of the plan. The
`ir_reader` codes: `I112` for `else` on a step of schema 1.3 or older or on another step kind, and for any
`goal_else` on a handled step; `I120` for an empty `else`; `I123` for nesting;
`I214` for a `fail` that is not last; `I217` for a `wait_until` without a handler (E316); `I209` for a path with no
goal; `I216` for the holder check.
