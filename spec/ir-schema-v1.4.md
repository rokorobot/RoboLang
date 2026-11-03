# RoboLang Task IR — schema 1.4

**Schema identifier:** `robolang.task-ir`
**Schema version:** `1.4`
**Produced from:** RoboLang language version `1.5` ([specification](robolang-v1.5.md))
**Builds on:** [Task IR schema 1.3](ir-schema-v1.3.md) and the schemas below it, unchanged.
Implemented in `src/robolang/ir.py` and read back by `src/robolang/ir_reader.py`.

Schema 1.4 is schema 1.3 plus a failure handler on five step kinds (decision
[0033](decisions/0033-step-failure-handlers.md)). Everything not stated here is as in schemas 1.0 to 1.3.

## 1. Version

- A 1.5 source produces `"ir_schema_version": "1.4"` and `"language_version": "1.5"`.
- A 1.1 to 1.4 source still produces schemas 1.0 to 1.3, byte-identical to before.

## 2. Failure handler

A `move`, `grasp`, `place`, `inspect` or `wait_until` step may have two more keys, after its own fields, always
together:

| Key | Content |
|---|---|
| `else` | Non-empty array of steps: run when the step fails |
| `goal_else` | Non-empty array of conditions the handler adds to the goal, or `null` |

A step without a handler has neither key. Every `wait_until` step has a handler. A handler counts toward the nesting
limit of 4 together with `if` steps. `fail` is allowed as the last step of a handler. Step numbering
(`ir.walk`) visits a handled step, then the steps of its handler, under the label `<step>.else.<n>`.

## 3. Consumer duties

The duties of the earlier schemas apply, and inside handlers everything rejected at the top level of the plan. The
`ir_reader` codes: `I112` for `else` or `goal_else` on a step of schema 1.3 or older, or on another step kind;
`I111` for one of the two keys without the other; `I120` for an empty `else` or `goal_else`; `I123` for nesting;
`I214` for a `fail` that is not last; `I217` for a `wait_until` without a handler (E316); `I209` for a path with no
goal; `I216` for the holder check.
