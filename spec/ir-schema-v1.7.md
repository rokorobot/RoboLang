# RoboLang Task IR — schema 1.7

**Schema identifier:** `robolang.task-ir`
**Schema version:** `1.7`
**Produced from:** RoboLang language version `1.8` ([specification](robolang-v1.8.md))
**Builds on:** [Task IR schema 1.6](ir-schema-v1.6.md) and the schemas below it, unchanged.
Implemented in `src/robolang/ir.py` and read back by `src/robolang/ir_reader.py`.

Schema 1.7 is schema 1.6 plus declared actions (decision [0040](decisions/0040-declared-actions.md)): one more kind of
step, `do`, and one more kind of condition, `in_state`. Everything not stated here is as in schemas 1.0 to 1.6.

## 1. Version

- A 1.8 source produces `"ir_schema_version": "1.7"` and `"language_version": "1.8"`.
- A 1.1 to 1.7 source still produces schemas 1.0 to 1.6, byte-identical to before.

## 2. The step `do`

```json
{ "kind": "do", "robot": "r", "action": "set", "target": "fryer",
  "value": { "kind": "quantity", "value": "175", "unit": "degC" },
  "else": [ { "kind": "fail", "reason": "fryer stuck" } ] }
```

- Keys, in this order: `kind`, `robot`, `action`, `target`, then `value` (only when the source has `to`), then `else`
  (only when the step has a failure handler, as for every handled step of schema 1.4).
- `robot` and `target` are names of parameters (a `robot`; an `object` or a `location`). `action` is a name, not a
  parameter: whether the robot declares it is not part of the IR (the world file decides).
- `value` is an operand of schema 1.5: `{ "kind": "quantity", "value": <exact decimal string>, "unit": <a unit of 1.6> }`
  or `{ "kind": "text", "value": <string> }`.
- A `do` **requests**: it carries no claim that anything happened. Holding the IR authorizes nothing (rule LE-13).

## 3. The condition `in_state`

```json
{ "kind": "in_state", "target": "closet", "label": "open" }
```

`target` is the name of an `object` or `location` parameter; `label` is a non-empty string. It may stand wherever a
condition does (`preconditions`, `goal`, `goal_then`, `goal_else`, the condition of an `if` or a `wait_until`, inside
`and`, `or` and `not`).

## 4. Consumer duties

The duties of the earlier schemas apply. The `ir_reader` also checks, for schema 1.7, the rules of
[RoboLang 1.8](robolang-v1.8.md) section 3 on the IR: the names of a `do` and an `in_state` are parameters of the right
types (`I203`, `I204`), a state label or a text value is not empty (`I207`), the `value` is a quantity with a unit of
1.6 (`I117`) or a text. Form errors: `I115` for a `do` or an `in_state` in a schema older than 1.7; `I111` / `I112` for a
missing or unknown key; `I118` for an action that is not a name.

## 5. Consumers

A consumer that does not know a `do` or an `in_state` must refuse the task, not ignore it: the ROS 2 backend refuses
with `S508`, `export-goal` and `delegate` refuse a goal with `in_state` with `X606` (decision 0040 A8).
