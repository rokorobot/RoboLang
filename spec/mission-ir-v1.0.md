# RoboLang Mission IR — schema 1.0

**Schema identifier:** `robolang.mission-ir`
**Schema version:** `1.0`, `1.1` for RoboLang 2.1, `1.2` for RoboLang 2.2, `1.3` for RoboLang 2.3 and `1.4` for RoboLang 2.4 (below)
**Produced from:** RoboLang language version `2.0` ([specification](robolang-v2.0.md)), for a file that has a mission; version
`1.1` from `2.1` ([specification](robolang-v2.1.md)); `1.2` from `2.2` ([specification](robolang-v2.2.md)); `1.3` from `2.3` ([specification](robolang-v2.3.md)); `1.4` from `2.4` ([specification](robolang-v2.4.md))
**Builds on:** [Task IR schema 1.8](ir-schema-v1.8.md), unchanged: the tasks of a Mission IR document are Task IR 1.8 tasks.
**Mission IR 1.1** is exactly this document with `"ir_schema_version": "1.1"`, `"task_ir_schema_version": "1.9"` and
`"language_version": "2.1"`: its tasks are [Task IR 1.9](ir-schema-v1.9.md) tasks, and its `until` may hold a
`message_received` (decision [0043](decisions/0043-incoming-messages.md) S1). A mission has no other change. A 2.0 file keeps
Mission IR 1.0, byte for byte; the reader reads all.
**Mission IR 1.2** is the same again with `"ir_schema_version": "1.2"`, `"task_ir_schema_version": "1.10"` and
`"language_version": "2.2"`: its tasks are [Task IR 1.10](ir-schema-v1.10.md) tasks (decision [0044](decisions/0044-more-measured-kinds.md)); a
mission has no other change.
**Mission IR 1.3** is the same again with `"ir_schema_version": "1.3"`, `"task_ir_schema_version": "1.11"` and
`"language_version": "2.3"`: its tasks are [Task IR 1.11](ir-schema-v1.11.md) tasks, which may hold the step `choose` (decision
[0045](decisions/0045-choosing-at-run-time.md) C10: each run of a mission chooses afresh); a mission has no other change.
**Mission IR 1.4** is the same again with `"ir_schema_version": "1.4"`, `"task_ir_schema_version": "1.12"` and
`"language_version": "2.4"`: its tasks are [Task IR 1.12](ir-schema-v1.12.md) tasks, which may read the charge, and its `until` may
hold the live `charge` operand (`robot_charge r < 30 %`, decision [0048](decisions/0048-the-robots-own-state.md) S6): it is read after each
`run` line, an unknown value ends the mission `failed`, and it is not a bound by itself (`I226`); a mission has no other change.
Implemented in `src/robolang/ir.py` and read back by `src/robolang/ir_reader.py`.

Task IR describes a task. A **mission** is a layer above tasks (decision [0041](decisions/0041-mission-layer.md) M1), so
it has its own document, which holds the tasks it runs and the missions. Task IR 1.8 itself does not change (M8): a robot
is never sent a mission, only a task, and a consumer of Task IR never needs to know this document.

## 1. Version

- A 2.0 source **with a mission** produces a Mission IR 1.0 document, with `"language_version": "2.0"`.
- A 2.1 source **with a mission** produces a Mission IR 1.1 document, with `"language_version": "2.1"`; a 2.2 source, Mission IR 1.2; a 2.3 source, Mission IR 1.3; a 2.4 source, Mission IR 1.4.
- A 2.0 source **with tasks only** produces a Task IR 1.8 document with `"language_version": "2.0"`: Task IR 1.8 is what
  both 1.9 and 2.0 compile a task to, so a 1.8 reader accepts `"1.9"` and `"2.0"` for it.
- A 1.1 to 1.9 source produces schemas 1.0 to 1.8, byte-identical to before.

## 2. The document

```json
{
  "ir_schema": "robolang.mission-ir",
  "ir_schema_version": "1.0",
  "task_ir_schema_version": "1.8",
  "language_version": "2.0",
  "tasks": [ ... Task IR 1.8 tasks ... ],
  "missions": [ ... ]
}
```

The keys are in this order. `tasks` is a non-empty array of Task IR 1.8 tasks, with the rules of that schema; `missions`
is a non-empty array. A task and a mission cannot share a name.

## 3. A mission

```json
{
  "name": "patrol",
  "parameters": [ { "name": "r", "type": "robot" }, { "name": "quay", "type": "region" } ],
  "repeat": {
    "max_runs": 12,
    "max_duration": { "value": "480", "unit": "min" },
    "until": { "kind": "region_clear", "region": "quay" }
  },
  "runs": [ { "task": "round", "arguments": ["r", "home", "far"] } ],
  "on_failure": { "policy": "retry", "retries": 2 }
}
```

- Keys, in this order: `name`, `parameters` (as in a task: `name`, `type`), `repeat` (only when the source has a
  `repeat`), `runs`, `on_failure`.
- **`repeat`** has, in this order and each only when written: `max_runs` (an integer from 1 to 1000), `max_duration` (a
  quantity in `ms`, `s` or `min`, greater than zero, as `timeout` is), `until` (a condition as in a task: the fact
  atoms, `and`, `or`, `not`, and a `compare` of a `live` or `clock` operand with a `quantity`, `text` or `time`
  operand; never a `binding`). At least one of `max_runs`, `max_duration` and an `until` of the form
  `clock_time >= T` (or `>`), alone or in an `or`, is there (decision 0041 M2, clarification 2). Without `repeat` the
  mission is one run.
- **`runs`** is a non-empty array of `{ "task", "arguments" }`: the name of a task of the document and the names of
  parameters of the mission, in the order of the task's parameters. A mission without `repeat` has exactly one.
- **`on_failure`** is always written: `{ "policy": "stop" }` (the default of the source), `{ "policy": "continue" }` or
  `{ "policy": "retry", "retries": N }` (N an integer from 1 to 100; `retries` only with `retry`).
- Counts are JSON integers, the only numbers the reader accepts. A duration is an exact decimal string, as everywhere.

## 4. Consumer duties

The duties of Task IR apply to the tasks. The `ir_reader` also re-checks, for a mission, the rules of
[RoboLang 2.0](robolang-v2.0.md) section 3: names and parameters are unique (`I201`, `I202`); each `run` names a task of
the document, not a mission, with the right number of arguments (`I227`, the form of `E326`), each a parameter of the
mission (`I203`) of the type of the task's parameter (`I204`); a repeat has a hard bound and the counts are at most 1000
and 100 (`I226`, the form of `E325`) and positive (`I205`); the `until` obeys the kind rules (`I218` to `I223`,
`I220`, `I221`). Form errors: `I113` for the header (the schema `1.0`, `task_ir_schema_version` `1.8`, language `2.0`; or `1.1`, `1.9`, `2.1`; or `1.2`, `1.10`, `2.2`; or `1.3`, `1.11`, `2.3`; or `1.4`, `1.12`, `2.4`); `I110`
for a count that is not an integer; `I111`, `I112` for a missing or unknown key; `I114` for an unknown policy; `I117` for
a duration unit not allowed (there is no hour unit); `I120` for an empty `missions` or `runs`.

## 5. Consumers

A consumer that does not know this schema must refuse the document, not read its tasks alone and ignore the missions:
a mission is the bound and the stop that the approval names (decision 0041 M9). The ROS 2 backend, `export-goal` and
`delegate` refuse a mission (`S514`, `X607`).
