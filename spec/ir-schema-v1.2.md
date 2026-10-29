# RoboLang Task IR — schema 1.2

**Schema identifier:** `robolang.task-ir`
**Schema version:** `1.2`
**Produced from:** RoboLang language version `1.3` ([specification](robolang-v1.3.md))
**Builds on:** [Task IR schema 1.1](ir-schema-v1.1.md) and [1.0](ir-schema-v1.0.md), unchanged.
Implemented in `src/robolang/ir.py`.

Schema 1.2 is schema 1.1 plus a task `goal`, three condition kinds and a change in what a sensor
name refers to. Everything not stated here, including the trust boundary (1.0 section 2),
quantities, strings and the determinism rules, is exactly as in schemas 1.0 and 1.1.

## 1. Version

- A 1.3 source produces `"ir_schema_version": "1.2"` and `"language_version": "1.3"`.
- A 1.1 source still produces schema 1.0, and a 1.2 source schema 1.1, byte-identical to today.
  Their tasks have no `goal` key.
- A schema 1.0 or 1.1 consumer must reject an unknown `ir_schema_version` and unknown kinds
  (1.0 section 2, duty 1), so it cannot misread a 1.2 document.

## 2. Task

A task gains the field `goal`, between `preconditions` and `steps`:

| Field | Content |
|---|---|
| `goal` | Array of one or more condition objects: the conditions that must hold at the end of the run. Present in every schema 1.2 task, absent from schemas 1.0 and 1.1 |

Key order: `name`, `parameters`, `safety`, `preconditions`, `goal`, `steps`.

## 3. Condition kinds

Added to the table of 1.0 section 3.3 and 1.1 section 2:

| `kind` | Other fields |
|---|---|
| `object_at` | `object`: parameter name; `location`: parameter name |
| `robot_at` | `robot`: parameter name; `location`: parameter name |
| `holding` | `robot`: parameter name; `object`: parameter name |

Key order is `kind`, then the fields as listed. A condition object may be any kind of 1.0, 1.1 or this
table, at any depth, in `preconditions`, `goal` and the `condition` of `wait_until`. The depth rules of
1.1 section 4 apply to `goal` entries too.

## 4. Parameter type `sensor`

`parameters[].type` may be `"sensor"`.

## 5. Sensor references

In schemas 1.0 and 1.1 the `sensor` of a `sensor_ok` condition and of an `inspect` step is a world
sensor ID written as a string in the source. In schema 1.2 it is **the name of a `sensor`
parameter** of the task, resolved through the task's bindings exactly like a robot or object name:

```json
{ "kind": "sensor_ok", "sensor": "cam" }
```

is the IR of `sensor_ok cam` when `cam` is a `sensor` parameter. The JSON shape is unchanged; the
meaning of the value depends on the schema version. A schema 1.2 task is recognised by its `goal`,
which is always present.

## 6. Consumer duties

The five duties of 1.0 section 2 and the additions of 1.1 section 4 apply. For duty 1, a schema 1.2
consumer also rejects a task without a non-empty `goal`, a `goal` entry that is not a valid condition,
a `sensor_ok` or `inspect` whose `sensor` is not a `sensor` parameter of the task, and a `sensor`
parameter used as anything but a sensor reference.
