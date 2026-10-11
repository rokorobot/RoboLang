# RoboLang Task IR — schema 1.0

**Status:** implemented in `src/robolang/ir.py` (RL-11D).
**Schema identifier:** `robolang.task-ir`
**Schema version:** `1.0`
**Produced from:** RoboLang language version `1.1` ([specification](robolang-v1.1.md))

The task IR is the validated content of a `.rob` file with every source detail removed. It is
what later stages (dry-run, simulation, execution) are meant to consume, so that none of them
reads source text again.

> **The IR is data, not permission.** An IR exists only for a program that passed parsing and
> static validation. That makes it well-formed. It does not make it safe, and holding an IR
> never authorizes anything to move a robot. Nothing in this repository executes an IR.

## 1. Two version numbers

| Field | Meaning | Changes when |
|---|---|---|
| `language_version` | The RoboLang syntax the source was read as (`1.1`) | The language changes |
| `ir_schema_version` | The shape of this document (`1.0`) | Fields are added, removed or change meaning |

They are independent. A future language version may still produce schema 1.0, and the schema
may change without any change to the language. A consumer must check `ir_schema` and
`ir_schema_version` before reading anything else and reject versions it does not know.

Versioning rule for the schema: the first number changes when an existing consumer could
misread the document (a field removed, renamed or redefined); the second when fields are only
added.

## 2. Document

```json
{
  "ir_schema": "robolang.task-ir",
  "ir_schema_version": "1.0",
  "language_version": "1.1",
  "tasks": [
    {
      "name": "pick",
      "parameters": [
        { "name": "r", "type": "robot" },
        { "name": "box", "type": "object" },
        { "name": "shelf", "type": "location" }
      ],
      "safety": [
        { "kind": "max_speed", "limit": { "value": "0.5", "unit": "m/s" } }
      ],
      "preconditions": [
        { "kind": "robot_ready", "robot": "r" },
        { "kind": "object_exists", "object": "box" }
      ],
      "steps": [
        { "kind": "move", "robot": "r", "target": "shelf" },
        { "kind": "grasp", "robot": "r", "object": "box" }
      ]
    }
  ]
}
```

### 2.1 Task

| Field | Content |
|---|---|
| `name` | Task name |
| `parameters` | Declared parameters in source order: `name`, and `type` one of `robot`, `object`, `location`, `region` |
| `safety` | Declared limits in source order. Empty list if none |
| `preconditions` | Conditions of the `pre` block in source order. Empty list if the block is absent or empty |
| `steps` | Statements of the `plan` block in source order. Never empty |

### 2.2 Safety limits

`{ "kind": "max_speed", "limit": <quantity> }`. A limit is a **declaration**. Nothing enforces
it. A `unit` of `null` marks the deprecated unitless form: its unit is unknown, it must not be
assumed or converted, and it must never be treated as an enforced physical limit.

### 2.3 Conditions

| `kind` | Other field |
|---|---|
| `robot_ready` | `robot`: parameter name |
| `region_clear` | `region`: parameter name |
| `object_exists` | `object`: parameter name |
| `sensor_ok` | `sensor`: string |

### 2.4 Steps

| `kind` | Other fields |
|---|---|
| `move` | `robot`, `target` |
| `grasp` | `robot`, `object` |
| `place` | `robot`, `object`, `target` |
| `inspect` | `robot`, `object`, `sensor` (string) |
| `wait_for` | `duration` (quantity) |
| `wait_until` | `condition` (a condition object), `timeout` (quantity) |
| `communicate` | `robot`, `channel` (string), `message` (string) |

`robot`, `object`, `target` and `region` hold the **name of a task parameter**. The IR does not
say which real robot, pose or zone a name stands for; that binding is not defined yet.

### 2.5 Quantities

`{ "value": "<decimal>", "unit": "<unit>" | null }`

- `value` is a **string** holding an exact decimal: digits with an optional fractional part,
  no sign, no exponent. It is never a JSON number, so no reader can turn it into a binary
  float by accident. `2.5s` in the source is `"2.5"`, exactly.
- The value is canonical: insignificant zeros are removed (`0.50` → `"0.5"`, `007` → `"7"`,
  `2.0` → `"2"`).
- `unit` is the unit **as written** (`ms`, `s`, `min`, `m/s`, `rad/s`). No unit is ever
  converted: `500ms` and `0.5s` stay different quantities.

### 2.6 Strings

Sensor, channel and message strings hold the decoded text (escapes resolved).

## 3. Determinism

- Key order is fixed, as shown above. `kind` comes first where present.
- Lists keep source order.
- The JSON text uses two-space indentation, ASCII only (other characters as `\uXXXX`), LF line
  endings and one trailing newline, on every platform.
- The IR depends only on the meaning of the source. Layout, comments, the optional
  `robolang 1.1;` marker and the spelling of a number (`0.5` or `0.50`) do not change it.

The same source therefore always produces byte-identical JSON. Golden snapshots in
`tests/snapshots/ir/` hold the expected output for every valid fixture.

## 4. What the IR leaves out

| Not in the IR | Why |
|---|---|
| Source positions, comments, formatting | The IR describes tasks, not text. Diagnostics come from the front end, before an IR exists |
| Whether the file carried a version marker | Unmarked files are read as 1.1; the result is the same |
| Warnings (`W401`) | Reported by the front end. The condition that causes `W401` is visible as a `null` unit |
| Resolved targets, frames, joint values | Not defined yet (v1.2) |
| Which task to run, run identifiers, authorization | Belongs to an executor, which does not exist |

## 5. Selecting a task

`inspect --task NAME` produces the same document with `tasks` reduced to the selected task.
The whole file is validated first, so a selected task is never shown from a file that has an
error elsewhere. Selecting a task is not running it.
