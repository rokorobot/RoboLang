# RoboLang Task IR — schema 1.11

**Schema identifier:** `robolang.task-ir`
**Schema version:** `1.11`
**Produced from:** RoboLang language version `2.3` ([specification](robolang-v2.3.md))
**Builds on:** [Task IR schema 1.10](ir-schema-v1.10.md) and the schemas below it, unchanged.
Implemented in `src/robolang/ir.py` and read back by `src/robolang/ir_reader.py`.

Schema 1.11 is schema 1.10 plus one step, `choose` (decision [0045](decisions/0045-choosing-at-run-time.md)). It adds no
condition. Everything not stated here is as in schemas 1.0 to 1.10. Schema versions are ordered as written, not as numbers:
`1.11` comes after `1.10`.

## 1. Version

- A 2.3 source **with tasks only** produces `"ir_schema_version": "1.11"` and `"language_version": "2.3"`.
- A 2.3 source **with a mission** produces a [Mission IR 1.3](mission-ir-v1.0.md) document, which holds Task IR 1.11 tasks.
- A 1.1 to 2.2 source still produces schemas 1.0 to 1.10 and Mission IR 1.0 to 1.2, byte-identical to before.

## 2. The step `choose`

```json
{
  "kind": "choose",
  "type": "robot",
  "name": "t",
  "source": { "kind": "list", "candidates": ["a", "b"] },
  "where": { "kind": "robot_ready", "robot": "t" },
  "order": { "mode": "nearest", "reference": "handoff" },
  "else": [ { "kind": "fail", "reason": "no tugger is free" } ]
}
```

| Key | Meaning |
|---|---|
| `type` | `robot`, `object` or `location` |
| `name` | the chosen name, a new name (`I202` otherwise). Used like a parameter from the end of the step to the end of its block, and in a `goal` of the plan or branch that holds it at its top level |
| `source` | `{"kind": "list", "candidates": [names]}` (at least one parameter of `type`, in the order written), `{"kind": "objects_at", "location": name}` (`type` is `object`), or `{"kind": "robots"}` (`type` is `robot`) |
| `where` | optional: a condition that names `name` and states world facts only: no `compare`, no `message_received` (`I231`) |
| `order` | optional: `{"mode": "nearest" or "farthest", "reference": name}`, a location, robot or object parameter, never an earlier choice (`I232`) |
| `else` | always there: at least one step, **the last a `fail`** (`I230`) |

The `else` is a failure handler as in schema 1.4: the steps run when no candidate is left. `walk` labels its steps under `.else.`.
A path that reaches the `else` ends in `fail`, so every path has a goal or ends in `fail` as before.

## 3. Consumer duties

The duties of the earlier schemas apply. The `ir_reader` also checks, for schema 1.11, the rules of
[RoboLang 2.3](robolang-v2.3.md) section 8 on the IR (`I202` to `I204`, `I230` to `I232`), and refuses the step `choose` in an older
schema (`I115`).

## 4. Consumers

A consumer that does not know `choose` must refuse the task, not ignore it. `export-goal` and `delegate` accept a task with
`choose` only when the planner can make every choice before the first action, and send the task with the choices filled in
(`X609` otherwise); the ROS 2 backend refuses it (`S508`). Holding the IR authorizes nothing (rule LE-13).
