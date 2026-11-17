# RoboLang Task IR — schema 1.6

**Schema identifier:** `robolang.task-ir`
**Schema version:** `1.6`
**Produced from:** RoboLang language version `1.7` ([specification](robolang-v1.7.md))
**Builds on:** [Task IR schema 1.5](ir-schema-v1.5.md) and the schemas below it, unchanged.
Implemented in `src/robolang/ir.py` and read back by `src/robolang/ir_reader.py`.

Schema 1.6 is schema 1.5 plus region limits (decision [0039](decisions/0039-region-limits.md)): two more kinds of
entry in a task's `safety` array. Everything not stated here is as in schemas 1.0 to 1.5.

## 1. Version

- A 1.7 source produces `"ir_schema_version": "1.6"` and `"language_version": "1.7"`.
- A 1.1 to 1.6 source still produces schemas 1.0 to 1.5, byte-identical to before.

## 2. Region limit entries

```json
"safety": [
  { "kind": "max_speed", "limit": { "value": "0.5", "unit": "m/s" } },
  { "kind": "no_go_region", "region": "wet" },
  { "kind": "in_region", "region": "lane" }
]
```

- `kind` is `no_go_region` or `in_region`; `region` is the name of a `region` parameter of the task. The keys are
  exactly `kind` and `region`, in that order. The entries stay in **source order**, `max_speed` entries among them.
- A region limit **declares** a request: nothing enforces it (rule LE-13). Holding the IR authorizes nothing.
- Schema 1.5 and older have no such entry: `I115` for one in a document of an older schema.

## 3. Consumer duties

The duties of the earlier schemas apply. The `ir_reader` also checks, for schema 1.6, the rules of
[RoboLang 1.7](robolang-v1.7.md) section 3 on the IR: `region` names a parameter (`I203`) of type `region` (`I204`);
`I224` a region named by more than one region limit (E323); `I225` more than 8 region limits (E324). Form errors: `I115`
for a region limit in a schema older than 1.6; `I111` / `I112` for a missing or unknown key (a region limit with a
`limit`, or a `max_speed` with a `region`). The `max_speed` rules (`I205`, `I206`, `I208`) apply to `max_speed` entries
only.

## 4. Consumers

A consumer that does not know a region limit must refuse the task, not ignore the entry: the ROS 2 backend refuses
with `S508`, `export-goal` and `delegate` with `X605` (decision 0039 P8).
