# RoboLang Task IR — schema 1.10

**Schema identifier:** `robolang.task-ir`
**Schema version:** `1.10`
**Produced from:** RoboLang language version `2.2` ([specification](robolang-v2.2.md))
**Builds on:** [Task IR schema 1.9](ir-schema-v1.9.md) and the schemas below it, unchanged.
Implemented in `src/robolang/ir.py` and read back by `src/robolang/ir_reader.py`.

Schema 1.10 is schema 1.9 plus four more measured kinds (decision [0044](decisions/0044-more-measured-kinds.md)). It adds no step
and no condition. Everything not stated here is as in schemas 1.0 to 1.9. Schema versions are ordered as written, not as numbers:
`1.10` comes after `1.9`.

## 1. Version

- A 2.2 source **with tasks only** produces `"ir_schema_version": "1.10"` and `"language_version": "2.2"`.
- A 2.2 source **with a mission** produces a [Mission IR 1.2](mission-ir-v1.0.md) document, which holds Task IR 1.10 tasks.
- A 1.1 to 2.1 source still produces schemas 1.0 to 1.9 and Mission IR 1.0 and 1.1, byte-identical to before.

## 2. What changes

- **`reading`** of a `measure` step and of a `live` operand may also be `fill`, `confidence`, `pressure` or `count` (before 1.10:
  `I125`).
- **`object`** of a `measure` or a `live` operand whose `reading` is `fill` or `count` is the name of an `object` **or a
  `location`** parameter (otherwise an `object`, `I204`).
- **`unit`** of a `quantity` operand may also be `%`, `Pa`, `kPa` or `bar` (before 1.10: `I117`). A count's literal has **no `unit`
  key**:

```json
{ "kind": "compare", "left": { "kind": "binding", "name": "n" }, "op": "==", "right": { "kind": "quantity", "value": "0" } }
{ "kind": "compare", "left": { "kind": "binding", "name": "level" }, "op": "<", "right": { "kind": "quantity", "value": "25", "unit": "%" } }
```

  A `quantity` with no `unit` is a count's: its left side is a binding of kind `count` and its `value` is whole, or the document is
  invalid (`I117` for another kind, `I229` for a count that is not whole). The `value` of a `do` step still needs its `unit`.
- The operators `==` and `!=` are allowed with a `count` (K3) as with a `label`; the other kinds take `<`, `<=`, `>` and `>=`
  (`I221`).

## 3. Consumer duties

The duties of the earlier schemas apply. The `ir_reader` also checks, for schema 1.10, the rules of
[RoboLang 2.2](robolang-v2.2.md) section 3 on the IR: kinds and units that fit (`I220`, the form of `E319`), a count's literal (`I229`,
the form of `E328`), and a bare number against a kind with units (`I117`, the form of `E206`).

## 4. Consumers

A consumer that does not know a kind must refuse the task, not ignore it. `delegate` and the ROS 2 backend refuse a task with a
`measure` whatever its kind (`measure_unsupported`, `S509`). `export-goal` is unaffected. Holding the IR authorizes nothing (rule LE-13).
