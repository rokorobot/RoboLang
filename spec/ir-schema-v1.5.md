# RoboLang Task IR — schema 1.5

**Schema identifier:** `robolang.task-ir`
**Schema version:** `1.5`
**Produced from:** RoboLang language version `1.6` ([specification](robolang-v1.6.md))
**Builds on:** [Task IR schema 1.4](ir-schema-v1.4.md) and the schemas below it, unchanged.
Implemented in `src/robolang/ir.py` and read back by `src/robolang/ir_reader.py`.

Schema 1.5 is schema 1.4 plus measured values (decision [0036](decisions/0036-lang-4-measured-values.md)): a `measure`
step, a `compare` condition, and a `communicate` that sends a binding. Everything not stated here is as in
schemas 1.0 to 1.4.

## 1. Version

- A 1.6 source produces `"ir_schema_version": "1.5"` and `"language_version": "1.6"`.
- A 1.1 to 1.5 source still produces schemas 1.0 to 1.4, byte-identical to before.

## 2. `measure` step

```json
{ "kind": "measure", "robot": "r", "reading": "temperature", "object": "pan", "sensor": "probe", "binding": "t" }
```

Keys in this order, then `else` if the step has a failure handler (a non-empty array of steps, as for the other
handled steps, schema 1.4 section 2). `reading` is the measured kind: `temperature`, `mass`, `distance` or `label`.
`robot`, `object` and `sensor` name parameters of the task of the types `robot`, `object` and `sensor`; `binding` is a
new name (a legal identifier, not reserved in 1.6). `measure` is one of the step kinds that may have a handler:
`move`, `grasp`, `place`, `inspect`, `measure`, `wait_until`.

## 3. `compare` condition

```json
{ "kind": "compare", "left": { "kind": "binding", "name": "t" }, "op": "<",
  "right": { "kind": "quantity", "value": "60", "unit": "degC" } }
```

`op` is one of `<`, `<=`, `>`, `>=`, `==`, `!=`. Operands, each an object with a `kind`:

| `kind` | Other keys | Allowed |
|---|---|---|
| `binding` | `name` | left or right: the name of a binding of an earlier `measure` |
| `quantity` | `value` (exact decimal string, unsigned), `unit` | right: as written in the source, never converted |
| `text` | `value` (a string) | right |
| `live` | `reading`, `object`, `sensor` | left, in the condition of a `wait_until` only: read at every evaluation |

A `compare` may appear in the condition of an `if` and of a `wait_until`, inside `and`, `or` and `not` as any
condition may. It never appears in `preconditions`, in `goal` or in `goal_then` / `goal_else`.

## 4. `communicate` with a binding

```json
{ "kind": "communicate", "robot": "r", "channel": "ops", "binding": "t" }
```

In schema 1.5 a `communicate` has `message` (text, as before) **or** `binding` (the name of a visible binding), not
both and not neither. Schemas 1.0 to 1.4 have `message` only.

## 5. Consumer duties

The duties of the earlier schemas apply, including inside handlers. The `ir_reader` also checks, for schema 1.5, the
rules of [RoboLang 1.6](robolang-v1.6.md) sections 3 to 6 on the IR (all errors, in document order): `I218` a name
used as a value that is not a visible binding (E317); `I219` a binding that clashes with a parameter or another
binding, outside the retry exception (E318); `I220` kinds that do not match, a unit of the wrong kind (E319); `I221` an
operator the kind does not take (E320); `I222` a `compare` in `preconditions`, `goal`, `goal_then` or `goal_else`
(E321); `I223` a `live` operand outside the condition of a `wait_until` (E322). Form errors: `I115` for `measure`,
`compare` or a binding in `communicate` in a schema older than 1.5; `I124` an unknown operator; `I125` an unknown
kind; `I117` a unit that is no unit; `I111` / `I112` a `communicate` with neither or both of `message` and `binding`.
Step numbering (`ir.walk`) is unchanged: a `measure` is one step, followed by the steps of its handler.
