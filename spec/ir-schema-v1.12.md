# RoboLang Task IR — schema 1.12

**Schema identifier:** `robolang.task-ir`
**Schema version:** `1.12`
**Produced from:** RoboLang language version `2.4` ([specification](robolang-v2.4.md))
**Builds on:** [Task IR schema 1.11](ir-schema-v1.11.md) and the schemas below it, unchanged.
Implemented in `src/robolang/ir.py` and read back by `src/robolang/ir_reader.py`.

Schema 1.12 is schema 1.11 plus one step, `read_charge`, and one comparison operand, `charge` (decision
[0048](decisions/0048-the-robots-own-state.md)). It adds no condition. Everything not stated here is as in schemas 1.0 to 1.11.
Schema versions are ordered as written, not as numbers: `1.12` comes after `1.11`.

## 1. Version

- A 2.4 source **with tasks only** produces `"ir_schema_version": "1.12"` and `"language_version": "2.4"`.
- A 2.4 source **with a mission** produces a [Mission IR 1.4](mission-ir-v1.0.md) document, which holds Task IR 1.12 tasks.
- A 1.1 to 2.3 source still produces schemas 1.0 to 1.11 and Mission IR 1.0 to 1.3, byte-identical to before.

## 2. The step `read_charge`

```json
{ "kind": "read_charge", "robot": "r", "binding": "c" }
```

| Key | Meaning |
|---|---|
| `robot` | a robot parameter (`I203`, `I204`): the robot whose charge is read |
| `binding` | the name the charge in `%` is bound to, a new name (`I219`); its kind in the rules of 1.5 is `charge` |
| `else` | optional failure handler, as in schema 1.4 |

It reads no object and no sensor. A robot with no known charge fails the step (`S239`).

## 3. The operand `charge`

```json
{ "kind": "compare", "left": { "kind": "charge", "robot": "r" }, "op": ">=", "right": { "kind": "quantity", "value": "80", "unit": "%" } }
```

`{ "kind": "charge", "robot": NAME }` is allowed as the **left** side of a `compare`, only in the condition of a `wait_until`, in a
mission's `until` and in the `where` of a `choose` (`I223` elsewhere, the live rule of 1.5). `robot` is a robot parameter, or the chosen
robot inside its own `where`. The right side is a quantity in `%` (`I220`) or a binding of kind `charge`; the operator is `<`, `<=`, `>` or
`>=` (`I221`). In a `where` the narrowed rule of 2.4 section 6 applies: the operand names the chosen name and the right side is a
quantity (`I231` otherwise).

## 4. Consumer duties

The duties of the earlier schemas apply. The `ir_reader` also checks, for schema 1.12, the rules of
[RoboLang 2.4](robolang-v2.4.md) section 8 on the IR, and refuses `read_charge` and the `charge` operand in an older schema (`I115`,
`I114`). A reading of `charge` in a `measure` step is `I125`.

## 5. Consumers

A consumer that does not know `read_charge` or `charge` must refuse the task, not ignore it. `export-goal` is unaffected (a goal holds
no binding); `delegate` refuses the task (`measure_unsupported`); the ROS 2 and G1 backends refuse it (`S509`). Holding the IR
authorizes nothing (rule LE-13).
