# Task IR reader and re-validator — version 0.1

**Status:** implemented in `src/robolang/ir_reader.py` (CORE-1); decision record
[0021](decisions/0021-core-1-ir-reader.md).
**Reads:** Task IR schemas [1.0](ir-schema-v1.0.md) to [1.12](ir-schema-v1.12.md), and the Mission IR schemas [1.0 to 1.4](mission-ir-v1.0.md)
**Duties implemented:** 1 and 2 of [ir-schema-v1.0.md](ir-schema-v1.0.md) section 2 (kernel rules K4 and K5
of the domain contracts)

An AI, another program or a person may write a Task IR document directly instead of `.rob` source.
`read_ir` is where that document stops being untrusted *as data*. It does not make the task safe
(duties 3 to 5 of the trust boundary are not implemented here), and a `TaskIR` it returns is not
permission to run anything.

```python
read_ir(data: bytes, filename: str = "<string>") -> TaskIR     # raises IRError
```

The result equals the `TaskIR` the compiler produces for any source with the same meaning, and
`ir.to_json` of it is the canonical text of the document (byte-identical to the compiler's JSON for every
document the compiler produces; the golden snapshots under `tests/snapshots/` are tested this way).

## 1. What is checked, in order

1. **Bytes.** UTF-8, no byte order mark, valid JSON, no duplicate object key, no number with a
   fraction or exponent, no `NaN` or `Infinity`, not nested too deeply for the JSON parser
   (`robolang.core.read_document`, the strict reading of Core 0.1 section 2.3).
2. **Header.** `ir_schema` is `robolang.task-ir`; `ir_schema_version` is `1.0` to `1.5`;
   `language_version` is the language that version is produced from (`1.1` to `1.6`). The version
   fields are claims: they only select the vocabulary below, and everything after them is checked
   against that vocabulary, so a document cannot claim 1.0 and use `and`. An unknown schema version
   stops the reading: nothing else can be interpreted.
3. **Shape of the claimed schema.** Objects have exactly the fields of the schema (none missing, none
   unknown); strings are text without unpaired surrogates; names are legal identifiers that are not
   reserved words of the language version; numbers are exact decimals in strings (`digits[.digits]`,
   at most 64 characters, no sign or exponent; insignificant zeros are accepted and canonicalised);
   units are `ms`, `s`, `min` for durations and `m/s`, `rad/s` or `null` for `max_speed`; kinds and
   parameter types are those the claimed schema has (`and`, `or`, `not` from 1.1; `object_at`,
   `robot_at`, `holding`, the `sensor` type, `goal` and `verify` from 1.2); `and` and `or` have two or
   more operands and no operand of their own kind (canonical grouping); a condition is at most 8
   levels deep (1.1 section 4); a task has at least one step and a document at least one task.
4. **Meaning**, the static rules of the compiler restated on IR (section 3). A task with a form error is
   not checked for meaning; the other tasks are.

Key order and white space in the document do not matter, and are not kept.

## 2. Diagnostics

`IRError.diagnostics` lists every problem found, in document order, at most 50 (`truncated` says there
were more). Each has a stable code, a message and an RFC 6901 JSON Pointer to the value at fault (`""` is
the whole document; a missing field is reported at the object that lacks it). The CLI prints
`FILE: POINTER: error[CODE]: message`.

| Code | Meaning |
|---|---|
| `I101` | Not UTF-8 (or a byte order mark) |
| `I102` | Not JSON (including `NaN`, `Infinity`) |
| `I103` | Duplicate object key |
| `I104` | A number with a fraction or exponent |
| `I105` | Nested too deeply to read |
| `I110` | Wrong JSON type (the message says what was expected) |
| `I111` | Missing field |
| `I112` | Unknown field for the claimed schema version |
| `I113` | Unsupported or inconsistent `ir_schema`, `ir_schema_version` or `language_version` |
| `I114` | Unknown condition, step or safety kind, or parameter type |
| `I115` | A kind or parameter type that the claimed schema version does not have |
| `I116` | Not an exact decimal string |
| `I117` | Unit not allowed here |
| `I118` | Not a legal name, or a reserved word |
| `I119` | String with an unpaired surrogate |
| `I120` | Empty `tasks` or `steps`; `and`/`or` with fewer than two operands |
| `I121` | Condition nested deeper than 8 levels |
| `I122` | `and` directly inside `and`, or `or` inside `or` (grouping not flattened) |
| `I123` | `if` step, or in schema 1.4 a failure handler, nested deeper than 4 levels (E216) |
| `I124` | Schema 1.5: an unknown comparison operator in a `compare` |
| `I125` | Schema 1.5: an unknown measured kind (`reading`); schema 1.8: an unknown clock kind in a `read_clock` or a `clock` operand |
| `I126` | Schema 1.8: a `time` operand that is not the text `HH:MM:SS` (24-hour) |
| `I201` | Duplicate task name (E301) |
| `I202` | Duplicate parameter name (E302) |
| `I203` | Undeclared name (E303) |
| `I204` | Name of the wrong parameter type, including a `sensor` parameter used as anything but a sensor (E304) |
| `I205` | Value not greater than zero: duration, timeout, `max_speed`, `max_age` (E305) |
| `I206` | Two `max_speed` limits with the same unit (E306) |
| `I207` | Empty sensor name (before 1.2), channel or message (E307) |
| `I208` | A unitless `max_speed` combined with another one (E308) |
| `I209` | Schema 1.2 task without a goal, or with an empty one; schema 1.3 task without a `goal` key, or whose empty goal leaves a path with no branch goal and no `fail` (E309) |
| `I210` | Schema 1.2 sensor reference that is not a parameter of the task: a world sensor ID where a name belongs (E310) |
| `I212` | `min_confidence` above 1 (E312) |
| `I213` | An empty `verify` object (the compiler never writes one) |
| `I214` | A `fail` step that is not the last step of an `if` or `else` branch (E313) |
| `I215` | A `fail` reason longer than 200 characters, or with a control character (E314) |
| `I216` | A `grasp` of an object a robot parameter still holds, or by a robot parameter that still holds an object (E315); checked only when the task has no other meaning error in its steps |
| `I217` | Schema 1.4: a `wait_until` step without a failure handler (`else`) (E316) |
| `I218` | Schema 1.5: a name used as a value that is not a visible binding (E317) |
| `I219` | Schema 1.5: a binding named like a parameter or another binding, outside the retry exception (E318) |
| `I220` | Schema 1.5: kinds that do not match: a unit of another kind, a string against a quantity, two bindings of different kinds, a retry of another kind (E319) |
| `I221` | Schema 1.5: an operator the kind does not take: `==` or `!=` on a quantity, ordering on a label (E320) |
| `I222` | Schema 1.5: a `compare` in `preconditions`, `goal`, `goal_then` or `goal_else` (E321) |
| `I223` | Schema 1.5: a `live` operand outside the condition of a `wait_until` (E322) |
| `I224` | Schema 1.6: a region named by more than one region limit (E323) |
| `I225` | Schema 1.6: more than 8 region limits in a task (E324) |
| `I226` | Mission IR 1.0: a `repeat` with no hard bound (no `max_runs`, `max_duration` or time-of-day `until`), or a count over its cap: 1000 runs, 100 retries (E325) |
| `I227` | Mission IR 1.0: a `run` that names no task of the document (unknown, or a mission), or gives it the wrong number of arguments (E326) |
| `I228` | Schema 1.9: a `message_received` in a goal (`goal`, `goal_then` or `goal_else`), however nested (E327) |
| `I229` | Schema 1.10: a count compared with a number that is not whole (E328) |
| `I230` | Schema 1.11: the `else` of a `choose` does not end in a `fail` (E329) |
| `I231` | Schema 1.11: the `where` of a `choose` holds a `compare` or a `message_received`, or does not name the chosen name (E330); schema 1.12 narrows it: a `compare` of the chosen robot's `charge` against a quantity is allowed |
| `I232` | Schema 1.11: a `choose` names a name that an earlier `choose` bound (E331) |

`I211` is not used: a repeated `@verify` key (E311) cannot be written in a JSON object, where it is `I103`.

## 3. Meaning: what is re-checked

Exactly the rules the compiler applies to source (`robolang-v1.1.md` section 5, `robolang-v1.3.md`
section 4), in the same order and with the same "one error per item" behaviour (`validator.py`):

- task and parameter names are unique (the first declaration stands);
- every name a condition or step uses is a declared parameter of the type that position needs;
- durations, timeouts, `max_speed` values and `max_age` are greater than zero; `min_confidence` is from 0 to 1;
- at most one `max_speed` per unit; a unitless `max_speed` is alone;
- channels and messages are not empty; before 1.2 a sensor ID is not empty;
- in schema 1.2 every task has a non-empty goal, and a sensor in `sensor_ok` or `inspect` names a `sensor` parameter.
- in schema 1.5 (RoboLang 1.6, [robolang-v1.6.md](robolang-v1.6.md) sections 3 to 6), by the same code the compiler uses
  (`measured.py`): bindings are visible where they are used, kinds and units match, operators fit the kind, and no
  comparison stands in a goal or a live reading outside a `wait_until` (`I218` to `I223`).
- in schema 1.6 (RoboLang 1.7, [robolang-v1.7.md](robolang-v1.7.md) section 3): a region limit names a `region`
  parameter (`I203`, `I204`), once (`I224`), and a task has at most 8 (`I225`).
- in schema 1.7 (RoboLang 1.8, [robolang-v1.8.md](robolang-v1.8.md) section 3): a `do` and an `in_state` name
  parameters of the right types (`I203`, `I204`), and a state label or a text value is not empty (`I207`).
- in schema 1.8 (RoboLang 1.9, [robolang-v1.9.md](robolang-v1.9.md) section 3): the robot of a `read_clock` is a `robot`
  parameter (`I203`, `I204`), a time literal is `HH:MM:SS` (`I126`), and the binding and kind rules of schema 1.5 extend to
  the two clock kinds (`I218` to `I223`).
- in schema 1.10 (RoboLang 2.2, [robolang-v2.2.md](robolang-v2.2.md) section 3): the four new kinds and the units `%`, `Pa`, `kPa` and
  `bar` are in no older schema (`I125`, `I117`); a `quantity` with no `unit` is a count's (`I117` against another kind, `I229` if not whole);
  the object of a `fill` or `count` reading may be a location (`I204` otherwise).
- in schema 1.11 (RoboLang 2.3, [robolang-v2.3.md](robolang-v2.3.md) section 8): the step `choose` is in no older schema (`I115`); its
  chosen name is new (`I202`), its candidates, place and reference are parameters of the right types (`I203`, `I204`), its `else` is
  there and ends in `fail` (`I230`), its `where` names the chosen name and holds no reading or message (`I231`), and a choice uses no
  name an earlier choice bound (`I232`); a goal may name what a `choose` at the top level of its plan or branch binds.
- in schema 1.12 (RoboLang 2.4, [robolang-v2.4.md](robolang-v2.4.md) section 8): the step `read_charge` and the `charge` operand are in no
  older schema (`I115`, `I114`); the robot of both is a robot parameter (`I203`, `I204`), a charge binding is new (`I219`), a reading of
  kind `charge` in a `measure` is `I125`, a charge is compared in `%` (`I220`) with `<`, `<=`, `>` or `>=` (`I221`), and the `charge`
  operand is live: a `wait_until`, a mission's `until` and a `where` only (`I223`); in a `where` it names the chosen robot and is
  compared with a quantity (`I231` otherwise).
- in schema 1.9 (RoboLang 2.1, [robolang-v2.1.md](robolang-v2.1.md) section 3): no `message_received` in a goal (`I228`), and
  its channel and text are not empty (`I207`; a text of spaces only is empty).

- in Mission IR 1.0 (RoboLang 2.0, [robolang-v2.0.md](robolang-v2.0.md) section 3, [mission-ir-v1.0.md](mission-ir-v1.0.md)
  section 4): the tasks are checked as Task IR 1.8; a mission's name and parameters are unique (`I201`, `I202`), each `run`
  names a task with the right arguments (`I227`, `I203`, `I204`), a repeat has a hard bound and counts in range (`I226`,
  `I205`), and an `until` obeys the kind rules (`I218` to `I223`). A document with a form error in a mission is not
  checked for that mission's meaning; other missions are. Mission IR 1.1 (RoboLang 2.1) is the same with Task IR 1.9 tasks.

Not re-checked, because the IR cannot express a violation: the optional `robolang` marker, `;`, block
structure, the mixing of `and` and `or` without parentheses, the source depth of parentheses.

## 4. What reading does not do

- It does not check the task against a world, a robot or a policy (duties 3 and 4), and it grants no
  authorization (duty 5). `simulate` still loads the world and runs only in simulation.
- It does not prove where the document came from. A run record of IR input says so
  (`inputs.source_kind`, `simulation-v1.0.md` section 5.1); `source_sha256` is then the hash of the IR bytes.
- It emits no warnings. A unitless `max_speed` is visible as `"unit": null`, and is still never treated
  as an enforced limit.

## 5. Command line

`validate`, `inspect`, `simulate`, `export-goal` and `delegate` take `--ir IR` instead of the positional
`FILE`; exactly one of the two is required (otherwise exit status 2). Exit status 1 means the input is not
valid, as for source. `delegate --ir` needs `--plans`: the robot's library is `.rob` files, and the IR
document is only the planner's goal.

```
robolang inspect examples/handover.rob --json > handover.json
robolang validate --ir handover.json
robolang simulate --ir handover.json --world examples/world.json
```
