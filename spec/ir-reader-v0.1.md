# Task IR reader and re-validator — version 0.1

**Status:** implemented in `src/robolang/ir_reader.py` (CORE-1); decision record
[0021](decisions/0021-core-1-ir-reader.md).
**Reads:** Task IR schemas [1.0](ir-schema-v1.0.md), [1.1](ir-schema-v1.1.md) and [1.2](ir-schema-v1.2.md)
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
2. **Header.** `ir_schema` is `robolang.task-ir`; `ir_schema_version` is `1.0`, `1.1` or `1.2`;
   `language_version` is the language that version is produced from (`1.1`, `1.2`, `1.3`). The version
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
| `I201` | Duplicate task name (E301) |
| `I202` | Duplicate parameter name (E302) |
| `I203` | Undeclared name (E303) |
| `I204` | Name of the wrong parameter type, including a `sensor` parameter used as anything but a sensor (E304) |
| `I205` | Value not greater than zero: duration, timeout, `max_speed`, `max_age` (E305) |
| `I206` | Two `max_speed` limits with the same unit (E306) |
| `I207` | Empty sensor name (before 1.2), channel or message (E307) |
| `I208` | A unitless `max_speed` combined with another one (E308) |
| `I209` | Schema 1.2 task without a goal, or with an empty one (E309) |
| `I210` | Schema 1.2 sensor reference that is not a parameter of the task: a world sensor ID where a name belongs (E310) |
| `I212` | `min_confidence` above 1 (E312) |
| `I213` | An empty `verify` object (the compiler never writes one) |

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
