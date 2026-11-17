# RoboLang Task IR — schema 1.8

**Schema identifier:** `robolang.task-ir`
**Schema version:** `1.8`
**Produced from:** RoboLang language version `1.9` ([specification](robolang-v1.9.md))
**Builds on:** [Task IR schema 1.7](ir-schema-v1.7.md) and the schemas below it, unchanged.
Implemented in `src/robolang/ir.py` and read back by `src/robolang/ir_reader.py`.

Schema 1.8 is schema 1.7 plus clock readings (decision [0042](decisions/0042-clock-readings.md)): one more kind of
step, `read_clock`, and two more kinds of comparison operand, `time` and `clock`. Everything not stated here is as in
schemas 1.0 to 1.7.

## 1. Version

- A 1.9 source produces `"ir_schema_version": "1.8"` and `"language_version": "1.9"`.
- A 1.1 to 1.8 source still produces schemas 1.0 to 1.7, byte-identical to before.

## 2. The step `read_clock`

```json
{ "kind": "read_clock", "robot": "r", "reading": "time", "binding": "now",
  "else": [ { "kind": "fail", "reason": "no clock" } ] }
```

- Keys, in this order: `kind`, `robot`, `reading`, `binding`, then `else` (only when the step has a failure handler, as
  for every handled step of schema 1.4).
- `robot` is the name of a `robot` parameter. `reading` is `"time"` (the time of day) or `"elapsed"` (the time since the
  task started). `binding` is a name, bound as the binding of a `measure` is (schema 1.5): it is visible, compared and
  sent by the same rules, with the kind `reading`.
- There is no `object` and no `sensor`: the clock is not an object. A `measure` of schema 1.5 keeps its keys, and its
  `reading` is never `time` or `elapsed`.

## 3. The operands `time` and `clock`

```json
{ "kind": "time", "value": "22:00:00" }
{ "kind": "clock", "reading": "time" }
```

- **`time`** (a right side): a time of day as the text `HH:MM:SS`, 24-hour, always with seconds: `22:00` and
  `22:00:00` are one IR. `value` is `00:00:00` to `23:59:59`. It compares with a binding of kind `time`.
- **`clock`** (a left side): the live clock, read whenever the `wait_until` that holds it is evaluated: `reading` is
  `"time"` (`clock_time`) or `"elapsed"` (`clock_elapsed`). It stands only in the condition of a `wait_until`.
- A comparison with kind `elapsed` has a `quantity` on the right with a unit `ms`, `s` or `min` (the units of a
  duration); a quantity with a duration unit is allowed in a comparison in schema 1.8 and in no other schema. A `do`
  value never takes a duration unit.

## 4. Consumer duties

The duties of the earlier schemas apply. The `ir_reader` also checks, for schema 1.8, the rules of
[RoboLang 1.9](robolang-v1.9.md) section 3 on the IR: the robot of a `read_clock` is a `robot` parameter (`I203`,
`I204`), the binding rules of schema 1.5 apply to a `read_clock` binding, and the kind rules to the new operands: kinds
that do not match (`I220`), `==` on a time or an elapsed value (`I221`), a comparison in a goal (`I222`), a live clock
outside a `wait_until` (`I223`). Form errors: `I115` for a `read_clock`, `time` or `clock` in a schema older than 1.8;
`I125` for a `reading` that is not a clock kind; `I126` for a time that is not `HH:MM:SS`; `I117` for a unit not allowed
here; `I111` / `I112` for a missing or unknown key.

## 5. Consumers

A consumer that does not know a `read_clock` or a `clock` operand must refuse the task, not ignore it: the ROS 2
backend refuses with `S510` (and `S508` for the live clock), and `delegate` does not delegate a plan with a clock
reading (decision 0042 T9). Holding the IR authorizes nothing (rule LE-13).
