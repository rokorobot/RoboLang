# RoboLang 1.6 (delta over 1.5)

**Status:** front end, Task IR 1.5, simulation (world file 1.2, run record 1.5), `validate --world` and the IR reader
complete in code. The shipped language reference and the benchmark prompts still teach 1.5: teaching 1.6 is the
next step of decision 0036 (V11, V13) and changes benchmark labels.
Decision: [0036](decisions/0036-lang-4-measured-values.md) (V1 to V13, approved 2026-11-09). Grammar:
[grammar-v1.6.ebnf](grammar-v1.6.ebnf). IR: [schema 1.5](ir-schema-v1.5.md). Simulation:
[section 14](simulation-v1.0.md). Everything not stated here is exactly as in [RoboLang 1.5](robolang-v1.5.md).

## 1. Version

`robolang 1.6;` selects 1.6. A file with this marker is read with the 1.5 rules, plus this document. A 1.5 front end
rejects it with `E213`. Files with no marker or with `1.1` to `1.5` are read, validated, compiled and run exactly as
before, byte for byte (Task IR 1.0 to 1.4, run records 1.0 to 1.4).

New reserved words in a 1.6 file: **`measure`** and **`as`**, and no others. The kind names (`temperature`, `mass`,
`distance`, `label`), `of` and the units (`degC`, `K`, `g`, `kg`, `mm`, `cm`, `m`) are recognised by their position
and are not reserved: a location named `m` or an object named `label` stays valid. In a 1.6 file the six comparison
operators `<`, `<=`, `>`, `>=`, `==` and `!=` are tokens; in 1.1 to 1.5 files they are still `E101`.

## 2. Reading a value: `measure`

```
measure r KIND of o using s as NAME;
measure r KIND of o using s as NAME else { <statement> ... }
```

- `r` is a robot parameter, `o` an object parameter, `s` a sensor parameter. `KIND` is one of `temperature`, `mass`,
  `distance`, `label` (any other word is `E217`). `NAME` is a new name, written as a parameter is (not a reserved
  word, `E212`).
- A `measure` takes **one** reading and binds it to `NAME` (section 3). It may end with a 1.5 failure handler
  instead of `;`, with all the rules of [1.5 section 2](robolang-v1.5.md): at least one statement, no `goal`, `fail`
  only last, one nesting level.
- A `measure` that **cannot read fails the step** ([simulation 14.2](simulation-v1.0.md): `S231` to `S236`, `S213`,
  `S214`), changing nothing, so the handler catches it. Without a handler it fails the run.
- Like `inspect`, `measure` does not need the robot to be ready (`robot_ready`).
- `inspect` never reads a value. It records whether an object is present and where; weighing or taking a
  temperature is `measure`.

## 3. Bindings

A binding is a name for one measured value. It is not a parameter and it is not mutable.

1. **Unique.** `NAME` must not be the name of a parameter or of another binding of the task (`E318`). One exception:
   a `measure` inside the failure handler of a `measure` may bind **the same name** again: a retried reading
   (`measure r mass of box using scale as m else { measure r mass of box using scale as m; }`). The retry must read
   the same kind (`E319`).
2. **Visible after its `measure`**, in the same block and in the blocks nested in it, never before. Using a name
   that is not visible, or that is a parameter, where a value is needed, is `E317`.
3. **Branches and handlers do not leak.** A binding made inside an `if` or `else` branch, or inside the handler of
   any step, is not visible after that branch or handler.
4. **The handler of a `measure` does not see its own binding**, and `t` is visible after a handled `measure` only if
   its handler cannot leave without it: every exit of the handler that does not end in `fail` must bind `t` too,
   which is exactly the retry case, at the top level of the handler (not inside an `if` in it). A handler that ends
   in `fail` has no such exit, so `t` is visible after it.
5. A binding keeps its value for the rest of the run. Time passing does not change it; to see the world now, measure
   again under a new name, or read live (section 6).

## 4. Kinds and units

| Kind | Units | Conversion (exact decimal) |
|---|---|---|
| `temperature` | `degC`, `K` | `K` = `degC` + 273.15 |
| `mass` | `g`, `kg` | `kg` = 1000 `g` |
| `distance` | `mm`, `cm`, `m` | `m` = 100 `cm` = 1000 `mm` |
| `label` | none: text (batch numbers, barcodes) | none |

A **quantity literal** is `NUMBER UNIT`: an unsigned decimal, as every number in RoboLang, and a unit. `60 degC`,
`1.2 kg`, `500 mm`. A word that is not a unit after a number is `E206`; a unit of the wrong kind for what it is
compared with is `E319`. Because literals are unsigned, a temperature below zero is written in kelvin (`255 K`).
°F is not a unit: the author converts. Comparing converts exactly; nothing is ever rounded.

## 5. Comparisons

A new condition atom, in an `if` condition or a `wait until` condition, combined with `and`, `or`, `not` and
parentheses as every condition is:

```
t < 60 degC          a binding against a quantity literal of its kind
a <= b               two bindings of the same kind
lot == "LOT-7731"    a label binding against a string, or another label binding
```

- The left side is a binding or a [live reading](#6-live-readings); the right side is a quantity literal, a string
  literal or a binding. A literal on the left (`60 degC > t`) is `E201`: write it the other way round.
- **Quantities** take `<`, `<=`, `>`, `>=` only; `==` or `!=` on a quantity is `E320` (with an uncertainty, equality
  is almost never decidable). **Labels** take `==` and `!=` only; ordering a label is `E320`.
- The two sides must be of the same kind: a unit of another kind, a string against a quantity, a quantity against
  a label, or two bindings of different kinds is `E319`.
- A comparison is **not allowed in `pre` or `goal` blocks**, the task's or a branch's (`E321`): goals state facts
  about the world, not values (V1). To use a reading in a goal, `if` on it and `fail` or recover.

## 6. Live readings

```
wait until temperature of pan using probe < 60 degC timeout 10 min else { fail "still hot"; }
```

`KIND of o using s OP Y` is a comparison whose left side is read **at every evaluation of the wait**: when the wait
starts and after every event up to its timeout, as other `wait until` conditions are. Only a `wait until` condition
may hold one; anywhere else a live reading is `E322` (to compare later, `measure` it). A live reading takes no robot:
it has no `r`. If the reading cannot be taken at that moment (the sensor is not declared, not available, does not
measure the kind, the object is absent or has no value, or a contact reading finds no robot at the object), that
comparison is **unknown** and the wait keeps waiting.

## 7. Uncertainty

A quantity reading is `v ± u`, with `u` declared by the sensor. For `t OP c` with a literal `c` (a point):

| Op | true | false | otherwise |
|---|---|---|---|
| `t < c` | `v + u < c` | `v − u ≥ c` | unknown |
| `t <= c` | `v + u ≤ c` | `v − u > c` | unknown |
| `t > c` | `v − u > c` | `v + u ≤ c` | unknown |
| `t >= c` | `v − u ≥ c` | `v + u < c` | unknown |

Two bindings `a OP b` compare the intervals `[v−u, v+u]` the same way: **true only if every pair of values satisfies
the comparison, false only if none does**, unknown otherwise. For example `a < b` is true if `a.v + a.u < b.v − b.u`
and false if `a.v − a.u ≥ b.v + b.u`. Labels are exact: `==` is true or false, never unknown.

An unknown condition behaves as it does today ([1.4](robolang-v1.4.md) section 3): an `if` fails the run with `S261`
before either branch; a `wait until` keeps waiting. So a reading of `59.8 ± 0.5 degC` against `< 60 degC` is `S261`
in an `if`: it may or may not be under the limit. `not` of an unknown stays unknown, and `and` / `or` follow the
three-valued rules of [1.2](robolang-v1.2.md) section 8.

## 8. Sending a value

`communicate r to "ch" with NAME;` sends the binding as a **typed value** (V8): the message record carries
`{subject, kind, value, unit, uncertainty, sensor, t_ms}`, with no `unit` and no `uncertainty` for a label. There is
no string formatting: `"Pan is {t}"` is not a feature, and `communicate r to "ch" with "text";` is unchanged. A name
that is not a visible binding is `E317`.

## 9. What stays out of 1.6

Decision 0036 V1 draws the line, and these are errors, not features: **arithmetic** (an average of three readings:
there is no operator to write), **string formatting** (a message built from a value), **counting**, **clock
readings**, **structured data**, **choosing an object at run time** (`detect`), and **values in goals**. Benchmark
requests `x01_average` and `x02_formatted_message` of set 5 cannot be written: the first needs `+` and `/`, which
the lexer rejects (`E101`); the second needs `{t}` inside a string, which is just text.

## 10. Where it runs

- `simulate` runs 1.6 tasks against world file 1.2 ([simulation 14](simulation-v1.0.md)). A 1.0 or 1.1 world has no
  sensor that measures anything, so every `measure` fails with `S234`.
- `validate --world` reports, for a `measure`, a sensor that is not on the robot (`S231`) or that does not measure the
  kind (`S234`). It does not check contact, which depends on where the robot is, as it does not check `grasp`
  locations; and it does not check live readings, which never fail a run.
- `export-goal` is unchanged: goals have no values.
- `delegate` refuses a task that contains a `measure`, with the plan result **`measure_unsupported`** (not a
  candidate, like `robots_unsupported`). The owner can lift this later.
- The ROS 2 backend refuses a task that contains a `measure` with **`S509`** before any motion.

## 11. Goals and paths

A handled `measure` is a handled step like any other ([1.5 section 3](robolang-v1.5.md)): its `ok` and `failed`
sides are paths, `E309` and the holder check `E315` follow both. A comparison in an `if` is an `if`: its two
branches are two paths, and the three values `true`, `false` and `unknown` map to `then`, `else` and `S261`.

## 12. Rules chosen where decision 0036 is silent

Each is the simplest rule that satisfies 0036; the owner can change any of them.

| # | Rule |
|---|---|
| R1 | `measure` is a handled step: it takes a 1.5 handler (section 2) and counts as a nesting level when it has one (`E216`). |
| R2 | A retry binds the same **kind** as the `measure` it retries, else `E319` (section 3.1). |
| R3 | The left side of a comparison is a binding or a live reading, the right side a literal or a binding; a live reading is never on the right (section 5). |
| R4 | Quantity literals are unsigned; a temperature below zero is written in kelvin (section 4). World property values may be signed (simulation 14.1). |
| R5 | Whether a name is out of scope, used before its `measure`, a parameter, or never bound, is one code, `E317`. |
| R6 | A `measure` checks, in this order: sensor declared and mounted on the robot (`S231`); available (`S232`, `S233`); declares the kind (`S234`); object present (`S213`, `S214`, as for `grasp`); contact, if the sensor's kind has `contact: true` (`S235`); the object has a value for the kind (`S236`). |
| R7 | An object held by the robot counts as at the robot, so a probe can measure what the gripper holds. An object held by another robot is not at this one. |
| R8 | A reading is in the **sensor's** unit: the property is converted exactly to it, and the uncertainty is the sensor's, absolute, in that unit. |
| R9 | A live reading has no robot: a `contact` reading uses the robot the sensor is mounted on; a contact sensor mounted on no robot cannot be read, so it is unknown. |
| R10 | World file 1.2: a `measures` entry has `contact` always, and `unit` and `uncertainty` for every kind but `label`. A label property is a non-empty string. |
| R11 | A `property` event whose target is a declared entity that is not an object is `E511` (like other facts); only an undeclared target is `E515`. |
| R12 | Task IR 1.5 names the measured kind `reading` and the binding `binding`; a `communicate` has `message` or `binding`, not both ([ir-schema-v1.5.md](ir-schema-v1.5.md)). |
| R13 | Run record 1.5: every step has `reading` (null unless a `measure`), every message has `value` (null for text). Live readings are not recorded; the wait's `condition_result` is. |
| R14 | A fault in the world file on a `measure` step gives `S251`, handled like any other step failure. |
| R15 | Comparisons keep their own place in the three-valued logic: `not unknown` is unknown. |

## 13. Diagnostics added or changed

| Code | Meaning |
|---|---|
| `E101` | Also: `=` or `!` alone in a 1.6 file, and any comparison operator in a file before 1.6 (lexical) |
| `E204` | Also: a name at the start of a condition that is not followed by an operator (syntax) |
| `E206` | Also: a number in a comparison with no unit, or with a word that is not a unit (syntax) |
| `E217` | A kind that is not `temperature`, `mass`, `distance` or `label` after `measure` or in a live reading (syntax) |
| `E317` | A name used as a value that is not a visible binding: used before its `measure`, outside the branch or handler that bound it, in the handler of its own `measure`, or a parameter (static) |
| `E318` | A binding with the name of a parameter or of another binding of the task, outside the retry exception (static) |
| `E319` | Kinds that do not match: a unit of another kind, a string against a quantity, two bindings of different kinds, a retry of another kind (static) |
| `E320` | An operator that the kind does not take: `==` or `!=` on a quantity, ordering on a label (static) |
| `E321` | A comparison in a `pre` or `goal` block (static, V1) |
| `E322` | A live reading anywhere but the condition of a `wait until` (static, V4) |
| `E512` | World file 1.2: an unknown kind in `measures`, `properties` or a `property` event |
| `E513` | World file 1.2: a unit that is not a unit of the kind |
| `E514` | World file 1.2: a label with a unit or an uncertainty |
| `E515` | World file 1.2: a `property` event whose target is not a declared entity |
| `S234` | `measure`: the sensor does not measure the kind. Also `validate --world` |
| `S235` | `measure`: a contact reading, and the robot is not at the object |
| `S236` | `measure`: the object has no value for the kind: nothing to read |
| `S509` | ROS 2 backend: a `measure` has no value sensor there; refused before any motion |
| `I124`, `I125` | Task IR 1.5: an unknown comparison operator; an unknown measured kind |
| `I218` to `I223` | Task IR 1.5: `E317` to `E322` restated on IR ([ir-reader-v0.1.md](ir-reader-v0.1.md)) |

`S213` and `S214` now also cover a `measure` of an object that is not present or whose presence is unknown.
`delegate` adds the plan result `measure_unsupported`. `E213` now lists `1.1, 1.2, 1.3, 1.4, 1.5 and 1.6`.

## 14. Compatibility

Nothing changes for 1.1 to 1.5 files, their IR, their run records or their world files 1.0 and 1.1. A world file 1.2
may serve a task of any language version: a `measures` or `properties` it holds is ignored by tasks that do not
measure, and a `property` event is applied and recorded in `events_applied` whatever the task.
