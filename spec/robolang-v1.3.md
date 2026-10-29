# RoboLang v1.3 Language Specification — declared goals, outcome atoms and typed sensors

**Status:** specification and implementation, written together as for 1.2. Every choice is a row
in [decision record 0014](decisions/0014-lang-2-goals-and-typed-sensors.md).
**Syntax version:** 1.3
**Builds on:** [RoboLang 1.2](robolang-v1.2.md) and [RoboLang 1.1](robolang-v1.1.md), unchanged;
[Task IR 1.0](ir-schema-v1.0.md) and [1.1](ir-schema-v1.1.md), unchanged
**Grammar:** [`grammar-v1.3.ebnf`](grammar-v1.3.ebnf)
**IR:** [Task IR schema 1.2](ir-schema-v1.2.md)
**Simulation:** [simulation specification](simulation-v1.0.md) section 10
**Fixtures:** [`tests/fixtures/v1.3/`](../tests/fixtures/v1.3/README.md)

RoboLang 1.3 is RoboLang 1.2 plus four things: a required `goal` block that says what must be
true when the task is done, three atoms that talk about outcomes (`object_at`, `robot_at`,
`holding`), a `sensor` parameter type so that sensors are bound like every other entity, and an
optional `@verify` annotation that says how the goal is checked when it is exported
([`export-goal-v0.1.md`](export-goal-v0.1.md)).
This document states only what differs from 1.2. Everything it does not mention is exactly as in
`robolang-v1.2.md` and `robolang-v1.1.md`, including the core invariant (1.1 section 1).

---

## 1. Why these constructs

The project charter says success is a **verified outcome**, and that execution alone does not
establish success. Until 1.2 a task had only `pre` and `plan`, so `robolang simulate` could report
`SUCCEEDED` when the steps merely ran. The task never said what "done" means.

1. **`goal`** lets the author declare the outcome. A run succeeds only when the goal is verified
   against the final state.
2. **`object_at`, `robot_at`, `holding`** are the facts a goal needs. Before them the only facts
   were readiness, region clearance, presence and sensor health.
3. **`sensor` parameters** remove the one untyped, global name left in the language. Robots,
   objects, locations and regions are typed, bound parameters; sensors were string literals that
   named a world ID directly, so a task could not be checked or re-bound.

## 2. Version and compatibility

### 2.1 Version marker

```rob
robolang 1.3;
```

| File starts with | Read as | Goal, new atoms, `sensor` |
|---|---|---|
| no marker, or `robolang 1.1;` | 1.1, exactly as today | Not available |
| `robolang 1.2;` | 1.2, exactly as today | Not available |
| `robolang 1.3;` | 1.3 | Available (the goal is required) |
| any other version | rejected with `E213` | — |

The new syntax requires the marker. A 1.1 or 1.2 file keeps its meaning, its diagnostics, its
IR and its run record **byte for byte**. A 1.2 front end rejects every 1.3 file with `E213` at the
marker.

### 2.2 Reserved words

In a 1.3 file five more words are reserved:

```text
goal sensor object_at robot_at holding
```

In 1.1 and 1.2 files they remain ordinary names. Moving a file that uses them as names to 1.3 is a
**breaking change for that file**: its author renames them (`E212`, as for every reserved word).
No file in this repository uses them as names.

## 3. Syntax

### 3.1 Grammar

The normative grammar is [`grammar-v1.3.ebnf`](grammar-v1.3.ebnf). The changes to 1.2 are:

```text
task        = "task" IDENT "(" [ param { "," param } ] ")" "{" { annotation } [ pre ] goal plan "}"
annotation  = "@safety" "max_speed" NUMBER [ SPEED_UNIT ] ";"
            | "@verify" "min_confidence" NUMBER ";"
            | "@verify" "max_age" duration ";"
goal        = "goal" "{" condition ";" { condition ";" } "}"
type        = "robot" | "object" | "location" | "region" | "sensor"
atom        = ... | "object_at" IDENT IDENT | "robot_at" IDENT IDENT | "holding" IDENT IDENT
            | "sensor_ok" IDENT
inspect     = "inspect" IDENT IDENT "using" IDENT
```

### 3.2 The goal block

```rob
robolang 1.3;

task demo(robot r, object box, location src, location dst) {
    pre  { robot_ready r; object_at box src; }
    goal { object_at box dst; }
    plan { move r to src; grasp r box; move r to dst; place r box at dst; }
}
```

- `goal` comes after `pre` (if any) and before `plan`. A task has at most one.
- It holds one or more conditions, each ending with `;`. They are an implicit conjunction,
  recorded one by one, exactly as the entries of `pre` are.
- Each entry is any 1.2 condition, combined or not (`and`, `or`, `not`, depth at most 8, `E214`,
  `E215`), over the atoms of 3.3.
- An empty goal, a second goal, a goal after `plan` and a goal before `pre` are `E208`.
- A 1.3 task **without** a goal is a static error, `E309`, reported at the task name. The goal is
  required because "the task is done" must always be stated, not assumed.

### 3.3 Outcome atoms

Usable in `pre`, `goal` and `wait until`, and inside combined conditions.

| Atom | True when |
|---|---|
| `object_at o l` | the object is present, **not held by a robot**, and at location `l` |
| `robot_at r l` | the robot is at location `l` |
| `holding r o` | robot `r` is holding object `o` |

An object held by a robot has no location of its own, so `object_at` is `false` while the object is
held: a task that grasps and moves the box but never places it has not put the box anywhere.

**Three-valued.** Each atom is `true`, `false` or `unknown` (simulation 10.2): `unknown` when the
underlying world fact is unknown. An object whose presence is unknown makes `object_at` and
`holding` on it `unknown`; an object that is not present makes them `false`. A robot's location is
always known, so `robot_at` is never `unknown`.

### 3.4 Sensor parameters

`sensor` is a parameter type, declared like any other:

```rob
task look(robot r, object box, sensor cam, location dst) {
    pre  { sensor_ok cam; }
    goal { robot_at r dst; }
    plan { inspect r box using cam; move r to dst; }
}
```

- `sensor_ok S` and `inspect R O using S` take an **identifier**, which must be a `sensor`
  parameter of the task (`E303` if undeclared, `E304` if it is a parameter of another type).
- A `sensor` parameter is used only there. Using it where a robot, object, location or region is
  expected is `E304`.
- **The 1.2 string form is an error in 1.3.** `sensor_ok "camera_top"` and
  `inspect r o using "camera_top"` are rejected with `E310`, whose message tells the author to
  declare a `sensor` parameter and write it without quotes. The string is read and then rejected
  (not a syntax error), so the message can name the fix.
- A world file binds a `sensor` parameter to a **declared sensor ID** (simulation 10.1).

### 3.5 The `@verify` annotation

```rob
task handover(...) {
    @safety max_speed 0.5 m/s;
    @verify min_confidence 0.95;
    @verify max_age 5s;
    ...
}
```

- `@verify` is an annotation like `@safety`: before `pre`, in any order with `@safety`
  annotations (after `pre` is `E208`). It exists **only in 1.3 files**; in a 1.1 or 1.2 file `@`
  followed by `verify` is `E101`, exactly as before.
- Two keys: `min_confidence` takes a decimal number (no unit); `max_age` takes a duration
  (`ms`, `s`, `min`). Another key is `E207`; a missing or wrong-unit value is `E201`/`E206`.
- They state how an *observation* must support each goal condition when the goal is exported as
  Core success criteria: at least this confident (0 to 1) and at most this old. They do not change
  `simulate`: a simulated run still checks the goal against the simulator's final state.
- Without `@verify`, the exporter uses documented defaults (`min_confidence` 0.9, `max_age` 10 s).

## 4. Static rules

1. Goal conditions are checked like `pre` conditions: rules 3 and 4 of 1.1 section 5 apply to every
   atom at any depth. `object_at` and `robot_at` need a location in their last place; `object_at`
   and `holding` need an object; `robot_at` and `holding` need a robot.
2. A sensor reference is a `sensor` parameter (`E303`, `E304`).
3. A 1.3 task has a goal (`E309`).
4. A string where a sensor parameter belongs is `E310`. This takes precedence over `E307`: the
   empty string `""` is also `E310`.

5. Each `@verify` key is declared at most once per task (`E311`); the first declaration stands.
6. `min_confidence` is from 0 to 1 inclusive (`E312`); `max_age` is greater than zero (`E305`).

Contradictory or duplicated goal conditions are not errors.

## 5. Diagnostics

1.1 section 6 and 1.2 section 5 apply unchanged, with these additions:

| Code | Stage | Meaning |
|---|---|---|
| `E309` | static | A 1.3 task has no `goal` block |
| `E310` | static | A quoted string where a sensor parameter is required |
| `E311` | static | A `@verify` key declared more than once in a task |
| `E312` | static | `@verify min_confidence` above 1 |

And these clarifications, all with existing codes:

| Input | Code |
|---|---|
| Empty goal, a second goal, a goal after `plan`, a goal before `pre` | `E208` |
| `goal`, `sensor`, `object_at`, `robot_at` or `holding` used as a name in a 1.3 file | `E212` |
| `object_at box;` (missing second name) | `E201` |
| `sensor_ok ;` or `using 3` (no sensor parameter) | `E201` |
| Unknown condition in a goal | `E204`; the message lists the 1.3 atoms |
| Unknown parameter type | `E205`; the message lists `sensor` |
| `robolang 1.4;` and every other unsupported version | `E213`; the message lists `1.1, 1.2 and 1.3` |

## 6. Abstract syntax tree

```python
ObjectAt(obj: Name, location: Name, span)
RobotAt(robot: Name, location: Name, span)
Holding(robot: Name, obj: Name, span)
GoalBlock(conditions: tuple[Condition, ...], span)
VerifyAnnotation(key: Name, value: Decimal | Duration, value_span, span)
```

- `ParamType.SENSOR` is added. `Task.goal` is a `GoalBlock`, or `None` before 1.3 (and for the
  `E309` case, which the validator reports).
- `SensorOk.sensor` and `Inspect.sensor` are a `StringLit` in 1.1 and 1.2 and a `Name` in 1.3.
  A 1.3 `StringLit` is kept in the tree so that `E310` can point at it.
- `Task.verify` is a tuple of `VerifyAnnotation`, empty before 1.3 and when none is written.
- `VersionDecl.version` may be `"1.3"`.

## 7. Task IR

A 1.3 source compiles to **Task IR schema 1.2**: see [`ir-schema-v1.2.md`](ir-schema-v1.2.md). It adds
the task fields `goal` and (when written) `verify`, and the condition kinds `object_at`, `robot_at` and `holding`. In schema 1.2
the `sensor` of `sensor_ok` and `inspect` is the **name of a `sensor` parameter**. 1.1 and 1.2
sources still compile to IR 1.0 and 1.1, byte-identical to today.

## 8. Simulation semantics

[Simulation section 10](simulation-v1.0.md#10-robolang-13-goals-and-typed-sensors) is normative.
In short:

1. The `pre` block and the steps run as before.
2. After the last step succeeds, **every goal condition is evaluated against the final state**
   (including events applied up to the final clock) and recorded.
3. All `true` → `SUCCEEDED`. Any `false` → `FAILED` with **`S301`**, phase `goal`. Otherwise
   (some `unknown`, none `false`) → `FAILED` with **`S302`**, phase `goal`.
4. A run that failed or was cancelled earlier records its goal conditions as `not_evaluated`.
5. The run record is **schema 1.2**: schema 1.1 plus a `goal` array after `preconditions` and a final
   `core_outcome` ([`simulation-v1.0.md`](simulation-v1.0.md) section 10.5).

| Code | Phase | Meaning |
|---|---|---|
| `S301` | goal | A goal condition is false: the steps ran but the declared outcome was not reached |
| `S302` | goal | No goal condition is false, but at least one is unknown: the outcome is not verified |

## 9. What 1.3 does not do

- No goal checking before the run (a goal that no plan could reach is not detected statically).
- No goal-driven planning, retries or recovery: the plan is still the plan.
- No change to `@safety`, run-ID derivation or any 1.1/1.2 behavior. The world file gains schema 1.1
  (simulation 3), which `simulate` reads like 1.0.
- No comparisons, branching or repetition.
- No Core schema: a goal becomes Core data only through `export-goal`, and only if it is a plain
  conjunction of outcome atoms.

## 10. Fixtures and acceptance

[`tests/fixtures/v1.3/`](../tests/fixtures/v1.3/README.md) holds:

| Set | Content | Count |
|---|---|---|
| `valid/` | 1.3 files a conforming front end must accept | 12 |
| `invalid/` | 1.3 files it must reject, with code, stage and line | 41 |
| `compat/` | 1.1 and 1.2 files whose result must not change | 8 |
| `simulation/` | Sources, worlds and cases for section 8 | 44 cases |

| # | Test | Expected |
|---|---|---|
| G-T1 | Every `valid/` fixture | Passes with no diagnostics; IR golden snapshot with `language_version` `1.3`, `ir_schema_version` `1.2` and a `goal` |
| G-T2 | Every `invalid/` fixture | Rejected with the stated code and line; no IR |
| G-T3 | Every `compat/` fixture | The stated 1.1 or 1.2 result, unchanged |
| G-T4 | Goal met (`g1_*`, `g8_*`) | `SUCCEEDED`; `goal` all `true`; `examples/pick_and_place_v13.rob` included |
| G-T5 | Goal false (`g2_*`) | `FAILED`, `S301`, phase `goal`; a held object is not `object_at` |
| G-T6 | Goal unknown (`g3_*`) | `S302`; `false` beats `unknown` (`S301`) |
| G-T7 | Outcome atoms in goals (`g4_*`) | `holding`, combined goals, unknown presence |
| G-T8 | Goal sees the final state (`g5_*`) | Events applied up to the final clock count |
| G-T9 | Run ended earlier (`g6_*`) | `goal` entries are `not_evaluated` after `S101`, `S102`, a step failure, a cancellation and `S001` |
| G-T10 | Atoms in `pre` and `wait until` (`g7_*`) | Evaluated as in `pre` and `wait until` of 1.2 |
| G-T11 | Sensor parameters in runs (`s*`) | Resolved through the bindings; `S101`, `S102`, `S231`–`S233`, `S241` as before |
| G-T12 | Sensor binding errors (`w1_*`) | Exit 3 with `E505` or `E507` |
| G-T13 | Regression | Every 1.1 and 1.2 fixture, golden IR snapshot, simulation case and run-ID vector, and the full suite, pass unchanged |

`tests/test_v1_3_fixtures.py` runs G-T1 to G-T12; `tests/test_lang2.py` adds unit tests for each
section. G-T13 is the existing suite, unchanged, apart from the tests that named `robolang 1.3;` as
an unsupported version.

## 11. Implementation status

| Part | Status |
|---|---|
| Version marker, reserved words (section 2) | `lexer.py`, `tokens.py`, `parser.py` |
| Syntax, `@verify`, `E208`, AST (sections 3, 5, 6) | `parser.py`, `nodes.py` |
| Static rules, `E309` to `E312` (section 4) | `validator.py` |
| `export-goal` (`@verify` becomes criteria) | `export_goal.py`, `cli.py`, `bridge.py` |
| Task IR 1.2 (section 7) | `ir.py` |
| Goal evaluation, `S301`, `S302`, run record 1.2, sensor binding (section 8) | `sim/executor.py`, `sim/record.py`, `sim/world.py` |
| Acceptance tests G-T1 to G-T13 (section 10) | Pass in CI |
