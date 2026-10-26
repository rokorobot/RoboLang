# RoboLang simulation — semantics, world file 1.0 and run record 1.0

**Status:** RL-BASIC-01a specification, **proposed**. Every choice in it is listed for owner
decision in [decision record 0004](decisions/0004-rl-basic-01a-simulation-semantics.md); none
is approved.
**Milestone:** RL-BASIC-01, as approved in
[decision record 0003](decisions/0003-language-foundation.md) (F4) and defined in
[RL-FOUNDATION-01 section 6](architecture/rl-foundation-01-language-foundation.md#6-rl-basic-01--functional-simulation-executor).
**Implementation:** none. RL-BASIC-01b implements this specification only after it is
approved and the owner separately authorizes implementation.
**Language:** RoboLang 1.1 ([`robolang-v1.1.md`](robolang-v1.1.md)), unchanged.
**IR:** Task IR 1.0 ([`ir-schema-v1.0.md`](ir-schema-v1.0.md)), unchanged.
**Fixtures:** [`tests/fixtures/simulation/`](../tests/fixtures/simulation/README.md)

> **Simulation only.** This document defines what a RoboLang 1.1 task does in a symbolic,
> deterministic simulation. A run that ends `SUCCEEDED` says that the task's steps were
> consistent with a declared simulated world. It says nothing about how a real robot would
> move, whether a task is safe to run on hardware, or whether any safety limit was enforced.
> No part of this specification connects to ROS 2, an adapter or hardware.

---

## 1. Scope

Defined here:

- the `robolang simulate` command: its inputs, processing order, exit status and output
  (section 2);
- the world file, schema 1.0 (section 3);
- what each RoboLang 1.1 construct does in simulation (section 4);
- the run record, schema 1.0 (section 5), and how its run ID is derived (section 6);
- the stable codes for world-file errors and run failures (section 7);
- how the acceptance tests T1–T19 map onto fixtures (section 8).

Not defined here: execution of externally supplied IR documents, ROS 2 or any adapter,
physical execution, RoboLang Core schemas, A2R and R2R, concurrency, new `.rob` syntax, motion
planning, kinematics, collisions and speed (section 4.8).

## 2. Running a simulation

### 2.1 Command

```text
robolang simulate FILE.rob --world WORLD.json [--task NAME] [--seed SEED]
```

- `--task NAME` selects the task. A file with exactly one task may omit it. A file with
  several tasks and no `--task` is a usage error. Running all tasks is never a default
  (`robolang-v1.1.md` section 9).
- `--seed SEED` is a string of 1 to 64 characters from `A–Z a–z 0–9 . _ -`. The default is
  `0`. The seed only distinguishes runs of identical inputs (section 6); the simulation has
  no random behavior.

`simulate` is a separate command. `validate` and `inspect` are unchanged and still never
execute anything.

### 2.2 Processing order

1. Check the arguments, including the seed.
2. Read `FILE.rob` and run the complete existing front end on it (`compile_source`): lexing,
   parsing and static validation of the whole file. Warnings are printed and do not stop.
3. Select the task.
4. Read `WORLD.json` and validate it against section 3, including the bindings for the
   selected task.
5. Create the run, execute it (section 4) and write the run record (section 5).

Each step runs only if the previous ones succeeded. In particular, nothing is executed for a
source file with any error, anywhere in the file (`robolang-v1.1.md` section 1).

### 2.3 Exit status

| Status | Meaning | Run record |
|---|---|---|
| 0 | The run ended `SUCCEEDED` | Written |
| 1 | The source file is not valid RoboLang 1.1 | None |
| 2 | Usage error: bad arguments or seed, unreadable file, unknown task name, several tasks and no `--task` | None |
| 3 | The world file is not valid (section 3.9) | None |
| 4 | The run ended `FAILED` | Written |
| 5 | The run ended `CANCELLED` | Written |

Statuses 0, 1 and 2 keep the meaning they have for `validate` and `inspect`.

### 2.4 Output

- **stdout:** the run record (section 5), only for statuses 0, 4 and 5.
- **stderr:** front-end warnings and diagnostics, world-file diagnostics, and for a run one
  summary line that begins with `simulation only:` and names the run ID, the final state and
  the failure code if there is one.

World-file diagnostics have the form

```text
<file>: error[E503] at /robots/arm1/colour: unknown field 'colour'
```

where the location is a JSON Pointer (RFC 6901) into the world document. A validator may stop
at the first error. Each invalid world fixture contains exactly one error.

## 3. World file, schema 1.0

### 3.1 Document

A world file is a UTF-8 JSON document (RFC 8259) with one object at the top level:

```json
{
  "world_schema": "robolang.sim.world",
  "world_schema_version": "1.0",
  "locations": ["home", "shelf", "table"],
  "regions": { "cell": { "clear": true } },
  "robots": {
    "arm1": {
      "ready": true,
      "location": "home",
      "gripper": true,
      "max_speed": [{ "value": "1", "unit": "m/s" }]
    }
  },
  "objects": { "box1": { "present": true, "location": "shelf" } },
  "sensors": { "camera_top": { "robot": "arm1", "available": true } },
  "events": [],
  "tasks": {
    "demo_pick_and_place": {
      "bindings": { "r": "arm1", "box": "box1", "src": "shelf", "dst": "table", "cell": "cell" }
    }
  }
}
```

| Field | Required | Content |
|---|---|---|
| `world_schema` | yes | Exactly `"robolang.sim.world"` |
| `world_schema_version` | yes | Exactly `"1.0"` |
| `locations` | yes | Array of location IDs |
| `regions` | no | Object: region ID → region (3.3) |
| `robots` | no | Object: robot ID → robot (3.3) |
| `objects` | no | Object: object ID → object (3.3) |
| `sensors` | no | Object: sensor name → sensor (3.3) |
| `events` | no | Array of scripted events (3.6) |
| `tasks` | yes | Object: task name → task entry (3.5) |

General rules:

- Unknown fields are errors at every level. Duplicate keys in any object are errors.
- A missing optional collection is the same as an empty one.
- JSON numbers appear only as step indices (3.5), which are positive integers written without
  a fraction or exponent. Measured values are quantities (3.7).
- The world file is input, not trusted state: it is validated completely before a run is
  created, and the world it describes exists only in the simulation.

### 3.2 Entities and names

Locations, regions, robots and objects are **entities**. An entity ID has the form of a
RoboLang identifier: an ASCII letter, then ASCII letters, digits or `_`. IDs are unique across
all four kinds, so a binding names exactly one entity. Reserved words are allowed as IDs; IDs
never appear in `.rob` source.

Sensors are not entities. A sensor is named by the non-empty string a task uses in
`inspect … using "…"` and `sensor_ok "…"`. Channels are not declared.

### 3.3 Entity state

State that a world file does not assert is **unknown**. An unknown fact is never treated as
true (section 4.4).

**Region:** `{ "clear": true | false }`. `clear` may be omitted: unknown.

**Robot:**

| Field | Required | Content |
|---|---|---|
| `location` | yes | ID of a declared location |
| `gripper` | yes | `true` if the robot has a gripper |
| `ready` | no | `true` or `false`; omitted means unknown. Readiness for physical actions (4.5) |
| `max_speed` | no | Array of quantities with unit `m/s` or `rad/s`, at most one per unit. The robot's declared maxima (section 4.3) |

**Object:**

| Field | Required | Content |
|---|---|---|
| `present` | no | `true` or `false`; omitted means unknown |
| `location` | no | ID of a declared location |
| `held_by` | no | ID of a declared robot |

An object that is `present: true` has exactly one of `location` and `held_by`. An object that
is not present, or whose presence is unknown, has neither. A robot named in `held_by` has a
gripper, and a robot holds at most one object.

**Sensor:**

| Field | Required | Content |
|---|---|---|
| `robot` | no | ID of the declared robot the sensor is mounted on. Omitted: not mounted on a robot |
| `available` | no | `true` or `false`; omitted means unknown |

### 3.4 Robot profile

In RoboLang 1.1 a robot's capability profile is minimal: `gripper`, its mounted sensors, and
its `max_speed` maxima. There is no reach, workspace, payload or travel model: every robot can
`move` to every declared location.

### 3.5 Task entries

`tasks` maps a task name to an entry for that task. Only the entry for the selected task is
used; it must exist. Entries for other names are validated for form only, so one world file
can serve several tasks.

| Field | Required | Content |
|---|---|---|
| `bindings` | yes | Object: parameter name → entity ID. Every parameter of the task is bound, to an entity of its type (`robot`, `object`, `location`, `region`). No other names appear. Two parameters may be bound to the same entity |
| `faults` | no | Array of `{ "step": N }`: the plan's step N fails when it is reached (4.6). N is a 1-based index into the task's `plan`; each N at most once |
| `cancel` | no | `{ "before_step": N }`: the run is cancelled when step N is reached, before it executes (4.6) |

### 3.6 Events

An event changes one fact at a simulated time:

```json
{ "at": { "value": "5", "unit": "s" }, "fact": "region_clear", "target": "cell", "value": true }
```

| `fact` | `target` |
|---|---|
| `robot_ready` | A robot ID |
| `region_clear` | A region ID |
| `sensor_available` | A declared sensor name |

`at` is a duration (3.7) measured from the start of the run and greater than zero. `value` is
`true` or `false`. Events are applied in order of `at`, and in file order for equal times,
when the simulated clock reaches their time (4.2).

### 3.7 Quantities

A quantity has the IR form (`ir-schema-v1.0.md` section 3.5): `{ "value": "<decimal>",
"unit": "<unit>" }`, with the value an exact decimal string (digits, optional fractional part,
no sign, no exponent). Durations use `ms`, `s` or `min`; speeds use `m/s` or `rad/s`. Event
times and speeds are greater than zero.

### 3.8 Validation

A world file is valid when it satisfies sections 3.1 to 3.7 and the bindings of the selected
task are complete and correctly typed. Validation runs after the task is selected, because the
bindings, fault indices and cancellation index depend on it.

### 3.9 World-file errors

| Code | Meaning |
|---|---|
| `E501` | Not readable as a UTF-8 JSON document, or an object has a duplicate key |
| `E502` | `world_schema` or `world_schema_version` is missing or not supported |
| `E503` | Unknown field |
| `E504` | A required field is missing, or a value has the wrong form (type, ID syntax, quantity, unit, integer) |
| `E505` | Reference to an undeclared location, robot, region, object or sensor |
| `E506` | No entry for the selected task, or a parameter of the selected task is not bound |
| `E507` | A parameter is bound to an entity of the wrong type |
| `E508` | A binding names something that is not a parameter of the selected task |
| `E509` | Duplicate entity ID, within a kind or across kinds |
| `E510` | Inconsistent state (section 3.3): an object's presence and position disagree, an object is held by a robot without a gripper, or a robot holds more than one object |
| `E511` | Invalid script: a fault or cancellation step outside the selected task's plan, a repeated fault step, two `max_speed` maxima with the same unit, or an event whose target is of the wrong kind for its fact |

## 4. Execution semantics

Names in a task are parameters; during a run each one stands for the entity it is bound to.
"Robot `r`" below means the robot bound to parameter `r`.

### 4.1 States

```text
CREATED → VALIDATED → AUTHORIZED → RUNNING → SUCCEEDED | FAILED | CANCELLED
```

| State | Entered when |
|---|---|
| `CREATED` | The source and world file are valid and the run ID is assigned |
| `VALIDATED` | The bindings have been resolved for the selected task |
| `AUTHORIZED` | The safety-limit check of 4.3 passed. The authorization is for simulation only; there is no other mode |
| `RUNNING` | Execution starts at simulated time 0 |
| `SUCCEEDED` | Every precondition was true and every step succeeded |
| `FAILED` | The authorization check, a precondition or a step failed |
| `CANCELLED` | A scripted cancellation was reached |

A run that fails the authorization check goes from `VALIDATED` to `FAILED`. Input errors
(exit statuses 1 to 3) create no run, so every run record starts with `CREATED, VALIDATED`.

### 4.2 Simulated clock

- Time starts at 0 and is held as an **exact decimal number of milliseconds**. Durations
  convert exactly: `ms` × 1, `s` × 1000, `min` × 60000. No floating-point arithmetic is used.
- Only `wait for` and `wait until` advance the clock. Every other step takes zero simulated
  time.
- Whenever the clock advances to time *t*, every event with `at` ≤ *t* that has not been
  applied yet is applied, in event order (3.6). Events later than the end of the run are never
  applied.
- Nothing waits in real time.

### 4.3 Authorization and safety limits

The `simulate` command authorizes this one run in simulation mode. Before the run reaches
`AUTHORIZED`, each `@safety max_speed` limit of the task is examined in source order:

- A limit **without a unit** (`W401`) is not compared with anything. Its status is
  `unit_unspecified_not_enforced`.
- A limit **with a unit** is compared with every robot bound to a robot parameter of the
  task, in parameter order. If a robot declares no maximum for that unit, the run fails with
  `S002`. If the limit is greater than the robot's maximum, it fails with `S001`. Otherwise
  the limit's status is `checked_against_profile_not_enforced`.

This is a configuration check. No speed is simulated, so **no limit is ever enforced**, and
no run record reports a limit as enforced.

### 4.4 Conditions

A condition evaluates to `true`, `false` or `unknown`:

| Condition | Value |
|---|---|
| `robot_ready r` | The robot's `ready` fact |
| `region_clear c` | The region's `clear` fact |
| `object_exists o` | The object's `present` fact |
| `sensor_ok "s"` | The sensor's `available` fact; `unknown` if no sensor `s` is declared |

A fact that is not asserted is `unknown`. Only `true` lets a run continue.

**`pre` block.** At time 0, after `RUNNING` is entered, every condition of the `pre` block is
evaluated, in order, and every result is recorded. If any result is not `true`, the run
fails before the first step: with `S101` if the first such result is `false`, with `S102` if
it is `unknown`.

### 4.5 Statements

Each step first checks for a scripted fault (4.6), then its requirements in the order listed,
and fails with the first unmet requirement. A step whose requirements are met has its effect
and succeeds.

**Readiness.** A robot's `ready` fact, and the `robot_ready` condition, mean that the robot
is ready for **physical actions**: `move`, `grasp` and `place`. It is not a statement about
every subsystem of the robot. `inspect` depends on its sensor's own `available` fact instead,
and `communicate` only records a simulated message. That a simulated `inspect` or
`communicate` succeeds for a robot that is not ready does not imply that real sensing or
communication hardware would work on such a robot.

| Statement | Requirements, in order | Effect |
|---|---|---|
| `move r to dst` | `r` is ready (`S201` false, `S202` unknown) | `r` is at `dst`. An object `r` holds moves with it |
| `grasp r o` | `r` is ready (`S201`, `S202`); `r` has a gripper (`S211`); `r` holds nothing (`S212`); `o` is present (`S213` false, `S214` unknown); `o` is at the location of `r` (`S215`) | `r` holds `o`; `o` has no location |
| `place r o at dst` | `r` is ready (`S201`, `S202`); `r` holds `o` (`S221`); `r` is at `dst` (`S222`) | `o` is at `dst`; `r` holds nothing |
| `inspect r o using "s"` | Sensor `s` is declared and mounted on `r` (`S231`); `s` is available (`S232` false, `S233` unknown) | An observation of `o` is recorded: its `present`, `location` and `held_by` as they are at that moment. The object's state does not make `inspect` fail |
| `wait for d` | — | The clock advances by `d` |
| `wait until c timeout d` | See below | See below |
| `communicate r to "ch" with "m"` | — | The message is recorded with status `recorded`. Delivery and acknowledgement are not modelled or claimed |

**`wait until c timeout d`.** Let *t₀* be the time the step starts. If `c` is `true` at *t₀*,
the step succeeds at once. Otherwise the clock advances through the event times in
(*t₀*, *t₀* + `d`], applying events and evaluating `c` at each distinct time; the step
succeeds at the first time `c` is `true`. If there is none, the clock is set to *t₀* + `d` and
the step fails with `S241`. The bound is inclusive: an event exactly at *t₀* + `d` that makes
`c` true lets the step succeed. The last value of `c` is recorded.

### 4.6 Failure, faults and cancellation

- A failing step ends the run `FAILED` at that step. Later steps do not run and are recorded
  as `not_run`. No step is ever skipped.
- A scripted **fault** for step N makes step N fail with `S251` when it is reached, before its
  requirements are checked and without its effect. A fault for a step that is never reached
  has no effect.
- A scripted **cancellation** before step N ends the run `CANCELLED` when step N is reached,
  before it executes. Step N and later steps are `not_run`. A cancellation that is never
  reached has no effect.

### 4.7 Determinism

A run is a function of the source file, the world file, the selected task and the seed. It
uses no wall-clock time, no random numbers and no state outside its inputs.

### 4.8 What the simulation does not model

Positions are symbolic locations. There is no motion planning, path, kinematics, reach,
payload, collision, speed or acceleration, and no physical time for actions. Motion planning
and speed enforcement remain RL-12B and RL-12C work.

## 5. Run record, schema 1.0

### 5.1 Document

```json
{
  "run_record_schema": "robolang.sim.run",
  "run_record_schema_version": "1.0",
  "mode": "simulation",
  "run_id": "sim-…",
  "inputs": { … },
  "task": { … },
  "bindings": { … },
  "states": ["CREATED", "VALIDATED", "AUTHORIZED", "RUNNING", "SUCCEEDED"],
  "state": "SUCCEEDED",
  "failure": null,
  "cancelled_before_step": null,
  "safety_limits": [ … ],
  "preconditions": [ … ],
  "steps": [ … ],
  "events_applied": [ … ],
  "end_t_ms": "0",
  "final_world": { … }
}
```

| Field | Content |
|---|---|
| `run_record_schema`, `run_record_schema_version` | `"robolang.sim.run"`, `"1.0"` |
| `mode` | Always `"simulation"` |
| `run_id` | Section 6 |
| `inputs` | `source_sha256`, `world_sha256`, `task`, `seed`, `language_version`, `ir_schema_version`, `world_schema_version` |
| `task` | The selected task in its IR form (`ir-schema-v1.0.md` section 3.1) |
| `bindings` | Parameter name → entity ID, in parameter order |
| `states` | Every state the run entered, in order |
| `state` | The final state |
| `failure` | `null`, or the object of 5.2 |
| `cancelled_before_step` | `null`, or the step index at which the run was cancelled |
| `safety_limits` | One entry per limit in source order: `limit` (the IR safety limit) and `status`: `unit_unspecified_not_enforced`, `checked_against_profile_not_enforced`, `failed_check` for the limit that failed the run (`S001`, `S002`), or `not_checked` for limits after it |
| `preconditions` | One entry per condition of `pre`: `index` (from 1), `condition` (IR form) and `result`: `true`, `false`, `unknown`, or `not_evaluated` when the run failed authorization |
| `steps` | One entry per plan step: section 5.3 |
| `events_applied` | Every event applied, in the order applied: `t_ms` (the event's `at` in milliseconds), `fact`, `target`, `value` |
| `end_t_ms` | Simulated time at the end of the run |
| `final_world` | Section 5.4 |

### 5.2 Failure

`{ "code", "phase", "step", "precondition", "safety_limit", "robot" }`, all present:

- `code`: a run failure code (section 7.2);
- `phase`: `authorization`, `precondition` or `step`;
- `step`: the failing step's index, or `null`;
- `precondition`: the index of the first condition that was not `true`, or `null`;
- `safety_limit`: the index of the failing limit (from 1), or `null`;
- `robot`: the robot entity concerned in an authorization failure, or `null`.

### 5.3 Steps

`{ "index", "step", "status", "t_start_ms", "t_end_ms", "observation", "message", "condition_result" }`,
all present:

- `index` from 1; `step` in IR form (`ir-schema-v1.0.md` section 3.4);
- `status`: `succeeded`, `failed` or `not_run`;
- `t_start_ms`, `t_end_ms`: simulated times, `null` for `not_run`;
- `observation`: for a completed `inspect`, `{ "object", "present", "location", "held_by" }`
  with entity IDs and `null` for unknown or absent; otherwise `null`;
- `message`: for a completed `communicate`, `{ "robot", "channel", "message", "status": "recorded" }`;
  otherwise `null`;
- `condition_result`: for a `wait until` that ran, the last value of its condition; otherwise
  `null`.

### 5.4 Final world

The world state at the end of the run, with entities sorted by ID (Unicode code point order):

```json
{
  "robots":  { "<id>": { "ready": true, "location": "<id>", "holding": null } },
  "objects": { "<id>": { "present": true, "location": "<id>", "held_by": null } },
  "regions": { "<id>": { "clear": true } },
  "sensors": { "<name>": { "robot": "<id>", "available": true } }
}
```

Unknown facts and absent positions are `null`.

### 5.5 Times and formatting

- Times (`t_ms`, `t_start_ms`, `t_end_ms`, `end_t_ms`) are exact decimal strings of
  milliseconds in the canonical form of `ir-schema-v1.0.md` section 3.5 (`"2500"`, `"0.5"`,
  `"0"`).
- Key order is fixed as shown in this section. Lists keep the order stated.
- The JSON text follows the IR determinism rules (`ir-schema-v1.0.md` section 4): two-space
  indentation, ASCII only, LF line endings and one trailing newline, on every platform.
- The record holds codes and structured values only, no free-text messages, so it does not
  change when diagnostic wording changes.

The same inputs therefore always produce a byte-identical run record.

## 6. Run ID

```text
normalize(bytes)  = bytes without a leading UTF-8 byte order mark (EF BB BF),
                    with every CR LF pair replaced by LF
source_sha256     = lowercase hex SHA-256 of normalize(source file bytes)
world_sha256      = lowercase hex SHA-256 of normalize(world file bytes)
preimage          = "robolang-sim-run/1.0\n" + source_sha256 + "\n" + world_sha256 + "\n"
                    + task name + "\n" + seed + "\n"          (ASCII)
run_id            = "sim-" + lowercase hex SHA-256 of preimage
```

- The normalization makes the ID the same on every platform, whatever line endings a
  checkout uses. It does not change any other byte: a change to a comment or to the world
  file's formatting gives a different ID.
- The same source, world, task and seed always give the same ID; a different seed gives a
  different ID.
- The ID identifies an **input-equivalent simulation run**, not an occurrence. Running the
  same inputs twice with the same seed intentionally produces the same ID and the same
  record. It is not an invocation identifier; a future operational executor may need a
  separate one for each invocation.
- The ID identifies a simulated run. It is not a signature and proves nothing about who
  produced a record (`ir-schema-v1.0.md` section 2).

`tests/fixtures/simulation/cases.json` contains test vectors for this derivation.

## 7. Codes

### 7.1 World-file errors

Section 3.9 (`E501`–`E511`). They stop processing with exit status 3; no run is created.

### 7.2 Run failures

| Code | Phase | Meaning |
|---|---|---|
| `S001` | authorization | A `max_speed` limit is greater than a bound robot's declared maximum for that unit |
| `S002` | authorization | A bound robot declares no maximum for the unit of a `max_speed` limit |
| `S101` | precondition | A `pre` condition is false |
| `S102` | precondition | A `pre` condition is unknown |
| `S201` | step | The robot is not ready |
| `S202` | step | The robot's readiness is unknown |
| `S211` | step | `grasp`: the robot has no gripper |
| `S212` | step | `grasp`: the robot already holds an object |
| `S213` | step | `grasp`: the object is not present |
| `S214` | step | `grasp`: the object's presence is unknown |
| `S215` | step | `grasp`: the object is not at the robot's location |
| `S221` | step | `place`: the robot does not hold the object |
| `S222` | step | `place`: the robot is not at the target location |
| `S231` | step | `inspect`: the sensor is not declared or not mounted on the robot |
| `S232` | step | `inspect`: the sensor is not available |
| `S233` | step | `inspect`: the sensor's availability is unknown |
| `S241` | step | `wait until`: the timeout expired before the condition was true |
| `S251` | step | A scripted fault |

## 8. Acceptance tests and fixtures

The acceptance tests are those of RL-FOUNDATION-01 section 6.4, made concrete by the cases in
[`tests/fixtures/simulation/cases.json`](../tests/fixtures/simulation/cases.json). Its README
describes the format.

| Test | Fixture cases |
|---|---|
| T1 Happy path | `t01_pick_and_place` |
| T2 Invalid sources | `t02_invalid_sources` (every file in `tests/fixtures/invalid/`) |
| T3 Precondition false | `t03_region_not_clear` |
| T4 Unknown condition | `t04_region_unknown`, `t04_robot_ready_unknown` |
| T5 Missing capability | `t05_no_gripper`, `t05_robot_not_ready` |
| T6 Object elsewhere | `t06_object_elsewhere` |
| T7 Place without holding | `t07_place_not_held`, `t07_place_wrong_location` |
| T8 Sensor | `t08_sensor_undeclared`, `t08_sensor_on_other_robot`, `t08_sensor_unavailable`, `t08_sensor_unknown`, `t08_inspect_ok` |
| T9 `wait until` | `t09_event_before_timeout`, `t09_timeout`, `t09_event_at_deadline`, `t09_already_true`, `t09_event_in_ms` |
| T10 `communicate` | `t01_pick_and_place` (message assertions) |
| T11 Determinism | `t11_seed_0`, `t11_seed_1` (each run twice; byte-identical records) |
| T12 Task selection | `t12_no_task_selected`, `t12_task_selected` |
| T13 Bindings | `t13_parameter_unbound`, `t13_wrong_type`, `t13_undeclared_entity`, `t13_extra_binding` |
| T14 World-file form | `t14_unknown_field`, `t14_unsupported_version` |
| T15 Injected fault | `t15_fault_step_3` |
| T16 Cancellation | `t16_cancel_before_step_2` |
| T17 Safety-limit check | `t17_limit_above_profile`, `t17_unit_not_in_profile`, `t17_limit_within_profile` |
| T18 Isolation | No fixture: RL-BASIC-01b adds a test that the executor imports no ROS 2 or adapter module, and checks `mode` and every safety-limit status in all records produced by the cases above |
| T19 Regression | No fixture: the 76 RoboLang 1.1 fixtures and the existing test suite pass unchanged |

Until RL-BASIC-01b exists, `tests/test_simulation_fixtures.py` checks only that the fixtures
are consistent with this specification and with the existing front end; it executes nothing.

## 9. Decisions

Every choice above is proposed in
[decision record 0004](decisions/0004-rl-basic-01a-simulation-semantics.md). The five choices
deferred by decision record 0003 section 3 are rows B1–B5 there.
