# RoboLang 2.3: choosing at run time

**Status:** front end, Task IR 1.11 (Mission IR 1.3 for a file with a mission), world file 1.8, the simulation (run record
1.11, mission record 1.3), `export-goal` and `delegate`, and the IR reader complete in code. The language reference and the
benchmark prompts teach 2.3 (L4 of decision 0045); the acceptance run on benchmark set 12 is the next step (L5), and until it
passes the bounds B1 to B9 2.3 is not accepted.
Decision: [0045](decisions/0045-choosing-at-run-time.md) (C1 to C12, and the clarifications approved 2026-11-19).
Grammar: [grammar-v2.3.ebnf](grammar-v2.3.ebnf). IR: [Task IR 1.11](ir-schema-v1.11.md) and [Mission IR 1.3](mission-ir-v1.0.md).
Simulation: [section 21](simulation-v1.0.md). Everything not stated here is exactly as in [RoboLang 2.2](robolang-v2.2.md).

## 1. Version

`robolang 2.3;` selects 2.3. A file with this marker is read with the 2.2 rules and may also write the statement `choose`
(section 2). A 2.2 front end rejects the marker with `E213`. Files with no marker or with `1.1` to `2.2` are read, validated,
compiled and run exactly as before, byte for byte; **in them `choose` is still an unknown statement (`E203`).**

**One new statement, no new reserved word, no new top-level declaration.** `choose`, `from`, `where`, `nearest`, `farthest`,
`objects` and `robots` are recognised by their position and are **not reserved** (decision 0045, owner's point 10): a program
of any version that uses them as names still parses. World file 1.8 adds one field, the `position` of a location (section 6).

## 2. The statement

```rob
robolang 2.3;

task send_tugger(robot a, robot b, location handoff) {
    goal { robot_at t handoff; }
    plan {
        choose robot t from (a, b) where robot_ready t nearest to handoff else { fail "no tugger is free"; }
        move t to handoff;
    }
}
```

`choose TYPE NAME from CANDIDATES [where CONDITION] [nearest to X | farthest from X] else { ... }` binds `NAME` to **one**
entity of `TYPE` (`robot`, `object` or `location`) (C1).

- **Candidates** (C2), all finite: a **list** `(a, b, c)` of parameters, in the order written; **`objects at L`**, the objects
  that are at location `L` when the choice is made (not held, present), in the order the world lists them (`TYPE` is `object`);
  **`robots`**, every robot the world declares, in the order it lists them (`TYPE` is `robot`). Nothing else: there is no "any
  object in the world". `objects at L` takes every object at `L`; use `where` to filter.
- **`where`** (C3) is a task condition over the chosen name: `robot_ready t`, `object_at p dock`, `in_state p "clean"`,
  `object_exists`, `region_clear`, `sensor_ok`, `robot_at`, `holding`, with `and`, `or` and `not`. It is evaluated from the world
  state at the choice. **A value that is unknown excludes that candidate** (it fails closed). It states world facts only: never a
  reading, the clock or a message (`E330`). It must name the chosen name.
- **Order** (C4): `nearest to X` or `farthest from X`, where `X` is a parameter that is a location, a robot or an object.
  **Ties, and a choice with no order, go to the first candidate** in the order written (or listed): a choice is deterministic and
  can be reproduced. A different order of the list is a different choice when two are equally near.
- **`else`** (C6) is **mandatory**, as the timeout of a `wait until` is (`E316`), and **must end in `fail`** (`E329`; owner's
  point 1). It runs when no candidate is left, and a message may come before the `fail`:
  `else { communicate r to "ops" with "no tugger"; fail "no tugger is free"; }`. After the statement the name is always bound.
- **The bound name** is used like a parameter from the end of the statement to the end of its block, in the goal too (C7): a
  `goal` block, and the `goal` of an `if` branch, may name what a `choose` at the top level of its own plan or branch binds,
  though the goal comes first. The goal is checked **for the entity chosen**, never "for some entity"; a run that ends before the
  choice has not reached the goal. A name chosen inside a branch is not visible after it, and a goal that names it elsewhere is
  `E303`. A `pre` block cannot name it.
- **A choice uses parameters, never another choice** (C9): the candidates, `objects at`'s place and the reference of an order
  are parameters. The name of an earlier choice there is `E331`. The name of a choice is not a parameter, a binding of
  `measure`, or an earlier choice's: that is `E302`.
- **`choose` never reads a value.** A finite named set that is compared by readings (`which of bin_1 and bin_2 is fuller`) stays
  an `if` over two `measure` bindings (2.2): that is a program, and it does not use `choose`.

## 3. Meaning (simulation, section 21)

At the choice the candidates are collected, those for which `where` is not true are excluded (false or unknown), the rest are
ordered by distance (section 6) and the first is bound. A candidate or reference point with no position is unknown and excluded
(C5). The run record lists, for each choice, the candidates, those excluded and why, the order used and the entity chosen
(C7). The `else` runs when none is left: the step fails with **`S272`** (handled by the `else`) and the run ends in `S262`.

**A choice whose reference point is a location with no position** is infeasible before the run: positions never change, so
every candidate would be left out whatever happens. It has its own code, **`S273`** (authorization phase, and
`validate --world`), so that `delegate` can refuse it before any robot moves (owner's point 7). A robot or an object as a
reference has a place that depends on the run: it is checked when the choice is made.

A step that uses a chosen robot or place is checked when it runs: `grasp` by a robot without a gripper is `S211`, a sensor
mounted on another robot is `S231`, and a region limit (`no_go_region`, `in_region`) fails the step with `S003` or `S004`. They
are the same checks as for a parameter, made when the name is known.

## 4. Missions (C10)

A task with `choose` may run in a mission, and **each run chooses afresh**, from the world as it is when that run starts. "Move
every new part from the conveyor" is a mission that repeats a task that chooses `from objects at conveyor_in`: a part that has
been moved is no longer a candidate. The mission's bound is the one the request states (a duration or a time of day, 0041);
a run that finds no candidate fails (`else`), and `on failure { stop; }` ends the mission as failed.

**A run that fails without advancing time** (owner's point 3). Under a time bound (`for at most D`, or an `until clock_time >=
T`), `on failure { continue; }` after a run that changed **neither the world nor the clock** would repeat that run for ever: such
a run is followed by an identical one. In 2.3 the mission **stops** instead of repeating: it ends `failed`, `ended_by`
`no_progress`, with the failed run's code. With only an `at most N runs` bound it ends at N, as before. Files below 2.3 keep the
earlier behaviour.

## 5. `export-goal` and `delegate` (C8, the Core decision)

**Core 0.2 is unchanged.** A robot only ever receives a goal whose IDs are fixed. A task with `choose` is accepted by `export-goal`
and `delegate` only if **every `choose` is at the top level of the plan with nothing but other `choose` statements before it**
(so before the first action) **and uses world facts only** (which `E330` already requires). The planner then makes the choices
from the world model as it is at the start, records them (`choices` in the delegate record, and in the exported goal's
planner-side output), and sends the task with the choices filled in: each chosen name is a parameter bound to the entity chosen.
Any other task with `choose` is refused with **`X609`**: a `choose` after an action, inside an `if` or a handler, or a choice with
no candidate in the world as it starts. A choice that uses only world facts may come first even when the request names it late
("take it to whichever place is nearest"): the choice is the same whether the robot holds the tote yet or not.

A robot's own library never holds a task with `choose`: a library task with one is `choose_unsupported`. **The ROS 2 / Gazebo
backend refuses `choose`** (`S508`, before any motion): it has no world model to choose from and no positions.

## 6. World file 1.8: positions (C5)

A location may have a `position`: `"locations": [ "dock_1", { "id": "dock_2", "position": [42, 18] } ]`. A coordinate is a
whole number or an exact decimal in a string (`"12.5"`), in metres; there is no float. A robot or an object is at the position
of its location (an object a robot holds is where the robot is). **Distance is the straight line**, compared exactly as the
square, so a tie is exact. Distance is used only to order candidates; **no number is readable by a task**. A later world version
may add routes, which would take precedence over straight lines.

## 7. What 2.3 cannot say (C9)

Decline each as inexpressible:

- **choosing by a reading of each candidate** ("the fullest bin on the row", "any pan above 80 degC"): `where` and the order use
  world facts, because a reading is a sequence of robot actions, not a choice (G-EACH);
- **acting on each candidate in turn**, with a memory of which are done ("every pan", "carry as many as counted") (G-EACH);
- **detecting objects the world does not list** (`objects at L` yields only what the world declares) (G-DETECT);
- **choosing a person** ("the robot nearest the person who called"): a person is no entity of the world (G-PROX);
- **several choices that depend on each other** (`E331`).

## 8. Static rules (`validate`)

1. In a file below 2.3, `choose` is `E203`.
2. The type of a choice is `robot`, `object` or `location`; every candidate is a parameter of that type, `objects at L` needs an
   `object` choice and a `location` parameter, `robots` a `robot` choice, and the reference of an order a location, robot or
   object parameter (`E303`, `E304`).
3. The chosen name is new: not a parameter, a binding or an earlier choice (`E302`).
4. The `else` is there (`E316`) and ends in `fail` (`E329`).
5. `where` names the chosen name and states world facts only (`E330`).
6. A choice uses no name chosen earlier (`E331`).
7. Every path has a goal or ends in `fail` (`E309`), as before: the `else` of a choice is a path that ends in `fail`.

## 9. Task IR and Mission IR

A 2.3 file compiles to **Task IR 1.11** ([ir-schema-v1.11.md](ir-schema-v1.11.md)) with `language_version` `2.3`: one step kind,
`choose`. A file with a mission compiles to **Mission IR 1.3**, which holds Task IR 1.11 tasks
([mission-ir-v1.0.md](mission-ir-v1.0.md)). The reader re-validates both in full ([ir-reader-v0.1.md](ir-reader-v0.1.md)).

## 10. Safety (C11)

**Hazard H27** ([safety case](../docs/safety-case-draft.md)): a stale or wrong world model makes a choice wrong, so a different
robot or object is sent than the reviewer expected. Controls: the candidates are finite and appear in the task, so an approval
([0038](decisions/0038-authorization-and-approval.md)) names them; the choice is recorded with its reasons; an unknown value
excludes a candidate; a step that uses a chosen robot or place is checked when it runs. In physical mode the positions and
states come from the fleet manager (G4, commissioning), not from a file. Declared is not enforced: nothing here moves a robot.

## 11. Diagnostics

| Code | Stage | Meaning |
|---|---|---|
| `E203` | syntax | (existing) `choose` in a file below 2.3 |
| `E302`, `E303`, `E304` | static | (existing) the chosen name is not new; a name is not a parameter; a name has the wrong type |
| `E316` | syntax | (existing) a `choose` has no `else` |
| `E329` | static | The `else` of a `choose` does not end in `fail` |
| `E330` | static | `where` uses a reading, the clock or a message, or does not name the chosen name |
| `E331` | static | A choice names a name that an earlier choice bound |
| `S211`, `S231`, `S003`, `S004` | run | (existing) a chosen robot or place fails the step that uses it |
| `S272` | run | A choice left no candidate: its `else` runs (the step is handled) |
| `S273` | authorization | The place an order measures from has no position in the world: infeasible before the run |
| `X609` | export | `export-goal` and `delegate` refuse a `choose` that the planner cannot make before the first action |
| `S508` | ROS 2 backend | (existing) `choose` is refused |

In the IR reader the forms are `I230` (`E329`), `I231` (`E330`) and `I232` (`E331`); a name that is not new is `I202`, undeclared
`I203`, of the wrong type `I204`. `E213` lists `1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9, 2.0, 2.1, 2.2 and 2.3`.

## 12. Compatibility

Nothing changes for 1.1 to 2.2 files, their IR, their run records or their world files. World files 1.0 to 1.7 serve a 2.3 task
as before; a location in them has no position, so an order measured from or to one excludes it (`S273` for a reference point).
World file 1.8 is 1.7 plus the `position` of a location, and nothing else.
