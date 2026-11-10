# RoboLang 1.6 language reference

RoboLang is a small, checked language for robot tasks. A program describes a desired final state and
a plan for reaching it in a symbolic world. A program is correct only when it validates, names the
right entities, states the operator's requested outcome in `goal`, and reaches that goal.

RoboLang 1.6 has bounded branching (`if`/`else`, see "Branching"), step failure handlers (a step
`else { ... }` that runs when that step fails, see "Failure handlers") and measured values (`measure`
reads a temperature, a mass, a distance or a label from a sensor, see "Measured values"). It has no loops
("until it works"), counts, clocks, schedules, arithmetic, string formatting, message waits, or
variables beyond typed task parameters and the names that `measure` binds. Do not silently drop part of a request. If a request needs an unsupported construct, or the available
world and robots cannot perform it, say that it cannot be represented or completed.

## File and task structure

A file starts with `robolang 1.6;` and holds one or more tasks. `//` starts a comment. The language
is case-sensitive. Whitespace only separates tokens.

```text
task NAME(type param, type param, ...) {
    @safety max_speed 0.5 m/s;     // zero or more annotations, first
    pre  { condition; ... }        // optional
    goal { condition; ... }        // required unless every branch has its own goal
    plan { statement; ... }        // required, at least one statement
}
```

The order inside a task is fixed: annotations, `pre`, `goal`, `plan`. Every condition and statement
ends with `;`.

Parameter types are `robot`, `object`, `location`, `region`, and `sensor`. A parameter is a
placeholder that a caller binds to a world entity of the same type. Names are ASCII letters, digits
and `_`, starting with a letter. These words are reserved and cannot be names:

```text
robolang task pre plan goal robot object location region sensor move grasp place inspect wait communicate
to at using with for until timeout robot_ready region_clear object_exists sensor_ok object_at robot_at holding
and or not in_region with_approach no_go_region require_guard message_received scalar duration
if else fail measure as
```

Numbers are unsigned decimals (`2`, `0.5`); there is no sign or exponent. A duration is a number
followed by `ms`, `s`, or `min`, and must be above zero. Write prose quantities numerically: half a
second is `500 ms`; a quarter minute is `15 s`.

## Plan statements

```text
move r to dst;
grasp r obj;
place r obj at dst;
inspect r obj using camera;
measure r temperature of obj using probe as t;
wait for 2 s;
wait until region_clear cell timeout 30 s else { fail "cell not clear"; }
communicate r to "channel" with "message";
communicate r to "channel" with t;
```

Here `r` is a robot, `obj` an object, `dst` a location, `camera` and `probe` sensor parameters, and `t` a name bound by
`measure` ("Measured values"). A sensor is
never a quoted string. `wait until` always needs a timeout and a failure handler (`E316`). Strings cannot contain a literal
line break; the only escapes are `\"`, `\\`, `\n`, and `\t`.

## Branching

```text
if region_clear cell {
    goal { object_at box a; }       // optional: adds to the goal on this path
    move r to a;
    place r box at a;
} else if robot_ready r2 {          // `else if` is shorthand for an `else` holding an `if`
    goal { object_at box b; }
    move r2 to b;
    place r2 box at b;
} else {
    communicate r to "alerts" with "no route";
    fail "no route";                // ends the run as failed; last statement of a branch only
}
```

- The condition is any condition (below). It is evaluated **once, when the `if` is reached**, against
  the world at that moment. An unknown condition fails the run (`S261`); no branch is taken.
- `else` is optional. A branch has at least one statement. Statements after an `if` run on every path
  that does not end in `fail`.
- A path is one choice of `then` or `else` at each `if`. Its goal is the task `goal` plus the `goal`
  block of each branch it takes. A `goal` block may only be the first item of a branch. **Every path
  must have at least one goal condition** (from the task or its branches), or end in `fail` (`E309`).
  An `if` with no `else` has an empty `else` path, so the task needs its own `goal` for it.
- `fail "reason";` is for "move nothing and report it": the run ends FAILED (`S262`) with the reason. The reason
  has at most 200 characters and no `\n` or `\t` (`E314`).
  Messages sent before it stand. Use it only as the last statement of a branch (`E313`); the reason
  must not be empty (`E307`).
- `if` is nested at most 4 deep (`E216`). `if` cannot be used in `pre` or in a `wait until` condition.
- An `if` is decided by the world, not by whether an earlier step worked. To react to a failed step,
  use a failure handler (below). Nothing is repeated, counted, or timed: if a request needs a loop
  ("until it works", "as many times as it takes"), elapsed time, or waiting for a message, it cannot
  be represented: say so.

**A condition about the moment the job runs is decided by the program, with `if`.** When the request
says "if arm1 is ready when the job starts", "if the cell is clear at that moment", "if the box is
still there", or "whichever robot is ready", the answer is not in the world description you were
given: it is whatever is true when the program runs. Never settle it by reading the description and
writing only the branch that is true there, and never write only one branch (with or without `pre`).
Write the `if` with both branches, and with a goal for each path. A program with one branch passes
in the world shown and fails in the other worlds the request was about.

## Failure handlers

```text
move r to src else { communicate r to "ops" with "cannot reach src"; fail "cannot reach src"; }
grasp r box else { grasp r box; }               // one more attempt
wait until region_clear aisle timeout 10 s else {
    place r box at src;                         // clean up
    fail "the aisle did not clear";             // then stop
}
```

- A `move`, `grasp`, `place`, `inspect`, `measure` or `wait until` may end with `else { ... }` instead of `;`.
  The block, the **handler**, runs only when that step fails. `wait for`, `communicate` and `fail`
  take no handler (`E201`). There is no handler for a group of steps: to react to any step failing,
  give each step that can fail its own handler.
- A step that fails changes nothing. Time a `wait until` spent waiting has passed.
- **Every `wait until` has a handler** (`E316`). If the request does not say what to do when the
  wait times out, the handler is `else { fail "<what did not happen>"; }`. If it does say (report,
  wait longer, clean up), do that, and end with `fail` unless the request says to carry on. "If it
  does not clear, do nothing" (or "move nothing") is also `else { fail "<reason>"; }`: the run ends
  failed and nothing moves, which is what was asked. It is representable: write the program.
- **A handler has no `goal`** (`E208`) and never changes what success means. It either recovers, so
  the run still reaches the task's own goal, or it cleans up and ends in `fail`. A request that counts
  a different end state as done ("otherwise leave it at the handoff and call it done") cannot be
  represented: say so.
- Unless a handler ends in `fail`, the plan goes on with the step after the handled one. Use that
  only when the request says to carry on (for example, skip an inspection and still deliver).
- **A fixed number of retries is allowed:** a handler may repeat its step, and handlers nest, so
  `grasp r box else { grasp r box else { grasp r box; } }` is up to three attempts. A handler counts
  as one nesting level, as an `if` does: at most 4 in all (`E216`). For more attempts, follow with
  `if not holding r box { grasp r box else { ... } }` steps. "Until it works" or "as many times as
  it takes" is a loop: it cannot be represented.
- A path is one choice at each `if` and at each handled step (it worked, or its handler ran). Every
  path needs a goal or must end in `fail` (`E309`).

## Measured values

```text
measure r temperature of pan using probe as t;
measure r mass of tray using scale as m else { fail "no weight"; }
measure r label of vial using scanner as lot;
if t < 60 degC { ... }
wait until temperature of pan using probe < 60 degC timeout 10 min else { fail "still hot"; }
communicate r to "pass" with t;
```

**`inspect` never reads a value.** It records whether an object is present and where it is, nothing else.
Weighing, taking a temperature, reading a batch number or a barcode is `measure`, even when the request
only says "weigh it, then move it": writing `inspect` for it does less than was asked. Choose `measure`
whenever a request asks for a weight, a temperature, a distance, a batch number or a label, whether or not the
program uses the value afterwards.

- **`measure r KIND of o using s as NAME;`** takes one reading of `KIND` of object `o` with sensor `s`, mounted on
  robot `r`, and binds it to `NAME`. `KIND` is `temperature`, `mass`, `distance` or `label`; any other word is
  `E217`. The world list shows what each sensor measures, for example
  `probe_temp (on chef_arm, available; measures temperature in degC +/- 0.5, contact)`. If no sensor on the
  robot the request names measures that kind, the request cannot be done in this world: decline it as
  `infeasible`. Do not substitute `inspect`, and do not use another kind.
- **Kinds and units.** `temperature`: `degC`, `K`. `mass`: `g`, `kg`. `distance`: `mm`, `cm`, `m`. `label`: text
  (a batch number, a barcode), no unit. A quantity is a number and a unit (`60 degC`, `1.2 kg`, `500 mm`).
  Units of one kind convert exactly (`1.5 kg` is `1500 g`), so write the limit in the unit the request used. A unit of
  another kind is `E319`. Numbers are unsigned, so write a temperature below zero in kelvin (`255 K`). **Degrees
  Fahrenheit is not a unit:** convert it yourself (`140 F` is `60 degC`) and write `degC`.
- **Contact.** A sensor that shows `contact` (a probe, a scale) reads only an object at the robot's own location, or one the
  robot holds: `move r` to the object first, as for `grasp`, or the step fails (`S235`). A sensor without `contact`
  (a scanner, a camera) reads from anywhere. The step also fails when the sensor is not on `r` (`S231`), is not
  available (`S232`, `S233`: `wait until sensor_ok s ...` first if it comes online later), does not measure the kind
  (`S234`), the object is not present (`S213`, `S214`), or the object has no value of that kind (`S236`). A failed
  `measure` changes nothing and binds nothing, and takes a handler like any other step. `measure` takes no time and does
  not need the robot to be ready.
- **The value is read when the program runs.** The world list never states the value to be measured. Never settle a
  comparison from a description, and never write a branch from a guess: write the `measure` and the comparison.
- **Comparisons** are conditions, written `t < 60 degC`, `a <= b`, `lot == "LOT-7731"`. The left side is a name bound by
  `measure` (or a live reading, below); the right side is a quantity, a string or another name of the same kind (a
  literal on the left is `E201`). Quantities take `<`, `<=`, `>`, `>=` only: `==` or `!=` on a quantity is `E320`, so
  "between 50 and 70" is `t >= 50 degC and t <= 70 degC`. Labels take `==` and `!=` only. Kinds must match
  (`E319`). Combine with `and`, `or`, `not` and parentheses like any condition. **A comparison is allowed in `if` and
  in `wait until` conditions only**, not in `pre` or `goal` (`E321`): goals state facts about the world, not values.
  To act only if a reading passes, write an `if`, and `fail` or recover on the other path.
- **Live readings, in `wait until` only.** `wait until temperature of pan using probe < 60 degC timeout 10 min else { ... }`
  reads the sensor again each time the wait looks at the world; it takes no robot, and a `contact` sensor is read
  through the robot it is mounted on, so `move` that robot to the object first, or the reading is never taken and the
  wait keeps waiting. A live reading anywhere else is `E322` (`measure` it instead). A name bound by `measure` is a
  single reading: it never changes later.
- **Uncertainty.** A reading is a value plus or minus the sensor's uncertainty. `t < 60 degC` is true only if the whole
  range is below 60, false only if the whole range is at or above 60, and **otherwise unknown**: a reading of
  `59.8 degC +/- 0.5` against `< 60 degC` is unknown. An unknown condition fails the run in an `if` (`S261`) and keeps
  a `wait until` waiting. This is deliberate. Write the comparison the request asks for; do not move the limit to
  avoid it. Two names are compared the same way, range against range. Labels are exact and never unknown.
- **Bindings.** `NAME` is set once and never changed. It must not be the name of a parameter or of another binding in the
  task (`E318`). It is visible after its `measure`, in the same block and in blocks nested in it; using it elsewhere
  is `E317`. A binding made inside an `if` branch or a handler is not visible after that branch or handler. It is not visible
  in the handler of its own `measure`. **One exception, to retry a reading:** the handler of a `measure` may bind the same
  name again (and must read the same kind, `E319`), and then it is visible after both paths:
  `measure r mass of tray using scale as m else { measure r mass of tray using scale as m; }`. If the handler ends in
  `fail`, the name is visible after the step. If it neither ends in `fail` nor measures again, nothing was read on that
  path and using the name later is `E317`.
- **Sending a value.** `communicate r to "pass" with t;` sends the reading itself, with its kind, value, unit and
  uncertainty. "Tell the channel the value you measured" is this, never a literal text. There is no formatting: a
  message cannot be built from a value ("Pan 1 is at <reading> degC"), and the request is then not representable.
- **Not in the language:** arithmetic (an average, a difference, a sum of readings), formatting a value into text,
  counting, clock readings, setting a value (no command writes one), and choosing an object at run time ("any pan
  above 80 degC": there is no way to search; name the objects). Say that these cannot be represented.

## Conditions

| Atom | True when |
|---|---|
| `robot_ready r` | robot `r` is ready for physical actions |
| `region_clear c` | region `c` is clear |
| `object_exists o` | object `o` is present |
| `sensor_ok s` | sensor parameter `s` is available |
| `object_at o l` | object `o` is present, not held, and at location `l` |
| `robot_at r l` | robot `r` is at location `l` |
| `holding r o` | robot `r` holds object `o` |
| `t < 60 degC`, `lot == "A"` | a measured value compares as stated (only in `if` and `wait until`; see "Measured values") |

Combine conditions with `not`, `and`, `or`, and parentheses:

```text
region_clear a or region_clear b;
not region_clear z;
(robot_ready r and region_clear c) or object_exists o;
```

`and` and `or` cannot be mixed at one level without parentheses. A condition is at most eight
levels deep. A fact the world does not state is unknown, and unknown is never treated as true.

## Preconditions, goals, and annotations

- `pre` conditions are checked once before the first step. If any is not true, the run stops and
  nothing moves. Use `pre` for "only if", "provided that", and "refuse to start unless". Use
  `wait until` for "wait for", "once it clears", and "when it is free".
- `goal` is the requested final state, checked after the plan. Every top-level goal condition must
  hold. State every requested end condition (`object_at`, `robot_at`, or `holding`) and nothing the
  request did not ask for. A held object is not `object_at` anywhere. A goal describes the outcome,
  not the steps.
- `@safety max_speed N m/s;` declares a linear limit and `@safety max_speed N rad/s;` an angular
  limit. The unit is required, each unit can appear once per task, and `N` must be above zero. Add a
  limit only when requested, converting units when needed (`30 cm/s` is `0.3 m/s`). A limit above a
  bound robot's declared maximum, or in a unit that robot does not declare, fails the simulation.
- `@verify min_confidence 0.9;` (from 0 to 1) and `@verify max_age 5 s;` state how observations must
  support a Core goal. Use them only when requested.

## What symbolic simulation requires

- Every `move`, `grasp`, and `place` requires its robot to be ready. Robots can move to any declared
  location. A robot carrying an object takes it along.
- `grasp r o` requires `r` to have a gripper and hold nothing, and requires present object `o` to be
  at `r`'s current location. Move there first. A robot holds one object at a time: `place` it before the
  same robot grasps another, or before another robot grasps it (`E315`).
- `place r o at dst` requires `r` to hold `o` and be at `dst`. Move there first.
- `inspect r o using s` requires sensor `s` to be mounted on `r` and available. The object can be
  anywhere.
- `measure r k of o using s as NAME` requires `s` mounted on `r` and available, to measure `k`, the object present and, for a
  `contact` sensor, at `r`'s location (or held by `r`), and the object to have a value of that kind. `inspect`
  reads no value.
- `wait until C timeout T` checks `C` immediately and whenever the world changes during `T`. It
  fails if `C` is not true by the timeout, and its handler runs. Only waits advance simulated time.
- `communicate` records a message and changes no other state.
- A task can bind more than one robot. Their steps still run sequentially in written order.

Validation checks source structure, names, and types. World-specific capability and state failures
appear when the task is checked against a world or simulated. A run fails with `S261` when the
condition of an `if` is unknown, and with `S262` when it reaches a `fail`. Common source diagnostics
are:

| Code | Meaning |
|---|---|
| `E201` | a handler after `wait for`, `communicate` or `fail` |
| `E202` | missing `;` |
| `E206` | missing or unknown unit, or a number in a comparison with no unit |
| `E208` | task blocks in the wrong order, an empty branch or handler, a `goal` block that is not first in a branch, or a `goal` block in a handler |
| `E212` | reserved word used as a name |
| `E214` | `and` and `or` mixed without parentheses |
| `E217` | a kind that is not `temperature`, `mass`, `distance` or `label` |
| `E216` | `if` statements and handlers nested more than 4 deep |
| `E303` | name is not a task parameter |
| `E304` | parameter used with the wrong type |
| `E307` | empty string (for example `fail ""`) |
| `E309` | a path has no `goal` condition and does not end in `fail` |
| `E313` | `fail` that is not the last statement of a branch or handler |
| `E315` | `grasp` of an object a robot still holds, or by a robot that still holds one: `place` it first |
| `E310` | quoted sensor name used instead of a sensor parameter |
| `E316` | `wait until` without a failure handler |
| `E317` | a name used as a value that is not a visible binding (before its `measure`, outside its branch or handler, a parameter) |
| `E318` | a binding named like a parameter or another binding (except a retry in the handler of its own `measure`) |
| `E319` | kinds that do not match: a unit of another kind, a string against a quantity, a retry of another kind |
| `E320` | `==` or `!=` on a quantity, or ordering on a label |
| `E321` | a comparison in `pre` or `goal` |
| `E322` | a live reading anywhere but the condition of a `wait until` |
| `S234` | `measure`: the sensor does not measure that kind |
| `S235` | `measure` with a `contact` sensor: the robot is not at the object |
| `S236` | `measure`: the object has no value of that kind |

## Complete example

This task waits for a cell (and stops if it does not clear in time), inspects a box, moves it, and
records completion. Its parameters still
need bindings to concrete world entities before simulation.

```rob
robolang 1.6;

task deliver(robot r, object box, location src, location dst, region cell, sensor camera) {
    @safety max_speed 0.5 m/s;
    pre { sensor_ok camera; }
    goal { object_at box dst; }
    plan {
        wait until region_clear cell timeout 20 s else {
            fail "the cell did not clear";
        }
        move r to src;
        inspect r box using camera;
        grasp r box;
        move r to dst;
        place r box at dst;
        communicate r to "fleet" with "DONE";
    }
}
```

This task chooses at run time. If the cell is clear when the job reaches the `if`, the box goes to
`a`; otherwise it goes to `b`. Each path has its own goal.

```rob
robolang 1.6;

task route(robot r, object box, location src, location a, location b, region cell) {
    plan {
        move r to src;
        grasp r box;
        if region_clear cell {
            goal { object_at box a; }
            move r to a;
            place r box at a;
        } else {
            goal { object_at box b; }
            move r to b;
            place r box at b;
        }
    }
}
```

This task retries a failed grasp once. If the second grasp also fails, it reports and stops.

```rob
robolang 1.6;

task fetch(robot r, object box, location src, location dst) {
    goal { object_at box dst; }
    plan {
        move r to src;
        grasp r box else {
            grasp r box else {
                communicate r to "ops" with "grasp failed";
                fail "grasp failed twice";
            }
        }
        move r to dst;
        place r box at dst;
    }
}
```

This task measures, compares and reports. The probe is a contact sensor, so the arm moves to the pan first. It sends the
reading to the "pass" channel, and moves the pan only if the reading is clearly below 60 degC. A borderline reading
(for example 59.8 degC with an uncertainty of 0.5) is unknown and fails the run (`S261`) instead of moving a pan that
may be too hot.

```rob
robolang 1.6;

task check_pan(robot r, object pan, sensor probe, location src, location dst) {
    goal { object_at pan dst; }
    plan {
        move r to src;
        measure r temperature of pan using probe as t else {
            communicate r to "pass" with "no reading";
            fail "no reading";
        }
        communicate r to "pass" with t;
        if t < 60 degC {
            grasp r pan;
            move r to dst;
            place r pan at dst;
        } else {
            fail "pan too hot";
        }
    }
}
```

`robolang validate` checks this source but does not execute it or establish that it is safe on a
physical robot. Hardware execution requires a separate robot-specific authorization and safety case.
