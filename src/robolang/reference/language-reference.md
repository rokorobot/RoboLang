# RoboLang 1.4 language reference

RoboLang is a small, checked language for robot tasks. A program describes a desired final state and
a plan for reaching it in a symbolic world. A program is correct only when it validates, names the
right entities, states the operator's requested outcome in `goal`, and reaches that goal.

RoboLang 1.4 has bounded branching (`if`/`else`, see "Branching") but no loops, retry, error handling,
counts, clocks, schedules, arithmetic, message waits, or variables beyond typed task parameters. Do
not silently drop part of a request. If a request needs an unsupported construct, or the available
world and robots cannot perform it, say that it cannot be represented or completed.

## File and task structure

A file starts with `robolang 1.4;` and holds one or more tasks. `//` starts a comment. The language
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
if else fail
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
wait for 2 s;
wait until region_clear cell timeout 30 s;
communicate r to "channel" with "message";
```

Here `r` is a robot, `obj` an object, `dst` a location, and `camera` a sensor parameter. A sensor is
never a quoted string in 1.3 and 1.4. `wait until` always needs a timeout. Strings cannot contain a literal
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
- `if` and `else` are not conditions: nothing is retried, repeated, counted, or timed. If a request
  needs a loop ("until it works"), a retry, "after N times", elapsed time, or waiting for a message,
  it cannot be represented: say so.

**A condition about the moment the job runs is decided by the program, with `if`.** When the request
says "if arm1 is ready when the job starts", "if the cell is clear at that moment", "if the box is
still there", or "whichever robot is ready", the answer is not in the world description you were
given: it is whatever is true when the program runs. Never settle it by reading the description and
writing only the branch that is true there, and never write only one branch (with or without `pre`).
Write the `if` with both branches, and with a goal for each path. A program with one branch passes
in the world shown and fails in the other worlds the request was about.

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
  at `r`'s current location. Move there first.
- `place r o at dst` requires `r` to hold `o` and be at `dst`. Move there first.
- `inspect r o using s` requires sensor `s` to be mounted on `r` and available. The object can be
  anywhere.
- `wait until C timeout T` checks `C` immediately and whenever the world changes during `T`. It
  fails if `C` is not true by the timeout. Only waits advance simulated time.
- `communicate` records a message and changes no other state.
- A task can bind more than one robot. Their steps still run sequentially in written order.

Validation checks source structure, names, and types. World-specific capability and state failures
appear when the task is checked against a world or simulated. A run fails with `S261` when the
condition of an `if` is unknown, and with `S262` when it reaches a `fail`. Common source diagnostics
are:

| Code | Meaning |
|---|---|
| `E202` | missing `;` |
| `E206` | missing or unknown unit |
| `E208` | task blocks in the wrong order, an empty branch, or a `goal` block that is not first in a branch |
| `E212` | reserved word used as a name |
| `E214` | `and` and `or` mixed without parentheses |
| `E216` | `if` nested more than 4 deep |
| `E303` | name is not a task parameter |
| `E304` | parameter used with the wrong type |
| `E307` | empty string (for example `fail ""`) |
| `E309` | a path has no `goal` condition and does not end in `fail` |
| `E313` | `fail` that is not the last statement of a branch |
| `E310` | quoted sensor name used instead of a sensor parameter |

## Complete example

This task waits for a cell, inspects a box, moves it, and records completion. Its parameters still
need bindings to concrete world entities before simulation.

```rob
robolang 1.4;

task deliver(robot r, object box, location src, location dst, region cell, sensor camera) {
    @safety max_speed 0.5 m/s;
    pre { sensor_ok camera; }
    goal { object_at box dst; }
    plan {
        wait until region_clear cell timeout 20 s;
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
robolang 1.4;

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

`robolang validate` checks this source but does not execute it or establish that it is safe on a
physical robot. Hardware execution requires a separate robot-specific authorization and safety case.
