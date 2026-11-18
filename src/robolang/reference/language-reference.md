# RoboLang 2.0 language reference

RoboLang is a small, checked language for robot tasks. A program describes a desired final state and
a plan for reaching it in a symbolic world. A program is correct only when it validates, names the
right entities, states the operator's requested outcome in `goal`, and reaches that goal.

RoboLang 2.0 has bounded branching (`if`/`else`, see "Branching"), step failure handlers (a step
`else { ... }` that runs when that step fails, see "Failure handlers"), measured values (`measure`
reads a temperature, a mass, a distance or a label from a sensor, see "Measured values"), region
limits (`@safety no_go_region R;`, `@safety in_region R;`: where the robots may be while they move, see
"Region limits"), declared actions (`do r open door;` asks a robot for an action its profile declares, and
`in_state door "open"` checks the result, see "Declared actions"), clock readings (`measure r time as t;` reads the
time of day, `22:00` is a time, `wait until clock_time >= 23:00 ...` waits for one, see "Clock readings") and
**missions** (a `mission` runs tasks one after another, again and again, with a hard bound: a count, a duration or a
time of day, see "Missions"). A task itself never loops: it runs once. **A fixed count is not a mission:** "three round
trips" is one task with the steps written out three times (a mission with `at most 3 runs` is also accepted), and
"try the grasp once more" or "retry the step up to 3 times" is a failure handler, a program, not a decline. It has no
repeat without a bound ("forever", "until it works" with no count or time), triggers ("every hour", "every morning",
"at 06:00", "when a message arrives"), loops inside loops, choosing the next object at run time, dates, arithmetic,
string formatting, message waits, limits on one step only, limits on an arm or a person, or variables beyond typed
parameters and the names that `measure` binds. Do not silently drop part of a request. If a request needs an unsupported construct, or the available
world and robots cannot perform it, say that it cannot be represented or completed.

## File and task structure

A file starts with `robolang 2.0;` and holds one or more tasks, and missions that run them (see "Missions"). A file of
tasks only needs no mission. `//` starts a comment. The language
is case-sensitive. Whitespace only separates tokens.

```text
task NAME(type param, type param, ...) {
    @safety max_speed 0.5 m/s;     // zero or more annotations, first
    @safety no_go_region wet;      // region limits are annotations too (see "Region limits")
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
if else fail measure as do in_state clock_time clock_elapsed
mission repeat run runs stop continue retry
```

Numbers are unsigned decimals (`2`, `0.5`); there is no sign or exponent. A duration is a number
followed by `ms`, `s`, or `min`, and must be above zero. Write prose quantities numerically: half a
second is `500 ms`; a quarter minute is `15 s`. **There is no unit for hours:** 90 minutes is `90 min` and 2 hours are
`120 min`. A time of day is not a duration: it is written `22:00` ("Clock readings").

## Plan statements

```text
move r to dst;
grasp r obj;
place r obj at dst;
inspect r obj using camera;
measure r temperature of obj using probe as t;
measure r time as now;
do r open door;
do r set fryer to 175 degC;
wait for 2 s;
wait until region_clear cell timeout 30 s else { fail "cell not clear"; }
wait until clock_time >= 23:00 timeout 90 min else { fail "23:00 did not come in time"; }
communicate r to "channel" with "message";
communicate r to "channel" with t;
```

Here `r` is a robot, `obj` an object, `dst` a location, `camera` and `probe` sensor parameters, `t` a name bound by
`measure` ("Measured values"; `now` is a time of day, "Clock readings"), and `door` an object or `fryer` a location that a `do` acts on ("Declared actions"). A sensor is
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
  use a failure handler (below). Nothing inside a task is repeated, counted, or timed: repeating whole
  task runs with a stated bound is a mission ("Missions"). A request that needs a loop with no bound
  ("until it works", "as many times as it takes"), or waiting for a message, cannot be represented: say so.

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
  `if not holding r box { grasp r box else { ... } }` steps. "Retry the step up to 3 times" and "try
  once more, then stop" are this: write the handler, do not decline them. "Until it works" or "as many
  times as it takes", with no number of tries, is a loop with no bound: it cannot be represented. Retrying a
  whole task run is a mission ("Missions").
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
  **The chance of an unknown is never a reason to decline.** "Which of the two is warmer", "if it is below 8 degC"
  are programs: compare the two names (or the name and the limit) in an `if`. If the readings are too close to
  tell, the run fails with `S261` instead of guessing, which is the intended outcome.
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
  counting, and choosing an object at run time ("any pan
  above 80 degC": there is no way to search; name the objects). Say that these cannot be represented. The clock has
  its own section ("Clock readings").

## Clock readings

```text
measure r time as now;                     // the time of day when this step runs, in the world's zone
measure r elapsed as took;                 // the time since the task started
if now < 22:00 { ... } else { ... }        // a time is HH:MM or HH:MM:SS, 24-hour
if took >= 2 min { ... }                   // an elapsed time compares with a duration
wait until clock_time >= 23:00 timeout 90 min else { fail "23:00 did not come in time"; }
communicate r to "lead" with now;          // a time is sent as HH:MM:SS; an elapsed time as seconds, `37 s`
```

```rob
robolang 2.0;

task tote_if_open(robot r, object tote, location inb, location outb) {
    plan {
        measure r time as now;
        if now < 22:00 {
            goal { object_at tote outb; }
            move r to inb; grasp r tote; move r to outb; place r tote at outb;
        } else {
            communicate r to "lead" with "the shift is over";
            fail "the shift has ended";
        }
    }
}
```

The world list says when the task starts and in which zone: `clock: the task starts at 2026-11-17 21:30:00
(Europe/Prague)`, or `clock: none (this world has no clock)`. The clock runs with the task: a `wait for 90 s` moves it
90 seconds on.

- **Reading the time.** `measure r time as now;` binds the time of day, `measure r elapsed as took;` the time since the
  task started. There is no `of` and no `using`: the clock is not an object and has no sensor. `r` is a robot of the
  task: use the robot that does the work or sends the message. It takes a handler like any step. "Tell the lead the time
  the scan finished" is `measure` right after the step it names, then `communicate ... with now;`. It is **never** a
  literal text: "the time" written as a string, a guessed `wait for`, or nothing at all does less than was asked.
- **A time literal** is `HH:MM` or `HH:MM:SS`, 24 hours, two digits each: `06:00`, `21:30`, `23:00:15`. **`24:00`, `9:30` and
  `5pm` are not times** (`E105`). Convert a 12-hour time yourself: 11 pm is `23:00`, 6 am is `06:00`, noon is `12:00`.
- **Comparing.** A time compares with a time (a literal or another time name) and an elapsed value with a duration
  (`90 s`, `2 min`, `500 ms`) or another elapsed name, with `<`, `<=`, `>`, `>=` only: `==` is `E320`, and the kinds
  never mix (`E319`: a time against `90 s`, a time against an elapsed value). "Only if it is before 21:30" is
  `now < 21:30`; "from 08:00 to 18:00" is `now >= 08:00 and now < 18:00`; "at 14:00 or later" is `>=`. A comparison
  stands in `if` and `wait until` only (`E321` in `pre` and `goal`), as for any measured value.
- **Waiting for a time.** `wait until clock_time >= 23:00 timeout 90 min else { ... }` reads the clock each time it looks;
  `clock_elapsed` does the same for elapsed time. They are words, not names: use them in `wait until` only (`E322`
  elsewhere: `measure` the time and compare the name). The timeout and the handler are required as for every wait
  (`E209`, `E316`). "At 11 pm send the cleaner" is a wait until 23:00 with a timeout (the time the request allows, in
  minutes: 2 hours are `120 min`), then the steps. A wait for a time ends at that exact time.
- **A wait that can time out.** "At 05:30, send tug1; wait up to 90 minutes for 05:30 and stop if it has not come" is one
  wait with the timeout the request gives and `fail` in the handler. Two robots waiting for one time are one wait,
  then both robots act.
- **A reading is exact** and is never unknown: the clock has no uncertainty.
- **One day only.** A time literal is that time **on the day the task starts**. "Tomorrow at 01:00", "at midnight
  tonight" (midnight is the next day), "on Monday" and every date cannot be said. If the clock would reach the end of
  the day, a time reading fails (`S238`).
- **A time that has already passed.** A wait for a time that is before the start of the task (`wait until clock_time >=
  09:00` when the task starts at 10:30) fails before anything moves (`S010`): "at 09:00" never silently means "now".
  If the request says "if it is already after 9, go now", write `measure` and `if`. A request that wants an action
  *before* a time that has already passed (the clock starts at 09:15 and the request says "before 08:00 today") or
  *between* times that have passed cannot be done in this world: decline it as `infeasible`, and do not write a program
  for it. The same for a wait whose timeout is shorter than the time left to wait (23:00 is 90 minutes after 21:30,
  and the request allows 10 minutes): decline it as `infeasible`.
- **No clock.** A `time` reading, or `clock_time` in a wait, fails when the world says `clock: none` (`S237`): decline
  the request as `infeasible` when it needs the time of day. `elapsed` needs no clock: "how long the job took" is a program
  even in a world with no clock.
- **How long.** An `elapsed` value is the time since the **task** started, never since another moment, so "how long the
  whole job took" is `measure r elapsed as took;` after the last step, and "how long the job has taken so far" is the same
  right where it is asked. Sent in a message, it is `37 s`. It is **not** the time between two moments: "how much time
  passed between the two scans", "how long the oil took to heat" after other steps, are a difference.
- **Not in the language** (decline as `inexpressible`): another day, a date or a weekday ("tomorrow at 01:00", "on
  Monday at 09:00", "tonight at midnight"); repeating at a time ("every morning at 6", "every night at 23:30", "every
  hour": a task runs once); arithmetic on times and durations ("two hours after the scan finished", the time between
  two readings, "in 2 hours" from a moment that is not the start); another format for a time or a duration ("just
  HH:MM", "9:30 pm", minutes, a bare number of seconds in a message); a time zone in the request ("23:00 UTC"). Say
  that these cannot be represented. **"As soon as possible" asks for no clock:** it is a plain move.
- **Never in a delegated goal.** The clock cannot be in a goal; `delegate` does not delegate a plan that reads it, and
  the ROS 2 backend refuses `measure r time` and `measure r elapsed`.

## Declared actions

```text
do r open door;                     // the robot r does the action `open`, which its profile declares, on `door`
do r set fryer to 175 degC;         // an action that takes a value (a quantity with a unit, or a text)
do r open door else { fail "door stuck"; }
goal { in_state door "open"; }      // the result is checked, never assumed
```

```rob
robolang 2.0;

task restock(robot r, object closet, object kit, location ward, location store) {
    goal { in_state closet "open"; object_at kit ward; }
    plan {
        move r to store;
        do r open closet;
        grasp r kit;
        move r to ward;
        place r kit at ward;
    }
}
```

A robot can do more than the seven statements: open a door, set a temperature, print, lock, charge, dock. It can do
exactly the **actions its profile declares**, and the world list shows them for each robot, for example
`moxi1 (... can do: open (on an object, leaves it open); set (on a location, leaves it heating, takes a temperature))`.
An action is a **name**, not a keyword: write it exactly as the world lists it (`open_door` is not `open`).

- **When to write `do`.** The request asks a robot for something that is not moving, grasping, placing, inspecting,
  measuring, waiting or sending a message: "open the closet", "set the fryer to 175", "print the layout", "lock the
  container", "charge it", "dock it". Write `do r ACTION TARGET;`, with the robot, the action the world lists for it and
  **one** target: an object or a location, of the kind the world says the action acts on (`on an object` or
  `on a location`).
- **A value.** An action that "takes a temperature" (or a mass, a distance, a text) needs `to VALUE`, written as the
  request says it with its unit: `do r set fryer to 175 degC;` (`degC` and `K`, `g` and `kg`, `mm`, `cm` and `m`, or a
  string for a text). An action that takes no value takes none. The value is recorded and **never assumed reached**: if
  the request wants the oil at 175 before the next step, follow with a `measure` or a `wait until` on the reading
  ("Measured values"), with a timeout and a failure handler as always.
- **Always check the result.** A `do` is a request to the robot's own controller: the run records `requested`, and
  nothing more. State the result the request wants in `goal` with `in_state x "label"`, using the result label the
  world lists ("leaves it open" is `in_state door "open"`). A program with the `do` and no `in_state` verifies nothing,
  and a program with the `in_state` and no `do` fails its goal. Never replace an action with a `move`, an `inspect` or
  a `communicate`: that does less than was asked and every other check accepts it.
- **Add no action the request did not ask for.** A world may list actions the request does not mention (a closed door
  the task does not need, a printer that could print). Do not use them. A request to fetch something does not ask to
  open the door of the room.
- **The robot has to be at the target.** As for `grasp`, move the robot to the target location, or to the location of
  the target object, before the `do` (`S271`). A robot that holds the object is at it. There are no actions from afar.
- **A `do` is a step like the others.** It takes a failure handler (`else { ... }`), may stand in an `if` or in a
  handler, and a failed `do` changes nothing. `in_state` is a condition: use it in `pre`, `goal`, `if` and
  `wait until`, combined with `not`, `and`, `or`. It is true when the state is that label, false for another
  label, and **unknown** when the world gives the object or location no state, and unknown fails closed.
- **One target only.** An action has one object or one location. Handing something to a person, or an action with two
  targets, cannot be represented. Nor can an action that returns a value (running a self-test and reporting its result:
  that would be a `measure`, which needs a sensor), a reusable skill with a name ("teach the robot a skill called
  restock"), an action repeated until it works, or one triggered at a clock time. Say that these cannot be represented.
  A duration is not a value: "fry for 3 minutes" is a `do`, then `wait for 3 min`.
- **Charge and dock.** The state is on the charger or the dock, a location: `do r charge charger;`,
  `in_state charger "charging"`. The robot's own state (its battery) cannot be asked for.
- **Regions.** A `do` on a location counts as a target for a region limit, as a `move` does: the robot has to be there.
- **Infeasible, not inexpressible.** The language can say it and this world cannot do it. Decline with
  `"category": "infeasible"` when (1) the robot the request names does not list the action (`S006`), (2) the target is
  the wrong kind for the action, an action on an object given a location or the reverse (`S007`), (3) the request gives
  a value the action does not take, a value of the wrong kind (2 kg for a temperature) or none where one is needed
  (`S008`), or (4) the request needs a result no listed action of the robots leaves, such as "keep the door locked"
  when the robot can only open and close (`S009`). Do not write a program for these.
- **Never in a delegated goal.** `export-goal` and `delegate` refuse a goal with `in_state` (`X606`), and the ROS 2
  backend refuses `do` and `in_state`: Core carries no states.

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
| `in_state x "open"` | the state of object or location `x` is that label (see "Declared actions"; unknown when the world gives `x` no state) |
| `t < 60 degC`, `lot == "A"` | a measured value compares as stated (only in `if` and `wait until`; see "Measured values") |
| `now < 22:00`, `clock_time >= 23:00` | a time of day compares as stated (only in `if` and `wait until`; see "Clock readings") |

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
  bound robot's declared maximum, or in a unit that robot does not declare, fails the simulation: decline such a
  request as infeasible instead of writing the limit (a robot that lists only `m/s` has no `rad/s` limit).
- `@safety no_go_region R;` and `@safety in_region R;` ask the robots to stay out of or inside a region while they
  move: see "Region limits".
- `@verify min_confidence 0.9;` (from 0 to 1) and `@verify max_age 5 s;` state how observations must
  support a Core goal. Use them only when requested.

## Region limits

```text
@safety no_go_region wet;    // every robot of the task stays out of region `wet`
@safety in_region lane;      // every robot of the task stays inside region `lane`
```

Both words go after `@safety`, name a `region` parameter, and are written before `pre`, `goal` and `plan`, in any
order with `max_speed`. Bind the parameter to the region of the world that the request names, by its exact ID.

```rob
robolang 2.0;

task to_lobby(robot r, location lobby, region wet) {
    @safety no_go_region wet;
    goal { robot_at r lobby; }
    plan { move r to lobby; }
}
```

- **When to write one.** The request says where a robot may or may not be *while it moves*: "keep it out of the
  hot zone", "never drive through the quay", "stay inside the lane the whole way", "it must not leave the field".
  Write one limit for each region the request names, of the kind it asks for (`no_go_region` for out,
  `in_region` for inside). A request that names two regions needs two limits; one that says "stay inside A and out of B"
  needs one of each.
- **Never drop a requested limit.** A `move` is a route, not a jump: a robot that "goes straight to the target" can
  still cross the region on the way. A program that reaches the goal without the limit the request asked for is
  a false program that every other check accepts; write the limit, or say that the request cannot be met.
- **Do not add one the request did not ask for.** A request to wait for a region to clear (`wait until region_clear`),
  or to do something only if a region is clear (`if region_clear`), is about the region's state, not about where the
  robot may be. A plain move needs no limit. Add a limit only for a named region, never for "busy areas",
  "crowded" or "dangerous" places that the request does not name.
- **The whole task.** A limit holds for every `move` of every robot of the task, on every path, recovery moves in
  failure handlers included. There is no limit on one step: a request that holds on one leg and not on another
  ("on the way to the hall stay out of the wet floor, but on the way back you may cross it") cannot be represented.
  Nor can a limit on a robot's arm or on what the arm swings over, a distance to a person, or a force: those are not region
  limits. Say that they cannot be represented.
- **At most 8 limits per task, and a region is named once.** The same region twice, or as both `no_go_region` and
  `in_region`, is `E323`; a ninth limit is `E324`. The name must be a `region` parameter (`E303`, `E304`).
- **A request, not an enforcement.** RoboLang checks the limit against the world before the run and records it as
  checked, never as enforced. Keeping a robot out of a region is the job of the robot's planner and safety controller.
  The program does not plan a route around a region and needs no extra moves for it.
- **What the world says.** The world list may show, for a region, the locations it `contains`, and for a robot the
  region limits it `honours`. A region with no list of locations is not known to hold any, so nothing is concluded from
  it. A robot that does not honour a kind of limit cannot be asked to keep it. The run fails before anything moves
  (`S003`, `S004`, `S005`), and the request is **infeasible**: the language can say it, this world cannot do it. Decline
  with `"category": "infeasible"` when (1) a robot that must move does not honour the kind of limit asked (`S003`),
  (2) a location the robot must go to lies inside the region it must stay out of, or outside the region it must stay
  inside, where the world lists the region's locations (`S004`), (3) the robot starts there (`S005`), or (4) the request
  hands the job to the delegated Core executor: Core carries no region limits, so a task with one cannot be
  delegated.

## Missions

A **mission** runs tasks one after another. Every task run is a complete run with its own `pre`, limits, steps and goal; the
mission only decides whether to start the next one. It is written next to the tasks it runs, in the same file.

```text
mission NAME(type param, ...) {
    repeat at most 30 runs, for at most 90 min {     // a hard bound: a count, a duration, or both
        run TASK(param, param, ...);                 // one or more run lines, each a run of a task of the file
    } until region_clear cell;                       // `until` is optional; the repeat always ends with `;`
    on failure { stop; }                             // optional: stop (default), continue, or retry 3
}
```

```rob
robolang 2.0;

task round(robot r, location home, location far) {
    goal { robot_at r home; }
    plan { move r to far; wait for 5 s; move r to home; }
}

mission shuttle(robot r, location home, location far, region quay) {
    repeat at most 12 runs, for at most 480 min {
        run round(r, home, far);
    } until region_clear quay;
    on failure { continue; }
}
```

- **Parameters and `run`.** A mission has typed parameters like a task. `run round(r, home, far);` starts a run of a task of
  the file, with one parameter of the mission for each parameter of the task, in order and of the same type (`E326`
  for a task that is not in the file, a mission or the wrong number of arguments; `E303`, `E304` for the names and
  types). Bind the **mission's** parameters to world entities, as for a task. Several `run` lines may stand in one
  `repeat`: they run in order, then again, and each run counts towards the bound ("carry it there, then carry it back, 24
  carries in all" is two `run` lines and `at most 24 runs`).
- **Every repeat has a hard bound** (`E325` with none): `at most N runs` (1 to 1000), `for at most D` (a duration in `ms`,
  `s` or `min`: **no hours, 3 hours are `180 min`**), or an `until clock_time >= T` (a time of day, "until 22:00"), alone or
  with a count. Give exactly the bounds the request states: a count when it says "30 times", a duration when it says
  "for 2 hours", both when it says both, and never a count that was not asked for (a duration-only request is
  `for at most 120 min` with no `at most`).
- **`until`** takes the conditions of a `pre` (`region_clear r`, `object_at o l`, `robot_at`, `holding`, `object_exists`,
  `in_state`, with `and`, `or`, `not`) over the mission's parameters, or a comparison with a live reading or the
  clock like `wait until` (`until temperature of pan using probe < 40 degC;`, `until clock_time >= 22:00;`). It is
  checked after each run, and the mission ends when it holds. A value that cannot be read, or a fact the world lacks,
  stops the mission (`S402`); the mission never goes on as if it were false. A `measure` name is not available in a
  mission (`E317`).
- **`on failure`** says what follows a run that failed. `stop;` (the default, when the request says nothing about
  failures) ends the mission as failed; `continue;` records the failed run and goes on with the next ("note it and go
  on", "do not stop the job"); `retry N;` runs the same task again, at most N more times, and the mission fails when
  they are used up ("run it again up to 12 more times"). Retries count against the bound.
- **A mission with no `repeat`** is one run, with `on failure { retry N; }`: "carry it, and if it fails, again, up to 12
  more times, then stop" is `run transfer(...);` and `on failure { retry 12; }`. It has exactly one `run` line.
- **Time.** All the runs share one world and one clock: an object stays where a run left it, a wait in a task advances
  the mission's time, and a world event is seen when the time reaches it. The duration is checked **before** each run
  and a run that has started is never cut: 10-minute runs in 3 hours are 18 runs. A task whose waits alone are longer than
  the duration cannot run even once, so the request is infeasible (`S401`): "12 rounds of 5 minutes within 30 minutes".
- **How it ends.** `completed` (the `until` holds, or the one run was done), `bounded` (a count or duration was reached),
  `failed` (a run failed under `stop`, retries were used up, every run failed under `continue`, or the `until` is
  unknown) or `stopped` (an operator stop or a lease end). A mission never claims more than its runs verified.
- **A robot never receives a mission:** the planner runs it, and each task run goes to the robot as an ordinary task.
- **More than 10 repeats, or repeating until something holds, is a mission.** "Up to 15 attempts", "until the door is
  open" or "until the fence is clear" is a `mission` with that bound and `until`: do not write it as one task with the
  steps or `if not ... { ... }` copied many times. Copies are only for a fixed count of 10 or fewer.
- **A fixed count needs no mission.** "Three round trips" is one task with the steps written out, a program (a mission
  with `at most 3 runs` is also accepted); "retry the grasp up to twice" is a failure handler (see "Failure handlers"):

```rob
robolang 2.0;

task three_round_trips(robot r, location a, location b) {
    goal { robot_at r a; }
    plan { move r to b; move r to a; move r to b; move r to a; move r to b; move r to a; }
}
```

- **Not in the language** (decline as `inexpressible`): **no bound** ("forever", "keep going", "until it works",
  "until I tell you to stop", "as long as it takes": do not invent one); **triggers** ("every hour", "every morning",
  "every 10 minutes", "at 06:00", "each time a message arrives", "when the oven is hot": a mission starts a run when the
  last one is done, never at a time or on an event, and "wait 60 min" at the end of a task is a different request);
  **nesting** (a loop inside a loop, a mission that runs a mission); **choosing the next object at run time** ("every
  part that arrives", "the next blank": a mission repeats a task whose parameters are fixed). A request whose count is
  small and fixed is **not** one of these: "three round trips" is one task with the steps repeated, and "retry the
  grasp up to twice" is a failure handler; neither is declined. A request that contradicts itself ("12 runs in all, and
  20 retries on top of the first") or cannot fit its own time is `infeasible`.

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
- `do r ACTION TARGET` requires `r` to be ready, to list the action in the world, and to be at the target (a location, or the
  location of the object). It sets the state of the target to the result the action lists, records `requested` and
  changes nothing else. It takes no time.
- `measure r time` and `measure r elapsed` read the world's clock, take no time and need no sensor; `wait until
  clock_time >= T` ends at the exact time `T`.
- `communicate` records a message and changes no other state.
- A task can bind more than one robot. Their steps still run sequentially in written order.

Validation checks source structure, names, and types. World-specific capability and state failures
appear when the task is checked against a world or simulated. A run fails with `S261` when the
condition of an `if` is unknown, and with `S262` when it reaches a `fail`. Common source diagnostics
are:

| Code | Meaning |
|---|---|
| `E105` | a time of day that is not `HH:MM` or `HH:MM:SS` in 24-hour form (`24:00`, `9:30`, `21:60`) |
| `E201` | a handler after `wait for`, `communicate` or `fail` |
| `E202` | missing `;` |
| `E206` | missing or unknown unit, or a number in a comparison with no unit |
| `E208` | task blocks in the wrong order, an empty branch or handler, a `goal` block that is not first in a branch, or a `goal` block in a handler |
| `E212` | reserved word used as a name |
| `E214` | `and` and `or` mixed without parentheses |
| `E217` | a kind that is not `temperature`, `mass`, `distance`, `label`, `time` or `elapsed` |
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
| `E319` | kinds that do not match: a unit of another kind, a string against a quantity, a time against a duration or an elapsed value, a retry of another kind |
| `E320` | `==` or `!=` on a quantity, a time or an elapsed value, or ordering on a label |
| `E321` | a comparison in `pre` or `goal` |
| `E325` | a `repeat` with no hard bound (no count, duration or time of day to end at), or a count over its cap (1000 runs, 100 retries) |
| `E326` | a `run` of something that is not a task of the file (a mission, an unknown name), or with the wrong number of arguments |
| `S401` | a mission's task cannot fit its duration bound even once: its waits alone are longer |
| `S402` | a mission's `until` is unknown after a run: the mission stops |
| `E322` | a live reading or `clock_time` / `clock_elapsed` anywhere but the condition of a `wait until` |
| `S010` | `wait until clock_time >= T` where T has already passed when the task starts |
| `S237` | a `time` reading and the world has no clock |
| `S238` | a `time` reading after the clock has reached the end of the day the task started on |
| `S006` | a `do` names an action the robot does not list in the world |
| `S007` | the target of a `do` is an object where the action acts on a location, or the reverse |
| `S008` | a `do` gives a value the action does not take, no value where it takes one, or a value of the wrong kind |
| `S009` | the goal needs an `in_state` that no action of the task's robots leaves, and the world does not already hold |
| `S271` | `do`: the robot is not at the target location, or at the target object's location |
| `E323` | a region named by more than one region limit, or as both `no_go_region` and `in_region` |
| `E324` | more than 8 region limits in a task |
| `S003` | a robot that moves does not honour the kind of region limit (its `honours` in the world) |
| `S004` | a target (a `move` or `place` location, or a `robot_at` goal) is inside a `no_go_region`, or outside an `in_region` whose locations the world lists |
| `S005` | a robot that moves starts inside a `no_go_region`, or outside an `in_region` whose locations the world lists |
| `S234` | `measure`: the sensor does not measure that kind |
| `S235` | `measure` with a `contact` sensor: the robot is not at the object |
| `S236` | `measure`: the object has no value of that kind |

## Complete example

This task waits for a cell (and stops if it does not clear in time), inspects a box, moves it, and
records completion. Its parameters still
need bindings to concrete world entities before simulation.

```rob
robolang 2.0;

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
robolang 2.0;

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
robolang 2.0;

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
robolang 2.0;

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
