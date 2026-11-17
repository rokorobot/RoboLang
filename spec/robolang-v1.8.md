# RoboLang 1.8 (delta over 1.7)

**Status:** front end, Task IR 1.7, simulation (world file 1.4, run record 1.7), `validate --world` and the IR reader
complete in code. The shipped language reference and the benchmark prompts teach 1.8 (L4 of decision 0040); the
acceptance run on benchmark set 7 is the next step (L5), and until it passes the bounds B1 to B8 1.8 is not accepted.
Decision: [0040](decisions/0040-declared-actions.md) (A1 to A11 and C1 to C15, approved 2026-11-17). Grammar:
[grammar-v1.8.ebnf](grammar-v1.8.ebnf). IR: [schema 1.7](ir-schema-v1.7.md). Simulation:
[section 16](simulation-v1.0.md). Everything not stated here is exactly as in [RoboLang 1.7](robolang-v1.7.md).

## 1. Version

`robolang 1.8;` selects 1.8. A file with this marker is read with the 1.7 rules, plus this document. A 1.7 front end
rejects it with `E213`. Files with no marker or with `1.1` to `1.7` are read, validated, compiled and run exactly as
before, byte for byte (Task IR 1.0 to 1.6, run records 1.0 to 1.6). A `do` in a file of those versions is still an
error (`E203`, an unknown statement), and `in_state` is `E204`, an unknown condition.

**Two new reserved words** (decision 0040 A1, A2): `do` and `in_state`. In a 1.8 file a parameter named `do` is
`E212`. An action name is *not* reserved: it is a name the robot's profile declares.

## 2. Declared actions

A robot's profile **declares** the actions it can do: for each, its name, what it acts on, the state it leaves that
thing in, and optionally the kind of value it takes (world file 1.4, [simulation section 16](simulation-v1.0.md)). A task
asks for an action with one statement, `do`, and states the result it needs with one condition, `in_state`. A new
action needs a profile entry, never a new keyword (A1).

```
do r ACTION TARGET;
do r ACTION TARGET to VALUE;
in_state TARGET "label"
```

```rob
robolang 1.8;

task restock(robot r, object closet, object kit, location ward) {
    goal { in_state closet "open"; object_at kit ward; }
    plan {
        do r open closet;
        grasp r kit;
        move r to ward;
        place r kit at ward;
    }
}
```

- **`do r ACTION TARGET;`** (A1, A4). `r` is a `robot` parameter. `ACTION` is a name, not a keyword: whether the robot
  declares it is checked against the world, not by the grammar (names match exactly, C12). `TARGET` is **one**
  `object` or `location` parameter, of the kind the action declares. Several targets are not in 1.8 (A10).
- **`to VALUE`** (A5): a quantity with a unit (`175 degC`, `2 kg`, `30 cm`, the units of 1.6) or a string, when the
  action declares a `value` kind. The value is recorded in the run. Reaching it is checked with `measure` or
  `wait until`, never assumed.
- **A `do` is a step like `move`:** it takes a failure handler (`do r open closet else { ... }`, 1.5), it may stand in
  an `if` branch or a handler, and a scripted fault (C6) fails it.
- **The robot must be at the target** (C2): at the target location, or at the location of the target object (or
  holding it), exactly as for `grasp`. Otherwise the step fails (`S271`). Remote actions are not in 1.8.
- **`in_state X "label"`** (A2) is a condition, true when the observed state of the object or location `X` is that
  label. It works in `pre`, `goal`, `if` and `wait until`, with `not`, `and` and `or`. It is **false** when the state
  is another label and **unknown** when the world gives `X` no state, and unknown fails closed as every unknown
  condition does (`S102`, `S261`, a `wait until` that keeps waiting, `S302`).
- **A `do` is a request, not an effect** (A7, A9). In the simulation it sets the target's state to the declared
  `result`, and the run record says `requested`. Whether the effect happened counts only through `in_state` in the
  goal. RoboLang does not know what "open" does physically: the integrator's profile entry is the only description of
  it, which is why a program that asks for an action needs the `do` **and** the `in_state` that verifies it.
- **What an action task's goal contains** (C3): the requested `in_state` plus the other requested end conditions; no
  `robot_at` unless the request asks for it.
- **Regions** (C13): a `do` on a location counts as a target for `S004`, as a `move` or `place` target does. The robot
  has to move there (C2), so the checks of 1.7 apply as they do to `move`.
- **Charge and dock** (C14): in 1.8 the state is on the charger or the dock, a location target. The robot's own state
  is G-SELF.
- **Not in 1.8** (A10): several targets (cooperation, G-PROX), actions that return values (that is `measure`), actions
  defined in RoboLang from other steps (macros, G-TEACH), and durations of an action (`do` and then `wait for`).

## 3. Static rules (`validate`)

1. The robot is a `robot` parameter (`E303`, `E304`). The target of `do` and of `in_state` is a parameter of type
   `object` or `location` (`E303`, `E304` with a message that says "an object or a location").
2. `ACTION` is a name: a reserved word there is `E212`. A value of `to` is a quantity with a known unit (`E206`) or a
   string; a string value or a state label is not empty (`E307`).
3. A `do` may take a failure handler. `in_state` takes no binding and no value: only an object or a location and a quoted
   label.

What the grammar cannot know (the robot's profile) is checked against the world, before the run (section 5).

## 4. Task IR

A `do` is a step and `in_state` a condition: [IR schema 1.7](ir-schema-v1.7.md). A 1.8 source produces schema 1.7.

## 5. Checks against the world

Before the run reaches `AUTHORIZED`, and in `robolang validate --world`, the task and the world are checked against
each other (A6, C7), as speed and region limits are. A failed check stops the run before anything moves.

| Code | Meaning |
|---|---|
| `S006` | A `do` names an action the robot does not declare (a world file 1.3 or older robot declares none) |
| `S007` | The target of a `do` is not of the kind the action acts on (an object action on a location, or the reverse) |
| `S008` | The value of a `do` is missing, given to an action that takes none, or not of the declared kind (`2 kg` for a temperature) |
| `S009` | A goal `in_state` that no declared action of the task's robots produces and that the world does not already hold (C7) |

## 6. Diagnostics

| Code | Stage | Meaning |
|---|---|---|
| `E203` | static | (existing) `do` in a file older than 1.8 is an unknown statement |
| `E204` | static | (existing) `in_state` in a file older than 1.8 is an unknown condition |
| `E303`, `E304`, `E307`, `E206`, `E212` | static | (existing) the names, the empty label or value, the unit of the value, a reserved word as an action name |
| `S006`, `S007`, `S008`, `S009` | authorization | Section 5 |
| `S271` | step | `do`: the robot is not at the target (C2) |
| `E517` | world file | World file 1.4: an unknown `target` or `value` kind in a declared action |
| `X606` | export | `export-goal` and `delegate`: the goal has an `in_state`; Core 0.2 has no state triples, so the task is refused (A8) |
| `S508` | ROS 2 backend | (existing) a `do` or an `in_state` is refused before any motion (A8) |

`E213` now lists `1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7 and 1.8`. `delegate` also refuses, among the plans of the robot's
library, one whose goal has an `in_state`, with the plan result `in_state_unsupported`.

## 7. Elsewhere (A8)

- **ROS 2 / Gazebo backend:** refuses a task with `do` or `in_state` with `S508` before any motion: it can neither run
  nor observe them ([ros2-backend-v0.1.md](ros2-backend-v0.1.md)).
- **`export-goal`, `delegate`:** Core 0.2 has no state triples, so a goal with `in_state` is **refused** with `X606`,
  never sent without it. Revisit in Core 0.3, with G-CHOOSE (I7).
- **Safety** (A9): an action is a request to the robot's own controller. The safety case has the hazard H20, "a
  declared action does more than its name says", controlled by the integrator's declaration and the signed task
  approval. The region and speed limits of 1.7 cover every motion, actions included.

## 8. Compatibility

Nothing changes for 1.1 to 1.7 files, their IR, their run records, or their world files 1.0 to 1.3. A world file 1.4
may serve a task of any language version; its `actions`, `state` and `location_states` are read only by a task with a
`do` or an `in_state`. A task with a `do` run against a world 1.0 to 1.3 fails closed with `S006`: those robots declare no
actions.
