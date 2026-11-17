# Decision record 0040: Declared actions (I2, G-ACT, RoboLang 1.8)

**Date:** 2026-11-17
**Status:** **APPROVED by the owner, 2026-11-17**. The keywords are `do` and `in_state`; setpoint values (A5) and
location targets (A4) are in 1.8; A1–A11 are as recommended. This is step L2 of iteration I2 in the
[language roadmap](../../docs/LANGUAGE_ROADMAP.md); the owner chose the gap (L1) on 2026-11-17.
**Applies to (when decided):**
- a new `spec/robolang-v1.8.md` and `grammar-v1.8.ebnf`;
- Task IR 1.7;
- world file 1.4;
- `simulation-v1.0.md`;
- `src/robolang/`;
- `docs/language-reference.md`;
- the ROS 2 backend's refusal list;
- `bench/` (the L3 set).

**Evidence:**
- **Atlas.** G-ACT is the first gap in the generated
  [suggested order](../../docs/plans/humanoid-activities.md#suggested-order-of-language-work). Six near
  sub-activities need it: fry stations, hospital delivery, layout printing, EV charging, floor cleaning and
  self-charging. Eight more need it further out, among them laundry, household chores, machine tending and care.
- **R3 probes.** `f_act_set_fryer`, `f_act_open_closet` and `f_act_print_layout`. Each attempt fails (E203), and both
  models declined all three ([evidence](../../docs/evidence/rsi-r3-gap-verification.md)).
- **Standing tasks.** `u15_open_oven`, `u16_charge_drone` and `x06_set_oven` ask for the same thing.

## The problem

A plan has seven statements: `move`, `grasp`, `place`, `inspect`, `measure`, `wait` and `communicate`. Real work
also needs actions like open, set, print, plug in, dock, vacuum and start a program. The list is open-ended: no fixed
set of keywords will cover what humanoids will do. Any new action also has to keep RoboLang's promise, that a run
succeeds only when its goal is **verified**, never merely because a step ran.

## The idea in one paragraph

Each robot's profile **declares** the actions it can do. For each action it gives:
- its name;
- what it acts on;
- the state it leaves that thing in.

A task calls an action with one generic statement, `do`, and states the result it needs with one new condition,
`in_state`. The task and the robot are checked against each other before the run, as speed and region limits are.
The goal still decides success: `in_state closet "open"` must hold at the end, observed and not assumed. A new
action needs a profile entry, never a new keyword. A skill taught to a robot later (G-TEACH) becomes one more
declared action.

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

World file 1.4, robot profile: `"actions": { "open": { "target": "object", "result": "open" } }`.
The closet carries a state: `"state": "closed"`.

## Decisions (recommended option first)

| # | Question | Recommended | Alternatives considered |
|---|---|---|---|
| A1 | Construct | One statement, `do r ACTION TARGET;`. `ACTION` is a name that the robot's profile declares, not a keyword. New keyword: `do` | Fixed built-ins (`open`, `close`, `set`, `dock`, `charge`, `start`, …): 6 or more keywords now and more for every new request, so they never cover the next activity. A free-text `do r "open the closet"`: nothing to check |
| A2 | Verifying the result | A new condition `in_state X "label"`, true when the entity's observed state is that label. It works in `pre`, `goal`, `if` and `wait until`, and is unknown when unobserved, so it fails closed. New keyword: `in_state` | Treat a completed `do` as success: "assumed, not verified", which breaks the north star. Read the state with `measure … label`: a binding cannot appear in a goal (0036) |
| A3 | Declaration (world file 1.4) | A robot profile gets `actions`: name → `{ "target": "object" \| "location", "result": "<label>", "value": "<kind>" (optional) }`. Objects and locations may carry a `state` label | Declare actions in the `.rob` file: the task would describe the robot's abilities instead of asking for them |
| A4 | Targets | One target, an object or a location parameter, of the kind the action declares | Several targets ("hand the wrench to Ana"): that is cooperation with a person (G-PROX, ACT-HRC). Later |
| A5 | Values (setpoints) | An optional value: `do r set fryer to 175 degC;`, when the action declares a `value` kind. It uses 1.6's kinds and units, and kinds are checked as in 1.6 (E319). The value is recorded, and reaching it is checked with `measure` or `wait until`, never assumed | No values: "set the fryer to 175 °C" (`f_act_set_fryer`, `x06`) stays inexpressible |
| A6 | Checks before the run | Static: `ACTION` is a name; the target is an `object` or `location` parameter; a value has a unit. Against the world (`validate --world` and before `AUTHORIZED`): the robot declares the action, the target kind matches, and the value kind matches. A failed check stops the run before anything moves (new S-codes) | Check only at run time: the error would appear mid-run |
| A7 | Simulation | `do` succeeds by setting the target's state to the declared `result`, and the value is recorded. It can fail like any step (a scripted fault), so 1.5 failure handlers can catch it. The run record says `requested`; the effect counts only through `in_state` in the goal | Model what each action does physically: impossible for an open list |
| A8 | Backends and Core | The ROS 2 / Gazebo backend refuses tasks with `do` or `in_state`: it can neither run nor observe them. `export-goal` and `delegate` refuse a goal with `in_state`, because Core 0.2 has no state triples. Revisit in Core 0.3, with G-CHOOSE (I7) | Send `in_state` as an opaque triple: the receiver could not verify it |
| A9 | Safety | An action is a **request to the robot's own controller**. RoboLang does not know what "open" does physically, so the integrator's profile entry is the only description. This adds hazard H20 to the safety case ("a declared action does more than its name says"), controlled by the integrator's declaration and the signed task approval (G6). The region and speed limits of 1.7 cover every motion, actions included | Let RoboLang describe what actions do: a safety function it must not carry (C1) |
| A10 | Not in 1.8 | Several targets, actions that return values (that is `measure`), actions defined in RoboLang from other steps (macros, a later G-TEACH question), and durations of an action ("fry for 3 minutes" is `do` and then `wait for 3 min`) | — |
| A11 | Design for the far needs | G-TEACH: a taught skill is a new profile action, with no syntax change. G-SELF (I8): `dock`, `charge` and `swap_battery` are declared actions, and battery level comes later as a reading. G-LOOP (the mission layer): `do` is an ordinary step inside a mission's tasks | Built-ins would need a language change for every taught skill |

## Pre-registration (L3), before any code

- **Set 7 "actions":**
  - the 3 R3 probes and `u15`, `u16`, `x06`, relabelled for 1.8;
  - new programs: open then fetch, set a value then wait for the reading, print at a location, an action inside a
    branch, an action with a handler, near-miss action names (decoys);
  - infeasible: the robot lacks the action, a wrong target kind, a value of the wrong kind;
  - inexpressible in 1.8: several targets, an action that returns a value, a macro;
  - controls.

  The set is frozen and its baseline run on 1.7 before L4.
- **Bounds:** fixed with the baseline in hand (rule 6), in the style of 0039 B1–B8. They include a B8-like measure:
  **an action replaced silently**, such as a `move` or an `inspect` written in place of the requested action.
- **Economy:** 2 new keywords (`do`, `in_state`), 1 statement, 1 condition, 2 world-file fields and about 5 codes.

## Owner's answers (2026-11-17)

1. The keywords are `do` and `in_state`.
2. Setpoint values are in 1.8 (A5).
3. Locations can be targets (A4).
