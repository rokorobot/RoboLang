# RoboLang 2.0: the mission layer

**Status:** front end, Mission IR 1.0, simulation (mission run, mission run record 1.0), `simulate --mission` and the IR
reader complete in code. The language reference and the benchmark prompts teach 2.0 (L4 of decision 0041); the
acceptance run on benchmark set 9 is the next step (L5), and until it passes the bounds B1 to B9 2.0 is not accepted.
Decision: [0041](decisions/0041-mission-layer.md) (M1 to M10, and clarifications 1 to 14 approved 2026-11-18).
Grammar: [grammar-v2.0.ebnf](grammar-v2.0.ebnf). IR: [Mission IR 1.0](mission-ir-v1.0.md), next to
[Task IR 1.8](ir-schema-v1.8.md). Simulation: [section 18](simulation-v1.0.md). Everything not stated here is exactly
as in [RoboLang 1.9](robolang-v1.9.md): a task is unchanged (M1).

## 1. Version

`robolang 2.0;` selects 2.0, because it adds a layer (M10). A file with this marker may hold **tasks**, read with the
1.9 rules (every 1.9 construct is allowed, with unchanged meaning), and **missions**. A 1.9 front end rejects it with
`E213`. Files with no marker or with `1.1` to `1.9` are read, validated, compiled and run exactly as before, byte for
byte. In a 1.9 file `mission` is a name, so a mission there is `E201` ("expected 'task', found 'mission'").

**Seven new reserved words** (B7): `mission`, `repeat`, `run`, `runs`, `stop`, `continue`, `retry`. In a 2.0 file a
parameter or task named with one of them is `E212`. `until`, `for`, `at` and `else` are reserved already; `most`, `on`
and `failure` are recognised by position and are not reserved. One top-level declaration is new: `mission`.

## 2. A mission

A **mission** runs tasks, one run after another. Every task run is a complete, bounded, verified run exactly as today:
its own `pre`, its limits (1.7), its steps and its verified goal. The mission decides only whether to start the next
run. It runs on the planner's side; a robot is never sent a mission (M8, section 7).

```rob
robolang 2.0;

task round(robot r, location home, location far) {
    goal { robot_at r home; }
    plan { move r to far; wait for 5 min; move r to home; }
}

mission patrol(robot r, location home, location far, region quay) {
    repeat at most 12 runs, for at most 480 min {
        run round(r, home, far);
    } until region_clear quay;
    on failure { continue; }
}
```

```
mission NAME(parameters) {
    repeat [at most N runs] [, for at most D] {
        run TASK(arguments);
        ...
    } [until CONDITION] ;
    [on failure { stop; | continue; | retry N; }]
}
```

- **Parameters** are typed like a task's (`robot`, `object`, `location`, `region`, `sensor`) and bound to world
  entities as a task's are (clarification 3). An **argument** of `run` is a parameter of the mission, with the type of
  the task's parameter in that place (`E303`, `E304`). A value that is chosen at run time ("the next blank") is not 2.0
  (G-CHOOSE).
- **`run TASK(args);`** starts a run of a task of the same file. Several `run` lines may stand in one `repeat`; they
  run in order, once each per turn of the repeat, and **each task run counts towards the bound** (clarification 1).
  `run` names a task: a name that is not a task, a mission (a mission cannot run a mission, M7), or a call with the
  wrong number of arguments is `E326`.
- **`repeat`** runs the `run` lines again and again until the mission ends (section 4). **Every repeat has a hard bound**
  (M2): at least one of
  - `at most N runs`: N from 1 to 1000 (`E305` for zero, `E325` for more);
  - `for at most D`: a duration in `ms`, `s` or `min`. **There is no hour unit** (clarification 12): 8 hours are
    `480 min`;
  - an `until` of the form **`clock_time >= T`** (or `>`), alone or as an operand of an `or`: a time of day is a bound
    (clarification 2), because a 1.9 time lies on the day the mission starts.

  Both counts and the duration may be given; the mission ends at the first bound reached. A repeat with none is
  `E325`. "Forever" is inexpressible. Whatever the bounds say, no mission makes more than 1000 task runs in one run of the
  simulation (a mission whose tasks never wait, with only a duration, would never reach its bound: 1000 runs end it
  `bounded`).
- **`} until CONDITION;`** (the repeat always ends with `;`, with or without an `until`) is checked **after each task run that verified**, in the order of the runs, and ends the
  mission `completed` when it holds (M3). The condition is a fact condition as in a `pre` (`region_clear`,
  `object_at`, `robot_at`, `holding`, `object_exists`, `robot_ready`, `sensor_ok`, `in_state`, with `and`, `or`,
  `not`) over the parameters of the mission, **or a comparison with a live reading or the live clock**, read as in a
  `wait until` (clarification 2): `until temperature of pan using probe < 40 degC;`, `until clock_time >= 22:00;`. A
  comparison with a `measure` binding is `E317`: a mission has none. A value that cannot be read, or a fact that is
  unknown, makes the condition **unknown, and the mission stops** (fails closed, `S402`).
- **`on failure`** says what happens after a task run that failed (M4). It is optional: the default is `stop`.
  - `stop;` ends the mission `failed`.
  - `continue;` records the failed run, never counts it as done, and goes on with the next run line.
  - `retry N;` runs the same line again, at most N more times (N from 1 to 100, `E305`, `E325`). A run that succeeds
    ends the retries and the mission goes on. When the retries are used up the mission ends `failed`. **Retries count
    against the bound** (clarification 7).
- **A mission without `repeat`** has exactly one `run` line (`E201` for a second one) and is a single run. With
  `on failure { retry N; }` it is the way to say "carry it, and if it fails, again, up to N more times"
  (clarification 5). A bounded number of retries of one *step* inside a task is a 1.5 failure handler, not a mission.
- **No nesting** (M7): a `repeat` holds `run` lines only (`E201`), and a mission cannot run a mission or be started by
  a task (`E326`).

A fixed count of repeats ("three round trips") is written as one task with the steps repeated (a 1.4 plan has no loop;
ten is the longest chain in the examples), or as a mission with that run count: both are accepted (clarification 10).

### 2.1 What 2.0 cannot say

- **No bound at all:** "forever", "keep going", "until it works" with no count or time. Every repeat has a hard bound
  (M2); a request without one is declined as inexpressible, never given a bound that was not asked for.
- **Triggers** (M6): "every hour", "every morning", "at 06:00", "on a message", "when the oven is hot". A mission starts
  a run when the previous one is done, not when something happens or a time comes. `repeat ... wait 60 min` is a different
  request (clarification 13). Time, message and self-state triggers are later iterations (atlas gap G-TRIGGER).
- **Nesting** (M7): a loop inside a loop, a mission that runs a mission, a task that starts a mission.
- **Choosing the next object at run time** (G-CHOOSE): "every part that arrives", "the next blank", "each tote in the
  list". A mission repeats a task whose parameters are fixed.

## 3. Static rules (`validate`)

1. A task and a mission cannot share a name, and a mission's parameters are unique (`E301`, `E302`).
2. Each `run` names a task of the file and gives it the right number of arguments (`E326`), each a parameter of the
   mission (`E303`) with the type the task's parameter has (`E304`).
3. A repeat has a hard bound (`E325`); a count is positive (`E305`) and at most 1000 (runs) or 100 (retries) (`E325`).
4. The `until` is well formed: its atoms name parameters of the right types (`E303`, `E304`, `E310`), and the kind
   rules of 1.6 apply to a comparison (`E317` for a binding, `E319`, `E320`).
5. A duration bound is positive (`E305`) and has no hour unit (`E206`).

## 4. Outcomes (M5, clarifications 5 and 6)

A mission ends in exactly one of four states, and says why (`ended_by`). It never claims more than its runs verified.

| Outcome | When | `ended_by` |
|---|---|---|
| `completed` | the `until` holds after a run that verified; or, with no `repeat`, the single run (or one of its retries) verified | `until`, `run_verified` |
| `bounded` | a bound (run count, duration) was reached, and at least one run verified | `run_bound`, `duration_bound` |
| `failed` | a run failed under `stop`; the retries are used up; the bound was reached while a retry was pending; every run failed under `continue` (never hidden as `bounded`); or a task cannot fit the duration bound even once (`S401`, before any run) | `run_failed`, `retries_used`, `bound_with_retry`, `all_failed`, `cannot_fit` |
| `stopped` | the `until` is unknown (`S402`); an operator stop or a lease end (M2) stops a mission at any time | `until_unknown` |

A mission with a count bound alone, reached, ends `bounded`, not `completed`: "do it 30 times" that was done is
`bounded` (clarification 5).

## 5. Time and state (clarifications 8 and 9)

- **One world, continuous time.** Objects stay where a run left them, the simulated time goes on from run to run, and
  the world's events are on **mission time**: an event at 25 minutes is first seen by the run that ends at or after it.
  `elapsed` of a task is the time since *that run* started; `clock_time` is the world clock plus the mission time, so a
  run that starts after the day has ended fails its readings (`S238`), and a wait for a time already passed when the run
  starts fails before it (`S010`).
- **The duration bound** is checked **before each run**, and a started run is never cut. 10-minute runs in 3 hours are
  18 runs. **A task that cannot fit even once** (its `wait for` steps alone are longer than the bound, on every path that
  can succeed) fails the mission before any run with `S401`.
- A run is a run of the simulation, with its own `pre` check, its limits and its goal, and its own run record.

## 6. Task IR and Mission IR

A 2.0 file of tasks only compiles to Task IR 1.8 with `language_version` `2.0`. A file with a mission compiles to a
**Mission IR 1.0** document, which holds the tasks (as Task IR 1.8) and the missions: [mission-ir-v1.0.md](mission-ir-v1.0.md).
The reader re-validates it in full: [ir-reader-v0.1.md](ir-reader-v0.1.md).

## 7. Elsewhere (M8, M9)

- **Core and the backends are unchanged.** A robot never receives a mission. `export-goal` and `delegate` refuse a mission
  with **`X607`** (exit 6). **`delegate` of a mission is refused, not built:** per-run delegation would need the world,
  the clock and the lease to carry from one delegated run to the next, and a signed exchange per run; today's
  `delegate` builds each exchange from a world file (owner decision: see the report of iteration I4). A mission's tasks
  can be exported or delegated one by one. The ROS 2 backend refuses a mission with **`S514`**.
- **Safety** (M9): each task run keeps its own `pre` checks, limits and verified goal. **The bound and the stop are part
  of the signed approval** (0038): an approval names the mission and its bound, so a mission cannot run past what was
  approved. Hazard **H22**, "a mission runs longer or more often than approved", in the
  [safety case](../docs/safety-case-draft.md).

## 8. Diagnostics

| Code | Stage | Meaning |
|---|---|---|
| `E201` | syntax | (existing) a malformed mission: no `repeat` or `run`; a second `run` without `repeat`; a `repeat` inside a `repeat`; `forever`; a trigger word |
| `E206` | syntax | (existing) a duration bound with an unknown unit, such as `h` |
| `E208` | syntax | (existing) an empty `repeat` |
| `E212`, `E301`, `E302`, `E303`, `E304`, `E305` | static | (existing) a reserved word as a name; duplicate names; undeclared or mistyped arguments; a count that is not positive |
| `E317`, `E319`, `E320` | static | (existing) a binding in an `until`; kinds that do not match; `==` on a reading |
| `E325` | static | A repeat has no hard bound, or a count is over its cap (1000 runs, 100 retries) |
| `E326` | static | A `run` names no task (unknown, or a mission), or gives it the wrong number of arguments |
| `S401` | mission | A task cannot fit the duration bound even once: the mission fails before any run (clarification 8) |
| `S402` | mission | The `until` condition is unknown: the mission stops, fails closed (M3) |
| `X607` | export, delegate | A mission is never sent to a robot (M8): `export-goal` and `delegate` refuse it |
| `S514` | ROS 2 backend | A mission is refused before any motion (M8) |

`E213` lists `1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9 and 2.0`. In the Task IR reader the new forms are `I226`
(`E325`) and `I227` (`E326`), and the existing `I201` to `I205` and `I218` to `I223` apply to missions.

## 9. Compatibility

Nothing changes for 1.1 to 1.9 files, their IR, their run records, or their world files. A world file 1.0 to 1.5 may
serve a mission: its `tasks.<mission>.bindings` (or `--bind`) binds the mission's parameters, and the world has no new
field. A world's scripted `faults` and `cancel` belong to a task's steps and have no meaning for a mission.
