# RoboLang held-out set (private)

*Template. Copy this folder to start the private repository:
`cp -r RoboLang/bench/heldout/template RoboLang-heldout`. Then delete the `x0*` and `garage` example files
before adding real ones.*

This repository holds the held-out benchmark set for [RoboLang](https://github.com/rokorobot/RoboLang). The rules
are in RoboLang's [`bench/heldout/README.md`](https://github.com/rokorobot/RoboLang/blob/main/bench/heldout/README.md).
In short:

- **Access:** the owner and the person who checks labels. Nobody else.
- **Never** open, paste or upload anything from here into an AI tool, a cloud agent workspace, an issue, a PR or
  public CI.
- Only `manifest.sha256`, `PREREGISTRATION.md` and totals from `reports/` go to the public repo.

## Layout

| Path | What | Frozen |
|---|---|---|
| `scenes/<world>.md` | the plain-English scene sent to the writers | no |
| `worlds/<world>.json` | the world file | yes |
| `worlds/<world>.notes.json` | optional: entity ID → short description, shown to the model under the world | yes |
| `raw/` | the writers' files exactly as received, plus `dropped.md` | no |
| `tasks/<id>.json` | one task per request | yes |
| `reference/` | the hand-written programs for each task | yes |
| `synthetic/tasks/`, `synthetic/reference/` | optional model-written tasks, same formats, scored separately | no |
| `results/<run>/`, `reports/<run>.md` | raw answers; totals | no |
| `manifest.sha256`, `PREREGISTRATION.md` | the freeze and the targets | no |

The `x001`–`x003` tasks and the `garage` world are worked examples of every file.

## 1. Scene and world

Write the world first, then describe it in `scenes/<world>.md` in plain words. The scene must say only what the
world file says, because writers can only ask about what they're told. Format: see `scenes/garage.md`.

**What a world can express** (world file 1.1, [spec](https://github.com/rokorobot/RoboLang/blob/main/spec/simulation-v1.0.md#3-world-file-schemas-10-and-11)):

| Entity | Fields |
|---|---|
| `locations` | a list of IDs. Every robot can move to every location |
| `regions` | `clear`: true or false (an area a task can wait for or must avoid) |
| `robots` | `location`, `gripper` (true or false), `ready` (true or false), `max_speed` in `m/s` and/or `rad/s` |
| `objects` | `present`, `location` or `held_by`, `class` (a lowercase word such as `toolbox`) |
| `sensors` | `robot` it is mounted on, `available` (true or false) |
| `events` | at a time, set `robot_ready`, `region_clear` or `sensor_available` to true or false |

IDs are letters, digits and `_`, starting with a letter, unique across all entities. Leave `tasks` as `{}`.

**What a world cannot express:** weight or payload, reach, size, colour (except through the ID, `class` or notes),
battery, distance, people, and anything else not in the table. Keep those out of the scene. If a writer still asks
for one, the task is `inexpressible` (see 3).

**Good worlds for testing** have look-alike things (two toolboxes), a robot without a gripper or not ready, a
broken sensor, an absent object (`"present": false`), and a region that becomes clear later. Make the worlds new:
not copies or renames of RoboLang's `bench/worlds/`.

Check a world by simulating any program in it (see 5).

## 2. Raw requests

Save each writer's file in `raw/` as received (`raw/w1.xlsx`, ...). Never edit them. Writers are `w1`, `w2`, ...
everywhere, with no names or contact details in this repository. Every request either becomes a task or gets a line
in `raw/dropped.md`.

## 3. Tasks

One file per request: `tasks/h001.json`, `h002.json`, ... The `prompt` is the writer's text **exactly**, with
typos. Fields for every task:

| Field | Content |
|---|---|
| `id` | the file name without `.json` |
| `source` | `"human"`; synthetic tasks: `"synthetic-<provider>"` |
| `writer` | `"w1"`, ...; for synthetic tasks the model ID |
| `prompt` | the request, verbatim |
| `world` | `"worlds/<world>.json"` |
| `group` | `feasible`, `decoy` (looks impossible or unclear but is feasible), `clarify`, `infeasible` or `inexpressible` |
| `expect` | `{"decision": "program" \| "clarify" \| "cannot", "category": "none" \| "infeasible" \| "inexpressible"}` |
| `tags` | short labels for the report, e.g. `pick_place`, `safety`, `wait_until`, `sensor_unavailable` |

Then, by decision:

**`program`** (`feasible`, `decoy`; example `x001`)

| Field | Content |
|---|---|
| `expected_goal` | what must be true at the end, as Core triples: `["rl:thing:<object>", "located_in", "rl:place:<location>"]`. Several triples mean all of them |
| `expected_goals` | instead, when several readings are acceptable: a list of such goals |
| `required_entities` | the entities the request names that the program must actually use, e.g. the robot |
| `expected_safety` | optional: limits the request asks for, e.g. `[{"kind": "max_speed", "value": "1.5", "unit": "m/s"}]` |

Reference: `reference/<id>.rob` and `<id>.json` (`{"task": ..., "bindings": {...}}`), which must end `SUCCEEDED`.

**`clarify`** (example `x002`): the request could mean several entities, or leaves out a value, and the world
doesn't decide.

| Field | Content |
|---|---|
| `slot` | what's missing: `object`, `robot`, `location`, or a value such as `speed` |
| `candidates` | the entity IDs it could mean |

References: one program per reading, `<id>.reading1.rob` + `.json`, `<id>.reading2.rob` + `.json`, ..., each ending
`SUCCEEDED`. If the request leaves the choice to the robot ("one of the toolboxes"), it's `feasible`, not `clarify`.

**`cannot`, `infeasible`** (example `x003`): RoboLang can express it, but this world can't do it.

| Field | Content |
|---|---|
| `why` | one sentence, with the diagnostic code |
| `blockers` | the entity IDs that make it impossible. Empty when the request names something that doesn't exist |
| `required_entities` | the entities the request names |
| `faithful_goal` | the goal the request would have, as triples |
| `fix_patch` | a change to the world under which the attempt would succeed, or a `why` saying none can |
| `attempt_fails_with` | the code the obvious attempt fails with, e.g. `S232` (sensor unavailable), `S201` (robot not ready) |

Reference: `<id>.attempt.rob` + `.json`, the program a careless model would write. It must fail with
`attempt_fails_with`.

**`cannot`, `inexpressible`**: the world might allow it, but RoboLang can't say it (a weight limit, a loop, "as
fast as possible"). Fields `why` (with the language feature that's missing) and `tags`. Add `<id>.attempt.rob` only
when an obvious attempt exists. Check `robolang reference` first: many things that sound inexpressible are not.

The public sets in RoboLang's `bench/tasks_ask/` and `bench/reference_ask/` have 80 more examples of these formats.
Reading the public sets is fine. Copying their requests is not.

## 4. Labelling rules

- Label what the writer meant. Use their "what done looks like" note, not what would be convenient.
- Don't fix wording, spelling or grammar.
- Drop a request only when its meaning can't be recovered. Record it in `raw/dropped.md`.
- A second person checks every task against the writer's note and signs off in `PREREGISTRATION.md`.
- Do it by hand, with `robolang validate` and `robolang simulate`. No AI.

## 5. Check before the freeze

From this folder, with RoboLang installed (`pip install robolang`):

```bash
robolang validate reference/x001.rob
robolang simulate reference/x001.rob --world worlds/garage.json \
    --bind r=rover o=drill src=workbench dst=door          # "state": "SUCCEEDED"
robolang simulate reference/x003.attempt.rob --world worlds/garage.json \
    --bind r=rover o=drill src=workbench dst=shelf s=rover_cam   # "state": "FAILED", "code": "S232"
```

Every `<id>.rob` and `<id>.readingN.rob` must succeed, and every `<id>.attempt.rob` must fail with its code.

## 6. Freeze and run

```bash
sha256sum tasks/* reference/* worlds/* > manifest.sha256
```

Copy `manifest.sha256` and `PREREGISTRATION.md` into RoboLang's `bench/heldout/` and commit them there. After that,
any change to a frozen file is an erratum: record it in `PREREGISTRATION.md` before the next run.

Run only at the milestones listed in `PREREGISTRATION.md`, on the owner's computer. Save the raw answers in
`results/<run>/` and the totals in `reports/<run>.md`. (Running from this folder needs a `--tasks-dir` option in
`bench/run.py`, which is still to be built.)

## Using AI here

AI may help **only** with worlds and scene descriptions, before writers see them. For example: "Invent a small
workplace for two or three robots, in RoboLang world file 1.1 [paste section 1 of this README], with look-alike
objects, one robot without a gripper and one broken sensor. Then describe it in plain English for a non-technical
reader." Never give an AI the writers' requests, the tasks or the references.
