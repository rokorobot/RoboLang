# RoboLang

RoboLang is a task language for robots, written mainly by AI systems and reviewed by people.
A task declares what must be true before it starts, the **goal** it must achieve, the steps to
take, and safety limits. RoboLang checks the task statically and runs it in a deterministic,
symbolic simulation. A run **succeeds only when the goal is verified** in the final world, not
merely because the steps ran.

- **Available today** (package `robolang` 0.3.1 on PyPI): RoboLang 1.1 to 1.6 with a validator,
  Task IR 1.0 to 1.5, a symbolic simulator with reproducible run records, and RoboLang Core 0.1
  and 0.2 messages: `export-goal` sends a task's goal, and `delegate` hands it to a robot and
  accepts success only after checking the signed evidence. All of it runs in simulation.
- **Not available:** physical robot execution, network transport, and real (non-simulation)
  signatures.

> ⚠️ **Simulation only.** Physical robot execution is not implemented or authorized. RoboLang
> does not replace ROS 2, MoveIt 2, Nav2 or hardware safety controllers. Do not run it against
> real hardware.

The [README](https://github.com/rokorobot/RoboLang#readme) has the full overview, command table
and roadmap. A [map of the documentation](overview.md) lists every specification and plan.

## Quickstart

You need Python 3.11 or newer. No ROS 2, no robot, no other packages.

```bash
python -m pip install robolang
robolang reference          # the language reference: everything a task author needs
robolang example            # writes fetch.rob and world.json
robolang validate fetch.rob --world world.json
robolang simulate fetch.rob --world world.json
```

`simulate` prints `simulation only: run sim-…: SUCCEEDED`, then the run record as JSON. If the
goal is false after the last step, the run fails (`S301`) even though every step ran.

## Example task

```rob
robolang 1.6;

task fetch(robot r, object box, location src, location dst) {
    pre { robot_ready r; }
    goal { object_at box dst; }
    plan {
        move r to src;
        grasp r box;
        move r to dst;
        place r box at dst;
    }
}
```

A world file lists the locations, robots and objects and binds each task parameter to one of
them ([format](https://github.com/rokorobot/RoboLang/blob/main/spec/simulation-v1.0.md)).
The language also has typed sensors (1.3), bounded `if`/`else` (1.4), step failure handlers
(1.5) and measured values with units and uncertainty (1.6).

The simulator tracks symbolic facts only: no motion planning, kinematics, collisions or physics.
`@safety` limits are checked against the robot profile and recorded, never enforced. A
simulation result is not evidence that a task is safe on hardware.

## Status

| Area | State |
|---|---|
| Language 1.1 to 1.6, validator, Task IR 1.0 to 1.5 | Delivered; 1.4, 1.5 and 1.6 accepted on benchmark sets 3, 4 and 5 |
| Symbolic simulation (`simulate`, `validate --world`) | Delivered |
| Core 0.1 reference simulation | Implemented, gate 1 signed off 2026-10-27 |
| Core 0.2 | Frozen 2026-10-30, opt-in with `--core 0.2` |
| Gate 2 independent interoperability | Passed 2026-11-10: an independent Core 0.2 implementation passes 25/25 cases |
| `.rob` → Core bridge (`export-goal`, `delegate`) | Delivered in simulation |
| ROS 2 / Gazebo backend | Optional, repository only, simulation only; not part of the installed package |
| Physical execution | Not implemented or authorized; a robot-independent safety case is in draft |

## Where to go next

- [Language reference (1.6)](language-reference.md): what `robolang reference` prints
- [RoboLang 1.6 language specification](https://github.com/rokorobot/RoboLang/blob/main/spec/robolang-v1.6.md)
- [Simulation semantics, world file and run record](https://github.com/rokorobot/RoboLang/blob/main/spec/simulation-v1.0.md)
- [Core 0.2](https://github.com/rokorobot/RoboLang/blob/main/spec/core-v0.2.md) and
  [`delegate`](https://github.com/rokorobot/RoboLang/blob/main/spec/delegate-v0.2.md)
- [Strategy](STRATEGY.md) and [development plan](DEVELOPMENT_PLAN.md)
- [Safety case draft](safety-case-draft.md)
- [Legacy RoboLang v1](legacy-v1.md): the original interpreter, the experimental ROS 2 adapter
  and the v1 PDF guides, kept as history
