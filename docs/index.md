# RoboLang

RoboLang is an open, robot-independent project for expressing, validating and coordinating
robotic tasks, goals and verifiable outcomes across AI systems and heterogeneous robots.

- **Available today:** RoboLang 1.1 language tooling, deterministic validation, Task IR 1.0
  and simulation-only task execution with reproducible run records.
- **Specified and approved, not implemented:** versioned Core semantic contracts, AI-to-Robot
  (A2R) and Robot-to-Robot (R2R) interoperability, observations, capabilities, authorization
  and verified outcomes.

> ⚠️ **Physical robot execution is not implemented or authorized.** RoboLang does not replace
> ROS 2, MoveIt 2, Nav2 or hardware safety controllers. Do not run RoboLang against real
> hardware.

The [README](https://github.com/rokorobot/RoboLang#readme) has the full overview, quickstart
and roadmap. A [map of the documentation](overview.md) lists every specification and plan.

## Quickstart

You need Python 3.11 or newer and Git. No ROS 2, no robot and no other packages.

```bash
git clone https://github.com/rokorobot/RoboLang.git
cd RoboLang
python -m pip install .
robolang validate examples/pick_and_place.rob
robolang inspect examples/pick_and_place.rob --json
robolang simulate examples/pick_and_place.rob --world tests/fixtures/simulation/worlds/base.json
```

- `validate` checks a `.rob` file and prints diagnostics with `file:line:column`.
- `inspect` validates first, then shows the task as text or as Task IR JSON (`--json`).
- `simulate` runs a validated task in a symbolic, deterministic simulation (no robot, no
  adapter, no ROS 2) and prints a run record. A simulation result is not evidence that a task
  is safe on hardware.

## Example task

```rob
task pick_and_place(robot r, object box, location src, location dst, region cell) {
    @safety max_speed 0.5 m/s;
    pre {
        robot_ready r;
        region_clear cell;
    }
    plan {
        move r to src;
        grasp r box;
        move r to dst;
        place r box at dst;
        communicate r to "fleet" with "TASK_COMPLETE";
    }
}
```

`@safety max_speed` is checked against the robot profile and reported in the run record, never
enforced. The simulator tracks symbolic facts only: no motion planning, kinematics, collisions
or physics.

## Where to go next

- [RoboLang 1.1 language specification](https://github.com/rokorobot/RoboLang/blob/main/spec/robolang-v1.1.md)
- [Simulation semantics, world file and run record](https://github.com/rokorobot/RoboLang/blob/main/spec/simulation-v1.0.md)
- [RFC-001 — AI-native architecture](https://github.com/rokorobot/RoboLang/blob/main/spec/architecture/rfc-001-ai-native-architecture.md):
  the approved, not yet implemented Core, A2R and R2R design
- [Development plan](DEVELOPMENT_PLAN.md)
- [Legacy RoboLang v1](legacy-v1.md): the original interpreter, the experimental ROS 2 adapter
  and the v1 PDF guides, kept as history
