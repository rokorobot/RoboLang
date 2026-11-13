# RoboLang v1 — legacy prototype

> **Historical material.** This page records the original RoboLang v1 prototype: a line-based
> interpreter and a simulated adapter that only prints. Neither is part of the installed
> `robolang` package, and neither passes through the RoboLang 1.1 validator. The experimental ROS 2
> adapter and its entry point, which were never validated and could command a robot without the
> validator, were **removed on 2026-11-13** (safety case H19, gap G8); they remain in git history
> before commit `e566321`. The supported tooling is described in the [README](https://github.com/rokorobot/RoboLang#readme).
>
> **Do not run any of this against real hardware.** Physical robot execution is not implemented
> or authorized.

The remaining files are kept unchanged, as the starting point of the project and the v1 compatibility fixture
(development plan A7):

| File | What it is |
|---|---|
| `legacy/robolang_runtime.py` | Line-based interpreter that scans the `plan { ... }` block of a `.rob` file and calls the adapter |
| `legacy/robolang_adapter.py` | Simulated adapter that prints each action; no ROS 2 required |
| `legacy/robolang_std_v1.rob` | The v1 standard library; one task, `pick_and_place` |

## Known limitations of the v1 interpreter

These are pinned down by `tests/test_known_limitations.py`. They describe the v1 interpreter
only; the RoboLang 1.1 validator and `robolang simulate` behave differently (see the README).

| Area | Status |
|------|--------|
| Instructions `move`, `grasp`, `place`, `inspect`, `wait for`, `communicate` inside `plan { ... }` | Prefix matching only, no grammar |
| `pre { ... }` preconditions | Accepted in source but **not evaluated** |
| `@safety` annotations | Accepted in source but **not enforced** |
| `move` destinations | **Ignored**: every `move` sends the same fixed demo joint values |
| Decimal durations (`wait for 2.5s`) | Truncated to whole seconds |
| Unknown or misspelled instructions | Printed as ignored; execution continues |

## Running the v1 demo

From a clone of the repository (Python 3.11 or newer, no ROS 2, nothing installed):

```bash
python legacy/robolang_runtime.py examples/pick_and_place.rob
```

Expected output:

```text
[RUNTIME] executing: move r to src;
[MOVE] robot_arm_1 → joints=[0.0, -1.0, 1.0, 0.0, 1.0, 0.0] (duration=2.0s)
[RUNTIME] executing: grasp r box;
[GRIPPER] robot_arm_1 → CLOSE (position=1.0, max_effort=50.0)
[RUNTIME] executing: move r to dst;
[MOVE] robot_arm_1 → joints=[0.0, -1.0, 1.0, 0.0, 1.0, 0.0] (duration=2.0s)
[RUNTIME] executing: place r box at dst;
[GRIPPER] robot_arm_1 → OPEN (position=0.0, max_effort=50.0)
[RUNTIME] executing: communicate r to "fleet" with "TASK_COMPLETE";
[COMM] robot_arm_1@fleet → TASK_COMPLETE
```

The demo parses the `plan { ... }` block line by line and calls the adapter primitives
`move_to_pose()`, `set_gripper()` and `communicate()`, which print to the terminal. Both `move`
lines send the same joint values because destinations are ignored.

## Why the v1 code is not installed

Earlier versions of `setup.py` installed the v1 modules (then four, with the ROS 2 adapter) and a `ros2_runtime` console
command. As of distribution `0.1.0a1` neither is installed, because that code acts without
passing through the validator. This was a deliberate compatibility change.

## The original ROS 2 adapter (removed)

The v1 ROS 2 adapter sent `FollowJointTrajectory` and `GripperCommand` goals with the same fixed joint values for
every `move`, and reported success without checking results. It was never validated. It was removed on 2026-11-13
so that no code in this repository can command a robot without passing through the validator. The supported ROS 2
path is the simulation-only Gazebo backend in `backends/ros2` ([spec](../spec/ros2-backend-v0.1.md)).

### Hardware safety notice

RoboLang v1 is a non-safety-certified demonstration and must not be used to move physical
hardware. `@safety` annotations and `pre { ... }` blocks are not enforced by the v1
interpreter. A `@safety` annotation is never a substitute for emergency stops, protective stops
or a robot risk assessment. Hardware execution would require a separate safety review and
explicit owner approval.

## v1 guides (PDF)

These describe the v1 design. Where they disagree with the RoboLang 1.1 specification, the
specification is authoritative.

- [RoboLang v1 Language Manual](RoboLang_v1_Ebook.pdf)
- [Standard Library + ROS 2 Adapter](RoboLang_v1_Library_ROS2_Adapter.pdf)
- [Developer Guide](RoboLang_v1_Developer_Guide.pdf)
- [ROS 2 Integration & Hardware Setup](RoboLang_v1_ROS2_Integration.pdf)
