# RoboLang v1 — legacy prototype and ROS 2 adapter

> **Historical material.** This page records the original RoboLang v1 prototype: a line-based
> interpreter, a simulated adapter that only prints, and an experimental ROS 2 adapter. None of
> it is part of the installed `robolang` package, none of it passes through the RoboLang 1.1
> validator, and the ROS 2 adapter has **never been validated** against ROS 2, a simulator or a
> robot. The supported tooling is described in the [README](https://github.com/rokorobot/RoboLang#readme).
>
> **Do not run any of this against real hardware.** Physical robot execution is not implemented
> or authorized.

The files are kept in the repository unchanged, as the starting point of the project:

| File | What it is |
|---|---|
| `src/robolang_runtime.py` | Line-based interpreter that scans the `plan { ... }` block of a `.rob` file and calls the adapter |
| `src/robolang_adapter.py` | Simulated adapter that prints each action; no ROS 2 required |
| `src/robolang_adapter_ros2.py` | Experimental adapter mapping actions to ROS 2 (`FollowJointTrajectory`, `GripperCommand`, topics) |
| `src/robolang_ros2_main.py` | ROS 2 entry point for the interpreter |
| `stdlib/robolang_std_v1.rob` | The v1 standard library; one task, `pick_and_place` |

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
| ROS 2 adapter | Never validated; reports success without checking action results; no ROS 2 tests |

## Running the v1 demo

From a clone of the repository (Python 3.11 or newer, no ROS 2, nothing installed):

```bash
python src/robolang_runtime.py examples/pick_and_place.rob
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

Earlier versions of `setup.py` installed the four v1 modules and a `ros2_runtime` console
command. As of distribution `0.1.0a1` neither is installed, because that code acts without
passing through the validator. This was a deliberate compatibility change.

## The original ROS 2 workflow (unvalidated, no longer works as written)

> Everything below describes the **originally intended** ROS 2 workflow. It has never been
> validated end to end: the repository ships no `package.xml` and no ROS 2 tests, and the steps
> have not been reproduced on a clean ROS 2 install. Step 5 uses the `ros2_runtime` command,
> which is no longer installed. The section is kept as a record of the design.
>
> A future ROS 2 backend is on the development roadmap as item F3-4 of Track E
> ([development plan](DEVELOPMENT_PLAN.md)). It will be built on the validated Task IR, not on
> this adapter, and needs separate owner authorization.

| Layer | Purpose | Runs on | Files |
|-------|---------|---------|-------|
| Language / runtime | Parses `.rob` task files, runs logic flow | Any OS | `src/robolang_runtime.py`, `examples/` |
| Simulated adapter | Prints actions for quick testing | Any OS | `src/robolang_adapter.py` |
| ROS 2 adapter | Sends ROS 2 actions and topics | ROS 2 host (Linux) | `src/robolang_adapter_ros2.py` |
| ROS 2 entry point | Runs the interpreter inside a ROS 2 node | ROS 2 host (Linux) | `src/robolang_ros2_main.py` |

The intended setup, on Ubuntu 22.04 with ROS 2 Humble or Iron:

1. Clone into a workspace:
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/rokorobot/RoboLang.git robolang
   ```
2. Add a minimal `package.xml`:
   ```xml
   <?xml version="1.0"?>
   <package format="3">
     <name>robolang</name>
     <version>1.0.0</version>
     <description>RoboLang — AI-readable DSL for robots using ROS2</description>
     <maintainer email="rokorobot@users.noreply.github.com">Robert Konecny</maintainer>
     <license>Apache-2.0</license>
     <exec_depend>rclpy</exec_depend>
     <exec_depend>control_msgs</exec_depend>
     <exec_depend>trajectory_msgs</exec_depend>
     <exec_depend>std_msgs</exec_depend>
   </package>
   ```
3. Build the workspace:
   ```bash
   cd ~/ros2_ws
   colcon build
   source install/setup.bash
   ```
4. Run a task (no longer works: `ros2_runtime` is not installed):
   ```bash
   ros2 run robolang ros2_runtime examples/pick_and_place.rob
   ```

If action servers such as `/arm_controller/follow_joint_trajectory` and
`/gripper_controller/gripper_action` were active, the adapter would send goals to them, with
the same fixed joint values and no joint names for every `move`.

Intended mapping:

| RoboLang command | ROS 2 mapping | Message type |
|---|---|---|
| `move r to X` | `FollowJointTrajectory` | `control_msgs/action/FollowJointTrajectory` |
| `grasp r object` | `GripperCommand` | `control_msgs/action/GripperCommand` |
| `place r object at dst` | `GripperCommand` (open) | `control_msgs/action/GripperCommand` |
| `inspect r object` | Custom service (e.g. `/sensor_status`) | `my_msgs/srv/SensorStatus` |
| `communicate r to "fleet" with "TASK_COMPLETE"` | Topic publish | `std_msgs/msg/String` |

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
