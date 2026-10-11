<p align="center">
  <img src="https://img.shields.io/badge/version-v1.0.0-blue?style=for-the-badge" alt="version"/>
  <img src="https://img.shields.io/badge/python-3.11%2B-brightgreen?style=for-the-badge" alt="python"/>
  <img src="https://img.shields.io/github/license/rokorobot/RoboLang?style=for-the-badge" alt="license"/>
  <img src="https://img.shields.io/github/stars/rokorobot/RoboLang?style=for-the-badge" alt="stars"/>
  <img src="https://img.shields.io/github/forks/rokorobot/RoboLang?style=for-the-badge" alt="forks"/>
</p>

<p align="center">
  <b>RoboLang</b> — An AI-readable domain-specific language for programming and coordinating robots.<br/>
  Built by <a href="https://github.com/rokorobot">@rokorobot</a>
</p>
<p align="center">
  <img src="docs/robolang_banner.png" width="600" alt="RoboLang Logo"/>
</p>

---

# 🤖 RoboLang v1 — The AI-Readable Programming Language for Robots

---

> **RoboLang** is a new domain-specific language (DSL) for robotic programming — designed to be readable by humans, executable through ROS2, and understandable by AI models.
> Its goal is to bridge natural language, LLM planning, and robot execution.

Created and maintained by **Robert Konecny (@rokorobot)**.

---

## 🚦 Project Status

RoboLang v1 is an **early prototype**. This section is the reference for what works today; other sections also describe design goals.

| Area | Status |
|------|--------|
| Simulated pick-and-place demo (`src/robolang_runtime.py` + `src/robolang_adapter.py`) | ✅ Implemented, covered by automated tests |
| Line-based interpreter for `move`, `grasp`, `place`, `inspect`, `wait for`, `communicate` inside `plan { ... }` | ✅ Implemented (prefix matching only, no grammar) |
| `pre { ... }` preconditions | ⚠️ Accepted in source but **not evaluated** |
| `@safety` annotations | ⚠️ Accepted in source but **not enforced** |
| `move` destinations | ⚠️ **Ignored** — every `move` sends the same fixed demo joint values |
| Decimal durations (`wait for 2.5s`) | ⚠️ Truncated to whole seconds |
| Unknown or misspelled instructions | ⚠️ Printed as ignored; execution continues |
| ROS2 adapter (`src/robolang_adapter_ros2.py`) | 🧪 Written, **never validated** against ROS2, a simulator or a robot; no ROS2 tests |
| v1.1 specification and grammar ([`spec/`](spec/robolang-v1.1.md)) | ✅ Written |
| v1.1 lexer, parser, AST and static checks (`src/robolang/`) | ✅ Implemented and tested; **not connected to the interpreter**, which still runs v1 behavior |
| `validate` / `inspect` commands and versioned task IR ([schema](spec/ir-schema-v1.0.md)) | ✅ Implemented and tested; read-only, run from a checkout (not pip-installable yet) |
| Execution of validated tasks, installable package | 🗓️ Planned |
| MoveIt2 integration, multi-robot handover, LLM planner | 🗓️ Planned |
| `requirements.txt`, `docker/*`, `examples/inspect_object.rob`, `examples/handover.rob` | 🗓️ Empty placeholders |

The limitations above are pinned down by `tests/test_known_limitations.py`.

> ⚠️ **Physical robot execution is not validated and is outside the approved scope of this project.**
> Do not run RoboLang against real hardware. The interpreter does not check preconditions, enforce safety limits or resolve motion targets, and the ROS2 adapter reports success without checking action results.

---

## 🧭 What Is RoboLang?

RoboLang defines a **universal task language** for robots.  
It allows engineers and AI systems to describe *what to do*, not *how to move each joint*.

### ✳️ Core Concepts
- **Declarative Syntax** — Simple, readable `task { plan { ... } }` structure.  
- **Safety Annotations** — Explicit `@safety` and `pre { ... }` sections for guard conditions *(syntax only; not enforced yet)*.
- **AI-Ready** — Designed for generation and reasoning by large language models (LLMs) *(no LLM integration yet)*.
- **ROS2 Integration** — An experimental adapter targets `FollowJointTrajectory`, `GripperCommand` and custom services *(unvalidated; MoveIt is planned)*.
- **Multi-Robot Collaboration** — A `communicate` primitive that sends a plain string *(handoff protocol planned)*.

---

## 📂 Project Layout

```text
robolang/
├── src/
│   ├── robolang_adapter.py              # Simulated adapter (prints commands)
│   ├── robolang_adapter_ros2.py         # ROS2 adapter (connects to real robot actions/services)
│   ├── robolang_runtime.py              # Core RoboLang interpreter and executor
│   ├── robolang_ros2_main.py            # ROS2 runtime entrypoint (used by ros2 run)
│   └── __init__.py
│
├── stdlib/
│   └── robolang_std_v1.rob              # Standard library of RoboLang primitives and macros
│
├── examples/
│   ├── pick_and_place.rob               # Basic pick & place example
│   ├── inspect_object.rob               # Inspection task example (empty placeholder)
│   └── handover.rob                     # Multi-robot handover example (empty placeholder)
│
├── tests/                               # pytest suite: smoke tests + known-limitation tests
│
├── docs/
│   ├── RoboLang_v1_Ebook.pdf            # Complete language manual and exercises
│   ├── RoboLang_v1_Library_ROS2_Adapter.pdf # Library + ROS2 mapping documentation
│   ├── RoboLang_v1_Developer_Guide.pdf  # Developer setup and internal architecture
│   ├── RoboLang_v1_ROS2_Integration.pdf # Full ROS2 setup & hardware integration guide
│   ├── robolang_banner.png              # Project banner image for README
│   └── index.md                         # Documentation homepage for GitHub Pages
│
├── docker/
│   ├── Dockerfile                       # Empty placeholder
│   └── docker-compose.yml               # Empty placeholder
│
├── README.md                            # Project overview and documentation
├── LICENSE                              # Apache 2.0 license
├── requirements.txt                     # Runtime dependencies (none; the demo uses only the standard library)
├── requirements-dev.txt                 # Test dependencies (pytest)
├── pytest.ini                           # Test configuration
└── setup.py                             # Package configuration + ROS2 entrypoint
```

## ⚙️ Getting Started

### 1. Prerequisites

- **Python 3.11+** — the simulated demo needs nothing else
- ROS2 is only needed for the experimental ROS2 adapter (see below)

### 2. Get RoboLang

```bash
git clone https://github.com/rokorobot/RoboLang.git
cd RoboLang
```

### 3. Run the Example

```bash
python src/robolang_runtime.py examples/pick_and_place.rob
```

### 4. Validate and Inspect a Task File

RoboLang v1.1 has two read-only commands. They need only Python: no ROS2, no robot, nothing to install.
Run them from the repository root:

```bash
python src/robolang validate examples/pick_and_place.rob
```

```bash
python src/robolang inspect examples/pick_and_place.rob
```

```bash
python src/robolang inspect examples/pick_and_place.rob --json
```

- `validate` checks the whole file (syntax, names, types, values) and prints `file:line:column` diagnostics. Exit status is 0 if valid, 1 if not, 2 for a bad argument or unreadable file.
- `inspect` validates first, then shows what the file describes, as text or as versioned JSON IR. `--task NAME` shows one task.
- Neither command executes a task. Validation covers the text of the program only; it does not make a task safe to run on a robot.

These commands are not installed by `pip install` yet. The simulated demo in step 3 still runs the older v1 interpreter and does not use this validation.

### 5. Run the Tests

```bash
python -m pip install -r requirements-dev.txt
python -m pytest -q
```

Tests marked `xfail` describe planned behavior that is not implemented yet.

---

## 🧠 Core Components

| Component | Description |
|------------|--------------|
| `RoboLang DSL` | The language syntax defining `task`, `pre`, `plan`, and `@safety`. |
| `robolang_std_v1.rob` | The standard library; currently one task, `pick_and_place`. |
| `robolang_adapter.py` | Simulated adapter that prints each action; no ROS2 required. |
| `robolang_adapter_ros2.py` | Experimental, unvalidated adapter mapping actions to ROS2 (`FollowJointTrajectory`, `GripperCommand`, topics). |
| `robolang_runtime.py` | Minimal line-based interpreter that scans the `plan { ... }` block of a `.rob` file and calls the adapter. |

---

## 📘 Documentation

| Guide | Description |
|-------|--------------|
| [RoboLang v1 Language Manual (PDF)](docs/RoboLang_v1_Ebook.pdf) | Learn the syntax and semantics of RoboLang. |
| [Standard Library + ROS2 Adapter (PDF)](docs/RoboLang_v1_Library_ROS2_Adapter.pdf) | Ready-to-use task definitions and ROS2 mapping guide. |
| [Developer Guide (PDF)](docs/RoboLang_v1_Developer_Guide.pdf) | Build and deploy the RoboLang runtime interpreter. |
| [ROS2 Integration & Hardware Setup (PDF)](docs/RoboLang_v1_ROS2_Integration.pdf) | Step-by-step guide for deploying RoboLang on real robots and ROS2 environments. |


---

## 🧠 Example RoboLang Task

```rob
task pick_and_place(robot r, object box, location src, location dst, region cell) {
    @safety max_speed 0.5;
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

---

## 🛰️ Integration Architecture

Target architecture. Today only the interpreter and the simulated adapter are exercised; there is no LLM planner or MoveIt integration.

```
[ Human / AI Prompt ]
          ↓
[ LLM Planner ] → RoboLang Task (.rob)
          ↓
[ Runtime Interpreter (Python) ]
          ↓
[ ROS2 Adapter → MoveIt, GripperCommand, Custom Services ]
          ↓
[ Real Robot or Simulation ]
```

---

## 🧩 Roadmap

| Version | Focus |
|----------|-------|
| v1.0.0 | Prototype interpreter, simulated demo, experimental ROS2 adapter |
| v1.1.0 | Full Grammar Parser, Unit Tests |
| v1.2.0 | LLM-Driven Task Planner Integration |
| v2.0.0 | Multi-Robot Runtime, Safety Simulation |

---

## 🧑‍💻 Contributing

Pull requests are welcome!  
Please follow PEP8 for Python code, and use the `.rob` examples in `/stdlib` as reference style.

### To contribute:

```bash
git checkout -b feature/your-feature
git commit -am "Add new library task"
git push origin feature/your-feature
```

Then open a pull request on GitHub.

---

## 📄 License

Licensed under the **Apache License 2.0**.  
You’re free to use, modify, and distribute RoboLang in open or commercial robotics projects.

---

## 🌍 Links & Resources

- 📘 [Official Docs](https://rokorobot.github.io/RoboLang/)
- 🐞 [Issues](https://github.com/rokorobot/RoboLang/issues)
- Planned, not available yet: Docker image, ROS Index listing, Hugging Face Space for LLM integration

---

### 🦾 *RoboLang is to robots what Python was to software.*

Bridging human intent and machine execution — safely, clearly, and universally.

---

## 🧪 Run the Demo

RoboLang comes with a ready-to-run **simulated pick-and-place example** — no ROS2 required.

### 1️⃣ Prerequisites
- **Python 3.11+** (tested successfully on Python 3.13)
- **Git**

### 2️⃣ Clone the repository
```bash
git clone https://github.com/rokorobot/RoboLang.git
cd RoboLang
```

### 3️⃣ Run the demo

Use Python to execute the RoboLang runtime with the included .rob example:
```bash
python src/robolang_runtime.py examples/pick_and_place.rob
```
### 4️⃣ Expected output
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

If you see this output — congratulations 🎉
Your RoboLang v1 runtime and simulated adapter are working!

🧩 What this demo does

Parses a .rob task file (examples/pick_and_place.rob)

Interprets the plan { ... } block

Calls RobotAdapter primitives:

move_to_pose()

set_gripper()

communicate()

Simulates robot motion and communication in the terminal

---

## 🦾 ROS2 Integration (Experimental, Unvalidated)

> ⚠️ Everything in this section describes the **intended** ROS2 workflow. It has not been validated end to end:
> the repository ships no `package.xml` or ROS2 tests, and the steps below have not been reproduced on a clean ROS2 install.
> Physical robot execution is outside the approved scope — see [Project Status](#-project-status).

The ROS2 adapter is meant to let tasks written in `.rob` format drive ROS2 action servers, starting with simulated or fake ones.

---

### 🧭 Architecture Overview

| Layer | Purpose | Runs On | Example Files |
|-------|----------|----------|----------------|
| **Language / Runtime** | Parses `.rob` task files, runs logic flow | Any OS (Windows, macOS, Linux) | `src/robolang_runtime.py`, `examples/` |
| **Simulated Adapter** | Prints fake actions for quick testing | Any OS | `src/robolang_adapter.py` |
| **ROS2 Adapter** | Sends real ROS2 actions & topics | **Robot / ROS2 Host (Linux)** | `src/robolang_adapter_ros2.py` |
| **ROS2 Entrypoint** | Runs RoboLang runtime inside a ROS2 node | **Robot / ROS2 Host (Linux)** | `src/robolang_ros2_main.py` |

---

### ⚙️ Setup on the Robot’s ROS2 Machine

These steps must be done on the **robot controller or any Ubuntu 22.04 machine with ROS2 Humble or Iron installed**.

1. **Open a terminal on the ROS2 system:**
   ```bash
   cd ~/ros2_ws/src
   ```
2. Clone the RoboLang repository:
   ```bash
   git clone https://github.com/rokorobot/RoboLang.git robolang
   ```
3. Add a minimal package.xml if not present:
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
4. Build the workspace:
   ```bash
   cd ~/ros2_ws
   colcon build
   source install/setup.bash
   ```
5. Run a RoboLang task inside ROS2:
   ```bash
   ros2 run robolang ros2_runtime examples/pick_and_place.rob
   ```

If ROS2 action servers are active (e.g. `/arm_controller/follow_joint_trajectory` and `/gripper_controller/gripper_action`), the adapter sends goals to them. Note that every `move` currently sends the same fixed joint values with no joint names, whatever the destination.

🧩 Intended ROS2 Mapping (v1, unvalidated)

| RoboLang Command                                | ROS2 Mapping                           | Message Type                                |
| ----------------------------------------------- | -------------------------------------- | ------------------------------------------- |
| `move r to X`                                   | `FollowJointTrajectory`                | `control_msgs/action/FollowJointTrajectory` |
| `grasp r object`                                | `GripperCommand`                       | `control_msgs/action/GripperCommand`        |
| `place r object at dst`                         | `GripperCommand (open)`                | `control_msgs/action/GripperCommand`        |
| `inspect r object`                              | Custom service (e.g. `/sensor_status`) | `my_msgs/srv/SensorStatus`                  |
| `communicate r to "fleet" with "TASK_COMPLETE"` | Topic publish                          | `std_msgs/msg/String`                       |

🧪 Testing Safely in Simulation

Before connecting to real hardware, you can test RoboLang inside:

🧠 Gazebo Sim (with your arm model + MoveIt2)

🧩 ROS2 Dummy Action Servers

💻 Headless test that simply confirms goal publishing and topic messages

⚠️ Hardware Safety Notice

RoboLang v1 is a non-safety-certified demonstration and must not be used to move physical hardware.

`@safety` annotations and `pre { ... }` blocks are not enforced by the current interpreter.

A `@safety` annotation is never a substitute for emergency stops, protective stops or a robot risk assessment.

Hardware execution would require a separate safety review and explicit owner approval.

🌐 Architecture Recap

┌─────────────────────┐

│  ------- Developer (PC)  ..---------...          │

│  ------- Edits .rob files   ..----------...      │

│  ------- Commits to GitHub  ..----...      │

└─────────┬───────────┘

                  │
          
                  ▼
          
┌─────────────────────┐

│ ---- ROS2 Host / Robot ------- ....       │

│  ---- Pulls from GitHub --------  ....    │

│  ---- Runs ROS2 adapter ------- ....     │

│  ---- Executes actions  ---------- ....     │

└─────────────────────┘

This section defines exactly what parts of RoboLang must run on robotic hardware versus your development environment.




