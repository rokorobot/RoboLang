# 🤖 RoboLang v1 — The AI-Readable Programming Language for Robots

[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](LICENSE)
[![ROS2](https://img.shields.io/badge/ROS2-Humble-green.svg)](https://docs.ros.org/en/humble/)
[![Python](https://img.shields.io/badge/Python-3.11%2B-yellow.svg)](https://www.python.org/)
[![Status](https://img.shields.io/badge/version-v1.0.0-success.svg)]()

---

> **RoboLang** is a new domain-specific language (DSL) for robotic programming — readable by humans, executable by ROS2, and understandable by AI models.  
> It bridges natural language, LLM planning, and physical robot execution.

Created and maintained by **Robert Konecny (@rokorobot)**.

---

## 🧭 What Is RoboLang?

RoboLang defines a **universal task language** for robots.  
It allows engineers and AI systems to describe *what to do*, not *how to move each joint*.

### ✳️ Core Concepts
- **Declarative Syntax** — Simple, readable `task { plan { ... } }` structure.  
- **Safety Annotations** — Explicit `@safety` and `pre { ... }` sections for guard conditions.  
- **AI-Ready** — Designed for generation and reasoning by large language models (LLMs).  
- **ROS2 Integration** — Maps directly to MoveIt, GripperCommand, and custom services.  
- **Multi-Robot Collaboration** — Built-in `communicate` primitives for task handoffs.

---

## 🧩 Repository Contents

```
robolang/
├── src/
│   ├── robolang_adapter.py
│   ├── robolang_runtime.py
│   └── __init__.py
├── stdlib/
│   └── robolang_std_v1.rob
├── examples/
│   ├── pick_and_place.rob
│   ├── inspect_object.rob
│   └── handover.rob
├── docs/
│   ├── RoboLang_v1_Ebook.pdf
│   ├── RoboLang_v1_Library_ROS2_Adapter.pdf
│   ├── RoboLang_v1_Developer_Guide.pdf
│   └── overview.md
├── docker/
│   ├── Dockerfile
│   └── docker-compose.yml
├── README.md
├── LICENSE
├── requirements.txt
└── setup.py
```

---

## ⚙️ Getting Started

### 1. Prerequisites

- **Python 3.11+**
- **ROS2 Humble or newer**
- Optional: MoveIt2, Gazebo, FastAPI

### 2. Install RoboLang

```bash
git clone https://github.com/rokorobot/robolang.git
cd robolang
pip install -r requirements.txt
```

### 3. Run an Example

```bash
ros2 run robolang runtime examples/pick_and_place.rob
```

or directly with Python:

```bash
python3 src/robolang_runtime.py examples/pick_and_place.rob
```

---

## 🧠 Core Components

| Component | Description |
|------------|--------------|
| `RoboLang DSL` | The language syntax defining `task`, `pre`, `plan`, and `@safety`. |
| `robolang_std_v1.rob` | The standard library with reusable actions like `pick_and_place`, `inspect_object`, etc. |
| `robolang_adapter.py` | Maps RoboLang actions to ROS2 entities (MoveIt, GripperCommand, etc). |
| `robolang_runtime.py` | Minimal interpreter that parses `.rob` files and executes them via the adapter. |

---

## 📘 Documentation

| Guide | Description |
|-------|--------------|
| [RoboLang v1 Language Manual (PDF)](docs/RoboLang_v1_Ebook.pdf) | Learn the syntax and semantics. |
| [Standard Library + ROS2 Adapter (PDF)](docs/RoboLang_v1_Library_ROS2_Adapter.pdf) | Ready-to-use task definitions and mapping guide. |
| [Developer Guide (PDF)](docs/RoboLang_v1_Developer_Guide.pdf) | Build and deploy the runtime interpreter. |

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
| v1.0.0 | Language, Library, ROS2 Adapter |
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

- 📘 [Official Docs](https://rokorobot.github.io/robolang)
- 💬 [Discussion Board](https://github.com/rokorobot/robolang/discussions)
- 🐳 [Docker Image (coming soon)](https://hub.docker.com/r/rokorobot/robolang)
- 🤖 [ROS Index Listing](https://index.ros.org/p/robolang/)
- 🧠 [Hugging Face Space for LLM Integration (planned)](https://huggingface.co/spaces/rokorobot/robolang)

---

### 🦾 *RoboLang is to robots what Python was to software.*

Bridging human intent and machine execution — safely, clearly, and universally.
