# RoboLang v1 — Documentation Overview

Welcome to the RoboLang documentation.  
This folder contains the core reference material for **RoboLang v1**, an AI-readable
domain-specific language (DSL) for programming and coordinating robots.

---

## 📚 Documents in this folder

- **RoboLang_v1_Ebook.pdf**  
  High-level language manual for engineers.  
  Introduces the syntax (`task`, `pre`, `plan`, `@safety`), core actions
  (`move`, `grasp`, `place`, `inspect`, `wait`, `communicate`) and lots of examples.

- **RoboLang_v1_Library_ROS2_Adapter.pdf**  
  Describes the standard library of RoboLang tasks and how they map to ROS2
  actions, services, and topics.  
  Useful if you are integrating RoboLang into an existing ROS2 system.

- **RoboLang_v1_Developer_Guide.pdf**  
  Shows how to build the RoboLang runtime interpreter and connect it to the
  Python adapter layer.  
  Covers parsing, execution, preconditions, and deployment.

---
## 📚 Downloadable Guides

- [RoboLang v1 Language Manual (PDF)](RoboLang_v1_Ebook.pdf)
- [Standard Library + ROS2 Adapter (PDF)](RoboLang_v1_Library_ROS2_Adapter.pdf)
- [Developer Guide (PDF)](RoboLang_v1_Developer_Guide.pdf)

## 🧠 What is RoboLang?

RoboLang is a domain-specific language designed for:

- **Human readability** – simple, declarative keywords and tasks  
- **AI generation** – easy to produce and reason about with LLMs  
- **Robotic safety** – explicit `@safety` annotations and `pre { ... }` checks  
- **ROS2 integration** – each primitive maps to ROS2 actions and services

A typical RoboLang task looks like:

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


🧪 Quick Start

For a quick, simulated demo (no ROS2 required), see the main README.md:

```bash
git clone https://github.com/rokorobot/RoboLang.git
cd RoboLang
python src/robolang_runtime.py examples/pick_and_place.rob
```
🛰️ Where to go next

Read the Language Manual to learn the syntax and patterns

Use the Library + ROS2 Adapter Guide when wiring RoboLang into real robots

Follow the Developer Guide if you want to extend the runtime or build your
own planner/integration

RoboLang aims to be the bridge between natural language, intelligent planning,
and safe robot execution.




