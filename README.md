<p align="center">
  <img src="https://img.shields.io/badge/version-v0.1.0a1-blue?style=for-the-badge" alt="version"/>
  <img src="https://img.shields.io/badge/python-3.11%2B-brightgreen?style=for-the-badge" alt="python"/>
  <img src="https://img.shields.io/github/license/rokorobot/RoboLang?style=for-the-badge" alt="license"/>
  <img src="https://img.shields.io/github/stars/rokorobot/RoboLang?style=for-the-badge" alt="stars"/>
  <img src="https://img.shields.io/github/forks/rokorobot/RoboLang?style=for-the-badge" alt="forks"/>
</p>

<p align="center">
  <b>RoboLang</b> — An AI-native language and interoperability foundation for robots.<br/>
  Built by <a href="https://github.com/rokorobot">@rokorobot</a>
</p>
<p align="center">
  <img src="docs/robolang_banner.png" width="600" alt="RoboLang Logo"/>
</p>

---

# RoboLang

RoboLang is an open, robot-independent project for expressing, validating and coordinating
robotic tasks, goals and verifiable outcomes across AI systems and heterogeneous robots.

- **Available today:** RoboLang 1.1 and 1.2 language tooling, deterministic validation, Task IR 1.0
  and simulation-only task execution with reproducible run records.
- **Specified and approved, not implemented:** versioned Core semantic contracts, AI-to-Robot
  (A2R) and Robot-to-Robot (R2R) interoperability, observations, capabilities, authorization
  and verified outcomes.

> ⚠️ **Physical robot execution is not implemented or authorized.** RoboLang does not replace
> ROS 2, MoveIt 2, Nav2 or hardware safety controllers. Do not run RoboLang against real
> hardware.

Created and maintained by **Robert Konecny (@rokorobot)**.

## Contents

- [Overview](#overview)
- [Implemented today](#implemented-today)
- [Quickstart](#quickstart)
- [Architecture](#architecture)
- [Current limitations](#current-limitations)
- [Roadmap](#roadmap)
- [Documentation](#documentation)
- [Contributing](#contributing)
- [License](#license)

---

## Overview

RoboLang is written primarily for AI systems to generate, read, exchange, validate and act on;
humans design, review and authorize it. Its central abstraction, from the approved
[AI-First Language Design Charter](spec/architecture/ai-first-language-charter.md):

> RoboLang's fundamental abstraction is a **verifiable transition from an observed world state
> toward a desired outcome**. Actions, plans and communication are means to achieve that
> outcome; execution alone does not establish success.

That shapes the design: goals, typed semantics, uncertainty, authorization and verified
outcomes come first, and a sequence of instructions is one possible means, not the centre.
RoboLang components stay deterministic and do not call a model at runtime; AI output is
treated as untrusted input that deterministic software checks before use.

| Maturity | Status |
|---|---|
| Language | RoboLang **1.1**, and **1.2** (adds `and`, `or`, `not` in conditions) |
| Python package | **0.1.0a1** (alpha; installed from a clone, not published on PyPI) |
| Foundation 1 (simulation-only execution) | **Delivered**, exit gate signed off 2026-10-27 ([evidence](docs/evidence/rl-basic-01-exit-gate.md)) |
| Next milestone | **RL-ARCH-02**: specification approved 2026-10-27 ([Core 0.1](spec/core-v0.1.md), [reference simulation](spec/reference-simulation-v0.1.md)); implementation needs separate owner authorization |

## Implemented today

| Component | Where | What it does |
|---|---|---|
| Lexer and parser | `src/robolang/lexer.py`, `parser.py` | Formal grammar ([1.1](spec/grammar.ebnf), [1.2](spec/grammar-v1.2.ebnf)), typed AST with source positions |
| Validator | `src/robolang/validator.py` | Names, types, values and units; stable diagnostic codes with `file:line:column` |
| Task IR 1.0 and 1.1 | `src/robolang/ir.py` | Versioned, deterministic JSON form of a validated task ([1.0](spec/ir-schema-v1.0.md), [1.1](spec/ir-schema-v1.1.md)) |
| Symbolic executor | `src/robolang/sim/` | Runs one validated task against a declared world file, fail-closed: false or unknown preconditions stop the run |
| Run records | `src/robolang/sim/record.py` | Versioned JSON record of every state, precondition, step and the final world, with a run ID derived from the inputs |
| Reference simulation | `src/robolang/arch02/` | Simulation-only run of the RFC-001 reference scenario: four participants exchanging Core 0.1 messages, symbolic world, scripted faults, all 13 failure variants, per-recipient message sequencing; a deterministic run record with a run ID ([spec](spec/reference-simulation-v0.1.md)) |
| Core 0.1 checks | `src/robolang/core/` | Per-message checks of RoboLang Core 0.1 documents: strict JSON, schemas, RFC 8785 canonical form, simulation-only signatures, stable rejection codes ([spec](spec/core-v0.1.md)). No transport |
| `robolang` command | `src/robolang/cli.py` | `validate`, `inspect`, `simulate`, `arch02` |

The behaviour is specified in [`robolang-v1.1.md`](spec/robolang-v1.1.md),
[`robolang-v1.2.md`](spec/robolang-v1.2.md), [`ir-schema-v1.1.md`](spec/ir-schema-v1.1.md),
[`ir-schema-v1.0.md`](spec/ir-schema-v1.0.md) and [`simulation-v1.0.md`](spec/simulation-v1.0.md).
CI runs the full test suite, lint and an install test of the wheel and source distribution on
Ubuntu and Windows.

## Quickstart

You need **Python 3.11 or newer** and Git. No ROS 2, no robot and no other packages.

```bash
git clone https://github.com/rokorobot/RoboLang.git
```

```bash
cd RoboLang
```

```bash
python -m pip install .
```

```bash
robolang validate examples/pick_and_place.rob
```

```bash
robolang inspect examples/pick_and_place.rob --json
```

```bash
robolang simulate examples/pick_and_place.rob --world tests/fixtures/simulation/worlds/base.json
```

```bash
robolang arch02 --scenario tests/fixtures/arch02/scenario.json --cases tests/fixtures/arch02/cases.json --case happy_path
```

What you should see from `validate`:

```text
examples/pick_and_place.rob:5:23: warning[W401]: 'max_speed' has no unit; write 'm/s' or 'rad/s'. A unitless limit is deprecated and is never treated as an enforced limit
examples/pick_and_place.rob: valid RoboLang 1.1 (1 task, 1 warning)
Validation covers the text of the program only. It does not make a task safe to run on a robot.
```

`simulate` prints a summary line such as `simulation only: run sim-…: SUCCEEDED`, then the run
record as JSON.

- `validate` checks the whole file and prints diagnostics. Exit status is 0 if valid, 1 if not,
  2 for a bad argument or unreadable file.
- `inspect` validates first, then shows what the file describes, as text or as Task IR JSON
  (`--json`). `--task NAME` shows one task.
- `simulate` runs a validated task in a symbolic, deterministic simulation (no robot, no
  adapter, no ROS 2). The same source, world, task and `--seed` give the same run record. Exit
  status: 0 succeeded, 1 invalid source, 2 usage error, 3 invalid world file, 4 failed,
  5 cancelled.
- `arch02` runs one case of the RFC-001 reference scenario: an AI planner stub asks a fixed
  arm to deliver a container, the arm hands custody to a mobile robot, and every message is a
  checked Core 0.1 document. **Simulation only**: one process, a symbolic world, no network, no
  ROS 2, no hardware, and simulation-only signatures. It prints a summary line such as
  `simulation only: run arch02-…: planner-1 recorded succeeded`, then the run record as JSON;
  the same scenario, case and seed give a byte-identical record. Other cases are in the same
  file (for example `v06_partition_before_release`) and in `sequencing_cases.json`. Exit status:
  0 the case ran, 2 usage error, 3 invalid scenario or cases file.

A simulation result is not evidence that a task is safe on hardware. Installing into a
[virtual environment](https://docs.python.org/3/library/venv.html) is recommended. The same
commands also run from the clone without installing: `python src/robolang validate FILE.rob`.

### Three version numbers

`robolang --version` prints all three. They change independently.

| Version | Current | What it identifies |
|---|---|---|
| Distribution | `0.1.0a1` | The installable Python package (`pyproject.toml`). A pre-release: `a1` means first alpha |
| Language | `1.1`, `1.2` | The syntax and rules of `.rob` files ([1.1](spec/robolang-v1.1.md), [1.2](spec/robolang-v1.2.md)). A file is 1.2 only with the marker `robolang 1.2;` |
| IR schema | `1.0`, `1.1` | The shape of the JSON printed by `inspect --json`: 1.0 for a 1.1 source, 1.1 for a 1.2 source ([1.0](spec/ir-schema-v1.0.md), [1.1](spec/ir-schema-v1.1.md)) |

The simulation world file and run record have their own schema versions, defined in
[`simulation-v1.0.md`](spec/simulation-v1.0.md): the world file is `1.0`; the run record is `1.0` for a
1.1 source and `1.1` for a 1.2 source ([`robolang-v1.2.md`](spec/robolang-v1.2.md) section 8.3).

### Example task

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

## Architecture

**Implemented** — the RoboLang 1.1 language front end (the Language Kernel) and the
simulation-only executor:

```text
.rob source
    ↓  lexer + parser              typed AST with source positions
    ↓  validator                   names, types, units → diagnostics, or
Task IR 1.0                        versioned JSON (robolang inspect --json)
    ↓  + world file 1.0
symbolic executor                  CREATED → VALIDATED → AUTHORIZED → RUNNING → SUCCEEDED / FAILED / CANCELLED
    ↓
run record 1.0                     deterministic, run ID derived from the inputs
```

**Approved target, not implemented** — defined in
[RFC-001](spec/architecture/rfc-001-ai-native-architecture.md) and the
[charter](spec/architecture/ai-first-language-charter.md):

- **Semantic Core:** the authoritative meaning of goals, plans, observations, capabilities,
  commitments and outcomes, as versioned JSON documents. An AI may produce Core JSON directly or
  write `.rob`, which compiles to the same meaning.
- **A2R and R2R profiles:** how AI agents and robots, and robots among themselves, exchange
  goals, commitments and evidence through a common message envelope.
- **Local authorization:** a robot runs a proposal only within the authority its own local
  policy grants; standing authorizations carry an explicit `simulation` or `physical` mode.
- **Verified outcomes:** success is established by an outcome checked against evidence, and
  `unknown` is a valid outcome.
- **Interchangeable execution backends**, ROS 2 among them, below RoboLang rather than replaced
  by it.

The work is organised by thirteen domains, from motion and manipulation to safety,
authorization and outcomes, described in
[RL-FOUNDATION-01](spec/architecture/rl-foundation-01-language-foundation.md).

## Current limitations

- **Symbolic world only.** The simulator tracks facts (locations, holding, readiness, region
  and sensor status) declared in a world file. No motion planning, kinematics, collisions or
  physics.
- **Speed limits are not enforced.** `@safety max_speed` is checked against the robot profile
  and reported in the run record, never enforced. A unitless limit is deprecated and reported
  as not enforced.
- **No hardware and no ROS 2 runtime.** No adapter is loaded and nothing is sent to a robot.
- **No Core schemas yet.** Core JSON, A2R and R2R are specified in RFC-001; their schemas are
  drafted in RL-ARCH-02.
- **Plans are sequential.** RoboLang 1.2 adds combined conditions (`and`, `or`, `not`, with
  three-valued logic) but no branching, loops, structured data or local bindings (planned as
  Track B language extensions).
- **Placeholders:** `requirements.txt` (no runtime dependencies), `docker/*`,
  `examples/inspect_object.rob` and `examples/handover.rob` are empty.
- **Legacy v1 code** (`src/robolang_runtime.py`, the adapters and the ROS 2 entry point) stays in
  the repository as history. It is not installed, bypasses the validator and has never been
  validated against ROS 2. See [Legacy RoboLang v1](docs/legacy-v1.md).

## Roadmap

Milestones are sequential and evidence-gated. Listing an item authorizes nothing: every
implementation item needs owner authorization before it starts. Full detail in the
[development plan](docs/DEVELOPMENT_PLAN.md).

| # | Milestone | State |
|---|---|---|
| 1 | v1.1 — language correctness | Delivered |
| 2 | RL-BASIC-01 — simulation-only execution of RoboLang 1.1 tasks (Foundation 1) | Delivered, signed off 2026-10-27 |
| 3 | RL-ARCH-02 — in-process simulation of the RFC-001 reference scenario: one AI agent stub, two robot stubs, failure variants | Implemented ([Core 0.1](spec/core-v0.1.md), [reference simulation](spec/reference-simulation-v0.1.md)): ARCH-1, ARCH-2, ARCH-SEQ, ARCH-3. Gate 1 exit needs owner sign-off |

| Track | Content | State |
|---|---|---|
| A — Foundation 1 | Simulation executor, run records, `robolang simulate` | Delivered |
| B — Language extensions | Combined conditions, branching, bounded repetition, typed values, local bindings, structured data | LANG-1 (combined conditions, RoboLang 1.2) delivered; the rest not started |
| C — Core | IR reader and re-validator; RL-ARCH-02 | RL-ARCH-02 implemented (simulation only); gate 1 exit needs owner sign-off. IR reader not started |
| D — Foundation 2 | Goal execution, Core-shaped observations, knowledge store, reasoner proposals, bounded recovery, world-model depth | Not started |
| E — Foundation 3 | Concurrency, resources, portable skills, ROS 2 backend, two-profile portability, A2A binding | Later |

## Documentation

**Normative specifications**

- [RoboLang 1.1 language](spec/robolang-v1.1.md) and [grammar](spec/grammar.ebnf)
- [RoboLang 1.2 language](spec/robolang-v1.2.md) (combined conditions) and [grammar](spec/grammar-v1.2.ebnf)
- [Task IR schema 1.0](spec/ir-schema-v1.0.md) and [1.1](spec/ir-schema-v1.1.md)
- [Simulation semantics, world file 1.0 and run record 1.0](spec/simulation-v1.0.md)

**Architecture**

- [RFC-001 — AI-native semantic and communication architecture](spec/architecture/rfc-001-ai-native-architecture.md)
- [AI-First Language Design Charter](spec/architecture/ai-first-language-charter.md)
- [RL-FOUNDATION-01 — language foundation](spec/architecture/rl-foundation-01-language-foundation.md)
- [RL-FOUNDATION-02 — domain contracts](spec/architecture/rl-foundation-02-domain-contracts.md)

**Decisions and plans**

- Decision records: [0001](spec/decisions/0001-rl-11a-language-decisions.md),
  [0002](spec/decisions/0002-ai-native-architecture.md),
  [0003](spec/decisions/0003-language-foundation.md),
  [0004](spec/decisions/0004-rl-basic-01a-simulation-semantics.md),
  [0005](spec/decisions/0005-ai-first-charter-and-domain-contracts.md)
- [Development plan](docs/DEVELOPMENT_PLAN.md)
- [Foundation 1 exit-gate evidence](docs/evidence/rl-basic-01-exit-gate.md)

**Legacy**

- [Legacy RoboLang v1](docs/legacy-v1.md): the original line-based interpreter, simulated demo,
  experimental ROS 2 adapter and the v1 PDF guides

## Contributing

Pull requests are welcome. Keep each change small, with CI passing. New language syntax, IR
kinds and Core schemas need a versioned specification and an owner decision first.

```bash
python -m pip install -r requirements-dev.txt
```

```bash
python -m pytest -q
```

```bash
python -m ruff check .
```

```bash
python tools/smoke_install.py
```

Tests marked `xfail` describe planned behaviour that is not implemented yet. The last command
builds the wheel and source distribution, installs each into a fresh virtual environment and
runs the installed `robolang` command; it needs network access.

Report problems in [Issues](https://github.com/rokorobot/RoboLang/issues). Documentation site:
[rokorobot.github.io/RoboLang](https://rokorobot.github.io/RoboLang/).

## License

Licensed under the **Apache License 2.0**. You are free to use, modify and distribute RoboLang
in open or commercial robotics projects.
