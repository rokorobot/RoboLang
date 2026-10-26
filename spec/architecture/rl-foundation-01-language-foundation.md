# RL-FOUNDATION-01 — RoboLang Language Foundation

**Date:** 2026-10-26
**Status:** **Proposed.** Nothing in this document is approved. The decisions it needs are
listed in [decision record 0003](../decisions/0003-language-foundation.md).
**Scope:** Documentation only. No change to the RoboLang 1.1 language, grammar, parser,
validator, IR schema 1.0, CLI or v1 runtime, to [RFC-001](rfc-001-ai-native-architecture.md),
to [decision record 0002](../decisions/0002-ai-native-architecture.md) or to
[`docs/DEVELOPMENT_PLAN.md`](../../docs/DEVELOPMENT_PLAN.md).

## 1. Summary

RoboLang 1.1 already has a working language front end: a formal grammar, a parser with
source positions, a static validator and a versioned task IR, with `validate` and
`inspect` commands. This document proposes how to keep developing that language instead of
replacing it:

1. **Thirteen domains** (section 3) give an architectural taxonomy of what a robot language
   has to cover. They are a map for placing work, not thirteen modules to build.
2. **The Language Kernel** (section 4) is the name for the existing front end. There is no
   second language engine.
3. **Foundations 1 to 3** (section 5) order the work in three stages, from essential
   language functions to intelligent behavior to advanced robotics.
4. **RL-BASIC-01** (section 6) is the first milestone that executes RoboLang 1.1 tasks, in
   simulation only, with defined scope and acceptance tests.
5. **A D8 scheduling amendment** (section 7) proposes that Foundation 1, including
   RL-BASIC-01, comes before RL-ARCH-02.

The order in section 7 takes effect only after the owner explicitly approves the D8
amendment. Until then, D8 as approved on 2026-10-19 stands: RL-ARCH-02 is the next planned
milestone.

## 2. Relationship to RFC-001

RFC-001 defines RoboLang Core: the shared semantics, schemas and A2R/R2R profiles through
which AI agents and robots interoperate. It keeps `.rob` as a supported authoring format and
Task IR 1.0 as one supported plan representation (decision D2, RFC-001 sections 6.3, 6.4
and 16).

This document is about that authoring language and its implementation. It does not change
the Core, the message envelope, the A2R and R2R profiles or any approved decision except
the proposed amendment of D8's scheduling in section 7. The layers fit together like this:

```text
.rob authoring (RoboLang 1.x)            Core JSON documents (RFC-001)
            |                                        |
   Language Kernel: lexer -> parser ->               |
   typed AST -> validator -> Task IR 1.0             |
            |                                        |
            +---------------> execution <------------+
                  simulation (RL-BASIC-01, RL-ARCH-02)
                  later: robot adapters, AI and planning backends
```

Where a domain in section 3 overlaps a Core concept, the Core definition in RFC-001 is
authoritative, and any `.rob` construct for it must map onto the Core rather than define a
competing meaning. Whether goals and outcomes get `.rob` syntax at all is RFC-001 open
question 8 and is not decided here.

## 3. The thirteen domains

The domains are grouped in four families. For each one the table records what RoboLang 1.1
already has, where RFC-001 covers it, and the foundation stage (section 5) in which work on
it is planned to start.

### A. Physical intelligence — interaction with the physical environment

| # | Domain | In RoboLang 1.1 today | RFC-001 | Starts in |
|---|---|---|---|---|
| 01 | Motion and navigation | `move r to dst;`, the `location` type | 7.5 Places | Foundation 1 |
| 02 | Physical manipulation | `grasp r box;`, `place r box at dst;`, the `object` type | 7.3 Capability descriptors, 7.4 Things | Foundation 1 |
| 03 | Sensing and perception | `inspect r box using "sensor";`, the conditions `sensor_ok` and `object_exists` | 7.6 Observations | Foundation 1 (simulated observations); richer perception in Foundation 2 |

### B. Cognitive intelligence — representation, reasoning and learned experience

| # | Domain | In RoboLang 1.1 today | RFC-001 | Starts in |
|---|---|---|---|---|
| 04 | Abstract goals and tasks | `task` with typed parameters, `pre` and `plan` | 7.7 Goals, 7.8 Plans; open question 8 | Foundation 1 (tasks); goals in Foundation 2 |
| 05 | Thinking and reasoning | Nothing | 13.4 AI-originated content; plan decision A4 | Foundation 2, as interfaces only |
| 06 | Memory and knowledge | Nothing | Open question 1 (ontology alignment) | Foundation 2 |
| 07 | Data and world model | Four symbolic parameter types; no structured data | 7.4 Things, 7.5 Places, 7.6 Observations; open question 2 (world model depth) | Foundation 1 (structured data); world model in Foundation 2 |

Domain 05 needs a boundary stated up front. Under decision A4 and RFC-001 section 13.4,
RoboLang's own components stay deterministic and do not call a model. Reasoning happens in
AI agents or planners outside the language; RoboLang defines how their proposals are
expressed, validated and authorized, and treats them as untrusted input. "Thinking and
reasoning" in RoboLang therefore means interfaces to reasoning, not a reasoning engine
inside the compiler or executor.

### C. Coordination and behavior — individual and cooperative behavior

| # | Domain | In RoboLang 1.1 today | RFC-001 | Starts in |
|---|---|---|---|---|
| 08 | Communication | `communicate r to "channel" with "message";` (free text) | 8 Message envelope, 9 A2R, 10 R2R | Foundation 1 (recorded messages in simulation); A2R and R2R in Foundation 3 |
| 09 | Logic and control flow | A sequential `plan`, `pre` conditions and `wait until`; no branching, loops or nested blocks | — | Foundation 2 |
| 10 | Time and coordination | `wait for`, `wait until … timeout`, durations in `ms`, `s`, `min` | 11.1 Leases, 11.4 Time; open question 3 | Foundation 1 (simulated clock); concurrency in Foundation 3 |

### D. Execution and governance — what a robot can do, may do, and has actually achieved

| # | Domain | In RoboLang 1.1 today | RFC-001 | Starts in |
|---|---|---|---|---|
| 11 | Capabilities and resources | Parameter types only; no capability model | 7.3 Capability descriptors, 10.3 Resources | Foundation 1 (minimal simulated profile); richer resources in Foundation 3 |
| 12 | Safety and authorization | `@safety max_speed` (declared, not enforced), `pre` conditions | 13 Trust, authority and safety; plan decision A5 | Foundation 1 (fail-closed checks in simulation) |
| 13 | Outcomes and recovery | Nothing: no executor exists | 7.10 Outcomes, 12 Failure model | Foundation 1 (run outcomes); recovery in Foundation 2 |

What the taxonomy is not:

- It is not a package layout or a list of thirteen modules. One component, such as the
  executor, serves several domains.
- It is not a requirement that every domain has work before RL-BASIC-01. RL-BASIC-01 touches
  only the domains marked Foundation 1, and only as far as section 6 says.
- It is not a claim that RoboLang implements anything listed as Foundation 2 or 3.

## 4. The Language Kernel

The Language Kernel is the shared technical foundation under all thirteen domains. Without
one, RoboLang would become thirteen disconnected interfaces. The kernel already exists: it
is the RoboLang 1.1 front end. This section names it; it adds no code.

| Kernel element | Existing implementation | Specification |
|---|---|---|
| Syntax | `src/robolang/lexer.py`, `tokens.py`, `parser.py` | [`robolang-v1.1.md`](../robolang-v1.1.md) sections 2–4, [`grammar.ebnf`](../grammar.ebnf) |
| Typed syntax tree | `src/robolang/nodes.py` (frozen dataclasses with source spans) | `robolang-v1.1.md` section 7 |
| Types | Four parameter types (`robot`, `object`, `location`, `region`), durations, speeds with units, strings | `robolang-v1.1.md` sections 4.3–4.6 |
| Variables and scopes | Task parameters only; names are local to their task; no globals, no shadowing | `robolang-v1.1.md` section 5, rule 3 |
| Expressions | None. Arguments are names, numbers with units, or strings | — |
| Validator and semantic rules | `src/robolang/validator.py` (static rules, `E301`–`E308`) | `robolang-v1.1.md` section 5 |
| Diagnostics | `src/robolang/diagnostics.py` (stable codes, file, line, column) | `robolang-v1.1.md` section 6 |
| Intermediate representation | `src/robolang/ir.py` (`compile_source`, Task IR 1.0) | [`ir-schema-v1.0.md`](../ir-schema-v1.0.md) |
| Command line | `src/robolang/cli.py` (`validate`, `inspect`) | README quickstart |

Rules for evolving the kernel:

1. **One engine.** New language features extend this front end. No parallel parser,
   validator or IR is started for a new domain.
2. **Versioned, compatible extensions.** Every valid RoboLang 1.1 file stays valid with the
   same meaning, or a change is listed as breaking with a migration note, as RL-11A did
   (`robolang-v1.1.md` section 8).
3. **Specification first.** A new construct gets a written specification, fixtures and an
   owner decision before it is implemented, the way RL-11A preceded RL-11B.
4. **The core invariant holds.** The whole file is lexed, parsed and validated before any
   action can occur (`robolang-v1.1.md` section 1).
5. **Core alignment.** A construct that expresses a Core concept (goals, observations,
   outcomes, capabilities) maps onto the RFC-001 definition.

## 5. Foundations 1 to 3

| Stage | Content | Domains | Approach |
|---|---|---|---|
| **Foundation 1 — Essential language** | Types, variables, tasks, the six original actions, conditions, structured data, outcomes | 01, 02, 03, 04 (tasks), 07 (structured data), 08 (local), 10 (clock), 11 (minimal), 12, 13 (outcomes) | Build and validate first |
| **Foundation 2 — Intelligent behavior** | World model, memory, planning, interfaces to reasoning, richer logic, recovery | 04 (goals), 05, 06, 07 (world model), 09, 13 (recovery) | Develop incrementally |
| **Foundation 3 — Advanced robotics** | Concurrent behavior, portable skills, richer resources, cooperation, A2R and R2R | 08 (A2R, R2R), 10 (concurrency), 11 (resources) | Develop once the basic semantics are proven |

The stages order content. They do not forbid a later-stage document from being written
early; they say what has to be proven before a later stage is built on it.

### 5.1 Foundation 1 in detail

Foundation 1 has two parts.

**Part A: execution semantics for the existing constructs.** RoboLang 1.1 defines what a
file is, not what it does (`robolang-v1.1.md` section 9). Part A defines what the six
actions, the four conditions, `wait for`, `wait until` and the run outcome mean when a task
runs, first in simulation. This is RL-BASIC-01 (section 6). It adds no syntax.

| Foundation 1 item | Status in 1.1 | Delivered by |
|---|---|---|
| Tasks | Syntax and static rules done | RL-BASIC-01: task selection and execution |
| Six original actions: `move`, `grasp`, `place`, `inspect`, `wait`, `communicate` | Syntax and static rules done | RL-BASIC-01: simulation semantics |
| Conditions: `robot_ready`, `region_clear`, `object_exists`, `sensor_ok` | Syntax and static rules done | RL-BASIC-01: evaluation against simulated state |
| Types | Four parameter types, durations, speeds, strings | RL-BASIC-01: binding parameters to simulated entities |
| Outcomes | Not in 1.1 | RL-BASIC-01: run outcome record |

**Part B: language extensions.** Variables beyond task parameters, structured data and
expressions are not in RoboLang 1.1. Each is a separate, versioned language proposal with
its own specification, fixtures and owner decision, following kernel rules 2 and 3. Part B
does not block RL-BASIC-01, and RL-BASIC-01 does not need it. The individual proposals are
not defined in this document; they are scoped after RL-BASIC-01, when execution semantics
exist for the new constructs to build on.

**Foundation 1 exit gate (proposed):** RL-BASIC-01 accepted; the Part B proposals the owner
selects for Foundation 1 specified, implemented and accepted; all RoboLang 1.1 fixtures still
pass; owner approval.

## 6. RL-BASIC-01 — functional simulation executor

### 6.1 Objective

Give the existing RoboLang 1.1 functions observable behavior: run a validated `.rob` task
against a deterministic simulated world and produce a structured, reproducible outcome. It
is the first slice of the plan's v1.2 work packages RL-12A (executor state machine),
RL-12B (preconditions), RL-12C (parameter binding, minimal) and RL-12E (deterministic
simulator), restricted to simulation. RL-12D (ROS 2) is not part of it.

### 6.2 Scope

1. **Input.** A `.rob` file, a selected task, and a simulation world file (JSON). The file is
   compiled in the same process by the existing front end (`compile_source`), so the core
   invariant holds. Executing an externally supplied IR document is out of scope: under the
   IR trust boundary (`ir-schema-v1.0.md` section 2) that would first need an IR reader that
   re-checks everything, which is separate work.
2. **Task selection.** As in `robolang-v1.1.md` section 9: a file with several tasks needs
   `--task NAME`; a single task may be selected implicitly; running all tasks is never a
   default.
3. **World file.** Declares simulated robots, objects, locations, regions and sensors; binds
   each task parameter to a declared entity of the matching type; gives initial state
   (robot readiness and location, object locations, region clearance, sensor availability);
   gives each robot a minimal capability profile (whether it has a gripper, which sensors it
   has, its speed limits); and may script timed events and injected faults. The world file
   is versioned and validated before execution. Unknown fields and unknown versions are
   rejected.
4. **Executor states.** `CREATED → VALIDATED → AUTHORIZED → RUNNING → SUCCEEDED | FAILED |
   CANCELLED`, as in RL-12A. `AUTHORIZED` means authorized for simulation; the executor has
   no other mode and no code path to ROS 2 or hardware.
5. **Semantics.** Section 6.3.
6. **Outcome record.** A JSON run record: run ID, task name, a hash of the source and of the
   world file, final state, per-step status, events with simulated timestamps, failure code
   and reason where applicable, and `"mode": "simulation"`. The record is designed so that it
   can later be mapped onto Core outcomes (RFC-001 section 7.10), but it is not a Core
   document and claims no Core conformance.
7. **Command.** `robolang simulate FILE.rob --world WORLD.json [--task NAME]`, kept separate
   from `validate` and `inspect`. Exit code 0 only for `SUCCEEDED`; a distinct nonzero code
   for each of: front-end errors, world-file errors, `FAILED`, `CANCELLED`.

### 6.3 Proposed simulation semantics

These are proposals for the owner to decide. The RL-BASIC-01 specification PR freezes them
before implementation.

| Construct | Proposed behavior in simulation |
|---|---|
| `pre` block | Every condition is evaluated before the first step. False or unknown fails the run with zero steps executed |
| `robot_ready r` | True only if the world declares `r` ready |
| `region_clear c` | True only if the world declares `c` clear at the current simulated time. Undeclared is unknown, which fails closed |
| `object_exists o` | True only if `o` is present in the world |
| `sensor_ok "s"` | True only if a sensor `s` is declared and available |
| `move r to dst` | Requires `r` ready. Sets the location of `r` to `dst`; an object held by `r` moves with it |
| `grasp r o` | Requires a gripper in the profile of `r`, an empty gripper, and `o` at the location of `r` |
| `place r o at dst` | Requires `r` to hold `o` and to be at `dst`. Puts `o` at `dst` and empties the gripper |
| `inspect r o using "s"` | Requires sensor `s` on `r` and available. Records an observation of `o` in the trace. No default sensor |
| `wait for d` | Advances the simulated clock by `d`. Never sleeps in real time |
| `wait until c timeout d` | Advances the simulated clock through scripted events until `c` is true, or fails with a timeout code when `d` expires |
| `communicate r to "ch" with "m"` | Appends the message to the simulated channel log. It records sending only; it does not claim delivery or acknowledgement |
| `@safety max_speed` | Recorded in the run record with its unit. A limit with a unit is checked against the robot profile, and a limit above the profile's maximum fails before the first step. A unitless limit (`W401`) is recorded as declared and not enforced. Motion timing is not modelled |
| Failing step | The run ends `FAILED` at that step. Later steps do not run. No step is skipped |
| Injected fault | A fault scripted for a step fails that step; the run is `FAILED`, never `SUCCEEDED` |
| Cancellation | A cancellation scripted at a step ends the run `CANCELLED` before that step |

The example `examples/pick_and_place.rob` runs to `SUCCEEDED` under these rules: `move r to
src` puts the robot at the object, `grasp` and `move r to dst` carry it, and `place … at dst`
happens at the robot's location.

### 6.4 Acceptance tests

Each test runs in CI without ROS 2, network access or hardware. Fixtures live beside the
existing ones.

| # | Case | Expected result |
|---|---|---|
| T1 | `pick_and_place.rob` with a complete world | `SUCCEEDED`; box at `dst`; gripper empty; five steps succeeded; one message on `fleet` |
| T2 | Each existing invalid fixture | Front-end error exit code; no run record; zero simulated steps |
| T3 | `region_clear` false in `pre` | `FAILED` before the first step; zero steps executed |
| T4 | A condition the world does not declare | `FAILED`; unknown is treated as false |
| T5 | `grasp` by a robot without a gripper | `FAILED` at `grasp`; earlier steps recorded; later steps not run |
| T6 | `grasp` of an object at another location | `FAILED` at `grasp` |
| T7 | `place` of an object the robot does not hold | `FAILED` at `place` |
| T8 | `inspect` with a missing or unavailable sensor | `FAILED` at `inspect` |
| T9 | `wait until` whose condition becomes true through a scripted event before the timeout, and the same without the event | First continues; second `FAILED` with the timeout code at the stated simulated time |
| T10 | `communicate` | Message in the channel log; run record does not claim delivery |
| T11 | Same source, world and run seed twice | Byte-identical run records |
| T12 | File with two tasks and no `--task` | Error; no execution. Single-task file runs without `--task` |
| T13 | Parameter unbound in the world, or bound to an entity of the wrong type | World-file error before `RUNNING`; zero steps |
| T14 | World file with an unknown field or version | Rejected before execution |
| T15 | Injected fault at step 3 | `FAILED` at step 3, never `SUCCEEDED` |
| T16 | Cancellation scripted at step 2 | `CANCELLED`; step 2 and later not run |
| T17 | `max_speed` above the robot profile's maximum | `FAILED` before the first step |
| T18 | Import check | The executor package imports no ROS 2 or adapter module; every run record says `"mode": "simulation"` |
| T19 | Regression | All 76 RoboLang 1.1 fixtures and the existing test suite pass unchanged |

### 6.5 Out of scope

ROS 2 and the existing adapters; physical hardware; RoboLang Core schemas and the A2R and
R2R profiles (RL-ARCH-02); concurrency; new `.rob` syntax; executing externally supplied IR;
AI model calls; changes to the v1 interpreter and its demo, which keep working unchanged.

### 6.6 Delivery

Two pull requests, each needing owner approval to merge:

1. **RL-BASIC-01a — specification.** The simulation semantics of section 6.3 as decided by
   the owner, the world-file and run-record formats, and the fixtures for section 6.4.
   Documentation and fixtures only.
2. **RL-BASIC-01b — implementation.** Executor, simulated world and `simulate` command, with
   the tests of section 6.4.

**Exit gate (proposed):** T1–T19 pass in CI on every supported platform; the README states
that `simulate` is simulation only; owner approval.

## 7. Proposed D8 scheduling amendment

**Approved D8 (2026-10-19):** RL-ARCH-02 is the next planned milestone; its implementation
requires separate explicit owner authorization.

**Proposed amendment:** Foundation 1, including RL-BASIC-01, is prioritized before
RL-ARCH-02. RL-ARCH-02 stays approved as a milestone and still needs its own explicit
authorization. It is scheduled after the Foundation 1 exit gate and does not have to wait for
Foundation 2.

Proposed development sequence:

| # | Step | State |
|---|---|---|
| 1 | RFC-001 | Approved and merged |
| 2 | Development-plan alignment (rokorobot/RoboLang#9) | Ready for review |
| 3 | Language Foundation specification (this document) | Proposed |
| 4 | Foundation 1: continue developing the original RoboLang functions | After approval of this document |
| 5 | RL-BASIC-01: functional simulation executor | After approval of this document and of RL-BASIC-01a |
| 6 | RL-ARCH-02 | After the Foundation 1 exit gate and separate authorization |

Reasons:

- RL-ARCH-02 needs robots that execute and report outcomes. RL-BASIC-01 builds that
  capability for the language that already exists, and RL-ARCH-02 can reuse its executor
  design.
- It keeps the existing investment in the 1.1 front end producing user-visible behavior
  before the larger Core work starts.
- Every step stays simulation only.

The developer-preview pilot of RoboLang 1.1 stays independent of this order, as before.

If the amendment is approved, its approval and date are recorded in decision record 0003
(the approved rows of decision record 0002 are not rewritten), and the plan's milestone order in
`docs/DEVELOPMENT_PLAN.md` is updated in a separate PR.
