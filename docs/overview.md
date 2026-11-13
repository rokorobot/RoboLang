# RoboLang documentation map

The current language is **RoboLang 1.6** (files of 1.1 to 1.5 keep their meaning), with Task IR
1.0 to 1.5, simulation-only execution and RoboLang Core 0.1 and 0.2 messages in simulation. The
installed package is `robolang` 0.3.1. Physical robot execution is not implemented or
authorized. Start with the [home page](index.md) or the
[README](https://github.com/rokorobot/RoboLang#readme).

## Normative specifications

These define what the implemented tooling does. They live in the repository's `spec/` folder.

- [Language reference (1.6)](language-reference.md): what `robolang reference` prints; the
  page for task authors
- RoboLang language [1.1](https://github.com/rokorobot/RoboLang/blob/main/spec/robolang-v1.1.md), [1.2](https://github.com/rokorobot/RoboLang/blob/main/spec/robolang-v1.2.md),
  [1.3](https://github.com/rokorobot/RoboLang/blob/main/spec/robolang-v1.3.md) (goals, typed sensors), [1.4](https://github.com/rokorobot/RoboLang/blob/main/spec/robolang-v1.4.md)
  (bounded `if`/`else`), [1.5](https://github.com/rokorobot/RoboLang/blob/main/spec/robolang-v1.5.md) (failure handlers) and
  [1.6](https://github.com/rokorobot/RoboLang/blob/main/spec/robolang-v1.6.md) (measured values), with
  [grammars (EBNF)](https://github.com/rokorobot/RoboLang/tree/main/spec)
- [Task IR schema 1.0](https://github.com/rokorobot/RoboLang/blob/main/spec/ir-schema-v1.0.md) to [1.5](https://github.com/rokorobot/RoboLang/blob/main/spec/ir-schema-v1.5.md): the JSON
  printed by `robolang inspect --json`, and the [IR reader](https://github.com/rokorobot/RoboLang/blob/main/spec/ir-reader-v0.1.md) behind `--ir`
- [Simulation semantics, world files and run records](https://github.com/rokorobot/RoboLang/blob/main/spec/simulation-v1.0.md): what
  `robolang simulate` does
- [RoboLang Core 0.1](https://github.com/rokorobot/RoboLang/blob/main/spec/core-v0.1.md) and [Core 0.2](https://github.com/rokorobot/RoboLang/blob/main/spec/core-v0.2.md) (frozen
  2026-10-30), the [Core 0.1 reference simulation](https://github.com/rokorobot/RoboLang/blob/main/spec/reference-simulation-v0.1.md) and
  the [gate 2 interoperability test](https://github.com/rokorobot/RoboLang/blob/main/spec/interop-gate-2-v0.1.md)
- [`export-goal`](https://github.com/rokorobot/RoboLang/blob/main/spec/export-goal-v0.1.md), [`delegate` 0.1](https://github.com/rokorobot/RoboLang/blob/main/spec/delegate-v0.1.md),
  [0.2](https://github.com/rokorobot/RoboLang/blob/main/spec/delegate-v0.2.md) and [Core 0.2 delegate](https://github.com/rokorobot/RoboLang/blob/main/spec/delegate-core-0.2.md)
- [Physical-run authorization 0.1](https://github.com/rokorobot/RoboLang/blob/main/spec/authorization-v0.1.md): signed cell configuration,
  task approval and run grant; a valid chain still ends in `A731` (physical mode not enabled)
- [ROS 2 / Gazebo backend 0.1](https://github.com/rokorobot/RoboLang/blob/main/spec/ros2-backend-v0.1.md): optional, repository only,
  simulation only

## Architecture

- [RFC-001 — AI-native semantic and communication architecture](https://github.com/rokorobot/RoboLang/blob/main/spec/architecture/rfc-001-ai-native-architecture.md):
  Semantic Core, A2R and R2R profiles, local authorization, verified outcomes. Core and the
  `.rob` → Core bridge are implemented in simulation; network transport, real signatures and
  physical execution are not
- [AI-First Language Design Charter](https://github.com/rokorobot/RoboLang/blob/main/spec/architecture/ai-first-language-charter.md)
- [RL-FOUNDATION-01 — language foundation](https://github.com/rokorobot/RoboLang/blob/main/spec/architecture/rl-foundation-01-language-foundation.md)
- [RL-FOUNDATION-02 — domain contracts](https://github.com/rokorobot/RoboLang/blob/main/spec/architecture/rl-foundation-02-domain-contracts.md)

## Strategy, decisions, plans and evidence

- [Strategy](STRATEGY.md): direction approved by the owner, reviewed monthly
- [Decision records](https://github.com/rokorobot/RoboLang/tree/main/spec/decisions)
- [Development plan](DEVELOPMENT_PLAN.md): milestones and development tracks; listing an item
  authorizes nothing
- [Development steps](development-steps.md): a timeline of what has been delivered so far
- [Safety case draft and hazard list](safety-case-draft.md): robot-independent; authorizes nothing
- Exit gates: [Foundation 1](evidence/rl-basic-01-exit-gate.md),
  [Core 0.1 reference simulation (gate 1)](evidence/rl-arch-02-exit-gate.md),
  [gate 2 independent interoperability](interop/interop-4-run-2.md)
- Benchmark: [history](evidence/benchmark-history.md), acceptance of
  [1.4](evidence/bench-set3-acceptance.md), [1.5](evidence/bench-1-5-acceptance.md) and
  [1.6](evidence/bench-1-6-acceptance.md)
- ROS 2 / Gazebo backend: [acceptance run](evidence/h3-phase4-acceptance.md)

## Current limitations

- The simulator tracks symbolic facts declared in a world file. No motion planning,
  kinematics, collisions or physics.
- `@safety max_speed` is checked against the robot profile and reported in the run record,
  never enforced.
- No hardware: nothing is sent to a robot. The installed package loads no adapter and does not
  use ROS 2; the optional Gazebo backend in the repository drives a simulated arm only.
- No network transport. Core signatures are simulation-only by default; optional Ed25519
  signatures (`robolang[ed25519]`) are on `main`, unreleased.

## Legacy v1

The original line-based interpreter, its print-only adapter and the four v1 PDF guides are
historical material; the experimental ROS 2 adapter was removed on 2026-11-13. They are not part of the installed package and bypass
the validator. See [Legacy RoboLang v1](legacy-v1.md).
