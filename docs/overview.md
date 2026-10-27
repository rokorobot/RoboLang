# RoboLang documentation map

The current language is **RoboLang 1.1**, with Task IR 1.0 and simulation-only execution
(Foundation 1, signed off 2026-10-27). The AI-native Core, A2R and R2R architecture is
specified and approved, not implemented. Physical robot execution is not implemented or
authorized. Start with the [home page](index.md) or the
[README](https://github.com/rokorobot/RoboLang#readme).

## Normative specifications

These define what the implemented tooling does. They live in the repository's `spec/` folder.

- [RoboLang 1.1 language](https://github.com/rokorobot/RoboLang/blob/main/spec/robolang-v1.1.md)
  and [grammar (EBNF)](https://github.com/rokorobot/RoboLang/blob/main/spec/grammar.ebnf)
- [Task IR schema 1.0](https://github.com/rokorobot/RoboLang/blob/main/spec/ir-schema-v1.0.md):
  the JSON printed by `robolang inspect --json`
- [Simulation semantics, world file 1.0 and run record 1.0](https://github.com/rokorobot/RoboLang/blob/main/spec/simulation-v1.0.md):
  what `robolang simulate` does

## Architecture (approved, not implemented)

- [RFC-001 — AI-native semantic and communication architecture](https://github.com/rokorobot/RoboLang/blob/main/spec/architecture/rfc-001-ai-native-architecture.md):
  Semantic Core, A2R and R2R profiles, local authorization, verified outcomes
- [AI-First Language Design Charter](https://github.com/rokorobot/RoboLang/blob/main/spec/architecture/ai-first-language-charter.md)
- [RL-FOUNDATION-01 — language foundation](https://github.com/rokorobot/RoboLang/blob/main/spec/architecture/rl-foundation-01-language-foundation.md)
- [RL-FOUNDATION-02 — domain contracts](https://github.com/rokorobot/RoboLang/blob/main/spec/architecture/rl-foundation-02-domain-contracts.md)

## Decisions, plans and evidence

- [Decision records](https://github.com/rokorobot/RoboLang/tree/main/spec/decisions)
- [Development plan](DEVELOPMENT_PLAN.md): milestones and development tracks; listing an item
  authorizes nothing
- [Foundation 1 exit-gate evidence](evidence/rl-basic-01-exit-gate.md)

## Current limitations

- The simulator tracks symbolic facts declared in a world file. No motion planning,
  kinematics, collisions or physics.
- `@safety max_speed` is checked against the robot profile and reported in the run record,
  never enforced.
- No hardware and no ROS 2 runtime: no adapter is loaded and nothing is sent to a robot.

## Legacy v1

The original line-based interpreter, the simulated and experimental ROS 2 adapters and the four
v1 PDF guides are historical material. They are not part of the installed package and bypass
the RoboLang 1.1 validator. See [Legacy RoboLang v1](legacy-v1.md).
