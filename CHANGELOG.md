# Changelog

The distribution version (this file) is independent of the language versions (1.1 to 1.4) and the Task IR schema
versions (1.0 to 1.3). Decision records are in [`spec/decisions/`](https://github.com/rokorobot/RoboLang/tree/main/spec/decisions).

## 0.2.0 (2026-10-30): first PyPI release

Everything below is simulation only. No command drives a robot, loads an adapter or uses ROS 2.

**Language**
- RoboLang 1.4: bounded `if` / `else` / `else if` in plans, with per-branch `goal` blocks, and `fail "reason";`
  for report-and-stop branches. Nesting depth is at most 4, and every path needs a goal or a `fail` (0027; #70, #72, #73).
- RoboLang 1.3: a required `goal` decides success, with the outcome atoms `object_at`, `robot_at` and `holding`,
  and typed `sensor` parameters (0014; #54).
- RoboLang 1.2: combined conditions with `and`, `or` and `not` (#22, #24).
- `validate` reports every error, not just the first (0015; #52).
- `validate --world W [--task T] [--bind …]`: before running anything, reports the failures a run would certainly
  hit in that world, path by path for branching plans. These are speed limits against robot profiles, grasp without
  a gripper, and a sensor on another robot (0026; #66, #73).
- `robolang reference` prints the shipped, tested language reference (#64, #73).

**Execution (symbolic simulation)**
- `simulate`: a deterministic run record and run ID, with world files 1.0 and 1.1 and `--bind` (0004, 0016; #13–#15, #53).
- 1.4 runs record every decision. An unknown condition fails the run (`S261`) and `fail` ends it with `S262`.
- `--ir`: every command reads Task IR 1.0 to 1.3 strictly (0021).

**RoboLang Core 0.1**
- Document checks (`robolang.core`) and the RFC-001 reference simulation (`robolang arch02`) (#26–#30).
- Core-shaped outcomes from simulated runs, `export-goal` and `delegate` with a peer, commitment, lease and custody
  (0018–0020, 0022; #55–#57). Conjunctive goals export (#63). A branching task exports when all its paths share
  one goal (0027 B7).

**Benchmark** (`bench/`, not installed)
- The authoring benchmark has set 1 (syntax), set 2 (robustness), set 3 (branching) and the "ask" set. Goals are
  compared by meaning and branching answers are scored path by path. A CI gate checks decision accuracy and false
  programs (0024, 0025; #59–#73).

## 0.1.0a1 (2026-10-11, not published)

- RoboLang 1.1: lexer, parser, validator, versioned Task IR and a read-only CLI (`validate`, `inspect`) (#3–#7).
