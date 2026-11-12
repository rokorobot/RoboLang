# Changelog

The distribution version (this file) is independent of the language versions (1.1 to 1.6) and the Task IR schema
versions (1.0 to 1.5). Decision records are in [`spec/decisions/`](https://github.com/rokorobot/RoboLang/tree/main/spec/decisions).

## Unreleased

- Optional extra `robolang[ed25519]` (installs `cryptography`): real Ed25519 signatures for Core documents
  (`sign_ed25519`, `verify_ed25519`, `generate_ed25519`), the first build step of decision 0038. A receiver accepts
  them only when its `Context.algorithms` lists `ed25519`; by default nothing changes. Simulation only: physical mode
  is still rejected. The base install still has no runtime dependencies.

## 0.3.1 (2026-11-12)

**First run from PyPI alone**
- `robolang example [DIR]` writes a sample task (`fetch.rob`) and a world for it (`world.json`), so a pip user can
  reach a verified simulation without cloning the repository. It never overwrites a file.
- A world file that lacks several required fields now names all of them in one error, and a missing `world_schema`
  or `world_schema_version` names the accepted values. Codes, exit statuses and the first-error rule are unchanged.
- The `simulation only:` summary line says what a failure means and at which step, after the code:
  `FAILED (S301): a goal condition is false after the last step`. The run record is unchanged.
- README quickstart with a task and world that need only the installed package.

## 0.3.0 (2026-11-11)

Everything below is simulation only. No command drives a robot. Files of every earlier language version keep their
meaning, IR and run records byte for byte.

**Language**
- RoboLang 1.5: step failure handlers (0033, 0034). `move`, `grasp`, `place`, `inspect` and `wait until` may end with
  `else { ... }`, run only when that step fails; a handler has no goal; a fixed number of retries is written by nesting;
  every `wait until` needs a handler (`E316`). Task IR 1.4 (`I217`).
- RoboLang 1.6: measured values (0036). `measure r KIND of o using s as NAME;` reads a temperature, mass, distance or
  label; bindings compare (`<`, `<=`, `>`, `>=`; `==`, `!=` on labels) in `if` and `wait until`, with exact unit
  conversion and the sensor's uncertainty (an overlapping reading is unknown, `S261`); `wait until` can read live;
  `communicate ... with NAME` sends the typed value. Reserved words `measure` and `as` in 1.6 files only. New codes
  `E217`, `E317` to `E322`. Task IR 1.5 (`I124`, `I125`, `I218` to `I223`).
- Holder check (0032): grasping something a robot still holds is `E315` (IR `I216`); a branch the known facts
  contradict is skipped.
- `robolang reference` prints the RoboLang 1.6 reference (`inspect` never reads a value; failure handlers; measured
  values), tested against the grammar and examples. The CLI help names 1.1 to 1.6.

**Execution (symbolic simulation)**
- 1.5 runs record which step's failure a handler took over (run record 1.4).
- World file 1.2: sensor `measures` (unit, uncertainty, contact), object `properties`, and the `property` event (a value
  that changes over time). New codes `E512` to `E515`. `measure` fails with `S231` to `S236`, `S213` or `S214` and
  changes nothing. Run record 1.5 records every reading and typed message.
- `validate --world` checks handler steps and reports a sensor that cannot measure the kind asked for (`S234`).
- `delegate` refuses a task with a `measure` (`measure_unsupported`).
- The IR reader reports `I209` for a schema 1.3+ task with no `goal` (it raised an error before).

**RoboLang Core 0.2** (frozen by the owner on 2026-10-30)
- Schema, receiver checks and conformance fixtures for Core 0.2 ("any of" goal alternatives, declared failure reasons,
  `met_alternative`); producers: `export-goal --core 0.2`, Core-shaped outcomes from `simulate`, `delegate --core 0.2`.
- Gate 2 passed on 2026-11-10: an independent Core 0.2 implementation, written from the frozen documents alone,
  interoperates with the reference on all 25 cases (docs/interop/interop-4-run-2.md). Errata 0037 clarify two points of
  Core 0.1/0.2 and the test binding; no message, schema, code or receipt changed.

**Benchmark** (`bench/`, not installed)
- Set 4 (failure handling) and set 5 (measured values), each pre-registered with a baseline; the reference teaches 1.6;
  new measures `unsafe_continue`, `value_ok` and `silent_substitution`; acceptance evidence for 1.5 and 1.6 in
  `docs/evidence/`.

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
