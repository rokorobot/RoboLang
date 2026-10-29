# Decision record 0019 — `export-goal`: a `.rob` goal as a Core `propose_goal`

**Date:** 2026-10-29
**Status:** **IMPLEMENTED, owner approval pending.** Phase 3 of
[`docs/plans/goals-and-core-bridge.md`](../../docs/plans/goals-and-core-bridge.md); not merged.
**Applies to:** [`export-goal-v0.1.md`](../export-goal-v0.1.md), [RoboLang 1.3](../robolang-v1.3.md),
[IR 1.2](../ir-schema-v1.2.md), [simulation](../simulation-v1.0.md) section 3
**Builds on:** plan decisions Q2, Q3, Q5 (approved 2026-10-29); [0014](0014-lang-2-goals-and-typed-sensors.md)

## 1. Why

An AI writes `.rob`; a robot understands Core. `export-goal` turns a task's goal into the signed
message an agent sends (Core 4.1 `propose_goal`). The plan stays local (Core R3).

## 2. Decisions

| # | Topic | Rule | Alternative considered |
|---|---|---|---|
| X1 | `@verify` | Amends 1.3, no 1.4 (1.3 is unreleased): `@verify min_confidence D;` and `@verify max_age DURATION;`, each at most once per task (`E311`), confidence 0 to 1 (`E312`), max_age > 0 (`E305`). 1.3 files only; in 1.1/1.2 the lexer still says `E101`. IR 1.2 `verify` object, omitted when absent, so no existing snapshot changes | A 1.4: needless while 1.3 is unreleased (plan Q2 said 1.4 before 1.3 was frozen) |
| X2 | Defaults | Without `@verify`: `min_confidence` `"0.9"`, `max_age` 10 s, the values of the Core fixtures | Require `@verify`: breaks all existing 1.3 files |
| X3 | World file 1.1 | Objects may have `class` and `mass`; 1.0 files still reject them (`E503`). `simulate` ignores them. The run record's `world_schema_version` is the file's own | Add them to 1.0: changes a released schema |
| X4 | Things | Every distinct object bound in the task, not only goal objects, so the robot knows what it may handle. A missing class or mass is `X602`, never invented | Only objects in the goal: hides a payload the plan moves |
| X5 | Exportable | Only a plain conjunction of outcome atoms (plan Q3); otherwise `X601` and exit 6, naming the condition | Export `or`/`not` as several goals: changes the meaning |
| X6 | Constraints | `m/s` limits only. `rad/s` and unitless limits are skipped with a warning, not an error: the goal is still correct, the limit is only declared, never enforced here | Fail on a skipped limit: blocks a valid goal for a limit Core cannot state |
| X7 | Recipient | `--to`, else the only robot parameter; several robots or none is a usage error (2), because choosing one would be a guess | First robot parameter: a silent guess |
| X8 | Determinism | Time defaults to the simulation epoch; IDs are hashes of sender, recipient and payload (not `sent_at`); the key is derived from its reference. Same inputs, same bytes | Random IDs and the wall clock: untestable |
| X9 | Signature | `sim-hmac-sha256` with a public key derived from `<from>#k1` (spec section 4). Simulation only; a physical-mode receiver rejects it (`C204`) | A key file: more setup for no extra safety in simulation |
| X10 | Exit 6, codes | New exit status 6 and diagnostics `X601`, `X602`. Exit 2 stays for usage, 3 for world and bindings | Reuse 3 for a missing mass: it is the world's content, but the file is valid |

## 3. Consequences

- Existing 1.1/1.2 sources, IR, run records and run IDs are unchanged. `examples/world.json` is now
  1.1, so run records of the examples show `world_schema_version` `1.1` and their run IDs change
  (the world's bytes changed); 1.0 world fixtures are untouched.
- `arch02`, the gate 1 evidence and the frozen gate 2 bundle are untouched.
- A skipped `rad/s` limit is only a warning on stderr: a caller that needs every limit in the
  message must check it.
