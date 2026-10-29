# Decision record 0016 — Task bindings on the command line (`simulate --bind`)

**Date:** 2026-10-29
**Status:** **IMPLEMENTED, owner approval pending.** Task C of the simulate work; not merged.
**Applies to:** [`spec/simulation-v1.0.md`](../simulation-v1.0.md) sections 2.5, 3.5, 5.1 and 6
**Builds on:** [decision record 0004](0004-rl-basic-01a-simulation-semantics.md) (bindings live in
the world file)

## 1. Why

A task's bindings had to be written inside the world file. To run a second task, or the same task
with another robot, you had to edit the world. `--bind NAME=ID` lets one world serve many runs.

## 2. Decisions

| # | Topic | Rule | Alternative considered |
|---|---|---|---|
| C1 | Syntax | `--bind NAME=ID`, repeatable; one flag may also carry several pairs (`--bind r=arm1 box=box1`). `FILE` goes before `--bind` | Repeat-only: stricter, but the several-pairs form is what people type and argparse handles it cleanly |
| C2 | Override | If any `--bind` is given, only the CLI bindings are used. The world's `bindings` for the task are ignored entirely. Its `faults` and `cancel` still apply. The task entry may be missing | Merge CLI over world: a run's bindings would depend on two places, and a forgotten world binding would pass silently |
| C3 | Validation | Same rules as the world file: all parameters bound once, no unknown names, ID declared, right kind. Reuses `E505`, `E506`, `E507`; no new codes | New E5xx codes: not needed |
| C4 | Exit codes | Exit 2 for a malformed item, a repeated name, or a name that is not a parameter (usage errors, found before the world is read). Exit 3 for unbound parameter, undeclared ID, wrong kind (they depend on the world). Diagnostics name `--bind` instead of the world file | Report everything as exit 3: a typo in a parameter name is not a world fault |
| C5 | Run ID | With `--bind`, the preimage gets one extra sorted line `bind NAME=ID` per binding. Without `--bind` it is unchanged, so existing run IDs and fixtures stay valid. Flag order does not matter | Hash the CLI bindings into a new world hash: would also change IDs of runs that do not use `--bind` |
| C6 | Record | `inputs.bindings_source: "cli"` is added only for `--bind` runs. `bindings` already holds the bindings used | Always write `"world"` or `"cli"`: clearer, but it changes every existing record without a schema version bump |

## 3. Consequences

- Records and run IDs of runs without `--bind` are byte-identical to before.
- A reader treats a missing `inputs.bindings_source` as `"world"`. If the owner prefers the key
  always present (C6 alternative), that is a record schema change and needs a version decision.
- Run IDs of `--bind` runs differ from the same bindings given in a world file, because the
  inputs differ (a different world hash and the extra lines).
- No change to RoboLang, the IR, the world schema or the validator.
