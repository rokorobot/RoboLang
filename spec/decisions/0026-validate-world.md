# Decision record 0026: World checks while authoring (`validate --world`)

**Date:** 2026-10-29
**Status:** **IMPLEMENTED, owner approval pending.** Development plan item N2; not merged.
**Applies to:** [`spec/simulation-v1.0.md`](../simulation-v1.0.md) section 11, the benchmark's repair loop
**Evidence:** [benchmark set 2 analysis](../../docs/evidence/bench-set2-analysis.md), F4 and F5: speed-limit,
gripper and sensor errors were found only by `simulate`, after the repair round

## 1. Why

An AI authoring a task gets a diagnostic from `validate` and one chance to repair it. A limit above the robot's
profile, a grasp by a robot with no gripper, or a camera on the wrong robot passed `validate`. They failed only
at simulation, too late for the repair loop.

## 2. Decisions

| # | Topic | Rule | Alternative considered |
|---|---|---|---|
| W1 | Where | A `--world` option on `validate`, with `simulate`'s `--task` and `--bind` | A new command: one more name to learn for the same loop |
| W2 | What | Only failures that are **certain** for a straight-line plan: `S001`, `S002`, `S211`, `S231`. They reuse the run's codes and rules | Warnings for possible failures (readiness, presence): they depend on events and steps, so this would be a second simulator |
| W3 | Severity | Errors, exit 1, like any invalid program. World and binding errors keep `simulate`'s exit 3 | A new exit code: the caller's question ("can this program succeed?") is the same |
| W4 | All at once | Every limit, every step, in one pass (as in [0015](0015-report-all-diagnostics.md)) | Stop at the first, as the run does: one repair round per error |
| W5 | Benchmark | `bench/lib.py` `check_answer` runs `validate --world` with the answer's bindings, so these diagnostics reach the repair round | Leave the benchmark alone: then N2 cannot be measured |

## 3. Consequences

- No change to the language, the IR, the world schema, the run record or `simulate`.
- When N4 adds branches, a step in a branch is no longer certain to run, and W2 must be revisited.
- Benchmark scores are not comparable for answers with these failures. Earlier they were valid and failed
  in simulation. Now they are invalid on the first answer and get a repair round. `valid_first` can fall
  while `plan_succeeds` rises.
