# Decision record 0004 — RL-BASIC-01a simulation semantics

**Date:** 2026-10-26
**Status:** **All sixteen decisions APPROVED — 2026-10-26**, as revised after owner review
(final approval given on the text at commit `ea5935e`). Implementation of RL-BASIC-01b is not
authorized by this record.
**Applies to:** [`spec/simulation-v1.0.md`](../simulation-v1.0.md),
[`tests/fixtures/simulation/`](../../tests/fixtures/simulation/README.md)

## 1. What this record does and does not do

It records the owner's decisions on the choices the RL-BASIC-01a specification makes. Rows
B1–B5 are the five choices that [decision record 0003](0003-language-foundation.md) section 3
left to RL-BASIC-01a; rows B6 onwards are further choices this specification had to make. A
later change to an approved row is recorded as a new row or a new decision record, with its
date; the approved rows are not rewritten.

It does not:

- change RoboLang 1.1, its grammar, parser, validator, IR schema 1.0, CLI or v1 runtime;
- change RFC-001 or decision records 0002 and 0003;
- authorize implementation. RL-BASIC-01b starts only after the owner separately authorizes
  implementation (decision record 0003, F4 and F5).

## 2. Decisions

| # | Topic | Decision | Spec | Status |
|---|---|---|---|---|
| B1 | `place` position | `place r o at dst` requires the robot to be at `dst` (`S222`); it does not move the robot | 4.5 | APPROVED — 2026-10-26 |
| B2 | Unknown state | A fact the world file does not assert is unknown, and unknown never counts as true: a `pre` condition fails with `S102`, a step requirement with its "unknown" code. An undeclared sensor in `sensor_ok` is unknown | 3.3, 4.4 | APPROVED — 2026-10-26 |
| B3 | `wait until` | A timeout ends the run `FAILED` with `S241`. The bound is inclusive: a condition that becomes true exactly at the deadline succeeds | 4.5 | APPROVED — 2026-10-26 |
| B4 | `max_speed` | A configuration check at authorization, never enforcement: a limit with a unit above a bound robot's declared maximum fails with `S001`, a robot with no maximum for that unit fails with `S002`, a unitless limit is not checked. Statuses `checked_against_profile_not_enforced` and `unit_unspecified_not_enforced`; no record reports a limit as enforced | 4.3 | APPROVED — 2026-10-26 |
| B5 | `communicate` | Records the message with status `recorded`; delivery and acknowledgement are neither modelled nor claimed | 4.5 | APPROVED — 2026-10-26 |
| B6 | Run ID | SHA-256 over a fixed preimage of the source hash, world hash, task name and seed; both files hashed after removing a byte order mark and turning CR LF into LF, so the ID is the same on Windows and Linux checkouts. Seed: 1–64 characters from `A–Z a–z 0–9 . _ -`, default `0`. The ID identifies an input-equivalent simulation run, not an occurrence: the same inputs and seed intentionally give the same ID and record, and a future operational executor may need a separate invocation identifier. Considered and not proposed: hashing the canonical IR instead of the source, which would ignore comments but would no longer identify the source file | 6 | APPROVED — 2026-10-26 |
| B7 | Simulated clock | Exact decimal milliseconds, no floating point. Only `wait for` and `wait until` take time; every other step takes zero time. Events are absolute times after the run starts, greater than zero, applied when the clock reaches them | 4.2, 3.6 | APPROVED — 2026-10-26 |
| B8 | Readiness | "Ready" means ready for physical actions, not a statement about every subsystem. `move`, `grasp` and `place` require it (`S201` false, `S202` unknown). `inspect` depends on its sensor's availability instead, and `communicate` only records a simulated message; neither implies that real sensing or communication hardware would work on a robot that is not ready | 3.3, 4.5 | APPROVED — 2026-10-26 |
| B9 | `inspect` | Requires the named sensor mounted on the robot and available. It records what it observes about the object and does not fail because of the object's state | 4.5 | APPROVED — 2026-10-26 |
| B10 | Preconditions | All `pre` conditions are evaluated at time 0 and recorded; the failure code is that of the first one not true | 4.4 | APPROVED — 2026-10-26 |
| B11 | Faults and cancellation | Scripted per task in the world file by 1-based step index. A fault fails its step with `S251` before requirements are checked; a cancellation ends the run `CANCELLED` before the step executes | 3.5, 4.6 | APPROVED — 2026-10-26 |
| B12 | World file 1.0 | One JSON document with locations, regions, robots, objects, sensors, events and per-task entries holding bindings, faults and cancellation. Entity IDs unique across kinds; unknown fields rejected; omitted facts are unknown; no JSON numbers except step indices | 3 | APPROVED — 2026-10-26 |
| B13 | Robot profile | Minimal: `gripper`, mounted sensors and `max_speed` maxima. Every robot can reach every location | 3.4 | APPROVED — 2026-10-26 |
| B14 | Run record 1.0 | Fixed key order, IR formatting rules, codes and structured values only, final world sorted by ID. `"mode": "simulation"` in every record | 5 | APPROVED — 2026-10-26 |
| B15 | Command and exit status | `robolang simulate FILE --world WORLD [--task NAME] [--seed SEED]`; run record on stdout. Exit 0 succeeded, 1 invalid source, 2 usage, 3 invalid world file, 4 failed, 5 cancelled; 0–2 keep their existing meaning | 2 | APPROVED — 2026-10-26 |
| B16 | Codes | `E501`–`E511` for world-file errors (diagnostics located by JSON Pointer); `S001`–`S251` for run failures, recorded in the run record | 3.9, 7 | APPROVED — 2026-10-26 |

## 3. Relationship to open questions

None of RFC-001's open questions is resolved here. The world file is a simulation input, not
a RoboLang Core world model (open question 2), and the run record is not a Core outcome
document, although its fields are chosen so that a later mapping onto Core outcomes
(RFC-001 section 7.10) is possible.

## 4. Revision history

| Date | Change |
|---|---|
| 2026-10-26 | First draft: B1–B16 proposed |
| 2026-10-26 | Owner review: B1–B16 recommended for approval, with two clarifications. B8 defines readiness as readiness for physical actions. B6 states that the run ID identifies an input-equivalent simulation run, not an invocation. No behavior and no fixture expectation changed |
| 2026-10-26 | Final owner approval of B1–B16 as revised, on the text at commit `ea5935e`. Status updated to APPROVED; no change to the decision text |
