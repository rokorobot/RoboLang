# Held-out set: preregistration

*Fill in and approve before the first run. The approved copy is committed in RoboLang's `bench/heldout/`
together with `manifest.sha256`. After that, only the "Runs" and "Errata" sections change.*

## Set

- Frozen on: YYYY-MM-DD, manifest SHA-256 of `manifest.sha256`: `...`
- Worlds: N (names)
- Writers: N (`w1` to `wN`), none of whom has worked on RoboLang
- Tasks: N total. feasible N, decoy N, clarify N, infeasible N, inexpressible N
- Dropped while labelling: N (see `raw/dropped.md`)
- Labels checked by: (initials), on YYYY-MM-DD

## Targets (fixed now, not after seeing results)

| Measure | Target |
|---|---|
| valid program within two attempts (`valid_after_repair`), tasks expecting a program | ≥ 90% |
| `false_program`: a program for a task expecting `cannot` or `clarify` | 0 |
| decision correct (`decision_correct`) | ≥ ...% |
| goal correct (`goal_correct`) | ≥ ...% |

## How it is run

- RoboLang version / commit: ...
- Prompt: `full`, prompt hash: ...
- Models: ...
- Effort: ..., repeats: ...
- Milestones at which it is run: v2.0 sign-off; ...

## Runs

| Date | Milestone | RoboLang commit | Models | Result against targets | Report |
|---|---|---|---|---|---|

## Errata

| Date | Task | Change | Why |
|---|---|---|---|
