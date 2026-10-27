# RL-BASIC-01 / Foundation 1 exit gate — evidence

**Date:** 2026-10-27
**Status:** **Signed off by the owner on 2026-10-27**, on the evidence at `f5b3d23`. At the owner's request a
docstring-only commit followed (`800fa27`); the stack head is now `69a71e3`, re-tested below. Nothing here is merged.
**Addendum (2026-10-27):** sections 1–7 describe the stack before integration and are kept as
written. The stack has since been merged into `main` at `efd5a73`; see [section 8](#8-post-merge-addendum).
**Gate:** RL-BASIC-01 exit gate ([RL-FOUNDATION-01 section 6.6](../../spec/architecture/rl-foundation-01-language-foundation.md#66-delivery)),
which is also the Foundation 1 exit gate (decision F3, [decision record 0003](../../spec/decisions/0003-language-foundation.md)).
**Specification:** [`spec/simulation-v1.0.md`](../../spec/simulation-v1.0.md) (B1–B16, [decision record 0004](../../spec/decisions/0004-rl-basic-01a-simulation-semantics.md))

## 1. Gate criteria

| Criterion | Source | Evidence | Met |
|---|---|---|---|
| T1–T19, as finalized in RL-BASIC-01a, pass in CI on every supported platform | RL-FOUNDATION-01 6.6 | Section 3; CI runs in section 2, Ubuntu and Windows, Python 3.11 and 3.13 | Yes |
| All RoboLang 1.1 fixtures still pass | F3 | `tests/test_fixtures.py`: 77 tests (the 76 fixtures and their count check) pass; `validate` and `inspect` pass every fixture in `tests/test_cli.py` | Yes |
| The README states that `simulate` is simulation only | RL-FOUNDATION-01 6.6 | `README.md`, Project Status row and Quickstart bullet (PR #15) | Yes |
| Owner sign-off | F3, RL-FOUNDATION-01 6.6 | Section 7 | Yes — 2026-10-27 |

## 2. What was tested

The three slices are stacked draft pull requests. Each head below was tested by the CI workflow
(`.github/workflows/ci.yml`: ruff, the full test suite, the v1 demo, and `tools/smoke_install.py`,
which builds, installs and runs the installed `robolang` command) in four jobs: Ubuntu and
Windows, Python 3.11 and 3.13.

| Slice | Pull request | Head commit | CI run | Result |
|---|---|---|---|---|
| BASIC-1 — values and world file 1.0 | [rokorobot/RoboLang#13](https://github.com/rokorobot/RoboLang/pull/13) | `c5a61e30bf5c7d8698e5b66593e2fed7d19b13aa` | [40426572425](https://github.com/rokorobot/RoboLang/actions/runs/40426572425) | 4/4 passed |
| BASIC-2 — executor | [rokorobot/RoboLang#14](https://github.com/rokorobot/RoboLang/pull/14) | `800fa275c360e9429731dc53be0974b37b7aaf67` | [40427544813](https://github.com/rokorobot/RoboLang/actions/runs/40427544813) | 4/4 passed |
| BASIC-3 — run record, run ID, `robolang simulate` | [rokorobot/RoboLang#15](https://github.com/rokorobot/RoboLang/pull/15) | `69a71e3fe70346d612b7591c2315baed4e5f86bb` | [40427546315](https://github.com/rokorobot/RoboLang/actions/runs/40427546315) | 4/4 passed |

`69a71e3` contains all three slices. The acceptance tests are judged at that commit. It differs from
`f5b3d23`, the commit signed off, only by the docstring of `run()` (`800fa27`).

Commits in the three slices, oldest first:

| Commit | Content |
|---|---|
| `e4307bb` | BASIC-1: simulation values and world file 1.0 validation |
| `e5fd481` | BASIC-2: simulation executor |
| `7fad80c` | BASIC-3: run record, run ID and `robolang simulate` |
| `c5a61e3` | BASIC-1 review fix: the loaded world is a read-only snapshot |
| `355dda6` | Merge of the BASIC-1 fix into BASIC-2 |
| `2bacd7d` | BASIC-2 review fix: `run()` rejects a task and world that do not belong together |
| `f5b3d23` | Merge of the BASIC-2 fix into BASIC-3 (evidence signed off) |
| `800fa27` | BASIC-2: docstring states that externally supplied IR is unsupported by `run()` (owner request; no behavior change) |
| `69a71e3` | Merge of the docstring into BASIC-3 |

**Local run at `69a71e3`** (Windows, Python 3.11.15): `python -m pytest -q` gives 868 passed,
6 xfailed, 0 failed; `python -m ruff check .` gives no issues. The 6 expected failures are the
documented v1 interpreter limitations in `tests/test_known_limitations.py`, unchanged.

## 3. Acceptance tests T1–T19

Fixture cases are defined in [`tests/fixtures/simulation/cases.json`](../../tests/fixtures/simulation/cases.json).
Each case is checked at three layers: the world loader (`test_sim_world.py::test_fixture_case`),
the executor (`test_sim_executor.py::test_fixture_case`) and the real command
(`test_sim_cli.py::test_case`). `test_simulation_fixtures.py` checks the fixtures themselves
against the specification. Counts are passing tests at `69a71e3`.

| Test | Requirement | Fixture cases | Passing tests (CLI / executor / world / fixture check) |
|---|---|---|---|
| T1 | Happy path ends `SUCCEEDED`; box at `dst`; gripper empty; five steps; message on `fleet` | `t01_pick_and_place` | 1 / 1 / 1 / 2 |
| T2 | Every invalid source: exit 1, no record, zero steps | `t02_invalid_sources` (all 61 files in `tests/fixtures/invalid/`) | 61 / — / — / 1 |
| T3 | `region_clear` false in `pre`: `FAILED` (`S101`) before any step | `t03_region_not_clear` | 1 / 1 / 1 / 1 |
| T4 | Undeclared condition: unknown fails closed (`S102`) | `t04_region_unknown`, `t04_robot_ready_unknown` | 2 / 2 / 2 / 2 |
| T5 | Missing capability or readiness: `S211`, `S201` | `t05_no_gripper`, `t05_robot_not_ready` | 2 / 2 / 2 / 2 |
| T6 | Object elsewhere: `S215` | `t06_object_elsewhere` | 1 / 1 / 1 / 1 |
| T7 | `place` without holding (`S221`) or away from target (`S222`) | `t07_place_not_held`, `t07_place_wrong_location` | 2 / 2 / 2 / 2 |
| T8 | Sensor undeclared, on another robot, unavailable, unknown (`S231`–`S233`); success records an observation | 5 `t08_*` cases | 5 / 5 / 5 / 5 |
| T9 | `wait until`: event before timeout, timeout (`S241`), event at the deadline, already true, event in `ms` | 5 `t09_*` cases | 5 / 5 / 5 / 5 |
| T10 | `communicate` is recorded; delivery is not claimed | `t01_pick_and_place` (message assertions) | 1 / 1 / 1 / 2 |
| T11 | Same inputs: byte-identical records and the expected run ID; a different seed: a different ID | `t11_seed_0`, `t11_seed_1` (each run twice) | 2 / 2 / 2 / 4 |
| T12 | Several tasks and no `--task`: exit 2; with `--task`: runs | `t12_no_task_selected`, `t12_task_selected` | 2 / 1 / 1 / 2 |
| T13 | Unbound, wrongly typed, undeclared or extra binding: exit 3 (`E505`–`E508`) before `RUNNING` | 4 `t13_*` cases | 4 / — / 4 / 4 |
| T14 | Unknown field or version: exit 3 (`E503`, `E502`) | `t14_unknown_field`, `t14_unsupported_version` | 2 / — / 2 / 2 |
| T15 | Injected fault at step 3: `FAILED` (`S251`), never `SUCCEEDED` | `t15_fault_step_3` | 1 / 1 / 1 / 1 |
| T16 | Cancellation before step 2: `CANCELLED`; step 2 and later not run | `t16_cancel_before_step_2` | 1 / 1 / 1 / 1 |
| T17 | `max_speed` above, missing from or within the robot profile (`S001`, `S002`, pass) | 3 `t17_*` cases | 3 / 3 / 3 / 3 |
| T18 | Isolation and honesty: no ROS 2 or adapter import; every record `"mode": "simulation"`; no limit reported as enforced | Every record from the cases above | Checked inside every `test_sim_cli.py::test_case` run; subprocess import checks in `test_sim_world.py`, `test_sim_executor.py`, `test_sim_cli.py` and `test_cli.py`: 4 passing |
| T19 | Regression: the 76 v1.1 fixtures and the existing suite pass | — | `test_fixtures.py` 77 passing; full suite 868 passed, 6 xfailed, 0 failed |

A dash means the layer does not apply. Invalid sources and invalid world files never reach the
executor.

Beyond the acceptance cases, the slices add unit tests for every world-file error code
(`E501`–`E511`), exact decimal time, the read-only world snapshot, the trust boundary of
`run()`, record key order, output bytes (one LF, no CR, ASCII), usage errors and run-ID
normalization across BOM and CRLF.

## 4. Demonstration

Run from a clone at `f5b3d23` (unchanged at `69a71e3`: docstring only) as `PYTHONPATH=src python -m robolang simulate …` on Windows,
observed on 2026-10-27. The installed `robolang simulate` command is checked by
`tools/smoke_install.py` in each CI job (section 2).

| Command | Exit | stderr summary |
|---|---|---|
| `robolang simulate examples/pick_and_place.rob --world tests/fixtures/simulation/worlds/base.json` | 0 | `run sim-74e8880d…: SUCCEEDED`; five steps succeeded. The run ID equals the T1 vector in `cases.json` |
| `… pick_and_place.rob --world …/region_blocked.json` | 4 | `FAILED (S101)`; no step run |
| `robolang simulate tests/fixtures/simulation/sources/wait_for_region.rob --world …/region_blocked.json` | 4 | `FAILED (S241)` at simulated time 32500 ms |
| `… pick_and_place.rob --world …/cancel_step_2.json` | 5 | `CANCELLED`; step 1 succeeded, the rest not run |
| `… pick_and_place.rob --world …/binding_wrong_type.json` | 3 | `error[E507] at /tasks/demo_pick_and_place/bindings/dst`; no record |
| `robolang simulate tests/fixtures/invalid/e203_unknown_statement.rob --world …/base.json` | 1 | `error[E203]` at 8:9; no record |

## 5. Review findings resolved before this evidence

| Found in | Finding | Resolution | Commit |
|---|---|---|---|
| BASIC-1 build | An event target was resolved as an entity before being checked as a sensor name | Each fact resolves its own namespace (spec 3.2) | `e4307bb` |
| BASIC-2 build | A `wait until` that timed out recorded no condition value | The last value is recorded (spec 4.5, 5.3) | `e5fd481` |
| BASIC-3 build | The source file was read twice, so the run ID could describe different text from what ran | The source is read once; the same bytes are compiled and hashed | `7fad80c` |
| Owner review of #13 | `World` held mutable dictionaries | Read-only mappings; test that changes fail | `c5a61e3` |
| Owner review of #14 | `run()` could fail with an uncaught lookup error on mismatched inputs | `SimulationInputError` before anything runs; six mismatch tests. `robolang simulate` always loads the world for the selected task | `2bacd7d` |
| Owner review of #14 | The trust boundary of `run()` should state that externally supplied IR is unsupported | One docstring sentence; CORE-1 is the planned IR reader | `800fa27` |

## 6. Known limitations and risk statement

- The simulation is **symbolic**: positions are named locations, and there is no motion
  planning, kinematics, collision, speed or physical time (spec 4.8). A `SUCCEEDED` run shows
  that the steps were consistent with a declared world, not that a robot could perform them.
- No speed limit is enforced. `max_speed` is only compared with the declared robot profile
  (B4).
- Capabilities are checked at the step that needs them, not in a preflight before the run
  starts. A comprehensive preflight is planned later (domain 11, RL-FOUNDATION-02).
- `run()` checks that its task and world belong together, but it cannot prove a `Task` came
  from the compiler; an in-process caller could build an internally inconsistent one. The CLI
  always compiles the source itself.
- Physical execution is not implemented, not authorized and not reachable: the simulation
  package imports no adapter and no ROS 2 module (T18). The v1 interpreter and ROS 2 adapter
  are unchanged and remain outside the installed package.
- This evidence is for **unmerged** branches. After the stack is merged in order (#9 to #15),
  CI on `main` must be checked once to confirm the merged result.

## 7. Owner sign-off

| Item | Decision | Date |
|---|---|---|
| RL-BASIC-01 exit gate met (sections 1–3) | **Signed off** by the owner, on the evidence at `f5b3d23` | 2026-10-27 |
| Foundation 1 exit gate met (F3) | **Signed off** by the owner, on the evidence at `f5b3d23` | 2026-10-27 |

The sign-off does not authorize merging any pull request, starting RL-ARCH-02, CORE-1,
any LANG item or any Foundation 2 item. Each still needs its own decision.

## 8. Post-merge addendum

**Date:** 2026-10-27. Added after integration. Sections 1–7 are historical: their "nothing
merged" wording and their commit and CI references describe the stack before it was merged.
The owner sign-off in section 7 stands as given, on the evidence at `f5b3d23`.

The owner authorized sequential squash-merging of the stack on 2026-10-27. Each pull request
was merged at a head whose CI passed 4/4, and its effective diff against `main` was
byte-identical to the approved diff.

| Pull request | Approved head | Squash commit on `main` |
|---|---|---|
| [#9](https://github.com/rokorobot/RoboLang/pull/9) Development plan aligned with RFC-001 | `fc59718` | `5308e32` |
| [#10](https://github.com/rokorobot/RoboLang/pull/10) RL-FOUNDATION-01 | `dcfd565` | `2ac0f67` |
| [#11](https://github.com/rokorobot/RoboLang/pull/11) RL-BASIC-01a | `c90b94a` | `64bdda9` |
| [#12](https://github.com/rokorobot/RoboLang/pull/12) RL-FOUNDATION-02 | `e69823a` | `65c1fac` |
| [#13](https://github.com/rokorobot/RoboLang/pull/13) BASIC-1 | `c5a61e3` | `ed91e10` |
| [#17](https://github.com/rokorobot/RoboLang/pull/17) BASIC-2, replacing [#14](https://github.com/rokorobot/RoboLang/pull/14) | `800fa27` (#14) | `ff71b3b` |
| [#15](https://github.com/rokorobot/RoboLang/pull/15) BASIC-3 | `69a71e3` | `fe71548` |
| [#16](https://github.com/rokorobot/RoboLang/pull/16) This evidence | `4936d10` | `efd5a73` |

- **#14:** an integration script error squash-merged #14 into its former base branch instead of
  `main`. Its approved changes reached `main` through #17, verified identical to #14's approved
  diff. A comment on #14 records this.
- **Conflicts:** two add/add conflicts in `src/robolang/sim/__init__.py`, during the #17 and #15
  integrations, were resolved by taking the approved head's file, as the owner authorized; the
  identical-diff check then passed.
- **The resulting `main`** equals the approved final head `4936d10` plus exactly #9's approved
  diff.

**Verification of `main` at `efd5a73`:**

| Check | Result |
|---|---|
| CI run [40431537343](https://github.com/rokorobot/RoboLang/actions/runs/40431537343) | 4/4 jobs passed (Ubuntu and Windows, Python 3.11 and 3.13) |
| Tests in every CI job | 868 passed, 6 expected failures |
| Lint | `ruff` passed |
| Installed command (`tools/smoke_install.py`) | The installed `robolang simulate` ran the example to `SUCCEEDED` in simulation mode |
| Run-ID vectors | The fixture vector tests pass; a run on `main` reproduced the T1 run ID `sim-74e8880d…` |
| Pages deployment [40431536651](https://github.com/rokorobot/RoboLang/actions/runs/40431536651) | Succeeded |

**After integration:** the F5 development-plan update ([#18](https://github.com/rokorobot/RoboLang/pull/18))
was merged as `776a0ad`, with CI run [40434729106](https://github.com/rokorobot/RoboLang/actions/runs/40434729106)
and Pages deployment [40434728854](https://github.com/rokorobot/RoboLang/actions/runs/40434728854)
both successful. Branch protection on `main` was enabled on 2026-10-27. Merging into `main`
now requires a pull request and the four CI jobs above; force-pushes and deletion are
disabled.
