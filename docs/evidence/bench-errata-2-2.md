# Benchmark errata for RoboLang 2.2 — 2026-11-18

Decision record [0044](../../spec/decisions/0044-more-measured-kinds.md), iteration I6, step L3 (pre-register set 11
"kinds"). At that step 2.2 was **not built** and the reference and both prompts still teach RoboLang 2.1. This file lists
every benchmark input that changed, so that `docs/evidence/bench-freeze-v2.json` can be regenerated with a recorded reason
(the previous manifest is in git history; the earlier errata are [1.5](bench-errata-1-5.md), [1.6](bench-errata-1-6.md),
[1.7](bench-errata-1-7.md), [1.8](bench-errata-1-8.md), [1.9](bench-errata-1-9.md), [2.0](bench-errata-2-0.md) and
[2.1](bench-errata-2-1.md)). Later entries (L4: labels, references, scorer, prompts) are added here. No paid run was made:
the baseline (2.1) is confirmed by the owner before it starts.

## Erratum P1 (2026-11-18, L3): set 11 "kinds" added

Plan and proposed bounds: [bench-11-kinds.md](../plans/bench-11-kinds.md) (draft for the owner; the bounds are placeholders
to fix with the baseline, rule 6; 18 open points where 0044 does not decide what a label needs). Nothing standing was changed
in its meaning by this erratum: sets 1 to 10, the prompts, the reference, the grammar, the scorer's measures, the simulator
and every existing world are as before (the two relabelled tasks are erratum P2).

| Input | Change |
|---|---|
| `bench/tasks_kinds/` | **new**, 24 tasks: 10 `kind` programs, 5 `infeasible`, 5 `inexpressible`, 4 `control`. `expect` is the 2.1 label, `expect_2_2` the 2.2 one (a program has `world_2_2`, `paths` forced by 1.7 property values, `expected_readings`, `expected_comparisons` and, where it combines a reading with an older feature, `expected_limits`, `expected_actions`, `expected_clock`, `expected_message_waits`, `expected_sensors`) |
| `bench/reference_kinds/` | **new**: a reference (`.rob`, `.json`) for each of the 4 controls; the obvious attempt (`.attempt.rob`, `.attempt.json`; fails with `E101`, `E217` or `E505`; the reading is marked `// @reading`) for each of the other 20 |
| `bench/worlds/stockroom.json`, `bench/worlds/plant_room.json` | **new** world files (world file 1.6; their sensors measure only the 1.6 kinds; the 1.7 kinds are in each task's `world_2_2`). The plan allows at most 3 |
| `bench/lib.py` | `KINDS_TASKS_DIR`, `KINDS_REFERENCE_DIR`, and `"kinds"` in `SETS`. No judging rule changed: `task_paths` already reads the `expect_2_1.paths` of the controls, and the 2.2 paths are inside `expect_2_2` |
| `tests/test_bench_kinds.py` | **new** |
| `bench/atlas.json` | the 20 tasks that 2.1 declines are in a gap: G-PERCEIVE 16 (the 10 programs, the 5 infeasible, `x01`), G-SELF 1 (`x02`), G-CHOOSE 1 (`x03`), G-CALC 1 (`x04`), G-PROX 1 (`x05`); `u01_if_dirty` and `u02_if_damaged` leave G-PERCEIVE (erratum P2); snapshot "I6 L3" in the history |
| `docs/plans/humanoid-activities.md` | regenerated from the atlas |
| `.github/workflows/benchmark.yml` | `kinds` is a value of `set_only`; new step for it (full and minimal prompts, 3 repeats, Sonnet and Haiku, `--max-usd 3` each, not gated); excluded from the per-PR and weekly runs; `kinds`, `kinds-minimal` in the summary loop. The workflow still has 5 `workflow_dispatch` inputs; every earlier set's run command is unchanged |
| `tests/test_bench_messages.py`, `tests/test_bench_ask.py` | the check of the `set_only` description reads "messages or kinds"; the "ask" set has 22 `infeasible` and 6 `inexpressible` tasks (erratum P2) |
| `bench/README.md`, `docs/LANGUAGE_ROADMAP.md` | a section for set 11; the I6 row |

Not copied, as in sets 6 to 10: the R3 probes `f_perceive_shelf_fill` and `f_perceive_weed_confidence`. They stay in set
"future" and in G-PERCEIVE, unchanged, until L4 relabels them (plan, "What this means for the standing tasks" and open point 1:
the shelf of the first is a location, not an object).

## Erratum P2 (2026-11-18, L3): `u01_if_dirty` and `u02_if_damaged` relabelled (0044 K6)

Both tasks (set "ask") still carried their RoboLang 1.3 reason: "no conditional statement, and `inspect` yields no condition".
Since 1.4 there is an `if` and since 1.6 a `label` reading can express "dirty" and "damaged". In their worlds no sensor
measures a `label` (`cam_chef` in `kitchen`, `cam_c2` in `yard`: world file 1.2 `measures` absent), so a faithful attempt
fails with `S234` and the request is `infeasible`, not `inexpressible`. 0044 K6 decided to relabel them in L3 by erratum.

| Task | Before | After |
|---|---|---|
| `u01_if_dirty` | group `inexpressible`; `expect` cannot / inexpressible; in G-PERCEIVE | group `infeasible`; `expect` **cannot / infeasible**; the old label kept as `expect_1_3` and the old reason as `why_1_3`; `blockers` `cam_chef`; `attempt_fails_with` `S234`; `fix_patch` (`cam_chef` measures `label`; `pan_1` is labelled `dirty`); no longer in G-PERCEIVE |
| `u02_if_damaged` | the same | the same, with `cam_c2` and `cont_102` labelled `damaged`; its `fix_patch` also sets `world_schema_version` 1.2, because `yard.json` is a 1.1 file and `measures` is a 1.2 field |

New files: `bench/reference_ask/u01_if_dirty.attempt.{rob,json}` and `bench/reference_ask/u02_if_damaged.attempt.{rob,json}` (a
RoboLang 2.1 program that reads the label and branches; it fails with `S234` in the world as it is and succeeds with the
`fix_patch`, as the "ask" test requires of every infeasible task). `tests/test_bench_ask.py` counts 22 `infeasible` and 6
`inexpressible` tasks (it was 20 and 8). The tasks' prompts, worlds and ids are unchanged; no model answer is affected (a
2.1 model that declined them as `inexpressible` now has a wrong category and a right decision, as for any infeasible task).

## Freeze

`docs/evidence/bench-freeze-v2.json` is regenerated for all of the above (the old manifest deleted, then
`python tools/bench_freeze.py`).
