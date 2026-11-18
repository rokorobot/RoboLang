# Benchmark errata for RoboLang 2.1 — 2026-11-18

Decision record [0043](../../spec/decisions/0043-incoming-messages.md), iteration I5, step L3 (pre-register set 10
"messages"). At that step 2.1 was **not built** and the reference and both prompts still teach RoboLang 2.0. This file lists
every benchmark input that changed, so that `docs/evidence/bench-freeze-v2.json` can be regenerated with a recorded reason
(the previous manifest is in git history; the earlier errata are [1.5](bench-errata-1-5.md), [1.6](bench-errata-1-6.md),
[1.7](bench-errata-1-7.md), [1.8](bench-errata-1-8.md), [1.9](bench-errata-1-9.md) and [2.0](bench-errata-2-0.md)). Later entries
(L4: labels, references, scorer, prompts) are added here. No paid run was made: the baseline (2.0) is confirmed by the owner
before it starts.

## Erratum N1 (2026-11-18, L3): set 10 "messages" added

Plan and proposed bounds: [bench-10-messages.md](../plans/bench-10-messages.md) (draft for the owner; the bounds are
placeholders to fix with the baseline, rule 6; 14 open points where 0043 does not decide what a label needs). Nothing
standing was changed in its meaning: sets 1 to 9, the prompts, the reference, the grammar, the scorer's measures, the
simulator and every existing world are as before.

| Input | Change |
|---|---|
| `bench/tasks_messages/` | **new**, 24 tasks: 10 `message` programs, 5 `infeasible`, 5 `inexpressible`, 4 `control`. `expect` is the 2.0 label, `expect_2_1` the 2.1 one (a program has `expected_message_waits`, `world_2_1` with the declared `channels`, and `paths` forced by incoming `messages`) |
| `bench/reference_messages/` | **new**: a reference (`.rob`, `.json`) for each of the 4 controls; the obvious attempt (`.attempt.rob`, `.attempt.json`, fails with `E211`, or `E201` for `x01` and `x02`; the message part is marked `// @message` or `// @message-tail`) for each of the other 20 |
| `bench/lib.py` | `MESSAGES_TASKS_DIR`, `MESSAGES_REFERENCE_DIR`, and `"messages"` in `SETS`; `task_paths` also reads `expect_2_0.paths` of a task whose label has no `expected_mission` (the paths of the set's controls; set 9's missions keep their paths out of it). No judging rule changed |
| `tests/test_bench_messages.py` | **new** |
| `tests/test_bench_missions.py` | the check that the standing G-TRIGGER tasks are in that gap is now a subset check, because set 10's `x02_each_time_the_line_says_ready` is in it too |
| `bench/atlas.json` | the 20 tasks that 2.0 declines are in a gap: G-MSG 16 (the 10 programs, the 5 infeasible, `x05`), G-CALC 2 (`x03`, `x04`), G-TRIGGER 1 (`x02`), and the new far gap **G-DELIVERY** 1 (`x01`, S7); snapshot "I5 L3" in the history |
| `docs/plans/humanoid-activities.md` | regenerated from the atlas |
| `.github/workflows/benchmark.yml` | `messages` is a value of `set_only`; new step for it (full and minimal prompts, 3 repeats, Sonnet and Haiku, `--max-usd 3` each, not gated); excluded from the per-PR and weekly runs; `messages`, `messages-minimal` in the summary loop. The workflow still has 5 `workflow_dispatch` inputs; every earlier set's run command is unchanged |
| `bench/README.md`, `docs/LANGUAGE_ROADMAP.md` | a section for set 10; the I5 row |

Not copied, as in sets 6 to 9: the standing message tasks (`f_msg_line_docked`, `f_msg_nurse_confirm`, `f_msg_wind_clear`,
`x_wait_message`, `u11_wait_go`, `u12_radio_clearance`, `x_wait_ready_message`, `f12_message_delivery`) and set 9's
`x03_for_each_go_message`. They stay in their sets, unchanged, until L4 relabels them (plan, "What this means for the standing
tasks").

## Freeze

`docs/evidence/bench-freeze-v2.json` is regenerated for all of the above (the old manifest deleted, then
`python tools/bench_freeze.py`).
