# Benchmark errata for RoboLang 2.0 — 2026-11-18

Decision record [0041](../../spec/decisions/0041-mission-layer.md), iteration I4, step L3 (pre-register set 9
"missions"). 2.0 is **not built**: the reference and both prompts still teach RoboLang 1.9. This file lists every
benchmark input that changed, so that `docs/evidence/bench-freeze-v2.json` can be regenerated with a recorded reason
(the previous manifest is `8b75d6bc…`, in git history; the earlier errata are [1.5](bench-errata-1-5.md),
[1.6](bench-errata-1-6.md), [1.7](bench-errata-1-7.md), [1.8](bench-errata-1-8.md) and [1.9](bench-errata-1-9.md)).
Later entries (L4: labels, references, scorer, prompts) are added here. No paid run was made: the baseline (1.9) is
confirmed by the owner before it starts.

## Erratum M1 (2026-11-18, L3): set 9 "missions" added

Plan and proposed bounds: [bench-9-missions.md](../plans/bench-9-missions.md) (draft for the owner; the bounds are
placeholders to fix with the baseline, rule 6; 14 open points where 0041 does not decide what a label needs). Nothing
standing was changed: sets 1 to 8, the prompts, the reference, the grammar, the scorer and every existing world are as
before. (0041 calls this set 8; the clock set took that number, so it is set 9.)

| Input | Change |
|---|---|
| `bench/tasks_missions/` | **new**, 26 tasks: 10 `mission` programs, 5 `infeasible`, 7 `inexpressible`, 4 `control`. `expect` is the 1.9 label, `expect_2_0` the 2.0 one (a program has an `expected_mission` and the mission outcome of each path) |
| `bench/reference_missions/` | **new**: a reference (`.rob`, `.json`) for each of the 4 controls; the obvious attempt (`.attempt.rob`, `.attempt.json`, fails with `E201`; the mission part is marked `// @mission`) for each of the other 22 |
| `bench/lib.py` | `MISSIONS_TASKS_DIR`, `MISSIONS_REFERENCE_DIR`, and `"missions"` in `SETS` |
| `tests/test_bench_missions.py` | **new** |
| `bench/atlas.json` | the 22 tasks that 1.9 declines are in a gap: G-LOOP 20 (the 10 programs, the 5 infeasible, `x01`, `x02`, `x04`, `x06`, `x07`), G-MSG 1 (`x03`), G-CHOOSE 1 (`x05`); snapshot "I4 L3" in the history |
| `docs/plans/humanoid-activities.md` | regenerated from the atlas |
| `.github/workflows/benchmark.yml` | the per-set booleans `set3`, `set4`, `future`, `regions`, `actions`, `clock` (10 inputs, GitHub's limit) are replaced by one string input `set_only` (`set3`, `set4`, `future`, `regions`, `actions`, `clock`, `missions`; empty is the usual run). Every earlier set's run command is unchanged; the weekly step runs only when `set_only` is empty. New step for `missions` (full and minimal prompts, 3 repeats, Sonnet and Haiku, `--max-usd 3` each, not gated); excluded from the per-PR and weekly runs; `missions`, `missions-minimal` in the summary loop. The workflow now has 5 `workflow_dispatch` inputs |
| `tests/test_bench_actions.py`, `test_bench_regions.py`, `test_bench_clock.py`, `test_bench_future.py` | read `inputs.set_only == '<set>'` instead of `inputs.<set>`; the declared-input check is for `set_only` |
| `bench/README.md`, `docs/LANGUAGE_ROADMAP.md` | a section for set 9; the earlier `<set>: true` dispatches read `set_only: <set>`; the I4 row |

The evidence files of earlier runs name the old inputs (`regions: true`, `actions: true`, `clock: true`): they record
what was dispatched at the time and are not changed.

Not copied, as in sets 6 to 8: the 15 standing G-LOOP tasks (`x_every_hour`, `x_keep_feeding`, `x_retry_grasp`,
`x_wait_forever`, `u03_until_stop`, `u04_back_and_forth`, `x04_every_minute`, `f10_retry_until_works`,
`f13_retry_every_minute`, `x_all_parts_ever`, `x_retry_until_grasped`, `x_shuttle_until_five`, `u18_every_morning`,
`x04_open_until_open`, `x01_every_night_round`). They stay in their sets, unchanged, until L4 relabels them. None states
a bound, so under 0041 M2 none becomes a program (plan, "What this means for the standing tasks").

## Freeze

`docs/evidence/bench-freeze-v2.json` is regenerated for all of the above.

## Erratum W1 (2026-11-18): the paid answers are always kept

The first baseline run of set 9 ([run 44252920152](https://github.com/rokorobot/RoboLang/actions/runs/44252920152))
answered every request, but GitHub cancelled the job while it was scoring. Because of that the artifact upload was
skipped and the answers were lost; no log survived. The upload now runs `if: always()`, and scoring has a 25-minute
timeout and prints each directory as it starts. The baseline is run again.
The second run ([run 44261560831](https://github.com/rokorobot/RoboLang/actions/runs/44261560831)) hung in the same
way. Scoring ran past its own 25-minute timeout, which suggests the runner itself had died, likely out of memory, and
nothing was uploaded. The orchestrator cancelled it. The raw answers are now uploaded as `benchmark-answers`
**before** scoring, so a failure while scoring can be reproduced offline.
