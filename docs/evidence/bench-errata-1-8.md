# Benchmark errata for RoboLang 1.8 — 2026-11-17

Decision record [0040](../../spec/decisions/0040-declared-actions.md), iteration I2, step L3 (pre-register set 7
"actions"). 1.8 is **not built**: the reference and both prompts still teach RoboLang 1.7. This file lists every
benchmark input that changed, so that `docs/evidence/bench-freeze-v2.json` can be regenerated with a recorded reason
(the previous manifest is `4eeecb49…`, in git history; the earlier errata are [1.5](bench-errata-1-5.md),
[1.6](bench-errata-1-6.md) and [1.7](bench-errata-1-7.md)). Later entries (L4: labels, references, scorer, prompts) are
added here. No paid run was made: the baseline (1.7) is confirmed by the owner before it starts.

## Erratum E1 (2026-11-17, L3): set 7 "actions" added

Plan and proposed bounds: [bench-7-actions.md](../plans/bench-7-actions.md) (draft for the owner; the bounds are
placeholders to fix with the baseline, rule 6). Nothing standing was changed: sets 1 to 6, the prompts, the reference,
the grammar, the scorer and the worlds are as before.

| Input | Change |
|---|---|
| `bench/tasks_actions/` | **new**, 24 tasks: 10 `action`, 5 `infeasible`, 5 `inexpressible`, 4 `control`. `expect` is the 1.7 label, `expect_1_8` the 1.8 one (with the world 1.4 facts as a note) |
| `bench/reference_actions/` | **new**: a reference (`.rob`, `.json`) for each of the 4 controls; the obvious attempt (`.attempt.rob`, `.attempt.json`, fails with `E203`) for each of the other 20 |
| `bench/lib.py` | `ACTIONS_TASKS_DIR`, `ACTIONS_REFERENCE_DIR`, and `"actions"` in `SETS` |
| `tests/test_bench_actions.py` | **new** (58 checks); `tests/test_atlas.py` docstring |
| `bench/atlas.json` | the 20 tasks that 1.7 declines are in a gap: G-ACT 16 (`a01`–`a10`, `i01`–`i05`, `x02`), G-PROX 1 (`x01`), G-TEACH 1 (`x03`), G-LOOP 1 (`x04`), G-CLOCK 1 (`x05`); snapshot "I2 L3" in the history; G-TEACH's scope note names its first task |
| `docs/plans/humanoid-activities.md` | regenerated from the atlas |
| `.github/workflows/benchmark.yml` | input `actions: true` (full and minimal prompts, 3 repeats, Sonnet and Haiku, `--max-usd 3` each, not gated); excluded from the per-PR and weekly runs; `actions`, `actions-minimal` in the summary loop |

No world file was added and no world file has a 1.4 field. The 1.4 facts (`actions`, `state`) are in
`expect_1_8.world_1_4`, a note that no code reads: L4 writes the world copies. The one 1.3 world used is the existing
`bench/worlds/site_1_3.json` (`a07` also asks a region limit).

Not copied, as in set 6: the three R3 probes (`f_act_set_fryer`, `f_act_open_closet`, `f_act_print_layout`) and the standing
tasks `u15_open_oven`, `u16_charge_drone`, `x06_set_oven`. They stay in their sets, unchanged, until L4 relabels them.

## Freeze

`docs/evidence/bench-freeze-v2.json` is regenerated for all of the above.
