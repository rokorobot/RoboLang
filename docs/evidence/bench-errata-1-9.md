# Benchmark errata for RoboLang 1.9 — 2026-11-17

Decision record [0042](../../spec/decisions/0042-clock-readings.md), iteration I3, step L3 (pre-register set 8
"clock"). 1.9 is **not built**: the reference and both prompts still teach RoboLang 1.8. This file lists every
benchmark input that changed, so that `docs/evidence/bench-freeze-v2.json` can be regenerated with a recorded reason
(the previous manifest is `6c3d9c1e…`, in git history; the earlier errata are [1.5](bench-errata-1-5.md),
[1.6](bench-errata-1-6.md), [1.7](bench-errata-1-7.md) and [1.8](bench-errata-1-8.md)). Later entries (L4: labels,
references, scorer, prompts) are added here. No paid run was made: the baseline (1.8) is confirmed by the owner before
it starts.

## Erratum E1 (2026-11-17, L3): set 8 "clock" added

Plan and proposed bounds: [bench-8-clock.md](../plans/bench-8-clock.md) (draft for the owner; the bounds are
placeholders to fix with the baseline, rule 6). Nothing standing was changed: sets 1 to 7, the prompts, the reference,
the grammar, the scorer and every existing world are as before.

| Input | Change |
|---|---|
| `bench/tasks_clock/` | **new**, 24 tasks: 10 `clock` programs, 5 `infeasible`, 5 `inexpressible`, 4 `control`. `expect` is the 1.8 label, `expect_1_9` the 1.9 one (with the world 1.5 facts as a note) |
| `bench/reference_clock/` | **new**: a reference (`.rob`, `.json`) for each of the 4 controls; the obvious attempt (`.attempt.rob`, `.attempt.json`, fails with `E101` or `E217`; the clock part is marked `// @clock`) for each of the other 20 |
| `bench/worlds/fry_station_at_fryer.json` | **new** (world 1.2): `fry_station.json` with flippy1 at the fryer, for `k09` |
| `bench/lib.py` | `CLOCK_TASKS_DIR`, `CLOCK_REFERENCE_DIR`, and `"clock"` in `SETS` |
| `tests/test_bench_clock.py` | **new** (61 checks) |
| `bench/atlas.json` | the 20 tasks that 1.8 declines are in a gap: G-CLOCK 17 (`k01`–`k10`, `i01`–`i05`, `x02`, `x05`), G-LOOP 1 (`x01`), G-CALC 2 (`x03`, `x04`); snapshot "I3 L3" in the history |
| `docs/plans/humanoid-activities.md` | regenerated from the atlas |
| `.github/workflows/benchmark.yml` | input `clock: true` (full and minimal prompts, 3 repeats, Sonnet and Haiku, `--max-usd 3` each, not gated); excluded from the per-PR and weekly runs; `clock`, `clock-minimal` in the summary loop. The workflow now has 10 `workflow_dispatch` inputs, GitHub's limit |
| `bench/README.md`, `docs/LANGUAGE_ROADMAP.md` | a section for set 8; the I3 row |

No world file has a 1.5 field. The 1.5 facts (`clock`) are in `expect_1_9.world_1_5`, a note that no code reads: L4 writes
the world copies.

Not copied, as in sets 6 and 7: the three R3 probes (`f_clock_scan_stamp`, `f_clock_shift_cutoff`,
`f_clock_clean_at_2300`) and the standing tasks `x_elapsed_time`, `x_report_duration`, `u17_at_1400`, `u18_every_morning`,
`x05_cooling_time` and `x05_open_at_six`. They stay in their sets, unchanged, until L4 relabels them.

## Freeze

`docs/evidence/bench-freeze-v2.json` is regenerated for all of the above.

## Erratum K (owner, 2026-11-17): gap membership after the clarifications of 0042

The owner approved K7 and K11, and with them four tasks change gap. Labels and scoring are unchanged.

| Task | From | To | Why |
|---|---|---|---|
| `x02_tomorrow_at_0100` | G-CLOCK | new **G-DATE** | another day (K11) |
| `x05_on_monday` | G-CLOCK | new **G-DATE** | a date (K11) |
| `x05_cooling_time` (set "values") | G-CLOCK | G-CALC | it stays inexpressible: the time between two moments (K7) |
| `u18_every_morning` (set "ask") | G-CLOCK | G-LOOP | a mission trigger (0041 M6), not a clock reading |

With these moves G-CLOCK holds only what 1.9 expresses, so it can close at L6.
`docs/evidence/bench-freeze-v2.json` is regenerated.
