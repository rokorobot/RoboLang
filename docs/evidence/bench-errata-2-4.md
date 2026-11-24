# Benchmark errata for RoboLang 2.4 — 2026-11-24

Decision record [0048](../../spec/decisions/0048-the-robots-own-state.md), iteration I8, step L3 (pre-register set 13 "self"). At that
step 2.4 was **not built** and the reference and both prompts still teach RoboLang 2.3. This file lists every benchmark input that changed,
so that `docs/evidence/bench-freeze-v2.json` can be regenerated with a recorded reason (the previous manifest is in git history; the earlier
errata are [1.5](bench-errata-1-5.md), [1.6](bench-errata-1-6.md), [1.7](bench-errata-1-7.md), [1.8](bench-errata-1-8.md),
[1.9](bench-errata-1-9.md), [2.0](bench-errata-2-0.md), [2.1](bench-errata-2-1.md), [2.2](bench-errata-2-2.md) and [2.3](bench-errata-2-3.md)).
No paid run was made.

## Erratum R1 (2026-11-24, L3): set 13 "self" added

Plan and proposed bounds: [bench-13-self.md](../plans/bench-13-self.md) (draft for the owner; the bounds are placeholders to fix with the
baseline, rule 6; 12 open points where 0048 does not decide what a label needs). Nothing standing was changed in its meaning by this erratum:
sets 1 to 12, the prompts, the reference, the grammar, the scorer's measures, the simulator and every existing world are as before (the one
task that changes gap is erratum R2).

| Input | Change |
|---|---|
| `bench/tasks_self/` | **new**, 24 tasks: 10 charge programs, 5 `infeasible`, 5 `inexpressible`, 4 `control`. `expect` is the 2.3 label, `expect_2_4` the 2.4 one (a program has `world_2_4`, `expected_charge`, `paths` forced by charges and scheduled events, and, where it combines a check with an older feature, `expected_limits`, `expected_actions`, `expected_readings`, `expected_message_waits`, `expected_mission`, `choices`) |
| `bench/reference_self/` | **new**: a reference (`.rob`, `.json`) for each of the 4 controls; the obvious attempt (`.attempt.rob`, `.attempt.json`; fails with `E217` or `E204`; the lines that read the robot's own state are marked `// @charge`) for each of the other 20 |
| `bench/worlds/charge_hall.json` | **new** world file (world file 1.8, no charge). The plan allows at most 3 |
| `bench/lib.py` | `SELF_TASKS_DIR`, `SELF_REFERENCE_DIR`, and `"self"` in `SETS`. No judging rule changed |
| `tests/test_bench_self.py` | **new** |
| `bench/atlas.json` | the 20 tasks that 2.3 declines are in a gap: G-SELF 15 (the 10 programs and the 5 infeasible), G-HEALTH 2, G-PREDICT 2, G-TRIGGER 1; the new far gap **G-HEALTH** (approved with 0048 S10); snapshot "I8 L3" in the history |
| `docs/plans/humanoid-activities.md` | regenerated from the atlas |
| `.github/workflows/benchmark.yml` | `self` is a value of `set_only`; new step for it (full and minimal prompts, 3 repeats, Sonnet and Haiku, `--max-usd 3` each, not gated); excluded from the per-PR and weekly runs; `self`, `self-minimal` in the summary loop. The workflow still has 5 `workflow_dispatch` inputs; every earlier set's run command is unchanged |
| `tests/test_bench_messages.py`, `tests/test_bench_kinds.py`, `tests/test_bench_choose.py` | the check of the `set_only` description reads "choose or self" |
| `bench/README.md`, `docs/LANGUAGE_ROADMAP.md` | a section for set 13; the I8 row |

Not copied, as in sets 6 to 12: the R3 probes `f_self_low_battery`, `f_self_enough_for_route` and `f_self_report_charge` and the standing task
`x02_own_battery_first`. They stay in sets "future" and "kinds" and in G-SELF, unchanged, until L4 relabels them (plan, "What this means for the
standing tasks").

## Erratum R2 (2026-11-24, L3): one standing task moves from G-SELF to G-HEALTH

0048 S10 keeps out of 2.4 the robot's health beyond `robot_ready` and `sensor_ok`: faults, self-test results and requests for service. The owner
approved the new far gap **G-HEALTH** (autonomy, open, far: no sub-activity claims it yet), so that G-SELF can close at L6.

| Task (set) | Before | After |
|---|---|---|
| `x02_selftest_result` (actions) | G-SELF (atlas only) | **G-HEALTH**: a self-test returns a value, which is health and not the battery; the task gets `gap` G-HEALTH, `gap_before` G-SELF and `erratum` R2 |

Its label, id and prompt are unchanged (`cannot / inexpressible`). Set 13 holds two more tasks of G-HEALTH
(`x01_run_your_self_test_and_report`, `x02_report_your_faults`), two of G-PREDICT (`x03_minutes_until_empty`,
`x04_if_you_will_not_make_it_back`) and one of G-TRIGGER (`x05_every_time_it_drops_below_twenty`); the 15 charge programs and infeasible tasks
are in G-SELF with the four that were there. G-SELF stays open until L6.
