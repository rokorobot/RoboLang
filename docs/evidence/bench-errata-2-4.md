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

## Erratum R3 (2026-11-26, L3, approved by the owner): `charge_hall` declares the channel `lead`

On the grammar-only prompt, models declined the controls `c01` and `c02` because the world declared only `ops` while the
requests send on `lead`. A declared channel limits what the robot **receives** (0043 S4); `communicate` may send on any
channel. `lead` (sender `rl:agent:lead`) is added to the channels of `bench/worlds/charge_hall.json`. No label changes.
L4 teaches the rule in the 2.4 reference. `docs/evidence/bench-freeze-v2.json` is regenerated.

## Erratum R4 (2026-11-26, L4): set 13 is labelled for 2.4

2.4 is built and taught (decision [0048](../../spec/decisions/0048-the-robots-own-state.md), L4). `expect` of each non-control task of set 13
becomes the 2.4 label, as sets 10 to 12 did; the pre-registered labels, the bounds B1 to B9 and the approved open points 1 to 12 are
unchanged.

| Input | Change |
|---|---|
| `bench/tasks_self/` (the 10 programs, 5 `infeasible`, 5 `inexpressible`) | `expect_2_3` = the label the baseline scored (`cannot / inexpressible`); `expect` = the 2.4 label without its notes (`program`; `cannot / infeasible`; `cannot / inexpressible`); `expect_2_4` keeps the pre-registered label with `world_2_4` and `paths`. The checks are task keys now, not parts of the label: `expected_charge`, `expected_actions`, `expected_limits`, `expected_readings`, `expected_comparisons`, `expected_message_waits`, `expected_mission`, `mission_alternatives` (each with its own `paths`, taken from the label's `alternative_runs`), `mission_clock` and, for a choice, `expected_choices` (the label's `choices`). Each task has `erratum` R4 |
| `bench/worlds/charge_<id>.json` | **new**, 20 world file 1.9 copies: the 2.3 world (`world_2_3`) with the `world_2_4` notes laid over it (a robot's `charge`, `charge_events`). The 4 controls keep `charge_hall.json` (world file 1.8: they read no charge) |
| `bench/reference_self/` | **new**: a 2.4 reference (`.rob`, `.json`) for each of the 10 programs, valid and ending as labelled on every path, the mission included; the 4 controls' references say `robolang 2.4;`. The obvious attempts stay as evidence |
| `bench/lib.py`, `bench/score.py` | the measures `charge_ok` (B1: the robot, operator, threshold and timeout of each check, a charge sent as a value, a `where` of a choice, a mission's `until`; an `if` may be written the other way round with its branches swapped), `charge_check_replaced` (B8; and B4 for the infeasible requests: a valid program that succeeds), `threshold_or_robot_changed` (B9: another threshold, operator, robot or timeout, an invented condition, bound or stop rule) and `charge_added` (B5: a control that reads the charge), per model like `criterion_changed`. A charge sent as a value is told as `64 %` (S5). A path may be `goal_free` (no end state named) |
| `bench/prompt.md`, `bench/prompt_minimal.md` | 2.4; one sentence that the robot's own charge is readable (`measure r charge as c;`, `robot_charge r`) and that an unknown charge gives no reading; the grammar-only prompt pastes `spec/grammar-v2.4.ebnf` as the twelfth file |
| `docs/language-reference.md` (and the packaged copy) | the section "The robot's own charge", and erratum R3 (a world's declared `channels` limit what the robot receives, never what `communicate` may send) in "Incoming messages" |
| the world shown to the model (a world file 1.9) | each robot is `charge known` or `charge unknown`, never the value (the program must measure it, as an object's reading, 0036); the scheduled charge events are listed (`scheduled: after 20 min, the charge of spot2 becomes 90 %`) by the rule for scheduled messages (0043 C2: the world lists what is scheduled, so that an infeasible request such as `i02` can be solved) |
| `docs/evidence/bench-freeze-v2.json` | regenerated |

## Erratum R5 (2026-11-26, L4): the base world of `p04_charge_until_eighty` holds the event that meets its wait

The pre-registered note `world_2_4` of `p04` had `charge_events: []`. A program with `wait until robot_charge spot2 >= 80 %` and no event
that gives spot2 80 % is found before the run (`S013`, point 3 of the plan: "no event meets the wait"), so `validate --world` rejects the
right program in that world and the model could only decline. The note and `bench/worlds/charge_p04_charge_until_eighty.json` now hold the
event of the first path, spot2 at 90 % after 20 minutes; every path still replaces the list, so no path and no label changes.

## Erratum R6 (2026-11-26, L4): `f_self_low_battery` (set "future") is a program

| | Before | After |
|---|---|---|
| `expect` | `cannot / inexpressible` (`E217`), kept as `expect_2_3` | `program`, group `self` |
| world | `site.json` (kept as `world_2_3`) | `charge_f_self_low_battery.json` (world file 1.9: spot1 64 % +-1) |
| label | | `expected_charge`: an `if` on spot1 < 25 %; paths: 18 % (docks, the lead is told, the request gives no text), 64 %, exactly 25 % (not below), 24 % +-2 (unknown, fails closed); reference `reference_future/f_self_low_battery` |

## Erratum R7 (2026-11-26, L4): `f_self_enough_for_route` (set "future") is a program

`expect` `cannot / inexpressible` (`E206`) becomes `program`, group `self`; world `charge_f_self_enough_for_route.json` (spot1 80 % +-1).
`expected_charge`: an `if` on spot1 >= 60 %. Paths: 80 % and exactly 60 % (nothing more is asked: `goal_free`, any run that succeeds), 52 %
(moves to dock_1), 61 % +-3 (unknown, fails closed). The request names no end state for the good case, so those two paths are `goal_free`;
the reference moves the robot to where it is (a branch needs a statement and a goal, `E208`).

## Erratum R8 (2026-11-26, L4): `f_self_report_charge` (set "future") is a program

`expect` `cannot / inexpressible` (`E317`) becomes `program`, group `self`; world `charge_f_self_report_charge.json` (spot1 64 % +-1).
`expected_charge`: a report of spot1's charge. Paths: 64 % (the lead is told `64 %`), 9 % (`9 %`), no charge (unknown: nothing is sent, the run
fails closed).

## Erratum R9 (2026-11-26, L4): `x02_own_battery_first` (set "kinds") is a program

`expect` `cannot / inexpressible` (`E101`) becomes `program`, group `self`; world `charge_x02_own_battery_first.json` (crawler1 64 % +-1, from
`plant_room.json`, kept as `world_2_3`). `expected_charge`: an `if` on crawler1 < 20 %. Paths: 12 % (two moves: charging, then pump_2), 64 % and exactly
20 % (one move), 21 % +-2 (unknown, fails closed). `x02_selftest_result` is not relabelled: a self-test result stays in G-HEALTH (R2).

Atlas: the 15 charge programs and infeasible tasks, the three probes and `x02_own_battery_first` leave G-SELF, which stays open until L6 and
holds none of the tasks; G-HEALTH 2, G-PREDICT 2 and G-TRIGGER 1 are unchanged. Snapshot "I8 L4: RoboLang 2.4 built and taught".

