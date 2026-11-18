# Benchmark errata for RoboLang 2.3 — 2026-11-18

Decision record [0045](../../spec/decisions/0045-choosing-at-run-time.md), iteration I7, step L3 (pre-register set 12 "choose"). At
that step 2.3 was **not built** and the reference and both prompts still teach RoboLang 2.2. This file lists every benchmark input that
changed, so that `docs/evidence/bench-freeze-v2.json` can be regenerated with a recorded reason (the previous manifest is in git history;
the earlier errata are [1.5](bench-errata-1-5.md), [1.6](bench-errata-1-6.md), [1.7](bench-errata-1-7.md), [1.8](bench-errata-1-8.md),
[1.9](bench-errata-1-9.md), [2.0](bench-errata-2-0.md), [2.1](bench-errata-2-1.md) and [2.2](bench-errata-2-2.md)). No paid run was
made: the baseline run on set 12 is next.

## Erratum Q1 (2026-11-18, L3): set 12 "choose" added

Plan and proposed bounds: [bench-12-choose.md](../plans/bench-12-choose.md) (draft for the owner; the bounds are placeholders to fix with
the baseline, rule 6; 11 open points where 0045 does not decide what a label needs). Nothing standing was changed in its meaning by this
erratum: sets 1 to 11, the prompts, the reference, the grammar, the scorer's measures, the simulator and every existing world are as
before (the two tasks that change gap are erratum Q2).

| Input | Change |
|---|---|
| `bench/tasks_choose/` | **new**, 25 tasks: 11 `choose` programs, 5 `infeasible`, 5 `inexpressible`, 4 `control`. `expect` is the 2.2 label, `expect_2_3` the 2.3 one (a program has `world_2_3`, `choices`, `paths` forced by positions, readiness and places, and, where it combines a choice with an older feature, `expected_limits`, `expected_clock`, `expected_mission`) |
| `bench/reference_choose/` | **new**: a reference (`.rob`, `.json`) for each of the 4 controls; the obvious attempt (`.attempt.rob`, `.attempt.json`; fails with `E203`; the `choose` line is marked `// @choice NAME=ENTITY`) for each of the other 21 |
| `bench/worlds/bin_hall.json`, `bench/worlds/parts_line.json` | **new** world files (world file 1.7, no `position`). The plan allows at most 3 |
| `bench/lib.py` | `CHOOSE_TASKS_DIR`, `CHOOSE_REFERENCE_DIR`, and `"choose"` in `SETS`. No judging rule changed |
| `tests/test_bench_choose.py` | **new** |
| `bench/atlas.json` | the 21 tasks that 2.2 declines are in a gap: G-CHOOSE 16 (the 11 programs and the 5 infeasible), G-EACH 3, G-PROX 1, G-DETECT 1; the new far gaps **G-EACH** and **G-DETECT** (approved with 0045); snapshot "I7 L3" in the history |
| `docs/plans/humanoid-activities.md` | regenerated from the atlas |
| `.github/workflows/benchmark.yml` | `choose` is a value of `set_only`; new step for it (full and minimal prompts, 3 repeats, Sonnet and Haiku, `--max-usd 3` each, not gated); excluded from the per-PR and weekly runs; `choose`, `choose-minimal` in the summary loop. The workflow still has 5 `workflow_dispatch` inputs; every earlier set's run command is unchanged |
| `tests/test_bench_messages.py`, `tests/test_bench_kinds.py` | the check of the `set_only` description reads "kinds or choose"; the kinds test counts `x03_carry_as_many_as_counted` in G-EACH (erratum Q2) |
| `bench/README.md`, `docs/LANGUAGE_ROADMAP.md` | a section for set 12; the I7 row |

Not copied, as in sets 6 to 11: the R3 probes `f_choose_nearest_tugger`, `f_choose_nearest_dock` and `f_choose_nearer_cart` and the standing
tasks `x_nearest_robot`, `x05_every_new_part` and `x_shuttle_until_five`. They stay in sets "future", "branch" and "missions" and in
G-CHOOSE, unchanged, until L4 relabels them (plan, "What this means for the standing tasks").

## Erratum Q2 (2026-11-18, L3): two standing tasks move from G-CHOOSE to G-EACH

0045 C9 keeps out of 2.3 acting on each member of a run-time set by a reading or by a count, and the owner approved the new far gaps
**G-EACH** (acting on each member of a run-time set) and **G-DETECT** (objects the world does not list). Two standing tasks fall under the
first, so that G-CHOOSE can close at L6.

| Task (set) | Before | After |
|---|---|---|
| `x03_any_hot_pan` (values) | G-CHOOSE (atlas only) | **G-EACH**: a reading of each pan and a move of every one above the limit; the task gets `gap` G-EACH, `gap_before` G-CHOOSE and `erratum` Q2 |
| `x03_carry_as_many_as_counted` (kinds) | G-CHOOSE | **G-EACH**: as many carries as a count; `gap_before` G-CHOOSE keeps the old gap as history |

Their labels, ids and prompts are unchanged (`cannot / inexpressible`). Set 12 holds three more tasks of G-EACH (`x01`, `x02`, `x03`), one
of G-DETECT (`x05_an_object_the_list_does_not_know`) and one of G-PROX (`x04_the_robot_nearest_the_caller`). `x05_every_new_part` and
`x_shuttle_until_five` stay in G-CHOOSE: under C10 they are programs (missions that choose afresh). The two new gaps are far and open and no
sub-activity claims them yet (`needs` unchanged).

## Freeze

`docs/evidence/bench-freeze-v2.json` is regenerated for all of the above (the old manifest deleted, then
`python tools/bench_freeze.py`).
