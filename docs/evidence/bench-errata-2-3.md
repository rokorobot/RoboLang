# Benchmark errata for RoboLang 2.3 — 2026-11-18

Decision record [0045](../../spec/decisions/0045-choosing-at-run-time.md), iteration I7, step L3 (pre-register set 12 "choose"). At
that step 2.3 was **not built** and the reference and both prompts still teach RoboLang 2.2. This file lists every benchmark input that
changed, so that `docs/evidence/bench-freeze-v2.json` can be regenerated with a recorded reason (the previous manifest is in git history;
the earlier errata are [1.5](bench-errata-1-5.md), [1.6](bench-errata-1-6.md), [1.7](bench-errata-1-7.md), [1.8](bench-errata-1-8.md),
[1.9](bench-errata-1-9.md), [2.0](bench-errata-2-0.md), [2.1](bench-errata-2-1.md) and [2.2](bench-errata-2-2.md)). No paid run was
made, at L3 or at L4. The L4 entries (RoboLang 2.3 built and taught, the labels, the references, the scorer and the prompts: erratum Q5 to
Q7) are below Q4; the acceptance run on set 12 (L5) is next.

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

## Erratum Q3 (2026-11-19, L3, approved by the owner): infeasible tasks whose 2.2 world says otherwise

In `i03_the_ready_robot_has_no_gripper`, `i04_nothing_at_the_rework_bench` and `i05_the_ready_scanbot_has_the_wrong_camera`,
the facts that make the task infeasible are only in the 2.3 notes (`world_2_3`). The 2.2 world files shown in the baseline
say the opposite:

- `digit2` is ready;
- `p5` is at `rework`;
- `scanbot2` is ready.

In the baseline ([bench-set12-baseline.md](bench-set12-baseline.md)), a program that is right for the world shown is
therefore not a false program. No label changes for 2.3, because the world 1.8 copies of L4 carry the facts.

## Erratum Q4 (2026-11-19, L3, approved by the owner): the if-ladder over listed objects is a right answer

The simulated world is closed: it lists every object. An `if object_at p1 L … else if object_at p2 L …` ladder over the
listed objects therefore does what `choose object p from objects at L` does. For `p05_any_part_from_the_conveyor`, and
for the runs of `p09` and `p11`, the ladder counts as a right answer in 2.2 and in 2.3. B8 counts a ladder only when it
drops a criterion that the request states, such as "nearer" in `p04`. L4 applies this to the labels and the scorer.

## Erratum Q5 (2026-11-19, L4): six standing tasks relabelled (world file 1.8 facts)

Each task is a program under 2.3 (0045 C1 to C5, C10): group `choose`, `expect` program, the old label kept (`expect_2_2`, and the earlier
`expect_*` as they were), a world file 1.8 copy `worlds/choose_<id>.json` with the positions it needs (the 2.2 file is `world_2_2`), a
reference in the set's reference directory (the old attempt stays), `expected_choices`, an `expect_2_3` label with paths, the tag list of a
choice, `erratum` "Q5". They leave the atlas (G-CHOOSE).

| Task (set) | Before | World file 1.8 facts | Choice and paths |
|---|---|---|---|
| `f_choose_nearest_tugger` (future; ACT-MFG-002) | `cannot / inexpressible` (E304), G-CHOOSE | `fulfilment` copy: tugger_bay (0, 0), line_feed (40, 0), handoff_point (38, 6) | `robot` from (tugger_a, tugger_b) nearest to handoff_point. (1) tugger_b at line_feed: **tugger_b**; (2) the two have swapped places: **tugger_a** |
| `f_choose_nearest_dock` (future; ACT-AUTO-001) | the same | `site` copy: stop_a (44, 20), stop_b (60, 10), dock_1 (50, 30), dock_2 (42, 18), dock_3 (55, 12) | `location` from (dock_1, dock_2, dock_3) nearest to spot1, **the robot itself** (a robot is at its location). (1) spot1 at stop_a: **dock_2**; (2) at stop_b: **dock_3**; (3) dock_2 has no position, so is left out: **dock_1** |
| `f_choose_nearer_cart` (future; ACT-LOG-001) | the same | `fulfilment` copy: inbound_conveyor (10, 20), cart_3 (12, 22), conveyor_b (30, 25) | `location` from (cart_3, conveyor_b) nearest to inbound_conveyor, then pick and place there. (1) **cart_3**; (2) conveyor_b moved next to the conveyor: **conveyor_b** |
| `x_nearest_robot` (branch) | `cannot / inexpressible` (E203), G-CHOOSE | `warehouse` copy: shelf_a (10, 10), shelf_b (20, 10), table (60, 10), dock (40, 10), charging (60, 40) | `robot` from `robots` nearest to shelf_b, bring box2 to the table. (1) arm1 at shelf_a: **arm1**; (2) arm2 at shelf_b: **arm2**. The goal does not name the robot, so each path also states where the chosen robot ends (`final`) |
| `x05_every_new_part` (missions) | `cannot / inexpressible` (E201), G-CHOOSE | `assembly` copy: positions for its locations; p1, p2, p3 at conveyor_in | a mission for at most 8 hours (`480 min`; the label writes `h`) that repeats a task choosing `object` from `objects at conveyor_in` (C10). Paths: three parts: runs **p1, p2, p3**, a fourth finds none and the mission fails (4 runs, 3 verified); a fourth part has arrived: 5 runs, 4 verified; an empty conveyor: 1 run, 0 verified |
| `x_shuttle_until_five` (branch) | `cannot / inexpressible` (E203), G-CHOOSE | the same, with a clock at 16:30 | the same task, `until clock_time >= 17:00` as the only bound |

**A reading the labels take (to confirm at L5).** The world is closed and neither request states a wait, so no run takes simulated time:
the clock never reaches the bound, and the run that finds the conveyor empty fails (`else`) and ends the mission as `failed`
(`on failure { stop; }`; the request states no policy, so a `continue` is also accepted, and under a time bound it stops with `no_progress`,
0045 open point 3). The paths of `p09` and `p11`, which state the waits and the stop, are the ones that end `bounded` or `completed`.
A model that adds waits the request does not state changes the request; B9 counts a bound the request did not state.

## Erratum Q6 (2026-11-19, L4): set 12 labelled for 2.3

| Input | Change |
|---|---|
| every task of `bench/tasks_choose/` that is not a control | `expect` is the **2.3** label (`program`, `cannot / infeasible` or `cannot / inexpressible`); the baseline's 2.2 label is `expect_2_2`; `expect_2_3` keeps the pre-registered label (`world_2_3` notes, `paths`, `because`). The checks are task keys, as in set 11: `expected_choices` (was `choices`), `expected_limits` (`p10`) and `expected_mission` (`p09`, `p11`); the clock label of `p11` stays in `expect_2_3` (the mission label judges its `until`) |
| `bench/worlds/choose_<id>.json` | **16 new world files 1.8**, one for each of the 11 programs and 5 infeasible tasks: the 2.2 world plus the notes of `world_2_3` (`position` of each location it names, the readiness that differs, the object that moved). The task key `world_2_2` names the file the baseline showed. This is what makes `i01`, `i03`, `i04` and `i05` infeasible (erratum Q3): in the 2.2 world each of them is a request that looks fine. The plan allowed at most 3 new worlds *for the 2.2 baseline* (`bin_hall`, `parts_line`); a copy is not a new fact, as in sets 6 and 11 |
| `p05`, `p09`, `p11` (erratum Q4) | `ladder_ok` on the choice: an `if` ladder over the listed objects at the conveyor is also right (and, for the two missions, `extra_entities_ok`: the parts of a ladder are entities the request does not name). B8 counts such a ladder only if it drops a criterion the request states, and these requests state none |
| `world_patch` | a path may give `positions: {location: [x, y] or null}`, which sets or removes a position of the 1.8 copy (the scorer applies it, `lib.patched_world`) |
| `bench/reference_choose/<id>.rob`, `.json` | **11 new 2.3 references**, one for each program, valid and ending as labelled on every path (missions included). `p03` and `p08` write their `choose` first, as the reference teaches (open point 8: a choice using only world facts may come first, so the task can be delegated). The `.attempt` files stay as evidence |
| the declined tasks (`x01` to `x05`, `i01` to `i05`) | `x01` to `x05` keep their reasons (0045 C9) and their gaps (G-EACH, G-PROX, G-DETECT); `i01` to `i05` are `cannot / infeasible` in 2.3 and leave the atlas |

## Erratum Q7 (2026-11-19, L4): scorer, prompts, grammar and the world shown to the model

| Input | Change |
|---|---|
| `bench/lib.py`, `bench/score.py` | **`choice_ok`** (B1): the answer chooses exactly what the request asks (type, source, `where`, order, reference point), a list in any order unless the request settles a tie by the order written (`p07`), with Q4's ladder where `ladder_ok`; it is a check (`choices`), so `request_satisfied` needs it. **`choice_replaced`** (B8; B4 for the infeasible tasks): valid, ends well, and a candidate fixed (no `choose`, no ladder, or a list cut down) or an asked order or `where` dropped; for an infeasible task, any valid program that ends well. **`criterion_changed`** (B9): the other order, another reference point, a `where` dropped, invented or changed, a tie settled the other way, or a mission with a bound or stop rule the request did not state. **`choose_added`** (B5): a `choose` on a control. Per model, like `message_wait_replaced`. The goal of a path is checked with the entity each choice chose (`lib.chosen_bindings`), so a goal that names the choice is the entity chosen, never "some entity"; a mission run whose label names no goal is right when it ends SUCCEEDED |
| `bench/prompt.md`, `bench/prompt_minimal.md` | RoboLang 2.3; the grammar-only prompt pastes `grammar-v2.3.ebnf` as well (eleven files). **One sentence is added to both**: after "any that arrive ... cannot be represented", the exception "`choose object p from objects at L` takes whichever listed object is at `L` when the choice is made", because the general rule would otherwise make a model decline the missions of `p09`, `p11` and `x05` |
| `docs/language-reference.md` and the packaged copy | 2.3; a section "Choosing at run time"; the old "choosing an object at run time" lines of "Measured values" and "Missions" now say what 2.3 does and does not do |
| `spec/grammar-v2.3.ebnf` | says what 2.3 cannot say (C9) and that `choose` never reads a value (the grammar-only prompt is this file; erratum E5 of [1.8](bench-errata-1-8.md)) |
| the world shown to the model | `describe_world` lists each location's position in a world file 1.8: `dock_1 (50, 30)`, or `(no position)` |
| tests | `tests/test_bench_choose.py` (labels, references, the new measures), `test_bench_future.py`, `test_bench_branch.py`, `test_bench_missions.py` and `test_reference.py` follow the relabels; the set-9 and set-3 tests treat `x05_every_new_part`, `x_nearest_robot` and `x_shuttle_until_five` as programs |

## Erratum Q8 (2026-11-24, L4): merged with R3b

`main` gained the R3b probes in set "future": four inexpressible probes for G-TRIGGER and G-DATE (PR #186, #187). Merging
them with the 2.3 relabels of Q5 changed no label. The test of set "future" counts both: 37 tasks, of which 12 are
inexpressible and 3 are `choose`. The safety case hazard of 0045 C11 is renumbered from H25 to **H27**, because H25 and H26
are now the humanoid hazards of 0046.

## Teaching fix Z1 (2026-11-24, L5, approved by the owner)

The acceptance run showed Haiku adding `where robot_ready` to "whichever is nearer" (`p01`, `p06`). The only `choose`
example in the reference had that `where`, so the models were copying our teaching. Changes:

- The example in `docs/language-reference.md` (top and "Choosing at run time"), in its packaged copy and in
  `spec/grammar-v2.3.ebnf` has no `where`.
- The reference says to write `where` only for a condition the request states.
- "Missions" says that a time of day in `until` is the bound by itself (0041 clarification 2), for `p11`.

## Erratum Q9 (2026-11-24, L5, approved by the owner): `c03` accepts a round trip run 6 times

"12 carries in all" may be one task that carries p1 there and back, run 6 times. The label lists that shape in
`expect_2_0.mission_alternatives`, and the scorer (`bench/lib.py` `judge_mission`) accepts the first listed shape that is
right. Rescored offline, Haiku's repeat 1 of the full prompt is now right. Repeat 3 is still wrong (24 carries).

## Erratum Q10 (2026-11-24, L5, approved by the owner): `x02` gets a second pan at the stove

`bench/worlds/choose_x02_every_pan_above_75.json` is `kitchen_values` with `pan_2` at the stove. With only `pan_1` there,
one choice and one reading did "each pan" for that world, so the task did not test "each".

## Erratum Q11 (2026-11-24, L5, approved by the owner): `x02` accepts a program that pins the listed pans

In a closed world, a program that pins the listed pans in `pre` and reads and moves each one does what "each pan on the
stove" means in that world, and fails closed otherwise. So `x02` accepts either a decline or such a program. The program
is described in `expect_2_3.program_also_ok`: the pinned `[object, location]` pairs and three paths. `bench/score.py`
scores it with `lib.pre_pins`. A program without the `pre` stays a false program.

## Freeze

`docs/evidence/bench-freeze-v2.json` is regenerated for all of the above (the old manifest deleted, then
`python tools/bench_freeze.py`), at L3 and again at L4.
