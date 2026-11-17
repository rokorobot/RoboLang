# Benchmark set 7: declared actions (pre-registration draft)

**Status:** **APPROVED by the owner, 2026-11-17** (tasks, labels, clarifications C1–C15 in 0040, bounds B1–B8); frozen.
Decision record: [0040](../../spec/decisions/0040-declared-actions.md) (APPROVED by the owner, 2026-11-17). Directory:
`bench/tasks_actions/`, `bench/reference_actions/`; tests `tests/test_bench_actions.py`; run with
`python bench/run.py --set actions`. Modelled on set 6, [bench-6-regions.md](bench-6-regions.md). No model was run and no
money was spent. **The labels follow the recommended options of 0040 A1–A11 as approved. Where 0040 does not decide what a
label needs, the label states the reading used and the question is listed under "Open points" below: an owner answer
that differs changes the labels named there.**

## How the set is used

- **Labels.** `expect` is the **1.7** label and is what the baseline scores. `expect_1_8` is the label for acceptance once
  1.8 is taught (L4 moves it to `expect` and keeps the old one as `expect_1_7`, as set 6 did). The 1.8 label of a program
  task carries its `paths` (goal and world patch) and, on every non-control task, the world 1.4 facts it rests on.
- **Baseline (now, 1.7 reference):** the 20 tasks that are not controls expect `cannot / inexpressible`: 1.7 has no `do`
  and no `in_state` (`do` fails with `E203`, `in_state` with `E204`; the attempts are in `bench/reference_actions/`).
  A false program here is the **silent replacement**: a valid program that reaches a goal and writes a `move`, an
  `inspect`, a `communicate` or nothing in place of the action. Every attempt is built so that without its `do` line it
  is a valid 1.7 program that ends SUCCEEDED (the tests run them), so no old check can tell. The 4 controls expect a
  program.
- **Acceptance (after 1.8 is taught):** the 1.8 labels apply. A program is judged as in set 6 (goal correct on every
  path) **and** by a new check `action_ok`, written in L4: the task calls exactly the actions asked in `expected_actions`
  (robot, action name, target, value), no more and no fewer, and none on a control. The 1.8 references are written in L4,
  not now.
- **Worlds:** existing worlds only (`ward`, `fry_station`, `site`, `site_1_3`, `kitchen_values`, `yard`, `pharmacy`); no
  new world file and no 1.4 field in any file. The 1.4 facts a 1.8 label needs (`actions` on a robot, `state` on an
  object or a location) are recorded in `expect_1_8.world_1_4`, a note no code reads: L4 writes the 1.4 copies from them.
  `site_1_3` is the one 1.3 file, because `a07` also asks a region limit.
- **`expected_actions`.** A top-level field on the 10 `action` tasks (a list of `{robot, action, target[, value]}`) and
  `[]` on the 4 controls, so a model that adds a `do` to a control fails it. It is the 1.8 label, read by no code until L4.
- **Not copied.** The three R3 probes (`f_act_set_fryer`, `f_act_open_closet`, `f_act_print_layout`) and `u15_open_oven`,
  `u16_charge_drone`, `x06_set_oven` stay in their own sets (`future`, `ask`, `values`) with new ids and new prompts here.
  L4 relabels them as programs there (0040 pre-registration); the bound below uses them.

## Tasks (24)

### Actions that become programs in 1.8 (10)

| id | world | Request | 1.7 label (attempt) | 1.8 label | Tests |
|---|---|---|---|---|---|
| `a01_open_then_fetch` | ward | Have moxi1 open closet_door, then take iv_kit_1 from the supply closet to room_412. | cannot / inexpressible (`E203`) | program; goal `in_state closet_door "open"` and kit at room_412; `do open` | open, then fetch |
| `a02_set_then_wait_reading` | fry_station | With flippy1, set the fryer to 175 degrees Celsius. Once the oil in oil_1 reads above 170 degrees Celsius (give it 60 seconds), move basket_2 from the prep_table to the fryer. If the oil does not get there, stop. | cannot / inexpressible (`E203`) | program; goal `in_state fryer "heating"` and basket_2 at the fryer; `do set ... to 175 degC`, then `wait until` the oil reads above 170 degC; paths: heats at 20 s → goal; never → the run stops | set a value, wait for the reading (A5) |
| `a03_print_gridline` | site | Have printer1 print the grid marks at gridline_c. | cannot / inexpressible (`E203`) | program; goal `in_state gridline_c "printed"`; `do print` on a location | location target (A4) |
| `a04_open_if_clear` | kitchen_values | If pass_area is clear when the job starts, have chef_arm open oven_door_1. If it is not, send "pass busy" to channel "kitchen" from chef_arm and stop the job as failed. | cannot / inexpressible (`E203`) | program; paths: clear → `in_state oven_door_1 "open"`; not clear → message and `fail` | action in an `if` |
| `a05_open_or_report` | ward | Have moxi1 open closet_door. If it will not open, tell channel "charge_nurse" "closet stuck" and stop the job as failed. | cannot / inexpressible (`E203`) | program; paths: as given → `in_state closet_door "open"`; the `open` fails → handler reports and stops | action with a failure handler (A7) |
| `a06_two_cranes_lock` | yard | Have crane1 lock cont_101 and crane2 lock cont_102. | cannot / inexpressible (`E203`) | program; goal both containers `in_state ... "locked"`; two `do`, one per robot | two robots, one action each |
| `a07_print_dry` | site_1_3 | Have printer1 print the layout at level_3_wall, and keep it out of wet_floor the whole way. | cannot / inexpressible (`E203`) | program; goal `in_state level_3_wall "printed"`; `no_go_region wet_floor` (1.7) and `do print` | action plus a region limit |
| `a08_charge_cart` | pharmacy | Send cart2 to the charger and have it charge there. | cannot / inexpressible (`E203`) | program; goal `in_state charger "charging"`; `do charge` on the charger | charge as a declared action (A11) |
| `a09_dock_drone` | yard | Fly drone1 to berth_1 and back to the workshop, then dock it there. | cannot / inexpressible (`E203`) | program; goal `in_state workshop "docked"` (with or without the drone at the workshop); `do dock` | dock as a declared action (A11) |
| `a10_decoy_open_closet` | ward | Have moxi1 open the supply closet. | cannot / inexpressible (`E203`) | program; goal `in_state closet_door "open"`; `do moxi1 open_door closet_door` (not `open`, not `supply_closet`) | near-miss action name and target (decoy) |

### Infeasible in 1.8 (5)

| id | world | Request | 1.7 label (attempt) | 1.8 label | Why |
|---|---|---|---|---|---|
| `i01_runner_cannot_open` | kitchen_values | Have the runner open oven_door_1. | cannot / inexpressible (`E203`) | cannot / infeasible (A6): the runner declares no action; chef_arm and helper_arm declare `open` | robot lacks the action |
| `i02_lock_a_berth` | yard | Have crane1 lock berth_2. | cannot / inexpressible (`E203`) | cannot / infeasible (A6): `lock` acts on an object, berth_2 is a location | wrong target kind (an object action on a location) |
| `i03_fryer_in_kilograms` | fry_station | Have flippy1 set the fryer to 2 kg. | cannot / inexpressible (`E203`) | cannot / infeasible (A6): `set` takes a temperature, 2 kg is a mass | a value of the wrong kind |
| `i04_open_by_distance` | ward | Have moxi1 open closet_door to 30 cm. | cannot / inexpressible (`E203`) | cannot / infeasible (A6): `open` declares no value | a value for an action that takes none |
| `i05_leave_it_locked` | ward | Have moxi1 leave closet_door locked. | cannot / inexpressible (`E203`) | cannot / infeasible (A7): `open` and `close` are declared and none leaves the door locked | the result is not the state the request needs |

### Still inexpressible in 1.8 (5)

| id | world | Request | 1.7 label (attempt) | 1.8 label | Why it stays out | Atlas gap |
|---|---|---|---|---|---|---|
| `x01_hand_to_ana` | ward | Have moxi1 hand iv_kit_1 to Ana at room_412. | cannot / inexpressible (`E203`) | cannot / inexpressible | several targets: the kit and a person (A4) | G-PROX |
| `x02_selftest_result` | fry_station | Have flippy1 run the fryer's self-test and tell channel "fry" the result. | cannot / inexpressible (`E203`) | cannot / inexpressible | an action that returns a value (A10): that is `measure`, and the self-test has no sensor | G-ACT |
| `x03_teach_restock` | ward | Teach moxi1 a skill called restock: open closet_door, then take iv_kit_1 to ward_3. Do not run it now. | cannot / inexpressible (`E203`) | cannot / inexpressible | a macro, a name for steps kept for later (A10, A11) | G-TEACH |
| `x04_open_until_open` | kitchen_values | Have chef_arm keep trying to open oven_door_1 until it is open. | cannot / inexpressible (`E203`) | cannot / inexpressible | an action repeated until something holds: a loop with no bound (A11, 0041) | G-LOOP |
| `x05_open_at_six` | kitchen_values | At 06:00, have helper_arm open oven_door_1. | cannot / inexpressible (`E203`) | cannot / inexpressible | an action triggered at a clock time: there is no clock | G-CLOCK |

### Controls: programs in both versions (4)

| id | world | Request | Label (both versions) |
|---|---|---|---|
| `c01_fetch_no_door` | ward | With moxi1, take iv_kit_1 from the supply closet to room_412. | program, plain `move`/`grasp`/`place`; the door of the closet is closed in the 1.8 world and the request does not mention it, so a model that adds an `open` fails |
| `c02_oil_reading` | fry_station | With flippy1, go to the fryer, measure the oil temperature and send the reading to the "fry" channel. | program, a `measure` and a value message; no `do` |
| `c03_wait_oil_by_itself` | fry_station | With flippy1, wait up to 60 seconds for the oil in oil_1 to read above 170 degrees Celsius, then move basket_2 from the prep_table to the fryer. If the oil does not get there, stop. | program, a `wait until` on a live reading; paths: heats at 20 s → basket at the fryer; never heats → the run stops; no `do set` |
| `c04_plain_move_printer` | site | Send printer1 to gridline_c. | program, a plain move; printer1 declares `print` in the 1.8 world and the request does not ask for it |

## Acceptance bounds (APPROVED by the owner 2026-11-17, with the baseline in hand, rule 6)

Baseline: [bench-set7-baseline.md](../evidence/bench-set7-baseline.md) (2026-11-17, 1.7, 3 repeats). The smallest step
is one answer (rule 6). The acceptance run repeats the baseline under 1.8, with world 1.4 copies.

| # | Group | Measure | Baseline (1.7) | Proposed bound (1.8, per model) |
|---|---|---|---|---|
| B1 | 10 `action` programs | goal on every path (including `in_state`) **and** `actions_ok`, meaning the right action on the right target with the right value; full prompt | not expressible | ≥ 90%: at least 27/30 |
| B2 | the same, grammar-only | the same | not expressible | for information |
| B3 | 5 `inexpressible` | false programs, full and grammar-only | 0/15 for each | at most 1/15 for each model and prompt |
| B4 | 5 `infeasible` | decision `cannot`; no valid program that succeeds | 15/15 for each | at least 14/15, and 0 that succeed |
| B5 | 4 `control` | right decision and no `do` added | 12/12 for each | at least 11/12 |
| B6 | sets 1 and 2 (standing) | `gate.json`; decision accuracy | the latest run | the gate passes; `decision_correct` drops at most 2 points or one answer, whichever is larger |
| B7 | economy | keywords, statements, conditions, world fields, codes | — | 2 keywords, 1 statement, 1 condition, 3 world fields (`actions`, `state`, `location_states`); about 5 user-facing codes plus their IR forms |
| B8 | 10 `action` programs | **action replaced silently**: a valid, succeeding program that leaves out or replaces the requested action; full prompt | Haiku 3/30, Sonnet 0/30 | at most 1/30 |

If a bound fails, 1.8 stays in code only and is not taught.

## Cost of the baseline and acceptance runs (not spent)

`python bench/run.py --set actions --repeats 3 --models claude-sonnet-5-5,claude-haiku-5-5`, effort medium, as the
`Benchmark` workflow runs it with `actions: true` (full and grammar-only prompts, `--max-usd 3` each, not gated, never in
the per-PR or weekly runs):

| Prompt | Requests | Estimate |
|---|---:|---:|
| Full | 90 per model | ~$3.58 (Sonnet ~$3.41, Haiku ~$0.17) |
| Grammar only | 90 per model | ~$3.48 (Sonnet ~$3.31, Haiku ~$0.17) |

The estimate is a guess (`bench/README.md`) and is a little above the $3 cap: the cap stops new requests, and a rerun with
the same `--run-id` resumes. The set-6 baseline cost $0.73 against an estimate of ~$6.5, so the real cost is likely to be
well below the cap.

## Open points: where 0040 does not decide what a label needs (owner, before L4)

Each point names the labels that depend on it. The label uses the reading given first.

1. **`in_state` in an expected goal.** Labels write it as a triple `[subject, "in_state", label]` next to `located_in`
   triples. 0040 A8 says Core 0.2 has no state triples and `export-goal` refuses `in_state`, so how L4's scorer compares a
   program's goal with an expected one (as `goal_means` does for atoms) is open. Affects every `action` task.
2. **Does `do` need the robot at the target?** `grasp` and a contact `measure` do (move first); A7 says only that `do`
   sets the target's state. Labels do not require a `move`, and the attempts add one where the request suggests it.
   Affects `a03`, `a06` (crane2 is at berth_2 and locks a container at berth_1), `a07`, `a08`, `a09`, and every action on a
   location.
3. **What the goal of an action task contains.** The labels require `in_state target "<result>"` and no `robot_at`
   (`a09` accepts both forms, `goals`). "Open, then fetch" (`a01`) does not check the order: A7 gives no rule that a closed door
   stops a `grasp`, so `in_state closet_door "open"` at the end is the only check of the opening.
4. **The result of a setpoint action (`a02`).** A3 declares one `result` label per action. The label uses `"heating"`,
   on the location `fryer` (the world has `fryer` as a location and `oil_1` as the object), and does not put the value in the
   goal: A5 records it and says it is checked with `measure` or `wait until`. If the owner wants the target to be `oil_1`, or the value
   in the state, `a02`, `i03` and `x02` change. Also: a request that says "until the oil reads 175" waits on `>= 175`, which
   is unknown for a reading of 175 ± 0.5 and never resolves (1.6 uncertainty); the task asks "above 170" for that reason.
5. **Where a location's state lives in a world file.** Locations are a plain list of IDs in worlds 1.1 to 1.3, and A3 says a
   location may carry `state`. The note uses `location_states: {id: label}` as a placeholder. Affects `a02`, `a03`,
   `a07`, `a08`, `a09`, `i03`, `x02`.
6. **How a world scripts a failing `do`.** A7 says it can fail "like any step (a scripted fault)", but no world field is
   named. `a05`'s second path records `scripted_fault: {step, action, target}` as a placeholder, not a `world_patch`.
7. **`i05` "leave the door locked".** A6 lists three checks before the run: the robot declares the action, the target kind
   matches, the value kind matches. "No declared action leaves this state" is not one of them. The label says `infeasible`
   (the request cannot be done in this world), which needs a fourth check, or the owner may prefer that such a program is
   valid and its run fails at the goal. Then `i05` becomes a different kind of task.
8. **Durations as values.** `set the fryer to 4 minutes` is not a task here: A10 excludes the duration of an action, and
   1.6's kinds (temperature, mass, distance, label) have no duration, so the verdict (static error or infeasible) is
   open. `i03` uses a mass instead.
9. **`x02` has no atlas gap of its own.** "An action that returns a value" is `measure` when a sensor reads it (A10), and
   the self-test has none. It sits in G-ACT, which L6 closes (A10 keeps it out), so the owner chooses at L6: a new gap, or
   G-ACT stays open for it, as set 6's `x01` and `x02` moved to new gaps. Whether it is `inexpressible` or `infeasible` (no
   sensor) is the same question.
10. **`x01` and a person.** Ana is not a world entity, so the request could also be judged `infeasible` (no such entity).
    The label follows A4 (several targets, cooperation: G-PROX).
11. **`x03` and macros.** A 1.8 task file is already a named list of steps. The request is out because it asks for the name
    to be kept (and not run); if the owner counts "write a task called `restock`" as expressing it, `x03` becomes a program.
12. **`a10` and the decoy.** The label resolves "open the supply closet" to the object `closet_door` with the declared name
    `open_door`: the world also holds a location `supply_closet`, and a model may bind that. It assumes the name must match
    the profile exactly (no fuzzy matching, 0040 A1). L4 must list each robot's actions in the world text the model reads,
    or the decoy cannot be solved.
13. **Limits and actions (`a07`).** A9 says the region and speed limits "cover every motion, actions included". What that
    means for the target of a `do` (is a location target checked as `S004`, does an action need `honours`?) is not decided.
    `a07` avoids the question: `level_3_wall` is outside `wet_floor`, and printer1 honours both kinds.
14. **Charge and dock (`a08`, `a09`).** A11 says they are declared actions and that the battery comes later (G-SELF). The
    labels put the state on the charger or the dock (a location target), not on the robot, because a robot cannot target itself
    in A4. If the owner wants the robot's own state, A4 needs a change.
15. **Atlas gap for `x03`.** G-TEACH had no task; `x03` is its first (A10, A11), and its scope note says so.
