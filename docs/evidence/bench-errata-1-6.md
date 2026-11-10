# Benchmark errata for RoboLang 1.6 — 2026-11-10

Decision record [0036](../../spec/decisions/0036-lang-4-measured-values.md), PR 2 (V11 step 4). From this change on, the
reference and both prompts teach RoboLang 1.6. Every benchmark input that changed is listed here, with its old and new
form. The set 5 baseline under 1.5, taken before these changes, is [bench-set5-baseline.md](bench-set5-baseline.md)
(`main` at `d640dbc`); the 1.5 freeze before this PR is in [bench-errata-1-5.md](bench-errata-1-5.md), erratum F4. No
paid run was made for this change: the acceptance run (V12) is confirmed by the owner before it starts.

## Labels (0036 "Relabels", V13)

The old label is kept in each task as `expect_1_5`. `expect` is the 1.6 label. `expect_1_6` holds the same label, with
`paths` where the task has them.

### Set 5 (`bench/tasks_values/`, 26 tasks)

| Task | Before (`expect`) | After (`expect`) |
|---|---|---|
| `v01_probe_report` … `v15_retry_reading` (15 `value` tasks) | cannot / inexpressible; `expect_1_6.planned_paths` was text | **program**, with real `expect_1_6.paths` (world patches, below) |
| `i01_scale_offline`, `i02_camera_temp`, `i03_probe_elsewhere` | cannot / inexpressible | cannot / **infeasible**; `entities_1_6` becomes `blockers` (the field the scorer reads) |
| `x01_average` … `x06_set_oven` (6) | cannot / inexpressible | unchanged |
| `c01_hot_pan_words`, `c02_inspect_not_measure` | program | unchanged |

Paths of the 15 `value` tasks (probe values are in `world_patch`; the base world values are in "Worlds"):

| Task | Paths (`when`) |
|---|---|
| `v01` | pan 1 at 55 degC → pan at sink, the reading sent to `pass` as a value |
| `v02` | 45 degC → pass; 80 degC → left on the stove (or the arm stays there), a message to `pass`; **59.8 degC → the run fails** (S261: 59.8 ± 0.5 against 60 is unknown) |
| `v03` | 80 degC, a `property` event cools pan 1 to 55 degC at 240 s → pass; 80 degC, never cools → the run fails |
| `v04` | tray B at 1.2 kg → at the pass; a `mass` reading of `tray_b` by `helper_arm` is recorded |
| `v05` | 1.2 kg → pass; 2 kg → message to `pass` and the run fails (the limit is 1500 g, the scale reports kg) |
| `v06` | LOT-7731 → insulin at packing; LOT-0001 → the run fails |
| `v07` | 5 degC → packing; 12 degC → left in fridge 2 (or the picker stays there), the temperature sent to `qa` as a value |
| `v08` | vial warmer → "vial"; flask warmer → "flask"; both at 10 degC (within uncertainty) → the run fails (S261) |
| `v09` | arm_x with thermo_1 (arm_X's thermo1 is offline until 8 s); the reading sent to `lab` |
| `v10` | probe available → bowl 1 at prep_1; probe unavailable → "no reading" sent, bowl 1 at prep_1 |
| `v11` | 45 degC → pass; 80 degC → left; the reading sent to `pass` on both |
| `v12` | 55 degC (131 F) → pass; 80 degC → the run fails (or leaves it) |
| `v13` | LOT-7731; cart1 sends picker1's reading to `dispatch` |
| `v14` | 45 degC → the run fails (or leaves it); 60 degC → pass; 75 degC → the run fails (or leaves it) |
| `v15` | probe works → the reading sent to `pass`; probe never works → the run fails |

`v15`: a world patch cannot make only the first try fail, so the "first fails, second works" case is not a path. It is a
test instead (`tests/test_bench_values.py::test_a_retry_in_a_handler_reads_the_same_name_when_the_first_reading_fails`,
a scripted fault on step 2).

### Other sets

| Set | Task | Before (`expect`) | After (`expect`) |
|---|---|---|---|
| "ask" | `u05_below_8c` | cannot / inexpressible, group `inexpressible` | **program**, group `value`; paths 5 degC → packing, 12 degC → stop (or leave it in fridge 2); a `measure` of `insulin_1` by `picker1` is expected; reference waits for `sensor_ok` (online at 5 s) first |
| "ask" | `u06_cooled_60c` | cannot / inexpressible | **program**, group `value`; paths pan 1 already at 55 degC (any timeout passes) → pass, 80 degC never cools → the run fails; **world** `kitchen.json` → `kitchen_values.json` (in `kitchen.json` the chef arm holds `knife_1` and cannot grasp the pan, S212) |
| "ask" | `u19_send_batch` | cannot / inexpressible | **program**, group `value`; `label` of `insulin_1` by `picker1`, sent to `dispatch` as a value; `goal_free` |
| "ask" | `u20_report_temp` | cannot / inexpressible | **program**, group `value`; `temperature` of `pan_1` by `chef_arm`, sent to `pass` as a value; `goal_free` |
| "ask" | `i12_scale_down` | cannot / inexpressible | cannot / **infeasible**, group `infeasible`, `blockers: ["scale_1"]`, `attempt_fails_with: S232`, `fix_patch` (scale available), new attempt `bench/reference_ask/i12_scale_down.attempt.*` |
| 1 | `w_scale_precheck` | program (an `inspect` counted as weighing) | program, **plus `expected_readings: [["arm2", "mass", "part7"]]`**; reference uses `measure` |
| 2 | `d_serum_tube` | program (`inspect` counted as taking a temperature) | program, plus `expected_readings: [["arm_X", "temperature", "tube_A"]]`; reference uses `measure` |
| "ask" | `f07_thermo_then_dispatch` | program (`inspect`) | program, plus `expected_readings: [["picker1", "temperature", "insulin_1"]]`; `expected_action_sequences` ask for a `measure`; reference uses `measure` |
| all | `x_count_report`, `x_report_duration`, every other label | | unchanged |

There was no set 2 twin of the five tasks: no other task needs a value. `d5_two_inspections` and `l_two_sensors`
("check ... with the thermometer") keep `inspect`: they do not ask for a reading.

`why` text changed for: the 15 `value` tasks (the 1.5 "cannot read a value" reason became what the task tests), the 6
`x` tasks ("RoboLang 1.5 also cannot read a value" removed), the 3 set 5 infeasible tasks and `i12`, and `u05`, `u06`,
`u19`, `u20` (set "ask" had a "no numeric reading" and a "communicate sends a literal" reason).

Group counts of set "ask": `feasible` 12, `decoy` 11, `value` 4, `infeasible` 20 (was 19), `inexpressible` 15 (was 20),
`clarify` 18.

## Worlds (world file 1.2)

`world_schema_version` "1.1" → "1.2" and `measures` / `properties` added. Every other field is unchanged. Each of these
worlds was checked against every task of every set that uses it: all tests pass, so no label and no reference of a task
that is not listed above changes. The model is shown what a sensor measures (`bench/lib.py` `describe_world`) and never
what an object reads.

| World | Sensors with `measures` | Object `properties` (base values) |
|---|---|---|
| `kitchen.json` (and the `kitchen_values.json` variant) | `probe_temp` temperature degC ± 0.5, contact; `scale_1` mass kg ± 0.01, contact (still unavailable in `kitchen.json`, available in `kitchen_values.json`); cameras measure nothing | `pan_1` 55 degC, `pan_2` 30 degC, `tray_a` 0.9 kg, `tray_b` 1.2 kg |
| `pharmacy.json` | `thermo_p1` temperature degC ± 0.2, contact; `scan_p1`, `scan_p2` label, not contact; `cam_cart1` nothing | `insulin_1` 5 degC and label LOT-7731; `vial_a1` LOT-0001, `vial_a2` LOT-0002 |
| `cleanroom.json` | `thermo_1`, `thermo1` temperature degC ± 0.2, contact; scopes and `cam_r` nothing | `vial_1` 22, `vial1` 4, `flask_1` 4, `flask1` 30, `tube_A` 4, `tube_a` 6 (all degC) |
| `warehouse.json` | `scale1` mass kg ± 0.01, contact | `box1` 2, `box2` 3, `crate1` 10, `part7` 0.5 (kg; the same as the existing `mass` fields) |

Why these values: pan 1 at 55 degC makes `v02`, `v11`, `v12`, `v14` and `u06` follow the path the request describes (cool
enough, in range, already cool), and every other path is a `world_patch`. 55 ± 0.5 is clearly below 60 and clearly in
50 to 70. Tray B at 1.2 kg is under the 1500 g of `v05`. Insulin at 5 degC is under the 8 degC of `u05` and `v07`. The
cleanroom decoys are chosen so that a program that measures the wrong vial or flask (`vial1`, `flask1`) gets the other
answer in `v08`. `kitchen_values.json` is re-serialised with the same content as before plus these fields.

## Reference

`docs/language-reference.md` and its package copy `src/robolang/reference/language-reference.md` (identical, ASCII):

| Before (1.5) | After (1.6) |
|---|---|
| Title "RoboLang 1.5 language reference"; files start `robolang 1.5;` | "RoboLang 1.6 language reference"; `robolang 1.6;` in every example |
| "no loops, counts, clocks, schedules, arithmetic, message waits, or variables beyond typed task parameters" | adds measured values (`measure`); no string formatting; variables are task parameters and the names `measure` binds |
| Reserved words list ends `if else fail` | adds `measure as` |
| A handler is allowed on `move`, `grasp`, `place`, `inspect`, `wait until` | also `measure` |
| Conditions table: the seven atoms | adds the comparison atom, "only in `if` and `wait until`" |
| — | New section **"Measured values"**: **`inspect` never reads a value; weighing, taking a temperature, reading a batch number or a barcode is `measure`** (V13 amendment 1), and a request for a weight or temperature is `measure` even when the value is not used afterwards. The four kinds and their units, exact conversion, °F is not a unit (the author converts), contact sensors (go to the object first, `S235`), the failures of a `measure`, "the value is read when the program runs, never settle a comparison from a description", comparisons and where they are allowed (`if`, `wait until`; not `pre` or `goal`, `E321`), live readings in `wait until` only (`E322`), uncertainty (a borderline reading is unknown and fails the run), bindings (set once, scope, the retry rebinding rule, `E317`, `E318`), `communicate ... with t`, no arithmetic, no formatting, no picking objects at run time |
| "What symbolic simulation requires" | adds `measure` |
| Diagnostics table | adds `E217`, `E317` to `E322`, `S234`, `S235`, `S236`; `E206` extended |
| Three complete examples (1.5) | all `robolang 1.6;`; a fourth measures, compares and reports |

## Prompts

| File | Before | After |
|---|---|---|
| `bench/prompt.md` | "You write RoboLang 1.5 programs", "when RoboLang 1.5 cannot say it"; example `robolang 1.5;` | 1.6 in all three places (the example is still a delivery) |
| `bench/prompt_minimal.md` | 1.5; "the 1.3 grammar with the changes of 1.4, then 1.5. The three files follow" | 1.6; adds `grammar-v1.6.ebnf`; "The four files follow" |
| `bench/lib.py` `GRAMMAR_FILES`, `GRAMMAR_FILE` | grammars 1.3, 1.4, 1.5; `GRAMMAR_FILE` = 1.5 | grammars 1.3, 1.4, 1.5, 1.6; `GRAMMAR_FILE` = 1.6 |
| `bench/lib.py` `ANSWER_SCHEMA` (sent as the output format) | descriptions said "RoboLang 1.4" (stale since 1.5) | "RoboLang 1.6" |
| `bench/lib.py` `describe_world` (what the model is shown) | a sensor: `probe_temp (on chef_arm, available)` | with `measures`: `probe_temp (on chef_arm, available; measures temperature in degC +/- 0.5, contact)`; a label sensor: `scan_p1 (on picker1, available; measures label)`; a `property` event is never listed; objects' `properties` are never listed. Byte-identical for worlds without `measures` |

The assembled prompt hashes change; each run records its own.

## References

17 new in `bench/reference_values/` (every program task of set 5; the 2 controls now start `robolang 1.6;`); in
`bench/reference_ask/`: `u05`, `u06`, `u19`, `u20` (new), `f07_thermo_then_dispatch` (rewritten) and the new
`i12_scale_down.attempt.*`; `bench/reference/w_scale_precheck.*` and `bench/reference_hard/d_serum_tube.*` (rewritten).
All are 1.6 programs that `validate`, run and pass their task on every path (`tests/test_bench_values.py`,
`test_bench.py`, `test_bench_hard.py`, `test_bench_ask.py`). `d_serum_tube` no longer exports a Core 0.1 goal: its
`wait until` has a handler (1.5), so the `fail` path has no goal; the test says so.

## Scorer (V13)

New optional task fields and measures. `bench/gate.py` is unchanged.

| Field | Meaning |
|---|---|
| `expected_readings: [[robot, kind, object], ...]` | each must appear as a recorded reading (a `measure` that succeeded, with that robot) on every path that is not a stop. A path may override it with its own `expected_readings` (`[]`: a path where the probe is down reads nothing) |
| `expected_messages: [{"channel", "value_of": [object, kind]}]` | the message was sent on that channel as a reading of that object and kind, not as text. Also allowed in a path's `messages`. The old `[channel, text]` entries are unchanged |
| `expect_1_6.paths` | `task_paths` looks at `expect_1_6`, then `expect_1_5`, then `expect_1_4` |
| path `goals` | several acceptable end states ("leave it": the object stays, or the robot that probed it stays); on a `fails` path, reaching one of them is also accepted |
| `goal_free: true` | the request names no end state ("send the batch number"): the goal is right if the run succeeds |

| Measure | Definition |
|---|---|
| `value_ok` | per answer, over tasks with `expected_readings` or a `value_of` message: every non-stop path passes both checks. False when the answer is not a valid program. In `report.md` (rates table) and in each row of `scores.json`, next to `checks_ok` |
| `silent_substitution` | per answer, over every answer to a task with `expected_readings`: a valid program with no `measure` of an expected (kind, object) and an `inspect` of that object (0036 V13 amendment 1). Reported like `unsafe_continue`; in `scores.json` per row |

The check of a `value_of` message is in `value_ok`, not in `plan_succeeds` or `checks_ok`: a program that reaches its goal
but sends a literal still has `goal_correct` and `plan_succeeds`, and fails `value_ok`. Tests: the v04 baseline answer
(`inspect r tray using scale`, then the move) is `silent_substitution` and not `value_ok`; its reference is the opposite.

## Other changes in the same PR

- `robolang reference` and the CLI help said 1.5; they now say 1.6. README updated.
- `tests/test_lang16.py::test_v10_no_reading_go_on` removed the pan's temperature before testing S236 (the benchmark
  world now states one).
- Tests that pinned the 1.5 prompts or the old labels: `tests/test_reference.py`, `test_bench.py`, `test_bench_ask.py`,
  `test_bench_hard.py`, `test_bench_values.py`.

## Freeze v2

`docs/evidence/bench-freeze-v2.json` regenerated (this document is the erratum). Before: `3a5383fbab6053ed09641dd3764cc17baac3f788517e8d6c7e8f3c9ac88b1512` (760 files). After:
`f740b232e52a939b3a8065c50a8671d1240a4e2a7aa784d37f408795bfecf234` (800 files). Run `python tools/bench_freeze.py --check` before a paid run.

## What happens next (V12, V13)

The acceptance run, confirmed by the owner before it starts: set 5 under 1.6 with both prompts, 3 models × 3 repeats; sets
1, 2, 3 and "ask" once each with the full prompt (and 4). Set 5 expressible tasks: goal correct on every path and
`value_ok` ≥ 90% (Sonnet, full prompt); `silent_substitution` ≤ 5% (full prompt); inexpressible controls: false programs ≤
5%, even grammar-only; other sets: `gate.json` passes and `decision_correct` drops no more than 2 points or one answer.
If it fails, 1.6 stays in code only and is not taught (the reference and prompts go back to 1.5).

## Erratum G1 — entities that may appear after the job starts (2026-11-10)

Found in the 1.6 acceptance run (`acc16`, $7.53): with the grammar-only prompt, Sonnet (3/9) and Haiku (2/9) wrote a
program for `x03_any_hot_pan` ("check every pan on the stove (there may be several by then) and move any above
80 °C") that measures the one pan the world lists. With the full prompt every model declined (0/27). The grammar-only
baseline under 1.5 was 0/18, because no reading could be written. Owner decision, 2026-11-10 (option C): add one general
rule to both prompt wrappers, then re-run set 5 with both prompts.

| | Before | After |
|---|---|---|
| `bench/prompt.md`, `bench/prompt_minimal.md`, before "Decline only when the request cannot be met;" | (no sentence) | "A request about things that may appear or change after the job starts (more objects than the world lists, \"any that arrive\", \"several by then\") cannot be represented: the program can only name entities that the world lists now, so do not write a program for the ones listed and call it done." |

Test: `tests/test_reference.py::test_both_prompts_say_a_request_about_entities_not_listed_now_cannot_be_represented`.
Freeze v2 regenerated: `f740b232e52a939b3a8065c50a8671d1240a4e2a7aa784d37f408795bfecf234` → `ad10351968706dd2d1555ada954fc6607e5567b95efec00f955eca3c91eac37e` (800 files).

## Erratum G2 — a possible unknown is not a reason to decline (2026-11-10)

Follow-up 1 of [bench-1-6-acceptance.md](bench-1-6-acceptance.md): after erratum G1, Sonnet declined
`v08_warmer_of_two` in 3 of 3 full-prompt answers, saying that comparing two readings "could be unknown".

| | Before | After |
|---|---|---|
| Reference, "Measured values", uncertainty bullet | ends "Labels are exact and never unknown." | adds: "**The chance of an unknown is never a reason to decline.** \"Which of the two is warmer\", \"if it is below 8 degC\" are programs: compare the two names (or the name and the limit) in an `if`. If the readings are too close to tell, the run fails with `S261` instead of guessing, which is the intended outcome." |

Confirming run (set 5, full prompt, 3 models × 3 repeats, $1.69): Sonnet value tasks 45/45 (was 41/45), `v08` written
3 of 3; Opus 42/45, Haiku 44/45 (one `v08` decline); 0 false programs on inexpressible and infeasible tasks;
0 `silent_substitution`. Test: `tests/test_reference.py::test_reference_says_a_possible_unknown_is_not_a_reason_to_decline`.
Freeze v2 regenerated: `ad10351968706dd2d1555ada954fc6607e5567b95efec00f955eca3c91eac37e` → `272da035ed05c09f12c5e16ce7c70b3f55d9e6bac11256689a79ec736911ebfb` (800 files).
