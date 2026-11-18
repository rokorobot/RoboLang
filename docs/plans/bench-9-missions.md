# Benchmark set 9: missions (pre-registration draft)

**Status:** **APPROVED by the owner, 2026-11-18** (tasks, labels, clarifications 1–14 in 0041, bounds B1–B9); frozen.
`tests/test_bench_missions.py`; run with `python bench/run.py --set missions`. Modelled on set 8,
[bench-8-clock.md](bench-8-clock.md). No model was run and no money was spent. **The labels follow 0041 M1–M10 as
approved. Where 0041 does not decide what a label needs, the label states the reading used and the question is listed
under "Open points" below: an owner answer that differs changes the labels named there.** The acceptance bounds are
placeholders, to be fixed with the baseline in hand (rule 6, [STRATEGY.md](../STRATEGY.md)); nothing here is frozen
yet. No language, spec, validator or simulator change: that is L4. (0041 numbers this set 8, as it was written before
the clock set existed: it is set 9.)

## How the set is used

- **Labels.** `expect` is the **1.9** label and is what the baseline scores: every non-control task expects `cannot /
  inexpressible`, because 1.9 has no repetition above a task. `expect_2_0` is the label for acceptance once 2.0 is
  taught (L4 moves it to `expect` and keeps the old one as `expect_1_9`, as set 8 did). The 2.0 label of a program
  carries an `expected_mission` and its `paths`.
- **`expected_mission`.** A placeholder form for L4's scorer: `runs` (the `run` lines in order, each a `task` intent,
  the `entities` it is given in the order of the request, the `goal` triples that its verified goal means, and its
  `limits` or `actions` where the task carries a 1.7 limit or a 1.8 `do`), `repeat` (`max_runs` and/or `max_duration`,
  or `null` for a mission of one run with a retry), `until` (a `condition` string and Core-style `triples` where the
  condition is a fact, `null` for none) and `on_failure` (`stop`, `continue` or `{"retry": N}`). On a control it is `null`:
  a model that adds a mission to a control fails it. Every `path` has a `when`, the `world_patch` (events, objects)
  that forces it, optional `run_faults` (`{"run": k}` or `{"run": "all"}`: the task run k fails, a form that the world
  file has no field for yet, as set 4's `scripted_fault` had none for a `do`), and the expected mission `outcome`
  (`completed`, `bounded`, `failed`), `runs`, `runs_verified` and `failed_runs`.
- **What makes a request a mission and not a task.** A request is a mission when its repetition is **not a fixed
  short list**: the count is more than ten, or it depends on the world (an `until`) or on the clock (a duration), or a
  failure policy applies between runs. Ten is the longest fixed chain a 1.9 task writes in this repository
  (`f07_ten_tries`, `if not holding` steps). A fixed small count is a task that 1.9 already has: that is the
  controls (`c01`, `c04`), so that a model that turns "three times" into a mission fails. The line is open point 10.
- **Baseline (now, 1.9 reference):** the 22 tasks that are not controls expect `cannot / inexpressible`. The obvious
  attempt of each (in `bench/reference_missions/`) is the task and a `mission` written as 0041 shows it; 1.9 rejects
  it with `E201` (`expected 'task', found 'mission'`). Every attempt marks the mission lines with `// @mission`; without
  them it is a valid 1.9 program that runs the task once and ends SUCCEEDED (the tests run all 22). That program is the
  **repetition replaced silently**: the repetition dropped, as the old checks cannot tell. The 4 controls expect a
  program (references `.rob`, `.json`, valid and SUCCEEDED on every path).
- **Acceptance (after 2.0 is taught):** the 2.0 labels apply. A mission is judged as in set 8 for its task runs (the
  goal of each run) **and** by checks written in L4: `mission_ok` (the runs, the bound, the `until` and the failure
  policy as in `expected_mission`, no more and no fewer), the outcome and the runs of each path in the simulator, and
  **bound dropped or invented** (below). The 2.0 references are written in L4, not now.
- **Worlds:** existing ones only (`yard`, `warehouse`, `assembly`, `site_1_3`, `kitchen_values_1_4`,
  `warehouse_1_5_1420`, `yard_1_5_2130`, `fulfilment`). No new world file. Missions need a clock only through a task
  that reads it, and none of the programs does; time passes through `wait for` inside the task, so a duration bound or a
  world event is reached after a known number of runs.
- **Not copied.** The standing G-LOOP tasks (`x_every_hour`, `x_keep_feeding`, `x_retry_grasp`, `x_wait_forever`,
  `u03_until_stop`, `u04_back_and_forth`, `x04_every_minute`, `f10_retry_until_works`, `f13_retry_every_minute`,
  `x_all_parts_ever`, `x_retry_until_grasped`, `x_shuttle_until_five`, `u18_every_morning`, set 7's
  `x04_open_until_open`, set 8's `x01_every_night_round`) stay in their sets with their ids; this set has new ids and new
  prompts for the same shapes. See "What this means for the standing tasks".

## Tasks (26)

Attempt code: the first diagnostic of the obvious attempt under 1.9, `E201` for all 22 (the others are the 4
controls, which have a reference). 26 tasks: 10 + 5 + 7 + 4.

### Programs in 2.0 (10)

| id | world | Request | 1.9 label (attempt) | 2.0 label: `expected_mission` and paths |
|---|---|---|---|---|
| `m01_shuttle_thirty_rounds` | yard | With tug1, drive from gate_in to gate_out and back to gate_in, again and again for at most 30 rounds. If a round fails, stop. | cannot / inexpressible (`E201`) | program; run `round(tug1, gate_in, gate_out)`; at most 30 runs; until none; on failure stop. Paths: every round works -> `bounded` (30 runs); round 7 fails -> `failed` (7 runs) |
| `m02_retry_the_carry` | warehouse | With arm2, carry part7 from the table to the inspection_bay. If the run fails, run it again, up to 12 more times; if it still fails, stop for good. | cannot / inexpressible (`E201`) | program; run `transfer(arm2, part7, table, inspection_bay)`; no `repeat` (one run); until none; on failure retry 12. Paths: the first run works -> `completed` (1 runs); runs 1 and 2 fail -> `completed` (3 runs); part7 is gone -> `failed` (13 runs) |
| `m03_inspect_for_three_hours` | assembly | With arm_b, inspect p4 with cam_s3, then wait 10 minutes, and repeat that for at most 3 hours. Stop as soon as fence_b is clear. | cannot / inexpressible (`E201`) | program; run `look(arm_b, p4, cam_s3, station3)`; at most 3 h; until `region_clear fence_b`; on failure stop. Paths: fence_b clears at 25 minutes -> `completed` (3 runs); fence_b never clears -> `bounded` (18 runs) |
| `m04_patrol_and_go_on` | site_1_3 | With spot1, patrol stop_a, stop_b, stop_c and stop_d in turn and come back to stop_a, for up to 12 rounds. If a round fails, note it and go on with the next round; do not stop. | cannot / inexpressible (`E201`) | program; run `patrol(spot1, stop_a, stop_b, stop_c, stop_d)`; at most 12 runs; until none; on failure continue. Paths: every round works -> `bounded` (12 runs); round 5 fails -> `bounded` (12 runs) |
| `m05_alternate_the_carry` | warehouse | With arm1, alternate between carrying box1 from shelf_a to the table, waiting 3 seconds after the placement, and carrying it back from the table to shelf_a, again waiting 3 seconds. Keep alternating for at most 24 carries in all, and stop as soon as the aisle is clear. | cannot / inexpressible (`E201`) | program; run `carry(arm1, box1, shelf_a, table)` + `carry(arm1, box1, table, shelf_a)`; at most 24 runs; until `region_clear aisle`; on failure stop. Paths: the aisle clears at 8 seconds -> `completed` (3 runs); the aisle never clears -> `bounded` (24 runs) |
| `m06_carry_until_there` | warehouse | With amr1, carry crate1 from the dock to the table. If crate1 is not at the table afterwards, carry it again, up to 15 times in all. A failed carry does not stop the job. | cannot / inexpressible (`E201`) | program; run `transfer(amr1, crate1, dock, table)`; at most 15 runs; until `object_at crate1 table`; on failure continue. Paths: the first carry works -> `completed` (1 runs); run 1 fails -> `completed` (2 runs) |
| `m07_rounds_until_quay_clear` | yard | With tug1, drive from gate_in to gate_out, wait 5 seconds and drive back to gate_in. Repeat that round until the quay is clear, but for at most 12 rounds. | cannot / inexpressible (`E201`) | program; run `round(tug1, gate_in, gate_out)`; at most 12 runs; until `region_clear quay`; on failure stop. Paths: the quay clears at 12 seconds -> `completed` (3 runs); the quay never clears -> `bounded` (12 runs) |
| `m08_rounds_out_of_the_wet_floor` | site_1_3 | With cleaner1, drive from dock_2 to the lobby and back to dock_2 without ever entering wet_floor. Do 25 such rounds, and stop at the first round that fails. | cannot / inexpressible (`E201`) | program; run `round(cleaner1, dock_2, lobby, wet_floor)`; at most 25 runs; until none; on failure stop. Paths: every round works -> `bounded` (25 runs); round 9 fails -> `failed` (9 runs) |
| `m09_open_the_oven_door` | kitchen_values_1_4 | With chef_arm, open oven_door_1. If it is still closed afterwards, try again, up to 15 attempts in all, and carry on after a failed attempt. | cannot / inexpressible (`E201`) | program; run `open_it(chef_arm, oven_door_1, oven)`; at most 15 runs; until `in_state oven_door_1 "open"`; on failure continue. Paths: the first attempt works -> `completed` (1 runs); attempts 1 and 2 fail -> `completed` (3 runs); every attempt fails -> `bounded` (15 runs) |
| `m10_scan_for_two_hours` | warehouse | With arm1, inspect box1 on shelf_a with camera_top and then wait 15 minutes, and keep doing that for 2 hours. | cannot / inexpressible (`E201`) | program; run `scan(arm1, box1, camera_top, shelf_a)`; at most 2 h; until none; on failure stop. Paths: every scan works -> `bounded` (8 runs); run 3 fails -> `failed` (3 runs) |

### Infeasible in 2.0 (5)

| id | world | Request | 1.9 label (attempt) | 2.0 label | Why |
|---|---|---|---|---|---|
| `i01_twelve_rounds_in_half_an_hour` | yard | With tug1, drive from gate_in to gate_out, wait 5 minutes and drive back to gate_in. Do 12 such rounds and have them all finished within 30 minutes. | cannot / inexpressible (`E201`) | cannot / infeasible (0041 M2) | 12 rounds with a wait of 5 minutes each need 60 minutes; the request allows 30 |
| `i02_the_same_box_again` | warehouse | With arm1, carry box1 from shelf_a to the table, and carry it from shelf_a to the table again, 15 times in all. | cannot / inexpressible (`E201`) | cannot / infeasible (0041 M4) | after run 1 box1 is at the table, so the grasp at shelf_a of run 2 fails |
| `i03_until_a_pallet_that_is_not_there` | yard | With tug1, drive from gate_in to gate_out and back to gate_in, at most 20 rounds, until pallet_9 is at wash. | cannot / inexpressible (`E201`) | cannot / infeasible (0041 M7) | the world has no pallet_9, so the until condition cannot be checked |
| `i04_more_retries_than_runs` | warehouse | With arm2, carry part7 from the table to the inspection_bay in at most 12 runs in all. If a run fails, run it again, up to 20 more times. | cannot / inexpressible (`E201`) | cannot / infeasible (0041 M4) | the request allows 12 runs in all and 20 retries on top of the first run |
| `i05_a_bound_shorter_than_one_round` | warehouse | With amr1, carry crate1 from the dock to the table, wait 2 minutes there and carry it back to the dock. Keep repeating that round, but stop everything after 90 seconds. | cannot / inexpressible (`E201`) | cannot / infeasible (0041 M2) | one round takes 2 minutes and the whole job may take 90 seconds |

### Still inexpressible in 2.0 (7)

| id | world | Request | 1.9 label (attempt) | 2.0 label | Why it stays out | Atlas gap |
|---|---|---|---|---|---|---|
| `x01_never_stop` | yard | Have tug1 drive between gate_in and gate_out forever, and never stop. | cannot / inexpressible (`E201`) | cannot / inexpressible (0041 M2) | No bound at all: "forever" stays inexpressible (0041 M2). | G-LOOP |
| `x02_every_hour_on_the_hour` | warehouse_1_5_1420 | Every hour on the hour until 18:00, have arm1 inspect box1 on shelf_a with camera_top. | cannot / inexpressible (`E201`) | cannot / inexpressible (0041 M6) | A time trigger: "every hour on the hour" starts a run at a time, which is a `when` clause that the mission layer gets in a later iteration (0041 M6). | G-LOOP |
| `x03_for_each_go_message` | yard | Each time channel "ops" sends "go", send tug1 from gate_in to wash and back to gate_in. Do this at most 10 times. | cannot / inexpressible (`E201`) | cannot / inexpressible (0041 M6) | A message trigger (0041 M6, I6): a run starts when a message arrives. | G-MSG |
| `x04_rounds_inside_rounds` | yard | Do up to 12 rounds. In each round, have tug1 drive from gate_in to gate_out and back to gate_in over and over, until lane_2 is clear, at most 30 times. | cannot / inexpressible (`E201`) | cannot / inexpressible (0041 M7) | A loop inside a loop: missions cannot nest and a task cannot start a mission (0041 M7). | G-LOOP |
| `x05_every_new_part` | assembly | For the next 8 hours, move every new part that arrives at conveyor_in to the bin with arm_a. | cannot / inexpressible (`E201`) | cannot / inexpressible (0041 (G-CHOOSE, I7)) | The bound is stated (8 hours) and the repetition is a mission, but the run needs the next part, which is chosen at run time from objects that do not exist yet: G-CHOOSE (I7). | G-CHOOSE |
| `x06_until_it_is_cool` | kitchen_values_1_4 | With chef_arm, measure the temperature of pan_1 with probe_temp and send the reading to channel "pass", then wait 1 minute. Repeat that for at most 30 times, until the reading is below 40 degrees Celsius. | cannot / inexpressible (`E201`) | cannot / inexpressible (0041 M3 (open point 2)) | The `until` is a comparison of a measured value. | G-LOOP |
| `x07_until_ten_to_ten` | yard_1_5_2130 | With tug1, drive from gate_in to gate_out, wait 5 minutes and drive back to gate_in. Repeat that round until 22:00, but for at most 40 rounds. | cannot / inexpressible (`E201`) | cannot / inexpressible (0041 M3 (open point 2)) | The `until` is a time of day. | G-LOOP |

### Controls (4)

| id | world | Request | Label (both versions) |
|---|---|---|---|
| `c01_three_round_trips` | yard | With tug1, drive from gate_in to gate_out and back to gate_in, three times. | program, one task with six `move`s (a fixed short sequence); no mission |
| `c02_grasp_once_more` | warehouse | With arm2, carry part7 from the table to the inspection_bay. If the grasp fails, try it once more, and if it fails again, stop. | program, a 1.5 step handler (`grasp ... else { grasp ... }`), paths: tries 1 / tries 2 then stop; no mission |
| `c03_wait_for_the_aisle` | warehouse | With amr1, carry crate1 from the dock to the table, but only after the aisle is clear. Wait up to 20 seconds for it and, if it is not clear by then, stop the job as failed. | program, one `wait until region_clear` with a timeout and `fail`; paths: clears / times out; no mission |
| `c04_three_named_totes` | fulfilment | With digit1, move tote_1, then tote_2, then tote_3 from inbound_conveyor to outbound_conveyor, one after the other. | program, one task that moves three named totes in turn; no mission |

## What this means for the standing tasks

Every one of the 15 standing G-LOOP tasks lacks a stated count or duration (two end at a time of day, open point 2): "until it works", "forever", "as long as it takes", "until I tell
you to stop", "every morning". Under M2 each stays inexpressible in 2.0 as written, whatever the mission layer does; 4 of
them are also triggers (M6) and 2 need the next object chosen (G-CHOOSE). The mission layer makes a request
expressible only when the request states its bound: this is why set 9's programs all do, and why `x01` and `x06` are in the
set (a request without a bound stays out, and a mission with an invented bound is a false program). If the owner wants
"until it works" to be a program, M2 has to give a default bound, and 0041 says it does not. L4 relabels the 15 as
`expect_2_0: cannot / inexpressible` and none moves to program.

## Acceptance bounds (APPROVED by the owner 2026-11-18, with the baseline in hand, rule 6)

Baseline: [bench-set9-baseline.md](../evidence/bench-set9-baseline.md) (2026-11-18, 1.9, 3 repeats). The smallest step is one answer (rule 6). The acceptance
run repeats the baseline under 2.0.

| # | Group | Measure | Baseline (1.9) | Proposed bound (2.0, per model) |
|---|---|---|---|---|
| B1 | 10 `mission` programs | `mission_ok` and the outcome of every path, full prompt | not expressible | ≥ 90%: at least 27/30 |
| B2 | the same, grammar-only | the same | not expressible | for information |
| B3 | 7 inexpressible tasks | false programs, full and grammar-only | 0/21 for each | at most 1/21 for each model and prompt |
| B4 | 5 infeasible tasks | decision `cannot`; no valid mission that succeeds | 15/15 for each (grammar-only Haiku 14/15) | at least 14/15, and 0 that succeed |
| B5 | 4 `control` | right decision and **no mission added**, full prompt | Haiku 12/12, Sonnet 9/12 (it declined the fixed count `c01` 3/3) | at least 11/12 |
| B6 | sets 1 and 2 (standing) | `gate.json`; decision accuracy | latest run | the gate passes; `decision_correct` drops at most 2 points or one answer, whichever is larger |
| B7 | economy | statements, kinds, literals, reserved words, top-level declarations, world fields, codes | — | 0 task statements, 1 top-level declaration (`mission`), at most 7 reserved words (`mission`, `repeat`, `run`, `runs`, `stop`, `continue`, `retry`; `until` and `for` exist), 0 world fields (the `run_faults` of the paths are a test form, not a world field), about 6 user-facing codes plus their IR forms |
| B8 | 10 `mission` programs | **repetition replaced silently**: a valid, succeeding 1.x program (no mission) that runs the task once, unrolls a number of copies the request did not ask for, or drops the repetition; full prompt | **Haiku 4/30, Sonnet 0/30** (grammar-only: Haiku 12/30, Sonnet 2/30) | at most 1/30 |
| B9 | 10 `mission` programs and `x01`, `x06`, `x07` | **bound dropped or invented** (0041): a mission without the bound the request states (a run count, a duration, `retry N`), or with one it did not state (a run count added to a duration-only request, a bound for "forever"), full prompt | not expressible | at most 1/30 on the programs; 0 missions written for `x01` |

If a bound fails, 2.0 stays in code only and is not taught.

## Cost of the baseline and acceptance runs (not spent)

`python bench/run.py --set missions --repeats 3 --models claude-sonnet-5-5,claude-haiku-5-5`, effort medium, as the
`Benchmark` workflow runs it with `set_only: missions` (full and grammar-only prompts, `--max-usd 3` each, not gated,
never in the per-PR or weekly runs):

| Prompt | Requests | Estimate |
|---|---:|---|
| Full | 98 per model | ~$4.00 (Sonnet ~$3.81, Haiku ~$0.19) |
| Grammar only | 98 per model | ~$3.86 (Sonnet ~$3.67, Haiku ~$0.18) |

The estimate is a guess (`bench/README.md`) and is above the $3 cap: the cap stops new requests, and a rerun with the same
`--run-id` resumes. The earlier sets cost well below their estimates. The workflow now has 5 `workflow_dispatch`
inputs: the string input `set_only` (values `set3`, `set4`, `future`, `regions`, `actions`, `clock`, `missions`; empty is
the usual run) replaces the six per-set booleans, which had reached GitHub's limit of 10.

## Open points: where 0041 does not decide what a label needs (owner, before L4)

Each point names the labels that depend on it. The label uses the reading given first.

1. **Several `run` lines in one repeat (`m05`).** 0041 shows one `run` line. M3 ("checked after each task run") and M4
   (`retry N`: runs "of the same task") do not say whether a repeat may hold several. The label allows it: the bound
   counts task runs (24 runs are 12 alternations), and `until` is checked after each run. If one run line only,
   `m05` becomes `cannot / inexpressible` (G-LOOP) and the program group has 9.
2. **Clock or measured conditions in `until` (`x06`, `x07`).** M3: "the conditions tasks already use". Tasks use no
   comparison in `pre` or `goal` (`E321`), and `measure` binds a name inside a task. The label reads M3 literally: a
   fact condition only (`object_at`, `robot_at`, `holding`, `in_state`, `region_clear`, ...), so a measured or clock
   `until` stays out. If values are allowed in `until`, the mission needs a reading of its own (a `measure` outside
   a task, which touches M8's "a robot never receives a mission"), and `x06`, `x07` become programs. A clock deadline
   ("until 22:00") is also not a duration in M2's list of bounds: is a time of day a bound?
3. **What a mission's parameters are.** The example has `mission keep_press_fed(robot r, location rack, location
   press)` and `run feed_press(r, next_blank, rack, press)`, where `next_blank` is not a parameter of the mission. The
   labels give each run its `entities` (world IDs) and assume that a mission takes parameters like a task and passes them
   on, and that an argument is a mission parameter or a literal entity. If an argument may be an undeclared name (the
   next blank), `x05` is a G-CHOOSE question first. How a mission gets a value from a world list ("a list declared in the
   world") has no task: a request to move every part of a list is `x05`, or `c04` when the parts are named.
4. **How the answer names a mission instead of a task.** The answer format has `task` and `bindings`. The attempts keep
   them (the task, as 1.9 has only tasks). For 2.0 the answer must name the mission and bind its parameters. `expected_mission`
   states entities per run and no parameter names, so that either form is scored; the choice (`task` that may name a mission,
   or a new `mission` field) changes `run.py`, the answer format, both prompts and the scorer, not the labels.
5. **Outcome without an `until`.** M5 says `completed` when the `until` holds. A repeat with only a bound that is
   reached (`m01`, `m04`, `m08`, `m10`) ends `bounded` in the labels: "the bound was reached". If a plain "do it 30 times"
   that was done should be `completed`, those paths change. A mission with no `repeat` and a `retry` (`m02`) ends
   `completed` when a run is verified; the labels read a `repeat` as optional there (an owner may require
   `repeat at most 1 runs`).
6. **Every run failed under `continue`.** `m09` path 3 (15 failed attempts) ends `bounded` with 0 verified runs. The
   alternative is `failed`. M5's `stopped` (operator stop, lease end) and an `until` that is unknown (M3 fails
   closed: which outcome?) have no task, since there is no operator in the simulator and a fact the world lacks
   is `i03`'s case.
7. **Do retries count against the run bound (`i04`)?** `i04` asks for at most 12 runs in all and 20 retries on top of the
   first run. The label reads that a retry is a run and counts against the bound, so the request contradicts itself and
   is infeasible. If the retries are on top of the bound, `i04` is a program with 12 runs and `retry 20`.
8. **When is a duration bound checked, and is a started run cut?** The labels read: before each run starts, and a run
   that has started always completes (a task run is complete and verified). So 10-minute runs in 3 hours are 18 runs
   (`m03`) and 15-minute runs in 2 hours are 8 (`m10`). Under this reading `i05` (a bound of 90 seconds, a round of 2
   minutes) is arguably a program with one run and `bounded`; the label says infeasible because the request cannot be
   met. If a run is cut at the bound, `m03` and `m10` have one more partial run, and `i05` is infeasible for sure.
9. **Time and state across runs.** The labels read one world for the whole mission: objects stay where a run left them,
   the simulated time continues, a world event at 25 minutes is first seen after the run that ends at or after it, and the
   mission's duration is that time (moves take no time, waits do). If every run restarts the world, `m03`, `m05`, `m06`,
   `m07` and `i02` change. The world has no field for a fault in run k: `run_faults` is a test form, to be decided with
   the simulator (L4).
10. **Where the line between a task and a mission is.** `c01` ("three times") and `c04` (three named totes) are controls: a
    model that writes a mission for them fails. 2.0 allows both, and a short repeat is a mission too. The set treats a
    fixed count of ten or fewer as a task, and the line (ten) is arbitrary. If the owner accepts both for such a request, the
    two controls drop the "no mission added" part of B5 and become programs that may take either form.
11. **A default failure policy.** `m10` states no policy, so the label is `stop`, the default of M4. A model that writes
    `continue` there has invented a policy, which B9 does not count. Count it in B9 or in `mission_ok` only?
12. **Time units in a bound.** Task durations have no hour unit (90 minutes is `90 min`); the example has `for at most 8
    h`. The labels write hours as `{"value": 3, "unit": "h"}` and the scorer compares as durations. Whether `h` (and `d`)
    is allowed in a mission bound is not decided. The attempts write `3 h`, as 0041 does.
13. **`x02` is a trigger with a bound.** "Every hour on the hour until 18:00" stays out by M6 (a `when` clause), as set 8's
    `x01` did. A run every 60 minutes after the last one (wait 60 min in the task, a duration bound) is a different request
    that 2.0 writes (`m10`). A model that rewrites the first as the second changed the request, which a label cannot
    tell: the paths do not.
14. **Atlas gaps.** The 22 tasks that 1.9 declines are in G-LOOP (20: all 10 programs, all 5 infeasible, `x01`, `x02`, `x04`,
    `x06`, `x07`), G-MSG (1: `x03`) and G-CHOOSE (1: `x05`). G-CLOCK is closed (L6 of I3), so `x02` and `x07`
    stay in G-LOOP with set 8's `x01` as the precedent; the owner may prefer a new gap for time triggers (I5).
