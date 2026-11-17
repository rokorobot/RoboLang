# Benchmark set 8: clock readings (pre-registration draft)

**Status:** draft for the owner. Decision record: [0042](../../spec/decisions/0042-clock-readings.md) (APPROVED by the
owner, 2026-11-17). Directory: `bench/tasks_clock/`, `bench/reference_clock/`; tests `tests/test_bench_clock.py`; run
with `python bench/run.py --set clock`. Modelled on set 7, [bench-7-actions.md](bench-7-actions.md). No model was run
and no money was spent. **The labels follow 0042 T1–T12 as approved. Where 0042 does not decide what a label needs, the
label states the reading used and the question is listed under "Open points" below: an owner answer that differs
changes the labels named there.** The acceptance bounds are placeholders, to be fixed with the baseline in hand
(rule 6, [STRATEGY.md](../STRATEGY.md)); nothing here is frozen yet.

## How the set is used

- **Labels.** `expect` is the **1.8** label and is what the baseline scores. `expect_1_9` is the label for acceptance
  once 1.9 is taught (L4 moves it to `expect` and keeps the old one as `expect_1_8`, as set 7 did). The 1.9 label of a
  program task carries its `paths` (goal, messages and a `world_patch` whose `clock` forces the path) and a note
  `world_1_5` with the world 1.5 facts it rests on (a `clock`, or no clock).
- **Baseline (now, 1.8 reference):** the 20 tasks that are not controls expect `cannot / inexpressible`: 1.8 has no
  `time` or `elapsed` kind, no time literal and no `clock_time`. The kind is refused with `E217`, the literal with `E101`
  (the attempts are in `bench/reference_clock/`). A false program here is the **silent replacement**: a valid program
  that reaches a goal and writes a guessed `wait for`, a fixed message or nothing in place of the time. Every attempt is
  built so that without its clock part (the lines marked `// @clock`) it is a valid 1.8 program that ends SUCCEEDED (the
  tests run them), so no old check can tell. The 4 controls expect a program.
- **Acceptance (after 1.9 is taught):** the 1.9 labels apply. A program is judged as in set 7 (goal correct on every
  path, here with the clock forced by each path) **and** by a new check `clock_ok`, written in L4: the task reads the
  clock exactly as asked in `expected_clock`, no more and no fewer, and never on a control. The 1.9 references are
  written in L4, not now.
- **Worlds:** existing worlds (`warehouse`, `fulfilment`, `yard`, `site`, `site_1_3`, `lab`, `kitchen_values`,
  `ward_1_4`, `examples/world.json`), plus **one** new world file: `fry_station_at_fryer.json`, a copy of
  `fry_station.json` (world 1.2) with flippy1 already at the fryer, so that the wait of `k09` can be the first step (a
  contact probe reads only at its own location). No world file has a 1.5 field. The 1.5 facts a 1.9 label needs are in
  `expect_1_9.world_1_5` (`{"clock": {"start": "2026-11-17T21:30:00", "zone": "Europe/Prague"}}`, or `"clock": null` for
  a world with none), a note no code reads: L4 writes the 1.5 copies from them. Every `clock` is on 2026-11-17 in
  `Europe/Prague` (0042 T7).
- **`expected_clock`.** A top-level field on the 10 `clock` tasks, a list of `{reading: "time"|"elapsed", use:
  "message"|"if"|"wait", robot, channel | conditions | op, value, timeout}`, and `[]` on the 4 controls, so a model that
  adds a clock reading to a control fails it. Its form is a placeholder for L4's scorer; the paths decide whether a
  program that reads the clock differently (an `if` with the branches swapped) is right.
- **Not copied.** The three R3 probes (`f_clock_scan_stamp`, `f_clock_shift_cutoff`, `f_clock_clean_at_2300`) and the
  standing tasks `x_elapsed_time`, `x_report_duration`, `u17_at_1400`, `u18_every_morning`, `x05_cooling_time` and
  `x05_open_at_six` stay in their own sets with their ids; this set has new ids and new prompts for the same shapes. L4
  relabels them as programs there (0042 "What it makes expressible"); the bounds below use them for B8.

## Tasks (24)

Attempt code: the first diagnostic of the obvious attempt under 1.8.

### Clock requests that become programs in 1.9 (10)

| id | world | Request | 1.8 label (attempt) | 1.9 label | Tests |
|---|---|---|---|---|---|
| `k01_scan_stamp` | warehouse | With arm1, inspect box1 on shelf_a with camera_top, then send channel "lead" the time of day the scan was done. | cannot / inexpressible (`E217`) | program; `measure arm1 time` after the `inspect`, sent to "lead" (T1, T8); goal arm1 at shelf_a | a time stamp in a message |
| `k02_shift_cutoff` | fulfilment | With digit2, move tote_2 from inbound_conveyor to outbound_conveyor, but only if it is before 21:30. If it is 21:30 or later, move nothing, send "after cutoff" to channel "lead" from digit2 and stop the job as failed. | cannot / inexpressible (`E101`) | program; paths: start 21:05 → tote at outbound; start 21:50 → message and `fail` | a cutoff with `if` (T3) |
| `k03_wait_for_0530` | yard | At 05:30, send tug1 from gate_in to gate_out. Wait up to 90 minutes for 05:30 and, if it has not come by then, stop the job as failed. | cannot / inexpressible (`E101`) | program; `wait until clock_time >= 05:30 timeout 90 min else { fail }` then `move`; paths: start 04:30 → tug1 at gate_out; start 02:00 → the wait times out, the run stops | a bounded wait for a time of day (T5) |
| `k04_job_elapsed` | examples/world.json | With arm1, move box1 from the shelf to the table, then send channel "ops" how long the whole job took. | cannot / inexpressible (`E217`) | program; `measure arm1 elapsed` after the last step, sent to "ops" (T1, T8); the world has no `clock`, `elapsed` needs none (T7) | elapsed of the whole task |
| `k05_open_hours_window` | warehouse | With arm2, move part7 from the table to shelf_b, but only within the open hours, 08:00 to 18:00. Outside them, send "outside hours" to channel "ops" from arm2 and stop the job as failed. | cannot / inexpressible (`E101`) | program; `if now >= 08:00 and now < 18:00`; paths: 09:40 → part at shelf_b; 19:15 → stop; 07:10 → stop | a time window (`and`, T3) |
| `k06_two_robots_at_2000` | fulfilment | Once it is 20:00, have digit1 move tote_1 and digit2 move tote_2 from inbound_conveyor to outbound_conveyor. Wait up to 1 hour for 20:00 and, if it has not come by then, stop the job as failed. | cannot / inexpressible (`E101`) | program; one wait on `clock_time >= 20:00`, then both robots act; paths: start 19:30 → both totes at outbound; start 17:00 → stop | two robots, one time condition |
| `k07_time_and_open` | ward_1_4 | With moxi1, open closet_door and take iv_kit_1 from the supply closet to room_412, but only if it is before 18:00. If it is 18:00 or later, send "supplies closed" to channel "charge_nurse" from moxi1 and stop the job as failed. | cannot / inexpressible (`E101`) | program; `if now < 18:00` around `do moxi1 open closet_door` and the fetch; goal `in_state closet_door "open"` and kit at room_412; paths: start 16:30 → goal; start 18:20 → stop | a time condition plus a 1.8 `do` |
| `k08_time_and_no_go` | site_1_3 | With printer1, go to gridline_c and keep it out of wet_floor the whole way, but only if it is before 17:00. If it is 17:00 or later, send "too late" to channel "site" from printer1 and stop the job as failed. | cannot / inexpressible (`E101`) | program; `@safety no_go_region wet_floor` (1.7) and `if now < 17:00`; paths: start 15:40 → printer1 at gridline_c; start 17:25 → stop | a time condition plus a 1.7 region limit |
| `k09_heating_time` | fry_station_at_fryer | With flippy1, which is at the fryer, wait up to 2 minutes for the oil in oil_1 to read above 170 degrees Celsius. As soon as it does, send channel "fry" how long the job has taken so far, then move basket_2 from the prep_table to the fryer. If the oil does not get there, stop. | cannot / inexpressible (`E217`) | program; `wait until` the oil reads above 170 degC, then `measure flippy1 elapsed` and send it, then the fetch; paths: heats at 20 s → basket at the fryer; never → stop | the cooling time of `x05_cooling_time`: elapsed after a wait that is the first step |
| `k10_decoy_11_pm` | site | At 11 pm tonight, send cleaner1 from dock_2 to the lobby. Wait up to 2 hours for 11 pm and, if it has not come by then, stop the job as failed. | cannot / inexpressible (`E101`) | program; `clock_time >= 23:00` (T2: the author converts); paths: start 22:20 → cleaner1 at lobby; start 20:00 → the wait times out, the run stops | decoy: a 12-hour time; a literal `11:00` is true at the 22:20 start, so the robot moves at once with no error |

### Infeasible in 1.9 (5)

| id | world | Request | 1.8 label (attempt) | 1.9 label | Why |
|---|---|---|---|---|---|
| `i01_stamp_no_clock` | lab | With arm1, inspect vial1 with microscope, then send channel "log" the time of day the inspection was done. | cannot / inexpressible (`E217`) | cannot / infeasible (T7): the world has no `clock`, a `time` reading fails the step | no clock for a `time` reading |
| `i02_before_it_has_passed` | warehouse | With arm2, move part7 from the table to shelf_b before 08:00 today. | cannot / inexpressible (`E101`) | cannot / infeasible (T4): the clock starts at 09:15, 08:00 today has passed and the request has no else-path | the requested time has passed at the start |
| `i03_timeout_too_short` | yard | At 23:00, send tug1 from gate_in to gate_out. Wait up to 10 minutes for 23:00 and, if it has not come by then, stop the job as failed. | cannot / inexpressible (`E101`) | cannot / infeasible (T5): the clock starts at 21:30, 23:00 is 90 minutes away | the timeout is shorter than the wait |
| `i04_wait_for_time_no_clock` | examples/world.json | At 14:00, have arm1 move box1 from the shelf to the table. Wait up to 12 hours for 14:00 and, if it has not come by then, stop the job as failed. | cannot / inexpressible (`E101`) | cannot / infeasible (T7): the world has no `clock`, the live `clock_time` cannot be read | no clock for a wait on the time |
| `i05_window_already_closed` | warehouse | With arm2, move part7 from the table to shelf_b, but only between 06:00 and 08:00 today. | cannot / inexpressible (`E101`) | cannot / infeasible (T4): the clock starts at 09:15, the window has closed | a window wholly in the past |

Not here: `time` on the Gazebo backend (T9) is not a benchmark mode, so it has no task; a request in 12-hour form
that the author must convert is a **program** (`k10`), never infeasible.

### Still inexpressible in 1.9 (5)

| id | world | Request | 1.8 label (attempt) | 1.9 label | Why it stays out | Atlas gap |
|---|---|---|---|---|---|---|
| `x01_every_night_round` | site | Every night at 23:30, send cleaner1 from dock_2 to the lobby and back to dock_2. | cannot / inexpressible (`E101`) | cannot / inexpressible | repeating at a time: a mission trigger (0041 M6, T11) | G-LOOP |
| `x02_tomorrow_at_0100` | fulfilment | Tomorrow at 01:00, send tugger_a from tugger_bay to line_feed. | cannot / inexpressible (`E101`) | cannot / inexpressible | another day (T4): `>= 01:00` is true at once on the start day | G-CLOCK |
| `x03_time_between_scans` | warehouse | With arm1, inspect box1 on shelf_a with camera_top, then inspect box2 on shelf_b with camera_top, and send channel "lead" how much time passed between the two scans. | cannot / inexpressible (`E217`) | cannot / inexpressible | a duration between two readings: arithmetic (T11) | G-CALC |
| `x04_pickup_two_hours_later` | warehouse | With arm1, inspect box1 on shelf_a with camera_top, then send channel "lead" the time of day two hours after the scan finished, as the pickup time. | cannot / inexpressible (`E217`) | cannot / inexpressible | arithmetic on a time (T11) | G-CALC |
| `x05_on_monday` | pharmacy | On Monday at 09:00, send cart1 from dispatch to intake. | cannot / inexpressible (`E101`) | cannot / inexpressible | a date: a weekday is not a time of day (T4, T11) | G-CLOCK |

### Controls: programs in both versions (4)

| id | world | Request | Label (both versions) |
|---|---|---|---|
| `c01_plain_two_moves` | yard | Send tug1 from gate_in to gate_out and then to wash. | program, two `move`s; no clock reading |
| `c02_wait_30_seconds` | examples/world.json | With arm1, wait 30 seconds, then move box1 from the shelf to the table. | program, a relative `wait for 30 s` and the fetch; a model that turns it into a time of day or reads the clock fails |
| `c03_pan_temperature` | kitchen_values | With chef_arm, measure the temperature of pan_1 with its probe and send the reading to channel "pass". | program, a `measure` of a temperature and a value message (`expected_readings` names it); no clock reading |
| `c04_as_soon_as_possible` | site | As soon as possible, send cleaner1 from dock_2 to the lobby. | program, a plain move; "as soon as possible" asks for no duration or time of day |

## Acceptance bounds (proposed with the baseline in hand, rule 6; for the owner)

Baseline: [bench-set8-baseline.md](../evidence/bench-set8-baseline.md) (2026-11-17, 1.8, 3 repeats): no false
programs at all. The smallest step is one answer (rule 6). The acceptance run repeats the baseline under 1.9 with
world 1.5 copies.

| # | Group | Measure | Baseline (1.8) | Proposed bound (1.9, per model) |
|---|---|---|---|---|
| B1 | 10 `clock` programs | goal on every path **and** `clock_ok` (the right reading, comparison and time), full prompt | not expressible | ≥ 90%: at least 27/30 |
| B2 | the same, grammar-only | the same | not expressible | for information |
| B3 | inexpressible tasks | false programs, full and grammar-only | 0/15 for each | at most 1/15 for each model and prompt |
| B4 | infeasible tasks | decision `cannot`; no valid program that succeeds | 15/15 for each | at least 14/15, and 0 that succeed |
| B5 | 4 `control` | right decision and no clock reading added, full prompt | 12/12 for each | at least 11/12 |
| B6 | sets 1 and 2 (standing) | `gate.json`; decision accuracy | latest run | the gate passes; `decision_correct` drops at most 2 points or one answer, whichever is larger |
| B7 | economy | statements, kinds, literals, reserved words, world fields, codes | — | 0 statements, 2 kinds, 1 literal, 2 reserved words, 1 world field; about 4 user-facing codes plus their IR forms |
| B8 | 10 `clock` programs | **time replaced silently**: a valid, succeeding program that replaces the time condition or the wait with a guessed `wait for`, a fixed message, or nothing; full prompt | 0/30 for each | at most 1/30 |

If a bound fails, 1.9 stays in code only and is not taught.


## Cost of the baseline and acceptance runs (not spent)

`python bench/run.py --set clock --repeats 3 --models claude-sonnet-5-5,claude-haiku-5-5`, effort medium, as the
`Benchmark` workflow runs it with `clock: true` (full and grammar-only prompts, `--max-usd 3` each, not gated, never in
the per-PR or weekly runs):

| Prompt | Requests | Estimate |
|---|---:|---|
| Full | 90 per model | ~$3.63 (Sonnet ~$3.45, Haiku ~$0.17) |
| Grammar only | 90 per model | ~$3.51 (Sonnet ~$3.34, Haiku ~$0.17) |

The estimate is a guess (`bench/README.md`) and is a little above the $3 cap: the cap stops new requests, and a rerun with
the same `--run-id` resumes. The set 7 baseline cost well below its estimate, so the real cost is likely to be below the
cap. The workflow now has exactly 10 `workflow_dispatch` inputs, GitHub's limit: the next set needs one string input
`set_only` (values `set3`, `set4`, `future`, `regions`, `actions`, `clock`, ...) in place of the per-set booleans.

## Open points: where 0042 does not decide what a label needs (owner, before L4)

Each point names the labels that depend on it. The label uses the reading given first.

1. **Which robot reads the clock?** T1 names the robot in `measure r time as t;`, T5 names none in `wait until
   clock_time …`. The labels accept any robot in the program that has a `communicate`, or any robot for a wait
   (`expected_clock.robot` lists the acceptable ones, a list for `k06`). If a robot has its own clock, or a reading must
   come from the robot that acts next, `k01`, `k02`, `k05`, `k06`, `k07` and `k08` change.
2. **A time that has passed with no else-path (`i02`, `i05`).** The label says `infeasible` (the request cannot be done
   in this world), as set 7's `i05_leave_it_locked` did. The alternative is that a program `if now < 08:00 { … } else {
   fail }` is valid and its run ends in `fail`. Then both tasks become programs with a stop path, and the infeasible
   group loses two tasks. Same question as 0040 open point 7.
3. **A wait for a time that has passed.** `wait until clock_time >= 09:00` at 10:30 is true at once, so "at 09:00,
   do X" started at 10:30 runs X at once. The labels do not use it (no task starts after its time with a `>=` wait),
   but a model's answer will do it silently. Is that a program (T4 says the time means the start day), or must it be
   refused, as `i02`? If refused, a new infeasible task and a rule in 0042 are needed.
4. **"Midnight", "tonight" and the end of the day.** T4 allows `00:00` to `23:59` on the start day. "At midnight
   tonight" is the end of the day, which is the next day's 00:00: inexpressible by T4, or `23:59`? `k10` says "11 pm
   tonight" and starts at 22:20, so it avoids the question. No task covers midnight.
5. **Exactness of a wait for a time.** Whether `clock_time >= 05:30` ends the wait at exactly 05:30:00 or at the next
   look (the wait's polling interval, 0036 V4) decides the `elapsed` value of `k09` and the end time of `k03`/`k06`/
   `k10`. The paths record no exact end time and no exact elapsed value. L4 should say which.
6. **`elapsed` starts when?** 0042 T7 says `elapsed` is the simulated time. `k04` ("the whole job") reads it after the
   last step, `k09` right after the first step, a wait. If the preconditions or a pending stop take simulated time,
   they count. The labels assume time 0 is the start of step 1.
7. **`x05_cooling_time` cannot be the standing task as written.** 0042 lists it as expressible "when the wait is the
   first step". In `kitchen_values` the probe is a contact sensor and the pan is at the stove, so the arm must `move`
   first, and a `wait until` that is the first step never resolves (the reading is unknown). `elapsed` after the wait
   then includes the move. `k09` avoids this with a new world; L4 must either relabel `x05_cooling_time` as `elapsed
   since the start` (a wrong number for the request) or keep it inexpressible (a difference of two readings, G-CALC).
8. **T8 formats.** A `time` binding is sent as `HH:MM:SS`, an `elapsed` binding as `37 s`. A request for `HH:MM`, for
   minutes, or for "how many seconds" (without the unit) is formatting or conversion (G-CALC). No task asks for it;
   `x03` and `x04` are the arithmetic ones. If the owner counts `37 s` as an answer to "how many seconds", nothing
   changes, but a stricter reading would make `k04` and `k09` ask for no unit.
9. **The world text the model reads.** 1.9 labels need L4 to show the clock in the world text, and to say so when a
   world has none (`i01`, `i04`): an old world (1.1 to 1.4) has no `clock` field, so the model sees no difference
   between "no clock" and "clock not shown". L4 must print `no clock` (as 0040 C12 listed actions) or both tasks are
   unsolvable.
10. **`clock_elapsed` in `wait until` has no task.** T6 reserves two words, but `wait until clock_elapsed > 90 s` equals
    `wait for 90 s` and no request needs it. The set exercises `clock_time` in `wait until` and both `measure` kinds.
    Add a task, or accept that `clock_elapsed` is exercised by the language tests only.
11. **`==` and the E319 case have no task.** T3 forbids `==` and mixed kinds, but no natural request says either ("at
    exactly 14:00" is read as `>=`). An infeasible task for a time compared with an elapsed value was dropped as
    unnatural. If the owner wants a task for T3, say which request.
12. **Atlas gaps.** The 20 tasks 1.8 declines are in G-CLOCK (17: all 10 programs, all 5 infeasible, `x02` another day,
    `x05` a date), G-LOOP (1: `x01`, a mission trigger, T11/0041 M6) and G-CALC (2: `x03`, `x04`). L6 closes G-CLOCK, so
    the owner chooses whether `x02` (another day) and `x05` (a date) stay in G-CLOCK, which then cannot close, or move
    to a new gap (for example `G-DATE`, "dates and times on another day"), as set 6's and set 7's leftovers did. The
    tasks are in G-CLOCK for now.
13. **`x04` could also be an event.** "Two hours after the delivery arrives" (the 0042 pre-registration) is both an
    event and arithmetic. The task uses a time stamp from a scan (a known moment), so only the arithmetic is out. A
    request keyed on an arrival would be G-MSG.
14. **Time zones.** The zone is the world's (T2, T7). A request in another zone ("23:00 UTC") is T11 and has no task.
