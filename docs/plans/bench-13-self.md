# Benchmark set 13: self (pre-registration draft)

**Status:** tasks, labels and open points 1–12 **APPROVED by the owner, 2026-11-25**; bounds are proposed with the baseline. Drafted 2026-11-24. Decision [0048](../../spec/decisions/0048-the-robots-own-state.md) is approved (S1-S11 as
recommended); this plan is its "Pre-registration (L3)" section made concrete: 24 tasks, labels for 2.3 (baseline) and 2.4 (acceptance),
placeholder bounds B1-B9 and the open points below. `tests/test_bench_self.py`; run with `python bench/run.py --set self`. Modelled on set
12, [bench-12-choose.md](bench-12-choose.md), and set 11. No model was run and no money was spent. **The labels follow 0048 S1-S11 as
approved. Where 0048 does not decide what a label needs, the label states the reading used and the question is listed under "Open points"
below: an owner answer that differs changes the labels named there.** The acceptance bounds are placeholders, to be fixed with the baseline
in hand (rule 6, [STRATEGY.md](../STRATEGY.md); LE-11). No language, spec, validator or simulator change: that is L4.

## How the set is used

- **Labels.** `expect` is the **2.3** label and is what the baseline scores: every non-control task expects `cannot / inexpressible`,
  because 2.3 has no `charge` kind (`E217`) and no `robot_charge` condition (`E204`). `expect_2_4` is the label for acceptance once 2.4 is
  taught (L4 moves it to `expect` and keeps the old one as `expect_2_3`, as sets 10 to 12 did). The controls are programs in both versions.
- **`world_2_4`.** The world file 1.9 facts that a task needs, in the label (as set 12's `world_2_3`): a robot's `charge` as
  `{"value", "unit": "%", "uncertainty"}` (S4) and the scheduled charge events `{"at", "robot", "charge"}` in a list `charge_events`. A robot with no
  `charge` has an unknown charge. **The worlds are the 2.3 ones**: a model under 2.3 sees no charge anywhere. The format of the field in a
  world file is L4's (open point 1); the note is a map `robot -> charge` and a list of events.
- **`expected_charge`.** What each charge check is, as the request states it: `use` (`if`, `report`, `wait`, `until` or `where`), the robot
  read, the operator, the threshold and unit, a `timeout` for a wait, and a `decoy`: another robot whose charge has the opposite state in the
  paths (a program that reads it is a different request, B9). L4 writes the scorer that judges them; the tests of this set run a small
  model of S2 to S7 over every path and check that the label says what the model says.
- **Paths.** A program has three to six paths. Each has a `world_patch` (the charges of the robot read and of the decoy, the scheduled
  events, and for `p09` a message and a `fill`), the goal it reaches (`goal` or, where the request is indifferent or a `do` leaves a state,
  `goals`: one alternative each, erratum P7 of [2.2](../evidence/bench-errata-2-2.md)), the messages it sends, or `fails`. A comparison
  is **true, false or unknown** (S2, 1.6 V7: the uncertainty interval of the reading against the threshold); an unknown comparison fails
  closed (`S261` in an `if`; a wait keeps looking until its timeout; an `until` ends the mission `failed`, S6). **Every program has a path
  with exactly the threshold** (no uncertainty: `<` and `>=` are told from `<=` and `>`) **and a path just beside it with an uncertainty
  that makes the comparison unknown** (a reading near the threshold that fails closed). Where the request names no end state (`p09`) the
  task is `goal_free` (erratum P7).
- **Missions (S6).** `p05` has `expected_mission` (the bound the request states, `at most 6 runs`; an `until robot_charge spot1 < 30 %` that is
  not a bound by itself, 0041 M2; `on failure` stop, not stated) and `mission_alternatives` for the second shape, `out` and `back` as two run
  lines with `at most 12 runs` (erratum Q9 of 2.3: a request that can be met by more than one program shape accepts each). The paths give
  the outcome and the runs of each shape; the `until` is checked after each run.
- **Baseline (now, 2.3 reference):** the 20 tasks that are not controls expect `cannot / inexpressible`. The obvious attempt of each (in
  `bench/reference_self/`) is the task as 0048 shows it: `measure r charge as c;` (`E217`, 14 attempts), or `robot_charge r` in a `wait until`,
  a mission `until` or a `choose ... where` (`E204`, 6 attempts). Every attempt marks the lines that read the robot's own state with `// @charge`
  (`// @charge => TEXT` for a line that is replaced, `// @charge who=a` for a choice): without them, and with the chosen name replaced by the
  first candidate, it is a valid 2.3 program that ends SUCCEEDED (the tests run all 20; `p09` needs the message of `attempt_world_patch`).
  That program is the **charge check replaced silently**: the same plan with the check dropped, which the old checks cannot tell (B8). In
  `i03` and `i05` the `do` that the robot cannot do (S006) is marked too: dropping the request's impossible part is the silent form. The 4
  controls expect a program (references `.rob`, `.json`, valid and SUCCEEDED on every path).
- **Acceptance (after 2.4 is taught):** the 2.4 labels apply. A program is judged as in sets 5 and 7 to 12 (the goal and the messages of each
  path in a patched world) **and** by checks written in L4: `charge_ok` (the checks of `expected_charge`: kind, robot, operator, threshold,
  place, no more and no fewer), the older checks where the task has them (`expected_limits`, `expected_actions`, `expected_readings`,
  `expected_message_waits`, `expected_mission`, `choices`), and **charge check replaced silently** and **threshold or robot changed**
  (below). The 2.4 references are written in L4, not now.
- **Worlds:** `site`, `site_1_3` (region limits need world file 1.3 or later) and **one new file**, `charge_hall` (world file 1.8: four robots,
  `spot1` with `charge`, `swap_battery` and `self_test` actions, `spot2` and `spot3` with `charge`, `carrier1` with none, two chargers
  and a swap bay with `location_states`, a bin with a `fill` and a camera that reads it, a channel `ops`, a clock at 21:30). The plan allows
  at most 3. No world file has a robot with a `charge`: they are notes until world file 1.9.
- **Not copied.** The R3 probes (`f_self_low_battery`, `f_self_enough_for_route`, `f_self_report_charge`) and the standing tasks
  (`x02_own_battery_first`, `x02_selftest_result`) stay in their sets with their ids; this set has new ids and new prompts for the same
  shapes, as sets 11 and 12 did. L4 relabels `f_self_*` and `x02_own_battery_first` in place (programs); `x02_selftest_result` moves to G-HEALTH
  now (erratum R2).

## Tasks (24)

Attempt code: the first diagnostic of the obvious attempt under 2.3. 24 tasks: 10 + 5 + 5 + 4.

### Programs in 2.4 (10)

| id | world | Request | 2.3 label (attempt code) | 2.4 label: check and paths |
|---|---|---|---|---|
| `p01_low_battery_then_dock` | site | With spot1, if your battery is below 25 percent, go to dock_2 and tell channel "lead" "spot1 is docked"; otherwise go on to stop_c. | cannot / inexpressible (`E217`) | program; an `if` on `spot1` < 25 %; decoy `cleaner1`. Paths: (1) spot1 has 18 % (+-1), below 25 %: it docks and the lead is told; cleaner1 has 90 %: ends as the goal says, tells `spot1 is docked` (2) spot1 has 64 % (+-1): it goes on to stop_c; cleaner1 (the decoy) has 12 %: ends as the goal says (3) spot1 has exactly 25 % (no uncertainty): not below 25 %, so it goes on (a `<=` docks): ends as the goal says (4) spot1 has 24 % +-2: the interval holds 25 %, the comparison is unknown and the step fails closed (S2): the job fails |
| `p02_enough_charge_for_the_round` | site | Before the round, check that spot1 has at least 60 percent charge. If it has, drive to stop_b and on to stop_c; if it has not, stop the job as failed. | cannot / inexpressible (`E217`) | program; an `if` on `spot1` >= 60 %; decoy `weeder1`. Paths: (1) spot1 has 80 % (+-1): the round is driven; weeder1 (the decoy) has 10 %: ends as the goal says (2) spot1 has exactly 60 %: at least 60 %, so the round is driven (a `>` fails): ends as the goal says (3) spot1 has 52 % (+-1): below 60 %, the job fails before anything moves; weeder1 has 95 %: the job fails (4) spot1 has 61 % +-3: the interval holds 60 %, the comparison is unknown and fails closed (a threshold of 55 would drive): the job fails |
| `p03_tell_the_lead_the_charge` | site | With spot1, go to dock_3 and tell channel "lead" how much charge you have left. | cannot / inexpressible (`E217`) | program; a report in a message on `spot1`; decoy `cleaner1`. Paths: (1) spot1 has 64 % (+-1): the lead is told `64 %`, value and unit (S5); cleaner1 has 31 %: ends as the goal says, tells `64 %` (2) spot1 has 9 % (+-1): the lead is told `9 %`; cleaner1 has 88 %: ends as the goal says, tells `9 %` (3) spot1 has no charge fact: unknown, the reading fails and nothing is sent (fails closed): the job fails |
| `p04_charge_until_eighty` | charge_hall | With spot2, drive to charger_1 and charge there until your battery is at least 80 percent, waiting up to 40 minutes. If it is not that high in time, stop the job as failed; otherwise drive on to stop_c. | cannot / inexpressible (`E204`) | program; a live `wait until` on `spot2` >= 80 %; decoy `spot3`. Declared action: `charge` on `charger_1`. Paths: (1) spot2 has 35 %, a charge event at 20 min makes it 90 %: the wait is met at 20 min: met at 20 min (2) spot2 has 35 %, the only event at 20 min makes it 78 %: 80 % is never reached, the wait times out at 40 min: times out, the job fails (3) spot2 has 35 %, the event that makes it 95 % comes at 50 min: too late, the wait times out at 40 min: times out, the job fails (4) spot2 already has 85 %: the wait is met at once; spot3 (the decoy) has 20 %: met at 0 min (5) spot2 has exactly 80 %: at least 80 %, the wait is met at once (a `>` times out): met at 0 min (6) spot2 has no charge fact and no event gives it one: unknown at every look, the wait fails closed: times out, the job fails |
| `p05_patrol_until_the_battery_is_low` | charge_hall | With spot1, patrol: drive to stop_b, wait 10 minutes there, drive back to stop_a, for at most 6 rounds. Stop as soon as your battery is below 30 percent. | cannot / inexpressible (`E204`) | program; a mission `until` on `spot1` < 30 %; decoy `spot2`. Paths: (1) spot1 has 80 % and nothing changes: no round ends below 30 %, the count ends it; spot2 (the decoy) has 20 %: bounded, 6 runs (two-line shape: 12) (2) spot1 has 80 %, a charge event at 25 min makes it 28 %: seen after the third round (30 min), the until holds: completed, 3 runs (two-line shape: 5) (3) spot1 starts with 27 %: the until holds after the first round: completed, 1 runs (two-line shape: 1) (4) spot1 starts with exactly 30 %: not below 30 %, nothing ends it but the count (a `<=` ends it after one round): bounded, 6 runs (two-line shape: 12) (5) spot1 has no charge fact: the until is unknown after the first round and the mission fails (S6): failed, 1 runs (two-line shape: 1) (6) spot1 has 31 % +-2: the interval holds 30 %, the until is unknown and the mission fails (S2, S6): failed, 1 runs (two-line shape: 1) |
| `p06_the_peer_with_enough_charge` | charge_hall | Send whichever of spot2 and spot3 has at least 50 percent charge to stop_c. If neither has, stop the job as failed. | cannot / inexpressible (`E204`) | program; a `choose ... where` on `candidate` >= 50 %. Paths: (1) spot2 has 40 %, spot3 70 %: only spot3 has enough (the first written does not): **spot3** (2) spot2 has 90 %, spot3 80 %: both have enough, the request is indifferent (any of them; the first written is taken): **spot2** (3) spot2 has 40 %, spot3 30 %: no candidate is left, `else` runs and the job fails: **none -> the job fails** (4) spot2 has exactly 50 %, spot3 45 %: at least 50 % is enough, spot2: **spot2** (5) spot2 has 52 % +-3: unknown, so it is left out (fails closed); spot3 has 80 %: **spot3** |
| `p07_charge_the_other_robot_if_low` | charge_hall | Spot1 and spot2 work together. Drive spot1 to stop_c. If spot2's battery is below 40 percent, also send spot2 to charger_2 and have it charge there; if it is not, leave spot2 where it is. | cannot / inexpressible (`E217`) | program; an `if` on `spot2` < 40 %; decoy `spot1`. Declared action: `charge` on `charger_2`. Paths: (1) spot2 has 30 % (+-1), spot1 (the decoy) has 80 %: spot2 charges at charger_2: ends as the goal says (2) spot2 has 85 % (+-1), spot1 has 20 %: spot2 stays at stop_b; the low charge is the decoy's: ends as the goal says (3) spot2 has exactly 40 %: not below 40 %, spot2 stays (a `<=` charges it): ends as the goal says (4) spot2 has 41 % +-2: the interval holds 40 %, unknown, fails closed; spot1 has 10 %: the job fails |
| `p08_dock_if_low_around_the_wet_floor` | site_1_3 | With cleaner1, never entering wet_floor: if your battery is at least 40 percent, drive to stop_c; if it is lower, drive to dock_1. | cannot / inexpressible (`E217`) | program; an `if` on `cleaner1` >= 40 %; decoy `weeder1`. Limit `no_go_region wet_floor`. Paths: (1) cleaner1 has 75 % (+-1): stop_c; weeder1 has 10 %: ends as the goal says (2) cleaner1 has exactly 40 %: at least 40 %, so stop_c (a `>` docks): ends as the goal says (3) cleaner1 has 22 % (+-1): dock_1; weeder1 has 90 %: ends as the goal says (4) cleaner1 has 41 % +-2: the interval holds 40 %, unknown, fails closed: the job fails |
| `p09_wait_for_go_then_look_if_charged` | charge_hall | With spot1, wait up to 15 minutes for channel "ops" to send "GO". Then, if your battery is at least 40 percent, read how full bin_1 is with cam_s1 and, if it is over 70 percent, tell channel "lead" "empty bin_1". If your battery is lower, tell channel "lead" "battery low" and stop the job as failed. If GO does not come in time, stop the job as failed. | cannot / inexpressible (`E217`) | program; an `if` on `spot1` >= 40 %; decoy `spot2`. Message wait 15 min; `fill` reading of bin_1. Goal-free (the request names no end state). Paths: (1) GO at 300 s, spot1 has 70 %, bin_1 is 85 % full: the lead is told: ends as the goal says, tells `empty bin_1` (2) GO at 300 s, spot1 has exactly 40 %, bin_1 is 30 % full: nobody is told: ends as the goal says (3) GO at 300 s, spot1 has 20 %: the lead is told `battery low` and the job fails; spot2 has 90 %: the job fails, tells `battery low` (4) GO at 300 s, spot1 has 41 % +-2: unknown, fails closed, nobody is told: the job fails (5) GO never comes: the wait times out at 15 min and the job fails; spot1 has 70 %: the job fails |
| `p10_swap_then_check_ninety` | charge_hall | With spot1, go to swap_bay and swap your battery there, wait 2 minutes, then check that your charge is at least 90 percent; if it is lower, stop the job as failed. Then drive to stop_c. | cannot / inexpressible (`E217`) | program; an `if` on `spot1` >= 90 %; decoy `spot2`. Declared action: `swap_battery` on `swap_bay`. Paths: (1) spot1 has 15 %; a charge event at 90 s (the new battery) makes it 98 %: the check at 120 s passes: ends as the goal says (2) a charge event at 90 s makes it 85 %: the check at 120 s fails: the job fails (3) the charge event of the new battery comes at 5 min: at 120 s spot1 still has 15 % and the check fails (a `wait for` of 10 minutes would pass): the job fails (4) a charge event at 90 s makes it exactly 90 %: at least 90 %, the check passes (a `>` fails): ends as the goal says (5) a charge event at 90 s makes it 91 % +-2: the interval holds 90 %, unknown, fails closed: the job fails |

### Infeasible in 2.4 (5)

| id | world | Request | 2.3 label (attempt code) | 2.4 label | Why |
|---|---|---|---|---|---|
| `i01_spot3_has_no_charge_reading` | charge_hall | Check that spot3 has at least 60 percent charge, then send it to stop_c. | cannot / inexpressible (`E217`) | cannot / infeasible (0048 S4 (spot3 has no charge fact: unknown, a reading fails closed, and the request names no fallback)) | Infeasible in 2.4: the world gives spot3 no charge, so every reading of it is unknown and fails closed (0048 S2, S4), and the request names no fallback. A program is a gamble that the check passes. Under 2.3 the attempt fails earlier, at the kind (`E217`). |
| `i02_the_charge_never_gets_to_ninety` | charge_hall | With spot2, charge at charger_1 until your battery is at least 90 percent, waiting up to 20 minutes, then drive to stop_c. | cannot / inexpressible (`E204`) | cannot / infeasible (0048 S4 (the only scheduled charge event brings spot2 to 70 %: the wait can never be met before its timeout)) | Infeasible in 2.4: the charge changes only by the scheduled events of the world, and the only one brings spot2 to 70 %, so `robot_charge r >= 90 %` is never true within the 20 minutes (as set 8's `i03_timeout_too_short`). The request names no fallback. Under 2.3 the attempt fails earlier, at `robot_charge` (`E204`). |
| `i03_the_carrier_cannot_charge` | charge_hall | With carrier1, charge at charger_1 until your battery is at least 80 percent, waiting up to 30 minutes, then drive to stop_c. | cannot / inexpressible (`E204`) | cannot / infeasible (S006 (carrier1 does not list the action `charge`)) | Infeasible in 2.4: carrier1 lists no `charge` action (S006, as in 1.8), so it cannot charge, whatever the scheduled events say. The fact is in the 2.3 world file too (`"actions": {}`; erratum Q3 of 2.3). Under 2.3 the attempt fails earlier, at `robot_charge` (`E204`). |
| `i04_nobody_has_seventy_percent` | charge_hall | Send whichever of spot2 and spot3 has at least 70 percent charge to stop_c. | cannot / inexpressible (`E204`) | cannot / infeasible (0048 S7 and 0045 C6 (no candidate is left: spot2 has 55 % and spot3 40 %, `else` runs and the request names no fallback)) | Infeasible in 2.4: in the world of the job neither robot has 70 %, so `where` leaves no candidate and `else` runs; the request names no fallback. Under 2.3 the attempt fails earlier, at `robot_charge` (`E204`). |
| `i05_spot2_cannot_swap_a_battery` | charge_hall | With spot2, go to swap_bay and swap your battery there, wait 2 minutes, then check that your charge is at least 90 percent; if it is lower, stop the job as failed. | cannot / inexpressible (`E217`) | cannot / infeasible (S006 (spot2 does not list the action `swap_battery`)) | Infeasible in 2.4: spot2 lists only `charge`, not `swap_battery` (S006), so it cannot swap its battery; only spot1 can. The fact is in the 2.3 world file too. Under 2.3 the attempt fails earlier, at the kind (`E217`) or at the action. |

`i01`, `i02` and `i04` rest on facts of the 1.9 world (no charge, a wait that no event meets, no candidate); `i03` and `i05` rest on a 1.8
fact that the 2.3 world file shows (`actions`): the request cannot be done in 2.3 either, and the 2.3 label stays `inexpressible` because the
charge cannot be read (open point 8). The diagnostic for the three 1.9 cases is not decided: L4 picks the codes (open point 3).

### Still inexpressible in 2.4 (5)

| id | world | Request | 2.3 label (attempt code) | 2.4 label | Why it stays out | Atlas gap |
|---|---|---|---|---|---|---|
| `x01_run_your_self_test_and_report` | charge_hall | With spot1, run your own self-test and tell channel "lead" the result. | cannot / inexpressible (`E217`) | cannot / inexpressible (0048 S10 (G-HEALTH: a self-test returns a value; an action returns none, 0040 A10)) | Health beyond `robot_ready` and `sensor_ok` (0048 S10, G-HEALTH): the self-test is a declared action that returns a value, and a `do` returns none. Doing the test and sending a literal text is the silent replacement. Under 2.3 the attempt fails at the kind (`E217`). | G-HEALTH |
| `x02_report_your_faults` | charge_hall | With spot2, drive to stop_b and tell channel "lead" which faults you have, if any. | cannot / inexpressible (`E217`) | cannot / inexpressible (0048 S10 (G-HEALTH: faults)) | Faults (0048 S10, G-HEALTH): a list of faults is not a charge and not `robot_ready`; the language has no kind for it. Reporting a literal 'no faults' is the silent replacement. Under 2.3 the attempt fails at the kind (`E217`). | G-HEALTH |
| `x03_minutes_until_empty` | charge_hall | With spot1, drive to dock_3 and tell channel "lead" how many minutes you can still run. | cannot / inexpressible (`E217`) | cannot / inexpressible (0048 S10 (G-PREDICT: time to empty is a prediction; the simulation has no drain model, S4)) | Time to empty (0048 S10, G-PREDICT): there is no drain model (S4), so a number of minutes cannot be computed; reporting the charge in % is the silent replacement. Under 2.3 the attempt fails at the kind (`E217`). | G-PREDICT |
| `x04_if_you_will_not_make_it_back` | charge_hall | With spot1, drive towards stop_c. If you will not have the charge to get back to dock_1 from there, drive back to dock_1 at once instead. | cannot / inexpressible (`E217`) | cannot / inexpressible (0048 S10 (G-PREDICT: energy for a route is a prediction; the request gives no threshold)) | Energy for a route (0048 S10, G-PREDICT): 'enough to get back' is a prediction of the charge a route costs, which a world without a drain model cannot give. A threshold the request does not state is a number the model invents. Under 2.3 the attempt fails at the kind (`E217`). | G-PREDICT |
| `x05_every_time_it_drops_below_twenty` | charge_hall | With spot1, patrol between stop_a and stop_b for 3 hours, 20 minutes at stop_b each round. The moment your battery drops below 20 percent, even in the middle of a round, go to charger_1 and charge, then carry on patrolling. | cannot / inexpressible (`E217`) | cannot / inexpressible (0048 S10 (G-TRIGGER: a self-state trigger fires at the moment; a mission starts a run only when the last one is done, 0041 M6)) | A self-state trigger (0048 S10, 0041 M6, G-TRIGGER): the charge is to be acted on at the moment it drops, even inside a round. A check at the start of each run only sees it at the next run, which is a different request. Under 2.3 the attempt fails at the kind (`E217`). | G-TRIGGER |

### Controls (4)

| id | world | Request | Label (both versions) |
|---|---|---|---|
| `c01_how_full_is_the_bin` | charge_hall | With spot1, drive to bay_1 and read how full bin_1 is with cam_s1 and, if it is under 30 percent, tell channel "lead" "restock bin_1"; if it is not, tell no one. | program, a 2.2 `fill` reading of an object, compared in an `if`; paths: 12 % full -> the lead is told, 70 % -> nobody; **no charge reading** |
| `c02_if_spot2_is_ready` | charge_hall | With spot2, if you are ready, drive to stop_c; if you are not, tell channel "lead" "spot2 is not ready" and stop the job as failed. | program, `robot_ready` in an `if`; paths: ready -> stop_c, not ready -> the lead is told and the job fails; **no charge reading** |
| `c03_charge_spot2_at_charger_2` | charge_hall | Send spot2 to charger_2 and have it charge there. | program, `do r charge charger_2` and the goal `in_state charger_2 "charging"`; **no charge reading** |
| `c04_patrol_twelve_rounds` | charge_hall | With spot1, patrol: drive to stop_b, wait 5 minutes there and drive back to stop_a, 12 rounds in all. | program, a mission with `at most 12 runs` (one run line, or two lines with `at most 24 runs`); **no `until`, no charge reading** |

### The 1.9 world notes (`world_2_4`)

| Task | Base facts (`charge`, `%`) and events | Per path |
|---|---|---|
| `p01`, `p02`, `p03` | site: spot1 64 +-1 (p01), 80 +-1 (p02), 64 +-1 (p03); the decoy cleaner1 (p01, p03) or weeder1 (p02) 90 / 95 / 31 | the charge of spot1 and of the decoy, as each path says |
| `p04` | charge_hall: spot2 35 +-1, spot3 95 +-1 (decoy); events `charge_events` of spot2 at 20 min (90 or 78) or 50 min (95) | `charge_events` replaces the list |
| `p05` | charge_hall: spot1 80 +-1, spot2 20 (decoy); an event of spot1 at 25 min to 28 | `charge_events` replaces the list |
| `p06` | charge_hall: spot2 90, spot3 80 | the two charges (spot2 or spot3 may have none) |
| `p07`, `p10` | charge_hall: spot1 80 / 15 and spot2 85 / 95; `p10` event of spot1 at 90 s to 98 | the charges and events |
| `p08` | site_1_3: cleaner1 75 +-1, decoy weeder1 90 | the charges |
| `p09` | charge_hall: spot1 70 +-1, spot2 10 (decoy); `bin_1` is 12 % full; a message GO on `ops` at 300 s (the patch changes it) | charges, `fill`, `messages` |
| `i01` to `i05`, `x01` to `x05` | the charges the `because` names (`i01`: spot3 has none; `i02`: one event to 70 %; `i04`: 55 and 40 %) | none |

Times: only a `wait for` moves the clock (the world has no positions, so a `move`, a `do` and a `grasp` take no time). An event is seen when
the clock reaches its `at`. Charge is in whole `%`; an `uncertainty` of `0` is an exact reading.

## What this means for the standing tasks

| Task | Now (2.3) | 2.4 | Note |
|---|---|---|---|
| `f_self_low_battery`, `f_self_enough_for_route`, `f_self_report_charge` (R3 probes) | `cannot / inexpressible` (`E217`, `E206`, `E317`) | L4 relabels (programs) | stay in set "future" and in G-SELF until then |
| `x02_own_battery_first` | `cannot / inexpressible` | program | stays in set "kinds" and in G-SELF until L4 |
| `x02_selftest_result` | `cannot / inexpressible` | stays out | **moves from G-SELF to G-HEALTH** by erratum R2 now; the old gap is kept in the task as `gap_before` |

## Acceptance bounds (APPROVED by the owner 2026-11-26, with the baseline in hand, rule 6)

Baseline: [bench-set13-baseline.md](../evidence/bench-set13-baseline.md). No false program in either prompt; the bounds are kept as proposed. Erratum R3 is approved.

| # | Group | Measure | Baseline (2.3) | Proposed bound (2.4, per model) |
|---|---|---|---|---|
| B1 | 10 charge programs | right decision, and every path in its patched world ends as the label says (the goal, the messages, `fails`, or the mission outcome), full prompt | not expressible | >= 90%: at least 27/30 |
| B2 | the same, grammar-only | the same | not expressible | for information |
| B3 | 5 inexpressible tasks | false programs, full and grammar-only (a valid program that succeeds: `x01` doing the test and sending a literal, `x02` sending "no faults", `x03` sending the charge in %, `x04` with an invented threshold, `x05` with a check at the start of each run) | 0/15 for each (placeholder) | at most 1/15 for each model and prompt |
| B4 | 5 infeasible tasks | decision `cannot`; no valid program that drops the check or the impossible part and succeeds | 15/15 `cannot` for each (placeholder) | at least 14/15, and 0 that drop the check and succeed |
| B5 | 4 `control` | right decision and **no charge reading added** (`c01` fills an object, `c02` asks `robot_ready`, `c03` charges with no reading, `c04` has no `until`), full prompt | 12/12 for each (placeholder) | at least 11/12 |
| B6 | sets 1 and 2 (standing) | `gate.json`; decision accuracy | latest run | the gate passes; `decision_correct` drops at most 2 points or one answer, whichever is larger |
| B7 | economy | statements, kinds, live forms, world fields, reserved words, codes | - | 0 statements, 1 kind (`charge`), 1 live form (`robot_charge`), 1 robot field and 1 world event, about 3 codes, 0 reserved words (0048 "Economy") |
| B8 | 10 charge programs | **charge check replaced silently**: a valid, succeeding 2.3-style program that drops the check, uses `robot_ready` as a stand-in, or guesses a `wait for` (for a wait, a mission or a swap); full prompt | 0/30 for each (placeholder) | at most 1/30 |
| B9 | 10 charge programs | **threshold or robot changed**: another threshold, operator or robot (the decoy), a condition the request did not state, a bound or a stop rule the request did not state; full prompt | 0/30 for each (placeholder) | at most 1/30 |

The tests show that a path tells each of these apart (`test_a_dropped_check_or_a_changed_threshold_operator_or_robot_gives_another_answer_on_some_path_b8_b9`):
for every program the constant "holds" and the constant "fails" (which is all a 2.3 program, `robot_ready` or a guessed `wait for` can say), a
threshold 5 or 10 points away, the strict operator for the other and the decoy's charge each give another answer on at least one path. The
2.3 world is closed and shows no charge, so no 2.3 program is right on every path (erratum Q4 of 2.3 applies to the labels: no task is
labelled inexpressible where a 2.3 program does it correctly). If a bound fails, 2.4 stays in code only and is not taught.

## Cost of the baseline and acceptance runs (not spent)

`python bench/run.py --set self --repeats 3 --models claude-sonnet-5-5,claude-haiku-5-5`, effort medium, as the `Benchmark` workflow runs
it with `set_only: self` (full and grammar-only prompts, `--max-usd 3` each, not gated, never in the per-PR or weekly runs):

| Prompt | Requests | Estimate |
|---|---:|---|
| Full | 90 per model | ~$3.86 (Sonnet ~$3.67, Haiku ~$0.18) |
| Grammar only | 90 per model | ~$3.71 (Sonnet ~$3.53, Haiku ~$0.18) |

The estimate is a guess (`bench/README.md`) and is above the $3 cap: the cap stops new requests, and a rerun with the same `--run-id`
resumes. The earlier sets cost well below their estimates (set 11: $1.21 for ~$7.4). Nothing was sent.

## Open points: where 0048 does not decide what a label needs (approved as recommended, 2026-11-25)

Each point names the labels that depend on it. The label uses the reading given first; the owner decides before L4.

1. **Where the charge events live in the world file, and what an event holds (S4; `p04`, `p05`, `p10`, `i02`, `i03`, `i05`).** S4 says
   `{ "at", "robot", "charge" }`, "as messages in 1.6". The labels use a top-level list `charge_events`, and an event's `charge` is the same
   object as a robot's (`value`, `unit`, `uncertainty`). Recommend: that, with `uncertainty` optional and absent meaning exact (the labels write `"0"`).
2. **What advances time (`p04`, `p05`, `p10`).** An event is seen when the clock reaches its `at` (0041 clarification 2). The labels
   assume that only a `wait for` moves the clock, so `move`, `do` and `grasp` take no time in a world with no positions. `p10` relies on it:
   the swap's event at 90 s is seen by the check after `wait for 2 min`. Recommend: keep, and say it in the 2.4 reference.
3. **Infeasible or a failing path? (S2, S4; `i01`, `i02`, `i04` and the `fails` paths of `p03`, `p04`, `p05`.)** A robot with no charge fact is
   the same world state in `i01` (label: infeasible) and in the third path of `p03` (label: the run fails closed). The difference the labels
   use: a path is a patched world that tests a program, a task is infeasible when the world of the job makes every program fail and the request
   names no fallback (as set 12's `i02` with no position, and set 8's `i03`, a timeout shorter than the wait). The same reading makes `i02` (no
   event reaches 90 % in 20 minutes) infeasible with a code of its own, not a timeout at run time. Recommend: keep, and give "unknown charge
   with no fallback" and "no event meets the wait" each a code decided in L4 (a planning-time check, so that `delegate` refuses it first).
4. **Reading a robot other than the one that works (S1; `p07`).** `measure b charge as c;` reads the charge of `b`, a robot of the task that
   is not the one that moves (as `measure r time` may name any robot of the task). The label of `p07` needs it ("if spot2's battery is below 40
   percent"). Recommend: allowed, the charge of any robot parameter of the task (the fleet manager reports it, S7).
5. **Unknown in a `choose ... where` (S2, S7; `p06`).** An unknown charge (no fact, or an interval that holds the threshold) leaves the
   candidate out, as a candidate with no position (0045 C5); if all are left out `else` runs. Recommend: keep (fails closed, no code of its own).
6. **A mission's `until` after each run line, and an unknown (S6; `p05`).** `until` is checked after each run line, so the two-line shape
   (`out`, `back`) can end after `out` and counts 5 runs where the one-line shape counts 3. An unknown `until` ends the mission `failed`
   (`S402`), and the run that came before it stays verified (`runs_verified` = `runs`). Recommend: keep.
7. **Is `do r charge charger` any use to the reading? (S4; `p04`, `i02`, `i03`.)** There is no charging model, so the `do` changes nothing
   in the charge: only the scheduled event does. A program without the `do` reaches the same readings; the `do` is checked by
   `expected_actions` and by the goal `in_state charger "charging"`. The goal of `p04` accepts the position alone or with the state.
   Recommend: keep, and say so in the reference: "wait for the event" is the 2.4 way to model charging.
8. **The 2.3 category of `i03` and `i05` (Q3 of 2.3).** Both are infeasible by `S006` under 2.3 as well (the robot lists no `charge` or
   `swap_battery`). The label for 2.3 is still `inexpressible`, the first reason (the charge cannot be read); a model that says `infeasible` is
   right in the decision and wrong in the category, as in set 12's `i01`. Recommend: keep.
9. **A threshold that touches the interval (S2; none of the paths).** The model treats an interval whose end is exactly the threshold as
   known (`26 +-1 < 25` is false, `24 +-1 < 25` is true), as 1.6 V7 does for objects. No path depends on it; the labels use exact readings and
   clear overlaps. Recommend: confirm in the 2.4 spec.
10. **`x05` and the boundary of G-TRIGGER (S10; `x05`).** "The moment your battery drops below 20 percent, even in the middle of a round" asks
    for an interrupt. A mission whose task checks the charge at its start sees the drop one run late, and a task that ends with `wait until
    robot_charge r < 20 %` is a different request. If the owner reads that second shape as an answer, `x05` is a program and G-TRIGGER
    loses a task. Recommend: keep it out (a trigger fires at the moment, 0041 M6).
11. **ACT-AUTO-003 and G-HEALTH (S10).** The atlas keeps `needs` as it is (set 12's rule: a far gap is claimed by no sub-activity yet).
    When G-SELF closes at L6, ACT-AUTO-003 (self-diagnosis and service requests) needs G-HEALTH instead. Recommend: move it at L6.
12. **The text of a sent charge (S5; `p03`).** `64 %`, with one space, as 2.2 sends `fill`. The labels write it so. Recommend: keep.
