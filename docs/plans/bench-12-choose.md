# Benchmark set 12: choose (pre-registration draft)

**Status:** **draft for the owner** (2026-11-18). Decision [0045](../../spec/decisions/0045-choosing-at-run-time.md) is approved
(C1-C12 as recommended); this plan is its "Pre-registration (L3)" section made concrete: 25 tasks, labels for 2.2 (baseline) and 2.3
(acceptance), placeholder bounds B1-B9 and the open points below. `tests/test_bench_choose.py`; run with `python bench/run.py
--set choose`. Modelled on set 11, [bench-11-kinds.md](bench-11-kinds.md), and set 10. No model was run and no money was spent.
**The labels follow 0045 C1-C12 as approved. Where 0045 does not decide what a label needs, the label states the reading used and
the question is listed under "Open points" below: an owner answer that differs changes the labels named there.** The acceptance
bounds are placeholders, to be fixed with the baseline in hand (rule 6, [STRATEGY.md](../STRATEGY.md); LE-11). No language, spec,
validator or simulator change: that is L4.

## How the set is used

- **Labels.** `expect` is the **2.2** label and is what the baseline scores: every non-control task expects `cannot /
  inexpressible`, because 2.2 has no `choose` (`E203`: unknown statement) and its worlds have no position to measure a distance
  from. `expect_2_3` is the label for acceptance once 2.3 is taught (L4 moves it to `expect` and keeps the old one as
  `expect_2_2`, as sets 10 and 11 did). The controls are programs in both versions (`expect` and `expect_2_2`).
- **`world_2_3`.** The world file 1.8 facts that a choice needs, in the label (as set 11's `world_2_2` is): a `position` `[x, y]` in
  metres for each location the task uses (C5), the readiness of the robots that differs from the 2.2 world, and which objects are at a
  place when the choice is made. The format of the field in a world file is L4's; the note is a map `location -> [x, y]` (the
  table "The 1.8 world notes" below). **The worlds are the 2.2 ones**: a model under 2.2 sees no position. Where a note changes a
  fact the 2.2 world shows (`i01`, `i03`, `i04`, `i05` make a robot not ready or move an object), the 2.2 world is as it is and the 2.3
  world copy of L4 carries the note, as set 11's 1.7 copies did.
- **`choices`.** What each choice is, as the label asks for it: `type` (`robot`, `object`, `location`), `from` (a list in the order
  the request writes it, `objects at L`, or `robots`, C2), `where` (`robot_ready` here) and `order` (`nearest_to` or
  `farthest_from` a reference point, C4). L4 writes the scorer that judges them; the tests of this set run a small model of C2-C5 over
  every path and check that the label says what the model says.
- **Paths.** A program has two or three paths. Each has a `world_patch` (a change to positions, readiness, a robot's place, an
  object's place or the clock; a `null` position removes it, C5), `chosen` (the entity each choice takes, `null` when no candidate is
  left and `else` runs), the goal it reaches and the messages it sends, or `fails`. **The goal names the entity chosen** (C7): the
  path where another candidate wins has another goal, so a program that fixed one candidate fails a path. A request that is
  indifferent (`p05`: "whichever part") has `goals`, one alternative per part that may be taken, never the one the rule would pick
  (erratum P7 of [2.2](../evidence/bench-errata-2-2.md): a goal label accepts exactly what the request says).
- **Missions (C10).** `expected_mission` has the bound the request states (a duration or a time of day, 0041: no count of runs
  is invented), `on_failure` (stated in the request: "stop the job as failed") and the paths of a mission model: `chosen` per run,
  `outcome`, `runs`, `runs_verified`. A part that has been moved is no longer a candidate; a run that finds none fails (`else`)
  and the mission stops.
- **Baseline (now, 2.2 reference):** the 21 tasks that are not controls expect `cannot / inexpressible`. The obvious attempt of
  each (in `bench/reference_choose/`) is the task as 0045 shows it, a `choose` statement; 2.2 rejects it with `E203` (the same
  code for all 21). Every attempt marks its choice line with `// @choice NAME=ENTITY`: with that line removed and `NAME` replaced by
  `ENTITY` (the first candidate the request or the world lists) it is a valid 2.2 program in the task's world that ends SUCCEEDED
  (the tests run all 21). That program is the **choice replaced silently**: one candidate fixed in advance, which the old checks cannot
  tell. For a mission the fixed part is gone after the first run, so the second run fails (`S215`): the silent form of a mission is the
  parts the world lists, named one by one (B8 below). The 4 controls expect a program (references `.rob`, `.json`, valid and SUCCEEDED
  on every path).
- **Acceptance (after 2.3 is taught):** the 2.3 labels apply. A program is judged as in sets 5 and 7 to 11 (the goal and the messages
  of each path in a patched world) **and** by checks written in L4: `choice_ok` (the choices of `choices`: type, candidates in the
  order written, `where`, order and reference point, no more and no fewer), the older checks where the task has them (`expected_limits`,
  `expected_clock`, `expected_mission`), and **choice replaced silently** and **criterion changed** (below). The 2.3 references are
  written in L4, not now.
- **Worlds:** `fulfilment`, `site`, `site_1_3` (region limits need world file 1.3 or later), `warehouse`, `stockroom`,
  `kinds_k10_the_right_camera` and `kitchen_values` (existing), and **two new files**, `bin_hall` (bins with a `fill`, crates, one
  picker, a camera that reads `fill` and `count`) and `parts_line` (a conveyor with four parts, a rework bench, a bin, three arms, a clock
  at 16:30), both world file 1.7 with no `position`. The plan allows at most 3. The test checks that no world file of the set has a
  position: they are notes until world file 1.8.
- **Not copied.** The R3 probes (`f_choose_nearest_tugger`, `f_choose_nearest_dock`, `f_choose_nearer_cart`) and the standing tasks
  (`x_nearest_robot`, `x05_every_new_part`, `x_shuttle_until_five`) stay in their sets with their ids; this set has new ids and new
  prompts for the same shapes, as set 11 did. L4 relabels them in place.

## Tasks (25)

Attempt code: the first diagnostic of the obvious attempt under 2.2. 25 tasks: 11 + 5 + 5 + 4.

### Programs in 2.3 (11)

| id | world | Request | 2.2 label (attempt code) | 2.3 label: choice and paths |
|---|---|---|---|---|
| `p01_nearer_tugger_to_the_trailer` | fulfilment | Send whichever of tugger_a and tugger_b is nearer to trailer_4 to trailer_4, then tell channel "ops" "tugger sent". | cannot / inexpressible (`E203`) | program; choice: `robot` from (tugger_a, tugger_b), nearest to `trailer_4`. Paths: (1) tugger_b waits at line_feed, about 28 m from trailer_4; tugger_a is 63 m away: **tugger_b**; (2) tugger_a is parked at trolley, 5 m from trailer_4: it is the nearer: **tugger_a** |
| `p02_nearest_dock_to_the_hall` | site | Move spot1 to whichever of dock_1, dock_2 and dock_3 is nearest to the hall. | cannot / inexpressible (`E203`) | program; choice: `location` from (dock_1, dock_2, dock_3), nearest to `hall`. Paths: (1) dock_2 is 8 m from the hall, dock_3 15 m, dock_1 22 m: **dock_2**; (2) dock_2 has no position: it is unknown and left out, so dock_3 is the nearest: **dock_3**; (3) dock_1 is moved to 1.4 m from the hall: **dock_1** |
| `p03_nearer_cart_for_tote_2` | fulfilment | With digit1, take tote_2 from inbound_conveyor to whichever of cart_3 and conveyor_b is closer to outbound_conveyor and place it there. | cannot / inexpressible (`E203`) | program; choice: `location` from (cart_3, conveyor_b), nearest to `outbound_conveyor`. Paths: (1) conveyor_b is 28 m from outbound_conveyor, cart_3 42 m: **conveyor_b**; (2) conveyor_b is moved to the far corner, 57 m away: cart_3 is closer: **cart_3** |
| `p04_ready_amr_nearer_to_shelf_a` | warehouse | Bring box1 from shelf_a to the handoff with whichever of amr1 and amr2 is ready and nearer to shelf_a. If neither is ready, stop the job as failed. | cannot / inexpressible (`E203`) | program; choice: `robot` from (amr1, amr2), where `robot_ready`, nearest to `shelf_a`. Paths: (1) amr2 is nearer but not ready (it is charging): the decoy; amr1 is ready: **amr1**; (2) amr2 is ready too and is the nearer: **amr2**; (3) neither is ready: no candidate is left and `else` runs: **none** -> the run fails |
| `p05_any_part_from_the_conveyor` | parts_line | With arm_a, move one part from conveyor_in to the bin: take whichever part is on the conveyor when the job starts. If the conveyor is empty, stop the job as failed. | cannot / inexpressible (`E203`) | program; choice: `object` from `objects at conveyor_in`. Paths: (1) four parts are on the conveyor: any of them: **p1**; (2) p1 was taken away: a part that is not there is not a candidate: **p2**; (3) the conveyor is empty: no candidate, `else` runs: **none** -> the run fails |
| `p06_closest_robot_to_the_lobby` | site | Send whichever robot is closest to the lobby to the lobby. | cannot / inexpressible (`E203`) | program; choice: `robot` from `robots`, nearest to `lobby`. Paths: (1) cleaner1 is at dock_2, 7 m from the lobby: **cleaner1**; (2) cleaner1 is at the headland: installer1 at module_pallet (41 m) is now the closest: **installer1** |
| `p07_equally_near_docks` | site | Move cleaner1 to whichever of dock_3 and dock_1 is nearer to the hall; if they are equally near, take the one I named first. | cannot / inexpressible (`E203`) | program; choice: `location` from (dock_3, dock_1), nearest to `hall`. Paths: (1) dock_1 and dock_3 are both 10 m from the hall: a tie goes to the first written, dock_3: **dock_3**; (2) dock_1 is moved to 5 m from the hall: it is the nearer: **dock_1** |
| `p08_farthest_spot_from_the_trailer` | fulfilment | With digit1, take case_1 from trailer_4 and store it at whichever of trolley, cart_3 and conveyor_b is farthest from trailer_4. | cannot / inexpressible (`E203`) | program; choice: `location` from (trolley, cart_3, conveyor_b), farthest from `trailer_4`. Paths: (1) cart_3 is 48 m from trailer_4, conveyor_b 30 m, trolley 5 m: **cart_3**; (2) cart_3 is moved 10 m from trailer_4: conveyor_b is now the farthest: **conveyor_b** |
| `p09_parts_for_six_minutes` | parts_line | With arm_a, move parts from conveyor_in to the bin one at a time, taking whichever part is on the conveyor when each one starts, and wait 2 minutes after each. Stop after 6 minutes. If the conveyor is empty when a part is due, stop the job as failed. | cannot / inexpressible (`E203`) | program; choice: `object` from `objects at conveyor_in`. Mission: bound for at most 6 min, `on failure { stop; }`. Paths: (1) four parts: runs at 0, 2 and 4 minutes, each moves a part that is still on the conveyor; 6 minutes end it: **p1, p2, p3** -> bounded, 3 runs; (2) two parts only: the third run finds the conveyor empty, `else` fails the run and the mission stops: **p1, p2, none** -> failed, 3 runs |
| `p10_dock_near_stop_c_around_the_wet_floor` | site_1_3 | Move printer1 to whichever of dock_1, dock_2 and dock_3 is nearest to stop_c, without ever entering wet_floor. | cannot / inexpressible (`E203`) | program; choice: `location` from (dock_1, dock_2, dock_3), nearest to `stop_c`. Limit: `no_go_region wet_floor`. Paths: (1) dock_2 is 28 m from stop_c, dock_3 37 m, dock_1 42 m: **dock_2**; (2) dock_3 is moved to 3 m from stop_c: it is the nearest: **dock_3** |
| `p11_parts_until_five` | parts_line | Keep moving parts from conveyor_in to the bin with arm_a, taking whichever part is on the conveyor each time and waiting 20 minutes after each, until 17:00. If the conveyor is empty when a part is due, stop the job as failed. | cannot / inexpressible (`E203`) | program; choice: `object` from `objects at conveyor_in`. Clock: `until clock_time >= 17:00`. Mission: bound until 17:00 only (no count), `on failure { stop; }`. Paths: (1) the clock starts at 16:30: run 1 ends 16:50, run 2 ends 17:10 and the until holds: **p1, p2** -> completed, 2 runs; (2) the clock starts at 16:55: run 1 ends 17:15 and the until holds: **p1** -> completed, 1 runs; (3) one part only: run 2 finds the conveyor empty, `else` fails the run and the mission stops: **p1, none** -> failed, 2 runs |

### Infeasible in 2.3 (5)

| id | world | Request | 2.2 label (attempt code) | 2.3 label | Why |
|---|---|---|---|---|---|
| `i01_no_tugger_is_ready` | fulfilment | Send whichever of tugger_a and tugger_b is ready and nearer to handoff_point to handoff_point. | cannot / inexpressible (`E203`) | cannot / infeasible (0045 C6) | Infeasible in 2.3: in the world of the job neither tugger is ready, so `where` leaves no candidate and `else` runs; the request names no fallback, so the job cannot be done. Under 2.2 the attempt fails earlier, at the `choose` (E203). |
| `i02_the_wall_has_no_position` | site | Move spot1 to whichever of dock_1, dock_2 and dock_3 is nearest to level_3_wall. | cannot / inexpressible (`E203`) | cannot / infeasible (0045 C5) | Infeasible in 2.3: the place the distance is measured from has no position in the world, so nothing is known about distance and every candidate is left out (0045 C5, fails closed). The request names no fallback. |
| `i03_the_ready_robot_has_no_gripper` | fulfilment | Use whichever of digit2 and tugger_a is ready and closer to inbound_conveyor to carry tote_1 from inbound_conveyor to outbound_conveyor. | cannot / inexpressible (`E203`) | cannot / infeasible (S211) | Infeasible in 2.3: digit2 is not ready, so the only candidate is tugger_a, which has no gripper (S211, as in 1.x). The request names no other way to carry the tote. |
| `i04_nothing_at_the_rework_bench` | parts_line | With arm_a, take whichever part is at rework to the bin. | cannot / inexpressible (`E203`) | cannot / infeasible (0045 C6) | Infeasible in 2.3: when the job starts no part is at the rework bench, so there is nothing to choose and the request names no fallback. |
| `i05_the_ready_scanbot_has_the_wrong_camera` | kinds_k10_the_right_camera | Have whichever of scanbot2 and scanbot1 is ready and nearer to aisle_2 read how full shelf_3 is with cam_scan2 and, if it is under 10 percent full, tell channel "lead" "restock shelf_3". | cannot / inexpressible (`E203`) | cannot / infeasible (S231) | Infeasible in 2.3: scanbot2 is not ready, so the robot is scanbot1, and cam_scan2 is mounted on scanbot2 (S231, as in set 11's `i02`). The sensor cannot follow the choice (dependent choices are out, 0045 C9). |

`i01`, `i02` and `i04` leave no candidate (`else` runs and the request names no fallback); `i03` and `i05` leave one that cannot do the job,
by a 1.x infeasibility that already exists (`S211`, no gripper; `S231`, the sensor is on another robot). The diagnostic for a choice that
leaves no candidate and has no fallback is new in 2.3 and is not decided (0045 C6 says only that `else` is mandatory): L4 picks the code (open
point 7).

### Still inexpressible in 2.3 (5)

| id | world | Request | 2.2 label (attempt code) | 2.3 label | Why it stays out | Atlas gap |
|---|---|---|---|---|---|---|
| `x01_fullest_bin_on_the_row` | bin_hall | With picker_h, take the fullest bin at bin_row to dock_b. Bins are swapped in and out all shift, so go by what is on the row when you start. | cannot / inexpressible (`E203`) | cannot / inexpressible (0045 C9 (choosing by a reading of each candidate)) | Choosing by a reading of each candidate (0045 C3, C9): `where` and `nearest to` use world facts, never a reading, because a reading is a sequence of robot actions, not a choice. G-EACH. With two or three named bins it is a 2.2 program (compare the names in an `if`, `c02`); here the set is whatever is on the row when the job starts. | G-EACH |
| `x02_every_pan_above_75` | kitchen_values | With chef_arm, take the temperature of every pan at the stove with probe_temp and put each pan above 75 degC in the sink. Pans come and go, so go by what is on the stove when you start. | cannot / inexpressible (`E203`) | cannot / inexpressible (0045 C9 (acting on each candidate in turn, with a reading of each)) | Acting on each member of a set chosen at run time, by a reading (0045 C9): the standing task `x03_any_hot_pan` in a new wording. A choice picks one entity; the pans to move are as many as the readings say. G-EACH. | G-EACH |
| `x03_carry_as_many_as_there_are_bins` | bin_hall | With picker_h, count the bins at bin_row with cam_h and carry that many crates from dock_a to dock_b. | cannot / inexpressible (`E203`) | cannot / inexpressible (0045 C9 (acting on each candidate: as many steps as the count)) | As many carries as a counted number (0045 C9): a plan has a fixed number of steps, and a mission needs a bound the request states. A fixed choice of one crate drops the count. G-EACH. | G-EACH |
| `x04_the_robot_nearest_the_caller` | warehouse | Send whichever robot is closest to the person who just called for help to that person. | cannot / inexpressible (`E203`) | cannot / inexpressible (0045 C9 (choosing a person: G-PROX)) | A person is no entity of the world and has no position in it, so the robot nearest to them cannot be chosen (0045 C9, G-PROX). A stand-in place for the person (the handoff) is the silent replacement. | G-PROX |
| `x05_an_object_the_list_does_not_know` | stockroom | With scanbot1, drive to aisle_2 and, if the camera sees an object there that the stock list does not know, tell channel "lead" "unknown object in aisle_2". | cannot / inexpressible (`E203`) | cannot / inexpressible (0045 C9 (detection of objects the world does not list)) | Detection of an object the world does not list (0045 C2, C9): `objects at L` yields only what the world declares, so an unlisted object is never a candidate. G-DETECT. The attempt either reports the listed shelf as 'unknown' or reports regardless. | G-DETECT |

### Controls (4)

| id | world | Request | Label (both versions) |
|---|---|---|---|
| `c01_carry_a_tote` | fulfilment | With digit1, take tote_1 from inbound_conveyor to outbound_conveyor. | program, one task with four steps; no choice |
| `c02_the_fuller_of_two_bins` | bin_hall | With picker_h, read how full bin_1 and bin_2 are with cam_h. If bin_1 is fuller, tell channel "lead" "bin_1 is fuller"; if not, tell channel "lead" "bin_2 is fuller". | program, two `fill` readings compared in an `if` (2.2 compares two names); paths: bin_1 fuller, bin_2 fuller, overlapping ranges -> `S261`; **no `choose`** |
| `c03_shuttle_p1_twelve_carries` | parts_line | With arm_a, carry p1 from conveyor_in to the bin and back again, 12 carries in all. | program, a mission of two run lines with `at most 12 runs`; **no `choose`** |
| `c04_carry_at_nine` | stockroom | With picker_s, wait until 09:00, for at most 20 minutes, and then carry carton_1 from the backroom to the checkout. If 09:00 does not come in time, stop the job as failed. | program, a 1.9 clock wait then a carry; paths: 08:50 start -> carried, 08:20 start -> the wait times out and the job fails; **no `choose`** |

### The 1.8 world notes (`world_2_3`)

| id | 2.2 world | Positions (metres), readiness and places that the 2.3 world has |
|---|---|---|
| `p01_nearer_tugger_to_the_trailer` | fulfilment | positions tugger_bay [0, 0], line_feed [40, 0], trailer_4 [60, 20], trolley [55, 18] |
| `p02_nearest_dock_to_the_hall` | site | positions hall [40, 10], dock_1 [50, 30], dock_2 [42, 18], dock_3 [55, 12] |
| `p03_nearer_cart_for_tote_2` | fulfilment | positions inbound_conveyor [10, 20], outbound_conveyor [50, 5], cart_3 [12, 22], conveyor_b [30, 25] |
| `p04_ready_amr_nearer_to_shelf_a` | warehouse | positions shelf_a [10, 10], charging [12, 8], dock [40, 10], handoff [60, 30] |
| `p05_any_part_from_the_conveyor` | parts_line | at conveyor_in: p1, p2, p3, p4 |
| `p06_closest_robot_to_the_lobby` | site | positions stop_a [0, 0], dock_2 [42, 18], gridline_a [5, 40], headland [80, 80], module_pallet [30, 50], lobby [45, 12] |
| `p07_equally_near_docks` | site | positions hall [40, 10], dock_1 [40, 20], dock_3 [50, 10] |
| `p08_farthest_spot_from_the_trailer` | fulfilment | positions trailer_4 [60, 20], trolley [55, 18], cart_3 [12, 22], conveyor_b [30, 25] |
| `p09_parts_for_six_minutes` | parts_line | at conveyor_in: p1, p2, p3, p4 |
| `p10_dock_near_stop_c_around_the_wet_floor` | site_1_3 | positions stop_c [20, 0], dock_1 [50, 30], dock_2 [42, 18], dock_3 [55, 12] |
| `p11_parts_until_five` | parts_line | at conveyor_in: p1, p2, p3, p4 |
| `i01_no_tugger_is_ready` | fulfilment | positions tugger_bay [0, 0], line_feed [40, 0], handoff_point [38, 6]; tugger_a: ready = false; tugger_b: ready = false |
| `i02_the_wall_has_no_position` | site | positions dock_1 [50, 30], dock_2 [42, 18], dock_3 [55, 12] |
| `i03_the_ready_robot_has_no_gripper` | fulfilment | positions staging [20, 10], tugger_bay [0, 0], inbound_conveyor [10, 20], outbound_conveyor [50, 5]; digit2: ready = false |
| `i04_nothing_at_the_rework_bench` | parts_line | p5 at station2; at rework: nothing |
| `i05_the_ready_scanbot_has_the_wrong_camera` | kinds_k10_the_right_camera | positions aisle_1 [0, 0], aisle_2 [10, 0], aisle_3 [20, 0]; scanbot2: ready = false |

Distances are straight lines (C5); the test squares them, so a tie (`p07`) is exact. A robot or an object is at the position of its
location. Where a path's patch moves a robot (`p01`, `p06`), the candidate is at its new place.

## What this means for the standing tasks

| Task | Now (2.2) | 2.3 | Note |
|---|---|---|---|
| `f_choose_nearest_tugger`, `f_choose_nearest_dock`, `f_choose_nearer_cart` (R3 probes) | `cannot / inexpressible` (`E304`) | L4 relabels (programs) | stay in set "future" and in G-CHOOSE until then |
| `x_nearest_robot` | `cannot / inexpressible` | program (`robots`, C2) | stays in G-CHOOSE until L4 |
| `x05_every_new_part`, `x_shuttle_until_five` | `cannot / inexpressible` | programs as missions (C10) | stay in G-CHOOSE until L4 |
| `x03_any_hot_pan`, `x03_carry_as_many_as_counted` | `cannot / inexpressible` | stay out (C9) | **move from G-CHOOSE to G-EACH** by erratum Q2 now; the old gap is kept in the task as `gap_before` |

## Acceptance bounds (PROPOSED, placeholders: to be fixed with the baseline in hand, rule 6)

Nothing was run: the baseline numbers are placeholders. The acceptance run repeats the baseline under 2.3.

| # | Group | Measure | Baseline (2.2) | Proposed bound (2.3, per model) |
|---|---|---|---|---|
| B1 | 11 `choose` programs | right decision, and every path in its patched world ends as the label says (the goal for the entity chosen, the messages, `fails`, or the mission outcome), full prompt | not expressible | >= 90%: at least 30/33 |
| B2 | the same, grammar-only | the same | not expressible | for information |
| B3 | 5 inexpressible tasks | false programs, full and grammar-only (a valid program that succeeds: `x01` with a fixed bin, `x02` with one pan, `x03` with one crate, `x04` with a stand-in place for the person, `x05` reporting regardless) | 0/15 for each (placeholder) | at most 1/15 for each model and prompt |
| B4 | 5 infeasible tasks | decision `cannot`; no valid program that fixes a candidate and succeeds | 15/15 `cannot` for each (placeholder) | at least 14/15, and 0 that fix a candidate and succeed |
| B5 | 4 `control` | right decision and **no `choose` added**, full prompt (`c02` is the test: the fuller of two named bins is an `if`) | 12/12 for each (placeholder) | at least 11/12 |
| B6 | sets 1 and 2 (standing) | `gate.json`; decision accuracy | latest run | the gate passes; `decision_correct` drops at most 2 points or one answer, whichever is larger |
| B7 | economy | statements, world fields, reserved words, top-level declarations, codes | - | 1 statement (`choose`), 1 world field (`position`), 0 new top-level declarations, about 4 new codes (0045 "Economy"); reserved words `choose`, `nearest`, `farthest` (open: recognised by position, as the kinds are?) |
| B8 | 11 `choose` programs | **choice replaced silently**: a valid, succeeding 2.2-style program that fixes one candidate, takes the first of the list with the order or the `where` dropped, or (a mission) names the parts the world lists; full prompt | 0/33 for each (placeholder) | at most 1/33 |
| B9 | 11 `choose` programs | **criterion changed**: another order (`nearest` for `farthest`, the list reordered so that a tie goes the other way), another reference point, a `where` dropped or invented, a bound or a stop rule that the request did not state; full prompt | 0/33 for each (placeholder) | at most 1/33 |

The tests show that a path tells each of these apart (`test_a_fixed_candidate_or_a_dropped_part_of_the_choice_gives_another_answer_on_some_path_b8_b9`):
for every program the first candidate, the other order, no order and no `where` each give another answer on at least one path. If a bound
fails, 2.3 stays in code only and is not taught.

## Cost of the baseline and acceptance runs (not spent)

`python bench/run.py --set choose --repeats 3 --models claude-sonnet-5-5,claude-haiku-5-5`, effort medium, as the `Benchmark` workflow
runs it with `set_only: choose` (full and grammar-only prompts, `--max-usd 3` each, not gated, never in the per-PR or weekly runs):

| Prompt | Requests | Estimate |
|---|---:|---|
| Full | 94 per model | ~$3.97 (Sonnet ~$3.78, Haiku ~$0.19) |
| Grammar only | 94 per model | ~$3.82 (Sonnet ~$3.64, Haiku ~$0.18) |

The estimate is a guess (`bench/README.md`) and is above the $3 cap: the cap stops new requests, and a rerun with the same `--run-id`
resumes. The earlier sets cost well below their estimates (set 11: $1.21 for ~$7.4). Nothing was sent.

## Open points: where 0045 does not decide what a label needs

Each point names the labels that depend on it. The label uses the reading given first; the owner decides before L4.

1. **Must an `else` end in `fail`? (C6; every program, `i01`, `i04`.)** 0045 says `else { ... }` is mandatory, as the timeout of `wait until` is.
   A `wait until` handler may recover and go on; a `choose` that recovers leaves its name unbound, so the next statement cannot use it.
   The labels read: every `else` ends in `fail` (the references and attempts all do), and a path with no candidate ends FAILED. Recommend: an
   `else` must end in `fail`, with its own code, as the last statement.
2. **A request that is indifferent among the candidates (C4; `p05`, `p09`, `p11`).** C4 takes the first candidate in the world's order, so
   a program is deterministic. But "whichever part is on the conveyor" does not ask for the first one. The labels accept **any** candidate
   in the goal (`goals`), and the second path takes the first part away so that a program that names one part fails. Recommend: the goal
   accepts any candidate; `choice_ok` checks the choice (type, source, no order), not which part came first.
3. **What ends a mission over `objects at L`, and a run that makes no progress (C10; `p09`, `p11`).** There is no condition "the
   conveyor is empty", and `until` takes the conditions of a `pre`. A run that finds nothing runs `else`, fails, and the default
   `on failure { stop; }` ends the mission as **failed** even if every part was moved. The two requests state this ("if the conveyor is
   empty when a part is due, stop the job as failed"), so `policy_stated` is true. Open: with `on failure { continue; }` a failing run uses no
   time, so a duration or a clock bound may never be reached (not tested here: the mission model of the tests stops at the first failed run). Recommend: keep the labels;
   L4's spec says that a mission whose run fails without advancing time and whose bound is a time stops (or `E3xx`).
4. **Time inside a mission (0041 M-bounds; `p09`, `p11`).** The runs of a choice move parts in zero time, so "for 6 minutes" is only
   meaningful with the waits the requests state (2 and 20 minutes). A request with no wait would make the duration bound unreachable. The
   labels read: the waits are part of the request. Recommend: keep (the same as set 9's `m03`).
5. **Is "the fuller of two named bins" in or out of C9? (C3, C9; `c02`, `x01`.)** C9 says choosing by a reading of each candidate is out, but
   2.2 already compares two readings in an `if` (language reference: "which of the two is warmer" is a program). With two or three named
   candidates the choice is a ladder of `if`s, so `c02` is a **control** (a program in both versions, a `choose` must not be added), and
   `x01` stays out only because its candidates are `objects at bin_row` (a run-time set) and the request says so. Recommend: confirm; state in
   the 2.3 reference that `choose` never reads a value and a finite named set compared by readings remains an `if`.
6. **`objects at L` takes every object at the place (C2; `p05`, `i04`, `x01`).** There is no class filter and `where` has no class condition,
   so in a world where a conveyor holds a part and a pallet, "take a part" would choose the pallet. The worlds of the set have only parts at
   the conveyor and only bins at the row, so no label depends on it. Recommend: tell L4's reference that `where` is how the program
   filters, and decide whether a `class` condition is wanted later.
7. **An unknown position of the reference point (C5; `i02`).** C5 says a candidate or `X` with no position is excluded. With `X`
   unknown every candidate is excluded, `else` runs, and the request names no fallback: the label is infeasible. Open: is this a refusal at
   planning time (a world fact, so `infeasible` with a code) or a run that fails? Recommend: infeasible with a code of its own, so that
   `delegate` (C8) refuses it before any robot moves.
8. **A choice after the first action (C8; `p03`, `p08`).** Both programs choose after `grasp`, the natural order for "take it to whichever
   place". C8 refuses `export-goal` and `delegate` for such a task. The labels are programs (the set judges authoring, not delegation), but the
   shape "pick, then choose the place" is common. Recommend: the 2.3 reference teaches that a choice that uses only world facts may come
   first (the choice is the same whether the robot has the tote yet or not) so the task can be delegated; or decide that C8 accepts such a
   choice.
9. **A person, an unlisted object and the boundary with `infeasible` (C9; `x04`, `x05`, `i04`).** "The person who just called" is G-PROX (a
   person is no entity); "an object the stock list does not know" is G-DETECT; "whichever part is at the rework bench" when the world lists none
   is `infeasible` (`i04`). The label separates them by what the request says it expects: an object the list may not know is G-DETECT; an
   object the list should have is infeasible. Recommend: keep.
10. **The code of each attempt is `E203`.** `choose` is not a word of 2.2 (it is an identifier), so the first diagnostic is the unknown
    statement, and the test requires it for all 21. If L4 reserves `choose`, `nearest` and `farthest` (B7, "open: recognised by
    position"), a 2.2 program that uses them as names would break; no task does.
11. **A mission's silent form is not a success (B8; `p09`, `p11`).** The strict reading of "the attempt without its choice succeeds"
    does not hold for a mission: a fixed part cannot be moved twice. The silent form that does succeed is the list of parts the world
    shows, named in `run` lines. B8 counts it; the tests do not build it (it would be a different program for each world).
