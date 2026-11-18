# Benchmark set 11: kinds (pre-registration draft)

**Status:** draft for the owner (2026-11-18). Decision [0044](../../spec/decisions/0044-more-measured-kinds.md) is approved
(K1-K9); this plan is its "Pre-registration (L3)" section made concrete: 24 tasks, labels for 2.1 (baseline) and 2.2
(acceptance), placeholder bounds B1-B9 and the open points below. `tests/test_bench_kinds.py`; run with `python bench/run.py
--set kinds`. Modelled on set 10, [bench-10-messages.md](bench-10-messages.md), and set 5 (the 1.6 measured values this set
extends, decision 0036). No model was run and no money was spent. **The labels follow 0044 K1-K9 as approved. Where 0044 does
not decide what a label needs, the label states the reading used and the question is listed under "Open points" below: an
owner answer that differs changes the labels named there.** The acceptance bounds are placeholders, to be fixed with the
baseline in hand (rule 6, [STRATEGY.md](../STRATEGY.md)). No language, spec, validator or simulator change: that is L4.

## How the set is used

- **Labels.** `expect` is the **2.1** label and is what the baseline scores: every non-control task expects `cannot /
  inexpressible`, because in 2.1 `fill`, `confidence`, `pressure` and `count` are no kind of `measure` (`E217`) and `%` is no
  token (`E101`). `expect_2_2` is the label for acceptance once 2.2 is taught (L4 moves it to `expect` and keeps the old one as
  `expect_2_1`, as set 10 did). Every check a 2.2 label asks for is a key **inside** `expect_2_2`, so that the 2.1 scorer
  does not read it: `expected_readings` (`[robot, kind, object]`), `expected_comparisons` (`{kind, op, value, unit}`; a count has
  no unit), and, on a program that combines a reading with an older feature, `expected_limits`, `expected_actions`,
  `expected_clock`, `expected_message_waits` (the forms of sets 6 to 10) and `expected_sensors` (the decoy, `k10`).
- **`world_2_2`.** The 1.7 additions to the world, in the label (as set 10's `world_2_1` is): the `measures` of each sensor
  the task reads, with unit and uncertainty (K5), and the default `properties` of the objects, with a value and a unit (a
  count has none). The world files themselves are the 2.1 ones: **world file 1.7 is L4's**. Under 2.1 the model sees the sensor
  and no `measures` for these kinds, and never sees a property value (as in 1.6).
- **Paths.** Each path of a program has a `when`, a `world_patch` with the 1.7 property values that force it, the goal it
  reaches (or `fails`), and the messages it sends. `world_patch` is read as in sets 5 and 10 (`lib._merge` into the world).
  A reading near the threshold is a path that `fails` (`S261`: an unknown `if` fails the run, 0044 K3 and 0036 V7);
  `k06` has it, and the test computes the three-valued comparison of every path with the sensor's uncertainty.
- **Baseline (now, 2.1 reference):** the 20 tasks that are not controls expect `cannot / inexpressible`. The obvious attempt of
  each (in `bench/reference_kinds/`) is the task with the new reading as 0044 shows it (or, for what stays out, the form a
  model would guess). 2.1 rejects it with `E101` (`%` is no token: the nine attempts that write `25 %`; the first diagnostic
  is the lexer's, `E217` follows), `E217` (10: the kind is not a kind) or `E505` (`x05`: there is no person to bind).
  Every attempt marks the reading lines with `// @reading`; without them it is a valid 2.1 program that runs the rest of the
  task and succeeds (the tests run all 20; `k09` needs the world to send its message, key `attempt_world_patch`). That
  program is the **reading replaced silently**: the reading dropped, as the old checks cannot tell. The 4 controls expect a
  program (references `.rob`, `.json`, valid and SUCCEEDED on every path, with their `expected_readings` taken).
- **Acceptance (after 2.2 is taught):** the 2.2 labels apply. A program is judged as in sets 5 and 7 to 10 (the goal and the
  messages of each path in a patched world) **and** by checks written in L4: `reading_ok` (the readings of
  `expected_readings`, each of the asked kind, object and sensor, no more and no fewer), `comparison_ok` (the comparisons of
  `expected_comparisons`: kind, operator, limit and unit, a conversion counting as the same), the older checks where the task
  has them, and **reading replaced silently** and **kind or threshold changed** (below). The 2.2 references are written in
  L4, not now.
- **Worlds:** `site` (existing) and **two new files**, `stockroom` (retail and warehouse stock: scanbots, shelves, bins,
  cartons, a forklift lane, a clock, a channel `stock`) and `plant_room` (inspection: crawlers, gauges, tanks, a relief
  valve), both world file 1.6 with sensors that measure only what 1.6 knows. The plan allows at most 3. No world in 2.1 shows
  a property value, and the test checks that the files and their description say nothing about the new kinds.
- **Not copied.** The R3 probes `f_perceive_shelf_fill` and `f_perceive_weed_confidence` stay in set "future" with their ids;
  this set has new ids and new prompts for the same shapes (as set 10 did). L4 relabels the probes in place. See open point 1:
  the shelf probe may not be a program.

## Tasks (24)

Attempt code: the first diagnostic of the obvious attempt under 2.1. 24 tasks: 10 + 5 + 5 + 4.

### Programs in 2.2 (10)

| id | world | Request | 2.1 label (attempt code) | 2.2 label: reading, checks and paths |
|---|---|---|---|---|
| `k01_restock_under_thirty_percent` | stockroom | With scanbot1, drive to aisle_2 and look at shelf_2 with cam_scan1. If it is less than 30 percent full, tell channel "lead" "restock shelf_2"; if it is not, tell no one. | cannot / inexpressible (`E101`) | program; reads `fill` of shelf_2 by scanbot1; compares `fill < 30 %`. Paths: (1) shelf_2 is 12 % full: below 30 %, so the lead is told; (2) shelf_2 is 70 % full: not below 30 %, nobody is told |
| `k02_pump_three_over_six_bar` | plant_room | With crawler1, drive to pump_3 and read gauge_p3 with cam_crawler1. If the pressure is above 6 bar, tell channel "maintenance" "pump 3 over pressure"; if it is not, tell channel "maintenance" "pump 3 ok". | cannot / inexpressible (`E217`) | program; reads `pressure` of gauge_p3 by crawler1; compares `pressure > 6 bar`. Paths: (1) gauge_p3 reads 7.2 bar: above 6 bar; (2) gauge_p3 reads 5.1 bar: not above 6 bar |
| `k03_shelf_three_out_of_stock` | stockroom | With scanbot1, drive to aisle_3, count the items on shelf_3 with tag_scan1 and, if there are none, tell channel "lead" "shelf_3 is out of stock". | cannot / inexpressible (`E217`) | program; reads `count` of shelf_3 by scanbot1; compares `count == 0`. Paths: (1) shelf_3 holds 0 items: out of stock; (2) shelf_3 holds 14 items: nobody is told |
| `k04_weed_and_ninety_percent_sure` | site | With weeder1, look at plant_7 with cam_weeder1. If it is labelled "weed" and the detector is at least 90 percent confident, tell channel "ops" "weed confirmed"; if not, tell channel "ops" "plant_7 skipped". | cannot / inexpressible (`E101`) | program; reads `label` of plant_7 by weeder1, `confidence` of plant_7 by weeder1; compares `label == weed`, `confidence >= 90 %`. Paths: (1) label weed, confidence 95 %: both hold, ops is told the weed is confirmed; (2) label weed, confidence 80 %: not sure enough, ops is told it is skipped; (3) label grass, confidence 95 %: not a weed, ops is told it is skipped |
| `k05_pump_one_below_450_kpa` | plant_room | With crawler2, drive to pump_1 and read gauge_p1 with cam_crawler2. If the pressure is below 450 kPa, tell channel "maintenance" "pump 1 low pressure"; if it is not, tell no one. | cannot / inexpressible (`E217`) | program; reads `pressure` of gauge_p1 by crawler2; compares `pressure < 450 kPa`. Paths: (1) gauge_p1 reads 4.2 bar = 420 kPa: below 450 kPa; (2) gauge_p1 reads 5.5 bar = 550 kPa: not below 450 kPa |
| `k06_tank_close_to_the_mark` | plant_room | With crawler2, drive to tank_farm and read how full tank_2 is with cam_crawler2. If it is more than 80 percent full, tell channel "maintenance" "tank_2 is nearly full"; if it is not, tell no one. | cannot / inexpressible (`E101`) | program; reads `fill` of tank_2 by crawler2; compares `fill > 80 %`. Paths: (1) tank_2 is 90 % full: above 80 %; (2) tank_2 is 50 % full: not above 80 %; (3) tank_2 reads 80.5 % +/- 2: the comparison with 80 % is unknown, so the run fails (S261) |
| `k07_around_the_forklift_lane` | stockroom | With scanbot1, drive from aisle_1 to aisle_3 without ever entering forklift_lane, look at shelf_3 with cam_scan1 and, if it is less than 25 percent full, tell channel "lead" "shelf_3 is low". | cannot / inexpressible (`E101`) | program; reads `fill` of shelf_3 by scanbot1; compares `fill < 25 %`. Limit: `no_go_region forklift_lane`. Paths: (1) shelf_3 is 15 % full: below 25 %; (2) shelf_3 is 60 % full: not below 25 % |
| `k08_open_the_relief_valve` | plant_room | With crawler1, drive to pump_3, read gauge_p3 with cam_crawler1 and, if the pressure is above 6 bar, open relief_valve_3; if it is not, leave the valve closed. | cannot / inexpressible (`E217`) | program; reads `pressure` of gauge_p3 by crawler1; compares `pressure > 6 bar`. Action: `do crawler1 open relief_valve_3`. Paths: (1) gauge_p3 reads 7 bar: above 6 bar, the valve is opened; (2) gauge_p3 reads 5 bar: not above 6 bar, the valve stays closed |
| `k09_nine_oclock_scan_on_the_signal` | stockroom | With scanbot1, wait up to 20 minutes for 09:00 and then up to 5 minutes for channel "stock" to send "SCAN NOW". Then count the items on shelf_1 with tag_scan1 and, if there are none, tell channel "lead" "shelf_1 is empty". If the time or the message does not come, stop the job as failed. | cannot / inexpressible (`E217`) | program; reads `count` of shelf_1 by scanbot1; compares `count == 0`. Clock: wait until 09:00, 20 min. Message: wait `stock` "SCAN NOW", 5 min. Paths: (1) SCAN NOW arrives at 100 s (counted when the wait is reached, S2); shelf_1 holds 0: lead told; (2) SCAN NOW arrives at 700 s; shelf_1 holds 9: nobody is told; (3) no message: the wait for SCAN NOW times out at 900 s (600 s + 5 min) |
| `k10_the_right_camera` | stockroom | With scanbot2, read how full shelf_3 is with its own camera and, if it is less than 10 percent full, tell channel "lead" "shelf_3 is nearly empty"; if it is not, tell no one. | cannot / inexpressible (`E101`) | program; reads `fill` of shelf_3 by scanbot2; compares `fill < 10 %`. Sensor: `cam_scan2`. Paths: (1) shelf_3 is 6 % full: below 10 %; (2) shelf_3 is 45 % full: not below 10 % |

### Infeasible in 2.2 (5)

| id | world | Request | 2.1 label (attempt code) | 2.2 label | Why |
|---|---|---|---|---|---|
| `i01_the_pickers_camera_reads_nothing` | stockroom | With picker_s, look at bin_1 with cam_picker and, if it is less than half full, tell channel "lead" "refill bin_1". | cannot / inexpressible (`E101`) | cannot / infeasible (S234) | Infeasible in 2.2: cam_picker, the only camera on picker_s, measures no kind (S234). `fill` is measured by the cameras of the two scanbots, which are not on picker_s. Under 2.1 the attempt fails earlier, with E217. |
| `i02_the_camera_is_on_the_other_robot` | stockroom | With scanbot1, read how full shelf_3 is with cam_scan2 and, if it is less than 10 percent full, tell channel "lead" "restock shelf_3". | cannot / inexpressible (`E101`) | cannot / infeasible (S231) | Infeasible in 2.2: cam_scan2 measures `fill` but is mounted on scanbot2, not on scanbot1 (S231), as in 1.6's `i03_probe_elsewhere`. Under 2.1 the attempt fails earlier, with E217. |
| `i03_the_gauge_camera_is_offline` | plant_room | With crawler3, read gauge_p1 with cam_crawler3 and, if the pressure is above 6 bar, tell channel "maintenance" "pump 1 over pressure". | cannot / inexpressible (`E217`) | cannot / infeasible (S232) | Infeasible in 2.2: cam_crawler3 measures `pressure` but is not available and the world never brings it online (S232), and the request names no other way to go on. Under 2.1 the attempt fails earlier, with E217. |
| `i04_gauge_nine_is_not_there` | plant_room | With crawler2, read gauge_p9 with cam_crawler2 and, if the pressure is above 6 bar, tell channel "maintenance" "pump 9 over pressure". | cannot / inexpressible (`E217`) | cannot / infeasible (S213) | Infeasible in 2.2: gauge_p9 is not present in the world (S213, S214): the sensor measures `pressure`, the robot carries it, and there is no object to read. Under 2.1 the attempt fails earlier, with E217. |
| `i05_count_then_carry_without_a_gripper` | stockroom | With scanbot1, drive to aisle_1, count the items on shelf_1 with tag_scan1 and, if there are none, carry carton_1 from the backroom to aisle_1. | cannot / inexpressible (`E217`) | cannot / infeasible (S2xx (no gripper)) | Infeasible in 2.2: the reading is possible (tag_scan1 measures `count` on scanbot1) but the request also needs a `grasp`, and scanbot1 has no gripper. A model must not drop the carry. Under 2.1 the attempt fails earlier, with E217. |

### Still inexpressible in 2.2 (5)

| id | world | Request | 2.1 label (attempt code) | 2.2 label | Why it stays out | Atlas gap |
|---|---|---|---|---|---|---|
| `x01_packaging_looks_worn` | stockroom | With scanbot1, look at shelf_1 with cam_scan1 and, if its packaging looks worn, tell channel "lead" "shelf_1 needs new packaging". | cannot / inexpressible (`E101`) | cannot / inexpressible (0044 K8) | A kind no sensor can declare (0044 K8): how worn something looks is not `fill`, `confidence`, `pressure` or `count`, and the world cannot name new kinds. A decline is the right answer; 2.2 does not add it. | G-PERCEIVE |
| `x02_own_battery_first` | plant_room | With crawler1, if its own battery is below 20 percent, drive to charging first; then drive to pump_2. | cannot / inexpressible (`E101`) | cannot / inexpressible (0044 K8) | The robot's own battery (0044 K8, G-SELF): a reading of the robot itself belongs to iteration I8, which will reuse `fill`. 2.2 reads objects with a sensor, not the robot that carries it. | G-SELF |
| `x03_carry_as_many_as_counted` | stockroom | With picker_s, count the items on bin_1 with cam_picker and carry that many cartons from the backroom to the checkout. | cannot / inexpressible (`E217`) | cannot / inexpressible (0044 K8) | Counting objects and acting on each of them (0044 K8, G-CHOOSE): the number of carry steps would come from the reading, and a plan has a fixed number of steps. 2.2 can compare a count, not repeat or choose by it. | G-CHOOSE |
| `x04_average_fill` | stockroom | With scanbot1, read how full shelf_1 and shelf_2 are with cam_scan1 and tell channel "lead" the average. | cannot / inexpressible (`E217`) | cannot / inexpressible (0044 K8) | Arithmetic on readings (0044 K8, G-CALC): an average of two readings has no representation, as in 1.6's `x01_average`. Reading both is possible in 2.2; computing is not. | G-CALC |
| `x05_nearest_person` | stockroom | With scanbot1, measure how far the nearest person is with cam_scan1 and, if it is less than 2 metres, tell channel "lead" "person close". | cannot / inexpressible (`E505`) | cannot / inexpressible (0044 K8) | A reading of a person (0044 K8, G-PROX): a person is not an object of the world, so there is nothing to bind and no sensor can declare a reading of one. Limits and reactions to people nearby are the gap G-PROX. | G-PROX |

### Controls (4)

| id | world | Request | Label (both versions) |
|---|---|---|---|
| `c01_carry_a_carton` | stockroom | With picker_s, carry carton_1 from the backroom to the checkout. | program, one task with four steps; no reading |
| `c02_probe_the_tank` | plant_room | With crawler1, take the temperature of tank_1 with probe_crawler1 and, if it is above 60 degC, tell channel "maintenance" "tank_1 is hot"; if it is not, tell no one. | program, a 1.6 `temperature` reading with `if`; paths: 70 degC -> message, 40 degC -> none; no new kind |
| `c03_weigh_a_carton` | stockroom | With picker_s, weigh carton_2 with scale_picker and tell channel "lead" the weight. | program, a 1.6 `mass` reading sent as a value to channel "lead"; no new kind |
| `c04_label_the_plant` | site | With weeder1, look at plant_7 with cam_weeder1. If it is labelled "weed", tell channel "ops" "weed"; if it is not, tell no one. | program, a 1.6 `label` reading with `==`; paths: weed -> message, grass -> none; **no `confidence` reading** |

### The 1.7 world notes (`world_2_2`)

| id | 2.1 world | Sensors (world file 1.7 `measures`) | Objects (world file 1.7 `properties`, the default value) |
|---|---|---|---|
| `i01_the_pickers_camera_reads_nothing` | stockroom | `cam_picker` measures nothing; `cam_scan1` fill in % +/- 2; `cam_scan2` fill in % +/- 2 | `bin_1` fill 20 % |
| `i02_the_camera_is_on_the_other_robot` | stockroom | `cam_scan1` fill in % +/- 2; `cam_scan2` fill in % +/- 2 | `shelf_3` fill 6 % |
| `i03_the_gauge_camera_is_offline` | plant_room | `cam_crawler3` not available, pressure in bar +/- 0.1 | `gauge_p1` pressure 7 bar |
| `i04_gauge_nine_is_not_there` | plant_room | `cam_crawler2` pressure in bar +/- 0.05 | - |
| `i05_count_then_carry_without_a_gripper` | stockroom | `tag_scan1` count | `shelf_1` count 0 |
| `k01_restock_under_thirty_percent` | stockroom | `cam_scan1` fill in % +/- 2 | `shelf_2` fill 12 % |
| `k02_pump_three_over_six_bar` | plant_room | `cam_crawler1` pressure in bar +/- 0.1 | `gauge_p3` pressure 7.2 bar |
| `k03_shelf_three_out_of_stock` | stockroom | `tag_scan1` count | `shelf_3` count 0 |
| `k04_weed_and_ninety_percent_sure` | site | `cam_weeder1` label, confidence in % +/- 1 | `plant_7` confidence 95 % |
| `k05_pump_one_below_450_kpa` | plant_room | `cam_crawler2` pressure in bar +/- 0.05 | `gauge_p1` pressure 4.2 bar |
| `k06_tank_close_to_the_mark` | plant_room | `cam_crawler2` fill in % +/- 2 | `tank_2` fill 90 % |
| `k07_around_the_forklift_lane` | stockroom | `cam_scan1` fill in % +/- 2 | `shelf_3` fill 15 % |
| `k08_open_the_relief_valve` | plant_room | `cam_crawler1` pressure in bar +/- 0.1 | `gauge_p3` pressure 7 bar |
| `k09_nine_oclock_scan_on_the_signal` | stockroom | `tag_scan1` count | `shelf_1` count 0 |
| `k10_the_right_camera` | stockroom | `cam_scan1` fill in % +/- 2; `tag_scan1` count; `cam_scan2` fill in % +/- 2; `tag_scan2` count | `shelf_3` fill 6 %, count 3 |

The infeasible tasks have the reading and no way to take it: `i01` the camera measures nothing (S234); `i02` the camera that
measures it is on another robot (S231); `i03` the camera is not available and never recovers (S232); `i04` the object is not
present (S213); `i05` the reading is possible and the carry is not (no gripper). `k10` has the two decoys.

## What this means for the standing tasks

| Task | Now (2.1) | 2.2 | Note |
|---|---|---|---|
| `u01_if_dirty`, `u02_if_damaged` (set "ask") | **`cannot / infeasible`** (erratum P2 now) | the same | K6: their worlds have no label sensor (S234), so under 1.6 the 1.3 reason is false. The old label is kept as `expect_1_3`; each has an attempt that fails with `S234` and a `fix_patch` (a label sensor and a label) that makes it succeed. They leave G-PERCEIVE (an infeasible task is in no gap) |
| `f_perceive_shelf_fill`, `f_perceive_weed_confidence` (R3 probes) | `cannot / inexpressible` | L4 relabels (programs, or infeasible: open point 1) | they stay in set "future" and in G-PERCEIVE until then |

## Acceptance bounds (proposed with the baseline in hand, rule 6; for the owner)

Baseline: [bench-set11-baseline.md](../evidence/bench-set11-baseline.md) (2026-11-18, 2.1, 3 repeats). Full prompt: no false
programs; grammar-only: `k04` written as a label check alone (2/30 for each model).

Baseline: not yet run (2.1, 3 repeats, full and grammar-only prompts). The smallest step is one answer (rule 6). The acceptance
run repeats the baseline under 2.2. A bound that the baseline makes pointless (for instance a rate that 2.1 cannot reach) is
replaced when the baseline is read, as sets 9 and 10 were.

| # | Group | Measure | Baseline (2.1) | Proposed bound (2.2, per model) |
|---|---|---|---|---|
| B1 | 10 `kind` programs | right decision, and every path in its patched world ends as the label says (goal, messages, or the `S261` failure), full prompt | not expressible | >= 90%: at least 27/30 |
| B2 | the same, grammar-only | the same | not expressible | for information |
| B3 | 5 inexpressible tasks | false programs, full and grammar-only (a valid program that succeeds: `x01` with the wear dropped, `x02` without the battery, `x03` with a fixed number of carries, `x04` sending one reading, `x05` with a stand-in) | 0/15 for each (placeholder) | at most 1/15 for each model and prompt |
| B4 | 5 infeasible tasks | decision `cannot`; no valid program that drops the reading and succeeds (`inspect`, a `label` stand-in, nothing) | 15/15 `cannot` for each (placeholder) | at least 14/15, and 0 that drop the reading and succeed |
| B5 | 4 `control` | right decision and **no new-kind reading added**, full prompt (`c04` is the test: no `confidence`) | 12/12 for each (placeholder) | at least 11/12 |
| B6 | sets 1 and 2 (standing) | `gate.json`; decision accuracy | latest run | the gate passes; `decision_correct` drops at most 2 points or one answer, whichever is larger |
| B7 | economy | statements, kinds, unit tokens, units, reserved words, world fields, codes | - | 0 statements, 4 kinds (`fill`, `confidence`, `pressure`, `count`), 1 unit token `%`, 3 pressure units, 0 new top-level declarations, 0 new world fields (the sensor `measures` and object `properties` exist), about 3 new codes (0044 "Economy"); the kind names as reserved words are open point 11 |
| B8 | 10 `kind` programs | **reading replaced silently**: a valid, succeeding 2.1-style program that uses `inspect`, a `label` stand-in, or nothing instead of the requested reading; full prompt | 0/30 for each (placeholder) | at most 1/30 |
| B9 | 10 `kind` programs | **kind or threshold changed**: a reading of another kind (`count` for `fill`, `label` where `confidence` is asked, a different sensor kind), a comparison whose operator, limit or unit differ after conversion (`kPa` against `bar` off by a factor, `>=` for `>`), `confidence` read without the `label` (`k04`), the limit moved to avoid an unknown (`k06`), the wrong sensor (`k10`); full prompt | not expressible | at most 1/30 |

If a bound fails, 2.2 stays in code only and is not taught.

## Cost of the baseline and acceptance runs (not spent)

`python bench/run.py --set kinds --repeats 3 --models claude-sonnet-5-5,claude-haiku-5-5`, effort medium, as the `Benchmark`
workflow runs it with `set_only: kinds` (full and grammar-only prompts, `--max-usd 3` each, not gated, never in the per-PR
or weekly runs):

| Prompt | Requests | Estimate |
|---|---:|---|
| Full | 90 per model | ~$3.78 (Sonnet ~$3.60, Haiku ~$0.18) |
| Grammar only | 90 per model | ~$3.63 (Sonnet ~$3.46, Haiku ~$0.17) |

The estimate is a guess (`bench/README.md`) and is above the $3 cap: the cap stops new requests, and a rerun with the same
`--run-id` resumes. The earlier sets cost well below their estimates. Nothing was sent.

## Open points: where 0044 does not decide what a label needs (owner, before L4)

Each point names the labels that depend on it. The label uses the reading given first.

1. **The R3 shelf probe may be infeasible, not a program (`f_perceive_shelf_fill`; K1).** "Scan shelf_a with camera_top ... less
   than a quarter full": in the `warehouse` world `shelf_a` is a **location**, and `measure` reads an **object**. No object
   there can hold a `fill` property, so with the world as it is the probe is `infeasible`, and L4 must either add a shelf
   object to its world copy (then a program) or relabel it infeasible. This set makes its shelves objects (`stockroom`).
2. **`==` on a count and its uncertainty (K3; `k03`, `k09`).** K3 says a count is exact, "its uncertainty is 0". The labels
   write a count sensor as `measures count` with no unit and no uncertainty, and a count property as a whole number with no
   unit. Open: may a world still give a count sensor an `uncertainty` (a counter that can miscount), and what then of `==`
   (unknown, as for any reading, or exact anyway)? Also: is `n < 2.5` or `n == 0.5` an error?
3. **How `%` is typed (K2; `k01`, `k04`, `k06`, `k07`, `k10`).** `%` is a unit of two kinds, `fill` and `confidence`, so the
   unit does not tell the kind. The labels read: the left name's kind decides, `f < 30 %` is fine for a `fill` name and for a
   `confidence` name, and `f < c` between a `fill` and a `confidence` name is a kind mismatch (`E319`). Also open: whether a
   bare number (`f < 0.3`, "a quarter") is `E319` (K2 rejects fractions) and whether the word `percent` is ever accepted (the
   requests say "percent"; the label wants `%`). A value outside 0 to 100, or a reading whose range crosses it (99 +/- 2), is
   not in any label.
4. **What "confidence belongs to a label" requires (K4; `k04`).** The label reads: the request is a `label` reading and a
   `confidence` reading of the same object by the same sensor. Open: (a) may `confidence` be read alone (a request "if the
   detector is at least 90 percent sure" with no label named)? `k04` has both, and no task has the alone form; (b) the sensor
   must measure both, or is a `label` sensor and a `confidence` sensor on one robot enough? I considered an infeasible task
   for (b) and did not write it, because 0044 does not say; (c) what an object with a `confidence` property and no `label`
   property is (`S236`).
5. **`S236` cannot be seen (K5; no task).** The world shown to the model never shows a property value (K5, as in 1.6), so "the
   kind is measured but not of that object" cannot be told from the world list. A request that reads a `fill` of an object
   that has none is a program whose run fails (`S236`, as `S261` does), not an infeasible task. The pre-registration listed it
   among the infeasible tasks; no task here has it. Owner: add a program with a failing path (`S236`) in L4, or leave it.
6. **No `psi` and no gauge or absolute pressure (K2; none).** K2 gives `Pa`, `kPa`, `bar`. A request in psi ("above 90 psi")
   is like 1.6's Fahrenheit: the model converts, or declines. No task has it; the labels would be program (convert, write
   `bar`). Absolute and gauge pressure are not distinguished.
7. **Live readings of the new kinds (K1; none).** 1.6 has `wait until temperature of pan using probe < 60 degC ...`. Do the
   new kinds also work in `wait until` ("wait until the tank is 80 percent full", "until the shelf is empty")? 0044 does not
   say; no task has it. The label of such a request would be a program if yes.
8. **`x03` may be a program (K8; `x03_carry_as_many_as_counted`).** The world has exactly two cartons, so "carry that many"
   can be unrolled: `if n >= 1 { carry carton_1 }`, `if n >= 2 { carry carton_2 }`. The label (`inexpressible`, G-CHOOSE)
   reads K8 literally (a number chosen at run time and an action on each), as set 10's `x05` read S1 before the owner's C14. A
   model that writes the unrolled form is not a false program by my reading. Owner: keep, or make it a program.
9. **`x05` and what a person is (K8; `x05_nearest_person`).** No world object is a person, so the attempt cannot bind one
   (`E505`) and the label is `inexpressible` (G-PROX). If a world declares a person as an object (class `person`), then
   `measure r distance of person_1 using lidar ...` would be a 1.6 program: out of this iteration, but the label depends on the
   world having none.
10. **The robot as the object of a reading (K8, K9; `x02`).** The label reads: a reading of the robot itself is out (G-SELF, I8).
    Open: is `measure r fill of r using ...` a validation error in 2.2 (which code), or accepted and left to I8? The attempt
    writes it and the 2.1 first diagnostic is `E101` (`%`).
11. **Are the kind names reserved words (B7)?** `fill`, `count`, `pressure` and `confidence` may already be identifiers in
    existing programs (a parameter named `count`, a binding `fill`). 1.6's four kinds are read by `measure ... KIND` only. If
    they stay contextual there is no cost; if they are reserved, an old program may break. No task uses them as names.
12. **The decoy of the pre-registration cannot be written as worded (`k10`).** "A sensor that measures `fill` of a different
    object" does not exist: sensors measure kinds, not objects. The decoy here is the near-miss sensor: `cam_scan1` (the right
    kind, on another robot: `S231`) and `tag_scan2` (this robot's, the wrong kind: `S234`). An owner who wants the object
    form needs `S236`, which is open point 5.
13. **`i04` and `i05` are infeasible for reasons older than 0044 (`i04_gauge_nine_is_not_there`,
    `i05_count_then_carry_without_a_gripper`).** The first has no object (S213, S214), the second no gripper. They test that
    a model does not drop the reading or the carry when it declines; they would be infeasible in 2.1 with `temperature`. Owner:
    keep as controls of the decision, or replace by tasks that depend on a new kind. A count written with a unit (pre-registration
    item) is `invalid`, not infeasible; it has no task, because a valid answer to a request that asks for it is a decline
    or a program without the unit, and the code of the error (K2 names none) is L4's.
14. **The code for each attempt in 2.1 is not always `E217` (all non-controls).** `%` is lexed before `measure` is read, so
    the percent kinds fail first with `E101`; `E217` follows. The labels do not depend on this; the table records the
    first diagnostic, the test requires `E217` among the codes of every other attempt. If L4 makes `%` a token, the 2.2 attempts
    of a model that writes `percent` fail differently.
15. **Infeasible in 2.1 or inexpressible (the five `i` tasks).** As in set 10, the 2.1 label is `cannot / inexpressible` for all
    of them: a 2.1 world cannot even say which sensor measures the kind. The 2.2 label is infeasible. A 2.1 model that declines
    as `infeasible` scores a wrong category (decision right), as set 10's did.
16. **The message and the 2.2 timing of `k09`.** "Counted when the wait is reached" (0043 S2) is assumed: a message at 100 s counts
    at 600 s, and the run ends at 600 s. The paths use no message at 0 s.
17. **Conversion by the model or by the validator (`k05`).** The label accepts `< 450 kPa` and `< 4.5 bar`, which are the same
    comparison after exact conversion (K2). The reading comes in bar, so a model that writes 450 against a bar *name* is
    unaffected: the validator converts. B9 counts a limit that is off by a factor (`450 bar`, `4.5 kPa`).
18. **Atlas gaps (G-PERCEIVE).** The 11 tasks 2.1 declines for want of a kind (the 10 programs and `x01`), the 5 infeasible
    ones and the two R3 probes are in G-PERCEIVE (18 in all; `u01` and `u02` left it by erratum P2). `x01` ("looks worn") is the
    free kind of K8 and fits the gap's name ("conditions on perceived properties"). Owner: keep it there, or add a far gap.
