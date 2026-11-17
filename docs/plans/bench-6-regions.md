# Benchmark set 6: region limits (pre-registration draft)

**Status:** draft for the owner. Written at step L3 of iteration I1 (language roadmap), before any code (LE-7, LE-11). **The labels follow the recommended option for P3–P10, which the owner has not yet confirmed:** a different answer changes the labels named in the open points below.
Decision record: [0039](../../spec/decisions/0039-region-limits.md) (status PROPOSED: the owner decided P1 and P2 on 2026-11-17; P3–P10 await review). Directory:
`bench/tasks_regions/`, `bench/reference_regions/`; tests `tests/test_bench_regions.py`;
run with `python bench/run.py --set regions`. Modelled on set 5, [bench-5-values.md](bench-5-values.md). No model was run
and no money was spent.

## How the set is used

- **Labels.** `expect` is the **1.6** label and is what the baseline scores. `expect_1_7` is the label for acceptance
  once 1.7 is taught (L4 moves it to `expect` and keeps the old one as `expect_1_6`, as set 5 did). The 1.7 label of a
  program task carries its `paths` (goal and world patch), as set 5's `expect_1_6`.
- **Baseline (now, 1.6 reference):** the 18 tasks that are not controls expect `cannot / inexpressible`: 1.6 has no
  region limit (`no_go_region` fails with `E211`, `in_region` with `E201`; the attempts are in `bench/reference_regions/`).
  A false program here is the **half program**: a valid program that reaches the goal and drops the limit. All 18 are
  valid and succeed once the limit line is removed (the tests run them). The 4 controls expect a program.
- **Acceptance (after 1.7 is taught):** the 1.7 labels apply. A program is judged as in set 5 (goal correct on every
  path) **and** by a new check `limit_ok`, written in L4: the task declares exactly the limits asked (kind, region ID,
  speed), no more and no fewer, and none on a control. The 1.7 references are written in L4, not now.
- **Worlds:** existing world 1.1 and 1.2 files only (`kitchen`, `kitchen_values`, `yard`, `site`, `pharmacy`,
  `cleanroom`, `warehouse`); no new world file and no 1.3 field in any file. The 1.3 facts a 1.7 label needs
  (`honours` on a robot, `contains` on a region) are recorded in `expect_1_7.world_1_3`, a note no code reads: L4 writes
  the 1.3 copies from them. A robot with no `honours` fails closed (0039 P6), so **every** 1.7 program needs its robots
  declared in the copy, and `spot1` is the one that is not.
- **Not copied.** The three R3 probes (`f_path_avoid_wet_floor`, `f_path_stay_in_zone`, `f_path_keep_off_road`) and
  `u09_avoid_quay`, `u10_inside_clean_room` stay in their own sets (`future`, `ask`) with new ids here. L4 relabels
  them as programs there (0039 pre-registration); the bound below uses them.

## Tasks (22)

### Region limits that become programs in 1.7 (9)

| id | world | Request | 1.6 label (attempt) | 1.7 label | Tests |
|---|---|---|---|---|---|
| `r01_runner_keep_off_hot` | kitchen | Send the runner to the sink, and keep it out of the hot zone the whole way. | cannot / inexpressible (`E211`) | program; goal runner at sink; `no_go_region hot_zone` | stay out |
| `r02_tug_stay_in_lane` | yard (1.1) | Drive tug1 from gate_in to gate_out, and keep it inside lane_1 the whole time. | cannot / inexpressible (`E201`) | program; goal tug1 at gate_out; `in_region lane_1` | stay inside |
| `r03_field_not_road` | site | Take weeder1 to row_14. It must stay inside field_zone and out of road_zone the whole way. | cannot / inexpressible (`E201`) | program; goal weeder1 at row_14; `in_region field_zone` and `no_go_region road_zone` | both limits |
| `r04_swap_carts` | pharmacy | Swap the carts: cart1 goes to intake and cart2 goes to dispatch. Neither may enter the corridor. | cannot / inexpressible (`E211`) | program; both robots at their targets; one `no_go_region corridor` | two robots, one limit (P2) |
| `r05_slow_and_dry` | site | Send cleaner1 from dock_2 to dock_3 at no more than 0.5 m/s, and keep it out of wet_floor the whole way. | cannot / inexpressible (`E211`) | program; goal cleaner1 at dock_3; `max_speed 0.5 m/s` and `no_go_region wet_floor` | with `max_speed` |
| `r06_field_clear_or_other_row` | site | If field_zone is clear when the job starts, drive weeder1 to row_14; if it is not, drive weeder1 to row_15 instead. Either way weeder1 must stay out of road_zone. | cannot / inexpressible (`E211`) | program; paths: clear → row 14, not clear → row 15; `no_go_region road_zone` | with an `if`; limit holds on both branches |
| `r07_carry_module_dry` | site | Have installer1 carry module_14 from the module pallet to the level 3 wall, without letting it, or the module, enter wet_floor. | cannot / inexpressible (`E211`) | program; goal module_14 at the wall; `no_go_region wet_floor` | carrying an object (P3) |
| `r08_decoy_door_zone2` | cleanroom | Send rover_1 to bench_1 and keep it out of door_zone2 on the way. | cannot / inexpressible (`E211`) | program; goal rover_1 at bench_1; `no_go_region door_zone2` (not `door_zone`) | decoy names |
| `r09_crate_around_aisle_and_cell` | warehouse | Have amr1 carry crate1 from the dock to the handoff, keeping out of both the aisle and the cell. | cannot / inexpressible (`E211`) | program; goal crate1 at handoff; two `no_go_region` (aisle, cell) | two regions, carrying |

### Infeasible in 1.7 (4)

| id | world | Request | 1.6 label (attempt) | 1.7 label | Why |
|---|---|---|---|---|---|
| `i01_hall_is_wet` | site | Send cleaner1 to the hall, and keep it out of wet_floor the whole way. | cannot / inexpressible (`E211`) | cannot / infeasible | 1.3 world: `wet_floor` contains `hall`; target inside the no-go region (P5) |
| `i02_hall_is_outside` | site | Move printer1 from gridline_a to the hall and keep it inside layout_zone the whole way. | cannot / inexpressible (`E201`) | cannot / infeasible | 1.3 world: `layout_zone` contains gridline_a, gridline_c, level_3_wall; the hall is outside (P5) |
| `i03_spot_cannot_honour` | site | Walk spot1 from stop_a to stop_d, and keep it out of road_zone the whole way. | cannot / inexpressible (`E211`) | cannot / infeasible | 1.3 world: `spot1` declares no `honours` (P6) |
| `i04_delegated_limit` | site | Hand this job to the delegated Core executor rather than running it here: send cleaner1 from dock_2 to dock_1, keeping it out of wet_floor. | cannot / inexpressible (`E211`) | cannot / infeasible | Core 0.2 carries no region limits; `delegate` refuses the task (P8) |

### Still inexpressible in 1.7 (5)

| id | world | Request | 1.6 label (attempt) | 1.7 label | Why it stays out | Atlas gap |
|---|---|---|---|---|---|---|
| `x01_first_leg_only` | site | Send cleaner1 to the hall and then on to dock_3. On the way to the hall it must stay out of wet_floor; on the way to dock_3 it may cross it. | cannot / inexpressible (`E202`) | cannot / inexpressible | per-step limit (P2, P9); a whole-task limit forbids the second leg too | G-PATH |
| `x02_arm_over_wet` | site | Have installer1 put module_14 on the level 3 wall. Its arm must never swing over wet_floor while it works. | cannot / inexpressible (`E211`) | cannot / inexpressible | arm workspace (P3, P9); a limit on the robot says nothing about the arm | G-PATH |
| `x03_person_distance` | pharmacy | Bring vial_b1 from fridge_1 to packing with picker1, keeping at least 1.5 m away from any person on the way. | cannot / inexpressible (`E207`) | cannot / inexpressible | a zone that moves with a person (P9, P10) | G-PROX |
| `x04_grip_ten_newtons` | kitchen | Have the helper arm carry cup_1 from the pantry to the pass, gripping it with no more than 10 N of force. | cannot / inexpressible (`E207`) | cannot / inexpressible | a force limit (P9, P10) | G-FORCE |
| `x05_avoid_busy_areas` | kitchen | Send the runner to the sink, going around whichever areas are busy at the time. | cannot / inexpressible (`E211`) | cannot / inexpressible | no region is named; which areas are busy is known only at run time (P4) | G-CHOOSE |

### Controls: programs in both versions (4)

| id | world | Request | Label (both versions) |
|---|---|---|---|
| `c01_wait_hot_zone_clear` | kitchen_values | With the chef arm, take bowl_2 from the fridge to the pass, but wait for hot_zone to clear first. Give it 60 seconds, and if it has not cleared by then, do nothing more. | program, `wait until region_clear`: a wait, not a limit; paths: clears at 30 s → bowl at pass; never clears → the run stops |
| `c02_plain_move` | site | Send weeder1 to row_14. | program, a plain move |
| `c03_layout_zone_clear_or_stop` | site | Send printer1 to gridline_c if layout_zone is clear when the job starts. If it is not, send "layout zone busy" to channel "site" from printer1 and stop the job as failed. | program, `if region_clear`; paths: clear → goal; not clear → message and `fail` |
| `c04_speed_only` | warehouse | Have amr1 carry crate1 from the dock to the handoff at no more than 0.5 m/s. | program, `max_speed` alone: no region limit is wanted |

## Proposed acceptance bounds (placeholders: fixed with the baseline in hand, rule 6)

Style of 0036 V13. Every `[ ]` is filled after the baseline run on 1.6 and before L4 starts; none is final.

| # | Group | Measure | Proposed bound |
|---|---|---|---|
| B1 | 9 `region` programs + the 5 standing tasks (3 R3 probes, `u09`, `u10`), relabelled as programs | goal correct on every path **and** `limit_ok`, Sonnet and Haiku, full prompt | ≥ [90]% each (0039: the three probes goal correct for both models) |
| B2 | the same, grammar-only prompt | the same | information only, not gated |
| B3 | 5 `inexpressible` | false programs | ≤ [baseline] and ≤ [5]% (full prompt); grammar-only ≤ [baseline] |
| B4 | 4 `infeasible` | decision `cannot` (any category) | ≥ [90]% (the 1.6 baseline is the floor); a program that declares the limit anyway is a false program |
| B5 | 4 `controls` | decision correct, no region limit declared | ≥ [100]%; `limit_ok` fails a program that adds a limit |
| B6 | sets 1 and 2 (standing) | `gate.json` passes; decision accuracy and false programs no worse (0039, strategy rule 2) | `decision_correct` drops no more than [2] points or one answer, whichever is larger |
| B7 | economy | 0 new keywords, 2 annotations, 1 world field, 1 profile field, about 4 new codes (0039) | as stated |

If a bound fails, 1.7 stays in code only and is not taught (0036 V13).

## Cost of the baseline and acceptance runs (not spent)

`python bench/run.py --set regions --repeats 3 --models claude-sonnet-5-5,claude-haiku-5-5`, effort medium, as the
`Benchmark` workflow runs it with `regions: true` (full and grammar-only prompts, `--max-usd 3` each, not gated, never in
the per-PR or weekly runs):

| Prompt | Requests | Estimate |
|---|---:|---:|
| Full | 83 per model | ~$3.29 (Sonnet ~$3.14, Haiku ~$0.16) |
| Grammar only | 83 per model | ~$3.22 (Sonnet ~$3.06, Haiku ~$0.15) |

The estimate is a guess (`bench/README.md`) and is a little above the $3 cap: the cap stops new requests, and a rerun
with the same `--run-id` resumes.

## Open points for the owner (0039 was silent or ambiguous; settle before L4)

Labels by decision they rest on: P5 `i01`, `i02`; P6 `i03`; P8 `i04`; P3 `r07`, `x02`; P4 `r03`, `r09`, `x05`; P9 and P10
`x03`, `x04`; P2 (decided) `r04`, `r06`, `x01`.

1. **Version of the world file.** 0039 P5/P6 need world 1.3. Is a 1.2 world a world whose robots honour nothing (every
   limit task infeasible by P6), or is the limit checked only against a 1.3 world? The labels assume 1.3 copies exist.
2. **Delegation (P8).** The benchmark has no delegated mode, so `i04` says "hand this job to the delegated Core
   executor" in the request. Is that right, and is the label `infeasible` (the task is valid, the route refuses it) or
   a new category? L4 needs to know what to score.
3. **`x05` (no region declared).** Declined as `inexpressible` and put in G-CHOOSE (the regions to avoid are chosen at
   run time). It could be G-PERCEIVE or G-PROX, or `infeasible` (the world has no such region). Needs a ruling.
4. **Gap membership after L4.** `x01` and `x02` stay in G-PATH, and so do all the 1.7 programs until they are
   relabelled. When 1.7 ships, G-PATH is partly closed: the atlas needs a ruling (split per-step and arm workspace into
   their own gaps, or keep G-PATH open).
5. **Fixed-base arms.** P3 covers "the robot's base and anything it carries". For `chef_arm` or `helper_arm`, whose
   only `move` is the arm itself, is the arm's motion base motion? No task here depends on it (the arm tasks use the
   mobile `installer1`).
6. **Bound robots that do not move, and recovery moves.** P6 says a task "bound to" a robot without `honours` fails:
   even a robot that is only named, or only moves in a handler? P2 says every motion: does a limit hold in a `fail`
   handler's recovery move? No task here separates these readings.
7. **The start location.** P5 checks targets only. A robot that starts inside a no-go region, or outside an `in_region`,
   is not covered; no task asks.
