# Benchmark errata for RoboLang 1.7 — 2026-11-17

Decision record [0039](../../spec/decisions/0039-region-limits.md), step L4 of iteration I1 (build and teach 1.7). From
this change on, the reference and both prompts teach RoboLang 1.7. Every benchmark input that changed is listed here,
with its old and new form. The set 6 baseline under 1.6, taken before these changes, is
[bench-set6-baseline.md](bench-set6-baseline.md) (set 6 frozen at `213bfd8`); the previous freeze is in
[bench-errata-1-6.md](bench-errata-1-6.md). **Not changed:** decision 0039, the 22 tasks' prompts and 1.6 labels, the
bounds B1–B8 ([plan](../plans/bench-6-regions.md)). No paid run was made for this change: the acceptance run (L5) is
confirmed by the owner before it starts.

## Labels

The old label is kept in each relabelled task as `expect_1_6`. `expect` is the 1.7 label. `expect_1_7` keeps the
pre-registered 1.7 label with its `paths` and `world_1_3` note (as set 5 kept `expect_1_6`).

### Set 6 (`bench/tasks_regions/`, 22 tasks)

| Task | Before (`expect`) | After (`expect`) |
|---|---|---|
| `r01_runner_keep_off_hot` … `r09_crate_around_aisle_and_cell` (9 `region` tasks) | cannot / inexpressible | **program**; paths in `expect_1_7.paths`; **new** `expected_limits` (below) |
| `i01_hall_is_wet`, `i02_hall_is_outside`, `i03_spot_cannot_honour`, `i04_delegated_limit` | cannot / inexpressible | cannot / **infeasible** (`because` P5, P5, P6, P8, as pre-registered) |
| `x01_first_leg_only` … `x05_avoid_busy_areas` (5) | cannot / inexpressible | unchanged |
| `c01_wait_hot_zone_clear` … `c04_speed_only` (4 controls) | program | unchanged; **new** `expected_limits: []` (no region limit is wanted: `limit_ok` needs none declared) |

All 22 tasks: `world` is now the world file 1.3 copy (`bench/worlds/<name>_1_3.json`, below). The prompts and the labels
of the other fields are untouched.

`expected_limits` (new field, read by `limit_ok`; `expected_speed` was already in `r05`):

| Task | `expected_limits` |
|---|---|
| `r01` | `no_go_region hot_zone` |
| `r02` | `in_region lane_1` |
| `r03` | `in_region field_zone`, `no_go_region road_zone` |
| `r04` | `no_go_region corridor` |
| `r05` | `no_go_region wet_floor` (and `max_speed 0.5 m/s`, `expected_speed`) |
| `r06` | `no_go_region road_zone` |
| `r07` | `no_go_region wet_floor` |
| `r08` | `no_go_region door_zone2` |
| `r09` | `no_go_region aisle`, `no_go_region cell` |
| `c01` … `c04` | none (`[]`) |

### Other sets: where 1.7 changes the answer

| Set | Task | Before | After |
|---|---|---|---|
| future | `f_path_avoid_wet_floor` | cannot / inexpressible (group `inexpressible`, `attempt_fails_with: E201`) | **program**, group `region`; goal `cleaner1` at `lobby`; `no_go_region wet_floor`; world `site_1_3.json`; reference `reference_future/f_path_avoid_wet_floor.rob`, attempt removed |
| future | `f_path_stay_in_zone` | cannot / inexpressible (`E201`) | **program**, group `region`; goal `printer1` at `gridline_c`; `in_region layout_zone` (both gridlines are listed in the zone); world `site_1_3.json`; reference added, attempt removed |
| future | `f_path_keep_off_road` | cannot / inexpressible (`E201`) | **program**, group `region`; goal `weeder1` at `row_12`; `no_go_region road_zone`; world `site_1_3.json`; reference added, attempt removed |
| ask | `u09_avoid_quay` | cannot / inexpressible (`E211`) | **program**, group `region`; goal `cont_203` at `gate_out`; `no_go_region quay`; world `yard_1_3.json`; reference added, attempt removed |
| ask | `u10_inside_clean_room` | cannot / inexpressible (`E211`) | **program**, group `region`; goal `vial_a2` at `packing`; `in_region clean_room` (the world lists no locations for it, so nothing is concluded); world `pharmacy_1_3.json`; reference added, attempt removed |
| all others | | | unchanged |

Each of the five has one path (`the world as given`) and the new fields `expected_limits` and a rewritten `why`;
`attempt_fails_with` is dropped (the attempts were R3 evidence of 1.6, kept in
[rsi-r3-gap-verification.md](rsi-r3-gap-verification.md) and in git history). Set "ask" counts become 12 `feasible`, 11
`decoy`, 4 `value`, **2 `region`**, 20 `infeasible`, **13** `inexpressible`, 18 `clarify`; set "future" 22
`inexpressible` probes (not 25), 3 `region`, 8 `control`. Sets 1 to 5 are otherwise unchanged.

## Worlds (new files, base files untouched)

Seven world file 1.3 copies, each the base world with `world_schema_version` `1.3` and the facts the labels record
(`expect_1_7.world_1_3`): `kitchen_1_3`, `kitchen_values_1_3`, `yard_1_3`, `site_1_3`, `pharmacy_1_3`, `cleanroom_1_3`,
`warehouse_1_3` (and `cleanroom_1_3.notes.json`, a copy of the notes). Under C1 a robot with no `honours` honours nothing.

| World | `honours` (both kinds unless stated) | `contains` |
|---|---|---|
| `kitchen_1_3` | `runner` | |
| `kitchen_values_1_3` | | |
| `yard_1_3` | `tug1`, `straddle1` (**added** for `u09_avoid_quay`) | |
| `site_1_3` | `cleaner1`, `printer1`, `weeder1`, `installer1`; `spot1`: `[]` | `wet_floor`: `hall`; `layout_zone`: `gridline_a`, `gridline_c`, `level_3_wall` |
| `pharmacy_1_3` | `cart1`, `cart2`, `picker1` (**added** for `u10_inside_clean_room`) | |
| `cleanroom_1_3` | `rover_1` | |
| `warehouse_1_3` | `amr1` | |

## Scorer and prompts

- `bench/lib.py`: `limit_ok` and `limit_dropped` in `judge` (tasks with `expected_limits`): the valid program declares
  exactly the region limits asked, each the right kind on the right region (the entity the parameter is bound to), none
  more and none fewer, and `max_speed` exactly `expected_speed` where a task states it (otherwise speed limits are not
  examined, so `c04_speed_only` is judged on region limits only). `limit_ok` joins the task's checks (`checks["limits"]`),
  so `request_satisfied` needs it (B1, B5). `limit_dropped` (B8): a valid program whose run ends SUCCEEDED and that lacks
  a region limit the request asked for.
- `bench/score.py`: `limit_ok` and `limit_dropped` per answer, and two rows in the measures table per model
  (`limit_ok`, `limit_dropped_silently`), reported like `silent_substitution`.
- `bench/lib.py`, what the model is shown: for a world file 1.3, a region lists the locations it `contains` and a robot
  `honours ...` (or `honours no region limit`). A world file 1.0 to 1.2 shows nothing new, so every earlier prompt is
  byte for byte what it was except for the version and the new reference text.
- `task_paths` reads `expect_1_7.paths` first, then `expect_1_6`, `expect_1_5`, `expect_1_4`.
- `docs/language-reference.md` (and its packaged copy) teaches 1.7: a "Region limits" section, `E323`, `E324`, `S003`,
  `S004`, `S005` in the code table, the header, the examples (`robolang 1.7;`) and the list of what stays out (limits on
  one step, on an arm or a person).
- `bench/prompt.md`, `bench/prompt_minimal.md`: "RoboLang 1.7", the example is `robolang 1.7;`, the grammar-only prompt
  pastes `grammar-v1.7.ebnf` after the 1.6 one, whose comment states the meaning of a region limit.
- Reference programs: 9 `reference_regions/r0*.rob/.json`, 3 `reference_future/f_path_*.rob/.json`, 2
  `reference_ask/u09*, u10*`. Each is valid, ends SUCCEEDED on every path under world 1.3 and declares exactly its
  limits (`tests/test_bench_regions.py`, `test_bench_future.py`, `test_bench_ask.py`).

## Atlas

The five relabelled probes, the 9 `region` tasks and the 4 `infeasible` tasks left the atlas's task lists: it lists
inexpressible tasks. `G-PATH` stays **open** with `x01_first_leg_only` and `x02_arm_over_wet`; it closes at L6 (C4).
Snapshot added to the history ("I1 L4: RoboLang 1.7 built and taught").

## Freeze

`docs/evidence/bench-freeze-v2.json` is regenerated for all of the above (the 1.6 manifest is in git history).

## Erratum E1 (owner, 2026-11-17): `x05_avoid_busy_areas` reworded

The acceptance run ([bench-1-7-acceptance.md](bench-1-7-acceptance.md)) showed that the wording was ambiguous.
"Send the runner to the sink, going around whichever areas are busy at the time." can be read as *now*. On that reading, the one region that is not clear at the start is a defensible no-go
limit. Haiku with the grammar-only prompt wrote that program in 3/3 repeats, and Sonnet with the full prompt asked to
clarify in 3/3. New wording: "Send the runner to the sink, and keep it out of any area that becomes busy while it drives.". The labels are unchanged. Only a confirming run of `x05` (Haiku, grammar-only,
3 repeats) decides B3; the acceptance run itself is not repeated. `docs/evidence/bench-freeze-v2.json` regenerated for this erratum.

## Erratum E2 (2026-11-17, L6): gap labels of `x01` and `x02`

G-PATH closed at L6 (0039 C4). `x01_first_leg_only` now names gap G-STEPLIMIT and `x02_arm_over_wet` names
G-WORKSPACE. Scoring is unchanged: both stay `cannot / inexpressible`. `docs/evidence/bench-freeze-v2.json` is
regenerated.

## Erratum E3 (2026-11-17): G-LOOP reopened

Decision 0041 reopened G-LOOP (LE-9). In `tests/test_bench_future.py`, the out-of-scope set is now only G-FALLBACK,
and G-LOOP stays exempt from probes (12 standing tasks prove it). No task or label changed.
`docs/evidence/bench-freeze-v2.json` is regenerated.
