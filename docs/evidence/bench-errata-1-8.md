# Benchmark errata for RoboLang 1.8 — 2026-11-17

Decision record [0040](../../spec/decisions/0040-declared-actions.md), iteration I2, step L3 (pre-register set 7
"actions"). At that step 1.8 was not built and the reference and both prompts still taught RoboLang 1.7; erratum E2 (L4,
below) records the build and the teaching. This file lists every
benchmark input that changed, so that `docs/evidence/bench-freeze-v2.json` can be regenerated with a recorded reason
(the previous manifest is `4eeecb49…`, in git history; the earlier errata are [1.5](bench-errata-1-5.md),
[1.6](bench-errata-1-6.md) and [1.7](bench-errata-1-7.md)). Later entries (L4: labels, references, scorer, prompts) are
added here. No paid run was made: the baseline (1.7) is confirmed by the owner before it starts.

## Erratum E1 (2026-11-17, L3): set 7 "actions" added

Plan and proposed bounds: [bench-7-actions.md](../plans/bench-7-actions.md) (draft for the owner; the bounds are
placeholders to fix with the baseline, rule 6). Nothing standing was changed: sets 1 to 6, the prompts, the reference,
the grammar, the scorer and the worlds are as before.

| Input | Change |
|---|---|
| `bench/tasks_actions/` | **new**, 24 tasks: 10 `action`, 5 `infeasible`, 5 `inexpressible`, 4 `control`. `expect` is the 1.7 label, `expect_1_8` the 1.8 one (with the world 1.4 facts as a note) |
| `bench/reference_actions/` | **new**: a reference (`.rob`, `.json`) for each of the 4 controls; the obvious attempt (`.attempt.rob`, `.attempt.json`, fails with `E203`) for each of the other 20 |
| `bench/lib.py` | `ACTIONS_TASKS_DIR`, `ACTIONS_REFERENCE_DIR`, and `"actions"` in `SETS` |
| `tests/test_bench_actions.py` | **new** (58 checks); `tests/test_atlas.py` docstring |
| `bench/atlas.json` | the 20 tasks that 1.7 declines are in a gap: G-ACT 16 (`a01`–`a10`, `i01`–`i05`, `x02`), G-PROX 1 (`x01`), G-TEACH 1 (`x03`), G-LOOP 1 (`x04`), G-CLOCK 1 (`x05`); snapshot "I2 L3" in the history; G-TEACH's scope note names its first task |
| `docs/plans/humanoid-activities.md` | regenerated from the atlas |
| `.github/workflows/benchmark.yml` | input `actions: true` (full and minimal prompts, 3 repeats, Sonnet and Haiku, `--max-usd 3` each, not gated); excluded from the per-PR and weekly runs; `actions`, `actions-minimal` in the summary loop |

No world file was added and no world file has a 1.4 field. The 1.4 facts (`actions`, `state`) are in
`expect_1_8.world_1_4`, a note that no code reads: L4 writes the world copies. The one 1.3 world used is the existing
`bench/worlds/site_1_3.json` (`a07` also asks a region limit).

Not copied, as in set 6: the three R3 probes (`f_act_set_fryer`, `f_act_open_closet`, `f_act_print_layout`) and the standing
tasks `u15_open_oven`, `u16_charge_drone`, `x06_set_oven`. They stay in their sets, unchanged, until L4 relabels them.

## Freeze

`docs/evidence/bench-freeze-v2.json` is regenerated for all of the above.

## Erratum A1 (owner, 2026-11-17): `x02_selftest_result` moves from G-ACT to G-SELF

Clarification C9 of 0040: an action that returns a value is the robot's own state, so the task belongs in G-SELF. It
stays `cannot / inexpressible`, and its scoring is unchanged. `docs/evidence/bench-freeze-v2.json` is regenerated.

## Erratum E2 (2026-11-17, L4): RoboLang 1.8 built and taught

Decision record [0040](../../spec/decisions/0040-declared-actions.md), step L4 of iteration I2 (build and teach 1.8). From
this change on, the reference and both prompts teach RoboLang 1.8. Every benchmark input that changed is listed here. The
set 7 baseline under 1.7, taken before these changes, is [bench-set7-baseline.md](bench-set7-baseline.md). **Not changed:**
decision 0040, the 24 tasks' prompts and 1.7 labels, the bounds B1–B8 ([plan](../plans/bench-7-actions.md)). No paid run was
made for this change: the acceptance run (L5) is confirmed by the owner before it starts.

### Labels

The old label is kept in each relabelled task as `expect_1_7`. `expect` is the 1.8 label. `expect_1_8` keeps the
pre-registered 1.8 label with its `paths` and its `world_1_4` note (as set 6 kept `expect_1_7`); `task_paths` reads
`expect_1_8.paths` first.

**Set 7** (`bench/tasks_actions/`, 24 tasks)

| Task | Before (`expect`) | After (`expect`) |
|---|---|---|
| `a01_open_then_fetch` … `a10_decoy_open_closet` (10 `action` tasks) | cannot / inexpressible | **program**; paths in `expect_1_8.paths`; `expected_actions` as pre-registered |
| `i01_runner_cannot_open` … `i05_leave_it_locked` (5) | cannot / inexpressible | cannot / **infeasible** (`because` A6 ×4, A7, as pre-registered) |
| `x01_hand_to_ana` … `x05_open_at_six` (5) | cannot / inexpressible | unchanged (`x02` is in G-SELF since erratum A1) |
| `c01_fetch_no_door` … `c04_plain_move_printer` (4 controls) | program | unchanged; `expected_actions: []` (no `do` is wanted: `actions_ok` needs none declared) |

All 24 tasks: `world` is now a world file 1.4 copy (below). The prompts and the other fields are untouched. The 1.7 attempts
(`bench/reference_actions/*.attempt.*`) stay as the evidence of the baseline: each still fails with `E203` under 1.7.

**Other sets** (each relabelled as a program, group `action`, with `expect_1_7` or the older labels kept, `expected_actions`
new, a 1.8 reference, and a world 1.4 copy; the prompt is unchanged)

| Task | Set | Before | After | World facts added |
|---|---|---|---|---|
| `f_act_set_fryer` | future (R3 probe) | cannot / inexpressible | **program**: `do flippy1 set fryer to 175 degC`, then a `wait until` on the reading; two paths (the oil heats; it never heats and the run stops) | `fry_station_hot_1_4`: `flippy1` declares `set` (a temperature, result `heating`) on the location `fryer`, state `off`; **the oil reads 176 degC at 20 s (was 175)**, because "reads 175" is `>= 175` and a reading of 175 +/- 0.5 is unknown against it (0036 V7) and would never resolve |
| `f_act_open_closet` | future (R3 probe) | cannot / inexpressible | **program**: `do moxi1 open closet_door`; a message on `charge_nurse`, any text | `ward_1_4`: `moxi1` declares `open` (an object, result `open`); `closet_door` state `closed` |
| `f_act_print_layout` | future (R3 probe) | cannot / inexpressible | **program**: `do printer1 print level_3_wall` | `site_1_4`: `printer1` declares `print` (a location, result `printed`); `gridline_c` and `level_3_wall` state `blank` |
| `u15_open_oven` | ask | cannot / inexpressible | **program**: the helper arm opens `oven_door_1` and takes the tray to the pass | `kitchen_1_4`: `helper_arm` declares `open`; `oven_door_1` state `closed` |
| `u16_charge_drone` | ask | cannot / inexpressible | **program**: `do drone1 charge workshop`, then fly to `berth_1`; goal `in_state workshop "charging"` and the drone at `berth_1` (the state is on the dock, 0040 C14) | `yard_1_4`: `drone1` declares `charge` and `dock` (a location, results `charging`, `docked`); `workshop` state `empty` |
| `x06_set_oven` | values | cannot / inexpressible | **program**: `do chef_arm set oven to 180 degC`; goal `in_state oven "heating"` (the value is recorded, not checked: 0040 A5, C4) | `kitchen_values_oven_1_4`: `chef_arm` declares `set` (a temperature, result `heating`) on the location `oven`, state `off` |

`expected_actions` (new field, read by `actions_ok` and `action_replaced`): `a01`, `a05` and `f_act_open_closet`: `moxi1 open
closet_door`; `a02` and `f_act_set_fryer`: `flippy1 set fryer 175 degC`; `a03`: `printer1 print gridline_c`; `a04`:
`chef_arm open oven_door_1`; `a06`: `crane1 lock cont_101`, `crane2 lock cont_102`; `a07` and `f_act_print_layout`: `printer1
print level_3_wall`; `a08`: `cart2 charge charger`; `a09`: `drone1 dock workshop`; `a10`: `moxi1 open_door closet_door`;
`u15`: `helper_arm open oven_door_1`; `u16`: `drone1 charge workshop`; `x06`: `chef_arm set oven 180 degC`. Every
control and no other task has `[]`.

`a06`: the reference gains the move that C2 requires: `crane2` stands at `berth_2` and `cont_102` at `berth_1`, so it moves to
`berth_1` before it locks. The prompt, the paths and `expected_actions` are as pre-registered.

`a05`: the second path scripts a fault, `scripted_fault: {step: "do", action: "open", target: "closet_door"}`. A scripted
fault is a step number in a world file, and the number depends on the program, so the benchmark writes the world's fault
script for each `do` of that action on that entity of the answer (0040 C6). A program without such a `do` has nothing to fail.

### Worlds (new files, base files untouched)

Fourteen world file 1.4 copies, each a base world with `world_schema_version` `1.4` and the facts the labels record
(`expect_1_8.world_1_4`): one file for each profile a request needs.

| World | Base | Declared actions | States |
|---|---|---|---|
| `ward_1_4` | `ward` | `moxi1`: `open` (object, `open`) | `closet_door`: `closed` |
| `ward_door_1_4` | `ward` | `moxi1`: `open_door` (object, `open`) only: the decoy of `a10` | `closet_door`: `closed` |
| `ward_close_1_4` | `ward` | `moxi1`: `open`, `close` (object, `closed`): `i05` | `closet_door`: `closed` |
| `ward_hand_1_4` | `ward` | `moxi1`: `hand` (object, `handed`): `x01` | `closet_door`: `closed` |
| `fry_station_1_4` | `fry_station` | `flippy1`: `set` (location, `heating`, a temperature) | `fryer`: `off` |
| `fry_station_hot_1_4` | `fry_station` | as above | `fryer`: `off`; the oil event reads 176 degC |
| `fry_station_selftest_1_4` | `fry_station` | `flippy1`: `self_test` (location, `tested`): `x02` | `fryer`: `off` |
| `site_1_4` | `site` | `printer1`: `print` (location, `printed`) | `gridline_c`, `level_3_wall`: `blank` |
| `site_limits_1_4` | `site_1_3` | as above; its `honours` and `contains` are the 1.3 ones (`a07`) | as above |
| `kitchen_values_1_4` | `kitchen_values` | `chef_arm`, `helper_arm`: `open` (object, `open`); `runner`: none | `oven_door_1`: `closed` |
| `kitchen_values_oven_1_4` | `kitchen_values` | `chef_arm`: `set` (location, `heating`, a temperature): `x06` | `oven`: `off` |
| `kitchen_1_4` | `kitchen` | `helper_arm`: `open` (object, `open`): `u15` | `oven_door_1`: `closed` |
| `yard_1_4` | `yard` | `crane1`, `crane2`: `lock` (object, `locked`); `drone1`: `dock` (location, `docked`), `charge` (location, `charging`) | `cont_101`, `cont_102`: `unlocked`; `workshop`: `empty` |
| `pharmacy_1_4` | `pharmacy` | `cart2`: `charge` (location, `charging`) | `charger`: `free` |

The controls use the files of their twin: `c01` the `ward_1_4` of `a01`, `c02` and `c03` the `fry_station_1_4` of `a02`,
`c04` the `site_1_4` of `a03`. `i03` uses `fry_station_1_4`, `i04` `ward_1_4`, `i01`, `x04` and `x05` `kitchen_values_1_4`,
`i02` `yard_1_4`, `a06` and `a09` `yard_1_4`, `x03` `ward_1_4`.

### Scorer, prompts and what the model is shown

- `bench/lib.py`: `actions_ok` and `action_replaced` in `judge` (tasks with `expected_actions`): the valid program has
  exactly the `do` steps asked, each the right action by the right robot on the right target (the entity the parameter is
  bound to) with the right value (a quantity compared exactly in the base unit, so `448.15 K` is `175 degC`; a text as itself),
  none more and none fewer. `actions_ok` joins the task's checks (`checks["declared_actions"]`), so `request_satisfied`
  needs it (B1, B5). `action_replaced` (B8): a valid program whose run ends SUCCEEDED and that lacks an action the request
  asked for.
- `bench/lib.py`, goals (C1): an `in_state` goal condition is a fact `[X, "in_state", label]` about the state of `X`, a
  subject of its own, so a state and a place of the same object are independent facts and a goal is compared by meaning as
  before (the labels it enumerates are the ones the formulas mention, plus "any other label").
- `bench/lib.py`, paths: `scripted_fault` (above), and `task_paths` reads `expect_1_8.paths` first.
- `bench/score.py`: `actions_ok` and `action_replaced` per answer, and two rows in the measures table per model
  (`actions_ok`, `action_replaced_silently`), reported like `limit_ok` and `limit_dropped_silently`.
- `bench/lib.py`, what the model is shown (C12): for a world file 1.4, each robot lists `can do: NAME (on an object|a
  location, leaves it RESULT[, takes a KIND]); ...` (or `can do no declared action`), each object with a state shows it, and
  one line lists the location states. A world file 1.0 to 1.3 shows nothing new. `honours` and `contains` are shown for a
  world 1.3 or 1.4 only where the file has them, so the prompt of a 1.4 world without region lists is not changed by them.
- `docs/language-reference.md` (and its packaged copy) teaches 1.8: a "Declared actions" section, `do` and `in_state` in the
  reserved words, the plan statements and the conditions, `S006` to `S009`, `S271` in the code table, the header, the
  examples (`robolang 1.8;`), and the list of what stays out (several targets, an action that returns a value, a macro, an
  action repeated until it works or at a clock time). It drops the sentence that no command writes a value.
- `bench/prompt.md`, `bench/prompt_minimal.md`: "RoboLang 1.8", the example is `robolang 1.8;`, the grammar-only prompt
  pastes `grammar-v1.8.ebnf` after the 1.7 one, whose comment states the meaning of `do` and `in_state`.
- Reference programs: 10 `reference_actions/a*.rob/.json` and 4 controls (now `robolang 1.8;`), 3
  `reference_future/f_act_*`, 2 `reference_ask/u15*, u16*`, 1 `reference_values/x06*`. Each is valid, ends SUCCEEDED on
  every path under its world 1.4 and does exactly its actions (`tests/test_bench_actions.py`, `test_bench_future.py`,
  `test_bench_ask.py`, `test_bench_values.py`). The three `f_act_*` attempts are replaced by these references.

### Atlas

The 10 `action` tasks, the 5 `infeasible` tasks, the six relabelled older tasks and the controls are not in the atlas's task
lists: it lists inexpressible tasks. `G-ACT` stays **open**, with no task left in it; it closes at L6 (A10 keeps nothing
inexpressible in it). `x01` stays in G-PROX, `x02` in G-SELF, `x03` in G-TEACH, `x04` in G-LOOP and `x05` in G-CLOCK.
Snapshot added to the history ("I2 L4: RoboLang 1.8 built and taught").

### Freeze

`docs/evidence/bench-freeze-v2.json` is regenerated for all of the above (the manifest of E1 and A1 is in git history).
