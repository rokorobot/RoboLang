# Benchmark errata for RoboLang 1.9 — 2026-11-17

Decision record [0042](../../spec/decisions/0042-clock-readings.md), iteration I3, step L3 (pre-register set 8
"clock"). At that step 1.9 was not built and the reference and both prompts still taught RoboLang 1.8; erratum E2
(L4, below) records the build and the teaching. This file lists every benchmark input that changed, so that `docs/evidence/bench-freeze-v2.json` can be regenerated with a recorded reason
(the previous manifest is `6c3d9c1e…` for L3 and the L3 one for L4, in git history; the earlier errata are [1.5](bench-errata-1-5.md),
[1.6](bench-errata-1-6.md), [1.7](bench-errata-1-7.md) and [1.8](bench-errata-1-8.md)). Later entries (L4: labels,
references, scorer, prompts) are added here. No paid run was made: the baseline (1.8) is confirmed by the owner before
it starts.

## Erratum E1 (2026-11-17, L3): set 8 "clock" added

Plan and proposed bounds: [bench-8-clock.md](../plans/bench-8-clock.md) (draft for the owner; the bounds are
placeholders to fix with the baseline, rule 6). Nothing standing was changed: sets 1 to 7, the prompts, the reference,
the grammar, the scorer and every existing world are as before.

| Input | Change |
|---|---|
| `bench/tasks_clock/` | **new**, 24 tasks: 10 `clock` programs, 5 `infeasible`, 5 `inexpressible`, 4 `control`. `expect` is the 1.8 label, `expect_1_9` the 1.9 one (with the world 1.5 facts as a note) |
| `bench/reference_clock/` | **new**: a reference (`.rob`, `.json`) for each of the 4 controls; the obvious attempt (`.attempt.rob`, `.attempt.json`, fails with `E101` or `E217`; the clock part is marked `// @clock`) for each of the other 20 |
| `bench/worlds/fry_station_at_fryer.json` | **new** (world 1.2): `fry_station.json` with flippy1 at the fryer, for `k09` |
| `bench/lib.py` | `CLOCK_TASKS_DIR`, `CLOCK_REFERENCE_DIR`, and `"clock"` in `SETS` |
| `tests/test_bench_clock.py` | **new** (61 checks) |
| `bench/atlas.json` | the 20 tasks that 1.8 declines are in a gap: G-CLOCK 17 (`k01`–`k10`, `i01`–`i05`, `x02`, `x05`), G-LOOP 1 (`x01`), G-CALC 2 (`x03`, `x04`); snapshot "I3 L3" in the history |
| `docs/plans/humanoid-activities.md` | regenerated from the atlas |
| `.github/workflows/benchmark.yml` | input `clock: true` (full and minimal prompts, 3 repeats, Sonnet and Haiku, `--max-usd 3` each, not gated); excluded from the per-PR and weekly runs; `clock`, `clock-minimal` in the summary loop. The workflow now has 10 `workflow_dispatch` inputs, GitHub's limit |
| `bench/README.md`, `docs/LANGUAGE_ROADMAP.md` | a section for set 8; the I3 row |

No world file has a 1.5 field. The 1.5 facts (`clock`) are in `expect_1_9.world_1_5`, a note that no code reads: L4 writes
the world copies.

Not copied, as in sets 6 and 7: the three R3 probes (`f_clock_scan_stamp`, `f_clock_shift_cutoff`,
`f_clock_clean_at_2300`) and the standing tasks `x_elapsed_time`, `x_report_duration`, `u17_at_1400`, `u18_every_morning`,
`x05_cooling_time` and `x05_open_at_six`. They stay in their sets, unchanged, until L4 relabels them.

## Freeze

`docs/evidence/bench-freeze-v2.json` is regenerated for all of the above.

## Erratum K (owner, 2026-11-17): gap membership after the clarifications of 0042

The owner approved K7 and K11, and with them four tasks change gap. Labels and scoring are unchanged.

| Task | From | To | Why |
|---|---|---|---|
| `x02_tomorrow_at_0100` | G-CLOCK | new **G-DATE** | another day (K11) |
| `x05_on_monday` | G-CLOCK | new **G-DATE** | a date (K11) |
| `x05_cooling_time` (set "values") | G-CLOCK | G-CALC | it stays inexpressible: the time between two moments (K7) |
| `u18_every_morning` (set "ask") | G-CLOCK | G-LOOP | a mission trigger (0041 M6), not a clock reading |

With these moves G-CLOCK holds only what 1.9 expresses, so it can close at L6.
`docs/evidence/bench-freeze-v2.json` is regenerated.

## Erratum E2 (2026-11-17, L4): RoboLang 1.9 built and taught

Decision record [0042](../../spec/decisions/0042-clock-readings.md), step L4 of iteration I3 (build and teach 1.9). From
this change on, the reference and both prompts teach RoboLang 1.9. Every benchmark input that changed is listed here. The
set 8 baseline under 1.8, taken before these changes, is [bench-set8-baseline.md](bench-set8-baseline.md). **Not changed:**
decision 0042, the 24 tasks' prompts and 1.8 labels, the bounds B1–B8 ([plan](../plans/bench-8-clock.md)). No paid run was
made for this change: the acceptance run (L5) is confirmed by the owner before it starts.

### Labels

The old label is kept in each relabelled task as `expect_1_8` (the 1.8 label; a task that already had one keeps it).
`expect` is the 1.9 label. `expect_1_9` keeps the pre-registered 1.9 label with its `paths` and its `world_1_5` note;
`task_paths` reads `expect_1_9.paths` first.

**Set 8** (`bench/tasks_clock/`, 24 tasks)

| Task | Before (`expect`) | After (`expect`) |
|---|---|---|
| `k01_scan_stamp` … `k10_decoy_11_pm` (10 `clock` tasks) | cannot / inexpressible | **program**; paths in `expect_1_9.paths`; `expected_clock` as pre-registered |
| `i01_stamp_no_clock` … `i05_window_already_closed` (5) | cannot / inexpressible | cannot / **infeasible** (`because` T7 ×2, T4 ×2, T5, as pre-registered) |
| `x01_every_night_round` … `x05_on_monday` (5) | cannot / inexpressible | unchanged (`expect_1_8` records the same label) |
| `c01_plain_two_moves` … `c04_as_soon_as_possible` (4 controls) | program | unchanged; `expected_clock: []` (no clock reading is wanted: `clock_ok` needs none) |

The 15 tasks with a clock fact (`k01`–`k10`, `i01`–`i05`) use a world file 1.5 copy (below). The prompts and the other fields are
untouched. The 1.8 attempts (`bench/reference_clock/*.attempt.*`) stay as the evidence of the baseline: each still fails
under 1.8 with `E101` or `E217`.

**Other sets** (each relabelled as a program, group `clock`, with the old label kept as `expect_1_8` (`expect_1_7` for
`x05`, which already had it), `expected_clock` new, a 1.9 reference, and a world file 1.5 copy; the prompt is unchanged;
the 1.8 attempt is removed, as for set 7's relabelled probes, except for `x05_open_at_six`, which is a task of set 7)

| Task | Set | Before | After | World facts added |
|---|---|---|---|---|
| `f_clock_scan_stamp` | future (R3 probe) | cannot / inexpressible | **program**: `measure arm1 time` after the `inspect`, sent to `lead` | `warehouse_1_5_1420`: the clock starts at 14:20 |
| `f_clock_shift_cutoff` | future (R3 probe) | cannot / inexpressible | **program**: `if now < 22:00` moves the tote, else the lead is told and nothing moves; two paths (start 21:15, start 22:30), each with its goal | `fulfilment_1_5_2115`: 21:15; the second path sets 22:30 |
| `f_clock_clean_at_2300` | future (R3 probe) | cannot / inexpressible | **program**: a wait for 23:00 (the request sets no limit, so any timeout that reaches it), then lobby and back | `site_1_5_2220`: 22:20 |
| `x_elapsed_time` | hard | cannot / inexpressible | **program**: `measure arm1 elapsed` after the last step, sent to `ops` | `example_1_5`: no clock (`elapsed` needs none) |
| `x_report_duration` | branch | cannot / inexpressible | **program**: as `x_elapsed_time` | `example_1_5` |
| `u17_at_1400` | ask | cannot / inexpressible | **program**: a wait for 14:00 (any timeout that reaches it), then the tray to dispatch | `pharmacy_1_5_1330`: 13:30 |
| `x05_open_at_six` | actions (set 7) | cannot / inexpressible | **program**: a wait for 06:00, then `do helper_arm open oven_door_1`; goal `in_state oven_door_1 "open"`; `expected_actions` as for `a04` | `kitchen_values_1_5_0530`: 05:30 (from `kitchen_values_1_4`: `helper_arm` declares `open`, `oven_door_1` is `closed`) |

`expected_clock` (new field, read by `clock_ok` and `time_replaced`): `k01`, `f_clock_scan_stamp`: `time` sent to `lead` by
`arm1`, after the `inspect`; `k02`: an `if` on `< 21:30` by `digit2`; `f_clock_shift_cutoff`: an `if` on `< 22:00` by `digit1`;
`k03`: a wait for `>= 05:30`, timeout 90 min; `k04`, `x_elapsed_time`, `x_report_duration`: `elapsed` sent to `ops` by `arm1`
after the last step; `k05`: an `if` on `>= 08:00` and `< 18:00` by `arm2`; `k06`: a wait for `>= 20:00`, timeout 1 h
(the labels write `1 h`; the language has no hour unit, see below); `k07`: an `if` on `< 18:00` by `moxi1`; `k08`: an `if` on
`< 17:00` by `printer1`; `k09`: `elapsed` sent to `fry` by `flippy1`, after the wait; `k10`: a wait for `>= 23:00`, timeout
2 h; `f_clock_clean_at_2300`, `u17_at_1400`, `x05_open_at_six`: a wait for the time asked with no timeout fixed (the request gives
none). Every control and no other task has `[]` or no field.

### Worlds (new files, base files untouched)

Sixteen world file 1.5 copies, each a base world with `world_schema_version` `1.5` and, where the label says, the `clock`
(the zone is `Europe/Prague`, the day 2026-11-17): one file for each base and start a request needs. A path sets another
start with `world_patch.clock`.

| World | Base | Clock start | Used by |
|---|---|---|---|
| `lab_1_5` | `lab` | none | `i01` |
| `warehouse_1_5_1420` | `warehouse` | 14:20 | `k01`, `f_clock_scan_stamp` |
| `warehouse_1_5_0940` | `warehouse` | 09:40 | `k05` |
| `warehouse_1_5_0915` | `warehouse` | 09:15 | `i02`, `i05` |
| `fulfilment_1_5_2105` | `fulfilment` | 21:05 | `k02` |
| `fulfilment_1_5_1930` | `fulfilment` | 19:30 | `k06` |
| `fulfilment_1_5_2115` | `fulfilment` | 21:15 | `f_clock_shift_cutoff` |
| `yard_1_5_0430` | `yard` | 04:30 | `k03` |
| `yard_1_5_2130` | `yard` | 21:30 | `i03` |
| `example_1_5` | `examples/world.json` (no task entries) | none | `k04`, `i04`, `x_elapsed_time`, `x_report_duration` |
| `site_1_5_2220` | `site` | 22:20 | `k10`, `f_clock_clean_at_2300` |
| `ward_1_5_1630` | `ward_1_4` | 16:30 | `k07` |
| `site_1_5_1540` | `site_1_3` | 15:40 | `k08` |
| `fry_station_at_fryer_1_5` | `fry_station_at_fryer` | none | `k09` |
| `pharmacy_1_5_1330` | `pharmacy` | 13:30 | `u17_at_1400` |
| `kitchen_values_1_5_0530` | `kitchen_values_1_4` | 05:30 | `x05_open_at_six` |

The controls and the five `inexpressible` tasks keep their worlds (a world older than 1.5 shows no clock line, so nothing about
them changes). Each copy equals its base apart from the version and the `clock`; `tests/test_bench_clock.py` checks it.

### Scorer, prompts and what the model is shown

- `bench/lib.py`: `clock_ok` and `time_replaced` in `judge` (tasks with `expected_clock`). `clock_ok` (B1, B5): the valid
  program reads the clock exactly as asked: the number of `measure r time|elapsed` steps equals the number of messages and
  `if`s that use one, and each use matches the label: a message sends the binding of the right kind to the right channel
  from a robot the label lists; an `if` compares the right kind with the right times, **modulo its complement** (`now <
  21:30` and `now >= 21:30` are the same test with the branches swapped; the paths decide whether the branches are right);
  a wait on `clock_time` has the time and operator asked and a timeout equal to the label's as a duration (60 min is 1 h) or
  any timeout when the label gives none; `after` orders the reading (after the `inspect`, after the last move/grasp/place,
  after a wait); no reading and no time literal anywhere else, and none on a control. It joins the task's checks
  (`checks["clock"]`), so `request_satisfied` needs it. `time_replaced` (B8): a valid program whose run ends SUCCEEDED and
  that lacks a use of the kind asked (a time `if`, a wait for a time, a time message): a guessed `wait for`, a fixed text or
  nothing in its place. A wrong operator or a wrong time that is there is `clock_ok` false, not a replacement.
- `bench/lib.py`, paths: `task_paths` reads `expect_1_9.paths` first. A path's `world_patch.clock` forces the start.
- `bench/lib.py`, what the model is shown (K9): for a world file 1.5, `clock: the task starts at 2026-11-17 21:30:00
  (Europe/Prague)` or `clock: none (this world has no clock)`, and each robot lists `can do` as for a 1.4 world; `honours` and
  `contains` are shown for a 1.5 world only where the file has them. A world file older than 1.5 shows nothing new.
- `bench/lib.py` and the prompts: the answer schema, `prompt.md` and `prompt_minimal.md` say RoboLang 1.9; the grammar-only
  prompt pastes `spec/grammar-v1.9.ebnf` after the 1.8 one (seven files). The grammar file states what 1.9 cannot say
  (another day, repeating, arithmetic, other formats), as PR #151's erratum E5 requires for 1.8.
- `bench/score.py`: `clock_ok` and `time_replaced` per answer, and two rows in the measures table per model (`clock_ok`,
  `time_replaced_silently`), reported like `actions_ok` and `action_replaced_silently`.
- `docs/language-reference.md` (and its packaged copy) teaches 1.9: a "Clock readings" section (`measure r time|elapsed`, time
  literals, comparisons, `clock_time` in `wait until`, one day, the world line, `infeasible` for a passed time, a short timeout or
  no clock, and what stays out), `clock_time` and `clock_elapsed` in the reserved words, the plan statements, the conditions,
  `E105`, `S010`, `S237`, `S238` in the code table, and the header. "Clock readings" is removed from "Not in the language".

### Language facts the labels now meet

- **K3 catches the decoy.** `k10_decoy_11_pm`: a program that writes `clock_time >= 11:00` (12-hour confusion) at the 22:20
  start is not a program that moves at once with no error, as the pre-registration noted: the wait names a time before the
  start, so the check before the run fails it (`S010`), and `check_answer` returns an invalid answer.
- **A wait in an `if` branch is not checked by K3** (the branch may not run): "if it is already after 9, go now" is written
  with `measure` and `if`.
- **No hour unit.** 0042 writes `timeout 12 h` (T5) and `1 h` (T2: "durations as they are written today"), but the language has
  `ms`, `s` and `min` only. 1.9 does not add a unit (it would change B7's economy): a request in hours is written in minutes
  (`60 min`, `120 min`, `720 min`), the reference says so, and `clock_ok` compares a timeout as a duration, so a label of `1 h`
  and a program of `60 min` agree. A model that writes `h` gets `E206` and the repair loop. If the owner wants the unit, it is
  one token in `DURATION_UNITS` and a revisit of this erratum.
- **One day**: a `time` reading once the clock has reached the end of the start day fails (`S238`); a wait that would cross
  midnight ends there. The zone is recorded and checked by form only (no zone database), so a daylight-saving change during a
  run is not modelled.

### Freeze

`docs/evidence/bench-freeze-v2.json` is regenerated for all of the above (the manifest was deleted and written again with
`python tools/bench_freeze.py`; `--check` passes).

## Atlas

`bench/atlas.json`: G-CLOCK lists no task (everything 1.9 expresses left it, as G-ACT's list did at 0040 L4), with a scope
note; it stays **open** and closes at L6, after the acceptance run (L5). Snapshot "I3 L4: RoboLang 1.9 built and taught" in
the history; `docs/plans/humanoid-activities.md` regenerated.

## Erratum K2 (2026-11-17, after the L5 run): `k01_scan_stamp` and `f_clock_scan_stamp` accept two readings

"Inspect box1 on shelf_a …, then send the time of day" asks for no end state. The label required only the
reference's `robot_at arm1 shelf_a`. `object_at box1 shelf_a` is equally defensible: the box stays where it was
inspected. That is the reading of `k_inspect_thrice` in set 2. Both are now accepted as `goals`. Haiku wrote that
reading in 3/3 answers, and Sonnet wrote `object_exists box1`, which stays wrong. The L5 answers are re-scored
offline, with no new request. `docs/evidence/bench-freeze-v2.json` is regenerated.
