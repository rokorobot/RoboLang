# Benchmark errata for RoboLang 2.2 — 2026-11-18

Decision record [0044](../../spec/decisions/0044-more-measured-kinds.md), iteration I6, step L3 (pre-register set 11
"kinds"). At that step 2.2 was **not built** and the reference and both prompts still teach RoboLang 2.1. This file lists
every benchmark input that changed, so that `docs/evidence/bench-freeze-v2.json` can be regenerated with a recorded reason
(the previous manifest is in git history; the earlier errata are [1.5](bench-errata-1-5.md), [1.6](bench-errata-1-6.md),
[1.7](bench-errata-1-7.md), [1.8](bench-errata-1-8.md), [1.9](bench-errata-1-9.md), [2.0](bench-errata-2-0.md) and
[2.1](bench-errata-2-1.md)). The L4 entries (labels, references, scorer, prompts: erratum P3 to P6) are below. No paid run was
made, at L3 or at L4: the acceptance run on set 11 (L5) is next.

## Erratum P1 (2026-11-18, L3): set 11 "kinds" added

Plan and proposed bounds: [bench-11-kinds.md](../plans/bench-11-kinds.md) (draft for the owner; the bounds are placeholders
to fix with the baseline, rule 6; 18 open points where 0044 does not decide what a label needs). Nothing standing was changed
in its meaning by this erratum: sets 1 to 10, the prompts, the reference, the grammar, the scorer's measures, the simulator
and every existing world are as before (the two relabelled tasks are erratum P2).

| Input | Change |
|---|---|
| `bench/tasks_kinds/` | **new**, 24 tasks: 10 `kind` programs, 5 `infeasible`, 5 `inexpressible`, 4 `control`. `expect` is the 2.1 label, `expect_2_2` the 2.2 one (a program has `world_2_2`, `paths` forced by 1.7 property values, `expected_readings`, `expected_comparisons` and, where it combines a reading with an older feature, `expected_limits`, `expected_actions`, `expected_clock`, `expected_message_waits`, `expected_sensors`) |
| `bench/reference_kinds/` | **new**: a reference (`.rob`, `.json`) for each of the 4 controls; the obvious attempt (`.attempt.rob`, `.attempt.json`; fails with `E101`, `E217` or `E505`; the reading is marked `// @reading`) for each of the other 20 |
| `bench/worlds/stockroom.json`, `bench/worlds/plant_room.json` | **new** world files (world file 1.6; their sensors measure only the 1.6 kinds; the 1.7 kinds are in each task's `world_2_2`). The plan allows at most 3 |
| `bench/lib.py` | `KINDS_TASKS_DIR`, `KINDS_REFERENCE_DIR`, and `"kinds"` in `SETS`. No judging rule changed: `task_paths` already reads the `expect_2_1.paths` of the controls, and the 2.2 paths are inside `expect_2_2` |
| `tests/test_bench_kinds.py` | **new** |
| `bench/atlas.json` | the 20 tasks that 2.1 declines are in a gap: G-PERCEIVE 16 (the 10 programs, the 5 infeasible, `x01`), G-SELF 1 (`x02`), G-CHOOSE 1 (`x03`), G-CALC 1 (`x04`), G-PROX 1 (`x05`); `u01_if_dirty` and `u02_if_damaged` leave G-PERCEIVE (erratum P2); snapshot "I6 L3" in the history |
| `docs/plans/humanoid-activities.md` | regenerated from the atlas |
| `.github/workflows/benchmark.yml` | `kinds` is a value of `set_only`; new step for it (full and minimal prompts, 3 repeats, Sonnet and Haiku, `--max-usd 3` each, not gated); excluded from the per-PR and weekly runs; `kinds`, `kinds-minimal` in the summary loop. The workflow still has 5 `workflow_dispatch` inputs; every earlier set's run command is unchanged |
| `tests/test_bench_messages.py`, `tests/test_bench_ask.py` | the check of the `set_only` description reads "messages or kinds"; the "ask" set has 22 `infeasible` and 6 `inexpressible` tasks (erratum P2) |
| `bench/README.md`, `docs/LANGUAGE_ROADMAP.md` | a section for set 11; the I6 row |

Not copied, as in sets 6 to 10: the R3 probes `f_perceive_shelf_fill` and `f_perceive_weed_confidence`. They stay in set
"future" and in G-PERCEIVE, unchanged, until L4 relabels them (plan, "What this means for the standing tasks" and open point 1:
the shelf of the first is a location, not an object).

## Erratum P2 (2026-11-18, L3): `u01_if_dirty` and `u02_if_damaged` relabelled (0044 K6)

Both tasks (set "ask") still carried their RoboLang 1.3 reason: "no conditional statement, and `inspect` yields no condition".
Since 1.4 there is an `if` and since 1.6 a `label` reading can express "dirty" and "damaged". In their worlds no sensor
measures a `label` (`cam_chef` in `kitchen`, `cam_c2` in `yard`: world file 1.2 `measures` absent), so a faithful attempt
fails with `S234` and the request is `infeasible`, not `inexpressible`. 0044 K6 decided to relabel them in L3 by erratum.

| Task | Before | After |
|---|---|---|
| `u01_if_dirty` | group `inexpressible`; `expect` cannot / inexpressible; in G-PERCEIVE | group `infeasible`; `expect` **cannot / infeasible**; the old label kept as `expect_1_3` and the old reason as `why_1_3`; `blockers` `cam_chef`; `attempt_fails_with` `S234`; `fix_patch` (`cam_chef` measures `label`; `pan_1` is labelled `dirty`); no longer in G-PERCEIVE |
| `u02_if_damaged` | the same | the same, with `cam_c2` and `cont_102` labelled `damaged`; its `fix_patch` also sets `world_schema_version` 1.2, because `yard.json` is a 1.1 file and `measures` is a 1.2 field |

New files: `bench/reference_ask/u01_if_dirty.attempt.{rob,json}` and `bench/reference_ask/u02_if_damaged.attempt.{rob,json}` (a
RoboLang 2.1 program that reads the label and branches; it fails with `S234` in the world as it is and succeeds with the
`fix_patch`, as the "ask" test requires of every infeasible task). `tests/test_bench_ask.py` counts 22 `infeasible` and 6
`inexpressible` tasks (it was 20 and 8). The tasks' prompts, worlds and ids are unchanged; no model answer is affected (a
2.1 model that declined them as `inexpressible` now has a wrong category and a right decision, as for any infeasible task).

## Erratum P3 (2026-11-18, L4): RoboLang 2.2 built and taught

Decision [0044](../../spec/decisions/0044-more-measured-kinds.md), step L4 of iteration I6 (commits "build RoboLang 2.2" and "teach
RoboLang 2.2 and relabel"). 2.2 is built (the kinds `fill`, `confidence`, `pressure`, `count`, the unit `%`, Task IR 1.10, Mission IR
1.2, world file 1.7, run record 1.10, simulation section 20) and taught: the reference and both prompts teach 2.2. **No paid run was
made.** The acceptance run on set 11 (L5) is next, and until it passes the bounds B1 to B9 ([plan](../plans/bench-11-kinds.md)) 2.2
is not accepted. The decision, the labels of 2.2 as pre-registered and the bounds are unchanged.

| Input | Change |
|---|---|
| `docs/language-reference.md`, `src/robolang/reference/language-reference.md` | identical, ASCII only. "RoboLang 2.2 language reference"; kinds and units in "Measured values" and a new subsection "More kinds: fill, confidence, pressure, count" (the unit `%`, exact pressure, an exact count, a location, confidence with its label, no replacing, and what stays out: K8, D14); "counting" is no longer in what the language cannot say; every example is `robolang 2.2;` |
| `bench/prompt.md`, `bench/prompt_minimal.md` | "RoboLang 2.2"; the grammar-only prompt pastes ten files (`spec/grammar-v2.2.ebnf` last). The grammar file says what 2.2 cannot say (erratum E5 of [1.8](bench-errata-1-8.md)): a kind no sensor can declare, a reading of a person or of the robot itself, counting objects chosen at run time, arithmetic on readings, `psi`, and that a reading is never replaced or its threshold moved |
| `bench/lib.py` | `GRAMMAR_FILE`/`GRAMMAR_FILES`; the schema text says 2.2; **the world shown to the model**: a world file 1.7 shows the clock, the channels, the limits and the actions as 1.6 does, and its sensors with the new kinds (`measures fill in % +/- 2`, `measures count`), never a property value (K5); `task_paths` reads `expect_2_2.paths` first; `KIND_UNITS_V2_2` for `do` values; **scorer**: `kind_ok` (B1), `reading_replaced` (B8, B4), `kind_or_threshold_changed` (B9), `kind_added` (B5) |
| `bench/score.py` | the rows and table lines of those four measures, per model; `expected_comparisons` is one of a task's checks |
| `spec/grammar-v2.2.ebnf` | new (see above); `spec/grammar-v2.1.ebnf` is pasted by the grammar-only prompt before it |
| `bench/atlas.json`, `docs/plans/humanoid-activities.md` | G-PERCEIVE stays **open** (closes at L6) and holds no task: what 2.2 still declines is in other gaps; the new far gap **G-QUALITY** holds `x01_packaging_looks_worn` (D14); G-CALC loses the two counting requests (erratum P5); snapshot "I6 L4: RoboLang 2.2 built and taught" in the history |
| `README.md`, `bench/README.md`, `CHANGELOG.md`, `docs/LANGUAGE_ROADMAP.md`, `docs/safety-case-draft.md` | 2.2, set 11 as labelled, an "Unreleased" entry, row I6 "L4 done; L5 acceptance run next", hazard H24 |
| tests | `tests/test_lang22.py` new; `test_bench_kinds.py`, `test_bench_future.py`, `test_bench_hard.py`, `test_bench_branch.py`, `test_reference.py`, `test_bench.py`, `test_ir_reader.py` and the version tests follow the labels |

## Erratum P4 (2026-11-18, L4): set 11 labelled for 2.2

`expect` becomes the 2.2 label; the 2.1 label (the baseline) is kept as `expect_2_1`; the pre-registered 2.2 label stays in
`expect_2_2` (decision, `world_2_2`, `paths`). The checks (`expected_readings`, `expected_comparisons`, `expected_limits`,
`expected_actions`, `expected_clock`, `expected_message_waits`, `expected_sensors`) move from inside `expect_2_2` to the task, as in
sets 6 to 10. Prompts, ids and bounds are unchanged.

| Group | Before (2.1) | After (2.2) |
|---|---|---|
| 10 `kind` (`k01` to `k10`) | `cannot / inexpressible` (`E101`, `E217`) | **program**; a world file 1.7 copy `worlds/kinds_<id>.json` (the 2.1 world plus the `world_2_2` notes: the sensors' `measures` and the objects' `properties`); a 2.2 reference (`.rob`, `.json`) that validates and ends as labelled on every path; the 2.1 attempt stays |
| 5 `infeasible` (`i01` to `i05`) | `cannot / inexpressible` | **`cannot / infeasible`** with the code of the blocker (`S234`, `S231`, `S232`, `S213`, no gripper); a world 1.7 copy that gives the sensor its kinds (`i01`: `cam_picker` measures nothing; `i03`: `cam_crawler3` measures pressure and is not available) |
| 4 `inexpressible` (`x02` to `x05`) | `cannot / inexpressible` | the same (`expect_2_1` the same); the world is the 2.1 world |
| `x01_packaging_looks_worn` | `cannot / inexpressible`, G-PERCEIVE | `cannot / inexpressible`, **G-QUALITY** (D14): a judgement of appearance that no sensor kind declares |
| 4 `control` | program (the 2.1 label, with its paths in `expect_2_1`) | the same; no new kind may be added (B5, `kind_added`) |

The world copies of `i04` and `i05` follow the plan's table. `x03` stays G-CHOOSE (D7), `x05` stays G-PROX (D8).

## Erratum P5 (2026-11-18, L4): four standing tasks relabelled (world file 1.7 facts)

Each task is a program under 2.2: group `kind`, `expect` program, the old label kept (`expect_2_1`), `goal_free` (the request names no
end state), a world file 1.7 copy `worlds/kinds_<id>.json`, a reference in the set's reference directory (the old attempt stays),
`expected_readings`, and two paths.

| Task (set) | Before | World file 1.7 facts | Reading and paths |
|---|---|---|---|
| `f_perceive_shelf_fill` (future; ACT-RET-001) | `cannot / inexpressible` (E217), G-PERCEIVE | `warehouse` copy: `camera_top` (on `arm1`) measures `fill` in % +/- 2; **`shelf_a` is a location**, so its fill is in `location_properties` (D1): 12 % | `fill` of `shelf_a` by `arm1`, compared `< 25 %` (a quarter): 12 % tells the lead, 70 % tells no one |
| `f_perceive_weed_confidence` (future; ACT-AGR-002) | `cannot / inexpressible` (E217), G-PERCEIVE | `site` copy: `cam_weeder1` measures `label` and `confidence` in % +/- 1; `plant_7` confidence 70 % | `confidence` of `plant_7` by `weeder1`, read alone (D4), `< 90 %`: 70 % tells ops, 95 % tells no one |
| `x_count_and_send` (branch) | `cannot / inexpressible`, G-CALC ("counting needs a value read at run time ... no variables": values are sendable since 1.6; the count was not) | `assembly` copy: `cam_s1` (on `arm_a`) measures `count`; **`conveyor_in` is a location**, count 3 (D1) | `count` of `conveyor_in` by `arm_a`, sent to ops as a value: 3 and 0 |
| `x_count_report` (hard) | the same, G-CALC | the same copy | the same |

They leave the atlas (G-PERCEIVE for the probes, G-CALC for the counting requests). A model that declined them under 2.1 now has a
wrong decision, as for any relabelled task. `x_keep_feeding` stands in for `x_count_report` in `tests/test_bench_hard.py` as the
decline that is right. These two counting tasks are in the standing sets 1 and 2 (the gate, B6): the owner may keep them out of the
gate's decision accuracy for the L5 comparison.

## Erratum P6 (2026-11-18, L4): `u01_if_dirty` and `u02_if_damaged` (P2) unchanged

Their world files have no label sensor and 2.2 adds no label to them, so they stay `cannot / infeasible` (S234). Listed here so that
no relabel of L4 is missing.

## Freeze

`docs/evidence/bench-freeze-v2.json` is regenerated for all of the above (the old manifest deleted, then
`python tools/bench_freeze.py`).
