# Benchmark errata for RoboLang 2.0 — 2026-11-18

Decision record [0041](../../spec/decisions/0041-mission-layer.md), iteration I4, step L3 (pre-register set 9
"missions"). At that step 2.0 was **not built** and the reference and both prompts still taught RoboLang 1.9; erratum M2
(L4, below) records the build and the teaching. This file lists every
benchmark input that changed, so that `docs/evidence/bench-freeze-v2.json` can be regenerated with a recorded reason
(the previous manifest is `8b75d6bc…`, in git history; the earlier errata are [1.5](bench-errata-1-5.md),
[1.6](bench-errata-1-6.md), [1.7](bench-errata-1-7.md), [1.8](bench-errata-1-8.md) and [1.9](bench-errata-1-9.md)).
Later entries (L4: labels, references, scorer, prompts) are added here. No paid run was made: the baseline (1.9) is
confirmed by the owner before it starts.

## Erratum M1 (2026-11-18, L3): set 9 "missions" added

Plan and proposed bounds: [bench-9-missions.md](../plans/bench-9-missions.md) (draft for the owner; the bounds are
placeholders to fix with the baseline, rule 6; 14 open points where 0041 does not decide what a label needs). Nothing
standing was changed: sets 1 to 8, the prompts, the reference, the grammar, the scorer and every existing world are as
before. (0041 calls this set 8; the clock set took that number, so it is set 9.)

| Input | Change |
|---|---|
| `bench/tasks_missions/` | **new**, 26 tasks: 10 `mission` programs, 5 `infeasible`, 7 `inexpressible`, 4 `control`. `expect` is the 1.9 label, `expect_2_0` the 2.0 one (a program has an `expected_mission` and the mission outcome of each path) |
| `bench/reference_missions/` | **new**: a reference (`.rob`, `.json`) for each of the 4 controls; the obvious attempt (`.attempt.rob`, `.attempt.json`, fails with `E201`; the mission part is marked `// @mission`) for each of the other 22 |
| `bench/lib.py` | `MISSIONS_TASKS_DIR`, `MISSIONS_REFERENCE_DIR`, and `"missions"` in `SETS` |
| `tests/test_bench_missions.py` | **new** |
| `bench/atlas.json` | the 22 tasks that 1.9 declines are in a gap: G-LOOP 20 (the 10 programs, the 5 infeasible, `x01`, `x02`, `x04`, `x06`, `x07`), G-MSG 1 (`x03`), G-CHOOSE 1 (`x05`); snapshot "I4 L3" in the history |
| `docs/plans/humanoid-activities.md` | regenerated from the atlas |
| `.github/workflows/benchmark.yml` | the per-set booleans `set3`, `set4`, `future`, `regions`, `actions`, `clock` (10 inputs, GitHub's limit) are replaced by one string input `set_only` (`set3`, `set4`, `future`, `regions`, `actions`, `clock`, `missions`; empty is the usual run). Every earlier set's run command is unchanged; the weekly step runs only when `set_only` is empty. New step for `missions` (full and minimal prompts, 3 repeats, Sonnet and Haiku, `--max-usd 3` each, not gated); excluded from the per-PR and weekly runs; `missions`, `missions-minimal` in the summary loop. The workflow now has 5 `workflow_dispatch` inputs |
| `tests/test_bench_actions.py`, `test_bench_regions.py`, `test_bench_clock.py`, `test_bench_future.py` | read `inputs.set_only == '<set>'` instead of `inputs.<set>`; the declared-input check is for `set_only` |
| `bench/README.md`, `docs/LANGUAGE_ROADMAP.md` | a section for set 9; the earlier `<set>: true` dispatches read `set_only: <set>`; the I4 row |

The evidence files of earlier runs name the old inputs (`regions: true`, `actions: true`, `clock: true`): they record
what was dispatched at the time and are not changed.

Not copied, as in sets 6 to 8: the 15 standing G-LOOP tasks (`x_every_hour`, `x_keep_feeding`, `x_retry_grasp`,
`x_wait_forever`, `u03_until_stop`, `u04_back_and_forth`, `x04_every_minute`, `f10_retry_until_works`,
`f13_retry_every_minute`, `x_all_parts_ever`, `x_retry_until_grasped`, `x_shuttle_until_five`, `u18_every_morning`,
`x04_open_until_open`, `x01_every_night_round`). They stay in their sets, unchanged, until L4 relabels them. None states
a bound, so under 0041 M2 none becomes a program (plan, "What this means for the standing tasks").

## Freeze

`docs/evidence/bench-freeze-v2.json` is regenerated for all of the above.

## Erratum W1 (2026-11-18): the paid answers are always kept

The first baseline run of set 9 ([run 44252920152](https://github.com/rokorobot/RoboLang/actions/runs/44252920152))
answered every request, but GitHub cancelled the job while it was scoring. Because of that the artifact upload was
skipped and the answers were lost; no log survived. The upload now runs `if: always()`, and scoring has a 25-minute
timeout and prints each directory as it starts. The baseline is run again.
The second run ([run 44261560831](https://github.com/rokorobot/RoboLang/actions/runs/44261560831)) hung in the same
way. Scoring ran past its own 25-minute timeout, which suggests the runner itself had died, likely out of memory, and
nothing was uploaded. The orchestrator cancelled it. The raw answers are now uploaded as `benchmark-answers`
**before** scoring, so a failure while scoring can be reproduced offline.

## Erratum W2 (2026-11-18): exporting a goal is linear in the plan, not in its paths

The cause of both lost runs was not GitHub. Scoring the grammar-only answers ran out of memory.
`bridge.goal_atoms` and `bridge.goal_alternatives` walked **every path** of a plan, and each `if` or failure handler
doubles the paths. Haiku's 1.9 answer to `m06_carry_until_there` had 18 handled steps. The same walk serves
`robolang export-goal` and `delegate`: on that valid task, `export-goal` took 15 s and then failed out of memory.
The new `bridge.distinct_path_goals` gives the same distinct goals in the same order (a test checks it against
`path_goals`), memoised on the position in the plan, what follows it, and the goal so far. It finishes in 0.16 s.
`tests/test_lang15.py` has a regression test with 40 handled steps. The results of earlier sets are unaffected:
they never had such plans. This is a fix to the package and goes into the next release.

## Erratum E5 (L4): the grammar file says what 2.0 cannot say

The grammar file is the grammar-only prompt (erratum E5 of [1.8](bench-errata-1-8.md)). `spec/grammar-v2.0.ebnf` ends with
a normative comment on what the mission layer cannot say: no bound ("forever", "until it works"), triggers ("every hour",
"at 06:00", "on a message"), nesting, choosing the next object at run time, hours in a bound (`480 min`), and a duration
that no run can fit. It also says that a fixed short count and a bounded retry of one step are programs (a task with the
steps written out, a 1.5 handler), so the grammar-only prompt does not over-decline them. `tests/test_lang20.py` checks
the phrases.

## Erratum M2 (2026-11-18, L4): RoboLang 2.0 built and taught

Decision record [0041](../../spec/decisions/0041-mission-layer.md), step L4 of iteration I4 (build and teach 2.0). From
this change on, the reference and both prompts teach RoboLang 2.0. Every benchmark input that changed is listed here. The
set 9 baseline under 1.9, taken before these changes, is [bench-set9-baseline.md](bench-set9-baseline.md). **Not
changed:** decision 0041, the 26 tasks' prompts and 1.9 labels, the bounds B1–B9 ([plan](../plans/bench-9-missions.md)).
No paid run was made for this change: the acceptance run (L5) is confirmed by the owner before it starts.

| Input | Change |
|---|---|
| `docs/language-reference.md` (and the packaged copy) | RoboLang 2.0: a "Missions" section (the syntax, `run`, the bound, `until`, `on failure`, time and outcomes, what stays out) and every example is `robolang 2.0;`. **Teaching fix (owner, 2026-11-18):** a fixed count ("three round trips") is one task with the steps repeated, or a mission with that run count (both accepted, clarification 10), and a bounded number of retries of one step is a 1.5 failure handler: models over-declined `c01_three_round_trips` (set 9's baseline), `c_count_two_totes` and `c_retry_flag_carry_on` (set "future"). It is in the intro, in "Failure handlers" and in "Missions", with a worked example |
| `bench/prompt.md`, `bench/prompt_minimal.md` | "You write RoboLang 2.0 programs"; the answer has an optional eighth key `mission` (the name of the mission; `bindings` then bind its parameters and `task` is empty); a small fixed count needs no mission and no decline. The grammar-only prompt pastes eight files, the last `spec/grammar-v2.0.ebnf` |
| `bench/lib.py` | `GRAMMAR_FILES` has `grammar-v2.0.ebnf`; `ANSWER_SCHEMA` has the optional `mission`; `parse_answer` reads it (`""` when left out, so every saved answer still reads); `check_answer` binds a mission's parameters as `simulate --mission` does; `judge_mission`, `mission_ok`, `bound_dropped_or_invented`, `repetition_replaced`, `mission_added`, `policy_invented` |
| `bench/score.py` | the rows and tables for `mission_ok`, `mission_paths_ok` (B1), `repetition_replaced_silently` (B8), `bound_dropped_or_invented` (B9), `no_mission_added` (B5), `policy_invented` (clarification 11); the docstring says what each is |
| `bench/run.py` | no change: the answer is read by `lib.parse_answer`, and the repair message names the mission too |
| `bench/tasks_missions/` | **Labels:** `expect` is the 2.0 label, `expect_1_9` the old `expect` (the 1.9 label, unchanged), `expect_2_0` the pre-registered label (`expected_mission`, `paths`). `x06_until_it_is_cool` and `x07_until_ten_to_ten` are programs (clarification 2), group `mission_until`; `m09_open_the_oven_door` path 3 (every attempt fails under `continue`) is `failed`, not `bounded` (clarification 6, approved after the label was written); `policy_stated: false` on `m03`, `m05`, `m07`, `m10`, `x06`, `x07` (clarification 11); `c01` has an `accepted_mission` (clarification 10); `x02`'s gap is G-TRIGGER |
| `bench/tasks_clock/x01_every_night_round.json` | `gap` G-LOOP → G-TRIGGER (clarification 14); its labels are unchanged |
| `bench/reference_missions/` | **new:** a 2.0 reference (`.rob`, `.json`) for each of `m01`–`m10`, `x06`, `x07`, and the mission form of `c01` (`c01_three_round_trips.mission.*`); the controls' references are `robolang 2.0;`; the 1.9 attempts stay as the evidence of the baseline |
| `tests/test_bench_missions.py`, `tests/test_reference.py`, `tests/test_bench.py`, `tests/test_bench_hard.py`, `tests/test_bench_clock.py` | updated for the labels, the prompts, the schema (one optional key) and the gaps; every 2.0 reference is checked on every path |
| `bench/atlas.json`, `docs/plans/humanoid-activities.md` | the far gap **G-TRIGGER** is new; snapshot "I4 L4: RoboLang 2.0 built and taught" (G-LOOP stays open until L5 and closes at L6) |
| `docs/LANGUAGE_ROADMAP.md`, `bench/README.md` | the I4 row: "L4 done; L5 acceptance run next"; a section for set 9 and the answer's `mission` key |

### Labels of set 9

| Task | Before (`expect`) | After (`expect`) |
|---|---|---|
| `m01`–`m10` (10 `mission` tasks) | cannot / inexpressible | **program** (`expect_2_0.expected_mission`, paths as pre-registered; `m09` path 3 `failed`) |
| `x06_until_it_is_cool`, `x07_until_ten_to_ten` | cannot / inexpressible | **program** (clarification 2: an `until` may read a live value or the clock; a time of day is a hard bound). Their `expected_mission` and two paths each are new: `x06` completes at the second run when the pan cools at 100 s, and is `bounded` at 30 runs when it does not; `x07` completes after the sixth round (22:00) and is `failed` when round 3 fails |
| `i01`–`i05` (5 infeasible) | cannot / inexpressible | cannot / **infeasible** (`because` M2, M4, M7, M4, M2 as pre-registered) |
| `x01`–`x05` | cannot / inexpressible | unchanged (`x01` no bound, `x02` time trigger, `x03` message trigger, `x04` nesting, `x05` choosing the object) |
| `c01`–`c04` (4 controls) | program | unchanged; `c01` also accepts a mission with three runs; `c02` to `c04` want no mission |

**B1 is counted on the 10 `mission` tasks** (`m01`–`m10`, at least 27/30 as approved); `x06` and `x07` are a group of their
own (`mission_until`) and are covered by B9 and by the report, so the approved bound is not widened. B3 is over the 5
tasks that are still inexpressible.

### The standing tasks that 2.0 might change (the G-LOOP list of the atlas)

Rule: a request becomes a program only when it **states a bound** a mission can carry (a count, a duration, a retry count,
or a time of day to end at) and every other part of it is expressible. Each of the 15 was read against it. **None of the 15
becomes a program**; two of them state a bound and are explained last.

| Task (set) | Request (short) | Label | Gap | Why |
|---|---|---|---|---|
| `x_every_hour` (hard) | every hour, as long as the cell is in service | inexpressible | G-TRIGGER (from G-LOOP) | a time trigger, and no bound |
| `x_keep_feeding` (hard) | keep moving every new part that arrives, as long as the line runs | inexpressible | G-TRIGGER | an event trigger ("that arrives"), and no bound |
| `x_retry_grasp` (hard) | try the grasp again as many times as it takes | inexpressible | G-LOOP | no retry count: "as many times as it takes" |
| `x_wait_forever` (hard) | keep checking the aisle for as long as it takes, never give up | inexpressible | G-LOOP | no bound at all (M2) |
| `u03_until_stop` (ask) | keep moving every vial that arrives, until I tell you to stop | inexpressible | G-TRIGGER | an event trigger; an operator stop is not a bound |
| `u04_back_and_forth` (ask) | keep driving back and forth as long as service lasts | inexpressible | G-LOOP | no bound: "as long as service lasts" |
| `u18_every_morning` (ask) | every morning at 06:00 | inexpressible | G-TRIGGER | a time trigger |
| `x_all_parts_ever` (branch) | whenever a new part appears during the shift | inexpressible | G-TRIGGER | an event trigger |
| `x_retry_until_grasped` (branch) | try again until it works | inexpressible | G-LOOP | no retry count |
| `x_shuttle_until_five` (branch) | keep moving parts as they arrive, until 17:00 | inexpressible | G-CHOOSE (from G-LOOP) | **it states a bound** (a time of day, clarification 2), but the next part is chosen at run time from parts that arrive: the same shape as set 9's `x05_every_new_part` (G-CHOOSE), whose bound (8 hours) is stated too. The world has no clock either (`S237`). Reported to the owner: the brief named this task as a program |
| `f10_retry_until_works` (failure) | keep trying until it works | inexpressible | G-LOOP | no retry count |
| `f13_retry_every_minute` (failure) | try again every minute until the end of the shift | inexpressible | G-LOOP | "the end of the shift" is not a time or a duration |
| `x04_every_minute` (values) | check every minute, report each reading, until it is below 40 °C | inexpressible | G-LOOP | the `until` is a value (allowed), but no count or duration bounds the repeat (M2) |
| `x04_open_until_open` (actions) | keep trying to open the door until it is open | inexpressible | G-LOOP | no bound |
| `x01_every_night_round` (clock) | every night at 23:30 | inexpressible | G-TRIGGER | a time trigger |

The ids and prompts are unchanged. The atlas now holds `x_retry_grasp`, `x_wait_forever`, `u04_back_and_forth`,
`x_retry_until_grasped`, `f10_retry_until_works`, `f13_retry_every_minute`, `x04_every_minute`, `x04_open_until_open`,
`x01_never_stop` and `x04_rounds_inside_rounds` in G-LOOP (a repeat with no bound, and nesting: what 2.0 still cannot say
that is not a trigger); the seven triggers in the new far gap G-TRIGGER (time, message and self-state triggers on
missions, M6: I5, I6, I8); `x03_for_each_go_message` stays in G-MSG. No sub-activity of the atlas names G-TRIGGER in
`needs` yet: moving the activities' needs that are about triggers there, and closing G-LOOP, is L6.

## Freeze

`docs/evidence/bench-freeze-v2.json` is regenerated for all of the above (the previous manifest is in git history).

## Erratum W3 (owner, 2026-11-18): an unknown `until` fails the mission

An unknown `until` (`S402`) now ends the mission `failed` (`until_unknown`), not `stopped`. `stopped` is kept for an
operator stop or a lease end. No set-9 label used the old outcome. `docs/evidence/bench-freeze-v2.json` is regenerated.
