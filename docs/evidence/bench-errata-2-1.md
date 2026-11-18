# Benchmark errata for RoboLang 2.1 — 2026-11-18

Decision record [0043](../../spec/decisions/0043-incoming-messages.md), iteration I5, step L3 (pre-register set 10
"messages"). At that step 2.1 was **not built** and the reference and both prompts still teach RoboLang 2.0. This file lists
every benchmark input that changed, so that `docs/evidence/bench-freeze-v2.json` can be regenerated with a recorded reason
(the previous manifest is in git history; the earlier errata are [1.5](bench-errata-1-5.md), [1.6](bench-errata-1-6.md),
[1.7](bench-errata-1-7.md), [1.8](bench-errata-1-8.md), [1.9](bench-errata-1-9.md) and [2.0](bench-errata-2-0.md)). Later entries
(L4: labels, references, scorer, prompts) are added here: erratum N2 below. No paid run was made: the baseline (2.0) is confirmed by the owner
before it starts.

## Erratum N1 (2026-11-18, L3): set 10 "messages" added

Plan and proposed bounds: [bench-10-messages.md](../plans/bench-10-messages.md) (draft for the owner; the bounds are
placeholders to fix with the baseline, rule 6; 14 open points where 0043 does not decide what a label needs). Nothing
standing was changed in its meaning: sets 1 to 9, the prompts, the reference, the grammar, the scorer's measures, the
simulator and every existing world are as before.

| Input | Change |
|---|---|
| `bench/tasks_messages/` | **new**, 24 tasks: 10 `message` programs, 5 `infeasible`, 5 `inexpressible`, 4 `control`. `expect` is the 2.0 label, `expect_2_1` the 2.1 one (a program has `expected_message_waits`, `world_2_1` with the declared `channels`, and `paths` forced by incoming `messages`) |
| `bench/reference_messages/` | **new**: a reference (`.rob`, `.json`) for each of the 4 controls; the obvious attempt (`.attempt.rob`, `.attempt.json`, fails with `E211`, or `E201` for `x01` and `x02`; the message part is marked `// @message` or `// @message-tail`) for each of the other 20 |
| `bench/lib.py` | `MESSAGES_TASKS_DIR`, `MESSAGES_REFERENCE_DIR`, and `"messages"` in `SETS`; `task_paths` also reads `expect_2_0.paths` of a task whose label has no `expected_mission` (the paths of the set's controls; set 9's missions keep their paths out of it). No judging rule changed |
| `tests/test_bench_messages.py` | **new** |
| `tests/test_bench_missions.py` | the check that the standing G-TRIGGER tasks are in that gap is now a subset check, because set 10's `x02_each_time_the_line_says_ready` is in it too |
| `bench/atlas.json` | the 20 tasks that 2.0 declines are in a gap: G-MSG 16 (the 10 programs, the 5 infeasible, `x05`), G-CALC 2 (`x03`, `x04`), G-TRIGGER 1 (`x02`), and the new far gap **G-DELIVERY** 1 (`x01`, S7); snapshot "I5 L3" in the history |
| `docs/plans/humanoid-activities.md` | regenerated from the atlas |
| `.github/workflows/benchmark.yml` | `messages` is a value of `set_only`; new step for it (full and minimal prompts, 3 repeats, Sonnet and Haiku, `--max-usd 3` each, not gated); excluded from the per-PR and weekly runs; `messages`, `messages-minimal` in the summary loop. The workflow still has 5 `workflow_dispatch` inputs; every earlier set's run command is unchanged |
| `bench/README.md`, `docs/LANGUAGE_ROADMAP.md` | a section for set 10; the I5 row |

Not copied, as in sets 6 to 9: the standing message tasks (`f_msg_line_docked`, `f_msg_nurse_confirm`, `f_msg_wind_clear`,
`x_wait_message`, `u11_wait_go`, `u12_radio_clearance`, `x_wait_ready_message`, `f12_message_delivery`) and set 9's
`x03_for_each_go_message`. They stay in their sets, unchanged, until L4 relabels them (plan, "What this means for the standing
tasks").

## Freeze

`docs/evidence/bench-freeze-v2.json` is regenerated for all of the above (the old manifest deleted, then
`python tools/bench_freeze.py`).

## Erratum C (owner, 2026-11-18): gaps after the clarifications of 0043

- `f12_message_delivery` moves from G-MSG to G-DELIVERY, which is now scope `out` (S7, C13).
- Set 9's `x03_for_each_go_message` moves from G-MSG to G-TRIGGER (S10).

Labels are unchanged. `x05_a_message_in_the_goal` becomes a program at L4 (C14).
`docs/evidence/bench-freeze-v2.json` is regenerated.

## Erratum N2 (2026-11-18, L4): RoboLang 2.1 built and taught, set 10 and the message tasks relabelled

Decision [0043](../../spec/decisions/0043-incoming-messages.md), step L4 of iteration I5 (commits "build RoboLang 2.1" and "teach
RoboLang 2.1 and relabel"). 2.1 is built (`message_received`, Task IR 1.9, Mission IR 1.1, world file 1.6, run record 1.9,
simulation section 19) and taught: the reference and both prompts teach 2.1. **No paid run was made.** The acceptance run on set 10
(L5) is next, and until it passes the bounds B1 to B9 ([plan](../plans/bench-10-messages.md)) 2.1 is not accepted. The decision,
the 2.0 labels and the bounds are unchanged.

| Input | Change |
|---|---|
| `docs/language-reference.md`, `src/robolang/reference/language-reference.md` | identical, ASCII only. "RoboLang 2.1 language reference"; a new section "Incoming messages"; the condition in the table, in `until`, in "Plan statements" and in the codes (`E327`, `S011`); every example is `robolang 2.1;`; "message waits" is no longer in what the language cannot say, and delivery, the content of a message and patterns are |
| `bench/prompt.md`, `bench/prompt_minimal.md` | "RoboLang 2.1"; the grammar-only prompt pastes nine files (`spec/grammar-v2.1.ebnf` last). The grammar file says what 2.1 cannot say (erratum E5 of [1.8](bench-errata-1-8.md)): delivery, for each message, the content of a message as a value, patterns, trust in the sender, and that a message wait is never replaced |
| `bench/lib.py` | `GRAMMAR_FILE`/`GRAMMAR_FILES`; the schema text says 2.1; **the world shown to the model**: for a world file 1.6 the line `channels: NAME (accepts messages from SENDER, ...)` or `channels: none (this world has no message service)` (C1), and for each scheduled message `scheduled: after 300 s, a message "WIND CLEAR" arrives on channel site from rl:agent:site-ctl` (C2); the clock, limits and actions lines are shown for 1.6 as for 1.5. `message_uses`, `message_ok`, `message_wait_replaced`, `channel_or_text_changed`, `judge_messages`; `task_paths` reads `expect_2_1.paths` of a program that is no mission; `mission_label` reads `expect_2_1.expected_mission` too; `condition_text` renders `message_received` |
| `bench/score.py` | `message_ok` (B1; one of the task's checks `message_waits`), `message_wait_replaced` (B8, and its own line for the infeasible requests, the B4 clause), `channel_or_text_changed` (B9), `message_added` (B5); the table lines |
| `bench/tasks_messages/` (24) | `expect` becomes the 2.1 label, `expect_2_0` is the 2.0 label (the controls keep their own `expect_2_0`, with the paths valid in both versions), `expect_2_1` keeps the pre-registered 2.1 label (`paths`, `world_2_1`, `because`, a mission's `expected_mission`). `expected_message_waits` and a program's `expected_clock`, `expected_limits`, `expected_actions` move out of `expect_2_1` to task keys, as in sets 6 to 8. The five infeasible tasks get `expected_message_waits` too (what the request asks, for the B4 clause). `world` points at the world file 1.6 copy `bench/worlds/msg_<id>.json`. **10 `m` tasks: `expect` program** (was `cannot / inexpressible`); **5 `i` tasks: `expect` `cannot / infeasible`**; `x01`-`x04` unchanged (`cannot / inexpressible`); the 4 controls unchanged |
| `x05_a_message_in_the_goal` | **becomes a program** (C14): group `message_goal`; the label is a final bounded `wait until message_received "line" with "RECEIVED"` with the world part in the goal (`object_at tote_1 line_feed`); two paths (RECEIVED at 60 s: the goal, ending at 60000 ms; no message: the run fails); the wait's timeout is any (the request gives none). It leaves the atlas. B3 now counts the four tasks that stay inexpressible (`x01` to `x04`) |
| `bench/reference_messages/` | a 2.1 reference (`.rob`, `.json`) for the 10 `m` tasks and `x05` (the obvious attempt with the message lines unmarked and `robolang 2.1;`); the 20 `.attempt.*` files stay as the evidence of the baseline, and still fail with `E211`/`E201` under 2.0 |
| `bench/worlds/msg_*.json` | **31 new files**: the world file 1.6 copy of each base world (24 for set 10, 7 for the standing tasks), with the `channels` and the scheduled `messages` of `world_2_1`, and nothing else changed. `msg_i04_no_message_service_in_the_ward.json` has no `channels` (no message service) |
| the seven standing tasks | **now programs** (`expect` program, `expect_2_0` the old `cannot / inexpressible`, group `message`, `expect_2_1` with two paths and `world_2_1`, `expected_message_waits`, a world copy, a reference): `f_msg_line_docked` (`line`, "TUGGER DOCKED", from `rl:agent:line-ctl`), `f_msg_nurse_confirm` (`nurse`, **no text: any message**, C9), `f_msg_wind_clear` (`site`, "WIND CLEAR"), `x_wait_message` and `x_wait_ready_message` (`handoff`, "READY", sender the world robot `rl:robot:amr1`, C6), `u11_wait_go` (`pharmacy`, "GO"), `u12_radio_clearance` (**no channel and no text**: the world declares exactly one, `port_authority`, so it is that channel and any message, C9). None names a timeout, so the label accepts any. Their 1.x attempts stay |
| `f12_message_delivery` (set 3) | unchanged: `cannot / inexpressible`, gap G-DELIVERY (S7) |
| `x03_for_each_go_message` (set 9) | unchanged: `cannot / inexpressible`, gap G-TRIGGER (S10) |
| `bench/atlas.json` | G-MSG stays **open** (it closes at L6) with no task: everything that was in it is a program, infeasible, or in G-TRIGGER, G-CALC, G-DELIVERY; snapshot "I5 L4: RoboLang 2.1 built and taught" |
| `docs/plans/humanoid-activities.md` | regenerated from the atlas |
| `bench/README.md`, `docs/LANGUAGE_ROADMAP.md` | the set 10 section; the I5 row "L4 done; L5 acceptance run next" |
| tests | `tests/test_lang21.py` (new); `tests/test_bench_messages.py` rewritten for the 2.1 labels, the world copies, the measures and the standing tasks; the counts and groups of `test_bench_ask.py`, `test_bench_branch.py`, `test_bench_future.py`, `test_bench_hard.py` (group `message`); `test_reference.py` (2.1, and "Incoming messages") |

## Freeze (L4)

`docs/evidence/bench-freeze-v2.json` is regenerated for all of the above (the previous manifest, `5336c21e…`, is in git history).
