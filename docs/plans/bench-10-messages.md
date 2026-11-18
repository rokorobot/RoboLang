# Benchmark set 10: messages (pre-registration draft)

**Status:** **draft for the owner** (2026-11-18): tasks, labels, open points 1-14 and the placeholder bounds B1-B9 are not approved and nothing is
frozen. `tests/test_bench_messages.py`; run with `python bench/run.py --set messages`. Modelled on set 9,
[bench-9-missions.md](bench-9-missions.md), and set 8, [bench-8-clock.md](bench-8-clock.md). No model was run and no money
was spent. **The labels follow 0043 S1-S11 as approved ([0043](../../spec/decisions/0043-incoming-messages.md), 2026-11-18).
Where 0043 does not decide what a label needs, the label states the reading used and the question is listed under "Open
points" below: an owner answer that differs changes the labels named there.** The acceptance bounds are placeholders, to
be fixed with the baseline in hand (rule 6, [STRATEGY.md](../STRATEGY.md)). No language, spec, validator or simulator
change: that is L4.

## How the set is used

- **Labels.** `expect` is the **2.0** label and is what the baseline scores: every non-control task expects `cannot /
  inexpressible`, because 2.0 has no condition that tests a message (`message_received` is reserved: `E211`). `expect_2_1`
  is the label for acceptance once 2.1 is taught (L4 moves it to `expect` and keeps the old one as `expect_2_0`, as set 9
  did). The 2.1 label of a program carries its `expected_message_waits` and its `paths`.
- **`expected_message_waits`.** A placeholder form for L4's scorer: a list with one entry per message condition the program
  must contain, in the order of the request: `use` (`wait` for a `wait until`, `if`, `until` for a mission's `until`),
  `channel`, `text` (the exact text, or `null` for any message on the channel) and `timeout` (a `wait` only). A control has
  an empty list: a model that adds a message wait fails it.
- **`world_2_1`.** The 2.1 additions to the world, in the label (as set 7's `world_1_4` is): `channels`, the channels the
  world declares, each with its allowed senders (`{"line": {"from": ["rl:agent:line-ctl"]}}`, 0043 S4) and, on an
  infeasible task, the `messages` the world holds. The world files themselves are the 2.0 ones: world file 1.6 is L4's.
- **Paths.** Each path of a program has a `when`, a `world_patch` with the incoming `messages` that force it (events
  `{at, channel, text, from}`, 0043 S6; an empty list is "nothing arrives"), the goal it reaches or `fails`, the messages the
  program sends (`messages`, as for earlier sets) and `end_ms`: the simulated time at which the run ends (the arrival of the
  message that releases a wait, or the end of its timeout). A mission's paths are as set 9's (`outcome`, `runs`,
  `runs_verified`). A control's paths are in `expect_2_0` (read by `lib.task_paths`, as they are valid in both versions).
- **Baseline (now, 2.0 reference):** the 20 tasks that are not controls expect `cannot / inexpressible`. The obvious
  attempt of each (in `bench/reference_messages/`) is the task with the message construct as 0043 shows it (or, for what
  stays out, the form a model would guess); 2.0 rejects it with `E211` (18; `message_received`, reserved) or `E201` (2: a
  handler on `communicate`, `x01`; `repeat when`, `x02`). Every attempt marks the message lines with `// @message`
  (`// @message-tail` where only the end of a line is the message: a handler on `communicate`, the `until` of a mission);
  without them it is a valid 2.0 program that runs the rest of the task and succeeds (the tests run all 20; the mission
  of `m04` ends `bounded`). That program is the **message wait replaced silently**: the wait dropped, as the old checks
  cannot tell. The 4 controls expect a program (references `.rob`, `.json`, valid and SUCCEEDED on every path).
- **Acceptance (after 2.1 is taught):** the 2.1 labels apply. A program is judged as in sets 7 to 9 (the goal of each path
  in a patched world, and, for the mission, its outcome) **and** by checks written in L4: `message_wait_ok` (the message
  conditions of `expected_message_waits`, each with its channel and exact text, no more and no fewer), the `end_ms` and
  outcome of each path in the simulator, and **message wait replaced silently** and **channel or text changed** (below).
  The 2.1 references are written in L4, not now.
- **Worlds:** existing ones only (`yard`, `warehouse`, `fulfilment`, `ward`, `site`, `pharmacy`, `assembly`, `site_1_3`,
  `ward_1_4`, `warehouse_1_5_0915`). No new world file. Time passes through waits only (a move takes no time), so a message
  at a given second is reached after a known wait: `m03` waits for the aisle (8 s) before its `if`, `m05` waits for 09:20
  (300 s) before the message wait.
- **Not copied.** The standing message tasks (the R3 probes `f_msg_line_docked`, `f_msg_nurse_confirm`, `f_msg_wind_clear`;
  `x_wait_message`, `u11_wait_go`, `u12_radio_clearance`, `x_wait_ready_message`, `f12_message_delivery`; set 9's
  `x03_for_each_go_message`) stay in their sets with their ids; this set has new ids and new prompts for the same shapes. See
  "What this means for the standing tasks".

## Tasks (24)

Attempt code: the first diagnostic of the obvious attempt under 2.0. 24 tasks: 10 + 5 + 5 + 4.

### Programs in 2.1 (10)

| id | world | Request | 2.0 label (attempt) | 2.1 label: message waits and paths |
|---|---|---|---|---|
| `m01_place_when_the_tugger_docks` | fulfilment | With digit1, carry tote_1 from the inbound_conveyor to the handoff_point and hold it there. Wait up to 10 minutes for channel "line" to send "TUGGER DOCKED", then place tote_1 on line_feed. If it does not come, stop the job as failed. | cannot / inexpressible (`E211`) | program; wait `line` "TUGGER DOCKED", 10 min. Paths: TUGGER DOCKED arrives from the line controller at 90 seconds -> goal at 90 s; no message arrives: the wait times out after 10 minutes -> fails at 600 s |
| `m02_confirm_then_go_back` | ward | With moxi1, take medication_1 from the pharmacy to room_412 and place it there. Then wait up to 15 minutes for the nurse to send "CONFIRMED" on channel "nurse", and send moxi1 back to the nurse_station. If no confirmation comes, tell channel "alerts" "no confirmation" and stop the job as failed. | cannot / inexpressible (`E211`) | program; wait `nurse` "CONFIRMED", 15 min. Paths: the nurse confirms at 4 minutes -> goal at 240 s; no confirmation: the wait times out after 15 minutes, the alert is sent and the run fails -> fails at 900 s |
| `m03_second_box_if_asked` | warehouse | With arm1, carry box1 from shelf_a to the table once the aisle is clear. Wait up to 20 seconds for the aisle and, if it is not clear by then, stop the job as failed. Afterwards, if channel "ops" has sent "ALSO BOX2", carry box2 from shelf_b to the table as well; if it has not, you are done. | cannot / inexpressible (`E211`) | program; if `ops` "ALSO BOX2". Paths: ALSO BOX2 arrives at 5 seconds, before the aisle clears at 8 seconds: the `if` sees it (S2) -> goal at 8 s; no message: box1 only -> goal at 8 s; ALSO BOX2 arrives at 30 seconds, after the `if` was reached: it does not count -> goal at 8 s; the aisle never clears: the wait for the aisle times out -> fails at 20 s |
| `m04_rounds_until_shift_end` | yard | With tug1, drive from gate_in to gate_out, wait 20 seconds and drive back to gate_in. Repeat that round until channel "ops" sends "SHIFT END", but for at most 12 rounds. | cannot / inexpressible (`E211`) | program; until `ops` "SHIFT END". Paths: SHIFT END arrives at 50 seconds: after run 3 (60 seconds) the until holds -> `completed` (3 runs); no message arrives: the bound of 12 runs is reached -> `bounded` (12 runs); SHIFT END arrives at 30 seconds from a sender the channel does not allow: ignored, the bound is reached -> `bounded` (12 runs) |
| `m05_release_after_the_clock_wait` | warehouse_1_5_0915 | With amr1, carry crate1 from the dock to the table, but not before 09:20 and only after channel "ops" has sent "RELEASE". Wait up to 10 minutes for 09:20, then up to 5 minutes for the message; if either does not come, stop the job as failed. | cannot / inexpressible (`E211`) | program; wait `ops` "RELEASE", 5 min. Paths: RELEASE arrives at 120 seconds, while the program still waits for 09:20: it counts (S2) -> goal at 300 s; RELEASE arrives at 400 seconds, during the message wait -> goal at 400 s; no RELEASE: the message wait times out 5 minutes after 09:20 -> fails at 600 s |
| `m06_any_message_from_customs` | yard | With straddle1, take cont_204 from stack_b to gate_out, but only after channel "customs" has sent a message, whatever it says. Wait up to 15 minutes for it and, if nothing comes, stop the job as failed. | cannot / inexpressible (`E211`) | program; wait `customs` (any text), 15 min. Paths: customs sends "CLEARED 7731" at 300 seconds -> goal at 300 s; the only message comes at 100 seconds from a sender the channel does not allow: ignored, the wait times out -> fails at 900 s; no message arrives: the wait times out after 15 minutes -> fails at 900 s |
| `m07_lobby_when_it_opens` | site_1_3 | With cleaner1, drive from dock_2 to the lobby without ever entering wet_floor, but only after channel "facility" has sent "LOBBY OPEN". Wait up to 10 minutes for it and, if it does not come, stay where cleaner1 is and stop the job as failed. | cannot / inexpressible (`E211`) | program; wait `facility` "LOBBY OPEN", 10 min. Paths: LOBBY OPEN arrives at 120 seconds -> goal at 120 s; no message arrives: the wait times out after 10 minutes -> fails at 600 s |
| `m08_open_the_door_when_unlocked` | ward_1_4 | With moxi1, take iv_kit_1 from the supply_closet to room_412, but only after channel "supply" has sent "DOOR UNLOCKED". Wait up to 10 minutes for it; then open closet_door, take the kit and bring it to room_412. If the message does not come, stop the job as failed. | cannot / inexpressible (`E211`) | program; wait `supply` "DOOR UNLOCKED", 10 min. Paths: DOOR UNLOCKED arrives at 200 seconds -> goal at 200 s; no message arrives: the wait times out after 10 minutes -> fails at 600 s |
| `m09_go_and_its_look_alikes` | pharmacy | With picker1, take tray_1 from packing to dispatch, but only after the pharmacist on channel "pharmacy_west" has sent "GO". Wait up to 10 minutes for it and, if it does not come, stop the job as failed. | cannot / inexpressible (`E211`) | program; wait `pharmacy_west` "GO", 10 min. Paths: the decoys arrive first (30, 60 and 90 seconds); the real GO arrives at 200 seconds -> goal at 200 s; only the decoys arrive: the wait times out after 10 minutes -> fails at 600 s |
| `m10_door_open_and_lane_clear` | fulfilment | With stretch1, take case_1 from trailer_4 to staging, but only after channel "dock" has sent "DOOR OPEN" and channel "safety" has sent "LANE CLEAR". Wait up to 10 minutes for each of them and, if one does not come, stop the job as failed. | cannot / inexpressible (`E211`) | program; wait `dock` "DOOR OPEN", 10 min; wait `safety` "LANE CLEAR", 10 min. Paths: LANE CLEAR arrives at 30 seconds and DOOR OPEN at 200 seconds: both are there when needed -> goal at 200 s; DOOR OPEN arrives at 60 seconds and LANE CLEAR never: the second wait times out -> fails at 660 s; DOOR OPEN never arrives: the first wait times out -> fails at 600 s |

### Infeasible in 2.1 (5)

| id | world | Request | 2.0 label (attempt) | 2.1 label | Why |
|---|---|---|---|---|---|
| `i01_a_channel_the_world_does_not_have` | pharmacy | With picker1, take tray_2 from packing to dispatch, but only after channel "packing" has sent "BAG READY". Wait up to 5 minutes for it and, if it does not come, stop the job as failed. | cannot / inexpressible (`E211`) | cannot / infeasible (0043 S4) | the world declares the channel "pharmacy" only; "packing" is a location, not a channel. S4: a task that names an undeclared channel fails before the run, so the request cannot be done in this world, with or without a handler for the timeout. |
| `i02_the_wrong_sender` | fulfilment | With digit1, take tote_2 from the inbound_conveyor to the handoff_point and hold it there until the line controller has sent "TUGGER DOCKED" on channel "line", then place it on line_feed. | cannot / inexpressible (`E211`) | cannot / infeasible (0043 S4) | the only "TUGGER DOCKED" comes at 60 seconds from rl:agent:contractor-3, which channel "line" does not allow. S4: a message from a sender the channel does not allow is ignored, and the request states no other way to go on. The world has no message that counts, so the tote would wait for ever. |
| `i03_thirty_seconds_for_a_late_message` | site | With installer1, take module_14 from the module_pallet and place it on row_14 within 30 seconds of the start, but only after the site controller has sent "WIND CLEAR" on channel "site". | cannot / inexpressible (`E211`) | cannot / infeasible (0043 S6) | "WIND CLEAR" arrives at 300 seconds; the request allows 30. The message arrives 5 minutes after the start and the request allows 30 seconds with no other way to finish: the wait cannot succeed. (A request that says what to do when the message is late is a program that ends in `fail`.) |
| `i04_no_message_service_in_the_ward` | ward | With moxi1, take iv_kit_1 from the supply_closet to room_412, but only after the charge nurse has sent "ROOM CLEAN" on channel "nurse". Wait up to 5 minutes for it. | cannot / inexpressible (`E211`) | cannot / infeasible (0043 S5) | the world declares no channel and has no message service. S5: a world with no message service makes `message_received` unknown, which fails closed (and S4 fails a task that names a channel the world does not declare). Either way the wait cannot succeed. |
| `i05_only_a_near_miss_arrives` | assembly | With arm_b, move p4 from station3 to the bin, but only after channel "qa" has sent "RELEASE". Wait up to 5 minutes for it. | cannot / inexpressible (`E211`) | cannot / infeasible (0043 S3) | channel "qa" gets "release" at 40 seconds and "RELEASE NOW" at 90 seconds, never "RELEASE". S3: a match is exact and case-sensitive. Neither message is the text the request names, so no message counts. |

### Still inexpressible in 2.1 (5)

| id | world | Request | 2.0 label (attempt) | 2.1 label | Why it stays out | Atlas gap |
|---|---|---|---|---|---|---|
| `x01_make_sure_it_was_delivered` | ward | With moxi1, send "MEDS READY" to channel "nurse" and make sure the message was delivered before taking medication_1 from the pharmacy to room_412 and placing it there. If it was not delivered, move nothing. | cannot / inexpressible (`E201`) | cannot / inexpressible (0043 S7) | Delivery of what the robot sends stays out of 2.1 (0043 S7): it is the transport's job and `communicate` takes no handler (E201). "Wait until the nurse confirms" is a reply message and is expressible; "make sure it was delivered" is not. | G-DELIVERY |
| `x02_each_time_the_line_says_ready` | fulfilment | Each time channel "line" sends "TOTE READY", send tugger_a from tugger_bay to line_feed and back to tugger_bay. Do this for the next 4 hours. | cannot / inexpressible (`E201`) | cannot / inexpressible (0043 S10) | A message trigger (0043 S10, S11): a run starts when a message arrives. 2.1 has the condition, not the trigger: a mission starts a run when the previous one is done (0041 M6). The bound is stated; the trigger is what stays out. | G-TRIGGER |
| `x03_forward_the_order_number` | fulfilment | With digit1, wait up to 5 minutes for channel "orders" to send an order number, send that number on to channel "labels", and then move tote_1 from the inbound_conveyor to the outbound_conveyor. | cannot / inexpressible (`E211`) | cannot / inexpressible (0043 S10) | The content of a message as a value stays out of 2.1 (0043 S1 alternative, S10, S11): a condition tests a text, it binds nothing, and nothing in the language can use received text. | G-CALC |
| `x04_any_message_containing_ready` | fulfilment | With digit1, take tote_3 from the inbound_conveyor to the handoff_point and hold it there until channel "line" sends any message that contains the word READY (for example "READY" or "LINE READY"), then place it on line_feed. Wait up to 10 minutes for it and, if it does not come, stop the job as failed. | cannot / inexpressible (`E211`) | cannot / inexpressible (0043 S3) | A text match is exact (0043 S3): there are no patterns and no "contains". Writing the wait for the exact text "READY" silently narrows the request, since "LINE READY" would not release the tote. | G-CALC |
| `x05_a_message_in_the_goal` | fulfilment | With digit1, move tote_1 from the inbound_conveyor to line_feed. The job is done only when tote_1 is on line_feed and channel "line" has sent "RECEIVED": state both as the goal of the job. | cannot / inexpressible (`E211`) | cannot / inexpressible (0043 S1) | A message is not an end state of the world, so `message_received` is never allowed in a `goal` (0043 S1), and a goal is what Core receives (`export-goal`). A wait for the message in the plan is a different request: the goal does not hold the message. | G-MSG |

### Controls (4)

| id | world | Request | Label (both versions) |
|---|---|---|---|
| `c01_plain_carry` | warehouse | With arm1, carry box2 from shelf_b to the table. | program, one task with four steps; no message wait |
| `c02_tell_the_ops` | warehouse | With arm1, carry box1 from shelf_a to the table and then tell channel "ops" "box1 is on the table". | program, one task that ends with `communicate`; a path checks the message sent; no message wait |
| `c03_wait_for_the_worker_lane` | fulfilment | With digit1, carry tote_2 from the inbound_conveyor to the outbound_conveyor, but only after the worker_lane is clear. Wait up to 30 seconds for it and, if it is not clear by then, stop the job as failed. | program, one `wait until region_clear` with a timeout and `fail`; paths: clears at 10 s / times out at 30 s; no message wait |
| `c04_wait_twenty_seconds_for_the_lab` | ward | With moxi1, take samples_1 from ward_3 to the lab and, after waiting 20 seconds there for the lab to be ready, place them at the lab. | program, one `wait for 20 s` (ends at 20 s); no message wait |

## What this means for the standing tasks

Eight standing tasks and one set 9 task are about messages. Under 0043 they are labelled as follows (L4 relabels them with
`expect_2_1`; none moves now):

| Task | 2.1 label | Note |
|---|---|---|
| `f_msg_line_docked`, `f_msg_wind_clear` | program | a `wait until message_received "line" with "TUGGER DOCKED"` and the same for `"site"`; the world declares the channel and its sender |
| `f_msg_nurse_confirm` | program | the request names the channel but no text: any message on `"nurse"` (`message_received "nurse"`); open point 9 |
| `u11_wait_go`, `x_wait_message`, `x_wait_ready_message` | program | `"pharmacy"` with `"GO"`, `"handoff"` with `"READY"`; in the last two the sender is `amr1`, a robot of the world: open point 6 |
| `u12_radio_clearance` | program or infeasible | "until the port authority radios clearance" names no channel and no text: the world must declare one, else the request is infeasible by S4; open point 9 |
| `f12_message_delivery` | inexpressible | S7 (delivery), as `x01` here; atlas gap G-MSG until L6 of I5 decides (open point 13) |
| `x03_for_each_go_message` (set 9) | inexpressible | a trigger (S10, S11); atlas gap G-MSG at present, G-TRIGGER by 0043 S10 (open point 13) |

## Acceptance bounds (proposed with the baseline in hand, rule 6; for the owner)

Baseline: [bench-set10-baseline.md](../evidence/bench-set10-baseline.md) (2026-11-18, 2.0, 3 repeats): no false
programs, every decision right.

Baseline: not yet run (2.0, 3 repeats, full and grammar-only prompts). The smallest step is one answer (rule 6). The acceptance
run repeats the baseline under 2.1. A bound that the baseline makes pointless (for instance a rate that 2.0 cannot reach) is
replaced when the baseline is read, as set 9's was.

| # | Group | Measure | Baseline (2.0) | Proposed bound (2.1, per model) |
|---|---|---|---|---|
| B1 | 10 `message` programs | right decision, and every path in its patched world ends as the label says (goal or fail, at `end_ms`; for `m04`, the outcome and runs), full prompt | not expressible | >= 90%: at least 27/30 |
| B2 | the same, grammar-only | the same | not expressible | for information |
| B3 | 5 inexpressible tasks | false programs, full and grammar-only (a valid program that succeeds: `x04` as an exact text, `x05` with the wait left out, `x01` without the delivery check) | 0/15 for each | at most 1/15 for each model and prompt |
| B4 | 5 infeasible tasks | decision `cannot`; no valid program that drops the message wait and succeeds | 15/15 for each | at least 14/15, and 0 that drop the wait and succeed |
| B5 | 4 `control` | right decision and **no message wait added**, full prompt | 12/12 for each | at least 11/12 |
| B6 | sets 1 and 2 (standing) | `gate.json`; decision accuracy | latest run | the gate passes; `decision_correct` drops at most 2 points or one answer, whichever is larger |
| B7 | economy | statements, kinds, literals, reserved words, top-level declarations, world fields, codes | - | 0 statements, 1 condition (`message_received`, reserved since 1.1; `with` exists), 0 new reserved words, 0 top-level declarations, 1 world field `channels` and the `messages` events, about 4 user-facing codes plus their IR forms |
| B8 | 10 `message` programs | **message wait replaced silently**: a valid, succeeding 2.0-style program (no `message_received`) that drops the wait, guesses a `wait for`, or puts a `region_clear` or `sensor_ok` stand-in where the message is; full prompt | to be measured (every valid answer to these tasks is one in 2.0) | at most 1/30 |
| B9 | 10 `message` programs, `x04` | **channel or text changed**: a message wait whose channel or text is not the request's (a decoy channel or text taken, a case change, any message where a text is named, an exact text invented where the request says "whatever it says"), and for `x04` the pattern written as an exact text; full prompt | not expressible | at most 1/30 on the programs; 0 exact texts written for `x04` |

If a bound fails, 2.1 stays in code only and is not taught.

## Cost of the baseline and acceptance runs (not spent)

`python bench/run.py --set messages --repeats 3 --models claude-sonnet-5-5,claude-haiku-5-5`, effort medium, as the
`Benchmark` workflow runs it with `set_only: messages` (full and grammar-only prompts, `--max-usd 3` each, not gated,
never in the per-PR or weekly runs):

| Prompt | Requests | Estimate |
|---|---:|---|
| Full | 90 per model | ~$3.73 (Sonnet ~$3.55, Haiku ~$0.18) |
| Grammar only | 90 per model | ~$3.59 (Sonnet ~$3.42, Haiku ~$0.17) |

The estimate is a guess (`bench/README.md`) and is above the $3 cap: the cap stops new requests, and a rerun with the same
`--run-id` resumes. The earlier sets cost well below their estimates. Nothing was sent.

## Open points: where 0043 does not decide what a label needs (owner, before L4)

Each point names the labels that depend on it. The label uses the reading given first.

1. **A world with no `channels` is S4 or S5 (`i01`, `i04`).** S4: a task that names an undeclared channel fails before the
   run. S5: a channel whose state cannot be known (no message service in the world) makes the condition `unknown`, which
   fails closed at run time. A world file with no `channels` field at all is both: every channel the task names is undeclared,
   and there is no message service. The labels are `infeasible` either way (`i01`: the world declares other channels; `i04`:
   it declares none), so the two tasks differ only in the code that a later check reports. Also open: how world file 1.6 says
   "no message service" for a world that declares channels (a flag on the world, or a channel with no `from`).
2. **Are the world's scheduled messages shown to the model (`i02`, `i03`, `i05`, `m06` path 2, `m09`)?** The world list shows
   scheduled events (`scheduled: after 8 s, region_clear of aisle becomes true`), and the infeasible labels of `i02` (the only
   message is from a sender the channel does not allow), `i03` (it arrives at 300 s, the request allows 30) and `i05` (only
   near-miss texts arrive) are decidable only if the list shows the messages, with their sender and the channel's allowed
   senders. The labels read that it does. If it does not, those three are programs whose run fails at the wait (a path, as
   in `c03`), and the infeasible group falls to `i01` and `i04`.
3. **Infeasible or a program that fails (`i02`, `i03`, `i05`).** 0043 does not say when a wait that cannot be satisfied is
   `infeasible`. The labels read: when the request states no way to go on without the message (no "if it does not come, stop"),
   the world makes success impossible and the request is `infeasible`; a request that says what to do when the message does not
   come is a program whose fail path is taken (`m01`'s second path). The 2.0 language reference tells a model to write an
   `else { fail ...; }` when the request says nothing, which turns every such request into a program that fails: if the owner
   prefers that reading, `i02`, `i03` and `i05` become programs with a failing path and the user-visible rule is "a wait that
   cannot succeed is not infeasible".
4. **Does a message at `at: 0` count?** S2: "after the task run started". The labels use no message at 0 s, and put a wait
   before an `if` (`m03`, the aisle at 8 s), because a move takes no time and a message at 5 s has not arrived when the first
   statement runs. A `pre { message_received ... }` is checked at the start: with S2 it can only be true for a message at 0 s
   or never; no label uses `pre`. Is it allowed, always false, or does it also count messages sent before the task started?
5. **Boundary and `end_ms`.** A message at exactly the end of a wait's timeout, and a message that arrives at the same instant
   as another event, are not in any label. The `end_ms` of a path assumes a wait released by a message ends at the message's
   `at`, and a wait that is reached after the message ends at once (`m05`, `m10`).
6. **Who may be a declared sender (`x_wait_message`, `x_wait_ready_message`; S4, S10).** 0043 writes senders as
   `rl:agent:line-ctl`. The two standing tasks have `amr1`, a robot of the world, send "READY": is `rl:robot:amr1` a sender, and
   does its message count when `amr1` is also bound in the same task? S10 says a robot's own `communicate` is not a received
   message and that messages between robots in one task stay out. The labels use `rl:agent:` only, and no task has the sender
   among its robots.
7. **A message may be used again (S2, latched).** S2 says a counted message stays counted. Two waits for the same channel and
   text would both pass. No label has this (`m10` waits for two different channels); say so in the reference if wanted.
8. **Each run of a mission starts fresh (S2).** The mission's `until` sees messages since the mission started (`m04`), and a task
   run inside it sees messages since that run started. No task checks the second half: a message that arrived during run 1
   does not satisfy a `wait until message_received` in run 2. Add one if the owner wants it tested before L4.
9. **A request that names no text or no channel (`f_msg_nurse_confirm`, `u12_radio_clearance`).** "Until the nurse confirms" has
   a channel but no text; "until the port authority radios clearance" has neither. The labels (and the standing relabel) read: a
   missing text is any message on the channel (`m06`), a missing channel is a channel the world declares for that party; if the
   world declares none, the request is infeasible (S4). A model that invents a text for a request that names none changes the
   request, which B9 counts.
10. **`not message_received` and combinations.** S1 says the condition stands wherever a condition stands, so `if not
    message_received "ops" with "STOP"` and `wait until message_received "a" or message_received "b"` are conditions. No
    label uses them. `not` is where S5 matters most: `not` of an unknown is unknown, `not` of a false is true. Add one task
    for L4 if the owner wants it.
11. **Text match and spaces (S3).** "Exact and case-sensitive, after trimming spaces at both ends": of the message, or of the
    literal in the task too? `i05` uses a case difference only, and `m09` a longer text ("GO BACK") and a lower-case one.
12. **A message wait inside a handler or a mission's run.** `m02` puts `communicate` in the handler of a message wait; no label
    has a message wait inside an `else` handler, or a `run`ning task whose own wait is for a message.
13. **Atlas gaps.** The 20 tasks that 2.0 declines are in G-MSG (16: the 10 programs, the 5 infeasible, and `x05`), G-CALC
    (2: `x03`, a number read from a message; `x04`, a pattern), G-TRIGGER (1: `x02`) and **a new gap, G-DELIVERY** (1: `x01`,
    confirmed delivery, S7), which the atlas did not have: scope `open` and far (no sub-activity needs it), but the owner may
    prefer `out` (a decline is the right answer, as for G-UNBOUNDED; 0043 S7 calls delivery "the transport's job"). `x05` (a
    message in the goal) stays in G-MSG, which L6 closes: it then needs a gap, perhaps G-FALLBACK-like. `x03` is in G-CALC as the
    plan asked; 0043 S10/S11 also relates it to G-CHOOSE (an order id used to choose the object).
14. **`x05` may be a program.** "The job is done only when the line has sent RECEIVED" can also be written as a final `wait until
    message_received "line" with "RECEIVED" ... else { fail ...; }` in the plan: a valid 2.1 program with the same behaviour at run
    time. The label (`inexpressible`, S1) reads the request literally ("state both as the goal"); a model that writes the final wait
    has changed the request from a goal to a step, which 0043 does not call a false program. The owner may prefer to make `x05`
    a program (with the goal reduced to `object_at`) and drop the goal half from the set.
