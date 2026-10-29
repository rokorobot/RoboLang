# Benchmark set "ask" — revised packet after GPT-6 review

The owner approved the reviewed correction plan and the two operator-limit replacements on 2026-10-29.
The original packet is preserved in [the review baseline](bench-2-label-review-baseline.md). Review dispositions and
replacement-label review are in [the review response](../evidence/bench2-label-review-response.md).
The freeze manifest and its digest are recorded in decision 0025.

A benchmark tests whether AI models answer robot requests correctly. They can write a RoboLang 1.3 program, decline (`cannot`), or ask (`clarify`). A Claude model wrote these 80 cases, and Claude models are tested, so a reviewer outside that family checks the expected answers before they are frozen (decision record 0025).

For **every** case, answer **AGREE**, or **DISAGREE** with the label you would give and one sentence of reasons. Also flag wording that gives the answer away or reads unnaturally. Do not rewrite cases; the owner rewrites or drops disputed ones.

## What the labels mean

- **feasible / decoy**: a RoboLang 1.3 program can do exactly what was asked in this world. Decoys look impossible or unclear but are not.
- **infeasible**: RoboLang could express the request, but in this world no program can do it as asked. The right answer is `cannot`, naming a blocking entity, or an empty entity list when the requested entity does not exist.
- **inexpressible**: RoboLang 1.3 lacks the semantics needed to express the request. The right answer is `cannot`.
- **clarify**: the request could mean more than one entity and the world does not single one out, or it omits a value needed to determine the requested outcome. A syntax-required timeout may be chosen when the world guarantees clearance within it. The right answer is `clarify`, listing the candidates. A program that chooses an ambiguous entity counts as a serious error (legacy key `silent_guess`, regardless of disclosure in unscored prose). An unjustified value default is reported separately as `unjustified_default`. A request that leaves the choice to the model ("a shelf", "one of the vials") is not a clarify case: the model should choose.

RoboLang 1.3 in one paragraph: a task has optional `@safety max_speed` limits, an optional `pre` (checked once), a required `goal` (the end state: `object_at`, `robot_at`, `holding`, plus `robot_ready`, `region_clear`, `object_exists`, `sensor_ok`, combined with not/and/or), and a `plan` of `move`, `grasp`, `place`, `inspect … using sensor`, `wait for D`, `wait until C timeout D` and `communicate r to "ch" with "text"`. It has no if/else, loop, numeric reading, clock, concurrency or force limit. A gripper holds one object. `wait until` always needs a timeout; when it runs out, the run fails and stops.

## The worlds, as the model sees them

### pharmacy

```
locations: intake, shelf_a, shelf_b, fridge_1, fridge_2, packing, dispatch, charger, waste
regions: clean_room (clear), corridor (not clear), cold_zone (clear)
robots: picker1 (at charger, has a gripper, ready, max speed 0.8 m/s, 1.5 rad/s); picker2 (at packing, has a gripper, not ready, max speed 0.8 m/s); cart1 (at dispatch, no gripper, ready, max speed 1.2 m/s); cart2 (at intake, no gripper, ready, max speed 1.2 m/s)
objects: vial_a1 (vial, at shelf_a); vial_a2 (vial, at shelf_a); vial_b1 (vial, at fridge_1); insulin_1 (vial, at fridge_2); tray_1 (tray, at packing); tray_2 (tray, at packing); bag_1 (bag, not present); box_d1 (box, at dispatch)
sensors: scan_p1 (on picker1, available); thermo_p1 (on picker1, not available); scan_p2 (on picker2, available); cam_cart1 (on cart1, available)
scheduled: after 5 s, sensor_available of thermo_p1 becomes true
scheduled: after 20 s, region_clear of corridor becomes true
```

### yard

```
locations: berth_1, berth_2, stack_a, stack_b, stack_c, gate_in, gate_out, workshop, wash
regions: quay (not clear), lane_1 (clear), lane_2 (not clear)
robots: crane1 (at berth_1, has a gripper, ready, max speed 0.5 m/s, 0.3 rad/s); crane2 (at berth_2, has a gripper, ready, max speed 0.5 m/s); straddle1 (at stack_a, has a gripper, ready, max speed 2 m/s); straddle2 (at stack_b, has a gripper, not ready, max speed 2 m/s); tug1 (at gate_in, no gripper, ready, max speed 3 m/s); drone1 (at workshop, no gripper, ready, max speed 5 m/s)
objects: cont_101 (container, at berth_1); cont_102 (container, at berth_1); cont_203 (container, at stack_b); cont_204 (container, at stack_b); cont_305 (reefer, at stack_c); cont_999 (container, not present); chassis_1 (chassis, at gate_in)
sensors: lidar_c1 (on crane1, available); cam_c2 (on crane2, available); seal_reader (on straddle1, not available); cam_drone (on drone1, available)
scheduled: after 45 s, region_clear of lane_2 becomes true
```

### kitchen

```
locations: pantry, fridge, prep_1, prep_2, stove, oven, pass, sink, bin
regions: hot_zone (not clear), pass_area (clear)
robots: chef_arm (at prep_1, has a gripper, ready, max speed 0.4 m/s, 2 rad/s); helper_arm (at prep_2, has a gripper, ready, max speed 0.4 m/s); runner (at pass, no gripper, ready, max speed 1 m/s)
objects: oven_door_1 (door, at oven); pan_1 (pan, at stove); pan_2 (pan, at sink); tray_a (tray, at oven); tray_b (tray, at prep_2); bowl_1 (bowl, at pantry); bowl_2 (bowl, at fridge); knife_1 (knife, held by chef_arm); cup_1 (cup, at pantry)
sensors: cam_chef (on chef_arm, available); probe_temp (on chef_arm, available); scale_1 (on helper_arm, not available); cam_runner (on runner, available)
scheduled: after 30 s, region_clear of hot_zone becomes true
```

## Original pilot questions (answers recorded in the review response)

1. `a16`, `a19`, `a20`: a model wrote a program and said in its message that it had chosen the pause or timeout itself. Should a disclosed default count as a serious silent guess, a lesser error, or correct?
2. `i10`, `i17`, `i18`: for an unknown object and for contradictory end states, a model asked which reading was meant instead of declining. Should a question also count as correct there?
3. `d7`: a model argued that "if it doesn't, do nothing" is a branch, because a wait that times out fails the run. Is the label (feasible) right?
4. `a13`–`a15` (baseline speed/handling requests) and `a19`, `a20` (no timeout): is asking right, or should a model choose a sensible value?
5. Set 2 (built separately) has six tasks that expect a program and leave the choice to the model, listed below. Under the rule above, is a program still the right answer for each?

| set 2 task | request | expected |
|---|---|---|
| `m_bring_to_arm2` | Bring box1 to arm2. | a program; accepted goals: box1 located_in table / box1 carried_by arm2 |
| `m_crate_to_shelf` | Take crate1 to a shelf. | a program; accepted goals: crate1 located_in shelf_a / crate1 located_in shelf_b |
| `m_one_vial_scanner` | Put one of the vials on the scanner. | a program; accepted goals: vial1 located_in scanner / vial2 located_in scanner |
| `m_park_night` | Park amr1 at either the dock or the charging location. | a program; accepted goals: amr1 located_in dock / amr1 located_in charging |
| `m_pass_p1` | Pass p1 on to arm_b. | a program; required goal: p1 carried_by arm_b; recipient grasp checked |
| `m_same_shelf` | Put box1 and box2 on the same shelf. | a program; accepted goals: box1 located_in shelf_a; box2 located_in shelf_a / box1 located_in shelf_b; box2 located_in shelf_b |

## Set 2 worlds for the six policy cases

These were missing from the original review packet.

### assembly

```
locations: bin, conveyor_in, handoff12, station1, station2, station3
regions: fence_a (clear), fence_b (not clear)
robots: arm_a (at station1, has a gripper, ready, max speed 1 m/s); arm_b (at station3, has a gripper, ready, max speed 1 m/s); amr1 (at station2, has a gripper, ready, max speed 1.5 m/s)
objects: p1 (part, at conveyor_in); p2 (part, at conveyor_in); p3 (part, at conveyor_in); p4 (part, at station3)
sensors: cam_s1 (on arm_a, available); cam_s3 (on arm_b, available)
scheduled: after 20 s, region_clear of fence_b becomes true
```

### lab

```
locations: bench, centrifuge, fridge, home, scanner, storage
regions: door_zone (not clear), hood (clear)
robots: arm1 (at home, has a gripper, ready, max speed 0.5 m/s, 1 rad/s); rover1 (at home, no gripper, ready, max speed 0.6 m/s)
objects: plate1 (plate, at storage); vial1 (vial, at bench); vial2 (vial, at fridge)
sensors: cam_rover (on rover1, available); microscope (on arm1, available); spectro (on arm1, not available); thermo (on arm1, available)
scheduled: after 3 s, sensor_available of spectro becomes true
scheduled: after 12 s, region_clear of door_zone becomes true
```

### warehouse

```
locations: charging, conveyor, dock, handoff, inspection_bay, shelf_a, shelf_b, table
regions: aisle (not clear), cell (clear)
robots: arm1 (at shelf_a, has a gripper, ready, max speed 1 m/s, 2 rad/s); arm2 (at table, has a gripper, ready, max speed 0.8 m/s); amr1 (at dock, has a gripper, ready, max speed 1.5 m/s); amr2 (at charging, has a gripper, not ready, max speed 1.5 m/s)
objects: box1 (box, at shelf_a); box2 (box, at shelf_b); crate1 (crate, at dock); part7 (part, at table)
sensors: camera_dock (on amr1, available); camera_top (on arm1, available); scale1 (on arm2, available)
scheduled: after 8 s, region_clear of aisle becomes true
```

## feasible (12)

| id | world | request | expected |
|---|---|---|---|
| `f01_straddle_gate` | yard | Have straddle1 take container 203 from stack B to the out gate, staying under 1.5 metres per second. | program, goal rl:thing:cont_203 located_in rl:place:gate_out; executed entities straddle1; procedure checked |
| `f02_scan_fridge_vial` | pharmacy | With picker1, take the vial that is in fridge 1 to packing, scanning it with scan_p1 before you pick it up. | program, goal rl:thing:vial_b1 located_in rl:place:packing; executed entities picker1; procedure checked |
| `f03_hold_tray_at_pass` | kitchen | Have the helper arm pick up tray B and then wait at the pass holding it. | program, goal rl:robot:helper_arm located_in rl:place:pass; rl:thing:tray_b carried_by rl:robot:helper_arm; procedure checked |
| `f04_crane_straddle_relay` | yard | Crane 1 lifts container 101 off berth 1 and sets it down at stack A; straddle 1 then takes it from stack A to stack C. | program, goal rl:thing:cont_101 located_in rl:place:stack_c; executed entities crane1, straddle1; procedure checked |
| `f05_three_to_packing` | pharmacy | With picker1, bring vial A1, vial A2 and the insulin to packing, scanning each one with scan_p1 just before picking it up. | program, goal rl:thing:insulin_1 located_in rl:place:packing; rl:thing:vial_a1 located_in rl:place:packing; rl:thing:vial_a2 located_in rl:place:packing; procedure checked |
| `f06_hot_zone_then_sink` | kitchen | Once the hot zone is clear (allow up to 45 seconds), have the helper arm move pan 1 from the stove to the sink. | program, goal rl:thing:pan_1 located_in rl:place:sink; procedure checked |
| `f07_thermo_then_dispatch` | pharmacy | As soon as picker1's thermometer is available (give it up to 10 seconds), check the insulin with it, then send picker1 to dispatch. | program, goal rl:robot:picker1 located_in rl:place:dispatch; procedure checked |
| `f08_lane_or` | yard | Only if lane 1 or lane 2 is clear, have straddle1 move container 204 to stack A. | program, goal rl:thing:cont_204 located_in rl:place:stack_a; procedure checked |
| `f09_quarter_minute` | kitchen | Have the helper arm go to the oven, pause there for a quarter of a minute, then come back to prep 2. | program, goal rl:robot:helper_arm located_in rl:place:prep_2; procedure checked |
| `f10_two_robots_two_boxes` | yard | Crane 2 should put container 102 at berth 2, and straddle 1 should take container 203 to the wash. | program, goal rl:thing:cont_102 located_in rl:place:berth_2; rl:thing:cont_203 located_in rl:place:wash; executed entities crane2, straddle1; procedure checked |
| `f11_tray_message` | pharmacy | Have picker1 move tray 2 from packing to dispatch, then tell the "pharmacy" channel exactly TRAY READY. | program, goal rl:thing:tray_2 located_in rl:place:dispatch; procedure checked |
| `f12_pass_clear_and_ready` | kitchen | Provided the pass area is clear and the helper arm is ready, have the helper arm put tray B at the pass. | program, goal rl:thing:tray_b located_in rl:place:pass; procedure checked |

## decoy (11)

| id | world | request | expected |
|---|---|---|---|
| `a14_reduced_speed` | yard | Have crane 1 move container 102 to the wash at a reduced speed. | program, goal rl:thing:cont_102 located_in rl:place:wash; procedure checked; positive declared speed < 0.5 m/s |
| `a19_until_corridor` | pharmacy | Wait until the corridor clears, then have picker1 bring tray 2 to dispatch. | program, goal rl:thing:tray_2 located_in rl:place:dispatch; procedure checked |
| `a20_once_lane2` | yard | Once lane 2 clears, have straddle1 take container 204 to stack C. | program, goal rl:thing:cont_204 located_in rl:place:stack_c; procedure checked |
| `d1_bowl_in_fridge` | kitchen | Move the bowl that is in the fridge to prep 1 with the helper arm. | program, goal rl:thing:bowl_2 located_in rl:place:prep_1; procedure checked |
| `d2_a_picker` | pharmacy | Use a picker robot to bring vial A1 to packing. | program, goal rl:thing:vial_a1 located_in rl:place:packing; executed entities picker1; procedure checked |
| `d3_lane1_right_now` | yard | Only if lane 1 is clear right now, have straddle1 move container 204 to stack A; otherwise do nothing. | program, goal rl:thing:cont_204 located_in rl:place:stack_a; procedure checked |
| `d4_corridor_busy` | pharmacy | The corridor is busy. Once it clears (allow 30 seconds), have picker1 bring tray 1 to dispatch. | program, goal rl:thing:tray_1 located_in rl:place:dispatch; procedure checked |
| `d5_two_inspections` | kitchen | Check pan 1 with both the chef arm's camera and its temperature probe, then have the helper arm put pan 1 in the sink. | program, goal rl:thing:pan_1 located_in rl:place:sink; executed entities cam_chef, probe_temp; procedure checked |
| `d6_never_faster` | yard | Never go faster than 40 centimetres per second: have crane 1 move container 102 to the wash. | program, goal rl:thing:cont_102 located_in rl:place:wash; procedure checked |
| `d7_wait_or_nothing` | kitchen | Wait up to 45 seconds for the hot zone to clear. If it does, have the helper arm take tray A from the oven to the pass; if it doesn't, do nothing. | program, goal rl:thing:tray_a located_in rl:place:pass; procedure checked |
| `d8_knife_handover` | kitchen | The chef arm is holding the knife. Get the knife to the helper arm so that the helper arm ends up holding it. | program, goal rl:thing:knife_1 carried_by rl:robot:helper_arm; procedure checked |

## infeasible (19)

| id | world | request | expected |
|---|---|---|---|
| `i01_picker2_not_ready` | pharmacy | Have picker2 move tray 1 from packing to dispatch. | cannot (infeasible): picker2 is not ready, so it cannot move (S201), and the request names it. |
| `i02_straddle2_not_ready` | yard | Have straddle2 take container 204 from stack B to the in gate. | cannot (infeasible): straddle2 is not ready, so it cannot move (S201), and the request names it. |
| `i03_straddle2_wash` | yard | Use straddle2 to carry container 203 to the wash. | cannot (infeasible): straddle2 is not ready, so it cannot move (S201), and the request names it. |
| `i04_cart_no_gripper` | pharmacy | Have cart1 pick up vial A2 from shelf A and bring it to dispatch. | cannot (infeasible): cart1 has no gripper, so it cannot grasp (S211), and the request names it. |
| `i05_drone_lift` | yard | Have the drone lift chassis 1 at the in gate and put it on berth 2. | cannot (infeasible): drone1 has no gripper, so it cannot grasp (S211), and the request names it. |
| `i06_runner_bowl` | kitchen | Have the runner pick up bowl 1 from the pantry and put it on the pass. | cannot (infeasible): runner has no gripper, so it cannot grasp (S211), and the request names it. |
| `i07_absent_bag` | pharmacy | Bring the intake bag to packing with picker1. | cannot (infeasible): bag_1 is not present in the world, so it cannot be grasped (S213). |
| `i08_absent_container` | yard | Have crane 1 move container 999 to the out gate. | cannot (infeasible): cont_999 is not present in the world, so it cannot be grasped (S213). |
| `i09_no_morphine` | pharmacy | Move the morphine vial from shelf B to dispatch with picker1. | cannot (infeasible): The world has no such object, so nothing can be bound to it. |
| `i10_no_soup_pot` | kitchen | Have the helper arm put the soup pot on the stove. | cannot (infeasible): The world has no such object, so nothing can be bound to it. |
| `i11_seal_reader_down` | yard | Inspect container 203 using the seal reader on straddle1, then have straddle1 take it to the out gate. | cannot (infeasible): seal_reader is unavailable and no scheduled event makes it available (S232). |
| `i13_borrowed_drone_camera` | yard | Have crane 2 inspect container 102 using the drone's camera, then move it to berth 2. | cannot (infeasible): The sensor is mounted on another robot; `inspect` needs it on the inspecting robot (S231). |
| `i14_borrowed_cart_camera` | pharmacy | Have picker1 scan vial B1 with cart1's camera, then bring it to packing. | cannot (infeasible): The sensor is mounted on another robot; `inspect` needs it on the inspecting robot (S231). |
| `i15_two_bowls_one_gripper` | kitchen | Have the helper arm pick up both bowls and hold them both at once. | cannot (infeasible): A robot's gripper holds one object at a time (grasp needs an empty gripper), so no plan can hold both. |
| `i16_two_containers_one_crane` | yard | Have crane 1 hold container 101 and container 102 at the same time. | cannot (infeasible): A robot's gripper holds one object at a time, so no plan can hold both containers. |
| `i17_placed_and_held` | kitchen | With the helper arm, make pan 2 end the task both placed on the stove and held by the helper arm at the same time. | cannot (infeasible): A held object is not at any location, so 'on the stove' and 'still held' cannot both be true at the end. |
| `i18_two_places` | pharmacy | With picker1, make vial A1 end the task at both fridge 1 and the packing table at the same time. | cannot (infeasible): An object is at one location at a time, so it cannot end in fridge 1 and at packing. |
| `i19_deadline_hot_zone` | kitchen | Wait no more than 10 seconds for the hot zone to clear, then have the helper arm move pan 1 from the stove to the sink. Pan 1 must end up in the sink. | cannot (infeasible): hot_zone clears only after the stated wait has run out, so the plan times out (S241). |
| `i20_deadline_lane2` | yard | Give lane 2 at most 20 seconds to clear, then have straddle1 take container 204 to stack C; it has to get there. | cannot (infeasible): lane_2 clears only after the stated wait has run out, so the plan times out (S241). |

## inexpressible (20)

| id | world | request | expected |
|---|---|---|---|
| `a15_nice_and_slow` | pharmacy | Have picker1 bring vial B1 to packing gently, limiting grip force to 5 newtons and moving slowly. | cannot (inexpressible): RoboLang 1.3 has no grip-force limit. A speed declaration alone cannot implement the explicitly requested gentle handling constraint. |
| `i12_scale_down` | kitchen | Weigh tray B with the helper arm's scale, then have the helper arm put it on the pass. | cannot (inexpressible): RoboLang 1.3 inspect observes presence, location and holder only; it cannot weigh an object or produce a mass reading, even with an available scale. |
| `u01_if_dirty` | kitchen | Check pan 1 with the chef arm's camera. If it is dirty, have the helper arm put it in the sink; if it is clean, leave it on the stove. | cannot (inexpressible): RoboLang 1.3 has no conditional statement, and inspect yields no condition a plan could test. |
| `u02_if_damaged` | yard | Have crane 2 scan container 102 with its camera; if the scan shows damage, move it to the workshop, otherwise to berth 2. | cannot (inexpressible): RoboLang 1.3 has no conditional statement, and inspect yields no condition a plan could test. |
| `u03_until_stop` | pharmacy | Keep moving every vial that arrives at intake to packing with picker1, until I tell you to stop. | cannot (inexpressible): RoboLang 1.3 has no loop; the number of repetitions is unbounded, so it cannot be unrolled. |
| `u04_back_and_forth` | kitchen | Have the runner keep driving back and forth between the pass and the sink for as long as service lasts. | cannot (inexpressible): RoboLang 1.3 has no loop; the number of repetitions is unbounded, so it cannot be unrolled. |
| `u05_below_8c` | pharmacy | Only if the insulin reads below 8 °C on picker1's thermometer, move it to packing. | cannot (inexpressible): RoboLang 1.3 has no numeric reading and no comparison atom. |
| `u06_cooled_60c` | kitchen | Probe pan 1 with the temperature probe and move it to the pass only once it has cooled below 60 °C. | cannot (inexpressible): RoboLang 1.3 has no numeric reading and no comparison atom. |
| `u09_avoid_quay` | yard | Have straddle1 take container 203 to the out gate, and never drive through the quay on the way. | cannot (inexpressible): Staying inside or out of a region while moving needs in_region / no_go_region, deferred (E211). |
| `u10_inside_clean_room` | pharmacy | Have picker1 bring vial A2 to packing, staying inside the clean room the whole way. | cannot (inexpressible): Staying inside or out of a region while moving needs in_region / no_go_region, deferred (E211). |
| `u11_wait_go` | pharmacy | Wait for the pharmacist to send "GO" on the pharmacy channel, then have picker1 bring tray 1 to dispatch. | cannot (inexpressible): Waiting for an incoming message needs message_received, deferred (E211). |
| `u12_radio_clearance` | yard | Leave container 305 where it is until the port authority radios clearance, then have straddle1 take it to the out gate. | cannot (inexpressible): Waiting for an incoming message needs message_received, deferred (E211). |
| `u13_simultaneous_lift` | yard | Have crane 1 and crane 2 lift container 101 and container 102 at exactly the same time. | cannot (inexpressible): Robots do their steps one after another; nothing runs at the same time. |
| `u14_while_carrying` | kitchen | While the helper arm carries tray B to the pass, the chef arm must look at pan 1 with its camera at that same moment. | cannot (inexpressible): Robots do their steps one after another; nothing runs at the same time. |
| `u15_open_oven` | kitchen | Open oven_door_1, then have the helper arm take tray A out to the pass. | cannot (inexpressible): The plan has six statements: move, grasp, place, inspect, wait, communicate. |
| `u16_charge_drone` | yard | Charge the drone at the workshop, then fly it to berth 1. | cannot (inexpressible): The plan has six statements: move, grasp, place, inspect, wait, communicate. |
| `u17_at_1400` | pharmacy | At 14:00 today, have picker1 take tray 2 to dispatch. | cannot (inexpressible): There is no clock; waits are relative durations only. |
| `u18_every_morning` | yard | Every morning at 06:00, have tug1 drive to the in gate. | cannot (inexpressible): There is no clock; waits are relative durations only. |
| `u19_send_batch` | pharmacy | Scan the insulin with picker1's scanner and send the batch number you read to the "dispatch" channel. | cannot (inexpressible): communicate sends a literal string; a measured value cannot be put in it. |
| `u20_report_temp` | kitchen | Probe pan 1's temperature and tell the "pass" channel the value you measured. | cannot (inexpressible): communicate sends a literal string; a measured value cannot be put in it. |

## clarify (18)

| id | world | request | expected |
|---|---|---|---|
| `a01_which_vial` | pharmacy | Have picker1 bring the vial from shelf A to packing. | clarify, candidates ['vial_a1', 'vial_a2'] |
| `a02_which_container` | yard | Have straddle1 move the container at stack B to the out gate. | clarify, candidates ['cont_203', 'cont_204'] |
| `a03_which_pan` | kitchen | Have the helper arm put the pan on prep 2. | clarify, candidates ['pan_1', 'pan_2'] |
| `a04_which_tray` | pharmacy | Have picker1 take the tray from packing to dispatch. | clarify, candidates ['tray_1', 'tray_2'] |
| `a05_which_berth_container` | yard | Have crane 1 put the container from berth 1 on stack C. | clarify, candidates ['cont_101', 'cont_102'] |
| `a06_which_crane` | yard | Have the crane put container 305 on berth 2. | clarify, candidates ['crane1', 'crane2'] |
| `a07_a_crane` | yard | Have the crane take container 203 to the wash. | clarify, candidates ['crane1', 'crane2'] |
| `a08_which_cart` | pharmacy | Have the cart drive to the waste area. | clarify, candidates ['cart1', 'cart2'] |
| `a09_which_fridge` | pharmacy | Have picker1 put vial A1 in the fridge. | clarify, candidates ['fridge_1', 'fridge_2'] |
| `a10_which_berth` | yard | Have straddle1 move container 203 to the berth. | clarify, candidates ['berth_1', 'berth_2'] |
| `a11_which_prep` | kitchen | Have the helper arm put bowl 2 on the prep station. | clarify, candidates ['prep_1', 'prep_2'] |
| `a12_which_shelf` | pharmacy | Have picker1 put tray 1 on the shelf. | clarify, candidates ['shelf_a', 'shelf_b'] |
| `a13_keep_it_slow` | kitchen | Have the helper arm carry pan 1 to the sink, and keep it slow. | clarify, missing speed |
| `a16_a_moment` | kitchen | Have the helper arm go to the oven, wait a moment there, then come back to prep 2. | clarify, missing duration |
| `a17_for_a_bit` | yard | Have tug1 drive to the out gate, pause for a bit, then go to the workshop. | clarify, missing duration |
| `a18_hold_briefly` | pharmacy | Have cart2 move to dispatch, hold there briefly, then go to the charger. | clarify, missing duration |
| `a21_operator_speed_limit` | kitchen | Have the helper arm take bowl 2 from the fridge to prep 2 using my required maximum speed limit. | clarify, missing speed (operator requirement) |
| `a22_operator_wait_deadline` | yard | Once lane 2 clears, have straddle1 take container 204 to stack C, subject to my maximum permitted waiting time. | clarify, missing timeout (operator requirement) |
