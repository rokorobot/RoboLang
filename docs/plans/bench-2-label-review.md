# BENCH-2 label review — for GPT 6

You are reviewing the expected answers of a benchmark that tests whether AI models write RoboLang 1.3 robot programs, reject requests, or ask questions. A Claude model wrote the cases, and Claude models are tested, so an outside reviewer checks the labels before they are frozen.

For **every** case below, answer: **AGREE**, or **DISAGREE** with the label you would give and one sentence of reasons. Flag any wording that leaks the answer or reads unnaturally. Do not rewrite cases; disputed cases are rewritten or dropped by the owner.

## What the labels mean

- **feasible**: a RoboLang 1.3 program can do exactly what was asked in this world (decoys look otherwise but are not).
- **infeasible**: RoboLang could express it, but in this world no program can do it as asked. The right answer is a rejection naming the blocking entity.
- **unsupported**: RoboLang 1.3 cannot express it in any world. The right answer is a rejection naming the missing construct.
- **ambiguous**: more than one reasonable reading, and the world does not decide. The right answer is a question naming the slot and the candidates. A program that picks one reading counts as a critical error.

RoboLang 1.3 in one paragraph: a task has optional `@safety max_speed` limits, an optional `pre` (checked once), a required `goal` (end state: `object_at`, `robot_at`, `holding`, plus `robot_ready`, `region_clear`, `object_exists`, `sensor_ok`, combined with not/and/or), and a `plan` of `move`, `grasp`, `place`, `inspect … using sensor`, `wait for D`, `wait until C timeout D`, `communicate r to "ch" with "text"`. There is no if/else, loop, numeric reading, clock, concurrency, or force limit. A gripper holds one object. `wait until` always needs a timeout; when it times out, the run fails and stops.

## The worlds (as the model sees them)

### pharmacy

```
locations: intake, shelf_a, shelf_b, fridge_1, fridge_2, packing, dispatch, charger, waste
regions: clean_room (clear), corridor (not clear), cold_zone (clear)
robots: picker1 (at charger, has a gripper, ready, max speed 0.8 m/s, 1.5 rad/s); picker2 (at packing, has a gripper, not ready, max speed 0.8 m/s); cart1 (at dispatch, no gripper, ready, max speed 1.2 m/s); cart2 (at intake, no gripper, ready, max speed 1.2 m/s)
objects: vial_a1 (vial, at shelf_a); vial_a2 (vial, at shelf_a); vial_b1 (vial, at fridge_1); insulin_1 (vial, at fridge_2); tray_1 (tray, at packing); tray_2 (tray, at packing); bag_1 (bag, absent); box_d1 (box, at dispatch)
sensors: scan_p1 (on picker1, available); thermo_p1 (on picker1, not available); scan_p2 (on picker2, available); cam_cart1 (on cart1, available)
scheduled: after 5 s, sensor_available of thermo_p1 becomes true
scheduled: after 20 s, region_clear of corridor becomes true
```

### yard

```
locations: berth_1, berth_2, stack_a, stack_b, stack_c, gate_in, gate_out, workshop, wash
regions: quay (not clear), lane_1 (clear), lane_2 (not clear)
robots: crane1 (at berth_1, has a gripper, ready, max speed 0.5 m/s, 0.3 rad/s); crane2 (at berth_2, has a gripper, ready, max speed 0.5 m/s); straddle1 (at stack_a, has a gripper, ready, max speed 2 m/s); straddle2 (at stack_b, has a gripper, not ready, max speed 2 m/s); tug1 (at gate_in, no gripper, ready, max speed 3 m/s); drone1 (at workshop, no gripper, ready, max speed 5 m/s)
objects: cont_101 (container, at berth_1); cont_102 (container, at berth_1); cont_203 (container, at stack_b); cont_204 (container, at stack_b); cont_305 (reefer, at stack_c); cont_999 (container, absent); chassis_1 (chassis, at gate_in)
sensors: lidar_c1 (on crane1, available); cam_c2 (on crane2, available); seal_reader (on straddle1, not available); cam_drone (on drone1, available)
scheduled: after 45 s, region_clear of lane_2 becomes true
```

### kitchen

```
locations: pantry, fridge, prep_1, prep_2, stove, oven, pass, sink, bin
regions: hot_zone (not clear), pass_area (clear)
robots: chef_arm (at prep_1, has a gripper, ready, max speed 0.4 m/s, 2 rad/s); helper_arm (at prep_2, has a gripper, ready, max speed 0.4 m/s); runner (at pass, no gripper, ready, max speed 1 m/s)
objects: pan_1 (pan, at stove); pan_2 (pan, at sink); tray_a (tray, at oven); tray_b (tray, at prep_2); bowl_1 (bowl, at pantry); bowl_2 (bowl, at fridge); knife_1 (knife, held by chef_arm); cup_1 (cup, at pantry)
sensors: cam_chef (on chef_arm, available); probe_temp (on chef_arm, available); scale_1 (on helper_arm, not available); cam_runner (on runner, available)
scheduled: after 30 s, region_clear of hot_zone becomes true
```

## Questions the pilot raised (please answer these too)

1. `a16`, `a19`, `a20`: Haiku wrote a program and said in its message that the pause or timeout was its own choice. Should a disclosed default count as a critical silent guess, a lesser error, or correct?
2. `i10`, `i17`, `i18`: on an unknown object and contradictory end states, Haiku asked which reading was meant instead of rejecting. Should a question also count as correct there?
3. `d7`: Haiku argued that 'if it doesn't, do nothing' is a branch, because a timed-out wait fails the run. Is the label (feasible) right?
4. `a19`, `a20`, `a13`–`a15` (no timeout, no number for 'slow'): is asking right, or should a model choose a sensible value?

## feasible (20)

| id | world | request | label |
|---|---|---|---|
| `d1_bowl_in_fridge` | kitchen | Move the bowl that is in the fridge to prep 1 with the helper arm. | decoy; program, goal bowl_2 located_in prep_1 |
| `d2_a_picker` | pharmacy | Use a picker robot to bring vial A1 to packing. | decoy; program, goal vial_a1 located_in packing |
| `d3_lane2_right_now` | yard | Only if lane 2 is clear right now, have straddle1 move container 204 to stack A; otherwise do nothing. | decoy; program, goal cont_204 located_in stack_a; expected run state FAILED (S101) |
| `d4_corridor_busy` | pharmacy | The corridor is busy. Once it clears (allow 30 seconds), have picker1 bring tray 1 to dispatch. | decoy; program, goal tray_1 located_in dispatch |
| `d5_two_inspections` | kitchen | Check pan 1 with both the chef arm's camera and its temperature probe, then have the helper arm put pan 1 in the sink. | decoy; program, goal pan_1 located_in sink |
| `d6_never_faster` | yard | Never go faster than 40 centimetres per second: have crane 1 move container 102 to the wash. | decoy; program, goal cont_102 located_in wash |
| `d7_wait_or_nothing` | kitchen | Wait up to 45 seconds for the hot zone to clear. If it does, have the helper arm take tray A from the oven to the pass; if it doesn't, do nothing. | decoy; program, goal tray_a located_in pass |
| `d8_knife_handover` | kitchen | The chef arm is holding the knife. Get the knife to the helper arm so that the helper arm ends up holding it. | decoy; program, goal knife_1 carried_by helper_arm |
| `f01_straddle_gate` | yard | Have straddle1 take container 203 from stack B to the out gate, staying under 1.5 metres per second. | program, goal cont_203 located_in gate_out |
| `f02_scan_fridge_vial` | pharmacy | With picker1, take the vial that is in fridge 1 to packing, scanning it with scan_p1 before you pick it up. | program, goal vial_b1 located_in packing |
| `f03_hold_tray_at_pass` | kitchen | Have the helper arm pick up tray B and then wait at the pass holding it. | program, goal tray_b carried_by helper_arm; helper_arm located_in pass |
| `f04_crane_straddle_relay` | yard | Crane 1 lifts container 101 off berth 1 and sets it down at stack A; straddle 1 then takes it from stack A to stack C. | program, goal cont_101 located_in stack_c |
| `f05_three_to_packing` | pharmacy | With picker1, bring vial A1, vial A2 and the insulin to packing, scanning each one with scan_p1 just before picking it up. | program, goal vial_a1 located_in packing; vial_a2 located_in packing; insulin_1 located_in packing |
| `f06_hot_zone_then_sink` | kitchen | Once the hot zone is clear (allow up to 45 seconds), have the helper arm move pan 1 from the stove to the sink. | program, goal pan_1 located_in sink |
| `f07_thermo_then_dispatch` | pharmacy | As soon as picker1's thermometer is available (give it up to 10 seconds), check the insulin with it, then send picker1 to dispatch. | program, goal picker1 located_in dispatch |
| `f08_lane_or` | yard | Only if lane 1 or lane 2 is clear, have straddle1 move container 204 to stack A. | program, goal cont_204 located_in stack_a |
| `f09_quarter_minute` | kitchen | Have the helper arm go to the oven, pause there for a quarter of a minute, then come back to prep 2. | program, goal helper_arm located_in prep_2 |
| `f10_two_robots_two_boxes` | yard | Crane 2 should put container 102 at berth 2, and straddle 1 should take container 203 to the wash. | program, goal cont_102 located_in berth_2; cont_203 located_in wash |
| `f11_tray_message` | pharmacy | Have picker1 move tray 2 from packing to dispatch, then tell the "pharmacy" channel exactly TRAY READY. | program, goal tray_2 located_in dispatch |
| `f12_pass_clear_and_ready` | kitchen | Provided the pass area is clear and the helper arm is ready, have the helper arm put tray B at the pass. | program, goal tray_b located_in pass |

## infeasible (20)

| id | world | request | label |
|---|---|---|---|
| `i01_picker2_not_ready` | pharmacy | Have picker2 move tray 1 from packing to dispatch. | reject; blockers ['picker2']; fix: {"robots": {"picker2": {"ready": true}}} |
| `i02_straddle2_not_ready` | yard | Have straddle2 take container 204 from stack B to the in gate. | reject; blockers ['straddle2']; fix: {"robots": {"straddle2": {"ready": true}}} |
| `i03_straddle2_wash` | yard | Use straddle2 to carry container 203 to the wash. | reject; blockers ['straddle2']; fix: {"robots": {"straddle2": {"ready": true}}} |
| `i04_cart_no_gripper` | pharmacy | Have cart1 pick up vial A2 from shelf A and bring it to dispatch. | reject; blockers ['cart1']; fix: {"robots": {"cart1": {"gripper": true}}} |
| `i05_drone_lift` | yard | Have the drone lift chassis 1 at the in gate and put it on berth 2. | reject; blockers ['drone1']; fix: {"robots": {"drone1": {"gripper": true}}} |
| `i06_runner_bowl` | kitchen | Have the runner pick up bowl 1 from the pantry and put it on the pass. | reject; blockers ['runner']; fix: {"robots": {"runner": {"gripper": true}}} |
| `i07_absent_bag` | pharmacy | Bring the intake bag to packing with picker1. | reject; blockers ['bag_1']; fix: {"objects": {"bag_1": {"present": true, "location": "intake"}}} |
| `i08_absent_container` | yard | Have crane 1 move container 999 to the out gate. | reject; blockers ['cont_999']; fix: {"objects": {"cont_999": {"present": true, "location": "berth_1"}}} |
| `i09_no_morphine` | pharmacy | Move the morphine vial from shelf B to dispatch with picker1. | reject; blockers (the thing does not exist); fix: {"objects": {"morphine_1": {"present": true, "location": "shelf_b", "class": "vial"}}} |
| `i10_no_soup_pot` | kitchen | Have the helper arm put the soup pot on the stove. | reject; blockers (the thing does not exist); fix: {"objects": {"pot_1": {"present": true, "location": "prep_2", "class": "pot"}}} |
| `i11_seal_reader_down` | yard | Read container 203's seal with the seal reader on straddle 1, then have straddle 1 take it to the out gate. | reject; blockers ['seal_reader']; fix: {"sensors": {"seal_reader": {"available": true}}} |
| `i12_scale_down` | kitchen | Weigh tray B with the helper arm's scale, then have the helper arm put it on the pass. | reject; blockers ['scale_1']; fix: {"sensors": {"scale_1": {"available": true}}} |
| `i13_borrowed_drone_camera` | yard | Have crane 2 inspect container 102 using the drone's camera, then move it to berth 2. | reject; blockers ['crane2', 'cam_drone']; fix: {"sensors": {"cam_drone": {"robot": "crane2"}}} |
| `i14_borrowed_cart_camera` | pharmacy | Have picker1 scan vial B1 with cart1's camera, then bring it to packing. | reject; blockers ['picker1', 'cam_cart1']; fix: {"sensors": {"cam_cart1": {"robot": "picker1"}}} |
| `i15_two_bowls_one_gripper` | kitchen | Have the helper arm pick up both bowls and hold them both at once. | reject; blockers ['helper_arm']; A robot's gripper holds one object at a time (grasp needs an empty gripper), so no plan can hold both. |
| `i16_two_containers_one_crane` | yard | Have crane 1 hold container 101 and container 102 at the same time. | reject; blockers ['crane1']; A robot's gripper holds one object at a time, so no plan can hold both containers. |
| `i17_placed_and_held` | kitchen | Put pan 2 on the stove with the helper arm, and make sure the helper arm is still holding it at the end. | reject; blockers ['pan_2']; A held object is not at any location, so 'on the stove' and 'still held' cannot both be true at the end. |
| `i18_two_places` | pharmacy | Put vial A1 in fridge 1 with picker1, and have it also end up on the packing table. | reject; blockers ['vial_a1']; An object is at one location at a time, so it cannot end in fridge 1 and at packing. |
| `i19_deadline_hot_zone` | kitchen | Wait no more than 10 seconds for the hot zone to clear, then have the helper arm move pan 1 from the stove to the sink. Pan 1 must end up in the sink. | reject; blockers ['hot_zone']; fix: {"events": [{"at": {"value": "5", "unit": "s"}, "fact": "region_clear", "target": "hot_zone", "value": true}]} |
| `i20_deadline_lane2` | yard | Give lane 2 at most 20 seconds to clear, then have straddle1 take container 204 to stack C; it has to get there. | reject; blockers ['lane_2']; fix: {"events": [{"at": {"value": "10", "unit": "s"}, "fact": "region_clear", "target": "lane_2", "value": true}]} |

## unsupported (20)

| id | world | request | label |
|---|---|---|---|
| `u01_if_dirty` | kitchen | Check pan 1 with the chef arm's camera. If it is dirty, have the helper arm put it in the sink; if it is clean, leave it on the stove. | reject, `branching`: RoboLang 1.3 has no conditional statement, and inspect yields no condition a plan could test. |
| `u02_if_damaged` | yard | Have crane 2 scan container 102 with its camera; if the scan shows damage, move it to the workshop, otherwise to berth 2. | reject, `branching`: RoboLang 1.3 has no conditional statement, and inspect yields no condition a plan could test. |
| `u03_until_stop` | pharmacy | Keep moving every vial that arrives at intake to packing with picker1, until I tell you to stop. | reject, `repetition`: RoboLang 1.3 has no loop; the number of repetitions is unbounded, so it cannot be unrolled. |
| `u04_back_and_forth` | kitchen | Have the runner keep driving back and forth between the pass and the sink for as long as service lasts. | reject, `repetition`: RoboLang 1.3 has no loop; the number of repetitions is unbounded, so it cannot be unrolled. |
| `u05_below_8c` | pharmacy | Only if the insulin reads below 8 °C on picker1's thermometer, move it to packing. | reject, `numeric_sensing`: RoboLang 1.3 has no numeric reading and no comparison atom. |
| `u06_cooled_60c` | kitchen | Probe pan 1 with the temperature probe and move it to the pass only once it has cooled below 60 °C. | reject, `numeric_sensing`: RoboLang 1.3 has no numeric reading and no comparison atom. |
| `u07_grip_5n` | kitchen | Have the helper arm move bowl 2 from the fridge to prep 2, never squeezing with more than 5 newtons of grip force. | reject, `force_limit`: @safety has max_speed only; there is no force or load limit. |
| `u08_lift_20kn` | yard | Have crane 1 move container 101 to stack A without exceeding 20 kilonewtons of lifting force. | reject, `force_limit`: @safety has max_speed only; there is no force or load limit. |
| `u09_avoid_quay` | yard | Have straddle1 take container 203 to the out gate, and never drive through the quay on the way. | reject, `path_constraint`: Staying inside or out of a region while moving needs in_region / no_go_region, deferred (E211). |
| `u10_inside_clean_room` | pharmacy | Have picker1 bring vial A2 to packing, staying inside the clean room the whole way. | reject, `path_constraint`: Staying inside or out of a region while moving needs in_region / no_go_region, deferred (E211). |
| `u11_wait_go` | pharmacy | Wait for the pharmacist to send "GO" on the pharmacy channel, then have picker1 bring tray 1 to dispatch. | reject, `message_wait`: Waiting for an incoming message needs message_received, deferred (E211). |
| `u12_radio_clearance` | yard | Leave container 305 where it is until the port authority radios clearance, then have straddle1 take it to the out gate. | reject, `message_wait`: Waiting for an incoming message needs message_received, deferred (E211). |
| `u13_simultaneous_lift` | yard | Have crane 1 and crane 2 lift container 101 and container 102 at exactly the same time. | reject, `concurrency`: Robots do their steps one after another; nothing runs at the same time. |
| `u14_while_carrying` | kitchen | While the helper arm carries tray B to the pass, the chef arm must look at pan 1 with its camera at that same moment. | reject, `concurrency`: Robots do their steps one after another; nothing runs at the same time. |
| `u15_open_oven` | kitchen | Open the oven door, then have the helper arm take tray A out to the pass. | reject, `unknown_action`: The plan has six statements: move, grasp, place, inspect, wait, communicate. |
| `u16_charge_drone` | yard | Charge the drone at the workshop, then fly it to berth 1. | reject, `unknown_action`: The plan has six statements: move, grasp, place, inspect, wait, communicate. |
| `u17_at_1400` | pharmacy | At 14:00 today, have picker1 take tray 2 to dispatch. | reject, `clock_time`: There is no clock; waits are relative durations only. |
| `u18_every_morning` | yard | Every morning at 06:00, have tug1 drive to the in gate. | reject, `clock_time`: There is no clock; waits are relative durations only. |
| `u19_send_batch` | pharmacy | Scan the insulin with picker1's scanner and send the batch number you read to the "dispatch" channel. | reject, `value_report`: communicate sends a literal string; a measured value cannot be put in it. |
| `u20_report_temp` | kitchen | Probe pan 1's temperature and tell the "pass" channel the value you measured. | reject, `value_report`: communicate sends a literal string; a measured value cannot be put in it. |

## ambiguous (20)

| id | world | request | label |
|---|---|---|---|
| `a01_which_vial` | pharmacy | Have picker1 bring the vial from shelf A to packing. | ask, slot `object`, candidates ['vial_a1', 'vial_a2'] |
| `a02_which_container` | yard | Have straddle1 move the container at stack B to the out gate. | ask, slot `object`, candidates ['cont_203', 'cont_204'] |
| `a03_which_pan` | kitchen | Have the helper arm put the pan on prep 2. | ask, slot `object`, candidates ['pan_1', 'pan_2'] |
| `a04_which_tray` | pharmacy | Have picker1 take the tray from packing to dispatch. | ask, slot `object`, candidates ['tray_1', 'tray_2'] |
| `a05_which_berth_container` | yard | Have crane 1 put the container from berth 1 on stack C. | ask, slot `object`, candidates ['cont_101', 'cont_102'] |
| `a06_which_crane` | yard | Have the crane put container 305 on berth 2. | ask, slot `robot`, candidates ['crane1', 'crane2'] |
| `a07_a_crane` | yard | Have the crane take container 203 to the wash. | ask, slot `robot`, candidates ['crane1', 'crane2'] |
| `a08_which_cart` | pharmacy | Have the cart drive to the waste area. | ask, slot `robot`, candidates ['cart1', 'cart2'] |
| `a09_which_fridge` | pharmacy | Have picker1 put vial A1 in the fridge. | ask, slot `location`, candidates ['fridge_1', 'fridge_2'] |
| `a10_which_berth` | yard | Have straddle1 move container 203 to the berth. | ask, slot `location`, candidates ['berth_1', 'berth_2'] |
| `a11_which_prep` | kitchen | Have the helper arm put bowl 2 on the prep station. | ask, slot `location`, candidates ['prep_1', 'prep_2'] |
| `a12_which_shelf` | pharmacy | Have picker1 put tray 1 on the shelf. | ask, slot `location`, candidates ['shelf_a', 'shelf_b'] |
| `a13_keep_it_slow` | kitchen | Have the helper arm carry pan 1 to the sink, and keep it slow. | ask, slot `speed`, value missing |
| `a14_reduced_speed` | yard | Have crane 1 move container 102 to the wash at a reduced speed. | ask, slot `speed`, value missing |
| `a15_nice_and_slow` | pharmacy | Have picker1 bring vial B1 to packing gently, nice and slow. | ask, slot `speed`, value missing |
| `a16_a_moment` | kitchen | Have the helper arm go to the oven, wait a moment there, then come back to prep 2. | ask, slot `duration`, value missing |
| `a17_for_a_bit` | yard | Have tug1 drive to the out gate, pause for a bit, then go to the workshop. | ask, slot `duration`, value missing |
| `a18_hold_briefly` | pharmacy | Have cart2 move to dispatch, hold there briefly, then go to the charger. | ask, slot `duration`, value missing |
| `a19_until_corridor` | pharmacy | Wait until the corridor clears, then have picker1 bring tray 2 to dispatch. | ask, slot `timeout`, value missing |
| `a20_once_lane2` | yard | Once lane 2 clears, have straddle1 take container 204 to stack C. | ask, slot `timeout`, value missing |
