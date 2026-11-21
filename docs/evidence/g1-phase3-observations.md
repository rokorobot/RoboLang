# G1 phase 3: goal and precondition verdicts from observations (checks O1-O7)

Date: 2026-11-21. Code: [`backends/unitree_g1/`](../../backends/unitree_g1/) (package `robolang-g1`), decision
[0046](../../spec/decisions/0046-humanoid-backend-unitree-g1.md) section 4, item 3 (U8); contract in
[`g1-backend-v0.1.md`](../../spec/g1-backend-v0.1.md), "Observations and verdicts". Simulation only. This mirrors the ROS 2
backend's phase 3 ([evidence](h3-phase3-observations.md)).

**Result: O1-O7 all pass in two consecutive full runs of `python -m robolang_g1.phase3` (29 and 31 minutes, exit 0), phase 2 (E1-E7)
passes, the harness (P1-P6) passes, `pytest` 161 passed, `ruff` clean.** `w_carry_hold`, `e_pick_place` and `w_part_precheck` end
`SUCCEEDED` from observations alone (6 of 6 in each run), the goal failures end S301 and S302, the preconditions S101 and S102
before the first command, and O6's agreement with the oracle is 34 of 34 with no false success in either run. What is not clean:
the carry pose's measured clearance is 1.6-2.2 cm, not 2 cm and more (planned 2.1 cm in the model; "Deviations"), one stand-in run in
each of phase 2 and phase 3 run 1 was not real-time valid (host load, not counted), and moves vary from 24 to 200 s.


## What the owner decided on 2026-11-21, and what was built

1. **An observation motion for a carried object: "0046 U8 exception, approved by the owner 2026-11-21".** In the carry pose a
   held object's marker is out of the head camera's view (next section), so `holding`, and `object_at` of a carried object,
   could never be observed. After the last step, and only if a goal atom names an object the robot holds, the arm shows it to
   the camera and goes back: arm only (no walking, no waist), in the site's arm-work stance, along the grasp's own `clear` and
   `lift` poses with one raised pose between them, then back to the carry pose; never for a precondition. Before it moves, the
   whole path is checked against the declared tables and obstacles of the site at the current station (below); **if it is not
   shown clear the arm does not move**, the snapshot is taken as the plan left the robot, and the atoms about the object are
   `unknown` (S302). A cancel, revoke or deadline during it stops it like any arm motion (the biped stop, hands unchanged). The
   record has an `observation_motion` entry apart from the steps: the phase, the atoms that asked, `performed`, the waypoints,
   the clearance (closest link and box), the stance, and the events. O3 and O6 are unchanged; **`w_carry_hold`'s goal
   (`holding r crate`) passes only through this motion**, and so does O4's goal without a `place` step (S301 needs the box seen
   in the hand).
2. **A carry pose that clears the held object from every link of the robot by at least 2 cm**, as close to the walking
   controller's arm pose as that allows (below).
3. **Arm paths of grasp and place checked against the declared tables** with the same margins, S502 if no path clears (below).

## A held object cannot be seen from the carry pose

`holding r o` needs the marker of `o` seen at the hand, and `object_at o l` of a carried object needs it seen too. In the carry
pose (the walking controller's own arm pose) the hand hangs at the side of the thigh, **behind the head camera's field of view:
the marker is 46° below the axis, against a vertical field of ±29°** (model and measured: 0 detections in 28 frames walking
and 10 at rest, in every run below). A waist lean up to 0.5 rad does not bring it into view (the hand ends up behind the
camera). Three acceptance tasks have this shape (`w_carry_hold`, `e_hold_box`, `l_vial_hold`).

**The carry-pose test the owner asked for.** Arm poses found offline on the robot's own model (inverse kinematics, MuJoCo
distances; the 9x9x8 cm, 0.3 kg crate welded) that keep the marker in view with the object clear of the body, and then the
protocol, three times each on the free stance, with the real grasp: walk 2 m at 0.5 m/s (4 s), stand 10 s, a 90° heading
change by the shuffle. [`exp_carry_protocol.py`](g1-phase3/exp_carry_protocol.py), results
[`carry_pose_*.json`](g1-phase3/), candidates `pose_candidates_*.json`. The first column is the walking controller's pose as
phase 2 held a carried object (before the hip-clearance fix); the last is the carry pose of the fix (section after next).

| | walking controller's pose | P1 in view: pinch (0.22, -0.15, 0.04) | P2 in view: pinch (0.30, -0.15, 0.00) | the carry pose now (hip clear) |
|---|---|---|---|---|
| marker against the field (±29° vertical, ±36.5°), at rest | not seen (41° below the axis in the model) | 18.5° below, 22.9° right, 0.46 m | 12.2° below, 19.3° right, 0.53 m | not seen |
| object clear of the torso / elbow (model) | the object touches the right hip link | 12.7 cm / 4.9 cm | 20 cm / 7 cm | plan 2.1 cm in the model; **measured at rest 1.6-2.2 cm** (below) |
| joint distance from the walking arm pose | 0 | 0.30 rad rms, right arm up to 1.07 | 0.18 rad rms, right arm up to 0.46 | 0.06 rad box, 0.08 rad crate (norm over the 7 right-arm joints, about 0.025 rms) |
| fell | no, no, no (max tilt 2.3°) | no, no, no (5.2°) | no, no, no (6.1°) | no, no, no (max tilt 2.4°) |
| forward in 4 s (2.0 m expected) | 1.846, 1.842, 1.840 | **-0.03, -0.01, -0.00: it did not walk** | 2.034, 2.049, 2.081 | 1.891, 1.885, 1.884 |
| sideways | -0.09, -0.07, -0.07 | +0.01, +0.02, +0.02 | -0.04, -0.04, -0.12 | -0.054, -0.040, -0.034 |
| heading change over the walk (rad) | -0.05, -0.06, -0.05 | -0.14, -0.01, +0.03 | **+0.32, +0.34, +0.27** | -0.008, -0.007, -0.007 |
| torso sway at rest, max deviation over 8 s (P3's limit 2 cm) | 0.39, 0.38, 0.39 cm | 0.91, 1.02, 0.99 cm | 1.45, 1.76, **2.18** cm | 0.38, 0.38, 0.38 cm |
| marker detections walking / at rest (confidence, spread) | 0/28, 0/10 | 28/28 (1.0, 0.2-0.7 cm), 10/10 | 28/28 (1.0, 0.6-0.8 cm), 10/10 | 0/28, 0/10 |
| arm tracking, max over the arm joints, walking / rest | 0.020 / 0.011 rad | 0.026-0.034 / 0.025-0.027 | 0.030-0.031 / 0.025 | 0.022 / 0.013 rad |
| 90° turn by the shuffle: error (rad), time, drift | -0.17, -0.20, -0.16; 20 s; 44-54 cm | not turned (+0.06, -0.08, +0.08 of 1.57; 120 s) | -1.00 (45 s), +0.15, +0.26; 12-74 cm | -0.101, -0.075, -0.099; 19.7 s; 44-50 cm |

P1 is unusable (the policy ignores the walking command with the arm that far from its trained pose). P2 walks but is not
clearly stable (four to five times the sway, a 0.3 rad curve, one turn that barely worked), so the carry pose was **not**
changed to an in-view pose; the owner chose the excursion.

## The hip-clearance carry pose

In the walking controller's arm pose the welded object is **0.6 cm (the 8 cm box) and 0.1 cm (the 9 cm crate) from the right
hip link** in the robot's own model (the scene lets an attached object pass through, so nothing showed). `Operations._plan_carry`
moves the right arm's seven joints from that pose only as far as it takes for the object, welded as the grasp holds it (its
centre and axes in the hand, from the plan before the arm moves and again from the measured grasp), to clear every link of the
robot by **2.1 cm in the model** (`CARRY_PLAN_M`; the holding hand's own palm and fingers are excepted: they touch it by design).
S502 before any arm motion if there is none. The limit is geometric: the crate is 2.6-3.1 cm from the wrist-pitch link in any arm
pose, and a plan for 3 cm found nothing for it; a first version planned for 2.5-3 cm moved the arm further from the walking pose
and the next `move` to the shelf then failed with S504 (the servo), so the plan is the smallest move that clears 2 cm with the
arm's tracking error ahead of it. A unit test
(`test_the_walking_controllers_arm_pose_has_the_held_object_against_the_hip_and_the_carry_pose_clears_it`) checks, for the box and
the crate, that the walking pose is under 2 cm and the carry pose at least 2 cm, on a model that did the planning; the paths
test is `test_the_grasp_and_place_paths_clear_the_declared_tables_with_the_margins_and_the_check_refuses_when_they_do_not`.

**What is measured is less than what is planned, and this is a deviation from "at least 2 cm" (accepted by the owner, 2026-11-21: "accept 1.6 cm as measured").** In the 15 grasps of the final
phase 2 and phase 3 (second) runs the record's `carry_pose.object_to_nearest_link_m` (computed from the measured joints once the
arm has reached the carry pose, so it includes the arm's tracking error, 0.01-0.03 rad) is **1.61-2.19 cm**: box (11 grasps) 1.61, 1.62, 1.74,
1.82, 1.84, 1.84, 1.85, 1.86, 1.94, 2.08, 2.12; crate (4) 1.94, 2.07, 2.09, 2.19 (the closest link is always `right_hip_pitch_link`);
1.65 cm in the carry protocol's crate. The model says 2.1 cm for each; the measured arm sits 0 to 0.5 cm closer. **10 of the 15
are under 2 cm as measured**, none under 1.6 cm, against 0.1-0.6 cm before the fix. The owner chose to accept 1.6 cm as measured over planning for 2.5 cm
(which measured 2.0 cm in the protocol run of that variant but broke the shelf approach with S504); the plan stays 2.1 cm.

## The arm paths of grasp and place, against the declared tables

The scene's hands touch neither objects nor tables, so the simulator never showed what the arm does near a table. With the
same geometry (`kinematics.Clearance`: MuJoCo distances on the robot's collision shapes, the held object as a box fixed to the
hand) and the declared tables grown by 3 cm sideways and 1.5 cm vertically:

* **They did not clear.** At the shipped grasp (hand pitched 15°, the pinch 1 cm below the object's centre) the open fingers
  were **3.2 cm inside the table** at the grasp pose and 3.5 cm along the approach from the hip, and on the retreat the held
  object passed **1 cm through the table's edge**; the same at the shelf for the place. The arm links themselves cleared by
  1.7 cm or more. (`exp_pathclear.py` and `exp_graspgeom.py`.)
* **The fix has three parts.** The hand comes in over the table through two raised poses (the clear-of-the-hip pose lifted
  above the pre-grasp height, tried 0-10 cm further out beside the body if the object would pass the table's corner, and that
  height straight above the pre-grasp) instead of a straight line from the hip, which crosses the table's edge. Every leg of
  every arm path is checked before the arm moves; a reach with no clear path is dropped for the next, S502 if none is left. And the
  grasp geometry changed: **no hand pose keeps both the open hand off the table by a margin and the closed fingers off the
  object's top face**, where the marker is (a table of pitches 0-30° and pinches -1 to +3 cm in
  [`exp_pitchrise.py`](g1-phase3/exp_pitchrise.py)). A first try with the pinch 3 cm above the centre cleared the table and
  put the fingers over the marker: S506, "not seen at the hand". Flat with the pinch at the centre, the open hand just touches
  the table top and the closed fingers stay level with the top face; so **on the legs where the hand is meant to be at the
  object (the last approach, the first lift, the lowering and the release) the hand and the object may overlap the table by
  1 cm and no more**, with no vertical margin (it was 5 mm; the place at the shelf then read 0.7 cm and failed E3 and E4 with S502). That is the one place the margins are not 3 cm and 1.5 cm.
* The check passes with the shipped sites at the bench, the shelf and the dock (grasp and place), with 0.1-0.5 cm over the limit on the closest
  leg (the held object just lifted off its table, within the 1 cm), and refuses with S502 when a post is declared where the hand
  has to go (test).

## Results

**Phase 3 (`python -m robolang_g1.phase3`), two full runs on the final code, exit 0 both:**

| | run 1 | run 2 |
|---|---|---|
| O1 atom verdicts (30 atoms, four scenes) | pass, 148 s | pass, 152 s |
| O2 preconditions on `w_part_precheck.rob` (true, false -> S101 with 0 commands, unknown -> S102 with 0 commands) | pass, 218 s | pass, 265 s |
| O3 successes: `e_pick_place` x2, `w_carry_hold` x2, `w_part_precheck` x2 | 6 `SUCCEEDED`, 762 s | 6 `SUCCEEDED`, 877 s |
| O4 goal failures (no place -> S301; camera covered -> S302; blocked view pose -> S302 with no arm motion; cancel in the observation motion -> S511) | pass, 506 s | pass, 473 s |
| O5 `@verify` (`max_age` 1 ms -> S302; relaxed; defaults) | pass, 103 s | pass, 103 s |
| O6 agreement with the oracle where it decides; false successes | 34 of 34, 0 | 34 of 34, 0 |
| O7 Core outcome accepted by `robolang.core` (8 `SUCCEEDED` records each) | pass | pass |
| elapsed | 29 min | 31 min |
| `realtime_valid` | 18 of 19 runs: `o4_no_place` ratio 0.968, worst second 0.21, 2 stalls (host load) | 19 of 19 (lowest ratio 0.963, worst second 0.9945) |

Run 1's invalid run is the S301 case; it is not counted (0046 U13), and the same case was valid and passed in run 2. The
four `w_carry_hold` goals (two in each run) are the ones that pass through the observation motion: `holding` true (the marker about
5 cm from the hand) and `robot_at` true, `observation_motion` performed in all four. In O3 the goal of `e_pick_place` and
`w_part_precheck` is `object_at` (true, the station marker's region) with no motion.

**Every `robot_at` verdict, against the brief's 5 cm / 0.1 rad** (offset of the station marker's pose from the station's pose, from
the camera; "margin" is the limit minus the offset; the verdict is `true` only to 5 mm and 0.005 rad inside the limit):

| run | scenario | verdict | the oracle | offset cm | offset rad | margin cm | margin rad |
|---|---|---|---|---|---|---|---|
| 1 | A at the bench (`bench`) | true | true | 1.23 | 0.0445 | 3.77 | 0.0555 |
| 1 | B 0.30 m short of the bench | false | false | 31.4 | 0.0413 | -26.4 | 0.0587 |
| 1 | B2 heading 0.2 rad off | false | false | 0.95 | 0.2422 | 4.05 | -0.1422 |
| 1 | `w_carry_hold` run 1, goal (`dst`) | true | true | 1.63 | 0.0315 | 3.37 | 0.0685 |
| 1 | `w_carry_hold` run 2, goal | true | true | 1.52 | **0.0877** | 3.48 | **0.0123** |
| 1 | O5 relaxed / defaults | true, true | true, true | 0.59, 2.45 | 0.0014, 0.0091 | 4.41, 2.55 | 0.0986, 0.0909 |
| 2 | A at the bench | true | true | 1.20 | 0.0434 | 3.80 | 0.0566 |
| 2 | B 0.30 m short of the bench | false | false | 31.5 | 0.0427 | -26.5 | 0.0573 |
| 2 | B2 heading 0.2 rad off | false | false | 0.97 | 0.2433 | 4.03 | -0.1433 |
| 2 | `w_carry_hold` run 1, goal | true | true | 2.24 | **0.0906** | 2.76 | **0.0094** |
| 2 | `w_carry_hold` run 2, goal | true | true | 3.10 | 0.0525 | 1.90 | 0.0475 |
| 2 | O5 relaxed / defaults | true, true | true, true | 0.62, 0.26 | 0.0010, 0.0083 | 4.38, 4.74 | 0.0990, 0.0917 |

(Scene A also reads `robot_at shelf` and `pad` `false` from the bench, with the bench's offset; scene A with the camera covered,
`max_age` 1 ms, and the scenes where no station marker is in view read `unknown`, as expected.) The smallest margins are in
**heading**: the arrival of `move` is within 0.09 rad by design, and the largest true verdict is 0.0906 rad, 0.0094 rad inside
the limit and 0.0044 inside the margin. Position margins are 1.9 cm and more.

**Phase 2 on the final code (`python -m robolang_g1.phase2`, exit 0, 21 minutes):** E1-E7 pass. Old numbers are the two runs
in [`g1-phase2-executor.md`](g1-phase2-executor.md). Real time: 14 of 15 runs valid (`e1_tour_1`: ratio 0.957, 14 stalls in its
first 17 s, the host; E1's second tour and its covered-marker run were valid).

| check | phase 2 as it was | now |
|---|---|---|
| E1 oracle error at the three stations | 0.7-3.8 cm, 0.002-0.057 rad | tour 1: 3.5 cm / 0.020, 3.1 cm / 0.074, 2.7 cm / 0.085 rad; tour 2: 0.3 cm / 0.006, 3.7 cm / 0.063, 0.9 cm / 0.0003 rad |
| E2 grasp | lift 8.8-8.9 cm, marker 5.4-5.9 cm from the hand, moved 1.8-2.1 cm | lift 14.3-14.4 cm, marker 5.0-5.1 cm from the hand, moved 1.8-2.0 cm |
| E3 place | box 2.741-2.745, 2.301-2.304; wrist 4.6 cm from carry | box 2.745, 2.305; wrist 4.7 cm from carry |
| E4 `e_pick_place` | move 24, grasp 21, move 64-145, place 18 s; `STEPS_COMPLETED`, exit 4 | 24, 28, 61, 25 s; `SUCCEEDED`, exit 0; robot at dst 2.3 cm / 0.009 rad |
| E4 `w_carry_hold` | move 24, grasp 21, move 50-51 s; robot at dst 1.9-2.4 cm / 0.018-0.025 rad | 24, 28, 53 s; `SUCCEEDED`, exit 0; crate on the right hand; robot at dst 2.6 cm / 0.077 rad |
| E5 stop | S511 1364/2181 ms, S513 1440/1452, S512 616/612, S516 | S511 2053 ms (pelvis 0.4 cm), S513 668, S512 616 (box still attached), S516 (fell at 10.03 s, nothing after) |
| E6 refusals | five codes, 0 commands | the same |
| E7 arm work, supported (limit 3 cm) | worst 2.1 cm | worst 2.0 cm; free-stance trial: the robot moved 15.1 cm (path 52 cm) and the step failed with S502 (no carry pose after the free stance had moved it), where it was 12-18 cm and `STEPS_COMPLETED` |

The grasp lifts higher because the lift waypoint is now 10 cm above a pinch at the object's centre (phase 2: a 15° hand with the
pinch 1 cm below it); the step takes 28 s instead of 21 (the raised waypoints). The `w_carry_hold` heading, 0.077 rad against
0.018-0.025 before, is where `move` stopped inside its 0.09 rad; the camera's `robot_at` heading in the four `w_carry_hold` runs of phase 3
is 0.032, 0.088, 0.091 and 0.053 rad. Phase 2 judged the pose after the run; see "Findings" for why the pelvis now stays welded until then.

**The harness (phase 1, `python -m robolang_g1.harness`): P1-P6 pass** (forward 1.72-1.75 m in 4 s at 0.5 m/s; coast 0.11-0.14 m and
sway 0.9-1.1 cm; the carry-walk 2.16 m with the 0.3 kg payload; marker error 0.5-0.65 cm at 0.94 m; real-time ratio 1.0).
**pytest** `backends/unitree_g1`: 161 passed (112 s, `MUJOCO_GL=egl`; 154 passed and 7 skipped without it). **ruff**
`python -m ruff check .`: all checks passed.

**The carry protocol with the final carry pose** (three runs, `exp_carry_protocol.py newcarry2`, the 0.3 kg crate as grasped,
free stance; the table above, last column). Against the walking controller's pose with the object at the hip: forward 1.885
against 1.84 m, sideways -0.04 against -0.07, heading change -0.007 against -0.05 rad, sway at rest 0.38 against 0.39 cm, the
shuffle turn short by 0.092 rad on average against 0.18, 19.7 s against 20 s. None fell, tilt 2.4 degrees. The new pose does
not make the walk worse; it is slightly better, and the marker is out of view as before (0 detections in 28 frames walking and 10
at rest).


**Files** (in [`g1-phase3/`](g1-phase3/)): `phase3-run1.json`, `phase3-run2.json` (the two runs: every check's details, every
`robot_at` verdict, every stand-in run's real-time numbers), `phase2-after-phase3.json`, `harness-after-phase3.json`,
`carry_pose_final.json` (the protocol with the final carry pose), and `records/`: run records of the second phase 3 run (the
`w_carry_hold` run with the observation motion, S101, S102, S301, S302 with a covered camera, S302 with a blocked view pose and no
arm motion, S511 during the observation motion, S302 from `@verify`, and `w_part_precheck` run 2).

## How a verdict is made, and the tolerances

The contract is in the spec. What was chosen, and why:

* **Fusion tolerance: 2 cm in position, and 0.05 rad in rotation for station markers.** Measured on the stand-in
  ([`exp_noise.py`](g1-phase3/exp_noise.py): three 10-frame snapshots at each of three ranges, the robot standing free): the
  largest deviation of any frame from the median position is 0.3-0.5 cm for the 6.4 cm object marker at 0.5 m, 1.7 cm at 0.9
  m (detected in 5 of 10 frames there, the detection limit), and for the 24 cm station marker 0.8 cm at 1.7 m, 1.1 cm at 2.1
  m and 1.5 cm at 2.5 m (it grows with the square of the range, as the depth uncertainty of a marker does). The ROS 2
  backend's 1 cm would have given a confidence of 0.5 for a station marker at 2.1 m although every frame was good; 2 cm covers
  the worst frame measured (1.7 cm) and is still far inside a region's depth (18 cm) and the 5 cm of `robot_at`. A frame's
  rotation (station markers only, the only ones whose rotation is used) is within 0.007-0.015 rad of the median at 1.7-2.5 m,
  so 0.05 rad; an object marker's rotation is 0.03-0.06 rad of noise and is not used, so it is not asked to agree. A station
  marker's pose must also put the vertical where the IMU puts it, to 0.03 rad (measured 0.004-0.015 at 1.7-2.5 m): the wrong
  one of the two poses a planar target allows is several degrees off.
* **Margins in favour of `unknown`.** `object_at`: a marker within 1.5 cm of the region's boundary is `unknown` (the depth is
  the weak axis; the camera's median is 0.4-0.9 cm from the oracle at the stations). `robot_at`: inside by at least 5 mm and
  0.005 rad is `true`, outside by that much is `false`, between is `unknown` (the measured offset of a station marker against
  the oracle is 0.4-0.9 cm and 0.001-0.005 rad, phase 2). The brief's 5 cm / 0.1 rad stand; the margins are a tightening,
  never a loosening. (The heading margin was 0.01 rad at first: a move that ended at 0.0903 rad, inside the 0.09 `move`
  promises, then read `unknown`; 0.005 rad is the largest measured systematic error.)
* **`holding`**: a hand is "closed" within 0.25 rad of the Dex3 closing pose (the tolerance `grasp` uses) and "open" within
  0.25 rad of all-zero; anything between is `unknown`. "At the hand" is the phase 2 radius, 9 cm from the grasp point (4-5 cm
  when held right).
* **An object seen at a closed hand is at no place**, so `object_at` is `false` for it even when the station marker is not in
  view: a held object is not at a location (RoboLang's own meaning).
* **`robot_ready`**: `GetFsmId` is 500 (the stand-in's only state, the SDK's `Start`). A real robot's other walking ids
  (501, 801) are not accepted yet, so a real robot would be `false` in them: fail closed.

## Findings along the way

* **A table hid a station marker.** A first part_precheck layout with a wider bench table put the bench marker behind it as
  seen from the start pose: S102 by design. The visibility rule (0046 section 3) was a cone test with no occlusion; it now also
  tests the line of sight from the camera to the point against the site's declared tables (`perception.line_of_sight_blocked`),
  and the site starts 0.2 m before the station. The "item elsewhere" case is the far edge of the table, 6 cm outside the region.
* **A planar marker's wrong pose.** At 2.5-2.9 m the better-fitting of the two poses IPPE returns is often the wrong one (a
  vertical 0.13 rad off the IMU's), and the executor discarded the sighting. `solvePnPGeneric` gives both and the one that puts
  the vertical where the IMU does is taken (`perception.pick_by_gravity`).
* **`move` to the shelf failed with S517 in two of four full runs of `e_pick_place`** (phase 2's E4 had passed twice). After the
  carry the belief was 0.2-0.4 m off sideways, the shelf marker (low, behind the table) was hidden by the table from there, and
  the scan turned the robot away from it. The same move without a carried object, three times, succeeded in 47-60 s with the
  belief within 1-2 cm of the oracle ([`exp_shelf_move.py`](g1-phase3/exp_shelf_move.py)). Two changes: when the marker is not
  seen at the approach point the robot now steps toward the marker's side of the axis (up to three 0.2 m steps, looking after
  each) before it turns to look, and the belief carries the shuffle's backward drift (0.035 m a burst, measured in phase 2 and
  ignored by the dead reckoning). A first fix, shifting the approach point 0.3 m sideways where a declared table hides the
  marker, passed O3 4 of 4 but left a 0.3 m sideways correction for the final servo, which then failed E3 and E4 with S504 (the
  robot ended 6 cm too far forward, against the table, with the lateral axis "blocked"); it was removed.
* **A 50 s plan on the free stance turned the robot 0.33 rad and the hand closed on air (S506).** Found in a phase 3 run on a
  host with a load average of 25: `grasp` measured the object, then planned the reach and the carry pose (seconds to a minute
  when loaded) while the free stance drifted in heading at about 0.01 rad a second, and the arm then went to a pose planned for
  where the robot had been. `grasp` and `place` now weld the pelvis (`supported`) before they measure and plan, as the observation
  motion already did; the arm-work window of the record starts at the weld.
* **Releasing the weld after the goal snapshot moved the robot out of `robot_at`.** Phase 2's E4 failed once for `w_carry_hold`
  with the oracle's heading 0.1022 rad from the station (limit 0.1) while the camera had read 0.0837 at the snapshot. A diagnostic run
  with the oracle's heading over time ([`exp_heading_after_weld.py`](g1-phase3/exp_heading_after_weld.py)): +0.0441 rad, constant to
  0.0005 while welded for 13 s (the camera: 0.0436), then +0.0476, +0.0550, +0.0613 in the three half-seconds after the stance
  went back to free. The verdict was right; the check looked after the release. The pelvis now stays welded after an observation
  motion (nothing walks after the goal snapshot) and `move` frees it first; the final oracle sample then reads 0.0479 against the
  camera's 0.0451.
* **Phase 2's checks that expected `STEPS_COMPLETED` now expect `SUCCEEDED`**: E4 as the brief says, and E1, E2, E3 and E7
  for the same reason (their tasks have goals the observations now decide). Nothing else in them changed.

## Deviations

1. **The observation motion is a deviation from 0046 U8** ("goal from observations; no motion for it"): "0046 U8 exception,
   approved by the owner 2026-11-21", scoped as above (after the last step; only for a held object a goal atom names; arm only; the
   site's arm-work stance; clearance against the declared tables and obstacles or no motion; never for a precondition; its own
   `observation_motion` record). `w_carry_hold` passes only through it, which the brief's O3 allows ("say if").
2. **The carry pose clears 2.1 cm in the model and 1.6-2.2 cm as measured**, not "at least 2 cm" as measured (above); accepted by
   the owner on 2026-11-21.
3. **The grasp geometry differs from phase 2**: a flat hand with the pinch at the object's centre (phase 2: 15°, 1 cm below), the
   hand comes in over the table through raised poses, and on the legs where the hand is meant to be at the object the hand and
   object may overlap the declared table by 1 cm (3 cm sideways and 1.5 cm vertical margins elsewhere). The object is released
   1.2 cm above the surface at a place. The 1 cm is a tolerance of the clearance check on contact legs, not a measured clearance.
4. **Phase 2's checks changed** in what they expect: E1, E2, E3, E4 and E7 expect `SUCCEEDED` and exit 0 for tasks that have a
   goal (the brief says so for E4); the oracle error limits (5 cm, 0.1 rad) were not changed. E7's free-stance trial now fails with
   S502 instead of completing (the trial is the contrast, not a pass condition).
5. **Phase 3's O4(c)** (blocked view pose) no longer expects the free stance after the refusal: the pelvis stays welded, the
   check keeps "no arm motion" (the wrist moved 0.2 cm) and "the hand still holds".
6. **The fusion tolerance is 2 cm** (the ROS 2 backend's is 1 cm), with 0.05 rad for station markers and the IMU vertical
   within 0.03 rad: measured ranges above. `robot_at` margins 5 mm and 0.005 rad inside the brief's tolerance (a tightening).
7. **`robot_ready` accepts the FSM id 500 only.**
8. **The visibility rule** (0046 section 3) also tests the line of sight against the declared tables; `part_precheck` is a new site
   (`sites/part_precheck.json`), `obstacles` a new optional site key, `w_part_precheck.rob` unchanged.
9. **The run record is schema 0.2** (`robolang.g1.run`): `snapshots`, `observation_motion`, `goal`, `goal_evaluated`, `core_outcome`; exit
   codes 0 (`SUCCEEDED`), 3 (S302), 1 (any other failure), 2 (usage); phase 2's exit 4 (`STEPS_COMPLETED`) is gone for tasks with a goal.
10. **Test hooks** (`--test-hooks` only): `API_COVER_CAMERA` (1007) and `API_PLACE_OBJECT` (1008) on the stand-in's `robolang_sim`
    API, for the scenes O1, O2 and O4 need; the guard's allow-list knows them.
11. **Real-time invalid runs happened** (host load average 17-131 while the checks ran; the stand-in is paced to real time on a
    shared machine): one in phase 2's final run (`e1_tour_1`) and one in phase 3 run 1 (`o4_no_place`), listed above and not
    counted; earlier runs lost more of them.

## Open issues

* **Carry-pose clearance** (above): 1.6-2.2 cm measured, accepted by the owner on 2026-11-21. A larger margin needs the shelf
  servo to tolerate a larger arm move first.
* **The observation motion is 15 s of arm motion with the pelvis welded**, at the end of every task whose goal asks `holding` or
  `object_at` for an object the robot carries. The weld stands in for the balance a real G1's controller gives; a real arm raise
  with 0.3 kg while standing is unmeasured. The motion is clear only of the tables and obstacles a site declares, not of
  anything else in the room; an undeclared obstacle is not seen.
* **The marker and the fingers.** The marker is on the object's top face and the closed fingers (flat hand, pinch at the centre)
  stay level with it: the marker is seen at 4.9-5.5 cm from the hand, but an object with a smaller top face or a marker elsewhere
  needs another grasp, and no pose in the table of pitches and pinches ([`exp_pitchrise.py`](g1-phase3/exp_pitchrise.py)) both
  keeps the open hand off the table by a margin and the fingers off the marker.
* **`move` is slow and variable**: 24 s to the first station, 47-200 s to the shelf in the phase 3 runs (the limit is 300 s,
  the servo's 150 s). The shelf marker is low behind its table, and the approach is the part that failed (S517, S504) before the
  sidestep search. Not seen in the four full runs on the final code (phase 2 once, phase 3 three times); not proved gone.
* **`robot_at`'s heading is the weak axis**: `move` arrives inside 0.09 rad and the free stance drifts 0.01 rad a second, so a
  `robot_at` read some seconds after the arrival can be `unknown` or `false`; only the observation motion's weld (held-object
  goals) pins it. The largest true verdict is 0.0906 rad.
* **Phase 3 is long** (29-31 minutes) and, with phase 2 (21 minutes), the harness, `pytest` (112 s) and the setup, the CI job is
  about an hour on this host; the workflow's timeout is 170 minutes.
* **x86 was not run**; all of this is on aarch64.
* **`robot_ready`** is the FSM id 500 only; a real robot's 501 and 801 are `false` for now.
