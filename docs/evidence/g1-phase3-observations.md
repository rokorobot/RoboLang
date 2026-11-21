# G1 phase 3: goal and precondition verdicts from observations (checks O1-O7)

Date: 2026-11-21. Code: [`backends/unitree_g1/`](../../backends/unitree_g1/) (package `robolang-g1`), decision
[0046](../../spec/decisions/0046-humanoid-backend-unitree-g1.md) section 4, item 3 (U8); contract in
[`g1-backend-v0.1.md`](../../spec/g1-backend-v0.1.md), "Observations and verdicts". Simulation only. This mirrors the ROS 2
backend's phase 3 ([evidence](h3-phase3-observations.md)).

@@RESULT@@

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
| object clear of the torso / elbow (model) | the object touches the right hip link | 12.7 cm / 4.9 cm | 20 cm / 7 cm | @@CARRY_CLEAR@@ |
| joint distance from the walking arm pose | 0 | 0.30 rad rms, right arm up to 1.07 | 0.18 rad rms, right arm up to 0.46 | @@CARRY_DEV@@ |
| fell | no, no, no (max tilt 2.3°) | no, no, no (5.2°) | no, no, no (6.1°) | @@CARRY_FELL@@ |
| forward in 4 s (2.0 m expected) | 1.846, 1.842, 1.840 | **-0.03, -0.01, -0.00: it did not walk** | 2.034, 2.049, 2.081 | @@CARRY_FWD@@ |
| sideways | -0.09, -0.07, -0.07 | +0.01, +0.02, +0.02 | -0.04, -0.04, -0.12 | @@CARRY_SIDE@@ |
| heading change over the walk (rad) | -0.05, -0.06, -0.05 | -0.14, -0.01, +0.03 | **+0.32, +0.34, +0.27** | @@CARRY_HEAD@@ |
| torso sway at rest, max deviation over 8 s (P3's limit 2 cm) | 0.39, 0.38, 0.39 cm | 0.91, 1.02, 0.99 cm | 1.45, 1.76, **2.18** cm | @@CARRY_SWAY@@ |
| marker detections walking / at rest (confidence, spread) | 0/28, 0/10 | 28/28 (1.0, 0.2-0.7 cm), 10/10 | 28/28 (1.0, 0.6-0.8 cm), 10/10 | @@CARRY_DET@@ |
| arm tracking, max over the arm joints, walking / rest | 0.020 / 0.011 rad | 0.026-0.034 / 0.025-0.027 | 0.030-0.031 / 0.025 | @@CARRY_TRACK@@ |
| 90° turn by the shuffle: error (rad), time, drift | -0.17, -0.20, -0.16; 20 s; 44-54 cm | not turned (+0.06, -0.08, +0.08 of 1.57; 120 s) | -1.00 (45 s), +0.15, +0.26; 12-74 cm | @@CARRY_TURN@@ |

P1 is unusable (the policy ignores the walking command with the arm that far from its trained pose). P2 walks but is not
clearly stable (four to five times the sway, a 0.3 rad curve, one turn that barely worked), so the carry pose was **not**
changed to an in-view pose; the owner chose the excursion.

## The hip-clearance carry pose

In the walking controller's arm pose the welded object is **0.6 cm (the 8 cm box) and 0.1 cm (the 9 cm crate) from the right
hip link** in the robot's own model (the scene lets an attached object pass through, so nothing showed). `Operations._plan_carry`
moves the right arm's seven joints from that pose only as far as it takes for the object, welded as the grasp holds it (its
centre and axes in the hand, from the plan before the arm moves and again from the measured grasp), to clear every link of the
robot by the first of 3 cm, 2.75 cm and 2.5 cm that exists (the holding hand's own palm and fingers excepted: they touch it by
design). The limit is geometric: the object is 2.6-3.1 cm from the wrist-pitch link in any arm pose. S502 before any arm
motion if there is none. In the runs it moves the arm by 0.06-0.08 rad rms and the measured clearance, with the arm's own
tracking error (0.01-0.03 rad), is 2.6 cm. A unit test (`test_the_walking_controllers_arm_pose_has_the_held_object_against_the_hip_and_the_carry_pose_clears_it`)
checks, for the box and the crate, that the walking pose is under 2 cm and the carry pose at least 2 cm, on a model that did the
planning.

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

@@RESULTS@@

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
* **Phase 2's checks that expected `STEPS_COMPLETED` now expect `SUCCEEDED`**: E4 as the brief says, and E1, E2, E3 and E7
  for the same reason (their tasks have goals the observations now decide). Nothing else in them changed.

## Deviations

@@DEVIATIONS@@

## Open issues

@@OPEN@@
