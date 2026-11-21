# G1 phase 3: goal and precondition verdicts from observations (checks O1-O7)

Date: 2026-11-21. Code: [`backends/unitree_g1/`](../../backends/unitree_g1/) (package `robolang-g1`), decision
[0046](../../spec/decisions/0046-humanoid-backend-unitree-g1.md) section 4, item 3 (U8); contract in
[`g1-backend-v0.1.md`](../../spec/g1-backend-v0.1.md), "Observations and verdicts". Simulation only. This mirrors the ROS 2
backend's phase 3 ([evidence](h3-phase3-observations.md)).

**Status: not complete. One thing waits for the owner, and it is the one that matters for `holding`.** Everything that
does not depend on it is built and passes (below); the scenes in which an object is *carried* (O1's `holding` true,
`w_carry_hold` in O3, the goal without a `place` step in O4) are **not in the checks yet**, and `go` stays false. Why:

## The finding: a held object cannot be seen from the carry pose

`holding r o` needs the marker of `o` seen at the hand, and `object_at o l` of a carried object needs it seen too. In the
carry pose (the walking controller's own arm pose, which the backend keeps while walking) the hand hangs at the side of the
thigh, **behind the head camera's field of view: the marker is 46° below the axis, against a vertical field of ±29°** (model
and measured: 0 detections in 28 frames while walking and 10 at rest, in every run below). A waist lean up to 0.5 rad does
not bring it into view (the hand ends up behind the camera). In the robot's own model the object also touches the right hip
link in that pose (−0.04 cm; the scene lets an attached object pass through, so nothing shows). So with the plan as it is,
after the last step the atoms about a carried object are `unknown`, and `w_carry_hold` can only end S302. Three acceptance
tasks have this shape (`w_carry_hold`, `e_hold_box`, `l_vial_hold`).

**The carry-pose test the owner asked for.** Arm poses found offline on the robot's own model (inverse kinematics, MuJoCo
distances; hand pitched as in the grasp; the 9x9x8 cm, 0.3 kg crate welded): two that keep the marker in view with the object
clear of the body. Then the protocol, three times each on the free stance, with the real grasp: walk 2 m at 0.5 m/s (4 s),
stand 10 s, a 90° heading change by the shuffle. [`exp_carry_protocol.py`](g1-phase3/exp_carry_protocol.py), results
[`carry_pose_*.json`](g1-phase3/), candidates `pose_candidates_*.json`.

| | current carry pose | P1: pinch (0.22, -0.15, 0.04) | P2: pinch (0.30, -0.15, 0.00) |
|---|---|---|---|
| marker against the field (±29° vertical, ±36.5° horizontal), measured at rest | not seen (41° below the axis in the model) | 18.5° below, 22.9° right, 0.46 m | 12.2° below, 19.3° right, 0.53 m |
| object / elbow clear of the torso (model) | object touches the right hip link | 12.7 cm / 4.9 cm | 20 cm / 7 cm |
| joint distance from the walking arm pose | 0 | 0.30 rad rms, right arm up to 1.07 | 0.18 rad rms, right arm up to 0.46 |
| fell | no, no, no (max tilt 2.3°) | no, no, no (max tilt 5.2°) | no, no, no (max tilt 6.1°) |
| forward in 4 s (2.0 m expected), per run | 1.846, 1.842, 1.840 | **-0.03, -0.01, -0.00: it did not walk** | 2.034, 2.049, 2.081 |
| sideways | -0.09, -0.07, -0.07 | +0.01, +0.02, +0.02 | -0.04, -0.04, -0.12 |
| heading change over the walk (rad) | -0.05, -0.06, -0.05 | -0.14, -0.01, +0.03 | **+0.32, +0.34, +0.27** |
| torso sway at rest, max deviation over 8 s (limit of P3: 2 cm) | 0.39, 0.38, 0.39 cm | 0.91, 1.02, 0.99 cm | 1.45, 1.76, **2.18** cm |
| marker detections walking / at rest (confidence, spread) | 0/28, 0/10 | 28/28 (1.0, 0.2-0.7 cm), 10/10 (1.0) | 28/28 (1.0, 0.6-0.8 cm), 10/10 (1.0) |
| arm tracking, max over the arm joints, walking / rest | 0.020 / 0.011 rad | 0.026-0.034 / 0.025-0.027 | 0.030-0.031 / 0.025 |
| 90° turn by the shuffle: error (rad), time, drift | -0.17, -0.20, -0.16; 20 s; 44-54 cm | not turned (+0.06, -0.08, +0.08 of 1.57; 120 s each) | -1.00 (45 s), +0.15 (23 s), +0.26 (22 s); 12-74 cm |

P1 is unusable: the policy ignores the walking command with the arm that far from its trained pose. P2 walks, but is not
clearly stable: four to five times the sway of the current pose (one run above the 2 cm of P3), a heading that curves by
0.3 rad over 2 m (the current pose: -0.05), and one turn that barely worked. So I did not switch the carry pose.

**Options, for the owner** (the report to the parent has the same):
 A. *The excursion* (parked on branch `room/g1-phase-3-excursion-parked`, not on this branch). After the last step, only if a
    goal atom names the held object: the arm shows it to the camera (clear of the hip, raised, the grasp's lift pose), the
    one snapshot is taken there, the arm returns to carry. Checked first against the site's declared tables and obstacles
    with the robot's own kinematics (margins 3 cm sideways, 1.5 cm vertical), and if not shown clear the arm stays and the
    atom is `unknown`. Measured: the box marker seen at 0.44 m, 6.2 cm from the hand's grasp point, the station marker in the
    same frames, arm tracking 0.01-0.03 rad; at the bench the path clears the table by 1.5 cm over the margins. It needed one
    waypoint that the grasp does not have ("raised"), because in the model the grasp's own path from the hip to the object goes
    through the table (the scene's hands touch nothing, so phase 2 never saw it). The excursion is motion outside the plan's
    steps, which 0046 U8 avoids; it is arm only, after the last step, in the arm-work stance (`supported`).
 B. *A carry pose that keeps the marker in view* (P2). No extra motion, the marker always in view, but it changes every walk
    with an object: the curve, the sway and the turns above, and phase 2's dead-reckoning gains and checks would have to be
    re-measured.
 I recommend A for 0.1 and B as a later item once the policy walks with such a pose as steadily as it does now. If the
 owner picks neither, `w_carry_hold` ends S302 by design.

## Result so far (everything that does not depend on that decision)

RESULTS_PLACEHOLDER

## What ran

* This sandbox: aarch64 Debian 12, 4 vCPU shared with other jobs, Python 3.11.2, MuJoCo 4.0.1, onnxruntime 1.32.0, numpy
  2.4.6, opencv-python-headless 5.1.0.95, cyclonedds 0.10.2, unitree_sdk2py 1.0.1; no GPU (EGL on Mesa llvmpipe). Pins in
  `scripts/fetch_assets.sh` and in every record.
* `python -m pytest -q backends/unitree_g1/tests`, `python -m robolang_g1.harness`, `python -m robolang_g1.phase2`,
  `python -m robolang_g1.phase3`, `python -m ruff check .`. **One stand-in at a time**: a unit test (`test_guard`) failed
  once while an experiment's stand-in ran, which is the phase 2 warning again.

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
  0.01 rad is `true`, outside by that much is `false`, between is `unknown` (the measured offset of a station marker against
  the oracle is 0.4-0.9 cm and 0.001-0.005 rad, phase 2). The brief's 5 cm / 0.1 rad stand; the margins are a
  tightening, never a loosening.
* **`holding`**: a hand is "closed" within 0.25 rad of the Dex3 closing pose (the tolerance `grasp` uses) and "open" within
  0.25 rad of all-zero; anything between is `unknown`. "At the hand" is the phase 2 radius, 9 cm from the grasp point (5.4-6.2
  cm when held right).
* **An object seen at a closed hand is at no place**, so `object_at` is `false` for it even when the station marker is not
  in view: a held object is not at a location (RoboLang's own meaning), and it is what makes the goal without a `place` step
  S301 rather than S302 once the object can be seen.
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
  carry the belief was 0.2-0.4 m off sideways, the shelf marker (low, behind the table) was hidden by the table from there, and the scan
  turned the robot away from it. The same move without a carried object, three times, succeeded in 47-60 s with the belief within
  1-2 cm of the oracle ([`exp_shelf_move.py`](g1-phase3/exp_shelf_move.py)). Two changes: the approach point is shifted
  sideways (0.3 m to the left for the bench and the shelf) where a declared table would hide the marker from a belief that is
  0.4 m off (`G1Operations.approach_lateral`), and the belief now carries the shuffle's backward drift (0.035 m a burst,
  measured in phase 2 and ignored by the dead reckoning). With them O3 passed 4 of 4 runs.

## Deviations

DEVIATIONS_PLACEHOLDER

## Open issues

OPEN_PLACEHOLDER
