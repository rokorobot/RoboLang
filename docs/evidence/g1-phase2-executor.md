# G1 phase 2: the executor (checks E1-E7)

Date: 2026-11-21. Code: [`backends/unitree_g1/`](../../backends/unitree_g1/) (package `robolang-g1`), decision
[0046](../../spec/decisions/0046-humanoid-backend-unitree-g1.md) section 4, item 2; contract in
[`g1-backend-v0.1.md`](../../spec/g1-backend-v0.1.md). Simulation only.

**Result: E1-E7 all pass, in two consecutive full runs of `python -m robolang_g1.phase2` (19 and 20 minutes, exit 0), and
the phase 1 harness still exits 0 (P1-P6 pass).** `realtime_valid` was true in every one of the 30 stand-in runs of the two
runs. The runs are [`phase2-run1.json`](g1-phase2/phase2-run1.json) and [`phase2-run2.json`](g1-phase2/phase2-run2.json);
the harness is [`harness-after-phase2.json`](g1-phase2/harness-after-phase2.json); [`records/`](g1-phase2/records/) has the
run records of the interesting scenarios (both E4 tasks of both runs; a record per E1 covered marker, E2, E5 and E7
scenario of the first run).

| check | run 1 | run 2 |
|---|---|---|
| E1 `move`, oracle error at each of 3 stations (limit 5 cm / 0.1 rad) | tour 1: bench 1.4 cm / 0.010 rad, shelf 1.3 cm / 0.044 rad, pad 1.2 cm / 0.019 rad; tour 2: bench 0.8 cm / 0.002 rad, shelf 1.6 cm / 0.054 rad, pad 2.6 cm / 0.029 rad | tour 1: bench 0.7 cm / 0.006 rad, shelf 1.4 cm / 0.016 rad, pad 3.8 cm / 0.057 rad; tour 2: bench 1.0 cm / 0.016 rad, shelf 1.2 cm / 0.042 rad, pad 1.9 cm / 0.026 rad |
| E1 covered marker | covered marker: S517, stop confirmed after 512 ms; oracle: pelvis path 0.1 cm, tilt 0.9 deg | covered marker: S517, stop confirmed after 512 ms; oracle: pelvis path 0.1 cm, tilt 1.1 deg |
| E2 `grasp` (lift at least 5 cm) | grasp 1: lift 8.9 cm, marker 5.8 cm from the hand, moved 1.9 cm; grasp 2: lift 8.8 cm, marker 5.4 cm from the hand, moved 2.1 cm; absent: S506 with 0 arm/hand/attach commands; out of reach: S502 with 0 arm commands | grasp 1: lift 8.9 cm, marker 5.9 cm from the hand, moved 1.8 cm; grasp 2: lift 8.8 cm, marker 5.6 cm from the hand, moved 2.1 cm; absent: S506 with 0 arm/hand/attach commands; out of reach: S502 with 0 arm commands |
| E3 `place` | box at [2.745, 2.304, 0.79] (region centre 2.75, 2.30, 0.79), in region True, released True, wrist 4.6 cm from its carry position | box at [2.741, 2.301, 0.79] (region centre 2.75, 2.30, 0.79), in region True, released True, wrist 4.6 cm from its carry position |
| E4 both tasks, unchanged, through the CLI | e_pick_place: move 24 s, grasp 21 s, move 64 s, place 18 s; box in the shelf region; record `STEPS_COMPLETED`, exit 4. w_carry_hold: move 24 s, grasp 21 s, move 51 s; crate on the right hand; robot at dst 2.4 cm / 0.018 rad; record `STEPS_COMPLETED`, exit 4 | e_pick_place: move 24 s, grasp 21 s, move 145 s, place 18 s; box in the shelf region; record `STEPS_COMPLETED`, exit 4. w_carry_hold: move 24 s, grasp 21 s, move 50 s; crate on the right hand; robot at dst 1.9 cm / 0.025 rad; record `STEPS_COMPLETED`, exit 4 |
| E5 stop | cancel: S511, confirmed after 1364 ms, oracle pelvis path 1.0 cm; revoke: S513, confirmed after 1440 ms, oracle pelvis path 0.0 cm; deadline: S512, confirmed after 616 ms, oracle pelvis path 0.0 cm, box still attached; push: S516, oracle fell at 10.02 s, last command at 7.47 s, none after the fall | cancel: S511, confirmed after 2181 ms, oracle pelvis path 0.3 cm; revoke: S513, confirmed after 1452 ms, oracle pelvis path 0.0 cm; deadline: S512, confirmed after 612 ms, oracle pelvis path 0.0 cm, box still attached; push: S516, oracle fell at 10.01 s, last command at 7.47 s, none after the fall |
| E6 refusals (each before any command) | S508 (0 commands), S509 (0 commands), S510 (0 commands), S514 (0 commands), S515 (0 commands) | S508 (0 commands), S509 (0 commands), S510 (0 commands), S514 (0 commands), S515 (0 commands) |
| E7 free stance during arm work (limit 3 cm) | worst 2.1 cm over 5 arm steps (supported); free-stance trial: STEPS_COMPLETED, robot moved 17.6 cm (path 56.7 cm) | worst 2.1 cm over 5 arm steps (supported); free-stance trial: STEPS_COMPLETED, robot moved 12.1 cm (path 46.2 cm) |
| real time | 15/15 stand-in runs realtime_valid; worst second 0.9964; step_compute_ms mean 0.467, p99 1.31 (worst run), max 87.6 | 15/15 stand-in runs realtime_valid; worst second 0.9554; step_compute_ms mean 0.454, p99 1.31 (worst run), max 74.9 |

Codes seen in the checks: S502, S506 (E2), S511, S512, S513, S516 (E5), S517 (E1), S508, S509, S510, S514, S515 (E6). In
development, before the fixes below: S503 (a wrist pushed aside by a hand on a table), S504 (the servo not converging) and
S517 (a marker hidden behind a table), each of which is in the history of this branch.

The checks did not pass on the first try, and none of the failures was a loosened check: the first full run failed E1
(`S517`: the approach ended where a table hid the marker) and E2 (the out-of-reach box was not seen at 0.7 m); the next
E3/E4 (`S506`: the marker-at-hand radius was 1 cm too tight); the next E4 (`S504`: a leg against the table's corner,
servoing the wrong axis); another E1 (the same hidden marker, again). Each is fixed in `ops.py` or the scenario (the
replanning approach, the servo to an approach point and its blocked-axis rule, the radius, the box's distance); the two
runs above are with the code as it is.

## What ran

* This sandbox: aarch64 Debian 12, 4 vCPU shared with other jobs (load average 17-25 throughout), Python 3.11.2, MuJoCo
  4.0.1, onnxruntime 1.32.0, numpy 2.4.6, opencv-python-headless 5.1.0.95, cyclonedds 0.10.2, unitree_sdk2py 1.0.1; no GPU
  (EGL on Mesa llvmpipe). Pins are in `scripts/fetch_assets.sh` and in every record.
* `bash backends/unitree_g1/scripts/fetch_assets.sh`, `pip install -e . -e "backends/unitree_g1[test]"`,
  `python -m pytest -q backends/unitree_g1/tests` (90 passed with `MUJOCO_GL=egl`: 36 of phase 1 and 54 new), `MUJOCO_GL=egl python -m robolang_g1.harness --out h1.json`
  (phase 1, still exit 0), `MUJOCO_GL=egl python -m robolang_g1.phase2 --out phase2.json` (twice), `python -m ruff check .`.
* Each check scenario starts its own stand-in (oracle on, test hooks on) and runs `python -m robolang_g1.cli` against it as
  a separate process. One stand-in at a time: **two stand-ins on one DDS domain mix their `lowstate`, camera frames and
  commands**. Several of my own early measurements in this phase were spoiled by a stray one before I found it; every
  number below was measured with none running (`ps` checked before each run).

## The G1 does not walk the way the brief assumes, and `move` is built around that

The brief asks for "turn, walk, turn". The whole-body policy behind the stand-in (Unitree's `velocity/v0`, as ported in
phase 1) does not turn in place. Measured with [`exp_grid.py`](g1-phase2/exp_grid.py) (3 s commands from rest, free stance,
oracle):

| command (vx, vy, wz) | norm | moved forward / left (m) | yaw (rad) |
|---|---|---|---|
| 0, 0, 0.2 | 0.20 | 0 / 0 | +0.08 |
| 0, 0.15, 0.2 | 0.25 | 0 / 0 | +0.08 |
| 0.1, 0, 0.2 | 0.22 | 0 / 0 | +0.07 |
| 0.2, 0, 0 | 0.20 | 0 / 0 | 0 |
| 0.2, 0, 0.2 | 0.28 | 0.29 / 0.04 | +0.21 |
| 0, 0.3, 0 | 0.30 | -0.06 / 0.58 | 0 |
| 0, 0.3, 0.2 | 0.36 | -0.15 / 0.56 | +0.41 |
| 0.3, 0, 0 | 0.30 | 0.72 / 0 | +0.06 |
| 0.3, 0, 0.2 | 0.36 | 0.76 / 0.15 | +0.42 |
| 0.3, 0.3, 0.2 | 0.47 | 0.62 / 0.85 | +0.49 |

* **A command of norm below about 0.3 is ignored** (the policy stands). A pure yaw of 0.2 rad/s, the largest it takes,
  turns the robot by 0.2 rad in 8 s.
* Yaw comes only with translation: 0.14 rad/s at best, on a circle of radius 1.4-1.9 m. The policy was also asked for yaw
  beyond its trained range (1.0 rad/s) in an experiment: asymmetric (left turned, right did not) and not used.
* **Turning in place works by shuffling**: alternating bursts of `(0, +0.3, 0.2)` and `(0, -0.3, 0.2)`, 1.2 s each. Six
  bursts turned +0.76 rad (left) and, with wz -0.2, -0.665 rad (right), with the position moving only 0.12-0.14 m back
  ([`exp_shuffle.py`](g1-phase2/exp_shuffle.py)); in the runs each burst turns 0.10-0.13 rad and drifts 0.03-0.04 m back.
  A 90-degree turn is about 13 bursts (16 s).
* **Sideways walking is good**: `(0, 0.3, 0)` moves 0.19 m/s with almost no yaw. So `move` does not turn to face where it
  goes: each leg commands the direction to the target in the robot's own frame (forward, diagonal, sideways, backward) with a
  yaw rate that works toward the station's heading as it goes. Shuffles are for turns that must happen in place.
* **The policy steps, so displacements come in steps** ([`exp_fine.py`](g1-phase2/exp_fine.py), four repeats from rest):

  | burst | moved (m) |
  |---|---|
  | forward 0.4 for 0.1 s, 0.2 s | 0 (no step) |
  | forward 0.4 for 0.3 s | 0.06, 0.10, 0.11, 0.11 |
  | forward 0.4 for 0.4 s | 0.15 (all four) |
  | backward -0.4 for 0.2 s / 0.3 s / 0.4 s | 0.07 (0.05-0.10) / 0.10 / 0.147 |
  | left `(0.1, 0.3)` for 0.3 s / 0.5 s | 0.001 / 0.002 (no step to the left this short) |
  | right `(0.1, -0.3)` for 0.5 s | 0.036, 0.026, 0.056, 0.063 |

  The servo (`ops._burst`) picks the burst whose step best cancels the error, from this table. Before it did, a rule that
  scaled a burst's length with the error oscillated: forward errors of +-0.1 m, 1 run in 3 not converging in 90 s. With the table
  it converges in 3-6 looks (27 s) with 0.4-1.6 cm of error.

## Localization and perception

* **Station markers** (ArUco 4x4_50, 0.24 m, vertical panels, centre 0.19 m up) stand 1.3 m ahead and 0.45 m to the left of
  the standing pose, behind the table. Why there: the head camera is pitched 47.6 degrees down with a 58-degree vertical
  field of view, so it sees elevations from 18.6 to 76.6 degrees below the horizon. A marker at table height is out of
  view from the approach; a marker behind a table is hidden by it from the side (a first layout failed this way). A marker
  must be low, a little to one side of the axis, and the robot must come in along the axis (`STAGING_M`).
* **Pose from a marker**: the full marker pose from `solvePnP` (IPPE), through the camera pose from the robot's own
  kinematic model fed with the joints measured at the frame's time. At the standing pose, 1.7 m: 0.4-0.9 cm and 0.001-0.005
  rad against the oracle (`tests/test_perception.py`). Beyond 2.3 m a 0.24 m marker's pose has the planar-target ambiguity:
  the wrong solution has a pelvis 2-7 degrees off upright, so a solution whose vertical disagrees with the IMU's by more than
  0.03 rad is ignored (`MAX_UP_DISAGREEMENT_RAD`). Tilting the panel back to face the camera was tried and is worse: a
  frontal marker is the worst case for that ambiguity.
* **Belief pose**: dead reckoning from commanded velocities with measured gains (`ops.WALK_GAIN` and the others), replaced
  by any station marker seen within 2.9 m. In the tour runs the belief was within 0.3-1.5 cm of the oracle whenever a marker
  was in view, and up to 0.9 m off after 3 m without one (the cause of the replanning approach).
* Object markers are on the top face (5.7 and 6.4 cm: the `box` is an 8 cm cube, the `crate` 9x9x8 cm). On the front face the closed hand
  covered it; on the top, with the pinch 1 cm below the object's centre, the fingers stay below the top face.

## Arms

* The G1's arm reaches 0.39 m from the shoulder at table height with the fingers 15-45 degrees down: objects at 0.28 m
  from the pelvis, 0.22 m from the table edge. Inverse kinematics (damped least squares on MuJoCo's Jacobians): its first
  version clipped a joint at its limit after computing the step, which wasted the whole step and made every target with a
  waist lean unsolvable; with an active set (a joint at a limit that the step would push through is dropped) the arm and the
  waist's pitch (up to 0.3 rad, at a cost) reach 0.42 m. Position tolerance 1 cm, orientation 0.35 rad.
* Lifting the hand straight from the walking controller's arm pose **catches the thumb on the hip** and kicks the wrist
  (5 N m): the first segment swings the arm out of the way.
* In the scene **hands touch neither objects nor tables** (collision bits in `sim/scene.py`): grasping is a weld, and a
  finger on a tabletop made the stiff arm push the weak wrist (5 N m) aside. The executor plans no collision-free paths
  yet; this is a simulation expedient, stated in the spec.
* **Stance during arm work.** The free stance moved the robot when the arm reached: in the E7 free-stance trial (the same grasp, with the
executor's site saying `arm_stance: free`) the pelvis ended 17.6 cm from where it was when the arm work began in run 1
(its path was 57 cm) and 12.1 cm in run 2 (path 46 cm); the grasp still
completed both times, with the reach planned from what the camera saw before the arm moved. In development the same trial once
failed (`S506`: the marker 7-8 cm from the hand after the robot had moved 6 cm). That is the 12 cm of phase 1 and 0046 U4,
measured again, and far above the 3 cm of E7. So both sites say `arm_stance: "supported"`: the stand-in welds the pelvis
for the arm work and frees it afterwards (`SetStance`; the stance model of each step is in its record). With it the
robot moved at most 2.1 cm (oracle) over the 5 grasp and place steps of each run (run 1: [1.9, 2.1, 1.8, 2.1, 2.1] cm; run 2: [1.8, 2.1, 2.1, 1.9, 1.6] cm), 1-2 cm of it
the weld settling and the arm's own mass. **This is the approved fallback, not evidence that the arm work balances**
(open issue).

## Stop

* Confirmation from `lowstate`: over 0.5 s the pelvis IMU's rate at most 0.045 rad/s, every joint's velocity at most 0.09
  rad/s, tilt at most 15 degrees. After a walk at 0.5 m/s the robot comes to rest in 1.3-2 s ([`exp_settle.py`](g1-phase2/exp_settle.py):
  at rest the IMU rate is 0.006 rad/s and the leg joints 0.004 rad/s; a coasting second after a walk is 0.09 and 0.4).
* S516: a 500 N push on the torso for 0.4 s while the robot walks makes it fall in every try (500, 800 and 1200 N: [`exp_push.py`](g1-phase2/exp_push.py)).
  The executor sees the IMU tilt reach 40 degrees before the pelvis is down, silences the SDK client (the `arm_sdk` thread
  stops, no `SetVelocity`) and records `kind: no_further_command`.

## Results

The two runs' numbers are in the table at the top; `phase2.json` has every detail, and each record has the executor's own
observations (marker readings with confidence, age and range; the belief's corrections; the stop and its confirmation).

## The garbage collector and `step_compute_ms.max`

On x86_64 CI the phase 1 run once showed a single 248 ms pause. Python's collector is **not** what the measurements here
support. The stand-in's heap after start-up is about 70 000 tracked objects, and a full collection of it takes 7-10 ms
here (`gc_check.py`, below), not hundreds. In 60 s of the real workload (walking, `arm_sdk` at 50 Hz, a hand, the camera
at 5 Hz) no collection took more than 1 ms in the loop, and no second-generation collection ran at all:

| 60 s run | before (collector as it was) | after (`gc.freeze()` after start-up, generation 2 effectively off) |
|---|---|---|
| `step_compute_ms` mean / p99 / max | 0.478 / 1.261 / 3.4 and 0.461 / 1.245 / 6.5 | 0.453 / 1.213 / 2.3 and 0.461 / 1.205 / 2.9 |
| collections by generation (0, 1, 2) | 8, 1, 0 (both runs) | 8, 0, 1 (the one at start-up, 9.6 and 10.4 ms, both runs) |
| longest collection | 0.89, 0.93 ms | 0.09 ms in the loop |

(`gc_before_*.json`, `gc_after_*.json`, `gc_check.py`; the maximum before also varied from 3.4 to 6.5 ms between two identical runs, so it
is not a measurement of the collector.) `gc.freeze()` is applied anyway because it costs nothing and removes the 7-10 ms
full collection; it is **not** shown to be the cause of the 248 ms. The stand-in's `status` now reports, for every run, the
collections by generation, the longest one, and every loop pass over 20 ms with the number of collections that ran inside
it (`gc.slow_steps`): if CI shows a long step again, that field says whether a collection was in it.

What the long steps are, here: in every phase 2 scenario the stand-in logs one to three loop passes of 20-87 ms at a
simulation time of 3.3-3.7 s, with **no collection inside any of them** (the third number of each `gc.slow_steps` entry is 0),
while its `step_compute_ms` is 5-15 ms at most between them. That is when the check starts the executor as a new process
(`python -m robolang_g1.cli` imports numpy, MuJoCo, OpenCV and the SDK) on a machine whose four cores are shared (load
average 17-25): the stand-in's thread is not scheduled for a few tens of milliseconds. A DDS client joining on its own does
not do it: [`join_stall.py`](g1-phase2/join_stall.py) starts a stand-in, joins with a participant and the executor's clients
at 1.5 s and at 9 s, with and without a reader of the oracle file, and gets no step over 8 ms. So the likelier reading of the
x86_64 CI's 248 ms is the runner scheduling one of the harness's own new processes (phase 1's P1 starts one) against the
stand-in, not the collector. In the 30 stand-in runs of the two passing full runs, none stalled and every second stayed
within 95% of real time (worst second 0.9554); an earlier development run had one second at 0.9462 (the E6 stand-in), which
`realtime_valid` reported as false.

## Deviations from the brief and from 0046

1. **`move` does not "turn, walk, turn"** (0046 U5, the brief): the policy ignores commands below about 0.3 and cannot turn
   in place except by shuffling. `move` walks in the direction of the target in the robot's own frame with a yaw rate
   toward the station's heading, comes in along the station's axis (a staging point 2 m before the station, then the
   approach point 0.5 m before it), and turns by shuffling only to correct a heading by more than 0.11 rad. See above.
2. **Arm work runs in the `supported` stance, if the site says so, and both sites do.** The brief allows "switch that step
   to `supported` if the site says so". The site key is `robot.arm_stance`; the stand-in gained a `SetStance` call on
   `robolang_sim` (the executor calls it around a grasp or place and restores `free` afterwards); the record's step details
   say `arm_work_stance`. 0046 U4 and open point 1 approve `supported` as a fallback recorded in every run; here it is
   recorded per step. The free stance moved the robot 18 and 12 cm in the two runs' trials (E7), against 3 cm allowed.
3. **Hands touch neither objects nor tables in the scene** (collision bits), because grasping is a weld (U7) and a finger on
   a tabletop made the stiff arm push the weak wrist aside. The executor plans no collision-free arm path (open issue).
4. **Objects are 8 cm and 9 cm cubes with the marker on the top face** (phase 1's box was 12 cm with the marker on a side
   face): the marker must be seen at the hand, which the closed hand prevents on a side face, and an object has to be
   seen from 0.7 m for the out-of-reach case. Station markers are 0.24 m panels at floor level (see above), not the 0.15 m
   post of phase 1.
5. **Every precondition is refused with S508** in phase 2 (their atoms cannot be observed until phase 3); the brief says
   "preconditions with atoms you cannot observe yet", and none can be. Goal atoms must be observable ones (S507/S508 else).
6. **The run's `state` is `STEPS_COMPLETED`, and the CLI exits 4**, not 0, when every step ran: "never reports SUCCEEDED
   from completed steps alone". A mission (S514) is refused before the guard, as in the ROS 2 CLI, but a record is written.
7. **`move` promises 5 cm and 0.1 rad and servos to 4.4 cm and 0.09 rad**, then confirms within 4.7 cm and 0.09 rad after
   settling, so that the camera's 0.5-1 cm leaves the promise intact.
8. **Interface additions to `robolang_sim` and the oracle**: the identity API says where the head camera's frames are
   (`head_camera_dir`; on the robot the camera is read directly); `status` carries the command counts per channel, the
   garbage collector's numbers and `step_compute_ms`; the oracle carries the commands, hand joints and wrist positions;
   `SetStance` (above); a push on the torso, answered only with `--test-hooks` and never called by the executor.
9. **The 248 ms pause is not shown to be the garbage collector** (above), so `gc.freeze()` is a precaution, not the fix the
   brief describes for that case.
10. **CI**: `g1-sim.yml` runs the phase 2 checks after the harness and its limit is 100 minutes (the brief: "sensible"); the
    phase 2 checks take about 20 minutes here.
11. 0046 U6's carry pose is the walking controller's own arm pose, taken from the measured joints at the start, not read
    from the policy's `deploy.yaml`. `w_carry_hold` stays in the family: the crate was carried 2 m with the arm held there.

## Open issues

* **x86_64 has not run phase 2.** The pull request's `g1-sim.yml` will. The phase 1 x86 runs saw a slow runner twice and
  one 248 ms step; phase 2 is about 35 minutes of simulated real time, so it is exposed to that. `realtime_valid` is in
  every record and in `phase2.json`, and CI's summary prints it per scenario. If it says false, the run is not valid for
  acceptance (U13), not a backend failure.
* **Moves are slow and depend on markers being in view.** 30-120 s per move here. Without a marker for about 3 m the belief
  drifts by up to 0.5 m, and a station whose marker is hidden from the approach direction by a table can fail with S517
  (a first layout did). A walk from the pad station back to the bench once ended S517 in development (backward and
  sideways walking with a 90-degree yaw change); it is not in E1, which is the brief's tour. The sites keep markers low,
  beside the axis and in view along it; the visibility check (`perception.visibility_violations`) is a cone test with no
  occlusion.
* **The stance is `supported` for arm work**, so these runs do not show that the arm work balances on the free stance
  (U4's caveat). A recovery step of the free stance while an object is welded to a hand is still untested. The free trial
  (E7) shows the size of the problem.
* **No collision-free planning**, and hands pass through tables and objects in the scene. A forearm or elbow can still touch
  a table; a wrist that cannot track its target for 1.5 s is S503.
* **Only the right hand is exercised** (both sites choose it); the left hand's mirrored closing pose and grasp frame are
  untested.
* Goal and precondition verdicts, `holding`, `object_at`, `robot_at`, `robot_ready`, are phase 3; the pieces it needs
  (marker readings with confidence and age, the hand's pose from measured joints, the station-frame region test) exist.
* The camera's field of view is still not checked against a D435 (phase 1), and the marker detection range (2.9 m for
  0.24 m, 0.9 m for 6 cm) depends on it.
* Phase 4 will want a faster `move`: legs without a settle between them, and a marker visible from further out.
