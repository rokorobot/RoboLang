# G1 phase 2: the executor (checks E1-E7)

Date: 2026-11-21. Code: [`backends/unitree_g1/`](../../backends/unitree_g1/) (package `robolang-g1`), decision
[0046](../../spec/decisions/0046-humanoid-backend-unitree-g1.md) section 4, item 2; contract in
[`g1-backend-v0.1.md`](../../spec/g1-backend-v0.1.md). Simulation only.

@@RESULT@@

## What ran

* This sandbox: aarch64 Debian 12, 4 vCPU shared with other jobs (load average 17-25 throughout), Python 3.11.2, MuJoCo
  4.0.1, onnxruntime 1.32.0, numpy 2.4.6, opencv-python-headless 5.1.0.95, cyclonedds 0.10.2, unitree_sdk2py 1.0.1; no GPU
  (EGL on Mesa llvmpipe). Pins are in `scripts/fetch_assets.sh` and in every record.
* `bash backends/unitree_g1/scripts/fetch_assets.sh`, `pip install -e . -e "backends/unitree_g1[test]"`,
  `python -m pytest -q backends/unitree_g1/tests` (@@TESTS@@), `MUJOCO_GL=egl python -m robolang_g1.harness --out h1.json`
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
  was in view, and up to 0.5 m off after 3 m without one.
* Object markers are on the top face (4-6 cm: `box` 8 cm cube, `crate` 9x9x8 cm). On the front face the closed hand
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
* **Stance during arm work.** @@STANCE@@

## Stop

* Confirmation from `lowstate`: over 0.5 s the pelvis IMU's rate at most 0.045 rad/s, every joint's velocity at most 0.09
  rad/s, tilt at most 15 degrees. After a walk at 0.5 m/s the robot comes to rest in 1.3-2 s ([`exp_settle.py`](g1-phase2/exp_settle.py):
  at rest the IMU rate is 0.006 rad/s and the leg joints 0.004 rad/s; a coasting second after a walk is 0.09 and 0.4).
* S516: a 500 N push on the torso for 0.4 s while the robot walks makes it fall in every try (500, 800 and 1200 N: [`exp_push.py`](g1-phase2/exp_push.py)).
  The executor sees the IMU tilt reach 40 degrees before the pelvis is down, silences the SDK client (the `arm_sdk` thread
  stops, no `SetVelocity`) and records `kind: no_further_command`.

## Results

@@E@@

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

(`gc_before_*.json`, `gc_after_*.json`; the maximum before also varied from 3.4 to 6.5 ms between two identical runs, so it
is not a measurement of the collector.) `gc.freeze()` is applied anyway because it costs nothing and removes the 7-10 ms
full collection; it is **not** shown to be the cause of the 248 ms. The stand-in's `status` now reports, for every run, the
collections by generation, the longest one, and every loop pass over 20 ms with the number of collections that ran inside
it (`gc.slow_steps`): if CI shows a long step again, that field says whether a collection was in it. The likelier
cause on a shared runner is the machine: the same stand-in had 84 ms and 51 ms single steps in two of my runs while other
jobs were loading this sandbox (load average 17-25), and none when it was quiet.

## Deviations from the brief and from 0046

@@DEVIATIONS@@

## Open issues

@@OPEN@@
