# RoboLang Unitree G1 backend 0.1 (MuJoCo, simulation only)

**Status:** phases 1–3 of decision [0046](decisions/0046-humanoid-backend-unitree-g1.md): the stand-in, the guard and
the harness checks P1–P6 ([evidence](../docs/evidence/g1-phase1-harness.md)), the executor (`move`, `grasp`,
`place`, `wait for`, the stop, the records; checks E1–E7, [evidence](../docs/evidence/g1-phase2-executor.md)), and goal
and precondition verdicts from observations (checks O1–O7, [evidence](../docs/evidence/g1-phase3-observations.md)), with
one exception to 0046 U8 for observing a carried object (approved by the owner 2026-11-21; see "Observations and
verdicts"). Two prerequisites of the phase 4 acceptance run followed ([evidence](../docs/evidence/g1-phase4-prereqs.md)):
`move`'s heading between marker fixes comes from the IMU ("The IMU"), and the stand-in keeps pace with real time on a
loaded 4-CPU host ("Time"). **Phase 4, the acceptance run, has run once and did not meet the bar** (2026-11-26: 37 of 42
runs `SUCCEEDED`, 38 needed; no false success, 98.2% agreement with the oracle, 5 of 5 failure cases;
[evidence](../docs/evidence/g1-phase4-acceptance.md)). Code: [`backends/unitree_g1/`](../backends/unitree_g1/).

## Scope and safety boundary

The optional `robolang-g1` package runs a validated task on a simulated Unitree G1 (29 DoF, Dex3-1 hands) in MuJoCo.
The core `robolang` wheel does not import it. It talks to the robot **only through the interfaces the real G1 offers**:
Unitree's SDK over CycloneDDS (`LocoClient` on the RPC service `sport`, `rt/arm_sdk`, `rt/dex3/*`, `rt/lowstate`) and
the head camera, which on the robot is read directly, not through the SDK. In simulation a **stand-in** process answers
on those interfaces; it is described in full in the docstring of
[`robolang_g1/sim/standin.py`](../backends/unitree_g1/robolang_g1/sim/standin.py). The stand-in's walking controller is
Unitree's pretrained whole-body policy, not the robot's own controller, so nothing here says how the real G1 walks.

**Guard (S501).** Before any command the backend requires, in order: a DDS domain other than 0 and the loopback
interface, checked before any DDS participant exists; an answer from the stand-in's simulation-identity API (service
`robolang_sim`, which no real robot has); `simulation: true` on the same domain; and a simulation clock that advances.
There is no flag, variable or argument that skips a step ([`guard.py`](../backends/unitree_g1/robolang_g1/guard.py)).

**Never evidence.** MuJoCo state and `SportModeState` are never read by the backend. With `--oracle` only, the
stand-in writes ground truth for the test harness; nothing else reads it.

## Time

Physics steps every 2 ms against the wall clock, the joint PD law is evaluated every step (500 Hz) and the policy
every 10th step (50 Hz). A 5 ms step is not allowed: MuJoCo integrates the PD torque explicitly, which is stable only
for dt·kd/I < 2, and the arms reach 4.5 at 5 ms (1.8 at 2 ms). Simulation time is in `LowState_.tick` (ms), in the
identity API and in every camera frame. The SDK's timeouts are wall-clock, as on the robot. The stand-in reports its
real-time ratio. A run in which any second falls more than 5% behind real time, or in which a step stalls for more than
0.25 s, does not count towards acceptance; **that bar is unchanged**.

**Keeping pace on a loaded host.** The loop is 0.3-0.5 ms of compute in a 2 ms step, so a second is lost only when the
loop thread is kept off a CPU: measured, a second at 0.80 of real time had the loop waiting 0.4-0.6 s in the run queue
while the executor started up and the camera rendered. Four things follow, none of which touches the 2 ms step or the PD
law: (1) `rt/lowstate` and the hand states are serialized once and their changing fields patched in a byte buffer
(`sim/fastdds.py`; 14 us a publish instead of 247 us, **the same bytes on the wire**, a test compares them); (2) the
camera thread and the renderer's workers run at nice 19, and the executor process and the checks' oracle reader at nice
10 (`harness.YIELD_NICE`; lowering a priority needs no privilege), so the loop thread, which keeps nice 0, wins every
contention among the backend's own processes; (3) the stand-in's `status` says why a slow second was slow: each entry of
`slow_windows` is `(sim time, ratio, loop thread CPU s, loop thread run-queue wait s)`; (4) the executor, when started by
hand rather than by `run_task` or the checks, is not niced: start it with `nice -n 10`.

**The head camera is a separate matter.** Software rendering is the stand-in's largest use of CPU (a 640x480 frame is 114 ms
on four CPUs, 207-287 ms on two), and a host short of it keeps the loop in time and lets the camera fall from 5 Hz to
2.5-3 Hz. `realtime_valid` does not look at the camera; the run records and the CI summary carry its rate (`camera`:
`frames`, `dropped`, `render_ms`, `hz`), and the harness's P6 fails below 4.5 Hz.

## The IMU

`rt/lowstate` carries the pelvis IMU: quaternion, gyroscope, accelerometer. MuJoCo's own sensors are exact, and a heading
taken from a perfect IMU would flatter the executor, so the stand-in spoils them as a MEMS IMU with an on-board attitude
filter is spoiled ([`sim/imu.py`](../backends/unitree_g1/robolang_g1/sim/imu.py)):

| | model | value |
|---|---|---|
| gyro bias | per axis, normal, drawn at start-up (what is left after turn-on calibration) | 1 sigma 0.002 rad/s (0.11 deg/s) |
| gyro bias walk | random walk in the run | 2e-5 rad/s per root second |
| gyro noise | white, per sample at 250 Hz | 0.002 rad/s rms (1.8e-4 rad/s per root hertz) |
| quaternion yaw | the integral of the gyro's error about the vertical (no magnetometer: nothing pulls it back) | drifts at the bias: 0.12 rad a minute at 1 sigma |
| quaternion roll, pitch | held by gravity: exact | |
| accelerometer | white noise | 0.02 m/s^2 rms |

`--imu-seed N` repeats a draw, `--imu-bias B` sets the gyro's z bias (rad/s; the stress check's worst case),
`--ideal-imu` is the exact sensor (measurement only). The oracle has the true bias and yaw error; the executor cannot see them.

**What the executor does with it** ([`heading.py`](../backends/unitree_g1/robolang_g1/heading.py), pure; the no-oracle
test covers it). The belief heading is `the last marker fix's heading + the change in the IMU's yaw since that fix - bias x
time`. The yaw is taken from the quaternion; nothing else (not the oracle, MuJoCo state or `SportModeState`) is read. A
marker fix resets the belief, as before. **Bias:** the IMU's yaw drifts, so the tracker learns the gyro bias from the
fixes themselves: over the time between two marker fixes the marker says how far the robot turned and the IMU says how far
it thinks it did, and the difference per second is the bias (least squares over all pairs at least 4 s long, with a prior
at zero, forgetting with a 300 s time constant, clipped at 0.01 rad/s; a rate above 0.02 rad/s between two fixes is taken
for a bad fix and teaches nothing). While the pelvis is welded for arm work (the supported stance) the belief heading
holds, since a welded pelvis cannot turn, and a pair of fixes across that interval is a pure bias measurement. The
position between fixes is still dead-reckoned from the commands (measured gains), along the IMU's heading. Each `move`
record has `heading: {source: "imu_yaw", fixes, bias_rad_s, jump_rad_max, jump_rad_mean}`, and each `localized` event
`heading_jump_rad`: how far the fix moved the heading, which is the IMU's error since the last fix, as the markers saw it.

## Stance and stop

`stance_model` is `free` by default: the policy alone, which comes to rest after a walk (about 0.13 m and 2 s of coast
at 0.5 m/s, then about 1 cm of torso sway). `free` takes a recovery step of 0.2–0.35 m about every 45 s while standing,
so arm work longer than about 40 s uses `supported`: the torso welded to a mocap body (the fallback the owner approved,
0046 open point 1). Every record states the stance model. The free stance also **drifts in heading by about 0.01 rad a second** after a move
(measured), so `grasp`, `place` and the observation motion weld the pelvis (`arm_stance: supported`) **before** they measure the
object and plan the reach, not after: a plan made on a loaded host took a minute and the robot had turned 0.33 rad, the hand closed on
air (S506). The pelvis stays welded from then to the end of the step; after an observation motion it stays welded to the end of the run
(nothing walks after the goal snapshot, and releasing it let the free stance turn the robot 0.017 rad in the next second, measured), and a
`move` frees it first.

The stand-in never collapses the robot: `SetFsmId` accepts only 500 (standing or walking) and refuses every other id
with code 7400, `Damp` included. The backend's stop is zero velocity, the arms held through `arm_sdk`, the
hands unchanged, the robot kept standing. It never calls `Damp()`, zero torque or `SwitchToUserCtrl`, and never writes
`rt/lowcmd` (safety case H25).

## Site file

`robolang-g1-site/2` (`/1` still loads), described in [`site.py`](../backends/unitree_g1/robolang_g1/site.py): the floor,
tables and objects of the scene; the head camera's calibration; the robot's start pose in the map; **stations** (a marker
with its id, printed size, pose in the map and `facing`, and where the pelvis stands relative to it: `stand = [d, l, a]`);
**locations** (a station, and for `object_at` a `region`, a box in the station marker's frame: x right, y up, z out of
the plate); **objects** (a marker on a face, which hand); and **bindings** from the `.rob` entity IDs (`--bind P=ID`) to
the site's robot, objects and locations. The scene generator builds MuJoCo from the same file. The executor does not use
an object's position: it finds the object by its marker. A scene may differ from the site the executor is given (an
object removed, a marker `covered`), which is how the scripted failure cases are made. The sites
[`pick_place.json`](../backends/unitree_g1/robolang_g1/sites/pick_place.json),
[`carry_hold.json`](../backends/unitree_g1/robolang_g1/sites/carry_hold.json) and
[`part_precheck.json`](../backends/unitree_g1/robolang_g1/sites/part_precheck.json) are for
`bench/reference/e_pick_place.rob`, `w_carry_hold.rob` and `w_part_precheck.rob`; a test checks the scene rule of 0046
section 3 for all three (goal locations in view of the camera from the station where the plan ends, precondition locations
from the start pose, and no declared table between the camera and the marker: the line of sight is tested against the
tables). A site may also declare `obstacles` (boxes in the map like the tables, built into the scene): the executor keeps
the arm clear of every declared table and obstacle (below).

## Steps, observations and records

Code: `cli.py` (entry), `executor.py` (the run), `ops.py` (what is done to the robot), `perception.py`, `kinematics.py`,
`robot.py` (the SDK clients, shared with the harness). A task runs only through the robot's interfaces: `LocoClient`,
`rt/arm_sdk`, `rt/dex3/*`, `rt/lowstate`, the head camera, and (simulation only, after the guard) `robolang_sim`'s attach,
detach and stance. A test fails if an executor module refers to the oracle, `SportModeState`, or what a biped must never
be sent.

**Refused before any command** (as the ROS 2 backend, after the guard): S507 (a binding or entity the site does not
have, an object without a marker, an `object_at` location without a region), S509 `measure`, S510 a clock reading, S515
`message_received`, S514 a mission (before the guard), and S508 for everything else: other steps, failure handlers, speed
and region limits, `do`, a task without a goal, and any condition other than `object_at`, `robot_at`, `holding`,
`robot_ready` and their `and`, `or`, `not` (in a precondition or the goal).

**`move r to L`.** The executor keeps a **belief pose** of the pelvis in the map, replaced whenever a station marker is
seen (the marker's map pose composed with the inverse of its observed pose; a solution whose vertical disagrees with the
IMU's is the wrong one of the two a planar marker allows, and is ignored). Between fixes the **position** is dead
reckoning from the velocities it commands, taken along the heading the robot had while the command ran, and the
**heading** is the IMU's ("The IMU", below), not the commanded yaw rates'.

The walking stand-in turns in place only by a *shuffle* (alternating sideways commands, each with a yaw rate),
takes a step only when a command lasts about 0.3 s, and ignores commands smaller than about 0.3, so a move is: turn to face
the approach point (0.5 m before the station, facing it), walk in legs with a look between them, turn to the station's
heading, find the station marker (a scan by turning in place if it is not in view; **S517** if not seen within 70 s), then
correct with short bursts chosen from the measured step sizes, first to the approach point (within 8 cm and 0.07 rad) and
then until the marker puts the robot within **4.4 cm and 0.085 rad** of the station pose, wait for the stance to settle
(from `lowstate` only: pelvis IMU rate and leg joint velocities quiet), and look again (within 4.7 cm and 0.09 rad).
**S504** if it does not converge in 90 s. **The heading creeps**: a standing robot turns slowly to the left by itself
(0.007 rad/s measured, 0.025 under the stress check's torque). The corrections are chosen to leave the heading, counted as
it will be at the next look and at the confirming look, inside the band; the creep they are aimed against is read from the
IMU's yaw over a quiet second (at least 2 s after the last command), less the learned bias, and from the markers between
two looks. After four looks without a better error a servo goes on from a wider band (the final servo to the confirming
look from 4.2 cm and 0.07 rad, which still decides; the approach point, which no confirmation follows, from 12 cm and
0.15 rad, which the last walk corrects: under a creep of 0.025 rad/s the heading was inside 0.07 rad for a moment in each
5 s look, and one move spent its whole 150 s at the approach point).

**`grasp r O`.** Re-observe the station marker (and re-servo if the robot moved); observe the object's marker (**S506**
before anything moves if it is not seen); choose the hand (the site's, or the nearer side); inverse kinematics on the
robot's own model fed with measured joints (damped least squares on MuJoCo's Jacobians, a joint-limit active set, the
waist's pitch allowed to lean up to 0.3 rad) for a pre-grasp, the grasp, a lift, and two raised poses that bring the hand
in over a table (below). The hand is pitched flat first (then 15 to 45 degrees) with the pinch point at the object's
centre; a reach is taken only if its arm paths clear the declared tables and a carry pose exists (below): **S502** if
none does, before any arm motion. The arm moves in min-jerk segments of at least 1.6 s with the `arm_sdk` weight ramped in,
first out of the hip's way, then raised beside the body and in over the object; a joint further than 0.12 rad from its
target when a segment ends is **S503**. The Dex3 hand closes (**S505** if it is more than 0.25 rad from its goal); the
object's marker must be seen within 9 cm of the hand's grasp point computed from the measured joints (**S506**; 4-5 cm
when held right); then the stand-in welds the object to the hand (`weld_after_hand_close`), and the arm lifts, retreats the
same way and goes to the **carry pose**. **`place r O at L`** is the mirror: the region's centre through the station marker
seen now, reach in over the table, lower (the object is released 1.2 cm above the surface and drops, so that the open
hand clears the table), open, detach, back out, to the walking controller's own arm pose.
**`wait for`** is simulation time (`lowstate.tick`).

**Arm paths and the carry pose** (`kinematics.Clearance`, `ops.py`). The scene's hands touch neither objects nor tables, so
the simulator never shows a path through a table; the executor therefore checks its own. With MuJoCo's distance queries on
the robot's own collision shapes (the arm links, the palm, the finger capsules, and the held object as a box fixed to the
hand, in the pose it is held in), fed the belief pose at the station, every straight joint-space leg of a grasp, a place and
the observation motion is tested against the site's declared tables and obstacles grown by **3 cm sideways and 1.5 cm
vertically**; on the legs where the hand is meant to be at the object (the last approach, the first lift, the lowering and
the release) the hand and the object, which rests on its table there, may overlap the table by 1 cm and no more. A reach
that has an inverse-kinematics solution but no clear path is dropped for the next one; if none is left, **S502** ("no clear
path"). Measured on the shipped sites, the phase 2 grasp (hand pitched 15 degrees, the pinch 1 cm below the object's centre)
put the open fingers 3.2 cm into the table and the retreat passed the object through its edge. Now the hand is flat with
the pinch at the object's centre (the closed fingers stay level with the object's top face, where the marker is, and the open
hand just touches the table top, to within 1 cm), the hand comes in through a pose raised beside the body (tried further out if the object
would pass the table's corner) and one straight above the pre-grasp. The **carry pose** of a held object is the walking
controller's own arm pose moved only as far as it takes for the object, welded as grasped, to clear **every link of the robot**
(the holding hand's own palm and fingers excepted) by 2.1 cm in the model (measured at rest, with the arm's tracking error: 1.6-2.2 cm, accepted by the owner 2026-11-21);
in the walking pose it is 0.1-0.6 cm from the right hip link. It is planned before the arm moves (S502 if there is none) and again from the object as it is actually held.

**Arm work stance.** The free stance moved the robot by centimetres to decimetres when an arm reached (measured in the
evidence), so a site may set `robot.arm_stance: "supported"`: the stand-in welds the pelvis for the arm step and frees it
afterwards. The record says so for every step. In the scene, hands touch neither objects nor tables (grasping is a weld);
the clearance check above stands in for the collisions the scene does not have.

**Stop and S516.** On a failed step, cancel, revoke or deadline: `SetVelocity(0)`, the arms held where they are through
`arm_sdk` (the target becomes the measured joints), the hands unchanged (a held object stays held), the robot kept
standing, never `Damp`, zero torque, `SwitchToUserCtrl` or `rt/lowcmd`. The stop is confirmed from `lowstate` within 3 s of
simulation time: over 0.5 s the IMU rate is at most 0.06 rad/s, every joint's velocity at most 0.12 rad/s and the tilt at
most 15 degrees; the record has the numbers. If the IMU's tilt reaches 40 degrees at any time the run ends with **S516**
and **nothing more is sent**, the stop's own commands included.

**Record.** Schema `robolang.g1.run` 0.2 (0.2 adds the verdicts, `snapshots` and `goal_evaluated`), mirroring the ROS 2 record: `backend: {kind: "unitree-g1-mujoco", versions,
grasp_model: "weld_after_hand_close", stance_model}`, the task, bindings, `verify`, each step with `status`, `t_start_ms`,
`t_end_ms`, `wall_s` and `details` (a failing step keeps its last events), `observations` (every marker reading, in the
pelvis frame, with confidence, age and range), `stop`, and `realtime` (the real-time ratio over the run; `realtime_valid`
false if any second fell more than 5% behind real time or any step stalled; `step_compute_ms`, GC and camera numbers).
Each precondition and goal condition has its `result` (`true`, `false`, `unknown`, or `not_evaluated` when the run never got
there) and, for every atom in it, the numbers behind the verdict. `snapshots` has every observation snapshot with each
marker reading's position, confidence and age. `state` is `SUCCEEDED` only when every goal condition is `true`, and
`FAILED` with a code otherwise. CLI exit status, as the ROS 2 backend's: 0 `SUCCEEDED`, 3 for S302, 1 for every other
failure or refusal, 2 usage or source error.

## Observations and verdicts

Mirrors the [ROS 2 backend's](ros2-backend-v0.1.md#observations-and-verdicts) (decision 0046 U8). The preconditions are
decided from **one snapshot at the start pose, before any command that moves the robot**; the goal from **one snapshot
after the last step**. A false precondition is S101, an unknown one S102; a false goal condition is S301, an unknown one S302,
each with `step: null` and no stop (nothing is moving); all goal conditions true is `SUCCEEDED`. Verdicts come only from the
robot's interfaces (head camera, `rt/dex3/*/state`, `rt/lowstate`, `GetFsmId`); never MuJoCo state, the oracle or
`SportModeState`. `observe.py` is pure (no DDS), and a test checks that no executor or observation module refers to the
oracle.

**A snapshot** is about 10 head-camera frames taken after the request. In each frame every marker's pose in the pelvis
frame comes from `solvePnP` and the robot's own kinematic model fed with the joints measured at the frame's time. Per
marker the position is the median, and a frame **agrees** when its position is within **2 cm** of that median and, for a
station marker (the only kind whose rotation is used), its rotation is within **0.05 rad** of the median rotation and the
vertical it implies is within **0.03 rad** of the IMU's (the wrong one of the two poses a planar target allows has a
tilted vertical). **Confidence** is the agreeing frames over the frames taken, so a frame without the marker counts
against it. The **age** of a reading is the snapshot's simulation time minus the stamp of the newest agreeing frame. A reading is
usable only if its confidence is at least `min_confidence` and its age at most `max_age` (`@verify`; defaults 0.9 and 10 s);
otherwise its atoms are `unknown`.

| Atom | `true` | `false` | otherwise |
|---|---|---|---|
| `object_at o l` | the marker of `o` is inside `l`'s region, in the frame of `l`'s station marker seen in the same snapshot, by at least **1.5 cm** | it is outside by at least 1.5 cm, or it is seen at a closed hand (a carried object is at no place) | `unknown`: either marker not usable, or within 1.5 cm of the region's boundary |
| `holding r o` | a hand is closed (`rt/dex3/*/state` within 0.25 rad of the closing pose) and the marker of `o` is within **9 cm** of that hand's grasp point (from the measured joints) | both hands are open, or the marker is seen away from the closed hand | `unknown`: a hand neither open nor closed, no hand state, or the marker not usable |
| `robot_at r l` | `l`'s station marker is seen at the station's expected offset within **5 cm and 0.1 rad** (the pelvis's plan pose from that one marker against the station's), by at least **5 mm and 0.005 rad** | it is outside by that margin, or another station's marker confirms the robot is at that station (inside its tolerance) and that station is more than twice the tolerance from `l`'s | `unknown`: no usable marker, or within the margin of the tolerance |
| `robot_ready r` | fresh `lowstate` (messages arriving), no motor error word, IMU tilt under 15°, and `GetFsmId` is 500 (the SDK's `Start`) | a motor error, tilt over 15°, or another state id | `unknown`: stale `lowstate`, no answer from the walking service |

`and`, `or` and `not` are three-valued as in the symbolic simulator. The margins are never in favour of `true`: a value
inside a margin is `unknown`. Every `robot_at` verdict records the measured offset next to the tolerance.

**Observing a carried object: an observation motion (0046 U8 exception, approved by the owner 2026-11-21).** In the carry
pose the object hangs at the side of the thigh, **46° below the head camera's axis against a field of ±29°**, so its marker
is never detected (measured: 0 of 28 frames walking, 0 of 10 at rest; a waist lean does not bring it into view, and an arm
pose that does, 12-19° off the axis, either stops the walking policy or makes it curve and sway). So **after the last
step, and only if a goal atom names an object the robot holds**, the arm shows it to the camera: clear of the hip, raised
beside the body, and to the pose where the grasp lifted the object, in the site's arm-work stance, then back to the carry
pose. Only the arm moves, never the waist or the legs, and never for a precondition. Before it moves the whole path is
checked as above (3 cm sideways, 1.5 cm vertically); **if it is not shown clear the arm does not move**, the snapshot is
taken as the plan left the robot and the atoms about the object are `unknown` (S302). A cancel, revoke or deadline during it
stops it like any arm motion (the biped stop: arms held where they are, hands unchanged). The record's
`observation_motion` lists each one apart from the plan's steps: the phase, the atoms that asked for it, whether it was
`performed`, the waypoints, the clearance numbers (the closest link and box), the stance, and the events; the snapshot says
which motion it was taken in.

**Core outcome.** `core_outcome` is built as the ROS 2 backend builds it: the Core `outcome` payload from the goal's
`object_at`, `robot_at` and `holding` atoms, with a signed observation (key `rl:sim:g1-mujoco`, fixed, published, simulation
only) as evidence for each criterion a `true` verdict meets: confidence rounded down to two places, `valid_for` equal to
`max_age`. `robot_ready` has no Core form, so a goal that uses it (or `or`, `not`) has `core_outcome: null`.

## Failure codes

The codes and meanings of the [ROS 2 backend](ros2-backend-v0.1.md#failure-codes) (S501–S515), with S502 extended, plus:

| Code | Meaning |
|---|---|
| S502 | (the ROS 2 meaning, and:) no clear path for the arm: every reach with an inverse-kinematics solution comes within the margins of a declared table or obstacle, or leaves no carry pose that clears the held object by 2 cm. Before any arm motion |
| S516 | The robot fell or lost balance (IMU tilt beyond the limit, or the walking service left its standing and walking states). The run stops; no further command |
| S517 | Localization lost: a station marker that `move` or an observation needs was not seen within its timeout |

## Acceptance scenes and the acceptance run (phase 4)

[`scenes.py`](../backends/unitree_g1/robolang_g1/scenes.py) makes one site per task of the acceptance family (0046
section 3: the 11 single-arm tasks that passed in Gazebo and `e_park_amr`, `w_carry_hold`, `w_clear_dock`), with the
entity IDs of `bench/reference/<task>.json`, so the reference `.rob` files run unchanged. Every object is an 8 cm box
with its marker on top. Every location is a region on a table in front of a station, or a station alone.

**Layout rules, each from a measurement:**

- **One camera, so goals that are named together stand together.** Locations that a goal names together are slots of
  one table, seen from one station (`a_sort_three`, `w_split_boxes`, `l_vial_hold`), and the dock of `e_box_then_dock`
  is a standing place 12 cm behind the destination's, with its own plate. `scenes.check_visibility` applies
  `perception.visibility_violations` and one more limit: the centre of an `object_at` region is at most **0.52 m ahead
  of the pelvis**. Further out a box's 5.7 cm marker is not read surely (confidence 0.4 at 0.61 m in one x86 run, which
  the backend refuses as not seen).
- **A hand for each side of a table.** The arm planner takes a box between 5 and 20 cm to the side of the pelvis with
  the hand of that side (measured on the robot's own model). So a table holds two boxes with a hand each (12 cm either
  side), or three with two for one hand (20 and 7 cm), which leaves about 2 cm to spare against where the robot stops.
  A box outside that band is refused with S502 before the arm moves.
- **A wide table is no wider on the right than the verified one.** Five centimetres more and the arm, swung out beside
  the body, came within its 3 cm margin of the table's corner (S502).
- **The robot stands 1.30 m from the plate to grasp and 1.35 m to place**, as verified in phases 2 and 3.

The five scripted failure cases run on `e_pick_place`: the box absent (S506), the box on the other table (S506), the
destination's region 0.8 m ahead (S502 at the place, the box still in the hand), the head camera covered (S517), and a
push while walking (S516, no command after the fall).

[`phase4.py`](../backends/unitree_g1/robolang_g1/phase4.py) is the acceptance run: each task three times on a fresh
stand-in through `robolang_g1.cli`, then the cases, judged against the oracle with the bar of 0046 section 3 (at least
90% of the runs `SUCCEEDED`, at least 98% of the atoms agreeing with the oracle, no false success, every case `FAILED`
with its code). A run that fell behind real time does not count (0046 U13) and is repeated, at most twice per task; a
false success in such a run still blocks acceptance. The run can be resumed: a run that has written its `result.json`
is not run again.

## Where it runs

`.github/workflows/g1-sim.yml`: on demand, nightly, and on pull requests that change `backends/unitree_g1/` or the
workflow itself (0046 U12 said on demand and nightly; the pull-request trigger is limited to the backend's own files so
that a backend change is checked on x86_64 before it merges). Plain x86_64 runner, no GPU, no Docker; the assets and the
CycloneDDS build are cached. The job runs the backend's tests, the phase 1 harness (P1-P6, about 2 minutes), the phase 2
checks (E1-E7, about 35 minutes of simulated real time) and the phase 3 checks (O1-O7, about 40 minutes), and uploads
`harness.json`, `phase2.json`, `phase3.json` and every run's record.
The acceptance run is a second job, started by hand (`acceptance: true`): about four hours of simulated real time.
The simulation runs in real time, so a run's `realtime_valid` (every second within 5% of real time, no stall) is part of
its record; an invalid run does not count towards acceptance (0046 U13).
