# RoboLang Unitree G1 backend 0.1 (MuJoCo, simulation only)

**Status:** phases 1 and 2 of decision [0046](decisions/0046-humanoid-backend-unitree-g1.md): the stand-in, the guard and
the harness checks P1–P6 ([evidence](../docs/evidence/g1-phase1-harness.md)), and the executor (`move`, `grasp`,
`place`, `wait for`, the stop, the records; checks E1–E7, [evidence](../docs/evidence/g1-phase2-executor.md)). Goal and
precondition verdicts from observations are phase 3. Code: [`backends/unitree_g1/`](../backends/unitree_g1/).

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
real-time ratio. A run in which any second falls more than 5% behind real time does not count towards acceptance.

## Stance and stop

`stance_model` is `free` by default: the policy alone, which comes to rest after a walk (about 0.13 m and 2 s of coast
at 0.5 m/s, then about 1 cm of torso sway). `free` takes a recovery step of 0.2–0.35 m about every 45 s while standing,
so arm work longer than about 40 s uses `supported`: the torso welded to a mocap body (the fallback the owner approved,
0046 open point 1). Every record states the stance model.

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
[`pick_place.json`](../backends/unitree_g1/robolang_g1/sites/pick_place.json) and
[`carry_hold.json`](../backends/unitree_g1/robolang_g1/sites/carry_hold.json) are for `bench/reference/e_pick_place.rob`
and `w_carry_hold.rob`; a test checks the scene rule of 0046 section 3 for both (goal locations in view of the camera from
the station where the plan ends).

## Steps, observations and records

Code: `cli.py` (entry), `executor.py` (the run), `ops.py` (what is done to the robot), `perception.py`, `kinematics.py`,
`robot.py` (the SDK clients, shared with the harness). A task runs only through the robot's interfaces: `LocoClient`,
`rt/arm_sdk`, `rt/dex3/*`, `rt/lowstate`, the head camera, and (simulation only, after the guard) `robolang_sim`'s attach,
detach and stance. A test fails if an executor module refers to the oracle, `SportModeState`, or what a biped must never
be sent.

**Refused before any command** (as the ROS 2 backend, after the guard): S507 (a binding or entity the site does not
have), S509 `measure`, S510 a clock reading, S515 `message_received`, S514 a mission (before the guard), and S508 for
everything else: other steps, failure handlers, speed and region limits, `do`, and **every precondition** (their
verdicts are phase 3). Goal atoms must be ones phase 3 can observe, or S508/S507.

**`move r to L`.** The executor keeps a **belief pose** of the pelvis in the map: dead reckoning from the velocities it
commands, replaced whenever a station marker is seen (the marker's map pose composed with the inverse of its observed
pose; a solution whose vertical disagrees with the IMU's is the wrong one of the two a planar marker allows, and is
ignored). The walking stand-in turns in place only by a *shuffle* (alternating sideways commands, each with a yaw rate),
takes a step only when a command lasts about 0.3 s, and ignores commands smaller than about 0.3, so a move is: turn to face
the approach point (0.5 m before the station, facing it), walk in legs with a look between them, turn to the station's
heading, find the station marker (a scan by turning in place if it is not in view; **S517** if not seen within 70 s), then
correct with short bursts chosen from the measured step sizes until the marker puts the robot within **4.4 cm and 0.085
rad** of the station pose, wait for the stance to settle (from `lowstate` only: pelvis IMU rate and leg joint velocities
quiet), and look again (within 4.7 cm and 0.09 rad). **S504** if it does not converge in 90 s.

**`grasp r O`.** Re-observe the station marker (and re-servo if the robot moved); observe the object's marker (**S506**
before anything moves if it is not seen); choose the hand (the site's, or the nearer side); inverse kinematics on the
robot's own model fed with measured joints (damped least squares on MuJoCo's Jacobians, a joint-limit active set, the
waist's pitch allowed to lean up to 0.3 rad) for a pre-grasp, the grasp and a lift, with the hand pitched 15 to 45
degrees: **S502** if there is no solution, before any arm motion. The arm moves in min-jerk segments of at least 1.6 s with
the `arm_sdk` weight ramped in, first out of the hip's way; a joint further than 0.12 rad from its target when a segment
ends is **S503**. The Dex3 hand closes (**S505** if it is more than 0.25 rad from its goal); the object's marker must be
seen within 6 cm of the hand's grasp point computed from the measured joints (**S506**); then the stand-in welds the
object to the hand (`weld_after_hand_close`), and the arm lifts to the carry pose, which is the walking controller's own
arm pose. **`place r O at L`** is the mirror: the region's centre through the station marker seen now, reach, lower, open,
detach, retract, carry. **`wait for`** is simulation time (`lowstate.tick`).

**Arm work stance.** The free stance moved the robot by centimetres to decimetres when an arm reached (measured in the
evidence), so a site may set `robot.arm_stance: "supported"`: the stand-in welds the pelvis for the arm step and frees it
afterwards. The record says so for every step. In the scene, hands touch neither objects nor tables (grasping is a weld).

**Stop and S516.** On a failed step, cancel, revoke or deadline: `SetVelocity(0)`, the arms held where they are through
`arm_sdk` (the target becomes the measured joints), the hands unchanged (a held object stays held), the robot kept
standing, never `Damp`, zero torque, `SwitchToUserCtrl` or `rt/lowcmd`. The stop is confirmed from `lowstate` within 3 s of
simulation time: over 0.5 s the IMU rate is at most 0.06 rad/s, every joint's velocity at most 0.12 rad/s and the tilt at
most 15 degrees; the record has the numbers. If the IMU's tilt reaches 40 degrees at any time the run ends with **S516**
and **nothing more is sent**, the stop's own commands included.

**Record.** Schema `robolang.g1.run` 0.1, mirroring the ROS 2 record: `backend: {kind: "unitree-g1-mujoco", versions,
grasp_model: "weld_after_hand_close", stance_model}`, the task, bindings, `verify`, each step with `status`, `t_start_ms`,
`t_end_ms`, `wall_s` and `details` (a failing step keeps its last events), `observations` (every marker reading, in the
pelvis frame, with confidence, age and range), `stop`, and `realtime` (the real-time ratio over the run; `realtime_valid`
false if any second fell more than 5% behind real time or any step stalled; `step_compute_ms`, GC and camera numbers).
**Goal and precondition verdicts are phase 3**: the goal is `not_evaluated`, `goal_evaluated` is false, and the run's
`state` is `STEPS_COMPLETED` or `FAILED`, **never `SUCCEEDED`**. CLI exit status: 4 every step ran (not a success), 1
failed or refused, 2 usage or source error.

## Failure codes

The codes and meanings of the [ROS 2 backend](ros2-backend-v0.1.md#failure-codes) (S501–S515), plus:

| Code | Meaning |
|---|---|
| S516 | The robot fell or lost balance (IMU tilt beyond the limit, or the walking service left its standing and walking states). The run stops; no further command |
| S517 | Localization lost: a station marker that `move` or an observation needs was not seen within its timeout |

## Where it runs

`.github/workflows/g1-sim.yml`: on demand, nightly, and on pull requests that change `backends/unitree_g1/` or the
workflow itself (0046 U12 said on demand and nightly; the pull-request trigger is limited to the backend's own files so
that a backend change is checked on x86_64 before it merges). Plain x86_64 runner, no GPU, no Docker; the assets and the
CycloneDDS build are cached.
