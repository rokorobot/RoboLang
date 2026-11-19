# Decision record 0046: a humanoid backend, Unitree G1 in MuJoCo (simulation only)

**Date:** 2026-11-19
**Status:** **APPROVED by the owner, 2026-11-19**: U1–U13 as recommended, and open points 1–4 as recommended
(section 6). Proposed and reviewed the same day (section 8). Direction given by the owner: "use Unitree G1 open SDK
and a good simulator for it". Phase 1 (section 7) starts. **Physical execution stays unauthorized**
([0038](0038-physical-mode-authorization-stop-approval.md); strategy H4).
**Applies to:** a new optional package `robolang-g1` in `backends/unitree_g1/` (not part of
`pip install robolang`), a new `spec/g1-backend-v0.1.md`, two new run failure codes, a CI workflow `g1-sim.yml`, an
acceptance run, and two new hazards in the [safety case draft](../../docs/safety-case-draft.md).
**Evidence and direction:** [STRATEGY](../../docs/STRATEGY.md) pillar P4 ("the same task runs in the symbolic
simulator, then in a physics simulator, then on hardware behind a safety case"). Humanoid work leads the
[activity atlas](../../docs/plans/humanoid-activities.md) (action is the top direction, priority 4.52). Safety case gaps
G2 and G5 need a named robot, and G7 asks the owner to choose one: this record makes the G1 a concrete candidate
without committing to it. The [feasibility spike](../../docs/evidence/g1-spike.md) of 2026-11-19 measured what is
below.

## 1. What changes, and why the ROS 2 backend is not reused

The ROS 2 backend ([0029](0029-physics-simulation-backend.md)) drives a fixed arm with MoveIt pose goals. A humanoid
is not controlled that way. The G1's own controller keeps it balanced and walking. Programs talk to it through
**Unitree's SDK** over CycloneDDS: a walking service (`LocoClient`, RPC service `sport`: velocity commands, stand,
damp), upper-body joint targets blended over the walking controller (`rt/arm_sdk`), the hands (`rt/dex3/*`) and the
robot state (`rt/lowstate`). The SDK has no G1 camera client (it has them for Go2 and B2), so on the robot the head
camera is read directly. The backend uses exactly these interfaces and nothing else.

```
.rob ─validate─▶ Task IR ─▶ robolang-g1 executor ─┬─ LocoClient (rpc "sport") ─▶ walking stand-in ─┐
                                 ▲                ├─ rt/arm_sdk, rt/dex3/*  ───▶ arm and hand PD  ──┼─▶ MuJoCo
                                 │                └─ rt/lowstate, head camera ◀────────────────────┘
                                 └─ goal evaluated from observations (head camera markers, joints, IMU)
```

In simulation, **stand-in services** answer on those interfaces: a walking controller (a pretrained Unitree policy
behind the real `sport` RPC API), the `arm_sdk` blending, the hands and a rendered head camera. On hardware (H4, only
with the owner's authorization), the stand-ins are removed and the robot's own services answer; the executor and the
`.rob` do not change. This is the same seam as 0029, where `gz_ros2_control` stands in for UR's driver, with one
difference stated plainly: **the walking stand-in is not Unitree's controller.** Gait, speed and drift will differ on
the real robot, so the executor never depends on how the stand-in walks. It commands, then checks with its own sensors.

## 2. Decisions proposed (recommendation first; owner picks)

| # | Question | Recommended | Alternatives |
|---|---|---|---|
| U1 | Robot | **Unitree G1 EDU, 29 DoF** (3-DoF waist, two 7-DoF arms), with **three-finger Dex3-1 hands**: the version with `arm_sdk` and dexterous hands. The MJCF comes from `unitree_mujoco` (BSD-3), with the hands from the `mujoco_menagerie` G1 model or Unitree's `g1_29dof_with_hand`; phase 1 picks one and confirms the hand kinematics | H1 or H1-2: larger, fewer users, same SDK. The 23-DoF G1: no wrist pitch or yaw, too weak for grasping |
| U2 | Simulator | **MuJoCo** (Apache-2.0). Spike: 24× real time with the whole-body policy at a 2 ms step (U13) on a 4-core sandbox with no GPU; head camera rendered in software at 113–148 ms per 640×480 frame. Unitree's own `unitree_mujoco` targets it | **Isaac Sim / Isaac Lab** (`unitree_sim_isaaclab`): Unitree's photorealistic G1 manipulation scenes, but it needs an NVIDIA RTX GPU (none in CI or the sandbox). Worth one later check of camera realism via `room-cli gpu`, not the base. **Gazebo**: no maintained G1 walking stack |
| U3 | Interface to the robot | **Unitree's SDK** (`unitree_sdk2_python`, BSD-3) over CycloneDDS, on **DDS domain 1 and the loopback interface only** (the real robot uses domain 0). Spike: the unmodified `LocoClient` reached a sim-side `sport` service, in the same process and from a separate process, round trip 0.4–1.3 ms | `unitree_ros2`: the same DDS messages through ROS 2. It adds ROS for nothing; the G1's native interface is DDS. Calling MuJoCo directly would skip the interface this backend exists to test |
| U4 | Walking and standing stand-in | **Walking:** `unitree_rl_lab`'s whole-body G1 velocity policy (Apache-2.0, ONNX, 50 Hz) behind the `sport` RPC API. Spike at a 2 ms step: stands from rest with 0.7 cm sway; walks 2.26 m in 5 s at 0.5 m/s with 0.3 cm sideways drift; never fell in 32 s that included a one-arm reach and 9 s with both arms overridden to a carry pose. **Standing still at a station: free.** After a walk it coasts about 0.14 m, then stands with 0.8 cm sway, no support (the earlier "does not come to rest" came from a 5 ms step; see U13). Fallback, approved under open point 1: the pelvis **supported** by a weld while the robot stands for arm work. Every record states which: `stance_model: "free"` (default) or `"supported"`. Arm motions are ramped: a sudden reach moved the free robot about 12 cm | `unitree_rl_gym`'s G1 walking policy (BSD-3): walked 2.17 m, but needs arms near its trained pose and also keeps stepping at zero command. Write our own balance controller: control research the real robot does not need from us. Train a policy: needs a GPU and owns a problem Unitree already solves on hardware |
| U5 | How a plan step runs | `move r to L`: walk with `SetVelocity` towards the site's **station** for `L` (a standing pose in front of it), then servo on the station's fiducial marker (a site installation, like the Gazebo cell's markers) until the head camera sees it at the expected offset. `grasp r O`: standing still, reach with inverse kinematics to the object's marker pose seen by the head camera, close the hand, check the marker is in the hand, attach, lift to a carry pose. `place r O at L`: reach over the location's region, open, detach, retract. `wait for`: simulation clock (U13). Everything else is refused before any motion, as in the ROS 2 backend (S508–S515) | Teleport the robot between stations: no walking tested. A navigation stack (SLAM, path planner) on day one: a whole project before the first verified pick |
| U6 | Arms and walking | **No reaching while walking.** Arms move only while the robot stands. While walking, a held object stays in one fixed carry pose: the policy's default arm pose, which the spike held by override while walking without a fall. Phase 1 adds a 0.2–0.5 kg object; if walking with it fails, the tasks that walk while holding (`w_carry_hold`, `w_clear_dock`) leave the acceptance family, and the evidence says so | Loco-manipulation (reaching while walking): research, and not something the real G1 offers through `arm_sdk` either |
| U7 | Grasping | **Attach after the hand closes** (a MuJoCo weld), only when the object's marker is observed in the hand. Records say `grasp_model: "weld_after_hand_close"`. Same reasoning as 0029 X5: the acceptance run tests planning, execution, observation and verdicts, not grasp contact | Friction grasping with Dex3 in MuJoCo: would make the acceptance run measure grasp tuning |
| U8 | Evidence (the point) | As 0029 X6: **observations only.** `object_at o l`: the marker of `o` seen by the head camera inside the region of `l`, located through the station marker in the same image. `holding r o`: the hand closed (from `rt/dex3/*/state`) and the marker of `o` seen at the hand, whose pose comes from the measured joints. `robot_at r l`: the station marker of `l` seen at the expected offset (tolerances set in phase 1, about 5 cm and 0.1 rad). `robot_ready r`: fresh `lowstate`, no motor error, upright from the IMU, and the walking service in its standing or walking state. A condition is observed from where the robot is: preconditions from the start pose, the goal from the pose after the last step; an atom out of view is `unknown`, so it fails closed (S102, S302). **Never evidence:** MuJoCo state and `SportModeState` (the simulator publishes the robot's position there; the real robot does not once its motion service is off). Both are used only by the test oracle | Fixed site cameras, as in the Gazebo cell: they make every goal visible, but a humanoid's site will not have them. They stay possible as an optional site entry later. An "observation tour" (walking to each goal location to look): it moves the robot outside the approved plan, so not in 0.1 |
| U9 | Safety mode and stop | Simulation only. Before any command the backend requires DDS domain ≠ 0, the loopback interface, an advancing simulation clock, and an answer from a simulation-identity API that only the stand-in implements (on a real robot it is absent, so the backend refuses with S501). No flag turns this off. **Stop ([0038](0038-physical-mode-authorization-stop-approval.md)) for a biped:** zero velocity, arms held where they are through `arm_sdk`, hands unchanged, the robot kept standing. **Never `Damp()`, zero torque or `SwitchToUserCtrl`:** for an arm, holding position is safe; for a biped, cutting power means falling. The executor never writes `rt/lowcmd` and never touches the leg motors | Power-off as the safe state: wrong for a humanoid (hazard H25 below) |
| U10 | Packaging | A separate package **`robolang-g1`** in `backends/unitree_g1/`, depending on `robolang`, `mujoco`, `numpy`, `opencv-python-headless`, `onnxruntime` (MIT), `cyclonedds` 0.10.x (with its C library) and `unitree_sdk2py` at a pinned commit. No PyTorch. Inverse kinematics uses MuJoCo's own Jacobians (damped least squares), no IK library. Robot models and policy weights are **fetched at pinned commits** by the harness, with their licences, not copied into this repository (about 35 MB of meshes) | Inside `robolang`: forbidden, the language package stays dependency-free (0029 X8). An IK library (`mink`, Apache-2.0): fine later if damped least squares falls short |
| U11 | Records | Run records gain `backend: {kind: "unitree-g1-mujoco", versions…}` with the MuJoCo version, SDK and model commits and the policy's hash, plus `grasp_model`, `stance_model` and the measured real-time ratio (U13). Physics runs are compared by outcome, not byte for byte (0029 X9) | — |
| U12 | Where it runs | GitHub Actions `g1-sim.yml`, on demand and nightly, on a plain x86_64 runner with no GPU and no Docker requirement; the CycloneDDS build is cached. It also runs in a Cloudroom sandbox (the ROS 2 backend does not), so development iterations are fast | Every PR: too slow. Docker-only like the ROS 2 harness: allowed, but not needed |
| U13 | Time | **Physics step 2 ms, joint PD every step, policy every 10th step (50 Hz).** At 5 ms the explicit arm PD is unstable (damping 10 on about 0.02 kg·m² of joint inertia: dt·kd/I ≈ 2.5 > 2), and the arms chatter at their torque limits. The SDK works in wall-clock time (RPC timeouts, the `duration` of `SetVelocity`), as on the robot. So the stand-in **paces MuJoCo to real time**, as `unitree_mujoco` does (the spike has 24× headroom at 2 ms), renders camera frames from a copy of the state on its own thread so rendering never stalls physics, and publishes simulation time. `wait for` and observation ages (`@verify max_age`) use simulation time. A run in which simulation falls more than 5% behind real time over any second is a harness failure: recorded, and not counted towards acceptance | Lockstep (the executor advances the simulation): exact, but the real robot does not wait for the executor, so it would test a timing the robot never has. Free-running faster than real time: breaks every wall-clock timeout |

**Failure codes.** The backend uses the ROS 2 backend's codes with the same meanings (S501 guard, S502 no motion
plan or no inverse-kinematics solution, S503 execution failed, S504 timeout, S505 hand, S506 attach or detach, S507 site
or binding, S508–S515 refusals before motion). It adds two, listed in `g1-backend-v0.1.md` and added as reserved rows
to `ros2-backend-v0.1.md`, so that no backend reuses them for something else:

| Code | Meaning |
|---|---|
| S516 | The robot fell or lost balance (IMU tilt beyond the limit, or the walking service left its standing and walking states). The run stops; no further command |
| S517 | Localization lost: a station marker that `move` or an observation needs was not seen within its timeout |

## 3. How it is judged

**Acceptance:** one benchmark family passes end to end on the humanoid, with the bar of 0029.

1. **Family:** the 11 single-arm tasks that passed in Gazebo (0029 section 7), now with walking between stations,
   plus the three mobile-base tasks Gazebo could not run: `e_park_amr`, `w_carry_hold`, `w_clear_dock` (14 tasks).
   The reference `.rob` files run **unchanged**; only the world changes (a MuJoCo scene and site file with the same
   entity IDs).
2. **Visibility rule for scenes** (follows from U8, open point 2): every location a task's goal names is in the head
   camera's view from the station where its plan ends, and every location a precondition names is in view from the
   start pose. Five tasks need this: `a_sort_three` (three goal slots), `w_split_boxes` (two), `e_box_then_dock` (the box's
   destination seen from the dock), `l_vial_hold` (the spinner seen from the bench), and `w_part_precheck` (its source
   seen from the start). The scene generator checks the rule from the site geometry before a run; a task whose
   layout cannot meet it leaves the family, and the evidence names it.
3. **Pass bar:** at least 90% of runs `SUCCEEDED` from observations over 3 repeats; observed goal verdicts agree with
   the ground-truth oracle on at least 98% of checks; **0 false successes** (blocks acceptance outright). The
   evidence reports the stance model of every run (U4).
4. **Scripted failure cases**, each ending `FAILED` with its code, never `SUCCEEDED`: object absent; object moved
   before the grasp; destination out of reach (too high or behind a barrier); head camera covered; and, new for a
   humanoid, **a push while walking** that makes the robot fall: `FAILED` with S516, a recorded stop and no further
   command.

## 4. Delivery (after approval; each its own PR)

1. **Harness and go/no-go** (section 7).
2. **Executor:** `move` (walk, then servo on the station marker), `grasp` and `place` (inverse kinematics on the
   robot's own model fed with measured joints; hand; attach), `wait for`, the biped stop, codes, records.
3. **Observations:** head-camera `object_at`, `holding`, `robot_at`, `robot_ready`, with confidence and age under
   `@verify`; the test-only oracle.
4. **Acceptance run** and its evidence document.

**Size:** larger than H3, which went from approval (2026-10-30) to a passing acceptance run in about two days.
Walking, balance and inverse kinematics are new. Estimate one to three weeks, with phase 1 as the go/no-go.

## 5. Consequences and risks

- **Standing still after walking: resolved in phase 1** at a 2 ms step (U13): the policy stands free after a walk.
  What remains is that arm work disturbs a free stance (a sudden reach moved the robot about 12 cm), so arm motions
  are ramped and phase 2 measures the stance during reaches. The supported stance stays the approved fallback; it
  replaces balance that Unitree's own controller provides on the robot, so it never proves that the arm work balances.
- **The stand-in is not Unitree's controller.** A pass in simulation says the executor, observations and verdicts
  work; it says nothing about how the real G1 walks. That is why every motion is closed on observation.
- **One moving camera.** Goals about places out of view are `unknown`, which fails closed (S302). The visibility rule
  (section 3) makes this a property of the scene, stated in the evidence, rather than a hidden source of failures.
- **Two large items in flight.** I7 (RoboLang 2.3, [0045](0045-choosing-at-run-time.md)) is approved; this would be
  the second (strategy rule 3 allows two). This backend does not depend on 0045: bindings stay fixed and objects
  carry markers. Choosing objects at run time on the humanoid is a later backend version. It changes no language,
  Core or benchmark file, so the per-PR benchmark does not run for it.
- **New hazards for the safety case:** **H25** *a stop that removes balance makes a humanoid fall* (U9's stop is the
  control; a confirmed-standing check like H18's follows in phase 2), and **H26** *the robot's own wireless services
  are compromised*. G1, H1, Go2 and B2 firmware were publicly reported vulnerable in 2025 (BLE setup: CVE-2025-60250,
  CVE-2025-35027), with reports of undisclosed telemetry. That does not touch simulation, but a G1 pilot (G7) would
  need network isolation and its own risk assessment.
- **Physical mode** stays out of scope. This record adds no way to reach hardware, and U9 makes the backend refuse a
  real robot.

## 6. Open points for the owner

1. **Supported stance (U4).** If phase 1 cannot make the robot stand free after walking, may the acceptance run use a
   supported stance during arm work, recorded in every run? I recommend **yes**: it stands in only for balance that
   the robot's own controller provides on hardware, as 0029's detachable joint stood in for grasp contact. Without it,
   the backend waits on locomotion work that is not RoboLang's.
2. **Visibility rule (section 3).** Accept that five of the 14 scenes place their goal or precondition locations in
   view of one station? I recommend **yes**. The alternatives are motion outside the approved plan (an observation
   tour) or fixed cameras a humanoid site will not have.
3. **G1 as a simulation target only.** This record does not pick the hardware pilot (G7). Given the 2025 CVEs,
   should the G1 stay a simulation target until G7 is decided on its own? I recommend **yes**.
4. **Second large item.** Start this beside I7? I recommend **yes**: it touches neither the language nor the
   benchmark, so the two do not compete for the same files or gates.

An approval can read: "U1–U13 as recommended; open points 1–4 as recommended".

## Owner's answers (2026-11-19)

The owner approved U1–U13 as recommended and answered the open points as recommended: (1) a supported stance during
arm work may stand in for the robot's own balance if phase 1 cannot make it stand free, recorded in every run as
`stance_model: "supported"`; (2) the visibility rule for the five scenes is accepted; (3) the G1 stays a simulation
target only, and the hardware pilot (G7) is decided on its own; (4) this backend runs beside I7. Hazards H25 and H26
are added to the [safety case draft](../../docs/safety-case-draft.md).

## 7. Phase 1 plan (starts on approval)

**Builds:** the `backends/unitree_g1/` skeleton; a script that fetches the G1 model, hands and policy at pinned
commits and builds CycloneDDS; the scene and site generator; the stand-in (MuJoCo paced to real time, the `sport` RPC
service, `arm_sdk` blending, hands, `lowstate`, head-camera frames, simulation time, the identity API); the guard; and
the `g1-sim.yml` workflow (on demand and nightly).

**Checks** (one harness script writes `harness.json`; the oracle may read MuJoCo state here, the backend never does):

| # | Check | Pass |
|---|---|---|
| P1 | Guard | Refuses with S501 on domain 0, on a non-loopback interface, and when the identity API is missing, with no command sent |
| P2 | Walking | `LocoClient.SetVelocity(0.5, 0, 0)` for 4 s moves the robot 1.6–2.4 m forward, ≤ 15 cm sideways, no fall; three runs |
| P3 | Standing | After P2's walk and the stand-in's stop: torso sway ≤ 2 cm over 10 s. Reports `stance_model` (`free` or `supported`) |
| P4 | Arms | While standing, `arm_sdk` targets are tracked within 0.05 rad; walking 2 m with the carry pose and a 0.3 kg object attached: no fall |
| P5 | Camera | A frame from the stand-in's camera, the marker detected, its position within 2 cm of the oracle at 0.5–1 m |
| P6 | Time | Simulation within 5% of real time over the whole check, with the camera streaming at 5 Hz |

**Go** when P1, P2, P5 and P6 pass, and P3 passes free or (with open point 1) supported. P4 failing removes the two
walk-while-holding tasks from the family (U6). **No-go**, back to the owner with the evidence, when P2 or P3 cannot pass
at all. The phase 1 PR's evidence document gives `harness.json` and the commands that produced it.

## Resolved in phase 1 (2026-11-19)

Phase 1 passed on aarch64: P1–P6 pass, twice with the free stance and once supported, and once more when re-run
independently ([evidence](../../docs/evidence/g1-phase1-harness.md); contract in
[`g1-backend-v0.1.md`](../g1-backend-v0.1.md)). Decided by it:

- **U1:** the hands come from `unitree_rl_gym`'s `g1_29dof_with_hand.xml` (the same 35.11 kg body as
  `unitree_mujoco`, torque motors, 36.17 kg with the hands). Menagerie's model is a different body revision with
  position actuators. Finger collision and visual shapes are capsules, because the meshes cost about 120 ms per
  camera frame.
- **U4:** `stance_model: "free"` by default; `supported` (a weld) remains the fallback. `free` takes a recovery step
  about every 45 s while standing, so arm work longer than about 40 s uses `supported`.
- **U13:** a 2 ms step, PD every step, policy at 50 Hz. 5 ms is unstable (dt·kd/I up to 4.5).
- **U9:** the stand-in refuses every walking-service state change except standing or walking (code 7400), `Damp`
  included.
- **U12:** the workflow also runs on pull requests that change the backend, so x86_64 is checked before merge.

Not yet run on x86_64; margins are tight in places (P3 free sway 1.1 cm against 2 cm; supported sideways drift
0.118 m against 0.15 m; dt·kd/I 1.8 against 2). Listed in the evidence's open issues.

## 8. Review and corrections (2026-11-19)

Reviewed before approval. Changed: U4 now rests on a corrected port of the whole-body policy (the first port applied
the joint gains in the wrong order and fell); its open problem is stopping after walking, not falling. U13 (time) was
missing: the SDK's wall-clock timeouts need a simulation paced to real time. The visibility rule (section 3) was
missing: five acceptance tasks would otherwise have ended `unknown` by design. U3 is now tested across processes.
U10 drops PyTorch and names the IK method. The head camera is outside the SDK on the G1. Hazards are numbered. Open
points (section 6) and the phase 1 plan (section 7) were added so that approval and the next step need no further
design.

**Correction in phase 1 (after approval, same day).** The spike ran the whole-body policy with a 5 ms physics step, at
which the explicit arm PD is unstable. That instability made the robot creep on after walking. At 2 ms the policy
stands free ([evidence](../../docs/evidence/g1-spike.md), round 3). U4 and U13 are updated: the free stance is the
default, the weld-supported stance the fallback the owner approved. The decisions themselves are unchanged.
