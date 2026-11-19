# Decision record 0046: a humanoid backend, Unitree G1 in MuJoCo (simulation only)

**Date:** 2026-11-19
**Status:** **PROPOSED for the owner, 2026-11-19.** Direction given by the owner the same day: "use Unitree G1 open SDK
and a good simulator for it". The owner picks U1–U12; the recommendation comes first. Design only: no backend code
yet. **Physical execution stays unauthorized** ([0038](0038-physical-mode-authorization-stop-approval.md); strategy H4).
**Applies to (when decided):** a new optional package `robolang-g1` in `backends/unitree_g1/` (not part of
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
is not controlled that way. The G1's own controller keeps it balanced and walking; programs talk to it through
**Unitree's SDK**: a walking service (`LocoClient`, RPC service `sport`: velocity commands, stand, damp), upper-body
joint targets blended over the walking controller (`rt/arm_sdk`), the hands (`rt/dex3/*`), and the robot state
(`rt/lowstate`), all over CycloneDDS. The backend uses exactly these interfaces and nothing else.

```
.rob ─validate─▶ Task IR ─▶ robolang-g1 executor ─┬─ LocoClient (rpc "sport") ─▶ walking stand-in ─┐
                                 ▲                ├─ rt/arm_sdk, rt/dex3/*  ───▶ arm and hand PD  ──┼─▶ MuJoCo
                                 │                └─ rt/lowstate, head camera ◀────────────────────┘
                                 └─ goal evaluated from observations (head camera markers, joints, IMU)
```

In simulation, **stand-in services** answer on those interfaces: a walking controller (a pretrained Unitree policy
behind the real `sport` RPC API), the `arm_sdk` blending, the hands and the head camera. On hardware (H4, only with
the owner's authorization), the stand-ins are removed and the robot's own services answer; the executor and the
`.rob` do not change. This is the same seam as 0029, where `gz_ros2_control` stands in for UR's driver, with one
difference stated plainly: **the walking stand-in is not Unitree's controller.** Gait, speed and drift will differ on
the real robot, so the executor never depends on how the stand-in walks. It commands, then checks with its own sensors.

## 2. Decisions proposed (recommendation first; owner picks)

| # | Question | Recommended | Alternatives |
|---|---|---|---|
| U1 | Robot | **Unitree G1 EDU, 29 DoF** (3-DoF waist, two 7-DoF arms), with **three-finger Dex3-1 hands**: the version with `arm_sdk` and dexterous hands. The MJCF comes from `unitree_mujoco` (BSD-3), with the hands from the `mujoco_menagerie` G1 model or Unitree's `g1_29dof_with_hand`; phase 1 picks one and confirms the hand kinematics | H1 or H1-2: larger, fewer users, same SDK. The 23-DoF G1: no wrist pitch or yaw, too weak for grasping |
| U2 | Simulator | **MuJoCo** (Apache-2.0). Spike: 18–24× real time on this 4-vCPU sandbox with no GPU; head camera rendered in software at 113–148 ms per 640×480 frame. Unitree's own `unitree_mujoco` targets it | **Isaac Sim / Isaac Lab** (`unitree_sim_isaaclab`): Unitree's photorealistic G1 manipulation scenes, but it needs an NVIDIA RTX GPU (none in CI or the sandbox). Worth one later check of camera realism via `room-cli gpu`, not the base. **Gazebo**: no maintained G1 walking stack |
| U3 | Interface to the robot | **Unitree's SDK** (`unitree_sdk2_python`, BSD-3) over CycloneDDS, on **DDS domain 1 and the loopback interface only** (the real robot uses domain 0). Spike: the unmodified `LocoClient` reached a sim-side `sport` service, round trip under 1 ms | `unitree_ros2`: the same DDS messages through ROS 2. It adds ROS for nothing; the G1's native interface is DDS. Calling MuJoCo directly would skip the interface this backend exists to test |
| U4 | Walking and standing stand-in | Behind the `sport` RPC API, in this order: **(a)** `unitree_rl_lab`'s whole-body G1 velocity policy (Apache-2.0, ONNX), if phase 1 makes it walk and stand; else **(b)** `unitree_rl_gym`'s G1 walking policy (BSD-3; walked 2.17 m in the spike) for walking, with the torso **supported** in MuJoCo only while the robot stands for arm work, stated in every record as `stance_model: "supported"` (a shortcut like 0029's detachable joint) | Write our own balance controller: control research the real robot does not need from us (its own controller balances). Train a policy: needs a GPU and owns a problem Unitree already solves on hardware |
| U5 | How a plan step runs | `move r to L`: walk with `SetVelocity` towards the site's **station** for `L` (a standing pose in front of it), then servo on the station's floor or table marker until the head camera sees it at the expected offset. `grasp r O`: standing still, reach with inverse kinematics to the object's marker pose seen by the head camera, close the hand, check the marker is in the hand, attach, lift to a carry pose. `place r O at L`: reach over the location's region, open, detach, retract. `wait for`: simulation clock. Everything else is refused before any motion, as in the ROS 2 backend (S508–S515) | Teleport the robot between stations: no walking tested. A navigation stack (SLAM, path planner) on day one: a whole project before the first verified pick |
| U6 | Arms and walking | **No reaching while walking.** Arms move only while the robot stands. While walking, a held object stays in one fixed carry pose close to the trained arm pose (the spike showed that raised forearms spoil the walking policy). Phase 1 measures walking with a carry pose and a 0.2–0.5 kg object; if that fails, tasks that walk while holding (`w_carry_hold`, `w_clear_dock`) leave the acceptance family, and the record says so | Loco-manipulation (reaching while walking): research, and not something the real G1 offers through `arm_sdk` either |
| U7 | Grasping | **Attach after the hand closes** (a MuJoCo weld), only when the object's marker is observed in the hand. Records say `grasp_model: "weld_after_hand_close"`. Same reasoning as 0029 X5: the acceptance run tests planning, execution, observation and verdicts, not grasp contact | Friction grasping with Dex3 in MuJoCo: would make the acceptance run measure grasp tuning |
| U8 | Evidence (the point) | As 0029 X6: **observations only.** `object_at o l`: the marker of `o` seen by the head camera inside the region of `l`, located through the station marker in the same image. `holding r o`: the hand closed (from `rt/dex3/*/state`) and the marker of `o` seen at the hand, whose pose comes from the measured joints. `robot_at r l`: the station marker of `l` seen at the expected offset (tolerances set in phase 1, about 5 cm and 0.1 rad). `robot_ready r`: fresh `lowstate`, no motor error, upright from the IMU, and the walking service in its standing or walking state. **Never evidence:** MuJoCo state and `SportModeState` (the simulator publishes the robot's position there, the real robot does not once its motion service is off). Both are used only by the test oracle | Fixed site cameras, as in the Gazebo cell: they make every goal visible, but a humanoid's site will not have them. They stay possible as an optional site entry later. An "observation tour" (walking to each goal location to look): it moves the robot outside the approved plan, so not in 0.1 (a goal atom out of view is `unknown`, so the run ends `FAILED` with S302) |
| U9 | Safety mode and stop | Simulation only. Before any command the backend requires DDS domain ≠ 0, the loopback interface, an advancing MuJoCo clock, and an answer from a simulation-identity API that only the stand-in implements (on a real robot it is absent, so the backend refuses with S501). No flag turns this off. **Stop ([0038](0038-physical-mode-authorization-stop-approval.md)) for a biped:** zero velocity, arms held where they are through `arm_sdk`, hands unchanged, the robot kept standing. **Never `Damp()`, zero torque or `SwitchToUserCtrl`:** for an arm, holding position is safe; for a biped, cutting power means falling. The executor never writes `rt/lowcmd` and never touches the leg motors | Power-off as the safe state: wrong for a humanoid (new hazard below) |
| U10 | Packaging | A separate package **`robolang-g1`** in `backends/unitree_g1/`, depending on `robolang`, `mujoco`, `numpy`, `opencv-python-headless`, `cyclonedds` 0.10.x (with its C library) and `unitree_sdk2py` at a pinned commit; `onnxruntime` only with U4 (a). Robot models and policy weights are **fetched at pinned commits** by the harness, with their licences, not copied into this repository (about 35 MB of meshes). The `rl_gym` policy runs in numpy (no PyTorch) | Inside `robolang`: forbidden, the language package stays dependency-free (0029 X8) |
| U11 | Records | Run records gain `backend: {kind: "unitree-g1-mujoco", versions…}` with the MuJoCo version, SDK and model commits and the policy's hash, plus `grasp_model` and `stance_model`. Physics runs are compared by outcome, not byte for byte (0029 X9) | — |
| U12 | Where it runs | GitHub Actions `g1-sim.yml`, on demand and nightly, on a plain x86_64 runner with no GPU and no Docker requirement; the CycloneDDS build is cached. It also runs in a Cloudroom sandbox (the ROS 2 backend does not), so development iterations are fast | Every PR: too slow. Docker-only like the ROS 2 harness: allowed, but not needed |

**Failure codes.** The backend uses the ROS 2 backend's codes with the same meanings (S501 guard, S502 no motion
plan or no inverse-kinematics solution, S503 execution failed, S504 timeout, S505 hand, S506 attach or detach, S507 site
or binding, S508–S515 refusals before motion). It adds two, reserved for every backend:

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
2. **Pass bar:** at least 90% of runs `SUCCEEDED` from observations over 3 repeats; observed goal verdicts agree with
   the ground-truth oracle on at least 98% of checks; **0 false successes** (blocks acceptance outright).
3. **Scripted failure cases**, each ending `FAILED` with its code, never `SUCCEEDED`: object absent; object moved
   before the grasp; destination out of reach (too high or behind a barrier); head camera covered; and, new for a
   humanoid, **a push while walking** that makes the robot fall: `FAILED` with S516, a recorded stop and no further
   command.

## 4. Delivery (after approval; each its own PR)

1. **Harness and go/no-go.** Package skeleton; scene and site generator; stand-in services (`sport` RPC, `arm_sdk`
   blending, hands, `lowstate`, head camera frames); the guard. CI walks the robot 2 m, **stands still 10 s with
   torso sway ≤ 2 cm**, takes a frame and detects a marker. This decides U1 (hand model) and U4 (a or b). If neither
   U4 option stands still, the work stops here and returns to the owner with the evidence.
2. **Executor:** `move` (walk, then servo on the station marker), `grasp` and `place` (inverse kinematics on the
   robot's own model fed with measured joints; hand; attach), `wait for`, the biped stop, codes, records.
3. **Observations:** head-camera `object_at`, `holding`, `robot_at`, `robot_ready`, with confidence and age under
   `@verify`; the test-only oracle.
4. **Acceptance run** and its evidence document.

**Size:** larger than H3, which went from approval (2026-10-30) to a passing acceptance run in about two days. Walking, balance and inverse kinematics
are new. Estimate one to three weeks, with phase 1 as the go/no-go.

## 5. Consequences and risks

- **Standing still is unsolved** (spike): the `rl_gym` policy keeps stepping at zero command (33–49 cm sway), PD
  holding falls, and a quick port of the `rl_lab` policy fell. Phase 1 resolves it or stops (section 4). U4 (b)'s torso
  support is a recorded shortcut, not a fix.
- **The stand-in is not Unitree's controller.** A pass in simulation says the executor, observations and verdicts
  work; it says nothing about how the real G1 walks. That is why every motion is closed on observation.
- **One moving camera.** Goals about places out of view are `unknown`, which fails closed (S302). Some tasks will fail
  that a fixed-camera cell would pass; that is the honest humanoid result.
- **Two large items in flight.** I7 (RoboLang 2.3, [0045](0045-choosing-at-run-time.md)) is approved; this would be
  the second (strategy rule 3 allows two). This backend does not depend on 0045: bindings stay fixed and objects
  carry markers. Choosing objects at run time on the humanoid is a later backend version.
- **New hazards for the safety case** (next free numbers): *a stop that removes balance makes a humanoid fall* (U9's
  stop is the control; a confirmed-standing check like H18 follows), and *the robot's own wireless services are
  compromised*. G1, H1, Go2 and B2 firmware were publicly reported vulnerable in 2025 (BLE setup: CVE-2025-60250,
  CVE-2025-35027), with reports of undisclosed telemetry. That does not touch simulation, but a G1 pilot (G7) would
  need network isolation and its own risk assessment.
- **Physical mode** stays out of scope. This record adds no way to reach hardware, and U9 makes the backend refuse a
  real robot.
