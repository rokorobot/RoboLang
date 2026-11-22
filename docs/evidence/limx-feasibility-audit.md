# LimX Dynamics / COSA: integration feasibility audit

**Date:** 2026-11-22. **Status:** evidence for the owner; nothing is built. **Question:** is there a public LimX
interface that RoboLang can drive and verify against, and where is the boundary?
**Sources:** the public repositories of the GitHub account `limxdynamics` (57 repositories; it is a user account,
not an organization), read at these commits:

| Repository | Commit (2026-11) | What it is |
|---|---|---|
| `limxdynamics` (hub) | `e043e02` | Index and quick starts. **No mention of COSA anywhere** |
| `limx-cli` | `8cd3d3e` | Python CLI and `SKILL.md` for AI agents; client of the robot's onboard `signaling` WebSocket service |
| `humanoid-mujoco-sim` | `390b13e` | MuJoCo sim for Oli (`HU_D04_01`, 31 joints); talks `limxsdk` low-level only |
| `humanoid-rl-deploy-python` | `592e6f0` | Stand, wave, walk ONNX controllers for Oli, switched from a (virtual) joystick |
| `tron2_mujoco_sim` | `524346d` | MuJoCo sim for TRON2; `limxsdk` low-level only |
| `tron2_env` | `d7dda51` | TRON2 arm runtime; WebSocket transport to the real robot, plus a mock transport |
| `limxsdk-lowlevel` | tree only | Headers, examples and **prebuilt** wheels/libraries; no SDK source |

## Findings

1. **COSA has no public interface.** No repository contains or documents COSA's System 2 (planning, memory) or
   an API to it. Whatever task-level interface COSA has, it cannot be integrated from public material today.
2. **The highest public layer is `limx-cli` over the `signaling` WebSocket** (`request_*`/`response_*` JSON with a
   GUID). It offers: state queries (joints, IMU, mode, `request_get_pose`), a control lock, a timed walk
   (`request_set_walk_vel_sync` repeated at `--rate-hz` for `--duration`, then zero), stand up / sit / lie down,
   pre-recorded actions and dances by name, audio and face emoji, and `raw` for any request title. It is a
   **skill/motion command surface, not a task language**: no grasp, no place, no go-to-pose, no object or location
   concepts, no goal or outcome.
3. **`--dry-run` is not a simulation.** It prints the request list the CLI would send and never connects. It checks
   nothing about the robot or the world. Using it as "verification" would be exactly the false success RoboLang
   exists to prevent.
4. **No public simulator speaks `signaling`.** Both LimX MuJoCo simulators expose only `limxsdk` low-level
   (`RobotCmd`/`RobotState`: joint q, dq, tau, Kp, Kd; IMU). The `signaling` service runs on the robot and is not
   published. So `limx-cli` cannot be exercised against anything public except a server we would write ourselves
   from the client code, which would test our guess of LimX's protocol, not LimX.
5. **The public Oli sim stack cannot manipulate.** The Oli MJCF has no hands (hand and gripper variants exist only as
   URDF); the public controllers are stand, one greeting motion and joystick-velocity walking.
6. **Safety surface differs from ours.** `limx-cli` exposes `zero-torque`, damping, `calibrate` and arbitrary `raw`
   requests, and does not take the control lock by default. RoboLang's humanoid rule (0046 U9) is that a biped is
   never put into damping or zero torque. Any adapter must be an allowlist, never a pass-through.

## Capability map: RoboLang operations against the public LimX surface

| RoboLang | `limx-cli` / `signaling` | Oli public sim (`limxsdk` low-level) |
|---|---|---|
| `move r to L` | Open-loop timed walk only; arrival needs `request_get_pose` (semantics undocumented) plus our own localization | Walk policy takes a velocity command; localization is ours (as in the G1 backend) |
| `grasp`, `place` | **None** (only named pre-recorded actions) | **None** (no hands in MJCF) |
| `wait for` | Client-side | Client-side |
| `robot_ready` | Mode + IMU queries: plausible | Joint state + IMU: yes |
| `robot_at`, `object_at`, `holding` | No perception or object API | No camera in the public sim |
| Stop (0038) | Zero-velocity walk command; must never map to damping | Our controller holds the stance |

Of the 14-task humanoid family in [0046](../../spec/decisions/0046-humanoid-backend-unitree-g1.md), **none** runs
end to end on the public LimX surface; only `move`-only tasks could, and only after we add localization and
perception ourselves.

## Corrections to the 2026-11-22 briefing

* "Physical simulation: deterministic symbolic" understates RoboLang. It already has a ROS 2 / Gazebo backend
  verified from sensor observations (H3, met 2026-11-01) and a **Unitree G1 humanoid in MuJoCo behind Unitree's own
  SDK**, with phase 2 (executor) done ([0046](../../spec/decisions/0046-humanoid-backend-unitree-g1.md)). The
  "independent second robot stack" the briefing asks for partly exists already.
* "Dry-run execution before movement" (finding 3) and "LimX CLI / simulator interface" (finding 4) do not hold as
  written.
* COSA's release dates and its 5 Hz / 50 Hz layering come from LimX's announcements and report, which this audit did
  not check. Nothing in the code above depends on them.

## Options for the owner (recommendation first)

| | Option | Cost | What it would prove |
|---|---|---|---|
| **A (recommended)** | **No LimX build now.** Finish G1 phase 3 (goal verdicts from observations), the current critical path. Keep LimX as a benchmark: re-check this audit when LimX publishes a `signaling` simulator, a hand-equipped Oli MJCF, or any COSA API | none | That RoboLang verifies outcomes on a vendor humanoid stack, which the briefing's decisive question needs first |
| B | A `robolang-limx` **command-mapping adapter** that emits allowlisted `signaling` requests, tested on fixtures. Small (days) | low | Only that the mapping is well formed. No execution and no verified outcome, so it does not answer the briefing's question |
| C | A second humanoid backend on Oli in MuJoCo through `limxsdk` low-level, built like the G1 one (stand-in walking from LimX's ONNX policy, our own camera and localization). Locomotion only; no grasping until a hand MJCF exists | weeks; x86_64 only; prebuilt binaries | Cross-vendor `move`/`robot_ready` with verified `robot_at`, through a second vendor SDK |

Rule 3 of the [strategy](../STRATEGY.md) allows two large items in flight. Option C would be the second next to G1
phase 3, and it is outside H4's exit criteria, so it needs an explicit owner decision. Option B is cheap but proves
the least, and should not be presented as interoperability evidence.
