# H3 phase 1: harness findings

Date: 2026-10-30. Decision: [0029](../../spec/decisions/0029-physics-simulation-backend.md), delivery item 1.
Code: [`backends/ros2/`](../../backends/ros2/README.md), workflow `.github/workflows/ros2-sim.yml`.
Green run (arm motion and camera both pass): <https://github.com/rokorobot/RoboLang/actions/runs/41040481258>
(started by a temporary `push` trigger on the branch; see "Caveats").

## Result

On the official `ros:lyrical` container (Ubuntu 26.04.1, amd64) in GitHub Actions, headless Gazebo Jetty (Gazebo Sim
10.5.0) runs a UR5e under `gz_ros2_control`; one joint trajectory was executed through ROS 2 and the final
`/joint_states` matched the target (max error 1.5e-12 rad, tolerance 0.05); one camera frame was rendered in software
and an ArUco marker in it was detected. Everything ran on `/clock` (simulation time). **The Jazzy/Harmonic fallback
was not needed.**

## Timings (one run, run 41040481258; cold image cache)

| What | Time |
|---|---|
| Image build (apt install of the stack, no layer cache yet) | 339 s |
| Smoke step, whole (Gazebo start to teardown) | 21 s |
| First `/clock` from Gazebo | 5.6 s after launch |
| Controller and `/joint_states` ready | 7.4 s |
| One 6 s (simulated) trajectory | 7.0 s wall, 7.0 s sim (real-time factor ~1.0 with this scene) |
| First camera frame in ROS | 7.8 s (640x480 rgb8, 10 Hz) |

Whole job about 6.5 min cold. The image layers are cached with the GitHub Actions cache from the second run on.

## Software rendering (0029 section 5 risk): usable

Gazebo's ogre2 renderer ran under Xvfb with Mesa llvmpipe. The frame is non-empty (mean 162, std 23), shows the table,
the box and the arm, and `cv2.aruco` detects marker id 7 (DICT_4X4_50) in it. So fiducial-based observations (X6) are
feasible in CI; the pose-sensor fallback is not needed. Notes: a `libEGL warning: Not allowed to force software
rendering` appears and is harmless; antialiasing is forced to off. Marker detection here is informational only. How
accurately its pose can be recovered (intrinsics, lighting, the grey cast on the texture) is phase 3 work.

## Resolved versions (full list in the `versions.txt` artifact)

| Package | Resolved | 0029 section 6 said |
|---|---|---|
| `ros-lyrical-ros-gz` | 3.0.10 | not stated |
| Gazebo Sim (Jetty) | 10.5.0 (`ros-lyrical-gz-sim-vendor` 0.4.6) | Jetty |
| `ros-lyrical-ur-simulation-gz` | 2.5.0-3 | 2.5.0 |
| `ros-lyrical-ur-moveit-config` | **6.1.0** | 6.2.0 |
| `ros-lyrical-moveit` | 2.15.2 | 2.15.2 |
| `ros-lyrical-gz-ros2-control` | **3.0.9** | 4.0.1 |
| `ros-lyrical-robotiq-description` | 0.0.1-4 | 0.0.1 |
| `ros-lyrical-controller-manager` | 6.10.1 | not stated |
| `ros-lyrical-ur-description` | 4.3.1 | not stated |

The two differences are version numbers on the apt index at build time, not missing packages. All the package names in
section 6 exist as `ros-lyrical-*` binaries. The Gazebo libraries come as `ros-lyrical-gz-*-vendor` packages.

## What did not work out of the box (and the workaround)

1. **Stock UR controller does not load on Lyrical.** `ur_sim_control.launch.py` starts
   `ur_controllers/ScaledJointTrajectoryController`; the controller manager reports the loader "not found" (it is not in
   the class list). Workaround: use `joint_trajectory_controller` instead.
2. **Controller parameters are not delivered.** With `initial_joint_controller:=joint_trajectory_controller` the
   controller failed with `parameter 'joints' ... length 0`, even with the YAML given as `controllers_file`. Workaround:
   spawn it ourselves with `spawner ... --param-file` (`run_smoke.sh`). The stock launch's own spawner is pointed at
   `joint_state_broadcaster`, so it logs one harmless duplicate-activation error.
3. **The camera is not bridged by the stock launch** (only `/clock`). `run_smoke.sh` starts `ros_gz_bridge` for the image.
4. The stock `ur_controllers.yaml` is saved in the artifact as `stock_ur_controllers.yaml` for reference.

## Not done in phase 1

- **Gripper.** `robotiq_description` 0.0.1 expands with `xacro` on its own (the job records it), but
  `ur_simulation_gz`'s `ur_gz.urdf.xacro` has no gripper hook, and the Robotiq `ros2_control` xacro targets real
  hardware plugins. Composing them needs our own combined xacro, a mimic-joint gripper controller under
  `gz_ros2_control`, and the detachable-joint grasp (X5). Not attempted here, so it is left for phase 2.
- **MoveIt goal.** `ur_moveit_config` and `moveit` install, but the smoke test sends a joint trajectory
  (allowed by the brief). Starting MoveIt reliably in CI was not tried.

## Caveats

- `workflow_dispatch` only works for a workflow that exists on the default branch, so the final workflow file
  (dispatch and nightly only) could not be run before merge. The green run above used the same file plus a temporary
  `push` trigger for the branch; that trigger and nothing else was removed. First dispatch after merge is the real check.
- One green run, not a flake rate. The nightly schedule will build that.
- amd64 only (that is the runner); this says nothing about arm64.

## Open risks for phase 2

1. Controller parameter delivery (item 2) looks like a Lyrical `ros2_control` / `ur_controllers` packaging gap. If the
   executor launches its own graph, it must pass `--param-file` too. Worth an upstream issue.
2. Gripper composition and grasp attach (X5) are untested; they are the biggest unknown left in the stack.
3. MoveIt 2 with `ur_moveit_config` 6.1.0 against the simulated arm is untested; the `joint_trajectory_controller`
   name the MoveIt config expects must match the one used here.
4. Real-time factor: the scene was ~1.0 with a static camera at 10 Hz. A grasp scene with contacts may be slower; the
   executor must wait on `/clock`, never on wall time.
5. Packages move: nightly images float on the Lyrical apt index (the versions artifact records what each run used).
   Pinning is a phase 2 decision.
6. Platform: no arm64 verification; contributors on Apple silicon or this sandbox need CI.
