# robolang-ros2 backend: phase 1 harness (decision 0029)

Simulation only. This directory is **not** part of `pip install robolang` (`pyproject.toml` lists no package from
here, and `robolang` has no dependency on ROS 2). Nothing here can address real hardware (0029 X7): the only robot is
the Gazebo model spawned by `ur_simulation_gz`.

Phase 1 is the harness only: a container, a scene, and a smoke test that moves the arm once and reads one camera frame.
There is no executor and no RoboLang code yet. Findings: [`docs/evidence/h3-phase1-harness.md`](../../docs/evidence/h3-phase1-harness.md).

| Path | What |
|---|---|
| `Dockerfile` | `FROM ros:lyrical`; installs `ros-lyrical-ros-gz` (Gazebo Jetty), `ur-simulation-gz`, `ur-moveit-config`, `moveit`, `gz-ros2-control`, `robotiq-description`, Xvfb and Mesa software GL, OpenCV |
| `world/h3_world.sdf` | table, shelf, a box with an ArUco marker (DICT_4X4_50, id 7; the PNG is generated at image build), a fixed overhead camera |
| `config/controllers.yaml` | `joint_state_broadcaster` and `joint_trajectory_controller` for the UR5e (the stock Lyrical config does not load, see the findings) |
| `scripts/run_smoke.sh` | starts headless Gazebo + the arm + a camera bridge, runs the smoke test, stops everything |
| `scripts/smoke.py` | rclpy, simulation time only: waits for `/clock` and the controller, sends one trajectory, checks `/joint_states`, reads one camera frame |
| `scripts/versions.sh` | resolved package versions |

## Run it

CI: the workflow `ros2-sim.yml` (on demand and nightly, never on pull requests) builds the image, runs the smoke test
and uploads `smoke.json`, `versions.txt`, `launch.log` and `camera.png` as the `ros2-sim-results` artifact.

Locally, on a machine with Docker (amd64; the sandbox this was written in has none):

```sh
docker build backends/ros2 -t robolang-ros2-harness
mkdir -p out && docker run --rm -v "$PWD/out:/out" robolang-ros2-harness /opt/h3/scripts/run_smoke.sh
```

The smoke test exits non-zero if the arm does not reach the target. The camera result is reported in `smoke.json`
either way.

## Gripper

Not attached in phase 1. `robotiq_description` expands on its own (`gripper_probe.txt` in the artifact), but
`ur_simulation_gz`'s xacro has no gripper hook, so attaching it means our own combined description plus a gripper
controller. That is phase 2 work; see the findings.
