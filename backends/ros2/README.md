# RoboLang ROS 2 backend

This optional backend runs a validated `.rob` task with a UR5e, Robotiq 2F-85 gripper, MoveIt 2 and Gazebo Jetty. It is simulation only and is separate from the core `robolang` wheel. The implementation contract and failure codes are in [the backend specification](../../spec/ros2-backend-v0.1.md).

The executor checks for advancing simulation time, the declared Gazebo model and Gazebo hardware plugins before creating motion clients. The site file maps task bindings to poses, Gazebo models, marker IDs, location regions and the camera calibration. `move` and the approach in `grasp` use MoveIt; a gripper trajectory closes before Gazebo attaches the object. `place` moves with MoveIt, opens the gripper, then detaches it. Attachment is a simulation shortcut and does not validate contact physics.

The phase 1 arm and camera findings are in [the harness report](../../docs/evidence/h3-phase1-harness.md); [`examples/transfer.rob`](examples/transfer.rob) is the program the phase 2 and 3 runs used.

## Run the acceptance test

The on-demand and nightly `ros2-sim.yml` workflow is the H3 acceptance run ([0029](../../spec/decisions/0029-physics-simulation-backend.md) sections 3 and 7). [`scripts/scenes.py`](scripts/scenes.py) generates a Gazebo world and site file for each benchmark task, with the task's entity IDs; the reference `.rob` in `bench/reference/` runs unchanged. Each task runs three times, each scripted failure case once, each in a fresh container. Goals are decided from the overhead and side cameras' ArUco markers, the `tool0` transform and the gripper joint state; Gazebo ground truth is read only afterwards by the test oracle. A final job applies the pass bar and uploads `acceptance.json`; each scene's logs, records and camera frames are in its `ros2-acceptance-<scene>` artifact.

On an amd64 host with Docker, one scene:

```sh
docker build -f backends/ros2/Dockerfile . -t robolang-ros2-harness
mkdir -p out
docker run --rm -v "$PWD/out:/out" robolang-ros2-harness /opt/h3/scripts/run_task.sh e_pick_place
python3 backends/ros2/scripts/acceptance.py out acceptance.json   # with all 15 scenes, the full verdict
```

Inside a running ROS 2 simulation graph, the CLI is:

```sh
python3 -m robolang_ros2.cli bench/reference/e_pick_place.rob --task pick_place --site site.json \
  --bind r=arm1 --bind box=box1 --bind src=shelf --bind dst=table --out executor.json
```

`python3 backends/ros2/scripts/scenes.py e_pick_place DIR` writes that scene's `site.json`, `world.sdf` and robot description. The `robolang-ros2` ament package lives in this directory. The Docker harness includes its Python modules and core source directly so the workflow exercises the checked out revision.
