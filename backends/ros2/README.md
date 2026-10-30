# RoboLang ROS 2 backend

This optional backend runs a validated `.rob` task with a UR5e, Robotiq 2F-85 gripper, MoveIt 2 and Gazebo Jetty. It is simulation only and is separate from the core `robolang` wheel. The implementation contract and failure codes are in [the backend specification](../../spec/ros2-backend-v0.1.md).

The executor checks for advancing simulation time, the declared Gazebo model and Gazebo hardware plugins before creating motion clients. The site file maps task bindings to poses and Gazebo models. `move` and the approach in `grasp` use MoveIt; a gripper trajectory closes before Gazebo attaches the object. `place` moves with MoveIt, opens the gripper, then detaches it. Attachment is a simulation shortcut and does not validate contact physics.

The phase 1 arm and camera findings are in [the harness report](../../docs/evidence/h3-phase1-harness.md).

## Run the integration test

The on-demand and nightly `ros2-sim.yml` workflow builds the image, runs the arm and camera smoke test, then executes [`transfer.rob`](examples/transfer.rob). It uploads logs and a JSON run record as the `ros2-sim-results` artifact. Each goal condition stays `not_evaluated` until phase 3 adds sensor observations; completed motion steps therefore exit with code 3, not success. [Phase 2 evidence](../../docs/evidence/h3-phase2-executor.md) records the green integration run.

On an amd64 host with Docker:

```sh
docker build -f backends/ros2/Dockerfile . -t robolang-ros2-harness
mkdir -p out
docker run --rm -v "$PWD/out:/out" robolang-ros2-harness /opt/h3/scripts/run_smoke.sh
```

Inside a running ROS 2 simulation graph, the CLI is:

```sh
python3 -m robolang_ros2.cli backends/ros2/examples/transfer.rob --task transfer \
  --site backends/ros2/site/h3_site.json \
  --bind r=arm1 --bind box=box1 --bind pickup=pickup --bind drop=drop \
  --out executor.json
```

The `robolang-ros2` ament package lives in this directory. The Docker harness includes its Python modules and core source directly so the integration workflow exercises the checked out revision.
