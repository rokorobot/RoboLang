#!/bin/bash
# Start the UR5e scene in headless Gazebo, check arm and camera, run the RoboLang transfer, stop everything.
# Simulation only: this launches the Gazebo model; there is no code path to a real controller.
OUT=${OUT:-/out}
mkdir -p "$OUT"
source /opt/ros/lyrical/setup.bash

cat /opt/ros/lyrical/share/ur_simulation_gz/config/ur_controllers.yaml >"$OUT/stock_ur_controllers.yaml"
/opt/h3/scripts/versions.sh | tee "$OUT/versions.txt"
xacro /opt/h3/urdf/ur5e_robotiq_gz.urdf.xacro ur_type:=ur5e \
    simulation_controllers:=/opt/h3/config/controllers.yaml >"$OUT/combined.urdf" || exit 1

xvfb-run -a -s "-screen 0 1280x1024x24" setsid ros2 launch ur_simulation_gz ur_sim_control.launch.py \
    ur_type:=ur5e initial_joint_controller:=joint_state_broadcaster controllers_file:=/opt/h3/config/controllers.yaml description_file:=/opt/h3/urdf/ur5e_robotiq_gz.urdf.xacro gazebo_gui:=false launch_rviz:=false world_file:=/opt/h3/world/h3_world.sdf \
    >"$OUT/launch.log" 2>&1 &
LAUNCH=$!
# On Lyrical the controller manager does not hand the YAML parameters to controllers loaded by the stock launch's
# spawner (joints came out empty), so the trajectory controller is spawned here with --param-file.
ros2 run controller_manager spawner joint_trajectory_controller -c /controller_manager \
    --param-file /opt/h3/config/controllers.yaml --controller-manager-timeout 240 >"$OUT/spawner.log" 2>&1 &
SPAWNER=$!
ros2 run controller_manager spawner gripper_trajectory_controller -c /controller_manager \
    --param-file /opt/h3/config/controllers.yaml --controller-manager-timeout 240 >"$OUT/gripper_spawner.log" 2>&1 &
GRIPPER_SPAWNER=$!
# Camera frames: Gazebo topic -> ROS 2 (the stock launch only bridges /clock).
ros2 run ros_gz_bridge parameter_bridge /overhead_camera/image@sensor_msgs/msg/Image[gz.msgs.Image \
    >"$OUT/camera_bridge.log" 2>&1 &
BRIDGE=$!
wait "$SPAWNER" || { cat "$OUT/spawner.log" >&2; exit 1; }
wait "$GRIPPER_SPAWNER" || { cat "$OUT/gripper_spawner.log" >&2; exit 1; }
# This Jetty build starts DetachableJoint attached even when initial_attach=false. Release it before arm motion.
gz topic -t /model/ur/detachable_joint/detach -m gz.msgs.Empty -p '' || exit 1
sleep 1
python3 /opt/h3/scripts/smoke.py --out "$OUT"
RC=$?
gz model --list >"$OUT/models.txt" 2>&1 || true
gz model -m ur --link >"$OUT/robot_links.txt" 2>&1 || true
ros2 control list_hardware_components -c /controller_manager >"$OUT/hardware.txt" 2>&1 || true
ros2 control list_controllers -c /controller_manager >"$OUT/controllers.txt" 2>&1 || true
timeout 5 ros2 run tf2_ros tf2_echo base_link tool0 >"$OUT/tool0_tf.txt" 2>&1 || true
# Phase 2: execute the validated .rob through MoveIt, gripper controller and Gazebo detachable joint.
timeout 180 python3 -m robolang_ros2.cli /opt/h3/examples/transfer.rob --task transfer \
    --site /opt/h3/site/h3_site.json --bind r=arm1 --bind box=box1 --bind pickup=pickup --bind drop=drop \
    --out "$OUT/executor.json" >"$OUT/executor.log" 2>&1
EXEC_RC=$?
if [ "$EXEC_RC" -ne 3 ]; then
    echo "executor exit $EXEC_RC (expected 3: steps completed, goal awaiting observations)" >&2
    RC=1
else
    python3 - "$OUT/executor.json" <<'PY_RECORD'
import json
import sys
record = json.load(open(sys.argv[1]))
assert record["state"] == "UNKNOWN" and len(record["steps"]) == 4
assert all(step["status"] == "succeeded" for step in record["steps"])
PY_RECORD
    [ $? -eq 0 ] || RC=1
fi
# Gazebo-side view of the camera, for the findings doc (header of one frame only).
timeout 20 gz topic -e -t /overhead_camera/image -n 1 2>&1 | head -c 400 >"$OUT/gz_camera_topic.txt"
kill -INT "$LAUNCH" "$SPAWNER" "$GRIPPER_SPAWNER" "$BRIDGE" 2>/dev/null || true
sleep 2
tail -n 60 "$OUT/launch.log"
exit $RC
