#!/bin/bash
# One H3 acceptance run (decision 0029 section 3): generate SCENE, start headless Gazebo, execute the scene's
# unchanged reference .rob through the backend, then compare its verdict with the test-only oracle.
# Simulation only: this launches the Gazebo model; there is no code path to a real controller.
# The exit code is 0 when the harness ran; the run's outcome is in the record and judged by acceptance.py.
SCENE=$1
OUT=${OUT:-/out}
S=/tmp/scene
mkdir -p "$OUT"
source /opt/ros/lyrical/setup.bash

python3 /opt/h3/scripts/scenes.py "$SCENE" "$S" || exit 1
cp "$S/scene.json" "$S/site.json" "$S/world.sdf" "$OUT/"
/opt/h3/scripts/versions.sh >"$OUT/versions.txt"
xacro "$S/robot.urdf.xacro" ur_type:=ur5e simulation_controllers:=/opt/h3/config/controllers.yaml \
    >"$OUT/combined.urdf" || exit 1

xvfb-run -a -s "-screen 0 1280x1024x24" setsid ros2 launch ur_simulation_gz ur_sim_control.launch.py \
    ur_type:=ur5e initial_joint_controller:=joint_state_broadcaster controllers_file:=/opt/h3/config/controllers.yaml description_file:="$S/robot.urdf.xacro" gazebo_gui:=false launch_rviz:=false world_file:="$S/world.sdf" \
    >"$OUT/launch.log" 2>&1 &
LAUNCH=$!
# On Lyrical the stock launch's spawner does not pass the YAML parameters, so the controllers are spawned here.
ros2 run controller_manager spawner joint_trajectory_controller -c /controller_manager \
    --param-file /opt/h3/config/controllers.yaml --controller-manager-timeout 240 >"$OUT/spawner.log" 2>&1 &
SPAWNER=$!
ros2 run controller_manager spawner gripper_trajectory_controller -c /controller_manager \
    --param-file /opt/h3/config/controllers.yaml --controller-manager-timeout 240 >"$OUT/gripper_spawner.log" 2>&1 &
GRIPPER_SPAWNER=$!
ros2 run ros_gz_bridge parameter_bridge /overhead_camera/image@sensor_msgs/msg/Image[gz.msgs.Image \
    /side_camera/image@sensor_msgs/msg/Image[gz.msgs.Image >"$OUT/camera_bridge.log" 2>&1 &
BRIDGE=$!
stop() {
    kill -INT "$LAUNCH" "$SPAWNER" "$GRIPPER_SPAWNER" "$BRIDGE" 2>/dev/null || true
    sleep 2
}
wait "$SPAWNER" || { cat "$OUT/spawner.log" >&2; stop; exit 1; }
wait "$GRIPPER_SPAWNER" || { cat "$OUT/gripper_spawner.log" >&2; stop; exit 1; }
# This Jetty build starts every detachable joint attached. Release each box before anything moves.
python3 - "$S/scene.json" >"$OUT/initial_detach.log" 2>&1 <<'PY' || { cat "$OUT/initial_detach.log" >&2; stop; exit 1; }
import json, sys
from robolang_ros2.ros import gazebo_joint_command
for model in json.load(open(sys.argv[1]))["box_models"]:
    gazebo_joint_command("ur", model, "detach")
PY

mapfile -t ARGS < <(python3 -c 'import json,sys; s=json.load(open(sys.argv[1]))
print(s["source"]); print("--task"); print(s["task"])
for k, v in s["bindings"].items(): print("--bind"); print(f"{k}={v}")' "$S/scene.json")
start=$(date +%s)
timeout 900 python3 -m robolang_ros2.cli "${ARGS[@]}" --site "$S/site.json" --out "$OUT/executor.json" \
    >"$OUT/executor.log" 2>&1
echo "$?" >"$OUT/exit_code"
echo "$(( $(date +%s) - start ))" >"$OUT/wall_seconds"
if [ -f "$OUT/executor.json" ]; then
    python3 /opt/h3/scripts/oracle.py "$OUT/executor.json" "$S/site.json" "$OUT/oracle.json" >"$OUT/oracle.log" 2>&1
fi
# One frame per camera, for the evidence document.
python3 /opt/h3/scripts/snapshot.py "$OUT" >"$OUT/snapshot.log" 2>&1 || true
stop
tail -n 40 "$OUT/launch.log" >"$OUT/launch_tail.log"
exit 0
