#!/bin/bash
# Start the UR5e + scene in headless Gazebo (software rendering), run the smoke test, stop everything.
# Simulation only: this launches the Gazebo model; there is no code path to a real controller.
OUT=${OUT:-/out}
mkdir -p "$OUT"
source /opt/ros/lyrical/setup.bash

cat /opt/ros/lyrical/share/ur_simulation_gz/config/ur_controllers.yaml >"$OUT/stock_ur_controllers.yaml"
/opt/h3/scripts/versions.sh | tee "$OUT/versions.txt"

# Informational: does the Robotiq 2F-85 description expand? (Phase 1 does not attach it; see the findings doc.)
{ xacro /opt/ros/lyrical/share/robotiq_description/urdf/robotiq_2f_85_gripper.urdf.xacro >/dev/null \
    && echo "robotiq xacro: expands" || echo "robotiq xacro: FAILED"; } 2>&1 | tee "$OUT/gripper_probe.txt"

xvfb-run -a -s "-screen 0 1280x1024x24" setsid ros2 launch ur_simulation_gz ur_sim_control.launch.py \
    ur_type:=ur5e initial_joint_controller:=joint_trajectory_controller controllers_file:=/opt/h3/config/controllers.yaml gazebo_gui:=false launch_rviz:=false world_file:=/opt/h3/world/h3_world.sdf \
    >"$OUT/launch.log" 2>&1 &
LAUNCH=$!
python3 /opt/h3/scripts/smoke.py --out "$OUT"
RC=$?
kill -INT "$LAUNCH" 2>/dev/null; sleep 3; pkill -TERM -f "gz sim" 2>/dev/null
tail -n 60 "$OUT/launch.log"
exit $RC
