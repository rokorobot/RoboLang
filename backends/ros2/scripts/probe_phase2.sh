#!/bin/bash
set -e
source /opt/ros/lyrical/setup.bash
{
  echo '=== moveit launch ==='
  ros2 launch ur_moveit_config ur_moveit.launch.py --show-args
  echo '=== robot launch ==='
  ros2 launch ur_simulation_gz ur_sim_control.launch.py --show-args
  echo '=== configs ==='
  cat /opt/ros/lyrical/share/ur_moveit_config/config/moveit_controllers.yaml
  echo '=== robot xacro ==='
  cat /opt/ros/lyrical/share/ur_simulation_gz/urdf/ur_gz.urdf.xacro
  echo '=== gripper files ==='
  find /opt/ros/lyrical/share/robotiq_description -maxdepth 3 -type f | sort
  echo '=== gripper xacro ==='
  cat /opt/ros/lyrical/share/robotiq_description/urdf/robotiq_2f_85_gripper.urdf.xacro
  echo '=== gripper macro ==='
  cat /opt/ros/lyrical/share/robotiq_description/urdf/robotiq_2f_85_macro.urdf.xacro
  echo '=== gripper control macro ==='
  cat /opt/ros/lyrical/share/robotiq_description/urdf/robotiq_gripper.ros2_control.xacro
} > /out/phase2_probe.txt 2>&1
