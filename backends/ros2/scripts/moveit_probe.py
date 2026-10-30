"""Informational phase 2 integration probe while the phase 1 Gazebo graph is alive."""
import json
import sys
import traceback
from pathlib import Path

import rclpy
from moveit.planning import MoveItPy
from robolang_ros2.ros import moveit_parameters

result = {}
try:
    rclpy.init()
    parameters = moveit_parameters(Path("/out/combined.urdf").read_text())
    result["controller_names"] = parameters.get("moveit_simple_controller_manager", {}).get("controller_names")
    result["pipeline_parameters"] = parameters["planning_pipelines"]
    robot = MoveItPy(node_name="phase2_moveit_probe", config_dict=parameters)
    arm = robot.get_planning_component("ur_manipulator")
    arm.set_start_state_to_current_state()
    result["set_home"] = arm.set_goal_state(configuration_name="home")
    plan = arm.plan()
    result["planned"] = bool(plan)
    if plan:
        status = robot.execute(plan.trajectory, controllers=["joint_trajectory_controller"])
        result["execution_status"] = status.status
except Exception as exc:
    result["error"] = repr(exc)
    result["traceback"] = traceback.format_exc()
finally:
    Path("/out/moveit_probe.json").write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(0)  # informational until the executor integration is wired
