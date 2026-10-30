"""Informational phase 2 integration probe while the phase 1 Gazebo graph is alive."""
import json
import sys
import traceback
from pathlib import Path

import rclpy
from moveit.planning import MoveItPy
from moveit_configs_utils import MoveItConfigsBuilder

result = {}
try:
    rclpy.init()
    config = (MoveItConfigsBuilder(robot_name="ur", package_name="ur_moveit_config")
              .robot_description_semantic(Path("srdf") / "ur.srdf.xacro", {"name": "ur"})
              .to_moveit_configs())
    result["controller_names"] = config.to_dict().get("moveit_simple_controller_manager", {}).get("controller_names")
    parameters = config.to_dict()
    parameters["planning_pipelines"] = {"pipeline_names": ["ompl"]}
    parameters["plan_request_params"] = {"planning_attempts": 1, "planning_pipeline": "ompl",
                                         "planning_time": 5.0}
    parameters["use_sim_time"] = True
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
