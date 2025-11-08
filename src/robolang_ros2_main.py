# src/robolang_ros2_main.py
"""
RoboLang ROS2 Runtime Entrypoint

This script runs RoboLang tasks (.rob files) using the ROS2-based adapter.
Usage (after building in a ROS2 workspace):

    ros2 run robolang ros2_runtime examples/pick_and_place.rob

It initializes ROS2, loads the task, and executes it through RobotAdapterROS2.
"""

from __future__ import annotations
import sys
import rclpy

from robolang_runtime import RoboLangRuntime
from robolang_adapter_ros2 import RobotAdapterROS2


def main(args=None):
    if len(sys.argv) < 2:
        print("Usage: ros2 run robolang ros2_runtime <file.rob>")
        sys.exit(1)

    robofile = sys.argv[1]

    # Initialize ROS2
    rclpy.init(args=args)

    adapter = RobotAdapterROS2("robot_arm_1")
    adapter.wait_for_servers()

    runtime = RoboLangRuntime(adapter)
    runtime.execute_file(robofile)

    adapter.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
