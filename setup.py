from setuptools import setup

package_name = "robolang"

setup(
    name=package_name,
    version="1.0.0",
    description="RoboLang v1 — AI-readable DSL and runtime for robots using ROS2.",
    author="Robert Konecny",
    author_email="",
    url="https://github.com/rokorobot/RoboLang",
    # Our Python modules live directly in src/
    package_dir={"": "src"},
    py_modules=[
        "robolang_adapter",
        "robolang_adapter_ros2",
        "robolang_runtime",
        "robolang_ros2_main",
    ],
    install_requires=[
        "rclpy",
    ],
    python_requires=">=3.11",
    entry_points={
        "console_scripts": [
            # This will be used by ROS2: `ros2 run robolang ros2_runtime <file.rob>`
            "ros2_runtime = robolang_ros2_main:main",
        ],
    },
)
