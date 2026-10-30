from setuptools import setup

setup(
    name="robolang_ros2",
    version="0.1.0",
    packages=["robolang_ros2"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/robolang_ros2"]),
        ("share/robolang_ros2", ["package.xml"]),
    ],
    install_requires=["setuptools", "robolang>=0.2.0"],
    zip_safe=True,
    maintainer="Robert Konecny",
    maintainer_email="rokorobot@users.noreply.github.com",
    description="Simulation-only ROS 2 backend for RoboLang",
    license="Apache-2.0",
    entry_points={"console_scripts": ["robolang-ros2 = robolang_ros2.cli:main"]},
)
