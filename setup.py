"""
Compatibility shim.

All package metadata lives in pyproject.toml. This file remains so that tools
which still invoke setup.py (for example a ROS 2 colcon build) find one.
That path is experimental and has not been tested.
"""

from setuptools import setup

setup()
