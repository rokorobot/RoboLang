#!/bin/sh
# Print resolved versions of the stack (installed packages), for the job log and the versions artifact.
. /etc/os-release
echo "os: $PRETTY_NAME"
echo "arch: $(dpkg --print-architecture)"
echo "python: $(python3 --version)"
echo "opencv: $(python3 -c 'import cv2; print(cv2.__version__)')"
echo "--- packages"
dpkg-query -W -f='${Package} ${Version}\n' \
    'ros-lyrical-ros-gz' 'ros-lyrical-ros-gz-sim' 'ros-lyrical-ur-simulation-gz' 'ros-lyrical-ur-moveit-config' \
    'ros-lyrical-moveit' 'ros-lyrical-gz-ros2-control' 'ros-lyrical-robotiq-description' \
    'ros-lyrical-ur-description' 'ros-lyrical-ros2-control' 'ros-lyrical-controller-manager' \
    'ros-lyrical-ros-base' 'libgz-sim*' 'libgz-sensors*' 'libgz-rendering*' 'libgz-physics*' 'libgz-transport*' \
    'libgl1-mesa-dri' 2>/dev/null | sort
echo "--- gz"
gz sim --versions 2>&1 | head -3
