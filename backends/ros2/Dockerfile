# H3 phase 1 harness image (decision record 0029, section 6). SIMULATION ONLY: nothing here can address hardware.
# Build context: backends/ros2/
FROM ros:lyrical

ENV DEBIAN_FRONTEND=noninteractive \
    LIBGL_ALWAYS_SOFTWARE=1 \
    GALLIUM_DRIVER=llvmpipe

RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-lyrical-ros-gz \
        ros-lyrical-ur-simulation-gz \
        ros-lyrical-ur-moveit-config \
        ros-lyrical-moveit \
        ros-lyrical-gz-ros2-control \
        ros-lyrical-robotiq-description \
        ros-lyrical-control-msgs \
        ros-lyrical-xacro \
        ros-lyrical-joint-state-broadcaster \
        ros-lyrical-joint-trajectory-controller \
        xvfb libegl1 libegl-mesa0 libgl1-mesa-dri libglx-mesa0 mesa-utils \
        python3-numpy python3-opencv \
    && rm -rf /var/lib/apt/lists/*

COPY world /opt/h3/world
COPY scripts /opt/h3/scripts
COPY config /opt/h3/config

# Fiducial texture for the box: a real ArUco marker (DICT_4X4_50, id 7), drawn with white quiet zone.
RUN python3 /opt/h3/scripts/make_marker.py /opt/h3/world/marker.png

WORKDIR /opt/h3
