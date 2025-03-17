# Use ROS2 Humble as base image
FROM ros:humble

# Install dependencies
RUN apt update && apt install -y \
    python3-colcon-common-extensions \
    python3-colcon-meson \
    git \
    && rm -rf /var/lib/apt/lists/*

# Create workspace and install camera_ros
WORKDIR /camera_ws/src
RUN git clone https://git.libcamera.org/libcamera/libcamera.git && \
    git clone https://github.com/christianrauch/camera_ros.git

# Build workspace
WORKDIR /camera_ws
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    rosdep install -y --from-paths src --ignore-src --rosdistro humble --skip-keys=libcamera && \
    colcon build --event-handlers=console_direct+"

# Source the workspace on container start
RUN echo "source /opt/ros/humble/setup.bash && source /camera_ws/install/setup.bash" >> /root/.bashrc

# Set entrypoint
ENTRYPOINT ["/bin/bash", "-c", "source /root/.bashrc && ros2 launch dual_camera_node.py"]
