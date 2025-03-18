# Use ROS 2 Humble base image
FROM ros:humble

# Install system dependencies
RUN apt-get update && apt-get install -y \
    git \
    python3-pip \
    python3-colcon-meson \
    libboost-dev \
    libgnutls28-dev \
    libevent-dev \
    libssl-dev \
    libyaml-dev \
    libdw-dev \
    && rm -rf /var/lib/apt/lists/*

# Create workspace
WORKDIR /ros_ws/src

# Clone Raspberry Pi's libcamera fork (for Pi Camera support)
RUN git clone https://github.com/raspberrypi/libcamera.git

# Clone camera_ros package
RUN git clone https://github.com/christianrauch/camera_ros.git

# Resolve dependencies and build
WORKDIR /ros_ws
RUN rosdep init && rosdep update
RUN rosdep install -y --from-paths src --ignore-src --rosdistro humble --skip-keys=libcamera
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build --event-handlers=console_direct+

# Copy the dual_camera_node.py into the container
COPY dual_camera_node.py /ros_ws/src/camera_ros/scripts/

# Copy the ros_entrypoint.sh into the container
COPY ros_entrypoint.sh /ros_ws/

# Make the entrypoint script executable
RUN chmod +x /ros_ws/ros_entrypoint.sh

# Set the entrypoint to use your script
ENTRYPOINT ["/ros_ws/ros_entrypoint.sh"]