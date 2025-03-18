# Use ROS 2 Humble base image
FROM ros:humble

# Enable universe repository and install system dependencies
RUN sed -i 's/^#\s*\(deb.*universe\)$/\1/g' /etc/apt/sources.list && \
    apt-get update && apt-get install -y \
    git \
    python3-pip \
    python3-colcon-meson \
    libboost-dev \
    libboost-python1.74.0 \  # Explicitly install Boost.Python 1.74
    libgnutls28-dev \
    libopencv-dev \          # OpenCV dependency
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

# Clone image_common (contains camera_info_manager)
RUN git clone https://github.com/ros-perception/image_common.git

# Clone vision_opencv (contains cv_bridge)
RUN git clone -b humble https://github.com/ros-perception/vision_opencv.git

# Resolve dependencies and build
WORKDIR /ros_ws
RUN rosdep update &&
    rosdep install -y --from-paths src --ignore-src --rosdistro humble --skip-keys=libcamera

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