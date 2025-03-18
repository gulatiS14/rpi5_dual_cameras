# Use ROS 2 Humble base image for ARM64
FROM ros:humble-ros-core

# Install Raspberry Pi OS dependencies
RUN apt-get update && apt-get install -y \
    git \
    python3-pip \
    python3-colcon-meson \
    python3-colcon-common-extensions \
    libboost-dev \
    libboost-python-dev \
    libgnutls28-dev \
    libopencv-dev \
    libevent-dev \
    libssl-dev \
    libyaml-dev \
    libdw-dev \
    libcamera-dev \  
    libcamera-apps-dev \
    v4l-utils \
    && rm -rf /var/lib/apt/lists/*

# Create workspace
WORKDIR /ros_ws/src

# Clone camera_ros package
RUN git clone https://github.com/christianrauch/camera_ros.git

# Clone image_common (for camera_info_manager)
RUN git clone https://github.com/ros-perception/image_common.git

# Clone vision_opencv (for cv_bridge)
RUN git clone -b humble https://github.com/ros-perception/vision_opencv.git

# Resolve dependencies and build
WORKDIR /ros_ws
RUN rosdep update && \
    rosdep install -y --from-paths src --ignore-src --rosdistro humble \
    --skip-keys="libcamera ament_cmake_clang_format ament_cmake_clang_tidy"

RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build --packages-up-to camera_ros --event-handlers=console_direct+

# Copy your dual_camera_node.py into the container
COPY dual_camera_node.py /ros_ws/src/camera_ros/scripts/

# Copy entrypoint script
COPY ros_entrypoint.sh /ros_ws/
RUN chmod +x /ros_ws/ros_entrypoint.sh

# Grant access to Raspberry Pi camera hardware
ENV LD_LIBRARY_PATH=/usr/lib/arm-linux-gnueabihf
ENTRYPOINT ["/ros_ws/ros_entrypoint.sh"]