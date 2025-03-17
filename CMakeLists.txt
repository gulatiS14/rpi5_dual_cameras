cmake_minimum_required(VERSION 3.5)
project(ros2_dual_camera)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(camera_ros REQUIRED)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(
  PROGRAMS
    src/dual_camera_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
