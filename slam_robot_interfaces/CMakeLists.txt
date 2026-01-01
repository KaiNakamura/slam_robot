cmake_minimum_required(VERSION 3.8)
project(slam_robot_interfaces)

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Frontier.msg"
  "msg/FrontierList.msg"
  DEPENDENCIES geometry_msgs
)

# Export dependencies
ament_export_dependencies(rosidl_default_runtime)

ament_package()

