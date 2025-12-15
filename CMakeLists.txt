cmake_minimum_required(VERSION 3.8)
project(gimbal_demo)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_executable(gimbal_teleop_node src/gimbal_teleop_node.cpp)
ament_target_dependencies(gimbal_teleop_node
  rclcpp
  trajectory_msgs
)

install(TARGETS
  gimbal_teleop_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY urdf launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
