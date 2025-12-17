cmake_minimum_required(VERSION 3.8)
project(gimbal_demo)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(teleop src/gimbal_teleop_node.cpp)
ament_target_dependencies(teleop
  rclcpp
  std_msgs
  trajectory_msgs
)

add_executable(tracker src/gimbal_visual_tracker_color_node.cpp)
ament_target_dependencies(tracker
  rclcpp
  trajectory_msgs
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
  OpenCV
)

install(TARGETS
  teleop
  tracker
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY urdf launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
