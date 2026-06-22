cmake_minimum_required(VERSION 3.8)
project(gazebo_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

# Gazebo Classic transport 사용
find_package(gazebo REQUIRED)
find_package(TBB REQUIRED)

include_directories(
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

add_definitions(
  ${GAZEBO_CXX_FLAGS}
)

# ============================================================
# Flipper joystick -> JointTrajectoryController
# ============================================================
add_executable(flipper_joy_control_node
  src/flipper_joy_control_node.cpp
)

ament_target_dependencies(flipper_joy_control_node
  rclcpp
  sensor_msgs
  std_msgs
  trajectory_msgs
)

# ============================================================
# /cmd_vel -> visual roller + SimpleTrackedVehiclePlugin
# ============================================================
add_executable(cmd_vel_track_control_node
  src/cmd_vel_track_control_node.cpp
)

ament_target_dependencies(cmd_vel_track_control_node
  rclcpp
  std_msgs
  geometry_msgs
)

target_link_libraries(cmd_vel_track_control_node
  ${GAZEBO_LIBRARIES}
  TBB::tbb
)

# ============================================================
# Install executables
# ============================================================
install(TARGETS
  flipper_joy_control_node
  cmd_vel_track_control_node
  DESTINATION lib/${PROJECT_NAME}
)

# ============================================================
# Install config files
# ============================================================
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
