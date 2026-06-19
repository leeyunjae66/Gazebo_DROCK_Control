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
find_package(trajectory_msgs REQUIRED) # <--- 1. trajectory_msgs 패키지 의존성 추가

add_executable(joy_rpm_control_node
  src/joy_rpm_control_node.cpp
)

ament_target_dependencies(joy_rpm_control_node
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
)

add_executable(flipper_joy_control_node
  src/flipper_joy_control_node.cpp
)

# flipper_joy_control_node가 컴파일될 때 trajectory_msgs를 링킹하도록 추가합니다.
ament_target_dependencies(flipper_joy_control_node
  rclcpp
  sensor_msgs
  std_msgs
  trajectory_msgs # <--- 2. flipper_joy_control_node에 의존성 주입
)

install(TARGETS
  joy_rpm_control_node
  flipper_joy_control_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
