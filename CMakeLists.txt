cmake_minimum_required(VERSION 3.5)
project(reef_control)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(reef_msgs REQUIRED)
find_package(rosflight_msgs REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
  src/pid_node.cpp
  src/controller.cpp
  src/PID.cpp
  src/simple_pid.cpp
)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  geometry_msgs
  reef_msgs
  rosflight_msgs
)

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
