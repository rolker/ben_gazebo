cmake_minimum_required(VERSION 3.8)
project(ben_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(marine_interfaces REQUIRED)

add_executable(gazebo_helm src/gazebo_helm_node.cpp)
ament_target_dependencies(gazebo_helm
  rclcpp
  rclcpp_lifecycle
  control_toolbox
  geometry_msgs
  nav_msgs
  std_msgs
  marine_interfaces
)

install(TARGETS gazebo_helm
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  urdf
  launch
  worlds
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
