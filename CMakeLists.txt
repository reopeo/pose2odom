cmake_minimum_required(VERSION 3.8)
project(pose2odom)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(pose2odom
  src/main.cpp
  src/pose2odom.cpp
)

ament_target_dependencies(pose2odom
  rclcpp
  tf2
  tf2_geometry_msgs
  nav_msgs
  geometry_msgs
)

ament_package()
