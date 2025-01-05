cmake_minimum_required(VERSION 3.10)
project(odometry)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(EIGEN_ROOT "/Users/wiktorkowalczyk/miniconda3/envs/ros_env/include/eigen3")
set(FLANN_ROOT "/Users/wiktorkowalczyk/miniconda3/envs/ros_env")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# ICP node
add_executable(icp src/icp.cpp)
ament_target_dependencies(icp rclcpp std_msgs Eigen3 pcl_ros nav_msgs sensor_msgs pcl_conversions pcl_msgs)
ament_export_dependencies(eigen3_cmake_module)
ament_export_dependencies(Eigen3)

# Laser scan debug publisher node
add_executable(laser_scan_debug_publisher src/debug_laser_scan_publisher.cpp)
ament_target_dependencies(laser_scan_debug_publisher rclcpp sensor_msgs)

install(TARGETS
  icp
  laser_scan_debug_publisher
  DESTINATION lib/${PROJECT_NAME})

INCLUDE_DIRECTORIES(
  include
)

ament_package()
