cmake_minimum_required(VERSION 3.8)
project(localisation_planning)

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
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pcl_ros REQUIRED)

# Talker node
add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(talker rclcpp std_msgs)

# Listener node
add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp std_msgs)

# ICP node
add_executable(icp src/icp.cpp)
ament_target_dependencies(icp rclcpp std_msgs Eigen3 pcl_ros)
ament_export_dependencies(eigen3_cmake_module)
ament_export_dependencies(Eigen3)

install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME})

INCLUDE_DIRECTORIES(
  include
)

ament_package()
