cmake_minimum_required(VERSION 3.10)
project(ekf)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_msgs REQUIRED)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    find_package(rclcpp REQUIRED)

    # comment the line when a copyright and license is added to all source files
    set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # comment the line when this package is in a git repo and when
    # a copyright and license is added to all source files
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()

# EKF installation
add_executable(ekf src/ekf.cpp)
ament_target_dependencies(ekf rclcpp sensor_msgs nav_msgs visualization_msgs Eigen3 geometry_msgs pcl_ros pcl_conversions pcl_msgs)

install(TARGETS
  ekf
  DESTINATION lib/${PROJECT_NAME})

INCLUDE_DIRECTORIES(
  include
)

ament_package()
