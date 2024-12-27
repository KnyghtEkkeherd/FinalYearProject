#ifndef ICP_H
#define ICP_H
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <Eigen3/Eigen/Dense>
#include <pcl/registration/icp.h>
#include <pcl/filters/random_sample.h>

#define MAX_ITERATIONS 100
#define MAX_DISTANCE 0.5

Eigen::Matrix4d icp_registration(pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud,
                                 pcl::PointCloud<pcl::PointXYZ>::Ptr tar_cloud,
                                 Eigen::Matrix4d init_guess);

#endif
