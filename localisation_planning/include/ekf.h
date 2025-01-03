#ifndef EKF_H
#define EKF_H

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <Eigen3/Eigen/Dense>
#include <pcl/filters/random_sample.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <mutex>
#include <queue>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>


double normalizeAngle(double angle);

Eigen::MatrixXd jacobGt(const Eigen::VectorXd& state, Eigen::Vector2d ut, double dt);

Eigen::MatrixXd jacobFt(const Eigen::VectorXd& state, Eigen::Vector2d ut, double dt);

Eigen::MatrixXd jacobB(const Eigen::VectorXd& state, Eigen::Vector2d ut, double dt);

void predictState(Eigen::VectorXd& state, Eigen::MatrixXd& cov, Eigen::Vector2d ut, double dt);

Eigen::Vector2d transform(const Eigen::Vector2d& p, const Eigen::Vector3d& x);

std::queue<sensor_msgs::msg::PointCloud2::SharedPtr> pointCloudQueue;
std::mutex pointCloudQueueMutex;

#endif
