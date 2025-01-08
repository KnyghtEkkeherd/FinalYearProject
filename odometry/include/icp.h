#ifndef ICP_H
#define ICP_H
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <Eigen/Dense>
#include <pcl/registration/icp.h>
#include <pcl/filters/random_sample.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <mutex>
#include <queue>

#define MAX_ITERATIONS 100
#define MAX_DISTANCE 0.5

Eigen::Matrix4d icp_registration(pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud,
                                 pcl::PointCloud<pcl::PointXYZ>::Ptr tar_cloud,
                                 Eigen::Matrix4d init_guess);

void laserScan2PointCloud(sensor_msgs::msg::LaserScan input_scan, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
std::queue<sensor_msgs::msg::LaserScan::SharedPtr> laserScanQueue;
std::mutex laserScanQueueMutex;


#endif
