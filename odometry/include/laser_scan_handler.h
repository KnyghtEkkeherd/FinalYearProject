#ifndef LASER_SCAN_HANDLER_H
#define LASER_SCAN_HANDLER_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <cmath>
#include <queue>
#include <mutex>

std::queue<sensor_msgs::msg::LaserScan::SharedPtr> laserScanQueue;
std::mutex laserScanQueueMutex;

#endif
