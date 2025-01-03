#include "ekf.h"

EKFSLAMNode::EKFSLAMNode() : Node("ekf_slam_node")
{
    point_cloud_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/pointCloud", 10, std::bind(&EKFSLAMNode::laserScanCallback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&EKFSLAMNode::odomCallback, this, std::placeholders::_1));
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/pose", 10);
}

void EKFSLAMNode::laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg){
    // Process the laser scan data and perform EKF SLAM
    // ...

    // Publish the estimated pose
    auto pose_msg = geometry_msgs::msg::PoseStamped();
    pose_msg.header.stamp = this->now();
    pose_msg.header.frame_id = "map";
    // Fill in pose_msg with the estimated pose
    // ...
    pose_pub_->publish(pose_msg);
}

void EKFSLAMNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg){
    // Process the odometry data
    // ...
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EKFSLAMNode>());
    rclcpp::shutdown();
    return 0;
}
