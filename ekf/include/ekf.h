#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <queue>
#include <mutex>
#include <pcl/registration/icp.h>
#include <pcl/filters/random_sample.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

class EKFSLAMNode : public rclcpp::Node
{
public:
    EKFSLAMNode();

private:
    void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr point_cloud_scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

    std::queue<pcl::PointCloud<pcl::PointXYZ>::Ptr> pointCloudQueue;
    std::mutex pointCloudQueueMutex;

};
