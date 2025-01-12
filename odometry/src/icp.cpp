#include "icp.h"
#include <cstddef>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class ICP : public rclcpp::Node
{
  public:
    ICP()
    : Node("icpOdomNode")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&ICP::laser_scan_enqueue, this, std::placeholders::_1));

        publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/icp_odom", 10);

        // 4Hz timer
        timer_ = this->create_wall_timer(std::chrono::milliseconds(250), std::bind(&ICP::icp, this));
        is_ready = false;
        transformation = Eigen::Matrix4d::Identity();
        init_guess = Eigen::Matrix4d::Identity();
        message = nav_msgs::msg::Odometry();
        past_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());

    }

  private:
    bool is_ready;
    pcl::PointCloud<pcl::PointXYZ>::Ptr past_cloud;
    Eigen::Matrix4d transformation;
    Eigen::Matrix4d init_guess;
    nav_msgs::msg::Odometry message;
    sensor_msgs::msg::LaserScan::SharedPtr scan_in;

    void laser_scan_enqueue(const sensor_msgs::msg::LaserScan::SharedPtr input_scan){
        laserScanQueueMutex.lock();
        laserScanQueue.push(input_scan);
        laserScanQueueMutex.unlock();
    }

    sensor_msgs::msg::LaserScan::SharedPtr laser_scan_dequeue() {
        sensor_msgs::msg::LaserScan::SharedPtr output_scan;

        laserScanQueueMutex.lock();
        if(!laserScanQueue.empty()){
            output_scan = laserScanQueue.front();
            laserScanQueue.pop();
            is_ready = true;
        }
        else{
            laserScanQueueMutex.unlock();
            is_ready = false;
        }
        laserScanQueueMutex.unlock();
        return output_scan;
    }

    void icp(){
        pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        scan_in = laser_scan_dequeue();

        // Handle no scan data
        if(!is_ready){
            RCLCPP_INFO(this->get_logger(), "Waiting for laser scan data...");
            return;
        }

        // Convert LaserScan to PointCloud
        laserScan2PointCloud(scan_in, current_cloud);

        // Handle the first scan
        if(past_cloud->empty()){
            *past_cloud = *current_cloud;
            RCLCPP_INFO(this->get_logger(), "First scan received.");
            return;
        }

        transformation = icp_registration(current_cloud, past_cloud, init_guess);
        message.pose.pose.position.x = transformation(0, 3);
        message.pose.pose.position.y = transformation(1, 3);
        message.pose.pose.position.z = transformation(2, 3);
        Eigen::Quaterniond q(transformation.block<3, 3>(0, 0));
        message.pose.pose.orientation.x = q.x();
        message.pose.pose.orientation.y = q.y();
        message.pose.pose.orientation.z = q.z();
        message.pose.pose.orientation.w = q.w();
        message.header.stamp = this->now();
        message.header.frame_id = "odom";
        RCLCPP_INFO(this->get_logger(), "Publishing header frame_id: '%s'", message.header.frame_id.c_str());
        publisher_->publish(message);
        *past_cloud = *current_cloud;
    }

    void laserScan2PointCloud(sensor_msgs::msg::LaserScan::SharedPtr msg, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out) const {
        // Conversion logic from LaserScan to pcl::PointCloud<pcl::PointXYZ>
        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            float range = msg->ranges[i];
            if (std::isinf(range) || range < msg->range_min || range > msg->range_max) {
                continue; // Skip points with infinite range or out of range bounds
            }
            float angle = msg->angle_min + i * msg->angle_increment;

            float x = range * std::cos(angle);
            float y = range * std::sin(angle);
            float z = 0.0;

            pcl::PointXYZ point;
            point.x = x;
            point.y = y;
            point.z = z;

            cloud_out->points.push_back(point);
        }

        cloud_out->width = cloud_out->points.size();
        cloud_out->height = 1;
        cloud_out->is_dense = false;
    }

    Eigen::Matrix4d icp_registration(pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr tar_cloud, Eigen::Matrix4d init_guess) {

        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        if (src_cloud->empty() || tar_cloud == nullptr || tar_cloud->empty()) {
            RCLCPP_ERROR(this->get_logger(), "Source or target cloud is empty or null. Empty: tar: %d, src: %d", tar_cloud->empty(), src_cloud->empty());
            return Eigen::Matrix4d::Identity();
        }

        icp.setInputSource(src_cloud);
        icp.setInputTarget(tar_cloud);
        icp.setMaximumIterations(100); // Assuming MAX_ITERATIONS is 50
        icp.setTransformationEpsilon(1e-6);
        icp.setMaxCorrespondenceDistance(0.05); // Assuming MAX_DISTANCE is 0.05
        pcl::PointCloud<pcl::PointXYZ> aligned_cloud;
        icp.align(aligned_cloud, init_guess.cast<float>());
        Eigen::Matrix4d transformation = icp.getFinalTransformation().cast<double>();

        return transformation;
    }

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]){
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ICP initialized");
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ICP>());
    rclcpp::shutdown();
    return 0;
}
