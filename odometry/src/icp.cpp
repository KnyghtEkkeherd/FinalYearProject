#include "icp.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/passthrough.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/path.hpp>

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class ICP : public rclcpp::Node
{
  public:
    ICP():Node("icpOdomNode"){
        // 4Hz timer
        timer_ = this->create_wall_timer(std::chrono::milliseconds(250), std::bind(&ICP::icp, this));

        /* Code added from the lab */
        Twb = Eigen::Matrix4d::Identity();  // initial pose
        laserCloudIn.reset(new pcl::PointCloud<pcl::PointXYZ>);
        refCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);

        // Initialize the ROS publisher
        lidar_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
                    "/scan", 1, std::bind(&ICP::laser_scan_enqueue, this, std::placeholders::_1));

        odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("icp_odom", 1);
        path_pub = this->create_publisher<nav_msgs::msg::Path>("icp_path", 1);
        scan_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("current_scan", 1);
        map_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_map", 1);

        firstFrame = true;

        RCLCPP_INFO(this->get_logger(), "Odometry ICP initialized");
    }

  private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudIn;
    pcl::PointCloud<pcl::PointXYZ>::Ptr refCloud;
    bool firstFrame;
    Eigen::Matrix4d Twb;       // transformation from body to world
    Eigen::Matrix4d Twk;       // transformation from keyframe to world
    Eigen::Matrix4d Twb_gt;    // transformation from body to world (ground truth)
    Eigen::Matrix4d Twb_prev;
    Eigen::Matrix4d deltaT_pred;
    std::ofstream traj_file;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr scan_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub;
    nav_msgs::msg::Path path;

    rclcpp::TimerBase::SharedPtr timer_;

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
        }
        else{
            laserScanQueueMutex.unlock();
        }
        laserScanQueueMutex.unlock();
        return output_scan;
    }

    void icp(){
        pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        // Handle no scan data
        if(laserScanQueue.empty()){
            RCLCPP_INFO(this->get_logger(), "Waiting for laser scan data...");
            return;
        }
        else{
            laserScan2PointCloud(laser_scan_dequeue(), laserCloudIn);
        }


        // Handle the first scan
        if(firstFrame){
            RCLCPP_INFO(this->get_logger(), "Received the first scan!");
            firstFrame = false;
            pcl::PointCloud<pcl::PointXYZ>::Ptr laserTransformed(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::transformPointCloud(*laserCloudIn, *laserTransformed,Twb.cast<float>());
            *refCloud = *laserTransformed;
            Twk = Twb;
            Twb_prev = Twb;
            deltaT_pred = Eigen::Matrix4d::Identity();
            return;
        }

        // ICP process
        //Eigen::Matrix4d guess = Twb_prev * deltaT_pred;
        Twb = icp_registration(laserCloudIn, refCloud, Twb);
        deltaT_pred = Twb_prev.inverse() * Twb;
        Twb_prev = Twb;

        // update map
        Eigen::Matrix4d Tbk = Twb.inverse() * Twk;
        double delta_t = Tbk.block<3, 1>(0, 3).norm();
        double delta_r = acos((Tbk.block<3, 3>(0, 0).trace() - 1) / 2) * 180 / M_PI;
        if (delta_t > 2.0 || delta_r > 5.0) {
          pcl::PointCloud<pcl::PointXYZ>::Ptr laserTransformed(
              new pcl::PointCloud<pcl::PointXYZ>);
          pcl::transformPointCloud(*laserCloudIn, *laserTransformed, Twb.cast<float>());
          *refCloud += *laserTransformed;
          Twk = Twb;
        }
        publishResult();
        RCLCPP_INFO(this->get_logger(), "Results published!");
    }

    void laserScan2PointCloud(sensor_msgs::msg::LaserScan::SharedPtr msg,
        pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out) const {

        cloud_out.reset(new pcl::PointCloud<pcl::PointXYZ>);

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

    void publishResult() {
        //    publish odom
        nav_msgs::msg::Odometry odom;
        odom.header.frame_id = "map";
        odom.child_frame_id = "laser_frame";
        odom.header.stamp = this->now();
        odom.pose.pose.position.x = Twb(0, 3);
        odom.pose.pose.position.y = Twb(1, 3);
        odom.pose.pose.position.z = Twb(2, 3);
        Eigen::Quaterniond q(Twb.block<3, 3>(0, 0));
        q.normalize();
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();
        odom_pub->publish(odom);

        //    publish path
        path.header.frame_id = "map";
        path.header.stamp = this->now();
        geometry_msgs::msg::PoseStamped pose;
        pose.header = odom.header;
        pose.pose = odom.pose.pose;
        path.poses.push_back(pose);
        path_pub->publish(path);

        //    publish map
        sensor_msgs::msg::PointCloud2 mapMsg;
        pcl::toROSMsg(*refCloud, mapMsg);
        mapMsg.header.frame_id = "map";
        mapMsg.header.stamp = this->now();
        map_pub->publish(mapMsg);

        //    publish laser
        sensor_msgs::msg::PointCloud2 laserMsg;
        pcl::PointCloud<pcl::PointXYZ>::Ptr laserTransformed(
            new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*laserCloudIn, *laserTransformed, Twb.cast<float>());
        pcl::toROSMsg(*laserTransformed, laserMsg);
        laserMsg.header.frame_id = "map";
        laserMsg.header.stamp = this->now();
        scan_pub->publish(laserMsg);
    }
};

int main(int argc, char * argv[]){
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ICP initialized");
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ICP>());
    rclcpp::shutdown();
    return 0;
}
