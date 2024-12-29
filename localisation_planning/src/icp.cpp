#include "icp.h"
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
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/pointCloud", 10, std::bind(&ICP::icp_callback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/icp_odom", 10);
    }

  private:
    void icp_callback(const sensor_msgs::msg::PointCloud2::SharedPtr input_cloud)
    {
        auto message = nav_msgs::msg::Odometry();

        pointCloudQueueMutex.lock();
        pointCloudQueue.push(input_cloud);
        pointCloudQueueMutex.unlock();

        Eigen::Matrix4d transformation = icp_body();

        RCLCPP_INFO(this->get_logger(), "Header frame_id: '%s'", message.header.frame_id.c_str());
        publisher_->publish(message);
    }

    Eigen::Matrix4d icp_body()
    {
        pcl::PCLPointCloud2 pcl_pc2;
        pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        static pcl::PointCloud<pcl::PointXYZ>::Ptr past_cloud = nullptr;
        Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();
        Eigen::Matrix4d init_guess = Eigen::Matrix4d::Identity();

        pointCloudQueueMutex.lock();
        if (!pointCloudQueue.empty()) {
            pcl_conversions::toPCL(*pointCloudQueue.front(), pcl_pc2);
            pcl::fromPCLPointCloud2(pcl_pc2, *current_cloud);
            pointCloudQueue.pop();
        }
        pointCloudQueueMutex.unlock();

        transformation = icp_registration(current_cloud, past_cloud, init_guess);

        past_cloud = current_cloud;
        return transformation;
    }

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    std::queue<sensor_msgs::msg::PointCloud2::SharedPtr> pointCloudQueue;
    std::mutex pointCloudQueueMutex;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ICP>());
  rclcpp::shutdown();
  return 0;
}

Eigen::Matrix4d icp_registration(pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud,
                                 pcl::PointCloud<pcl::PointXYZ>::Ptr tar_cloud,
                                 Eigen::Matrix4d init_guess) {

  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(src_cloud);
  icp.setInputTarget(tar_cloud);
  icp.setMaximumIterations(MAX_ITERATIONS);
  icp.setTransformationEpsilon(1e-6);
  icp.setMaxCorrespondenceDistance(MAX_DISTANCE);
  pcl::PointCloud<pcl::PointXYZ> aligned_cloud;
  icp.align(aligned_cloud, init_guess.cast<float>());

  Eigen::Matrix4d transformation = icp.getFinalTransformation().cast<double>();
  return transformation;
}
