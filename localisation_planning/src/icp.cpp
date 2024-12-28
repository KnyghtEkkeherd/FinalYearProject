#include "icp.h"

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
    void icp_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        auto message = nav_msgs::msg::Odometry();
        // Placeholder info for now
        // TODO: finish ICP calculations here
        message.header.frame_id = "odom";
        message.child_frame_id = "base_link";

        RCLCPP_INFO(this->get_logger(), "Header frame_id: '%s'", message.header.frame_id.c_str());
        publisher_->publish(message);
    }
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
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


Eigen::Matrix4d icp_body(const sensor_msgs::msg::PointCloud2::SharedPtr input)
{
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*input, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_pc2, *current_cloud);

    static pcl::PointCloud<pcl::PointXYZ>::Ptr past_cloud = nullptr;

    if (!past_cloud) {
        past_cloud = current_cloud;
        return Eigen::Matrix4d::Identity();
    }

    Eigen::Matrix4d init_guess = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d transformation = icp_registration(current_cloud, past_cloud, init_guess);

    past_cloud = current_cloud;
    return transformation;
}
