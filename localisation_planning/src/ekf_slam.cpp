#include "ekf.h"


/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class EKF : public rclcpp::Node{
  public:
    EKF()
    : Node("icpOdomNode")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/pointCloud", 10, std::bind(&EKF::ekf_callback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/icp_odom", 10);
    }

  private:
    void ekf_callback(const sensor_msgs::msg::PointCloud2::SharedPtr input_cloud)
    {
        auto message = nav_msgs::msg::Odometry();

        pointCloudQueueMutex.lock();
        pointCloudQueue.push(input_cloud);
        pointCloudQueueMutex.unlock();

        // Call ekf slam here

        RCLCPP_INFO(this->get_logger(), "Header frame_id: '%s'", message.header.frame_id.c_str());
        publisher_->publish(message);
    }

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

double normalizeAngle(double angle)
{
  return atan2(sin(angle), cos(angle));
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EKF>());
  rclcpp::shutdown();
  return 0;
}
