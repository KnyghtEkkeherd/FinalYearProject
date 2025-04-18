#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class LaserScanPublisher : public rclcpp::Node {
  public:
    LaserScanPublisher()
    : Node("laser_scan_publisher") {
      publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);
      timer_ = this->create_wall_timer(
        std::chrono::milliseconds(2000),
        std::bind(&LaserScanPublisher::publish_scan, this));
    }

      private:
        void publish_scan() {
          auto message = sensor_msgs::msg::LaserScan();
          message.header.stamp = this->now();
          message.header.frame_id = "laser_frame";
          message.angle_min = -3.14;
          message.angle_max = 3.14;
          message.angle_increment = 6.28 / 100;
          message.time_increment = (1 / 40) / (2 * 6.28);
          message.scan_time = 1 / 40;
          message.range_min = 0.0;
          message.range_max = 10.0;

          int num_readings = 100;
          message.ranges.resize(num_readings);
          message.intensities.resize(num_readings);
          for (int i = 0; i < num_readings; ++i) {
            message.ranges[i] = 6.0;
            message.intensities[i] = 100;
          }

          publisher_->publish(message);

          // Display the data to the console
          RCLCPP_INFO(this->get_logger(), "Publishing LaserScan data:");
          RCLCPP_INFO(this->get_logger(), "Header: frame_id=%s, stamp=%d", message.header.frame_id.c_str(), message.header.stamp.sec);
          RCLCPP_INFO(this->get_logger(), "Angle min: %f, Angle max: %f", message.angle_min, message.angle_max);
          RCLCPP_INFO(this->get_logger(), "Range min: %f, Range max: %f", message.range_min, message.range_max);
        }

    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaserScanPublisher>());
  rclcpp::shutdown();
  return 0;
}
