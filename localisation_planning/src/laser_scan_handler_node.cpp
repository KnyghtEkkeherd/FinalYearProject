#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <cmath>
#include <vector>
using std::placeholders::_1;

class LaserScan2PointCloud : public rclcpp::Node {
  public:
    LaserScan2PointCloud()
    : Node("laserScan2PointCloud") {
      subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "sensor_msgs/LaserScan", 10, std::bind(&LaserScan2PointCloud::conversion, this, _1));

      publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/pointCloud", 10);
    }

  private:
    void conversion(const sensor_msgs::msg::LaserScan & msg) const {
      // Conversion logic from LaserScan to PointCloud2
      auto point_cloud_msg = sensor_msgs::msg::PointCloud2();

      // Fill in the header
      point_cloud_msg.header = msg.header;

      // Set the fields of the PointCloud2 message
      point_cloud_msg.height = 1;
      point_cloud_msg.width = msg.ranges.size();
      point_cloud_msg.is_dense = false;
      point_cloud_msg.is_bigendian = false;

      // Define the fields of the PointCloud2 message
      sensor_msgs::msg::PointField field;
      field.name = "x";
      field.offset = 0;
      field.datatype = sensor_msgs::msg::PointField::FLOAT32;
      field.count = 1;
      point_cloud_msg.fields.push_back(field);

      field.name = "y";
      field.offset = 4;
      point_cloud_msg.fields.push_back(field);

      field.name = "z";
      field.offset = 8;
      point_cloud_msg.fields.push_back(field);

      point_cloud_msg.point_step = 12;
      point_cloud_msg.row_step = point_cloud_msg.point_step * point_cloud_msg.width;

      point_cloud_msg.data.resize(point_cloud_msg.row_step * point_cloud_msg.height);

      for (size_t i = 0; i < msg.ranges.size(); ++i) {
        float range = msg.ranges[i];
        float angle = msg.angle_min + i * msg.angle_increment;

        float x = range * std::cos(angle);
        float y = range * std::sin(angle);
        float z = 0.0;

        memcpy(&point_cloud_msg.data[i * point_cloud_msg.point_step + 0], &x, sizeof(float));
        memcpy(&point_cloud_msg.data[i * point_cloud_msg.point_step + 4], &y, sizeof(float));
        memcpy(&point_cloud_msg.data[i * point_cloud_msg.point_step + 8], &z, sizeof(float));

        // Print the received data and the converted data
        RCLCPP_INFO(this->get_logger(), "X: %f, Y: %f, Z: %f", x, y, z);
      }

      publisher_->publish(point_cloud_msg);
    }
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaserScan2PointCloud>());
  rclcpp::shutdown();
  return 0;
}
