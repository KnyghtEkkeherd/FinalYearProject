#ifndef EKF_H
#define EKF_H

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
#include <cmath>
#include <Eigen3/Eigen/Dense>
#include <pcl/filters/voxel_grid.h>

#define ASSOCIATION_THRESHOLD 3

// Map
pcl::PointCloud<pcl::PointXYZ>::Ptr mapCloud;
pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudIn;

static inline double safeDivide(double numerator, double denominator){
    const double epsilon = 1e-6;
    if (std::abs(denominator) < epsilon) {
        denominator = (denominator < 0) ? -epsilon : epsilon;
    }
    return numerator / denominator;
}

template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 3, 1> R2ypr(
    const Eigen::MatrixBase<Derived> &R) {
  typedef typename Derived::Scalar Scalar_t;
  Eigen::Matrix<Scalar_t, 3, 1> n = R.col(0);
  Eigen::Matrix<Scalar_t, 3, 1> o = R.col(1);
  Eigen::Matrix<Scalar_t, 3, 1> a = R.col(2);

  Eigen::Matrix<Scalar_t, 3, 1> ypr(3);
  Scalar_t y = atan2(n(1), n(0));
  Scalar_t p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
  Scalar_t r =
      atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
  ypr(0) = y;
  ypr(1) = p;
  ypr(2) = r;

  return ypr / M_PI * 180.0;
}

inline Eigen::Vector3d Pose6DTo3D(const Eigen::Matrix4d &pose6d) {
  Eigen::Vector3d pose3d;
  pose3d(0) = pose6d(0, 3);
  pose3d(1) = pose6d(1, 3);
  pose3d(2) = atan2(pose6d(1, 0), pose6d(0, 0));
  return pose3d;
}

inline Eigen::Matrix4d Pose3DTo6D(const Eigen::Vector3d &pose3d) {
  Eigen::Matrix4d pose6d;
  pose6d << cos(pose3d(2)), -sin(pose3d(2)), 0, pose3d(0), sin(pose3d(2)),
      cos(pose3d(2)), 0, pose3d(1), 0, 0, 1, 0, 0, 0, 0, 1;
  return pose6d;
}

inline Eigen::Matrix3d ExpSE2(const Eigen::Vector3d &se2) {
  Eigen::Matrix3d R;
  R << cos(se2(2)), -sin(se2(2)), se2(0), sin(se2(2)), cos(se2(2)), se2(1), 0,
      0, 1;
  return R;
}

inline Eigen::Vector3d LogSE2(const Eigen::Matrix3d &SE2) {
  Eigen::Vector3d se2;
  se2(0) = SE2(0, 2);
  se2(1) = SE2(1, 2);
  se2(2) = atan2(SE2(1, 0), SE2(0, 0));
  return se2;
}


// EKF SLAM node
class EKFSLAM : public rclcpp::Node
{
public:
    EKFSLAM();

private:
    // Node handle
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;

    std::queue<pcl::PointCloud<pcl::PointXYZ>::Ptr> pointCloudQueue;
    std::mutex pointCloudQueueMutex;

    std::queue<nav_msgs::msg::Odometry::SharedPtr> odomQueue;
    std::mutex odomQueueMutex;

    // SLAM functions
    void run();
    double normalizeAngle(double angle);
    Eigen::MatrixXd jacobGt(const Eigen::VectorXd& state, Eigen::Vector2d ut, double dt);
    Eigen::MatrixXd jacobFt(const Eigen::VectorXd& state, Eigen::Vector2d ut, double dt);
    Eigen::MatrixXd jacobB(const Eigen::VectorXd& state, Eigen::Vector2d ut, double dt);
    void predictState(Eigen::VectorXd& state, Eigen::MatrixXd& cov, Eigen::Vector2d ut, double dt);
    Eigen::Vector2d transform(const Eigen::Vector2d& p, const Eigen::Vector3d& x);
    void accumulateMap();
    void updateMeasurement();
    void addNewLandmark(const Eigen::Vector2d& lm, const Eigen::MatrixXd& InitCov);

    Eigen::Matrix4d Twb;     // body to world
    Eigen::VectorXd mState;  // x, y, yaw, l1, ..., lN (l_x, l_y)
    Eigen::MatrixXd mCov;    // covariance matrix of mState
    Eigen::MatrixXd R;       // process noise
    Eigen::MatrixXd Q;       // measurement noise

    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCLoudIn, mapCloud;
    Eigen::MatrixX2d cylinderPoints;
    int globalId = -1;
};
#endif
