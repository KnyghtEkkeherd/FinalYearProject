#include "ekf.h"

EKFSLAM::EKFSLAM() : Node("ekf_slam"){
    point_cloud_scan_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/pointCloud", 10, std::bind(&EKFSLAM::pointCloudCallback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&EKFSLAM::odomCallback, this, std::placeholders::_1));
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/pose", 10);
    map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/map", 10);
}

void EKFSLAM::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg){
    odomQueueMutex.lock();
    if (!odomQueue.empty()) {
        auto odom_msg = odomQueue.front();
        odomQueue.pop();
        odomQueueMutex.unlock();

        // TODO: EKF SLAM


    } else {
        odomQueueMutex.unlock();
    }

    // Publish the estimated pose
    auto pose_msg = geometry_msgs::msg::PoseStamped();
    pose_msg.header.stamp = this->now();
    pose_msg.header.frame_id = "map";
    // TODO: Fill in pose_msg with the estimated pose

    pose_pub_->publish(pose_msg);
}

void EKFSLAM::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg){
    odomQueueMutex.lock();
    odomQueue.push(msg);
    odomQueueMutex.unlock();
}

void EKFSLAM::run() {
}

double EKFSLAM::normalizeAngle(double angle) {
  if (angle > M_PI) {
    angle -= 2 * M_PI;
  } else if (angle < -M_PI) {
    angle += 2 * M_PI;
  }
  return angle;
}

Eigen::MatrixXd EKFSLAM::jacobGt(const Eigen::VectorXd& state,
                                 Eigen::Vector2d ut, double dt) {
  int num_state = state.rows();
  Eigen::MatrixXd Gt = Eigen::MatrixXd::Identity(num_state, num_state);
   Gt(0, 2) = (-safeDivide(ut[0], ut[1]))*cos(state[2]) + (safeDivide(ut[0], ut[1]))*cos(state[2] + ut[1]*dt);
   Gt(1, 2) = (-safeDivide(ut[0], ut[1]))*sin(state[2]) + (safeDivide(ut[0], ut[1]))*sin(state[2] + ut[1]*dt);
  return Gt;
}

Eigen::MatrixXd EKFSLAM::jacobFt(const Eigen::VectorXd& state,
                                 Eigen::Vector2d ut, double dt) {
  int num_state = state.rows();
  Eigen::MatrixXd Ft = Eigen::MatrixXd::Zero(num_state, 2);
   Ft(0, 0) = -safeDivide(1.0, ut[1])*sin(state[2]) + safeDivide(1.0, ut[1])*sin(state[2] + ut[1] * dt);
   Ft(0, 1) = safeDivide(1.0, ut[1]) * cos(state[2]) - safeDivide(1.0, ut[1]) * cos(state[2] + ut[1] * dt);
   Ft(1, 0) = safeDivide(1.0, ut[1]) * cos(state[2]) - safeDivide(1.0, ut[1]) * cos(state[2] + ut[1] * dt);
   Ft(1, 1) = -safeDivide(1.0, ut[1]) * sin(state[2]) + safeDivide(1.0, ut[1]) * sin(state[2] + ut[1] * dt);
   Ft(2, 1) = dt;
  return Ft;
}

Eigen::MatrixXd EKFSLAM::jacobB(const Eigen::VectorXd& state,
                                Eigen::Vector2d ut, double dt) {
  int num_state = state.rows();
  Eigen::MatrixXd B = Eigen::MatrixXd::Zero(num_state, 2);
  double v = ut(0);
  double w = ut(1);
  B(0, 0) = -safeDivide(sin(state(2)) - sin(state(2) + w * dt), w);
  B(1, 0) = safeDivide(cos(state(2)) - cos(state(2) + w * dt), w);
  B(2, 1) = dt;
  return B;
}

void EKFSLAM::predictState(Eigen::VectorXd& state, Eigen::MatrixXd& cov,
                           Eigen::Vector2d ut, double dt) {
  // Note: ut = [v, w]
  state = state + jacobB(state, ut, dt) * ut;  // update state
  Eigen::MatrixXd Gt = jacobGt(state, ut, dt);
  Eigen::MatrixXd Ft = jacobFt(state, ut, dt);
  cov = Gt * cov * Gt.transpose() + Ft * R * Ft.transpose(); // update
}

Eigen::Vector2d EKFSLAM::transform(const Eigen::Vector2d& p,
                                   const Eigen::Vector3d& x) {
  Eigen::Vector2d p_t;
  p_t(0) = p(0) * cos(x(2)) - p(1) * sin(x(2)) + x(0);
  p_t(1) = p(0) * sin(x(2)) + p(1) * cos(x(2)) + x(1);
  return p_t;
}

void EKFSLAM::addNewLandmark(const Eigen::Vector2d& lm,
                             const Eigen::MatrixXd& InitCov) {
  // add new landmark to mState and mCov
   mState.conservativeResize(mState.size() + lm.size());
   mState.tail(lm.size()) = lm;

   //cout << "add a new landmark to mCov" << endl;
   int new_size = mState.size();
   Eigen::MatrixXd new_cov = Eigen::MatrixXd::Zero(new_size, new_size);
   new_cov.topLeftCorner(mCov.rows(), mCov.cols()) = mCov;
   new_cov.block<2, 2>(new_size - 2, new_size - 2) = InitCov;
   mCov = new_cov;
}

void EKFSLAM::accumulateMap() {
  Eigen::Matrix4d Twb = Pose3DTo6D(mState.segment(0, 3));
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformedCloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(*pointCloudIn, *transformedCloud, Twb);
  *mapCloud += *transformedCloud;

  pcl::VoxelGrid<pcl::PointXYZ> voxelSampler;
  voxelSampler.setInputCloud(mapCloud);
  voxelSampler.setLeafSize(0.5, 0.5, 0.5);
  voxelSampler.filter(*mapCloud);
}

void EKFSLAM::updateMeasurement() {
  Eigen::Vector3d xwb = mState.block<3, 1>(0, 0);  // pose in the world frame
  int num_landmarks = (mState.rows() - 3) / 2;  // number of landmarks in the state vector
  int num_obs = 0;  // FIXME: change this to the number of observations of any landmark
  Eigen::VectorXi indices = Eigen::VectorXi::Ones(num_obs) *
                            -1;  // indices of landmarks in the state vector
  for (int i = 0; i < num_obs; ++i) {
    Eigen::Vector2d pt_transformed = 0; // FIXME: change this to any landmark type
    double min_distance = std::numeric_limits<double>::max();
    double current_distance = std::numeric_limits<double>::max();
    int min_landmark_idx = -1; // Initialize to -1 (no landmark found)

    if (num_landmarks > 0) {
        for (int j = 0; j < num_landmarks; ++j) {
            current_distance = (pt_transformed - mState.segment<2>(3 + j * 2)).norm();

            if (current_distance < min_distance) {
                min_distance = current_distance;
                min_landmark_idx = j;
            }
        }

        if (min_distance <= ASSOCIATION_THRESHOLD && min_landmark_idx != -1) {
            indices(i) = min_landmark_idx;
        }
    }
    if (indices(i) == -1) {
      indices(i) = ++globalId;
      addNewLandmark(pt_transformed, Q);
    }
  }
  // simulating bearing model
  Eigen::VectorXd z = Eigen::VectorXd::Zero(2 * num_obs);
  for (int i = 0; i < num_obs; ++i) {
    const Eigen::Vector2d& pt = 0; // FIXME: change the landmark type and udpate this!
    z(2 * i, 0) = pt.norm();
    z(2 * i + 1, 0) = atan2(pt(1), pt(0));
  }
  // update the measurement vector
  num_landmarks = (mState.rows() - 3) / 2;
  for (int i = 0; i < num_obs; ++i) {
    int idx = indices(i);
    if (idx == -1 || idx + 1 > num_landmarks) continue;
    const Eigen::Vector2d& landmark = mState.block<2, 1>(3 + idx * 2, 0);
     double delta_x = landmark(0) - mState(0);
     double delta_y = landmark(1) - mState(1);
     double q = pow(delta_x, 2) + pow(delta_y, 2);
     Eigen::Vector2d z_hat (sqrt(q), atan2(delta_y, delta_x) - mState(2));

     Eigen::Matrix<double, 2, 5> low_Ht = Eigen::Matrix<double, 2, 5>::Zero();
     low_Ht(0, 0) = -sqrt(q)*delta_x;
     low_Ht(0, 1) = -sqrt(q)*delta_y;
     low_Ht(0, 3) = sqrt(q)*delta_x;
     low_Ht(0, 4) = sqrt(q)*delta_y;
     low_Ht(1, 0) = delta_y;
     low_Ht(1, 1) = -delta_x;
     low_Ht(1, 2) = -q;
     low_Ht(1, 3) = -delta_y;
     low_Ht(1, 4) = delta_x;

     Eigen::MatrixXd high_Ht(5, 2 * num_landmarks + 3);
     high_Ht.setZero();
     high_Ht(0, 0) = 1;
     high_Ht(1, 1) = 1;
     high_Ht(2, 2) = 1;
     high_Ht(3, 3 + 2 * idx) = 1;
     high_Ht(4, 3 + 2 * idx + 1) = 1;

     Eigen::MatrixXd Ht = safeDivide(1.0, q)*low_Ht*high_Ht;
     Eigen::MatrixXd Kt = mCov * Ht.transpose() * (Ht * mCov * Ht.transpose() + Q).inverse();
     mState = mState + Kt * (z.segment<2>(2 * i) - z_hat);
     mCov = (Eigen::MatrixXd::Identity(mCov.rows(), mCov.cols()) - Kt * Ht) * mCov;
  }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EKFSLAM>());
    rclcpp::shutdown();
    return 0;
}
