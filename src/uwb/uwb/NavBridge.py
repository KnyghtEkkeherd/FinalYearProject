import numpy as np
from collections import deque
from geometry_msgs.msg import PoseStamped, Pose
from rclpy.clock import Clock
from scipy.signal import medfilt, savgol_filter

class KalmanFilter:
    def __init__(self, process_variance=1e-5, measurement_variance=0.1):
        self.process_variance = process_variance
        self.measurement_variance = measurement_variance
        self.estimate = 0.0
        self.error_covariance = 1.0

    def update(self, measurement):
        self.error_covariance += self.process_variance
        kalman_gain = self.error_covariance / (self.error_covariance + self.measurement_variance)
        self.estimate += kalman_gain * (measurement - self.estimate)
        self.error_covariance *= (1 - kalman_gain)
        return self.estimate

class NavBridge:
    def __init__(self, frame_id="uwb_frame", filter_instance=None):
        self.frame_id = frame_id
        self.dist_buf = deque(maxlen=10)
        self.azimuth_buf = deque(maxlen=10)
        self.kalman_filter_dist = KalmanFilter()
        self.kalman_filter_azimuth = KalmanFilter()

    def smooth_distance(self):
        if len(self.dist_buf) < 3:
            return np.mean(self.dist_buf)

        filtered_distance = medfilt(list(self.dist_buf), kernel_size=3)

        window_length = min(len(filtered_distance), 5)
        if window_length % 2 == 0:
            window_length -= 1

        smoothed_distance = savgol_filter(filtered_distance, window_length=window_length, polyorder=2)
        return self.kalman_filter_dist.update(smoothed_distance[-1])

    def smooth_azimuth(self):
        if len(self.azimuth_buf) < 3:
            return np.mean(self.azimuth_buf)

        filtered_azimuth = medfilt(list(self.azimuth_buf), kernel_size=3)

        window_length = min(len(filtered_azimuth), 5)
        if window_length % 2 == 0:
            window_length -= 1

        smoothed_azimuth = savgol_filter(filtered_azimuth, window_length=window_length, polyorder=2)
        return self.kalman_filter_azimuth.update(smoothed_azimuth[-1])

    def convert_message_to_goal(self, message):
        goal = PoseStamped()
        goal.pose = Pose()
        
        goal.header.frame_id = self.frame_id
        goal.header.stamp = Clock().now().to_msg()

        raw_distance = message.parsed_fields["distance"]
        raw_azimuth = message.parsed_fields["azimuth"]

        self.dist_buf.append(float(raw_distance))
        self.azimuth_buf.append(float(raw_azimuth))

        smoothed_distance = self.smooth_distance()
        smoothed_azimuth = self.smooth_azimuth()

        azimuth_radians = np.deg2rad(smoothed_azimuth)

        goal.pose.position.x = 0.0 # smoothed_distance * np.cos(azimuth_radians) / 100.0
        goal.pose.position.y = 0.0 # smoothed_distance * np.sin(azimuth_radians) / 100.0 * -1
        goal.pose.position.z = 0.0

        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0

        return goal