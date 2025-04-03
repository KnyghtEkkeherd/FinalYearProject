import math
import numpy as np
from collections import deque
from geometry_msgs.msg import PoseStamped
from .Message import Message
from rclpy.clock import Clock


class NavBridge:
    def __init__(self, frame_id="laser_frame"):
        self.frame_id = frame_id
        self.window_size = 5
        self.history = {
            "distance": deque(maxlen=self.window_size),
            "azimuth": deque(maxlen=self.window_size),
            "elevation": deque(maxlen=self.window_size),
        }

    def smooth_value(self, history, value):
        history.append(value)
        return np.mean(history)

    def remove_spikes(self, value, history_key, threshold):
        if self.history[history_key]:
            last_value = self.history[history_key][-1]
            if abs(value - last_value) > threshold:
                return last_value
        return value

    def convert_message_to_goal(self, message: Message):
        goal = PoseStamped()
        goal.header.frame_id = self.frame_id
        goal.header.stamp = Clock().now().to_msg()

        distance_threshold = 100.0
        azimuth_threshold = 10.0
        elevation_threshold = 10.0

        filtered_distance = self.remove_spikes(
            message.distance, "distance", distance_threshold
        )
        filtered_azimuth = self.remove_spikes(
            message.azimuth, "azimuth", azimuth_threshold
        )
        filtered_elevation = self.remove_spikes(
            message.elevation, "elevation", elevation_threshold
        )

        smoothed_distance = self.smooth_value(
            self.history["distance"], filtered_distance
        )
        smoothed_azimuth = self.smooth_value(self.history["azimuth"], filtered_azimuth)
        smoothed_elevation = self.smooth_value(
            self.history["elevation"], filtered_elevation
        )

        goal.pose.position.x = (
            smoothed_distance * np.cos(math.radians(smoothed_azimuth)) / 100.0
        )
        goal.pose.position.y = (
            smoothed_distance * np.sin(math.radians(smoothed_azimuth)) / 100.0
        )
        goal.pose.position.z = smoothed_elevation / 100.0

        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = np.sin(math.radians(smoothed_azimuth / 2.0))
        goal.pose.orientation.w = np.cos(math.radians(smoothed_azimuth / 2.0))

        return goal
