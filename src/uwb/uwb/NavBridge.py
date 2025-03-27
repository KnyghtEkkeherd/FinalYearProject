import math
import numpy as np
from collections import deque
from geometry_msgs.msg import PoseStamped
from .Message import Message
from rclpy.clock import Clock


class NavBridge:
    def __init__(self, frame_id="laser_frame"):
        self.frame_id = frame_id
        self.spike_threshold = 0.5
        self.window_size = 5
        self.max_vel_diff = 5  # tweak this
        self.max_az_diff = 30  # tweak this

        self.x_hist = deque(maxlen=self.window_size)
        self.y_hist = deque(maxlen=self.window_size)
        self.az_hist = deque(maxlen=self.window_size)

        self.prev_x = None
        self.prev_y = None
        self.prev_time = None
        self.prev_az = None

    def _ema(self, new_val: float, prev_val: float, alpha: float = 0.5):
        return alpha * new_val + (1 - alpha) * prev_val

    def _calc_vel(self, new_x: float, new_y: float, current_time):
        if self.prev_time is None:
            self.prev_x, self.prev_y, self.prev_time = new_x, new_y, current_time
            return 0.0

        time_diff = (current_time - self.prev_time).nanoseconds / 1e9
        if time_diff <= 0:
            print("Invalid time difference detected.")
            return 0.0

        distance = math.sqrt((new_x - self.prev_x) ** 2 + (new_y - self.prev_y) ** 2)
        return distance / time_diff

    def _unwrap_angle(self, angle: float):
        return ((angle + 180) % 360) - 180

    def _smooth_angle(self, new_angle: float, prev_angle: float, alpha: float = 0.5):
        diff = self._unwrap_angle(new_angle - prev_angle)
        smoothed_angle = prev_angle + alpha * diff
        return self._unwrap_angle(smoothed_angle)

    def _handle_vel_spike(self, new_x: float, new_y: float, time):
        vel = self._calc_vel(new_x, new_y, time)

        if vel > self.max_vel_diff:
            print(
                f"Velocity spike detected: {vel} m/s (threshold: {self.max_vel_diff})"
            )
            return self.prev_x, self.prev_y

        pos_diff = math.sqrt((new_x - self.prev_x) ** 2 + (new_y - self.prev_y) ** 2)
        if pos_diff > self.spike_threshold:
            print(
                f"Position spike detected: {pos_diff} m (threshold: {self.spike_threshold})"
            )
            return self.prev_x, self.prev_y

        self.prev_x, self.prev_y, self.prev_time = new_x, new_y, time
        return new_x, new_y

    def _handle_az_spike(self, az: float):
        if self.prev_az is None:
            self.prev_az = az
            return az

        az_diff = self._unwrap_angle(az - self.prev_az)
        if abs(az_diff) > self.max_az_diff:
            print(
                f"Angular spike detected: {az_diff}° (threshold: {self.max_az_diff}°)"
            )
            return self.prev_az

        self.prev_az = az
        return az

    def convert_message_to_goal(self, message: Message):
        time = Clock().now()

        goal = PoseStamped()
        goal.header.frame_id = self.frame_id
        goal.header.stamp = time.to_msg()

        # cm to m, div by 100
        raw_x = message.distance * np.cos(math.radians(message.azimuth)) / 100.0
        raw_y = message.distance * np.sin(math.radians(message.azimuth)) / 100.0
        raw_z = message.elevation / 100.0

        swag_x, swag_y = self._handle_vel_spike(raw_x, raw_y, time)
        swagger_x, swagger_y = (
            self._smooth(swag_x, self.x_hist),
            self._smooth(swag_y, self.y_hist),
        )

        goal.pose.position.x = swagger_x
        goal.pose.position.y = swagger_y
        goal.pose.position.z = raw_z

        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0

        raw_az = message.azimuth
        swag_az = self._handle_az_spike(raw_az)
        swagger_az = self._smooth(swag_az, self.az_hist)

        goal.pose.orientation.z = np.sin(math.radians(swagger_az / 2.0))
        goal.pose.orientation.w = np.cos(math.radians(swagger_az / 2.0))

        return goal
