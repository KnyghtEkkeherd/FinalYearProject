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

    def _smooth(self, val: int, hist: deque):
        hist.append(val)
        return sum(hist) / len(hist)

    def _calc_vel(self, new_x: int, new_y: int, current_time: int):
        if self.prev_x is None or self.prev_y is None or self.prev_time is None:
            self.prev_x, self.prev_y = new_x, new_y
            self.prev_time = current_time
            return 0.0

        time_diff = (current_time - self.prev_time).nanoseconds / 1e9
        if time_diff == 0:
            return 0.0

        distance = math.sqrt((new_x - self.prev_x) ** 2 + (new_y - self.prev_y) ** 2)

        velocity = distance / time_diff

        return velocity

    def _handle_vel_spike(self, new_x: int, new_y: int, time: int):
        vel = self._calc_vel(new_x, new_y, time)

        if vel > self.max_vel_diff:
            print("KABOOM KABOOM KABOOM WRONG WAY WRONG WAY")
            return self.prev_x, self.prev_y, self.prev_time

        self.prev_x, self.prev_y, self.prev_time = new_x, new_y, time

        return new_x, new_y

    def _unwrap_angle(self, angle: int):
        # shift up 180 so no -ve
        # wrap within 360
        # minus 180 again
        return ((angle + 180) % 360) - 180
        # ie; -10 and 350 lowkey the same

    def _handle_az_spike(self, az: int):
        if self.prev_az is None:
            self.prev_az = az
            return az

        az_diff = self._unwrap_angle(az - self.prev_az)

        if abs(az_diff) > self.max_az_diff:
            print("AHHHHHH ANGLES THE ANGLES ARE TOO MUCH AHHH")
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
