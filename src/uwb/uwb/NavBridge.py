import math
import numpy as np
from collections import deque
from geometry_msgs.msg import PoseStamped
from .Message import Message
from rclpy.clock import Clock

# SciPy filtering functions
from scipy.signal import (
    savgol_filter,
    medfilt,
)  # or you could alternatively use lfilter for a Butterworth filter


class NavBridge:
    def __init__(self, frame_id="laser_frame"):
        self.frame_id = frame_id
        self.spike_threshold = 0.3
        self.window_size = 15  # history length for filtering
        self.max_vel = 5  # tweak as needed
        self.max_az_diff = 30  # tweak as needed
        self.hysteresis_threshold = 0.1

        # Histories now used for SciPy filtering
        self.x_hist = deque(maxlen=self.window_size)
        self.y_hist = deque(maxlen=self.window_size)
        self.az_hist = deque(maxlen=self.window_size)

        self.prev_x = None
        self.prev_y = None
        self.prev_time = None
        self.prev_az = None

        # Parameters for filtering. For savgol_filter, window_length must be odd.
        self.smoothing_window_length = (
            5  # Adjust depending on how many samples you want to consider
        )
        self.polyorder = 2  # Savitzky–Golay polynomial order

    def _smooth(self, new_val: float, hist: deque) -> float:
        """
        Append the new value to the history, then apply a median filter (to remove outliers)
        followed by a Savitzky–Golay filter for smoothing.
        """
        hist.append(new_val)
        arr = np.array(hist)

        # If there’s not enough data, just return the mean of what we have.
        if len(arr) < self.smoothing_window_length:
            return np.mean(arr)

        # 1. Median filter to mitigate spikes.
        #    The kernel size must be odd and less than or equal to the length of `arr`.
        kernel_size = self.smoothing_window_length
        if len(arr) < kernel_size:
            kernel_size = len(arr) if len(arr) % 2 == 1 else len(arr) - 1

        median_filtered = medfilt(arr, kernel_size=kernel_size)

        # 2. Apply the Savitzky–Golay filter for smoothing.
        smoothed = savgol_filter(
            median_filtered,
            window_length=self.smoothing_window_length,
            polyorder=self.polyorder,
        )
        # Return the latest (smoothed) value.
        return smoothed[-1]

    def _calc_vel(self, new_x: float, new_y: float, current_time) -> float:
        if self.prev_time is None:
            self.prev_x, self.prev_y, self.prev_time = new_x, new_y, current_time
            return 0.0

        time_diff = (current_time - self.prev_time).nanoseconds / 1e9
        if time_diff == 0:
            return 0.0

        distance = math.sqrt((new_x - self.prev_x) ** 2 + (new_y - self.prev_y) ** 2)
        velocity = distance / time_diff

        return velocity

    def _handle_vel_spike(self, new_x: float, new_y: float, time) -> tuple:
        """
        Check if the new (x, y) is a spike compared to the previous measurement.
        """
        vel = self._calc_vel(new_x, new_y, time)

        if vel > (self.max_vel + self.hysteresis_threshold):
            print("VELOCITY SPIKE DETECTED!!! AAAAAAAAA")
            return self.prev_x, self.prev_y

        pos_diff = math.sqrt((new_x - self.prev_x) ** 2 + (new_y - self.prev_y) ** 2)
        if pos_diff > (self.spike_threshold + self.hysteresis_threshold):
            print("POSITION SPIKE DETECTED!!! AAAAAAA")
            return self.prev_x, self.prev_y

        self.prev_x, self.prev_y, self.prev_time = new_x, new_y, time
        return new_x, new_y

    def _unwrap_angle(self, angle: float) -> float:
        # Convert angle to a symmetric range around zero.
        # For example, -10 and 350 both become -10.
        return ((angle + 180) % 360) - 180

    def _handle_az_spike(self, az: float) -> float:
        """
        Handle abrupt changes in azimuth by verifying the angle difference.
        """
        if self.prev_az is None:
            self.prev_az = az
            return az

        az_diff = self._unwrap_angle(az - self.prev_az)

        if abs(az_diff) > self.max_az_diff:
            print("AHHHHHH ANGLES THE ANGLES ARE TOO MUCH AHHH")
            return self.prev_az

        # Simple averaging for azimuth; you may also use the filtering approach below if needed.
        smoothed_az = (self.prev_az + az) / 2.0
        self.prev_az = smoothed_az

        return smoothed_az

    def convert_message_to_goal(self, message: Message):
        time = Clock().now()

        goal = PoseStamped()
        goal.header.frame_id = self.frame_id
        goal.header.stamp = time.to_msg()

        # Convert polar values (with units) to x, y, z (meters)
        raw_x = message.distance * np.cos(math.radians(message.azimuth)) / 100.0
        raw_y = message.distance * np.sin(math.radians(message.azimuth)) / 100.0
        raw_z = message.elevation / 100.0

        # Use the velocity spike handler before smoothing.
        corrected_x, corrected_y = self._handle_vel_spike(raw_x, raw_y, time)
        smooth_x = self._smooth(corrected_x, self.x_hist)
        smooth_y = self._smooth(corrected_y, self.y_hist)

        goal.pose.position.x = smooth_x
        goal.pose.position.y = smooth_y
        goal.pose.position.z = raw_z

        # Orientation x and y are fixed for now.
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0

        raw_az = message.azimuth
        corrected_az = self._handle_az_spike(raw_az)
        smooth_az = self._smooth(corrected_az, self.az_hist)

        # Convert the azimuth angle to quaternion (assuming rotation around z)
        goal.pose.orientation.z = np.sin(math.radians(smooth_az / 2.0))
        goal.pose.orientation.w = np.cos(math.radians(smooth_az / 2.0))

        return goal
