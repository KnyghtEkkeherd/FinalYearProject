import math
import time
from collections import deque

import numpy as np
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise


class Filter:
    def __init__(self, dt=0.1, alpha=0.2, velocity_buffer_size=5):
        self.positions_x = []
        self.positions_y = []
        self.azimuth_values = []
        self.timestamps = []

        self.alpha = alpha
        self.ema_azimuth = None
        self.velocity_buffer = deque(maxlen=velocity_buffer_size)
        self.last_time = None
        self.dt = dt

        self.kf = KalmanFilter(dim_x=4, dim_z=2)
        self.kf.F = np.array([[1, 0, dt, 0], [0, 1, 0, dt], [0, 0, 1, 0], [0, 0, 0, 1]])
        self.kf.H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]])
        self.kf.x = np.array([0, 0, 0, 0])
        self.kf.P = np.eye(4) * 1000
        self.kf.R = np.eye(2) * 1.0
        self.kf.Q = Q_discrete_white_noise(dim=4, dt=dt, var=1e-6)

    def exponential_moving_average(self, old_value, new_value):
        if old_value is None:
            return new_value
        return self.alpha * new_value + (1 - self.alpha) * old_value

    def process_message(self, message):
        current_time = time.time()
        dt_local = (
            (current_time - self.last_time)
            if (self.last_time and current_time > self.last_time)
            else self.dt
        )

        az = message.azimuth
        x_meas = message.distance * math.cos(math.radians(az)) / 100.0
        y_meas = message.distance * math.sin(math.radians(az)) / 100.0

        if self.positions_x and self.positions_y:
            prev_x = self.positions_x[-1]
            prev_y = self.positions_y[-1]
            displacement = math.hypot(x_meas - prev_x, y_meas - prev_y)
            current_velocity = displacement / dt_local
        else:
            current_velocity = 0.0

        self.velocity_buffer.append(current_velocity)
        if len(self.velocity_buffer) >= 3:
            median_velocity = np.median(list(self.velocity_buffer))
            factor = 2.5
            min_threshold = 0.05
            allowed_threshold = max(median_velocity * factor, min_threshold)
        else:
            allowed_threshold = float("inf")

        if current_velocity > allowed_threshold:
            if self.positions_x and self.positions_y:
                x_valid = self.positions_x[-1]
                y_valid = self.positions_y[-1]
            else:
                x_valid = x_meas
                y_valid = y_meas
        else:
            x_valid = x_meas
            y_valid = y_meas

        self.kf.predict()
        self.kf.update(np.array([x_valid, y_valid]))
        x_filtered, y_filtered = self.kf.x[0], self.kf.x[1]

        self.ema_azimuth = self.exponential_moving_average(self.ema_azimuth, az)

        self.positions_x.append(x_filtered)
        self.positions_y.append(y_filtered)
        self.azimuth_values.append(self.ema_azimuth)
        self.timestamps.append(current_time)
        self.last_time = current_time

        return {
            "x": x_filtered,
            "y": y_filtered,
            "azimuth": self.ema_azimuth,
            "timestamp": current_time,
        }
