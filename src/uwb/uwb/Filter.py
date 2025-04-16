import math
import time
from collections import deque

import numpy as np
import matplotlib.pyplot as plt
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
from scipy.signal import savgol_filter


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

        plt.ion()
        self.fig, (self.ax_scatter, self.ax_x, self.ax_y, self.ax_a) = plt.subplots(
            4, 1, figsize=(10, 12)
        )
        plt.tight_layout()

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

    def update_plot(self):
        n = len(self.positions_x)
        max_points = 1000
        indices = (
            np.linspace(0, n - 1, max_points).astype(int)
            if n > max_points
            else np.arange(n)
        )
        traj_x = np.array(self.positions_x)[indices]
        traj_y = np.array(self.positions_y)[indices]
        current_x = self.positions_x[-1] if self.positions_x else 0
        current_y = self.positions_y[-1] if self.positions_y else 0
        current_az = np.deg2rad(self.azimuth_values[-1]) if self.azimuth_values else 0

        self.ax_scatter.clear()
        self.ax_scatter.plot(
            traj_x, traj_y, linestyle="-", color="lightblue", label="Trajectory"
        )
        self.ax_scatter.plot(current_x, current_y, "ro", markersize=8, label="Current")
        arrow_length = 0.5
        arrow_dx = arrow_length * np.cos(current_az)
        arrow_dy = arrow_length * np.sin(current_az)
        self.ax_scatter.arrow(
            current_x,
            current_y,
            arrow_dx,
            arrow_dy,
            head_width=0.1,
            head_length=0.1,
            fc="k",
            ec="k",
        )
        self.ax_scatter.set_xlabel("X (m)")
        self.ax_scatter.set_ylabel("Y (m)")
        self.ax_scatter.set_title("Position & Azimuth")
        self.ax_scatter.grid(True)
        self.ax_scatter.axis("equal")
        self.ax_scatter.legend()

        n_time = min(
            len(self.timestamps),
            len(self.positions_x),
            len(self.positions_y),
            len(self.azimuth_values),
        )
        time_data = np.array(self.timestamps)[-n_time:]
        raw_x = np.array(self.positions_x)[-n_time:]
        raw_y = np.array(self.positions_y)[-n_time:]
        raw_az = np.array(self.azimuth_values)[-n_time:]
        if len(raw_x) > 5:
            smooth_x = savgol_filter(raw_x, 5, 2)
            smooth_y = savgol_filter(raw_y, 5, 2)
        else:
            smooth_x, smooth_y = raw_x, raw_y

        self.ax_x.clear()
        self.ax_x.plot(
            time_data, raw_x, marker="o", linestyle="-", color="red", label="Raw X"
        )
        self.ax_x.plot(
            time_data, smooth_x, linestyle="--", color="blue", label="Smooth X"
        )
        self.ax_x.set_title("X vs Time")
        self.ax_x.set_xlabel("Time (s)")
        self.ax_x.set_ylabel("X (m)")
        self.ax_x.grid(True)
        self.ax_x.legend()

        self.ax_y.clear()
        self.ax_y.plot(
            time_data, raw_y, marker="o", linestyle="-", color="green", label="Raw Y"
        )
        self.ax_y.plot(
            time_data, smooth_y, linestyle="--", color="blue", label="Smooth Y"
        )
        self.ax_y.set_title("Y vs Time")
        self.ax_y.set_xlabel("Time (s)")
        self.ax_y.set_ylabel("Y (m)")
        self.ax_y.grid(True)
        self.ax_y.legend()

        self.ax_a.clear()
        self.ax_a.plot(
            time_data,
            raw_az,
            marker="o",
            linestyle="-",
            color="orange",
            label="Azimuth",
        )
        self.ax_a.set_title("Azimuth vs Time")
        self.ax_a.set_xlabel("Time (s)")
        self.ax_a.set_ylabel("Azimuth (°)")
        self.ax_a.legend()
        self.ax_a.grid(True)

        plt.tight_layout()
        plt.draw()
        plt.pause(0.001)
