import sys
import asyncio
import serial_asyncio
import math
import time
import numpy as np
import matplotlib.pyplot as plt
from collections import deque
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
from scipy.signal import savgol_filter
from SerialAsync import SerialAsync

positions_x = []
positions_y = []
azimuth_values = []
timestamps = []
alpha = 0.2
ema_azimuth = None
velocity_buffer = deque(maxlen=5)
last_time = None
dt = 0.1

kf = KalmanFilter(dim_x=4, dim_z=2)
kf.F = np.array([[1, 0, dt, 0], [0, 1, 0, dt], [0, 0, 1, 0], [0, 0, 0, 1]])
kf.H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]])
kf.x = np.array([0, 0, 0, 0])
kf.P = np.eye(4) * 1000
kf.R = np.eye(2) * 1.0
kf.Q = Q_discrete_white_noise(dim=4, dt=dt, var=1e-6)


def exponential_moving_average(old_value, new_value, alpha):
    if old_value is None:
        return new_value
    return alpha * new_value + (1 - alpha) * old_value


def process_message(message):
    global ema_azimuth, last_time
    current_time = time.time()
    dt_local = (
        (current_time - last_time)
        if (last_time is not None and current_time > last_time)
        else dt
    )
    az = message.azimuth
    x_meas = message.distance * math.cos(math.radians(az)) / 100.0
    y_meas = message.distance * math.sin(math.radians(az)) / 100.0
    if positions_x and positions_y:
        prev_x = positions_x[-1]
        prev_y = positions_y[-1]
        displacement = math.hypot(x_meas - prev_x, y_meas - prev_y)
        current_velocity = displacement / dt_local
    else:
        current_velocity = 0.0
    velocity_buffer.append(current_velocity)
    if len(velocity_buffer) >= 3:
        median_velocity = np.median(list(velocity_buffer))
        factor = 2.5
        min_threshold = 0.05
        allowed_threshold = max(median_velocity * factor, min_threshold)
    else:
        allowed_threshold = float("inf")
    if current_velocity > allowed_threshold:
        if positions_x and positions_y:
            x_valid = positions_x[-1]
            y_valid = positions_y[-1]
        else:
            x_valid = x_meas
            y_valid = y_meas
    else:
        x_valid = x_meas
        y_valid = y_meas
    kf.predict()
    kf.update(np.array([x_valid, y_valid]))
    x_filtered, y_filtered = kf.x[0], kf.x[1]
    ema_azimuth = exponential_moving_average(ema_azimuth, az, alpha)
    positions_x.append(x_filtered)
    positions_y.append(y_filtered)
    azimuth_values.append(ema_azimuth)
    timestamps.append(current_time)
    last_time = current_time
    print(
        f"Filtered position: x: {x_filtered:.2f} m, y: {y_filtered:.2f} m, az: {ema_azimuth:.2f}°"
    )


plt.ion()
fig, (ax_scatter, ax_x, ax_y, ax_a) = plt.subplots(4, 1, figsize=(10, 12))


def update_plot():
    n = len(positions_x)
    max_points = 1000
    indices = (
        np.linspace(0, n - 1, max_points).astype(int)
        if n > max_points
        else np.arange(n)
    )
    traj_x = np.array(positions_x)[indices]
    traj_y = np.array(positions_y)[indices]
    if positions_x:
        current_x = positions_x[-1]
        current_y = positions_y[-1]
    else:
        current_x, current_y = 0, 0
    if azimuth_values:
        current_az = np.deg2rad(azimuth_values[-1])
    else:
        current_az = 0
    ax_scatter.clear()
    ax_scatter.plot(
        traj_x, traj_y, linestyle="-", color="lightblue", label="Trajectory"
    )
    ax_scatter.plot(current_y, current_x, "ro", markersize=8, label="Current")
    arrow_length = 0.5
    arrow_dx = arrow_length * np.cos(current_az)
    arrow_dy = arrow_length * np.sin(current_az)
    ax_scatter.arrow(
        current_x,
        current_y,
        arrow_dx,
        arrow_dy,
        head_width=0.1,
        head_length=0.1,
        fc="k",
        ec="k",
    )
    ax_scatter.set_xlabel("X (m)")
    ax_scatter.set_ylabel("Y (m)")
    ax_scatter.set_title("Position & Azimuth")
    ax_scatter.grid(True)
    ax_scatter.axis("equal")
    ax_scatter.legend()
    n_time = min(
        len(timestamps), len(positions_x), len(positions_y), len(azimuth_values)
    )
    time_data = np.array(timestamps)[-n_time:]
    raw_x = np.array(positions_x)[-n_time:]
    raw_y = np.array(positions_y)[-n_time:]
    raw_az = np.array(azimuth_values)[-n_time:]
    if len(raw_x) > 5:
        smooth_x = savgol_filter(raw_x, 5, 2)
        smooth_y = savgol_filter(raw_y, 5, 2)
    else:
        smooth_x, smooth_y = raw_x, raw_y
    ax_x.clear()
    ax_x.plot(time_data, raw_x, marker="o", linestyle="-", color="red", label="Raw X")
    ax_x.plot(time_data, smooth_x, linestyle="--", color="blue", label="Smooth X")
    ax_x.set_title("X vs Time")
    ax_x.set_xlabel("Time (s)")
    ax_x.set_ylabel("X (m)")
    ax_x.grid(True)
    ax_x.legend()
    ax_y.clear()
    ax_y.plot(time_data, raw_y, marker="o", linestyle="-", color="green", label="Raw Y")
    ax_y.plot(time_data, smooth_y, linestyle="--", color="blue", label="Smooth Y")
    ax_y.set_title("Y vs Time")
    ax_y.set_xlabel("Time (s)")
    ax_y.set_ylabel("Y (m)")
    ax_y.grid(True)
    ax_y.legend()
    ax_a.clear()
    ax_a.plot(
        time_data, raw_az, marker="o", linestyle="-", color="orange", label="Azimuth"
    )
    ax_a.set_title("Azimuth vs Time")
    ax_a.set_xlabel("Time (s)")
    ax_a.set_ylabel("Azimuth (°)")
    ax_a.legend()
    ax_a.grid(True)
    plt.tight_layout()
    plt.draw()
    plt.pause(0.001)


async def plot_updater():
    while True:
        update_plot()
        await asyncio.sleep(0.2)


async def run_serial_connection():
    transport, protocol = await serial_asyncio.create_serial_connection(
        asyncio.get_running_loop(),
        lambda: SerialAsync(process_message),
        "/dev/tty.PL2303G-USBtoUART10",
        baudrate=230400,
    )
    asyncio.create_task(plot_updater())
    await asyncio.Future()


def main():
    try:
        asyncio.run(run_serial_connection())
    except KeyboardInterrupt:
        print("Interrupted")
        sys.exit()
    finally:
        plt.ioff()
        plt.show()


if __name__ == "__main__":
    main()
