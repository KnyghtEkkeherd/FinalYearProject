import sys
import asyncio
import serial_asyncio
import math
import matplotlib.pyplot as plt
import time
from SerialAsync import SerialAsync
import numpy as np
from scipy.signal import savgol_filter

positions_x = []
positions_y = []
azimuth = []
timestamps = []


kf_x = {"x": 0, "p": 1}
kf_y = {"x": 0, "p": 1}
kf_q = 0.01
kf_r = 0.1


alpha = 0.2
ema_azimuth = None

plt.ion()
fig, (ax_scatter, ax_x, ax_y, ax_a) = plt.subplots(4, 1, figsize=(10, 12))


def kalman_filter(kf, measurement):
    kf["p"] += kf_q

    k = kf["p"] / (kf["p"] + kf_r)
    kf["x"] += k * (measurement - kf["x"])
    kf["p"] *= 1 - k
    return kf["x"]


def exponential_moving_average(ema, new_value, alpha):
    if ema is None:
        return new_value
    return alpha * new_value + (1 - alpha) * ema


def update_plot():
    n = min(len(positions_x), len(positions_y), len(timestamps), len(azimuth))
    raw_x = positions_x[-n:]
    raw_y = positions_y[-n:]
    time_data = timestamps[-n:]
    raw_az_plot = azimuth[-n:]

    if len(raw_x) > 5:
        smooth_x = savgol_filter(raw_x, 5, 2)
        smooth_y = savgol_filter(raw_y, 5, 2)
    else:
        smooth_x = raw_x
        smooth_y = raw_y

    ax_scatter.clear()
    ax_scatter.scatter(smooth_y, smooth_x, color="blue", label="Smoothed")
    ax_scatter.set_xlabel("X (m)")
    ax_scatter.set_ylabel("Y (m)")
    ax_scatter.set_title("Position Plot")
    ax_scatter.grid(True)
    ax_scatter.axis("equal")
    ax_scatter.legend()

    ax_x.clear()
    ax_x.plot(time_data, raw_x, marker="o", linestyle="-", color="red", label="Raw")
    ax_x.plot(time_data, smooth_x, linestyle="--", color="blue", label="Smoothed")
    ax_x.set_title("X Position Over Time")
    ax_x.set_xlabel("Time (s)")
    ax_x.set_ylabel("X (m)")
    ax_x.grid(True)
    ax_x.legend()

    ax_y.clear()
    ax_y.plot(time_data, raw_y, marker="o", linestyle="-", color="green", label="Raw")
    ax_y.plot(time_data, smooth_y, linestyle="--", color="blue", label="Smoothed")
    ax_y.set_title("Y Position Over Time")
    ax_y.set_xlabel("Time (s)")
    ax_y.set_ylabel("Y (m)")
    ax_y.grid(True)
    ax_y.legend()

    ax_a.clear()
    ax_a.plot(
        time_data, raw_az_plot, marker="o", linestyle="-", color="orange", label="Raw"
    )
    ax_a.set_title("Azimuth Over Time")
    ax_a.set_xlabel("Time (s)")
    ax_a.set_ylabel("Azimuth (θ)")
    ax_a.legend()
    ax_a.grid(True)

    plt.tight_layout()
    plt.draw()
    plt.pause(0.001)


def process_message(message):
    global ema_azimuth

    az = message.azimuth
    x = message.distance * math.cos(math.radians(az)) / 100.0
    y = message.distance * math.sin(math.radians(az)) / 100.0

    x_filtered = kalman_filter(kf_x, x)
    y_filtered = kalman_filter(kf_y, y)

    ema_azimuth = exponential_moving_average(ema_azimuth, az, alpha)

    positions_x.append(x_filtered)
    positions_y.append(y_filtered)
    azimuth.append(ema_azimuth)
    timestamps.append(time.time())

    print(
        f"Calculated position: az={az:.2f}, x={x_filtered:.2f} m, y={y_filtered:.2f} m"
    )
    update_plot()


async def run_serial_connection():
    print("Connecting to serial device...")
    transport, protocol = await serial_asyncio.create_serial_connection(
        asyncio.get_running_loop(),
        lambda: SerialAsync(process_message),
        "/dev/tty.PL2303G-USBtoUART130",
        baudrate=230400,
    )
    print("Serial connection established.")
    await asyncio.Future()


def main():
    try:
        asyncio.run(run_serial_connection())
    except KeyboardInterrupt:
        print("Interrupted by user. Exiting...")
        print(
            f"Raw Azimuth: mean={np.mean(azimuth):.2f}, std={np.std(azimuth):.2f}, var={np.var(azimuth):.2f}"
        )
        print(
            f"Raw X position: mean={np.mean(positions_x):.2f}, std={np.std(positions_x):.2f}, var={np.var(positions_x):.2f}"
        )
        print(
            f"Raw Y position: mean={np.mean(positions_y):.2f}, std={np.std(positions_y):.2f}, var={np.var(positions_y):.2f}"
        )
        sys.exit()
    finally:
        plt.ioff()
        plt.show()


if __name__ == "__main__":
    main()
