import sys
import asyncio
import serial_asyncio
import math
import matplotlib.pyplot as plt
import time
from SerialAsync import SerialAsync
import numpy as np

positions_x = []
positions_y = []
azimuth = []
timestamps = []

plt.ion()
fig, (ax_scatter, ax_x, ax_y, ax_a) = plt.subplots(4, 1, figsize=(10, 12))


def update_plot():
    n = min(len(positions_x), len(positions_y), len(timestamps), len(azimuth))
    raw_x = positions_x[-n:]
    raw_y = positions_y[-n:]
    time_data = timestamps[-n:]
    raw_az_plot = azimuth[-n:]
    ax_scatter.clear()
    ax_scatter.scatter(raw_y, raw_x, color="blue", label="Raw")
    ax_scatter.set_xlabel("X (m)")
    ax_scatter.set_ylabel("Y (m)")
    ax_scatter.set_title("Position Plot")
    ax_scatter.grid(True)
    ax_scatter.axis("equal")
    ax_scatter.legend()
    ax_x.clear()
    ax_x.plot(time_data, raw_x, marker="o", linestyle="-", color="red", label="Raw")
    ax_x.set_title("X Position Over Time")
    ax_x.set_xlabel("Time (s)")
    ax_x.set_ylabel("X (m)")
    ax_x.grid(True)
    ax_x.legend()
    ax_y.clear()
    ax_y.plot(time_data, raw_y, marker="o", linestyle="-", color="green", label="Raw")
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
    az = message.azimuth
    x = message.distance * math.cos(math.radians(az)) / 100.0
    y = message.distance * math.sin(math.radians(az)) / 100.0
    positions_x.append(x)
    positions_y.append(y)
    azimuth.append(az)
    timestamps.append(time.time())
    print(f"Calculated position: az={az:.2f}, x={x:.2f} m, y={y:.2f} m")
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
