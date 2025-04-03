import asyncio
import serial_asyncio
from ...SerialAsync import SerialAsync


async def main():
    print("Command sent. Send <SIGINT> to stop.")
    transport, protocol = await serial_asyncio.create_serial_connection(
        asyncio.get_running_loop(),
        SerialAsync,
        "/dev/tty.PL2303G-USBtoUART1130",
        baudrate=230400,
    )

    await asyncio.Future()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Exiting due to user interruption.")
