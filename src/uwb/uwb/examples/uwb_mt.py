import time
import threading
from ...Serial import Serial


def reader_thread(ser: Serial):
    while True:
        ser.receive_responses()


def main():
    ser = Serial(port="/dev/tty.PL2303G-USBtoUART1130")

    thread = threading.Thread(target=reader_thread, args=(ser,), daemon=True)
    thread.start()

    try:
        ser.send_command(b"\x20\x01")
        print("Command sent. Send <SIGINT> to stop.")
        while 1:
            time.sleep(1)

    except KeyboardInterrupt:
        print("User triggered shutdown. Exiting...")

    finally:
        ser.close()


if __name__ == "__main__":
    main()
