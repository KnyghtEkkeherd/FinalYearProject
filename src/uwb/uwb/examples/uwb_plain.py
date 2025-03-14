import time
from ... import Serial

def main():
    ser = Serial(port="/dev/tty.PL2303G-USBtoUART1130")

    try:
        ser.send_command(b"\x20\x01")
        print("Command sent. Send <SIGINT> to stop.")
        while(1):
            ser.receive_responses()
    except KeyboardInterrupt:
        print("User triggered shutdown. Exiting...")
    finally:
        ser.close()

if __name__ == "__main__":
    main()
