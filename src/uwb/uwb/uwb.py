from Serial import Serial
import time

def main():
    ser = Serial(port="/dev/tty.PL2303G-USBtoUART1130")

    try:
        ser.send_command(b"\x20\x01")
        print("Command sent. Press \"q\" to stop.")

        while(1):
            ser.receive_responses()
            time.sleep(0.01)
    finally:
        ser.close()

if __name__ == "__main__":
    main()
