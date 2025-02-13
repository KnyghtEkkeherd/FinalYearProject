import serial
import time

def send_data(ser, data):
    ser.write(data)
    print(f"Sent: {data.hex()}")

def receive_data(ser):
    # Read the header first (4 bytes)
    header = ser.read(4)
    if len(header) < 4:
        print("Error: Incomplete header received")
        return None

    print(f"Received Header: {header.hex()}")

    # Extract Payload Length (last byte of the header)
    payload_length = header[3]
    print(f"Payload Length: {payload_length}")

    # Read the payload based on the length
    payload = ser.read(payload_length)
    if len(payload) < payload_length:
        print("Error: Incomplete payload received")
        return None

    print(f"Received Payload: {payload.hex()}")
    return header + payload

def main():
    port = "/dev/tty.usbmodemCC210983BA011"  # Replace with your TTY port
    baudrate = 115200

    # Initialize serial port
    ser = serial.Serial(port, baudrate, timeout=1)
    time.sleep(2)  # Give some time for initialization

    # Construct the CORE_GET_DEVICE_INFO command
    # MT=0b001 (Command), PBF=0b0 (Complete), GID=0x0, OID=0x02
    command = bytes([0b00100000, 0x02, 0x00, 0x00])  # MT + PBF + GID + OID
    send_data(ser, command)

    # Receive and process the response
    response = receive_data(ser)

    # Close the serial port
    ser.close()

    # Parse the response (if received)
    if response:
        status = response[4]  # First byte of the payload is the Status
        print(f"Device Info Response Status: {status}")
        # Additional parsing of the response payload can be added here

if __name__ == "__main__":
    main()