import serial
from . import Message

class Serial:
    def __init__(self, port, baudrate=230400, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS, timeout=0.1):
        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            stopbits=stopbits,
            bytesize=bytesize,
            timeout=timeout
        )
        self.buffer = bytearray()

    def send_command(self, command):
        self.ser.write(command)

    def receive_responses(self):
        available = self.ser.in_waiting
        if available:
            data = self.ser.read(available)
            self.buffer.extend(data) 
            while len(self.buffer) >= 37:
                msg_bytes = self.buffer[:37]
                del self.buffer[:37]
                new_msg = Message(bytes(msg_bytes), 37)
                print(new_msg)

    def close(self):
        self.ser.close()
        print('Serial port closed.')
