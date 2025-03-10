import serial
from Message import Message

class Serial:
    def __init__(self, port, baudrate=230400, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS):
        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            stopbits=stopbits,
            bytesize=bytesize
        )
        self.curr_msg = []

    def send_command(self, command):
        self.ser.write(command)

    def receive_responses(self):
        response = self.ser.read()
        if response:
            self.curr_msg.append(int(response.hex(), 16))
            # print(f"Current message: {self.curr_msg}")

        if len(self.curr_msg) >= 37:
            new_msg = Message(bytes(self.curr_msg), 37)
            print(new_msg)
            self.curr_msg = []

    def close(self):
        self.ser.close()
        print('Serial port closed.')
