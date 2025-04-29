import serial
import threading
import time
from .Message import Message

class SerialMT:
    def __init__(self, port, baudrate=230400, stopbits=serial.STOPBITS_ONE, 
                 bytesize=serial.EIGHTBITS, timeout=0.1, message_callback=None):
        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            stopbits=stopbits,
            bytesize=bytesize,
            timeout=timeout
        )
        self.buffer = bytearray()
        self.message_callback = message_callback
        self._running = True
        self._thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._thread.start()

    def send_command(self, command):
        self.ser.write(command)

    def _poll_loop(self):
        while self._running:
            try:
                available = self.ser.in_waiting
                if available:
                    data = self.ser.read(available)
                    self.buffer.extend(data)
                    while len(self.buffer) >= 37:
                        msg_bytes = self.buffer[:37]
                        del self.buffer[:37]
                        new_msg = Message(bytes(msg_bytes), 37)
                        if self.message_callback:
                            self.message_callback(new_msg)
                        else:
                            print(new_msg)
                time.sleep(0.05)
            except Exception as e:
                print("Error in serial polling loop:", e)
                time.sleep(0.1)

    def close(self):
        self._running = False
        self._thread.join()
        self.ser.close()
        print("Serial port closed.")
