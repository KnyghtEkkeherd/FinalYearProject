import asyncio
from .Message import Message


def default(msg: Message):
    print(msg)


class SerialAsync(asyncio.Protocol):
    def __init__(self, callback=default):
        self.buffer = bytearray()
        self.callback = callback

    def conn_made(self, transport):
        print("Serial port opened.")
        transport.write(b"\x20\x01")

    def data_received(self, data):
        self.buffer.extend(data)
        while len(self.buffer) >= 37:
            msg_bytes = self.buffer[:37]
            del self.buffer[:37]
            new_msg = Message(bytes(msg_bytes), 37)
            self.callback(new_msg)

    def conn_lost(self, exc):
        print("Serial port closed. Stopping event loop.")
        asyncio.get_event_loop().stop()
