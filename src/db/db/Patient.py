from Table import Table


class Patient(Table):
    name: str
    encodings: bytes

    def __init__(self, name):
        self.name = name
        self.encodings = b""
