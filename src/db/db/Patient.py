from Table import Table


class Patient(Table):
    name: str
    encodings: bytes

    def __init__(self, name):
        self.name = name
        self.encodings = b""

    def _blobify(filename: str):
        with open(filename, "rb") as file:
            blobData = file.read()
        return blobData
