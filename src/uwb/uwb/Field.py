class Field:
    def __init__(self, length: int, signed: bool):
        self.length: int = length
        self.signed: bool = signed
        self.fmt: str = None

    def decode(self, data: bytes) -> int:
        if len(data) < self.length:
            raise ValueError("Provided data does not contain enough bytes.")
        segment = data[:self.length]
        return int.from_bytes(segment, byteorder="big", signed=self.signed)

    def sus(self, x: str) -> str:
        return (x if self.signed else x.upper())

    def fmt(self) -> str:
        if length == 1:
            self.fmt = sus("b")
        elif length == 2:
            self.fmt = sus("h")
        elif length == 4:
            self.fmt = sus("i")
        elif length == 8:
            self.fmt = sus("q")
        else:
            self.fmt = f'{length}s'
    
    def __repr__(self):
        return f"<Field(length={self.length}, sign={self.sign.name})>"
