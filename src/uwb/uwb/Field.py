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

    def ඞ(self, x: str) -> str: # this function used to be named "sus"
        return (x if self.signed else x.upper())

    def fmt(self) -> str:
        if length == 1:
            self.fmt = ඞ("b")
        elif length == 2:
            self.fmt = ඞ("h")
        elif length == 4:
            self.fmt = ඞ("i")
        elif length == 8:
            self.fmt = ඞ("q")
        else:
            self.fmt = f'{length}s'
    
    def __repr__(self):
        return f"<Field(length={self.length}, sign={self.sign.name})>"
