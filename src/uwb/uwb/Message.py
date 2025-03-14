from .Field import Field

class Message:
    def __init__(self, data: bytes, expected_size = 37):
        if len(data) != expected_size:
            raise ValueError(
                f"Invalid message length: expected {expected_size} bytes, got {len(data)}."
            )
        self.data: bytes = data
        self.parsed_fields: Dict[str, int] = {}
        self._parse()

    def checksum(self) -> int:
        checksum = 0
        for byte in self.data[:-1]:
            checksum ^= byte
        return checksum
    
    def _parse(self):
        fields_info = [
            ("message_header", Field(4, False)),
            ("packet_length",  Field(2, False)),
            ("sequence_id",    Field(2, False)),
            ("request_command",Field(2, False)),
            ("version_id",     Field(2, False)),
            ("anchor_id",      Field(4, False)),
            ("tag_id",         Field(4, False)),
            ("distance",       Field(4, False)),
            ("azimuth",        Field(2, True)),
            ("elevation",      Field(2, True)),
            ("tag_status",     Field(2, False)),
            ("batch_sn",       Field(2, False)),
            ("reserve",        Field(4, False)),
            ("xor_byte",       Field(1, False))
        ]

        self.parsed_fields: Dict[str, float] = {}
        offset = 0
        for name, field in fields_info:
            segment = self.data[offset: offset + field.length]
            value = field.decode(segment)
            self.parsed_fields[name] = value
            setattr(self, name, value)
            offset += field.length

        if self.message_header != 0xFFFFFFFF:
            raise ValueError(f"Invalid message header: {hex(self.message_header)}")
        
        computed_checksum = self.checksum()
        if computed_checksum != self.xor_byte:
            raise ValueError(
                f"Checksum mismatch: expected {hex(self.xor_byte)}, computed {hex(computed_checksum)}"
            )
    
    def __repr__(self):
        fields_str = ", ".join(f"{k}={hex(v) if isinstance(v, float) and v > 9 else v}"
                               for k, v in self.parsed_fields.items())
        return f"<Message {fields_str}>"
