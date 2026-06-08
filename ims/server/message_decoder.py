"""
Decodes incoming JSON bytes from airside_comms into typed msgspec Structs.
Raises msgspec.DecodeError if the message doesn't match the expected schema.
"""

import msgspec

from utils.messages import AirsideMessage

decoder = msgspec.json.Decoder(AirsideMessage)


def decode(data: bytes) -> AirsideMessage:
    return decoder.decode(data)