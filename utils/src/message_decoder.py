"""
Decodes incoming JSON bytes from airside_comms into typed msgspec Structs.
"""

import msgspec

from utils.messages import AirsideMessage

decoder = msgspec.json.Decoder(AirsideMessage)


def decode(data: bytes) -> AirsideMessage:
    return decoder.decode(data)