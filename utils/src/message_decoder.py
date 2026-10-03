"""
DEPRECATED — see utils/src/messages.py. Orphaned by the rosbridge migration (#124);
ims/server, its only consumer, no longer exists.

Decodes incoming JSON bytes from airside_comms into typed msgspec Structs.
"""

import msgspec

from utils.src.messages import AirsideMessage

decoder = msgspec.json.Decoder(AirsideMessage)


def decode(data: bytes) -> AirsideMessage:
    return decoder.decode(data)