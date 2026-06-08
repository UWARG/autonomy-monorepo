"""
Decodes incoming JSON bytes from airside_comms into typed msgspec Structs.
Raises msgspec.DecodeError if the message doesn't match the expected schema.
"""