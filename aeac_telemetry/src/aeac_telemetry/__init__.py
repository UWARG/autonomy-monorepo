"""Sends our drone's telemetry from a MAVLink stream to the AEAC server."""

import os

os.environ.setdefault("MAVLINK20", "1")
