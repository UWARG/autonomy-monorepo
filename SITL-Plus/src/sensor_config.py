"""Load sensor port configuration from sensor_ports.yaml"""

from pathlib import Path

import yaml

_CONFIG_PATH = Path(__file__).parent / "sensor_ports.yaml"
#assign file sensor_ports.yaml (find file yaml next to sensor_config.py)
# in that container wil be sensor_ports.yaml, right in the place volume to overlap

#open file yaml in "r" (read mode), encode through utf-8, put in f
with open(_CONFIG_PATH, "r", encoding="utf-8") as f:
    _cfg = yaml.safe_load(f)
#read from file f and convert in yaml into data related to python
#use safe load instead of load for safety

HOST = _cfg["host"]
GROUNDSIDE_OFFSET = _cfg["groundside_offset"]
CAMERA_PORTS = _cfg["camera_ports"]
RANGE_FINDER_PORTS = _cfg["range_finder_ports"]
# tạo đúng tên biến file py có
