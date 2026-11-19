"""ROS-independent monitoring tests; no flight hardware or running nodes."""
import json
import tempfile
import unittest
from pathlib import Path

from dashboard_monitor.sensors import read_temperatures


class SensorTests(unittest.TestCase):
    def test_linux_cpu_gpu_and_unknown_sensors(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            for index, (label, value) in enumerate([
                ("CPU-therm", "48500"), ("GPU-therm", "51000"),
                ("battery", "40000"), ("cpu", "nan"), ("gpu", "invalid"),
            ]):
                zone = root / "class/thermal" / f"thermal_zone{index}"
                zone.mkdir(parents=True)
                (zone / "type").write_text(label)
                (zone / "temp").write_text(value)
            hwmon = root / "class/hwmon/hwmon0"
            hwmon.mkdir(parents=True)
            (hwmon / "name").write_text("coretemp")
            (hwmon / "temp1_label").write_text("Package id 0")
            (hwmon / "temp1_input").write_text("42000")
            result = read_temperatures(root)
            self.assertEqual([r["celsius"] for r in result["cpu"]], [48.5, 42])
            self.assertEqual(result["gpu"][0]["celsius"], 51)
            json.dumps(result, allow_nan=False)

    def test_missing_sensors(self):
        with tempfile.TemporaryDirectory() as tmp:
            self.assertEqual(read_temperatures(Path(tmp)), {"cpu": [], "gpu": []})
