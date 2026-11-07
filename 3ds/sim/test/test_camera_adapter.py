"""Test the simulation adapter without ROS, Gazebo or flight commands."""
import base64
import io
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from PIL import Image
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from gazebo_camera import jpeg_packet


class CameraAdapterTests(unittest.TestCase):
    def frame(self, **values):
        data = dict(width=2, height=1, step=8, pixel_format_type=3,
                    data=bytes([255, 0, 0, 0, 255, 0, 99, 99]),
                    header=SimpleNamespace(stamp=SimpleNamespace(sec=123, nsec=456)))
        data.update(values)
        return SimpleNamespace(**data)

    def test_real_pixels_timestamp_and_padded_stride(self):
        packet = jpeg_packet(self.frame())
        self.assertEqual((packet['sec'], packet['nanosec']), (123, 456))
        image = Image.open(io.BytesIO(base64.b64decode(packet['data'])))
        self.assertEqual(image.size, (2, 1))
        self.assertEqual(image.format, 'JPEG')

    def test_rejects_bad_encoding_dimensions_and_truncated_data(self):
        for change in [dict(pixel_format_type=999), dict(width=0),
                       dict(step=1), dict(data=b'bad')]:
            with self.subTest(change=change), self.assertRaises(ValueError):
                jpeg_packet(self.frame(**change))
