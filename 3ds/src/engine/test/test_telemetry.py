"""Exercise tree snapshots without loading ROS or mission behavior code."""
import unittest
from types import SimpleNamespace
from engine.telemetry import snapshot


class TreeTests(unittest.TestCase):
    def test_preorder_and_tip(self):
        child = SimpleNamespace(id="b", name="Wait", status=SimpleNamespace(name="RUNNING"), children=[])
        root = SimpleNamespace(id="a", name="Root", status=SimpleNamespace(name="RUNNING"), children=[child], tip=lambda: child)
        self.assertEqual(snapshot(root, 12.5), {
            "version": 1, "stamp": 12.5, "tip": "b", "nodes": [
                {"id": "a", "parent": None, "name": "Root", "status": "RUNNING", "feedback": ""},
                {"id": "b", "parent": "a", "name": "Wait", "status": "RUNNING", "feedback": ""},
            ],
        })
        root.tip = lambda: None
        self.assertIsNone(snapshot(root, 13)["tip"])
