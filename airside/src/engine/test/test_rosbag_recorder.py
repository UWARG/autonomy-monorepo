"""Exercise recorder lifecycle with real subprocesses, without requiring ROS."""

import json
import os
import sys
import tempfile
import time
import unittest
from pathlib import Path

from engine.rosbag_recorder import RosbagRecorder

FAKE_RECORDER = '''
import json, os, signal, sys, time
from pathlib import Path
mode = sys.argv[1]
args = sys.argv[2:]
if mode == "fail":
    print("storage unavailable", flush=True)
    sys.exit(3)
bag = Path(args[args.index("--output") + 1])
bag.mkdir()
(bag / "arguments.json").write_text(json.dumps(args))
(bag / "pid").write_text(str(os.getpid()))
def stop(*_):
    if mode == "hang":
        return
    (bag / "metadata.yaml").write_text("finalized: true")
    sys.exit(0)
signal.signal(signal.SIGINT, stop)
if mode != "slow":
    (bag / "recording_0.db3").touch()
if mode == "crash":
    time.sleep(.1)
    sys.exit(4)
while True:
    time.sleep(.01)
'''


class RecorderTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.script = self.root / "recorder.py"
        self.script.write_text(FAKE_RECORDER)
        self.recorders = []

    def tearDown(self):
        for recorder in self.recorders:
            recorder.close()
        self.temp.cleanup()

    def recorder(self, mode="normal", topics=()):
        recorder = RosbagRecorder(
            self.root / "bags", topics=topics,
            command=(sys.executable, str(self.script), mode),
            startup_timeout=.5, stop_timeout=.15,
        )
        self.recorders.append(recorder)
        return recorder

    def test_start_stop_restart_finalizes_distinct_bags(self):
        recorder = self.recorder()
        self.assertTrue(recorder.set_recording(True).success)
        first = Path(recorder.status()["bag_path"])
        self.assertTrue(recorder.status()["recording"])
        self.assertTrue(recorder.set_recording(False).success)
        self.assertFalse(recorder.status()["recording"])
        self.assertTrue((first / "metadata.yaml").exists())
        self.assertTrue(recorder.set_recording(True).success)
        self.assertNotEqual(str(first), recorder.status()["bag_path"])

    def test_duplicate_start_keeps_the_same_recording(self):
        recorder = self.recorder()
        recorder.set_recording(True)
        first = recorder.status()["bag_path"]
        self.assertTrue(recorder.set_recording(True).success)
        self.assertEqual(first, recorder.status()["bag_path"])
        bags = [path for path in (self.root / "bags").glob("bag_*") if path.is_dir()]
        self.assertEqual(1, len(bags))

    def test_explicit_topics_replace_all_topics_argument(self):
        recorder = self.recorder(topics=("/heartbeat", "/camera/image_raw"))
        self.assertTrue(recorder.set_recording(True).success)
        arguments = json.loads((Path(recorder.status()["bag_path"]) / "arguments.json").read_text())
        self.assertNotIn("--all", arguments)
        self.assertIn("/heartbeat", arguments)
        self.assertIn("/camera/image_raw", arguments)

    def test_start_failure_is_reported_without_recording(self):
        recorder = self.recorder("fail")
        result = recorder.set_recording(True)
        self.assertFalse(result.success)
        self.assertFalse(recorder.status()["recording"])
        self.assertIn("storage unavailable", result.message)

    def test_missing_command_is_reported_without_recording(self):
        recorder = RosbagRecorder(self.root / "bags", command=("/missing/ros2",))
        self.recorders.append(recorder)
        self.assertFalse(recorder.set_recording(True).success)
        self.assertFalse(recorder.status()["recording"])

    def test_unexpected_exit_clears_recording_and_reports_error(self):
        recorder = self.recorder("crash")
        self.assertTrue(recorder.set_recording(True).success)
        time.sleep(.2)
        status = recorder.status()
        self.assertFalse(status["recording"])
        self.assertIn("unexpectedly", status["error"])

    def test_stop_timeout_preserves_process_until_retry_or_shutdown(self):
        recorder = self.recorder("hang")
        recorder.set_recording(True)
        self.assertFalse(recorder.set_recording(False).success)
        self.assertTrue(recorder.status()["recording"])
        self.assertIn("timed out", recorder.status()["error"])

    def test_shutdown_stops_and_finalizes_active_recording(self):
        recorder = self.recorder()
        recorder.set_recording(True)
        bag = Path(recorder.status()["bag_path"])
        recorder.close()
        self.assertFalse(recorder.status()["recording"])
        self.assertTrue((bag / "metadata.yaml").exists())

    def test_startup_timeout_reaps_process_without_claiming_recording(self):
        recorder = self.recorder("slow")
        self.assertFalse(recorder.set_recording(True).success)
        self.assertFalse(recorder.status()["recording"])
        self.assertIn("startup timeout", recorder.status()["error"])
        pid = int((Path(recorder.status()["bag_path"]) / "pid").read_text())
        with self.assertRaises(ProcessLookupError):
            os.kill(pid, 0)

    def test_shutdown_reaps_a_recorder_that_ignores_stop(self):
        recorder = self.recorder("hang")
        recorder.set_recording(True)
        pid = int((Path(recorder.status()["bag_path"]) / "pid").read_text())
        recorder.close()
        with self.assertRaises(ProcessLookupError):
            os.kill(pid, 0)
        self.assertIn("incomplete", recorder.status()["error"])


if __name__ == "__main__":
    unittest.main()
