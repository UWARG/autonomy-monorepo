"""Real ROS 2 service and bag-readback test; skipped outside a ROS environment."""

import importlib.util
import json
import os
import tempfile
import threading
import time
import unittest
from pathlib import Path
from unittest.mock import patch


@unittest.skipUnless(importlib.util.find_spec("rclpy"), "Requires sourced ROS 2 Humble")
class RosbagIntegrationTest(unittest.TestCase):
    @patch.dict(os.environ, {"ROS_DOMAIN_ID": "194"})
    def test_services_record_and_finalize_readable_bags(self):
        import rclpy
        import rosbag2_py
        from engine.rosbag_node import RosbagNode
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.node import Node
        from rclpy.serialization import deserialize_message
        from std_msgs.msg import String
        from std_srvs.srv import SetBool, Trigger

        with tempfile.TemporaryDirectory() as directory:
            rclpy.init(args=["--ros-args", "-p", f"output_directory:={directory}"])
            controller = RosbagNode()
            verifier = Node("rosbag_integration_test")
            publisher = verifier.create_publisher(String, "/rosbag_test", 10)
            verifier.create_timer(.1, lambda: publisher.publish(String(data="recording test")))
            command = verifier.create_client(SetBool, "/ims/rosbag/set_recording")
            query = verifier.create_client(Trigger, "/ims/rosbag/get_status")
            executor = MultiThreadedExecutor(num_threads=2)
            executor.add_node(controller)
            executor.add_node(verifier)
            thread = threading.Thread(target=executor.spin, daemon=True)
            thread.start()

            def call(client, request):
                self.assertTrue(client.wait_for_service(timeout_sec=5))
                future = client.call_async(request)
                deadline = time.monotonic() + 20
                while not future.done() and time.monotonic() < deadline:
                    time.sleep(.02)
                self.assertTrue(future.done(), "ROS service did not respond")
                response = future.result()
                self.assertTrue(response.success, response.message)
                return response

            try:
                call(command, SetBool.Request(data=True))
                first = json.loads(call(query, Trigger.Request()).message)["bag_path"]
                call(command, SetBool.Request(data=True))
                self.assertEqual(first, json.loads(call(query, Trigger.Request()).message)["bag_path"])
                time.sleep(2)
                call(command, SetBool.Request(data=False))
                status = json.loads(call(query, Trigger.Request()).message)
                self.assertFalse(status["recording"])
                self.assertTrue((Path(first) / "metadata.yaml").is_file())

                reader = rosbag2_py.SequentialReader()
                reader.open(
                    rosbag2_py.StorageOptions(uri=first, storage_id="sqlite3"),
                    rosbag2_py.ConverterOptions("cdr", "cdr"),
                )
                messages = []
                while reader.has_next():
                    topic, data, _timestamp = reader.read_next()
                    if topic == "/rosbag_test":
                        messages.append(deserialize_message(data, String).data)
                self.assertGreater(len(messages), 0, "Bag contains no test messages")
                self.assertTrue(all(message == "recording test" for message in messages))
                del reader

                call(command, SetBool.Request(data=True))
                second = json.loads(call(query, Trigger.Request()).message)["bag_path"]
                self.assertNotEqual(first, second)
                call(command, SetBool.Request(data=False))
                self.assertTrue((Path(second) / "metadata.yaml").is_file())
                print(f"Real ROS recording/readback passed: {len(messages)} test messages; restart finalized")
            finally:
                executor.shutdown(timeout_sec=5)
                thread.join(timeout=5)
                controller.destroy_node()
                verifier.destroy_node()
                rclpy.shutdown()


if __name__ == "__main__":
    unittest.main()
