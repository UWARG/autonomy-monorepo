from time import monotonic

import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("diagnostic_updater")

from diagnostic_msgs.msg import DiagnosticArray
from engine.preflight import PREARM_CHECK
from engine.preflight_node import PreflightNode
from mavros_msgs.msg import State, SysStatus
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import Image, NavSatFix
from std_msgs.msg import Empty, Float64

from airside_interfaces.srv import CapturePhoto


def test_ros_readiness_startup_failure_recovery_and_timeout():
    rclpy.init()
    monitor = PreflightNode()
    source = rclpy.create_node("preflight_test_source")
    executor = SingleThreadedExecutor()
    executor.add_node(monitor)
    executor.add_node(source)
    results = []

    def receive(msg):
        results.extend(
            status
            for status in msg.status
            if status.name == "preflight_node: readiness"
        )

    subscription = source.create_subscription(
        DiagnosticArray, "/diagnostics", receive, 10
    )
    service = source.create_service(
        CapturePhoto, "/camera/capture", lambda request, response: response
    )
    publishers = {
        "ground": source.create_publisher(Empty, "/groundstation/heartbeat", 10),
        "heartbeat": source.create_publisher(Empty, "/heartbeat", 10),
        "gps": source.create_publisher(NavSatFix, "/mavros/global_position/global", 10),
        "alt": source.create_publisher(Float64, "/mavros/global_position/rel_alt", 10),
        "state": source.create_publisher(State, "/mavros/state", 10),
        "sys": source.create_publisher(SysStatus, "/mavros/sys_status", 10),
        "image": source.create_publisher(Image, "/camera/image_raw", 10),
    }
    connected = True
    healthy = True
    black = False
    send_ground = False

    def publish():
        if send_ground:
            publishers["ground"].publish(Empty())
        publishers["heartbeat"].publish(Empty())
        publishers["gps"].publish(NavSatFix())
        publishers["alt"].publish(Float64())
        publishers["state"].publish(State(connected=connected, armed=False))
        publishers["sys"].publish(
            SysStatus(
                sensors_present=PREARM_CHECK,
                sensors_enabled=PREARM_CHECK,
                sensors_health=PREARM_CHECK if healthy else 0,
            )
        )
        msg = Image(height=2, width=2, encoding="rgb8", step=6)
        msg.header.stamp = source.get_clock().now().to_msg()
        msg.data = bytes(12) if black else bytes([10] * 6 + [240] * 6)
        publishers["image"].publish(msg)

    def wait_for(predicate, timeout=8.0):
        deadline = monotonic() + timeout
        previous_count = len(results)
        while monotonic() < deadline:
            publish()
            executor.spin_once(timeout_sec=0.05)
            if len(results) > previous_count and predicate(results[-1]):
                return results[-1]
        pytest.fail(
            f"Readiness condition not met: {results[-1] if results else 'no diagnostics'}"
        )

    def ready(status):
        return status.level == 0 and any(
            value.key == "ready" and value.value == "true" for value in status.values
        )

    try:
        initial = wait_for(
            lambda status: not ready(status) and "groundstation" in status.message
        )
        assert not ready(initial)
        send_ground = True
        wait_for(ready)
        healthy = False
        wait_for(
            lambda status: (
                not ready(status) and "pre-arm checks failing" in status.message
            )
        )
        healthy = True
        wait_for(ready)
        black = True
        wait_for(lambda status: not ready(status) and "too dark" in status.message)
        black = False
        wait_for(ready)
        connected = False
        wait_for(lambda status: not ready(status) and "disconnected" in status.message)
        connected = True
        wait_for(ready)
        send_ground = False
        wait_for(
            lambda status: (
                not ready(status)
                and "groundstation: Last message age" in status.message
            )
        )
    finally:
        source.destroy_subscription(subscription)
        source.destroy_service(service)
        executor.shutdown()
        monitor.destroy_node()
        source.destroy_node()
        rclpy.try_shutdown()
