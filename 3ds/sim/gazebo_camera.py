"""Bridge the existing Harmonic camera to the isolated Humble dashboard graph.

Uses installed Gazebo Python bindings and Pillow on WSL, and existing rclpy in
Docker. No new dependency, simulation restart, flight command or network API.
"""
import argparse
import base64
import io
import json
import queue
import signal
import subprocess
import threading
import time

from PIL import Image as PILImage

DEFAULT_TOPIC = "/world/iris_runway/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image"


def jpeg_packet(message):
    """Validate observed Gazebo RGB_INT8 pixels, respecting padded row stride."""
    if message.pixel_format_type != 3:
        raise ValueError(f"Expected Gazebo RGB_INT8 (3), got {message.pixel_format_type}")
    width, height, step = message.width, message.height, message.step
    if not 0 < width * height <= 4_000_000 or step < width * 3:
        raise ValueError("Invalid Gazebo dimensions/stride")
    if len(message.data) < step * height:
        raise ValueError("Truncated Gazebo image")
    image = PILImage.frombytes("RGB", (width, height), message.data, "raw", "RGB", step)
    output = io.BytesIO()
    image.save(output, format="JPEG", quality=85)
    return {
        "version": 1, "sec": message.header.stamp.sec,
        "nanosec": message.header.stamp.nsec,
        "data": base64.b64encode(output.getvalue()).decode("ascii"),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--container", default="3ds-3ds-1")
    parser.add_argument("--topic", default=DEFAULT_TOPIC)
    parser.add_argument("--fps", type=float, default=2.0)
    args = parser.parse_args()
    if not 0 < args.fps <= 5:
        parser.error("fps must be greater than zero and at most five")
    from gz.transport13 import Node
    from gz.msgs10.image_pb2 import Image

    latest = queue.Queue(maxsize=1)
    stop = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    node = Node()

    def receive(message):
        try:
            latest.put_nowait(message)
        except queue.Full:
            try:
                latest.get_nowait()
            except queue.Empty:
                pass
            try:
                latest.put_nowait(message)
            except queue.Full:
                pass

    if not node.subscribe(Image, args.topic, receive):
        raise SystemExit(f"Could not subscribe to Gazebo topic {args.topic}")
    command = [
        "docker", "exec", "-i", "-e", "ROS_DOMAIN_ID=73", args.container,
        "/entrypoint.sh", "python3", "-u",
        "/monorepo/3ds/sim/ros_camera_receiver.py",
    ]
    receiver = subprocess.Popen(command, stdin=subprocess.PIPE)
    count = 0
    try:
        while not stop.is_set():
            if receiver.poll() is not None:
                raise RuntimeError(f"ROS image receiver exited: {receiver.returncode}")
            try:
                message = latest.get(timeout=1.0)
            except queue.Empty:
                continue
            started = time.monotonic()
            packet = jpeg_packet(message)
            receiver.stdin.write((json.dumps(packet) + "\n").encode())
            receiver.stdin.flush()
            count += 1
            if count == 1 or count % 20 == 0:
                print(f"Gazebo rendered frames forwarded: {count}; {message.width}x{message.height}; source stamp {packet['sec']}.{packet['nanosec']:09d}", flush=True)
            stop.wait(max(0, 1.0 / args.fps - (time.monotonic() - started)))
    finally:
        receiver.stdin.close()
        try:
            receiver.wait(timeout=5)
        except subprocess.TimeoutExpired:
            receiver.terminate()
            receiver.wait(timeout=5)


if __name__ == "__main__":
    main()
