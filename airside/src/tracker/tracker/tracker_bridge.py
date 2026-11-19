# Subscribes to the LTE tracker on MQTT, republishes each fix as
# sensor_msgs/NavSatFix on /tracker/location, and keeps an always-valid
# New csv per session ID, tracker_<session>.csv
# (timestamp,latitude,longitude; sorted, no duplicates).

import csv
import json
import os
import queue
from datetime import datetime, timezone

import paho.mqtt.client as mqtt
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix


def parse_payload(payload: bytes):
    """Return (session_id, sequence, unix_time, lat, lon), or None to skip."""
    d = json.loads(payload)
    if not d.get("fix") or d.get("latitude") is None or d.get("gnss_time") is None:
        return None                      # no GPS lock yet
    if d["sequence"] <= 0:
        return None                      # debug-page test sample
    t = datetime.fromisoformat(d["gnss_time"].replace("Z", "+00:00")).timestamp()
    return d["session_id"], d["sequence"], t, float(d["latitude"]), float(d["longitude"])


def to_iso(t: float) -> str:
    return datetime.fromtimestamp(t, tz=timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z")

def load_csv(path: str) -> dict[str, tuple[float, float, float]]:
    """Read one session's CSV back in so a restarted node continues it.
    Returns {iso_timestamp: (unix_time, lat, lon)}. Missing file -> empty."""
    fixes = {}
    if not os.path.exists(path):
        return fixes
    with open(path, newline="") as f:
        reader = csv.reader(f)
        next(reader, None)  # skip header
        for row in reader:
            try:
                iso, lat, lon = row[0], float(row[1]), float(row[2])
                t = datetime.fromisoformat(iso.replace("Z", "+00:00")).timestamp()
            except (ValueError, IndexError):
                continue  # skip a half-written or malformed line
            fixes[iso] = (t, lat, lon)
    return fixes

class TrackerBridge(Node):
    def __init__(self):
        super().__init__("tracker_bridge")
        self.declare_parameter("broker_host", "broker.emqx.io")
        self.declare_parameter("broker_port", 1883)
        self.declare_parameter("session_id", "+")  # "+" = any tracker
        self.declare_parameter("data_dir", "/ros_ws/data")

        host = self.get_parameter("broker_host").value
        port = self.get_parameter("broker_port").value
        session = self.get_parameter("session_id").value
        self.data_dir = self.get_parameter("data_dir").value
        self.topic = f"warg/efs-lte-tracker/{session}/location"

        self.pub = self.create_publisher(NavSatFix, "/tracker/location", 10)
        # session id -> {iso timestamp: (t, lat, lon)}; each session loaded from its file on first sight
        self.sessions: dict[str, dict[str, tuple[float, float, float]]] = {}
        self.inbox: queue.Queue = queue.Queue()

        # paho runs on its own thread; hand messages to ROS through a queue
        self.mqtt = mqtt.Client(
            mqtt.CallbackAPIVersion.VERSION2,
            client_id="warg-airside-tracker-bridge",  # fixed and unique: lets the broker queue messages for us
            clean_session=False,
        )
        self.mqtt.on_connect = self._on_connect
        self.mqtt.on_message = lambda c, u, msg: self.inbox.put(msg)
        self.mqtt.connect_async(host, port, keepalive=30)
        self.mqtt.loop_start()  # also reconnects automatically

        self.create_timer(0.05, self._drain)
        self.get_logger().info(f"listening on {host}:{port} {self.topic}, writing {self.data_dir}/tracker_<session>.csv")

    def _on_connect(self, client, userdata, flags, reason_code, properties):
        self.get_logger().info(f"MQTT connected ({reason_code})")
        client.subscribe(self.topic, qos=1)  # re-subscribe on every reconnect

    def _path(self, session: str) -> str:
        safe = "".join(c if c.isalnum() or c in "-_" else "_" for c in session)
        return os.path.join(self.data_dir, f"tracker_{safe}.csv")

    def _session(self, session: str) -> dict[str, tuple[float, float, float]]:
        # first message from a session: resume its file if one exists
        if session not in self.sessions:
            self.sessions[session] = load_csv(self._path(session))
            if self.sessions[session]:
                self.get_logger().info(f"resumed {len(self.sessions[session])} fixes for session {session}")
        return self.sessions[session]

    def _drain(self):
        changed: set[str] = set()
        while not self.inbox.empty():
            msg = self.inbox.get()
            try:
                parsed = parse_payload(msg.payload)
            except Exception as e:
                self.get_logger().warn(f"bad payload on {msg.topic}: {e}: {msg.payload[:200]!r}")
                continue
            if parsed is None:
                continue  # no GPS fix yet, or a test sample: skip quietly

            session, seq, t, lat, lon = parsed
            fixes = self._session(session)
            key = to_iso(t)
            if key in fixes:
                continue  # duplicate within this session (QoS 1 resend, or already in its reloaded CSV)

            out = NavSatFix()
            out.header.stamp.sec = int(t)
            out.header.stamp.nanosec = int((t % 1) * 1e9)
            out.header.frame_id = session
            out.latitude, out.longitude = lat, lon
            self.pub.publish(out)

            fixes[key] = (t, lat, lon)
            changed.add(session)
        for session in changed:
            self._write_csv(session)

    def _write_csv(self, session: str):
        # rewrite this session's file sorted, via temp file + fsync + rename
        os.makedirs(self.data_dir, exist_ok=True)
        path = self._path(session)
        tmp = path + ".tmp"
        with open(tmp, "w") as f:
            f.write("timestamp,latitude,longitude\n")
            for t, lat, lon in sorted(self.sessions[session].values()):
                f.write(f"{to_iso(t)},{lat:.7f},{lon:.7f}\n")
            f.flush()
            os.fsync(f.fileno())  # survive a power cut
        os.replace(tmp, path)

    def destroy_node(self):
        self.mqtt.loop_stop()
        self.mqtt.disconnect()
        super().destroy_node()


def main():
    rclpy.init()
    node = TrackerBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()