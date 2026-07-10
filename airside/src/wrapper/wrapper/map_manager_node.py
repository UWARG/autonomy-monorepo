from __future__ import annotations

import json
import os
import shutil
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, String

from utils.src.enums import Colours
from utils.src.types import Coordinate, Target


def parse_target(payload: str) -> Target:
    """Deserialize a JSON payload from /capture/target_location into a Target.

    Expected schema:
        {"colour": "RED", "location": {"lat": 0.0, "lon": 0.0, "alt": 0.0}}
    """
    data = json.loads(payload)
    location = data["location"]
    return Target(
        colour=Colours[data["colour"]],
        location=Coordinate(
            lat=float(location["lat"]),
            lon=float(location["lon"]),
            alt=float(location["alt"]),
        ),
    )


class TargetLog:
    """
    Append-only JSONL log of received targets, archived on demand.
    """

    WORKING_FILENAME = "targets.jsonl"

    def __init__(self, data_dir: Path) -> None:
        self._data_dir = data_dir
        self._data_dir.mkdir(parents=True, exist_ok=True)
        self._working_file = self._data_dir / self.WORKING_FILENAME
        self._working_file.write_text("")

    @property
    def working_file(self) -> Path:
        return self._working_file

    def append(self, target: Target, stamp: str) -> None:
        record = {
            "stamp": stamp,
            "colour": target.colour.name,
            "lat": target.location.lat,
            "lon": target.location.lon,
            "alt": target.location.alt,
        }
        with self._working_file.open("a") as f:
            f.write(json.dumps(record) + "\n")

    def archive(self, now: datetime) -> Path:
        archive_file = (
            self._data_dir / f"targets_{now.strftime('%Y-%m-%dT%H-%M-%S')}.jsonl"
        )
        shutil.copyfile(self._working_file, archive_file)
        return archive_file


class MapManagerNode(Node):
    TARGET_TOPIC = "/capture/target_location"
    TRIGGER_TOPIC = "/trigger_post_processing"
    DEFAULT_DATA_DIR = "/ros_ws/data"

    def __init__(self) -> None:
        super().__init__("map_manager_node")

        data_dir = Path(os.environ.get("MAP_MANAGER_DATA_DIR", self.DEFAULT_DATA_DIR))
        self._log = TargetLog(data_dir)

        self._target_sub = self.create_subscription(
            String, self.TARGET_TOPIC, self._on_target, 10
        )
        self._trigger_sub = self.create_subscription(
            Empty, self.TRIGGER_TOPIC, self._on_trigger, 10
        )

        self.get_logger().info(
            f"Map manager ready - logging targets from '{self.TARGET_TOPIC}' "
            f"to '{self._log.working_file}'."
        )

    def _on_target(self, msg: String) -> None:
        try:
            target = parse_target(msg.data)
        except (KeyError, ValueError, json.JSONDecodeError) as error:
            self.get_logger().error(f"Dropping malformed target message: {error}")
            return

        self._log.append(target, stamp=datetime.now().isoformat())
        self.get_logger().info(f"Logged target: {target}")

    def _on_trigger(self, msg: Empty) -> None:
        archive_file = self._log.archive(datetime.now())
        self.get_logger().info(f"Post-processing triggered - saved '{archive_file}'.")
        # TODO: run post processing on the targets file.


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MapManagerNode()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
