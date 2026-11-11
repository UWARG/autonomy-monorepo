# Behaviour tree reporter: numbers every node in the mission tree and saves the
# table to a file that is copied to the ground before takeoff. In flight, only
# the number of the running node is sent over STATUSTEXT ("S,<number>").

from pathlib import Path

import rclpy
from rclpy.node import Node
from py_trees_ros_interfaces.msg import BehaviourTree
from py_trees_ros_interfaces.srv import OpenSnapshotStream

from engine.ground_log import send_to_ground


class BehaviourTreeReporter(Node):
    
    # Mounted to airside/log/ on the host
    TREE_FILE = Path("/ros_ws/log/behaviour_tree.txt")

    def table_to_lines(self, table):
        # One line per node: number, parent, name
        return [f"T,{number},{parent},{name}" for number, parent, name in table]

    def __init__(self):
        super().__init__("behaviour_tree_reporter")

        # maps a node UUID as bytes so it can be a key (our node id our number)
        self._ids: dict[bytes, int] = {}
        self._parents: dict[int, int] = {}
        self._table: list[tuple[int, int, str]] = []
        self._last_running_number: int | None = None
        self._sub = None  # created once we know the real topic name

        self._client = self.create_client(
            OpenSnapshotStream, "/engine_manager/snapshot_streams/open"
        )
        # keep trying until the engine's tree is up
        self._open_timer = self.create_timer(2.0, self._open_snapshot_stream)

        # resend the running node for ground stations that connect late
        self.create_timer(30.0, self._resend_running)

    def _open_snapshot_stream(self) -> None:
        if not self._client.service_is_ready():
            self.get_logger().info(
                "waiting for the engine's snapshot stream service",
                throttle_duration_sec=10.0,
            )
            return

        self._open_timer.cancel()  # only open the stream once

        request = OpenSnapshotStream.Request()
        request.topic_name = ""
        request.parameters.snapshot_period = 0.5  # seconds between snapshots
        request.parameters.blackboard_data = False
        request.parameters.blackboard_activity = False
        # once it replies run the callback to subscribe to the real topic name
        future = self._client.call_async(request)
        future.add_done_callback(self._on_stream_opened)

    def _on_stream_opened(self, future) -> None:
        # response is the topic name to subscribe to for the snapshots
        topic_name = future.result().topic_name
        self.get_logger().info(f"subscribing to snapshot topic: {topic_name}")
        self._sub = self.create_subscription(
            BehaviourTree, topic_name, self._on_snapshot, 10
        )

    def _depth(self, number: int) -> int:  # how far a behaviour is from the root
        depth = 0
        while number in self._parents:
            parent = self._parents[number]
            if parent == -1:
                break
            depth += 1
            number = parent
        return depth

    def _resend_running(self) -> None:
        if self._last_running_number is not None:
            send_to_ground(self, f"S,{self._last_running_number}")

    def _save_table(self) -> None:
        self.TREE_FILE.parent.mkdir(parents=True, exist_ok=True)
        self.TREE_FILE.write_text("\n".join(self.table_to_lines(self._table)) + "\n")
        self.get_logger().info(
            f"saved tree table ({len(self._table)} nodes) to {self.TREE_FILE}"
        )

    def _on_snapshot(self, msg: BehaviourTree) -> None:
        # First snapshot (or whenever we see a new node): rebuild the table
        # and save it for the ground.
        table_changed = False
        for behaviour in msg.behaviours:
            key = bytes(behaviour.own_id.uuid)
            if key not in self._ids:
                self._ids[key] = len(self._ids)
                table_changed = True

        if table_changed:
            self._table = []
            for behaviour in msg.behaviours:
                number = self._ids[bytes(behaviour.own_id.uuid)]
                parent_key = bytes(behaviour.parent_id.uuid)
                parent_number = self._ids.get(parent_key, -1)  # root has no parent, so -1
                self._table.append((number, parent_number, behaviour.name))
                self._parents[number] = parent_number
            self._save_table()

        # find whichever node is RUNNING and report it if it changed.
        running_numbers = [
            self._ids[bytes(behaviour.own_id.uuid)]
            for behaviour in msg.behaviours
            if behaviour.status == behaviour.RUNNING
        ]
        running_number = max(running_numbers, key=self._depth) if running_numbers else None

        if running_number is not None and running_number != self._last_running_number:
            self._last_running_number = running_number
            send_to_ground(self, f"S,{running_number}")


def main():
    rclpy.init()
    node = BehaviourTreeReporter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()