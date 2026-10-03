"""
Behaviour tree reporter: numbers in every node in the mission tree, builds a table describing the tree
reports which node is running by the number 
"""
import py_trees
import rclpy
from rclpy.node import Node
from py_trees_ros_interfaces.msg import BehaviourTree
from py_trees_ros_interfaces.srv import OpenSnapshotStream

from engine.ground_log import send_to_ground

def local_tree_table (root: py_trees.behaviour.Behaviour):
    ids = {} #used to look up node number
    table = [] # sent to the ground, includes number, parent number, name of the node

    def visit(node, parent_number):
        number = len(table)
        ids[node.id] = number
        table.append((number, parent_number, node.name))
        for child in node.children:
            visit(child, number)

    visit(root, -1) # root has no parent, so -1
    return ids, table

def local_running_id(root: py_trees.behaviour.Behaviour, ids: dict) -> int | None:
    running = [
        ids[node.id]
        for node in root.iterate()
        if node.status == py_trees.common.Status.RUNNING
    ]
    return max(running) if running else None


def table_to_messages(table):
    # short text message per node that includes, the number, parent, and name 
    return [f"T,{number},{parent},{name}" for number, parent, name in table]

class BehaviourTreeReporter(Node):

    def __init__(self):
        super().__init__("behaviour_tree_reporter")
            
        #maps a node UUID as bytes so it can be a key (our node id our number)
        self._ids: dict[bytes, int] = {}
        self._parents: dict[int, int] = {}
        self._table: list[tuple[int, int, str]] = []
        self._last_running_number: int | None = None
        self._sub = None #created once we know the real topic name 

        # open the snapshot stream to get the behaviour tree snapshots
        self._open_snapshot_stream()

        # Resend the tree table periodically for ground stations that connect late
        self.create_timer(30.0, self._resend_table)
    


    def _open_snapshot_stream(self) -> None:
        client = self.create_client(OpenSnapshotStream, "/engine_manager/snapshot_streams/open")

        if not client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("Service not available: open_snapshot_stream")
            return

        request = OpenSnapshotStream.Request()
        request.topic_name = ""
        request.parameters.snapshot_period = 0.5 # 5 seconds it asks for updates -> 
        request.parameters.blackboard_data = False
        request.parameters.blackboard_activity = False
        # once it replies run the callback to subscribe to the real topic name
        future = client.call_async(request)
        future.add_done_callback(self._on_stream_opened)
    

    def _on_stream_opened(self, future) -> None:
        # response is the topic name to subscribe to for the snapshots
        response = future.result()
        topic_name = response.topic_name
        self.get_logger().info(f"subscribing to snapshot topic: {topic_name}")
        self._sub = self.create_subscription(
            BehaviourTree, topic_name, self._on_snapshot, 10
        )
    def _depth(self, number: int) -> int: # counts how far behavior is from root for parent relations 
        depth = 0
        while number in self._parents:
            parent = self._parents[number]
            if parent == -1: 
                break

            depth += 1
            number = parent

        return depth
    def _resend_table(self) -> None:
        if not self._table:
            return

        for message in table_to_messages(self._table):
            send_to_ground(self, message)

        if self._last_running_number is not None:
            send_to_ground(self, f"S,{self._last_running_number}")
            
    def _on_snapshot(self, msg: BehaviourTree) -> None:
        # First snapshot (or whenever we see a new node): (re)build the
        # table and send it to the ground.
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
            for message in table_to_messages(self._table):
                send_to_ground(self, message)

        # Now find whichever node is RUNNING and report it if it changed.
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