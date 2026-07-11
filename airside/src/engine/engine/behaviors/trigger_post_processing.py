from __future__ import annotations

import py_trees
import rclpy.node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from std_msgs.msg import Empty


class TriggerPostProcessing(py_trees.behaviour.Behaviour):
    """
    Publishes the topic ``trigger_post_processing`` to signal
    post processing should begin.

    The topic is latched (transient local) so a subscriber that joins
    late still receives the trigger. Publishes once and returns SUCCESS.
    """

    TRIGGER_TOPIC = "trigger_post_processing"

    def __init__(self, name: str = "TriggerPostProcessing") -> None:
        super().__init__(name=name)

    def setup(self, **kwargs: rclpy.node.Node) -> None:
        self._node = kwargs["node"]

        self._trigger_pub = self._node.create_publisher(
            msg_type=Empty,
            topic=self.TRIGGER_TOPIC,
            qos_profile=QoSProfile(
                depth=1,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )

    def update(self) -> py_trees.common.Status:
        self._trigger_pub.publish(Empty())
        self._node.get_logger().info(
            f"{self.name}: published on '{self.TRIGGER_TOPIC}'"
        )
        return py_trees.common.Status.SUCCESS
