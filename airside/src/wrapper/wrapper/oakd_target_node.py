"""OAK-D spatial ObjectTracker -> candidate and sticky TrackedTarget topics."""

from __future__ import annotations

from pathlib import Path
from typing import Optional

import rclpy
from airside_interfaces.msg import TrackedTarget
from camera.src.oakd_follow_pipeline import build_follow_pipeline
from camera.src.target_source import (
    DepthAITrackletProvider,
    RealTargetSource,
    TrackletPacket,
    observation_from_tracklet,
    select_closest_tracked,
)
from rclpy.node import Node
from std_msgs.msg import Empty

MM_PER_M = 1000.0


class OakDTargetNode(Node):
    CANDIDATE_TOPIC = "perception/target_candidate"
    TARGET_TOPIC = "perception/target"

    def __init__(self) -> None:
        super().__init__("oakd_target")
        self.declare_parameter("blob_path", "")
        self.declare_parameter("person_label", 15)
        self.declare_parameter("camera_fps", 20)
        self.declare_parameter("detector_stride", 1)
        blob_path = str(self.get_parameter("blob_path").value)
        person_label = int(self.get_parameter("person_label").value)
        camera_fps = int(self.get_parameter("camera_fps").value)
        detector_stride = int(self.get_parameter("detector_stride").value)
        if not blob_path or not Path(blob_path).is_file():
            raise RuntimeError(
                "oakd_target requires an existing 'blob_path' model file"
            )

        import depthai as dai

        self._device = dai.Device(
            build_follow_pipeline(
                blob_path,
                person_label=person_label,
                camera_fps=camera_fps,
                detector_stride=detector_stride,
            )
        )
        queue = self._device.getOutputQueue("tracklets", maxSize=4, blocking=False)
        detector_queue = self._device.getOutputQueue(
            "detector_frames", maxSize=4, blocking=False
        )
        self._provider = DepthAITrackletProvider(
            queue,
            detector_queue=detector_queue,
            host_sync_now_fn=dai.Clock.now,
        )
        self._source = RealTargetSource(poll_fn=self._consume_packet)
        self._latest_packet: Optional[TrackletPacket] = None
        self._latest_acquisition_packet: Optional[TrackletPacket] = None
        self._candidate_pub = self.create_publisher(
            TrackedTarget, self.CANDIDATE_TOPIC, 10
        )
        self._target_pub = self.create_publisher(TrackedTarget, self.TARGET_TOPIC, 10)
        self.create_subscription(Empty, "perception/acquire_target", self._acquire, 10)
        self.create_subscription(Empty, "perception/reset_target", self._reset, 10)
        self.create_timer(0.01, self._poll)
        detector_fps = camera_fps / detector_stride
        self.get_logger().info(
            "OAK-D spatial ObjectTracker ready; "
            f"camera={camera_fps} Hz detector={detector_fps:g} Hz "
            f"stride={detector_stride}; target ownership is disabled"
        )

    def _consume_packet(self):
        packet = self._latest_packet
        self._latest_packet = None
        return packet

    def _to_msg(self, observation) -> TrackedTarget:
        message = TrackedTarget()
        self._set_stamp(message.header.stamp, observation.capture_time_s)
        self._set_stamp(
            message.host_receipt_stamp,
            observation.received_time_s or observation.capture_time_s,
        )
        now = self.get_clock().now().nanoseconds * 1e-9
        self._set_stamp(message.publish_stamp, now)
        message.header.frame_id = "camera"
        message.position.x = observation.x_mm / MM_PER_M
        message.position.y = observation.y_mm / MM_PER_M
        message.position.z = observation.z_mm / MM_PER_M
        message.track_id = observation.track_id
        message.sequence_num = observation.sequence_num
        self._set_stamp(
            message.detector_stamp,
            observation.detector_capture_time_s or observation.capture_time_s,
        )
        message.detector_sequence_num = (
            observation.detector_sequence_num
            if observation.detector_sequence_num is not None
            else observation.sequence_num
        )
        message.detector_confirmed = observation.detector_confirmed
        message.within_validated_range = observation.within_validated_range
        return message

    @staticmethod
    def _set_stamp(stamp, value: float) -> None:
        seconds = int(value)
        stamp.sec = seconds
        stamp.nanosec = int((value - seconds) * 1e9)

    def _acquire(self, _message: Empty) -> None:
        packet = self._latest_acquisition_packet
        if packet is not None and self._source.enable(packet):
            self._latest_acquisition_packet = None
            self.get_logger().info(f"locked target ID {self._source.locked_track_id}")
        else:
            self.get_logger().warning(
                "target acquisition requested with no valid candidate"
            )

    def _reset(self, _message: Empty) -> None:
        old_id = self._source.locked_track_id
        self._source.reset_target()
        self.get_logger().warning(f"cleared target lock ID {old_id}")

    def _poll(self) -> None:
        packet = self._provider.poll()
        if packet is None:
            return
        self._latest_packet = packet
        if self._source.enabled:
            observation = self._source.get_target()
            if observation is not None:
                self._target_pub.publish(self._to_msg(observation))
            return
        if not packet.detector_confirmed:
            return
        self._latest_acquisition_packet = packet
        candidate = select_closest_tracked(packet.tracklets, "TRACKED")
        if candidate is not None:
            self._candidate_pub.publish(
                self._to_msg(observation_from_tracklet(candidate, packet))
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OakDTargetNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
