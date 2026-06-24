#!/usr/bin/env python3
"""Bridges Isaac ROS Visual SLAM topics to a remote Rerun viewer over TCP."""

import os
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import rerun as rr


class RerunVSLAMBridge(Node):
    def __init__(self):
        super().__init__("rerun_vslam_bridge")

        host = os.environ.get("RERUN_VIEWER_HOST", "127.0.0.1")
        rr.init("vslam", spawn=False)
        rr.connect_tcp(f"{host}:9876")
        self.get_logger().info(f"Streaming to Rerun viewer at {host}:9876")

        self.create_subscription(PointCloud2, "/visual_slam/vis/landmarks_cloud", self._landmarks_cb, 10)
        self.create_subscription(PointCloud2, "/visual_slam/vis/observations_cloud", self._observations_cb, 10)
        self.create_subscription(Path, "/visual_slam/tracking/slam_path", self._path_cb, 10)
        self.create_subscription(Odometry, "/visual_slam/tracking/odometry", self._odom_cb, 10)

    def _set_time(self, stamp):
        rr.set_time_seconds("ros_time", stamp.sec + stamp.nanosec * 1e-9)

    def _read_xyz(self, msg):
        raw = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        if not raw:
            return None
        return np.array([(p[0], p[1], p[2]) for p in raw], dtype=np.float32)

    def _landmarks_cb(self, msg):
        self._set_time(msg.header.stamp)
        pts = self._read_xyz(msg)
        if pts is not None:
            rr.log("vslam/map/landmarks", rr.Points3D(pts, colors=[100, 180, 255], radii=0.02))

    def _observations_cb(self, msg):
        self._set_time(msg.header.stamp)
        pts = self._read_xyz(msg)
        if pts is not None:
            rr.log("vslam/map/observations", rr.Points3D(pts, colors=[255, 150, 0], radii=0.015))

    def _path_cb(self, msg):
        if not msg.poses:
            return
        self._set_time(msg.header.stamp)
        strip = [[p.pose.position.x, p.pose.position.y, p.pose.position.z] for p in msg.poses]
        rr.log("vslam/path", rr.LineStrips3D([strip], colors=[0, 220, 0]))

    def _odom_cb(self, msg):
        self._set_time(msg.header.stamp)
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        rr.log(
            "vslam/pose",
            rr.Transform3D(
                translation=[p.x, p.y, p.z],
                rotation=rr.Quaternion(xyzw=[q.x, q.y, q.z, q.w]),
            ),
        )


def main():
    rclpy.init()
    node = RerunVSLAMBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
