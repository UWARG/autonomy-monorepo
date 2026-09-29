#!/usr/bin/env python3
"""Run multiple ArduPilot controllers in one PyBullet world."""

import importlib
import logging
import os
import socket
import threading
from pathlib import Path

import pybullet as p
import rerun as rr

import sensor_ports
import state
from camera import Camera
from drone import Drone
from environment import Environment
from range_finder import Range_Finder


def main():
    """Collect controls from every drone, step once, then reply to each SITL."""
    logging.basicConfig(level=logging.INFO)
    sensor_host = os.environ["SENSOR_HOST"]
    hosts = os.getenv("DRONE_HOSTS", "sitl-plus").split(",")
    module, factory = os.getenv("DRONE_MODEL", "iris:Iris").split(":")
    vehicle_type = getattr(importlib.import_module(module), factory)
    state.dir_path = Path(__file__).resolve().parent / (
        "ardupilot/libraries/SITL/examples/JSON/pybullet/models"
    )
    world = Environment(int(os.getenv("SIM_RATE_HZ", "800")))
    rr.init("SITL-Plus")
    rr.connect_grpc("rerun+http://host.docker.internal:9876/proxy")
    try:
        with (
            socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock,
            socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sensor_socket,
        ):
            sock.bind(("", 9002))
            sock.settimeout(0.5)
            state.airside_socket = sensor_socket
            drones = []
            for index, host in enumerate(hosts):
                name = "drone" if len(hosts) == 1 else f"drone_{index + 1}"
                drone = Drone(
                    name,
                    host,
                    vehicle_type(position=(index * 3, 0, 0.2)),
                    sensor_host,
                    index * 10,
                )
                drones.append(drone)
                rr.log(
                    name,
                    rr.Boxes3D(
                        centers=[[0, 0, 0]],
                        half_sizes=[[1, 0.5, 0.2]],
                        colors=[[255, 0, 0]],
                        fill_mode="solid",
                    ),
                )
                prefix = "" if len(hosts) == 1 else name + "/"
                for sensor_type, ports, target in (
                    (Camera, sensor_ports.CAMERA_PORTS, "camera_thread"),
                    (Range_Finder, sensor_ports.RANGE_FINDER_PORTS, "range_thread"),
                ):
                    for port, config in ports.items():
                        config = dict(config, port=port + drone.port_offset)
                        sensor = sensor_type(
                            attached_to_object=drone.vehicle.robot_id,
                            sensor_host=sensor_host,
                            log_prefix=prefix,
                            **config,
                        )
                        threading.Thread(
                            target=getattr(sensor, target), daemon=True
                        ).start()
            world.load_scene()
            peers = {}
            frame_count = 0
            while True:
                try:
                    data, address = sock.recvfrom(100)
                except socket.timeout:
                    continue
                if address[0] not in peers:
                    # Resolve lazily: controller containers may start after physics.
                    for drone in drones:
                        try:
                            peers[socket.gethostbyname(drone.host)] = drone
                        except socket.gaierror:
                            continue
                drone = peers.get(address[0])
                if drone is None:
                    continue
                drone.receive(data, address, sock)
                if not all(drone.pending for drone in drones):
                    continue
                world.step(drones)
                frame_count += 1
                for drone in drones:
                    drone.send(sock, world.time_now, world.time_step)
                    if frame_count % 50 == 0:
                        pos, orn = p.getBasePositionAndOrientation(
                            drone.vehicle.robot_id
                        )
                        rr.log(
                            drone.name,
                            rr.Transform3D(
                                translation=pos, rotation=rr.Quaternion(xyzw=orn)
                            ),
                        )
    finally:
        world.close()


if __name__ == "__main__":
    main()
