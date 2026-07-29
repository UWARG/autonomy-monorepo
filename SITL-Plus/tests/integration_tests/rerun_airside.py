"""Host-side MAVLink mission controller without groundside relay."""

import logging
import os
import socket
import struct
import threading
import time
from pathlib import Path

import cv2
import numpy as np
from pymavlink import mavutil

import constants
import sensor_ports

logging.basicConfig(level=logging.INFO)
PORT = 5761
TELEM_PORT = 4000
FRAME_RATE = constants.CAMERA_FPS
groundside_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
groundside_socket.settimeout(100)
telem_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
telem_socket.bind(("", TELEM_PORT))
telem_socket.settimeout(0.5)
frame_count = 0  # pylint: disable=invalid-name
PRINT_INTERVAL = 1000


def range_finder_thread(port):
    """Receive range finder packets from the simulation container."""
    range_finder_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    range_finder_socket.bind(("", port))
    range_finder_socket.settimeout(0.5)
    while True:
        try:
            data_range, _ = range_finder_socket.recvfrom(65535)
            _ = struct.unpack("f", data_range[:4])[0]
        except OSError:
            continue


def camera_thread(port):
    """Receive camera packets from the simulation container."""
    global frame_count
    airside_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    airside_socket.bind(("", port))
    airside_socket.settimeout(0.5)
    while True:
        try:
            data, _ = airside_socket.recvfrom(65535)
            if not data:
                logging.error("Received no data")
                continue
            if len(data) < 24:
                logging.error("Received bad data: %s", len(data))
                continue
            header = data[:24]
            rgb_length, depth_length, far, near = struct.unpack("QQff", header)
            if rgb_length == 0 or depth_length == 0 or far == 0 or near == 0:
                logging.error("Received bad data: %s %s", rgb_length, depth_length)
                continue
            rgb_data = data[24 : 24 + rgb_length]
            depth_data = data[24 + rgb_length : 24 + rgb_length + depth_length]
            rgb_array = np.frombuffer(rgb_data, np.uint8)
            rgb_image = cv2.imdecode(rgb_array, cv2.IMREAD_COLOR)
            depth_buffer = np.frombuffer(depth_data, np.uint8)
            depth_array = 0.01 * np.array(
                cv2.imdecode(depth_buffer, cv2.IMREAD_UNCHANGED)
            )
            middle = (rgb_image.shape[0] // 2, rgb_image.shape[1] // 2)
            middle_depth = float(depth_array[middle[0], middle[1]])
            if depth_length > 0:
                arr_min = np.min(depth_array)
                arr_max = np.max(depth_array)
                if arr_max - arr_min != 0:
                    _ = (depth_array - arr_min) / (arr_max - arr_min)
            if frame_count % PRINT_INTERVAL == 0:
                if middle_depth < 100:
                    logging.info(
                        "Received %s bytes of rgb data and %s and middle depth:%s",
                        rgb_length,
                        depth_length,
                        middle_depth,
                    )
                else:
                    logging.info("nothing within range of %s to %s", near, far)
        except OSError:
            continue


def telem_thread():
    """Receive telemetry packets from the simulation container."""
    while True:
        try:
            telem_data, _ = telem_socket.recvfrom(65535)
            if not telem_data:
                logging.error("Received no data")
                continue
            struct.unpack("ffffff", telem_data)
        except OSError:
            continue


def frame_counter():
    """Increment the shared frame counter."""
    global frame_count
    while True:
        time.sleep(1 / FRAME_RATE)
        frame_count += 1


def main():
    """Upload a mission and drive the vehicle through takeoff."""
    continue_flag = True
    threading.Thread(target=frame_counter, daemon=True).start()
    for _, value in sensor_ports.CAMERA_PORTS.items():
        thread_camera = threading.Thread(
            target=camera_thread, args=(value["port"],), daemon=True
        )
        thread_camera.start()
    for _, value in sensor_ports.RANGE_FINDER_PORTS.items():
        thread_range_finder = threading.Thread(
            target=range_finder_thread, args=(value["port"],), daemon=True
        )
        thread_range_finder.start()
    threading.Thread(target=telem_thread, daemon=True).start()
    conn = mavutil.mavlink_connection(f"tcp:127.0.0.1:{PORT}")
    conn.wait_heartbeat()
    print(f"Heartbeat from vehicle: {conn.target_system} {conn.target_component}")
    mission_file = os.path.abspath(
        Path.joinpath(
            Path(__file__).parent.parent.parent, "src", "mission_load.waypoints"
        )
    )
    lines = []
    with open(mission_file, "r", encoding="utf-8") as file:
        for line in file:
            if line.startswith("#") or line.startswith("QGC WPL"):
                continue
            file_line = line.split("\t")
            file_line[-1] = file_line[-1].strip("\n")
            lines.append(file_line)
    conn.mav.param_set_send(
        target_system=conn.target_system,
        target_component=conn.target_component,
        param_id=b"FRAME_TYPE",
        param_value=1,
        param_type=mavutil.mavlink.MAV_PARAM_TYPE_INT32,
    )
    conn.mav.param_set_send(
        target_system=conn.target_system,
        target_component=conn.target_component,
        param_id=b"FRAME_CLASS",
        param_value=1,
        param_type=mavutil.mavlink.MAV_PARAM_TYPE_INT32,
    )
    time.sleep(10)
    conn.mav.param_set_send(
        target_system=conn.target_system,
        target_component=conn.target_component,
        param_id=b"SIM_RATE_HZ",
        param_value=400,
        param_type=mavutil.mavlink.MAV_PARAM_TYPE_INT32,
    )
    while continue_flag:
        time.sleep(20)
        continue_flag = False
        conn.mav.mission_clear_all_send(
            target_system=conn.target_system,
            target_component=conn.target_component,
        )
        conn.mav.mission_count_send(
            target_system=conn.target_system,
            target_component=conn.target_component,
            count=len(lines),
            mission_type=mavutil.mavlink.MAV_MISSION_TYPE_MISSION,
        )
        for line in lines:
            print(line)
            if len(line) == 12:
                result = conn.recv_match(
                    type=["MISSION_REQUEST", "MISSION_REQUEST_INT"],
                    blocking=True,
                    timeout=5,
                )
                if result is None:
                    logging.error("Failed to receive mission request: %s", result)
                    continue_flag = True
                    continue
                conn.mav.mission_item_int_send(
                    target_system=conn.target_system,
                    target_component=conn.target_component,
                    seq=int(line[0]),
                    current=int(line[1]),
                    frame=int(line[2]),
                    command=int(line[3]),
                    param1=float(line[4]),
                    param2=float(line[5]),
                    param3=float(line[6]),
                    param4=float(line[7]),
                    x=int(float(line[8]) * 1e7),
                    y=int(float(line[9]) * 1e7),
                    z=float(line[10]),
                    autocontinue=int(line[11]),
                )
            else:
                logging.error("Invalid line: %s", line)
                continue_flag = True
                continue
        if continue_flag:
            continue
        message = conn.recv_match(type="MISSION_ACK", blocking=True, timeout=5)
        if message is not None:
            logging.info("Mission uploaded")
        else:
            logging.error("Failed to upload mission")
            continue_flag = True
            continue
        mode_id = conn.mode_mapping()["LOITER"]
        conn.mav.command_long_send(
            target_system=conn.target_system,
            target_component=conn.target_component,
            command=mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            confirmation=0,
            param1=1,
            param2=mode_id,
            param3=0,
            param4=0,
            param5=0,
            param6=0,
            param7=0,
        )
        ack = conn.recv_match(type="COMMAND_ACK", blocking=True, timeout=5)
        if ack is not None and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            logging.info("Mode set")
        else:
            logging.error(
                "Failed to set mode: %s",
                ack.result if ack is not None else "No ack received",
            )
            continue_flag = True
            continue
        conn.mav.command_long_send(
            target_system=conn.target_system,
            target_component=conn.target_component,
            command=mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            confirmation=0,
            param1=1,
            param2=0,
            param3=0,
            param4=0,
            param5=0,
            param6=0,
            param7=0,
        )
        ack = conn.recv_match(type="COMMAND_ACK", blocking=True, timeout=5)
        if ack is not None and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            logging.info("Armed")
        else:
            logging.error(
                "Failed to arm: %s",
                ack.result if ack is not None else "No ack received",
            )
            continue_flag = True
            continue
        conn.motors_armed_wait()

        conn.mav.command_long_send(
            target_system=conn.target_system,
            target_component=conn.target_component,
            command=mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            confirmation=0,
            param1=0,
            param2=0,
            param3=0,
            param4=0,
            param5=0,
            param6=0,
            param7=20,
        )
        ack = conn.recv_match(type="COMMAND_ACK", blocking=True, timeout=5)
        if ack is not None and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            logging.info("Mission started")
        else:
            logging.error(
                "Failed to takeoff: %s",
                ack.result if ack is not None else "No ack received",
            )
            continue_flag = True
            continue
        time.sleep(10)

        mode_id = conn.mode_mapping()["AUTO"]
        conn.mav.command_long_send(
            target_system=conn.target_system,
            target_component=conn.target_component,
            command=mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            confirmation=0,
            param1=1,
            param2=mode_id,
            param3=0,
            param4=0,
            param5=0,
            param6=0,
            param7=0,
        )
        ack = conn.recv_match(type="COMMAND_ACK", blocking=True, timeout=5)
        if ack is not None and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            logging.info("Auto mode set")
        else:
            logging.error(
                "Failed to start mission: %s",
                ack.result if ack is not None else "No ack received",
            )
            continue_flag = True
    logging.info("Starting!")

    while True:
        msg = conn.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if msg is not None:
            print(f"Heartbeat from vehicle: {msg.get_type()}")


if __name__ == "__main__":
    main()
