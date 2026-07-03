from pymavlink import mavutil
import os
from pathlib import Path
import socket
import logging
import time
import struct
import threading
import sensor_ports

logging.basicConfig(level=logging.INFO)
PORT=5761
groundside_socket=socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
groundside_socket.settimeout(100)
print_interval=1000


def range_finder_thread(port):
    range_finder_socket=socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    range_finder_socket.bind(('', port))
    range_finder_socket.settimeout(0.5)
    while True:
        try:
            data_range, _ = range_finder_socket.recvfrom(65535)
            logging.info(f"Received {len(data_range)} bytes of range data")
            latest_range=struct.unpack("f", data_range[:4])[0]
            groundside_socket.sendto(struct.pack("f",latest_range), ('127.0.0.1', port))
        except OSError:
            continue

def camera_thread(port):
    camera_socket=socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    camera_socket.bind(('', port))
    camera_socket.settimeout(0.5)
    while True:
        try:
            data_camera,_=camera_socket.recvfrom(65535)
            logging.info(f"Received {len(data_camera)} bytes of camera data")
            rgb_length, depth_length, far, near = struct.unpack("QQff", data_camera[:24])
            data_camera=data_camera[24:]
            header=struct.pack("QQff",rgb_length,depth_length,float(far),float(near))              
            groundside_socket.sendto(header+data_camera, ('127.0.0.1', port))
        except OSError:
            continue



def main():
    for key,value in sensor_ports.CAMERA_PORTS.items():
        thread_camera=threading.Thread(target=camera_thread,args=(value["port"],),daemon=True)
        thread_camera.start()
    for key,value in sensor_ports.RANGE_FINDER_PORTS.items():
        thread_range_finder=threading.Thread(target=range_finder_thread,args=(value["port"],),daemon=True)
        thread_range_finder.start()
    conn=mavutil.mavlink_connection(f"tcp:127.0.0.1:{PORT}") 
    conn.wait_heartbeat()
    print(f"Heartbeat from vehicle: {conn.target_system} {conn.target_component}")
    mission_file=os.path.abspath(Path.joinpath(Path(__file__).parent.parent.parent,"src","mission_load.waypoints"))
    lines=[]
    with open(mission_file, "r") as file:
        for line in file:
            if line.startswith("#") or line.startswith("QGC WPL"):
                continue
            file_line=line.split("\t")
            file_line[-1]=file_line[-1].strip("\n")
            lines.append(file_line)
    conn.mav.param_set_send(
        target_system=conn.target_system,
        target_component=conn.target_component,
        param_id=b"FRAME_TYPE",
        param_value=1,
        param_type=mavutil.mavlink.MAV_PARAM_TYPE_INT32
    )
    conn.mav.param_set_send(
        target_system=conn.target_system,
        target_component=conn.target_component,
        param_id=b"FRAME_CLASS",
        param_value=1,
        param_type=mavutil.mavlink.MAV_PARAM_TYPE_INT32
    )
    time.sleep(5)
    conn.mav.param_set_send(
        target_system=conn.target_system,
        target_component=conn.target_component,
        param_id=b"SIM_RATE_HZ",
        param_value=200,
        param_type=mavutil.mavlink.MAV_PARAM_TYPE_INT32
    )
    conn.mav.mission_clear_all_send(
        target_system=conn.target_system,
        target_component=conn.target_component,
    )
    conn.mav.mission_count_send(
        target_system=conn.target_system,
        target_component=conn.target_component,
        count=len(lines),
        mission_type=mavutil.mavlink.MAV_MISSION_TYPE_MISSION    
    )
    for line in lines:
        print(line)
        if len(line)==12:
            result=conn.recv_match(type=["MISSION_REQUEST","MISSION_REQUEST_INT"],blocking=True,timeout=5)
            if result is None:
                logging.error(f"Failed to receive mission request: {result}")
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
                x=int(float(line[8])*1e7),
                y=int(float(line[9])*1e7),
                z=float(line[10]),
                autocontinue=int(line[11])
            )
        else:
            logging.error(f"Invalid line: {line}")

    message=conn.recv_match(type="MISSION_ACK",blocking=True)
    if message is not None:
        logging.info("Mission uploaded")
    else:
        logging.error("Failed to upload mission")
    
    mode_id=conn.mode_mapping()["LOITER"] #keep as loiter, guided does not work
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
        param7=0
    )
    ack=conn.recv_match(type="COMMAND_ACK",blocking=True)
    if ack.result==mavutil.mavlink.MAV_RESULT_ACCEPTED:
        logging.info("Mode set")
    else:
        logging.error(f"Failed to set mode: {ack.result}")
    
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
        param7=0
    )
    ack=conn.recv_match(type="COMMAND_ACK",blocking=True)
    if ack.result==mavutil.mavlink.MAV_RESULT_ACCEPTED:
        logging.info("Armed")
    else:
        logging.error(f"Failed to arm: {ack.result}")
    
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
        param7=20
    )
    ack=conn.recv_match(type="COMMAND_ACK",blocking=True)
    if ack.result==mavutil.mavlink.MAV_RESULT_ACCEPTED:
        logging.info("Mission started")
    else:
        logging.error(f"Failed to takeoff: {ack.result}")
    time.sleep(10)
    
    mode_id=conn.mode_mapping()["AUTO"]
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
        param7=0
    )
    ack=conn.recv_match(type="COMMAND_ACK",blocking=True)
    if ack.result==mavutil.mavlink.MAV_RESULT_ACCEPTED:
        logging.info("Auto mode set")
    else:
        logging.error(f"Failed to start mission: {ack.result}")


    while True:
        msg=conn.recv_match(type="HEARTBEAT",blocking=True,timeout=1)
        if msg is not None:
            print(f"Heartbeat from vehicle: {msg.get_type()}")

if __name__ == "__main__":
    main()