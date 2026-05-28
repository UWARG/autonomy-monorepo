import logging
import sys
import socket
import cv2
import struct
import time
import numpy as np
import threading
import sensor_ports

logging.basicConfig(level=logging.INFO)
HOST = '127.0.0.1'
frame_count=0
PRINT_INTERVAL=50

def camera_thread(port):
    global frame_count
    airside_socket=socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    airside_socket.bind(('', port))
    airside_socket.settimeout(0.5)
    while True:
        try:
            data, _ = airside_socket.recvfrom(65535) #24 bytes for the header
            if not data:
                continue
            if len(data) < 24:
                logging.error(f"Received bad data: {len(data)}")
                continue
            header=data[:24]
            rgb_length, depth_length, far, near = struct.unpack("QQff", header)
            if rgb_length == 0 or depth_length == 0 or far == 0 or near == 0:
                logging.error(f"Received bad data: {rgb_length} {depth_length}")
                continue
            rgb_data=data[24:24+rgb_length]
            depth_data=data[24+rgb_length:24+rgb_length+depth_length]
            rgb_array=np.frombuffer(rgb_data, np.uint8)
            rgb_image=cv2.imdecode(rgb_array, cv2.IMREAD_COLOR)
            depth_buffer=np.frombuffer(depth_data, np.uint8)
            depth_array=0.01*np.array(cv2.imdecode(depth_buffer, cv2.IMREAD_UNCHANGED))
            middle=(rgb_image.shape[0]//2, rgb_image.shape[1]//2)
            middle_depth=float((depth_array[middle[0],middle[1] ]))

            cv2.imshow(str(port), rgb_image)
            if depth_length > 0:
                arr_min=np.min(depth_array)
                arr_max=np.max(depth_array)
                normalized_array=(depth_array-arr_min)/(arr_max-arr_min)
                cv2.imshow(str(port)+"_depth_map", normalized_array)
            if frame_count % PRINT_INTERVAL == 0:
                logging.info(f"Received {rgb_length} bytes of rgb data and {depth_length} and middle depth:"
                f"{middle_depth}" if middle_depth<100 else f"nothing within range of {near} to {far}")

            if cv2.waitKey(1) & 0xFF == ord('q'):
                sys.exit(0)
        except OSError:
            continue

def range_finder_thread(port):
    global frame_count
    airside_socket=socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    airside_socket.bind(('', port))
    airside_socket.settimeout(0.5)
    while True:
        try:
            data, _ = airside_socket.recvfrom(65535)
            range=struct.unpack("f", data[:4])[0]
            if frame_count % PRINT_INTERVAL == 0:
                logging.info(f"Received {len(data)} bytes of range data with range: {range}")
        except OSError:
            continue

def main():
    global frame_count
    for key,value in sensor_ports.CAMERA_PORTS.items():
        thread_camera=threading.Thread(target=camera_thread,args=(value["port"]+sensor_ports.GROUNDSIDE_OFFSET,),daemon=True)
        thread_camera.start()
    for key,value in sensor_ports.RANGE_FINDER_PORTS.items():
        thread_range_finder=threading.Thread(target=range_finder_thread,args=(value["port"]+sensor_ports.GROUNDSIDE_OFFSET,),daemon=True)
        thread_range_finder.start()
    while True:
        frame_count+=1
        time.sleep(0.01)


if __name__ == "__main__":
    main()