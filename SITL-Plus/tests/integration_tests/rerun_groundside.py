import logging
import sys
import socket
import cv2
import struct
import time
import numpy as np
import threading
import sensor_ports
import os
import constants
logging.basicConfig(level=logging.INFO)
frame_count=0
PRINT_INTERVAL=50
FRAME_RATE=constants.RANGE_FINDER_FPS
def range_finder_thread(port):
    global frame_count
    groundside_socket=socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    groundside_socket.bind(('', port))
    groundside_socket.settimeout(0.5)
    while True:
        try:
            data, _ = groundside_socket.recvfrom(65535)
            range=struct.unpack("f", data[:4])[0]
            if frame_count % PRINT_INTERVAL == 0:
                logging.info(f"Received {len(data)} bytes of range data with range: {range}")
        except OSError:
            continue

def main():
    logging.info(f"HOST: {os.getenv('SENSOR_HOST')}")
    global frame_count
    for key,value in sensor_ports.RANGE_FINDER_PORTS.items():
        thread_range_finder=threading.Thread(target=range_finder_thread,args=(value["port"]+sensor_ports.GROUNDSIDE_OFFSET,),daemon=True)
        thread_range_finder.start()
    while True:
        time.sleep(1/FRAME_RATE)
        frame_count+=1


if __name__ == "__main__":
    main()