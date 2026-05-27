import logging
import sys
import subprocess
import socket
import cv2
import struct
import numpy as np
import time
import threading

logging.basicConfig(level=logging.INFO)
HOST = '127.0.0.1'
PORT = 8000
RANGE_FINDER_PORT = 8001
print_interval=50


def range_finder_thread():
    s=socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind((HOST, RANGE_FINDER_PORT))
    frame_count=0
    while True:
        data, address = s.recvfrom(65535)
        range=struct.unpack("f", data)
        if frame_count % print_interval == 0:
            logging.info(f"Received range: {range}")
        frame_count+=1


def main():
    s=socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind((HOST, PORT))
    thread_range_finder=threading.Thread(target=range_finder_thread,daemon=True)
    thread_range_finder.start()
    frame_count=0
    while True:
        try:
            data, address = s.recvfrom(65535) #16 bytes for the header
            header=data[:16]
            rgb_length, depth_length = struct.unpack("QQ", header)
            if rgb_length == 0 or depth_length == 0:
                logging.error(f"Received bad data: {rgb_length} {depth_length}")
                continue
            rgb_data=data[16:16+rgb_length]
            depth_data=data[16+rgb_length:16+rgb_length+depth_length]
            rgb_array=np.frombuffer(rgb_data, np.uint8)
            rgb_image=cv2.imdecode(rgb_array, cv2.IMREAD_COLOR)
            cv2.imshow("camera_stream", rgb_image)

            if frame_count % print_interval == 0:
                logging.info(f"Received {rgb_length} bytes of rgb data and {depth_length} bytes of depth data")
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                sys.exit(0)
        except OSError:
            logging.error(f"Received bad data: {rgb_length} {depth_length}")
            time.sleep(0.01)
            continue
        frame_count+=1


if __name__ == "__main__":
    main()