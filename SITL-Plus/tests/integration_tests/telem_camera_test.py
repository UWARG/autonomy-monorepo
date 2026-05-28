import logging
import sys
import socket
import cv2
import struct
import numpy as np
import time
import threading
import constants

logging.basicConfig(level=logging.INFO)
HOST = '127.0.0.1'
AIRSIDE_PORT=7000
airside_socket=socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
airside_socket.bind(('', 7000))
airside_socket.settimeout(0.5)
CAMERA_PORT=8003
RANGE_FINDER_PORT = 8004
print_interval=50
frame_count=0




def main():
    global frame_count
    while True:
        try:
            data, _ = airside_socket.recvfrom(65535) #28 bytes for the header
            if not data:
                continue
            header=data[:28]
            rgb_length, depth_length, range, far, near = struct.unpack("QQfff", header)
            if rgb_length == 0 or depth_length == 0 or far == 0 or near == 0:
                logging.error(f"Received bad data: {rgb_length} {depth_length}")
                continue
            rgb_data=data[28:28+rgb_length]
            depth_data=data[28+rgb_length:28+rgb_length+depth_length]
            rgb_array=np.frombuffer(rgb_data, np.uint8)
            rgb_image=cv2.imdecode(rgb_array, cv2.IMREAD_COLOR)
            depth_buffer=np.frombuffer(depth_data, np.uint8)
            depth_array=0.01*np.array(cv2.imdecode(depth_buffer, cv2.IMREAD_UNCHANGED))
            middle=(rgb_image.shape[0]//2, rgb_image.shape[1]//2)
            middle_depth=float((depth_array[middle[0],middle[1]]))-constants.CAMERA_OFFSET
            cv2.imshow("camera_stream", rgb_image)

            if frame_count % print_interval == 0:
                logging.info(f"Received {rgb_length} bytes of rgb data and {depth_length} bytes of depth data with range: {range} and middle depth:"
                 f"{middle_depth}" if middle_depth<100 else f"nothing within range of {near} to {far}")

            if cv2.waitKey(1) & 0xFF == ord('q'):
                sys.exit(0)
        except OSError:
            continue
        frame_count+=1


if __name__ == "__main__":
    main()