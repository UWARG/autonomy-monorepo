import socket
import struct
import cv2
import numpy as np

sock=socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", 2000))

def main():
    while True:
        data, addr=sock.recvfrom(65535)
        if not data:
            continue
        img=cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
        cv2.imshow("Image", img)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()