import socket
import struct
import cv2
import numpy as np

HOST = "0.0.0.0"
PORT = 2000


def recv_exact(conn: socket.socket, n: int) -> bytes | None:
    """Read exactly n bytes, or None if the peer closes early."""
    buf = bytearray()
    while len(buf) < n:
        chunk = conn.recv(n - len(buf))
        if not chunk:
            return None
        buf.extend(chunk)
    return bytes(buf)


def main():
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((HOST, PORT))
    server.listen(1)
    print(f"Listening on {HOST}:{PORT}")
    conn, addr = server.accept()
    print(f"Connected by {addr}")
    header=recv_exact(conn, 4)
    if header is None:
        print("No header received")
        return
    (length,) = struct.unpack("!I", header)
    data = recv_exact(conn, length)
    if data is None:
        print("No data received")
        return
    img1 = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
    try:
        while True:
            header = recv_exact(conn, 4)
            if header is None:
                break
            (length,) = struct.unpack("!I", header)
            data = recv_exact(conn, length)
            if data is None:
                break
            img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
            if img is None:
                continue
            a=cv2.hconcat([img1, img])
            cv2.imshow("Image", a)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        conn.close()
        server.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
