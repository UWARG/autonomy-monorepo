from pymavlink import mavutil

ADDR="/dev/serial0"

def main():
    conn=mavutil.mavlink_connection(ADDR, baud=57600)
    while True:
        msg=conn.recv_match(type="HEARTBEAT",blocking=True, timeout=1)
        if msg is not None:
            print(msg)
        else:
            print("No heartbeat received")

if __name__ == "__main__":
    main()