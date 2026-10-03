# groundside utility, connects to MAVLink 
# listens for STATUSTEXT messages, from the airside behavior tree reporter
# prints current path through the behavior tree as it changes

from pymavlink import mavutil

from src.behaviour_tree_decoder import BehaviourTreeDecoder

def main() -> None:
    connection  = mavutil.mavlink_connection("udpin:127.0.0.1:14550")
    decoder = BehaviourTreeDecoder()

    while True:
        msg = connection.recv_match(type="STATUSTEXT", blocking=True)
        print("received:", msg) # debug
        if msg is None:
            continue

        decoder.handle_message(msg.text)
        print(decoder.current_path())

if __name__ == "__main__":
    main()