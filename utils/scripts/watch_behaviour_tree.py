# groundside utility, connects to MAVLink
# loads the tree table saved by the airside reporter, then listens for
# STATUSTEXT running-node messages and prints the tree as it changes

import argparse

from pymavlink import mavutil

from src.behaviour_tree_decoder import BehaviourTreeDecoder


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--tree", required=True, help="behaviour_tree.txt saved by the airside reporter")
    parser.add_argument("--url", default="udpin:0.0.0.0:14550", help="MAVLink connection")
    args = parser.parse_args()

    decoder = BehaviourTreeDecoder()
    decoder.load_table(args.tree)
    connection = mavutil.mavlink_connection(args.url)

    while True:
        msg = connection.recv_match(type="STATUSTEXT", blocking=True)
        if msg is None:
            continue
        text = msg.text.strip("\x00 ")
        if not text.startswith("S,"):
            continue  # flight controller messages
        try:
            decoder.handle_message(text)
            print(decoder.render() + "\n", flush=True)
        except ValueError as error:
            print(f"skipped message: {error}")


if __name__ == "__main__":
    main()