#!/usr/bin/env python3
"""Override RC channel 7 on the SITL vehicle (MAVProxy TCP 5761)."""

from __future__ import annotations

import argparse
import time

from pymavlink import mavutil

PWM = {
    "low": 988,
    "normal": 1500,
    "high": 2000,
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Set RC switch channel 7 via RC_CHANNELS_OVERRIDE on SITL."
    )
    parser.add_argument(
        "position",
        choices=sorted(PWM),
        help="Switch position: low=988, normal=1500, high=2000",
    )
    parser.add_argument(
        "--host",
        default="127.0.0.1",
        help="SITL / MAVProxy host (default: 127.0.0.1)",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=5762,
        help="TCP port (default: 5761)",
    )
    parser.add_argument(
        "--channel",
        type=int,
        default=7,
        help="RC channel number, 1-based (default: 7)",
    )
    parser.add_argument(
        "--hold",
        type=float,
        default=0.0,
        metavar="SEC",
        help="Keep re-sending the override for SEC seconds (0 = send a few times then exit)",
    )
    return parser.parse_args()


def override_channel(master: mavutil.mavfile, channel: int, pwm: int) -> None:
    """Send RC_CHANNELS_OVERRIDE with only ``channel`` set; others released (0)."""
    if not 1 <= channel <= 18:
        raise ValueError(f"channel must be 1-18, got {channel}")

    channels = [65535] * 8 #unchanged 
    channels[channel - 1] = pwm
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        *channels,
    )


def main() -> None:
    args = parse_args()
    pwm = PWM[args.position]
    url = f"tcp:{args.host}:{args.port}"

    print(f"Connecting to {url}...")
    master = mavutil.mavlink_connection(url)
    res=master.wait_heartbeat(timeout=10)
    if not res:
        print("No heartbeat received")
        master.target_system = 1
        master.target_component = 1
    else:
        print(
            f"Heartbeat from system {master.target_system} "
            f"component {master.target_component}"
        )

    print(f"RC ch{args.channel} -> {args.position} ({pwm} us)")
    deadline = time.monotonic() + max(args.hold, 0.3)
    while time.monotonic() < deadline:
        override_channel(master, args.channel, pwm)
        time.sleep(0.1)

    print("Done.")


if __name__ == "__main__":
    main()
