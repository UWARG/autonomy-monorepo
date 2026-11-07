"""Optional visual-only terrain and downward camera setup for iris_runway.

No aircraft flight command or physical/collision geometry is added. Use once
per world; --camera-only avoids creating a duplicate terrain model.
"""
import argparse
import json
import math
from pathlib import Path
import subprocess
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--camera-only', action='store_true')
    parser.add_argument('--degrees', type=float, default=90.0)
    args = parser.parse_args()
    if not -180 <= args.degrees <= 180:
        parser.error('Camera pitch must be within the existing joint limits')
    if not args.camera_only:
        path = Path(__file__).with_name('dashboard_terrain.sdf').resolve()
        result = subprocess.run([
            'gz', 'service', '-s', '/world/iris_runway/create',
            '--reqtype', 'gz.msgs.EntityFactory', '--reptype', 'gz.msgs.Boolean',
            '--timeout', '4000', '--req',
            f'sdf_filename: {json.dumps(str(path))}, allow_renaming: false',
        ], check=True, capture_output=True, text=True)
        print(result.stdout.strip())
        if 'data: true' not in result.stdout:
            raise RuntimeError('Terrain was not created; if already present use --camera-only')
    from gz.transport13 import Node
    from gz.msgs10.double_pb2 import Double
    node = Node()
    publisher = node.advertise('/gimbal/cmd_pitch', Double)
    deadline = time.monotonic()+3
    while not publisher.has_connections() and time.monotonic()<deadline:
        time.sleep(0.1)
    if not publisher.has_connections():
        raise RuntimeError('No simulated camera pitch subscriber')
    message = Double()
    message.data = math.radians(args.degrees)
    deadline = time.monotonic()+3
    while time.monotonic()<deadline:
        publisher.publish(message)
        time.sleep(0.1)
    print(f'Simulated camera pitch: {args.degrees} degrees; no flight command')


if __name__ == '__main__':
    main()
