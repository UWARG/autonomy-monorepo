"""Repeatable polygon scenarios and cumulative mapping coverage benchmarks."""

import argparse
import asyncio
import json
import math
import random
import sys
from pathlib import Path

import yaml
from shapely.geometry import MultiPoint, Polygon
from websockets.exceptions import ConnectionClosed
from websockets.server import serve


def polygon(points):
    """Validate a simple polygon in a shared local coordinate frame."""
    if not isinstance(points, list) or len(points) < 3:
        raise ValueError("polygon requires at least three [x, y] vertices")
    for point in points:
        if not isinstance(point, (list, tuple)) or len(point) != 2:
            raise ValueError("each vertex must be [x, y]")
        if any(
            isinstance(v, bool)
            or not isinstance(v, (int, float))
            or not math.isfinite(v)
            for v in point
        ):
            raise ValueError("coordinates must be finite numbers")
    result = Polygon(points)
    if not result.is_valid or result.area <= 0:
        raise ValueError("polygon must be simple and have positive area")
    return result


def generate(zone, count=10, seed=0):
    """Generate distinct convex targets entirely covered by the zone."""
    boundary = polygon(zone)
    if isinstance(count, bool) or not isinstance(count, int) or count < 1:
        raise ValueError("count must be a positive integer")
    rng = random.Random(seed)
    xmin, ymin, xmax, ymax = boundary.bounds
    scenarios, seen = [], set()
    for _ in range(count * 1000):
        points = [
            (rng.uniform(xmin, xmax), rng.uniform(ymin, ymax))
            for _ in range(rng.randint(3, 8))
        ]
        target = MultiPoint(points).convex_hull
        if not isinstance(target, Polygon) or not boundary.covers(target):
            continue
        key = target.normalize().wkb
        if key in seen:
            continue
        seen.add(key)
        scenarios.append(
            {
                "id": f"scenario-{len(scenarios) + 1:04d}",
                "polygon": [list(p) for p in target.exterior.coords[:-1]],
            }
        )
        if len(scenarios) == count:
            return {
                "version": 1,
                "coordinate_frame": "local_meters",
                "seed": seed,
                "zone": zone,
                "scenarios": scenarios,
            }
    raise ValueError("could not generate enough targets; use a less narrow zone")


def load_suite(path):
    suite = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(suite, dict) or suite.get("version") != 1:
        raise ValueError("expected scenario schema version 1")
    if suite.get("coordinate_frame") != "local_meters":
        raise ValueError("coordinate_frame must be local_meters")
    boundary = polygon(suite.get("zone"))
    scenarios = suite.get("scenarios")
    if not isinstance(scenarios, list) or not scenarios:
        raise ValueError("suite must contain scenarios")
    targets = {}
    for scenario in scenarios:
        if not isinstance(scenario, dict):
            raise TypeError("scenario must be an object")
        identifier = scenario.get("id")
        if not isinstance(identifier, str) or not identifier or identifier in targets:
            raise ValueError("scenario IDs must be unique nonempty strings")
        target = polygon(scenario.get("polygon"))
        if not boundary.covers(target):
            raise ValueError("target must be inside zone")
        targets[identifier] = target
    return targets


class Harness:
    """Union incremental footprints so duplicate imagery earns no extra credit."""

    def __init__(self, targets):
        self.targets = targets
        self.mapped = {identifier: Polygon() for identifier in targets}
        self.elapsed = {identifier: 0.0 for identifier in targets}

    def update(self, message):
        if not isinstance(message, dict):
            raise TypeError("progress message must be an object")
        identifier = message.get("scenario_id")
        if not isinstance(identifier, str) or identifier not in self.targets:
            raise ValueError("unknown scenario_id")
        elapsed = message.get("elapsed_s")
        if (
            isinstance(elapsed, bool)
            or not isinstance(elapsed, (float, int))
            or not math.isfinite(elapsed)
            or elapsed < self.elapsed[identifier]
        ):
            raise ValueError("elapsed_s must be finite, nonnegative and nondecreasing")
        footprints = message.get("footprints")
        if not isinstance(footprints, list):
            raise TypeError("footprints must be a list of polygons")
        # Validate the entire message before mutating run state.
        additions = [polygon(points) for points in footprints]
        mapped = self.mapped[identifier]
        for addition in additions:
            mapped = mapped.union(addition)
        target = self.targets[identifier]
        covered = target.intersection(mapped).area
        outside = mapped.difference(target).area
        self.mapped[identifier], self.elapsed[identifier] = mapped, elapsed
        return {
            "scenario_id": identifier,
            "elapsed_s": elapsed,
            "target_area_m2": target.area,
            "covered_area_m2": covered,
            "missed_area_m2": max(0.0, target.area - covered),
            "outside_area_m2": outside,
            "coverage": covered / target.area,
            "iou": covered / (target.area + outside),
        }


async def listen(targets, host, port):
    """Serve one independent benchmark run per WebSocket connection."""

    async def handle(websocket):
        harness = Harness(targets)
        try:
            async for raw in websocket:
                try:
                    result = harness.update(json.loads(raw))
                except (ValueError, TypeError) as error:
                    await websocket.send(json.dumps({"error": str(error)}))
                    continue
                print(json.dumps(result, allow_nan=False), flush=True)
                await websocket.send(json.dumps(result, allow_nan=False))
        except ConnectionClosed:
            pass

    async with serve(handle, host, port, max_size=1024 * 1024):
        await asyncio.Future()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    generator = commands.add_parser("generate")
    generator.add_argument("--zone", required=True, help="YAML list of [x, y] meters")
    generator.add_argument("--output", required=True)
    generator.add_argument("--count", type=int, default=10)
    generator.add_argument("--seed", type=int, default=0)
    scorer = commands.add_parser("score")
    scorer.add_argument("--suite", required=True)
    scorer.add_argument("--updates", required=True, help="JSONL progress recording")
    listener = commands.add_parser("listen")
    listener.add_argument("--suite", required=True)
    listener.add_argument("--host", default="127.0.0.1")
    listener.add_argument("--port", type=int, default=8765)
    args = parser.parse_args()
    try:
        if args.command == "generate":
            zone = yaml.safe_load(Path(args.zone).read_text(encoding="utf-8"))
            suite = generate(zone, args.count, args.seed)
            Path(args.output).write_text(
                yaml.safe_dump(suite, sort_keys=False), encoding="utf-8"
            )
        else:
            targets = load_suite(args.suite)
            if args.command == "listen":
                asyncio.run(listen(targets, args.host, args.port))
            else:
                harness = Harness(targets)
                with Path(args.updates).open(encoding="utf-8") as recording:
                    for line in recording:
                        if line.strip():
                            print(
                                json.dumps(
                                    harness.update(json.loads(line)), allow_nan=False
                                )
                            )
    except (ValueError, TypeError, OSError, yaml.YAMLError) as error:
        parser.exit(2, f"error: {error}\n")
    except KeyboardInterrupt:
        sys.exit(0)


if __name__ == "__main__":
    main()
