import json

import pytest
import yaml
from src.benchmark import Harness, generate, load_suite, polygon

SQUARE = [[0, 0], [10, 0], [10, 10], [0, 10]]


def test_repeatable_unique_contained_targets(tmp_path):
    zone = [[0, 0], [10, 0], [10, 4], [4, 4], [4, 10], [0, 10]]
    suite = generate(zone, 20, 42)
    assert suite == generate(zone, 20, 42)
    assert suite != generate(zone, 20, 43)
    path = tmp_path / "suite.yaml"
    path.write_text(yaml.safe_dump(suite))
    targets = load_suite(path)
    assert len({target.normalize().wkb for target in targets.values()}) == 20
    assert all(polygon(zone).covers(target) for target in targets.values())


def test_coverage_union_and_outside_penalty():
    harness = Harness({"test": polygon(SQUARE)})
    footprint = [[0, 0], [5, 0], [5, 10], [0, 10]]
    update = {"scenario_id": "test", "elapsed_s": 1, "footprints": [footprint]}
    assert harness.update(update)["coverage"] == 0.5
    assert harness.update(update)["covered_area_m2"] == 50
    update["footprints"] = [[[5, 0], [15, 0], [15, 10], [5, 10]]]
    result = harness.update(update)
    assert result["coverage"] == 1
    assert result["outside_area_m2"] == 50
    assert result["iou"] == pytest.approx(2 / 3)


@pytest.mark.parametrize(
    "points",
    [
        [],
        [[0, 0]] * 3,
        [[0, 0], [1, 1], [0, 1], [1, 0]],
        [[0, 0], [1, 0], [0, float("nan")]],
    ],
)
def test_bad_polygons(points):
    with pytest.raises(ValueError):
        polygon(points)


def test_invalid_message_is_atomic():
    harness = Harness({"test": polygon(SQUARE)})
    with pytest.raises(ValueError):
        harness.update(
            {"scenario_id": "test", "elapsed_s": 2, "footprints": [SQUARE, []]}
        )
    assert (
        harness.update({"scenario_id": "test", "elapsed_s": 0, "footprints": []})[
            "coverage"
        ]
        == 0
    )
    harness.update({"scenario_id": "test", "elapsed_s": 3, "footprints": []})
    with pytest.raises(ValueError):
        harness.update({"scenario_id": "test", "elapsed_s": 2, "footprints": []})


def test_cli_generate_and_replay(tmp_path, monkeypatch, capsys):
    from src.benchmark import main

    zone, suite, updates = [
        tmp_path / name for name in ("zone.yaml", "suite.yaml", "updates.jsonl")
    ]
    zone.write_text(yaml.safe_dump(SQUARE))
    monkeypatch.setattr(
        "sys.argv",
        [
            "benchmark",
            "generate",
            "--zone",
            str(zone),
            "--output",
            str(suite),
            "--count",
            "1",
        ],
    )
    main()
    identifier, target = next(iter(load_suite(suite).items()))
    updates.write_text(
        json.dumps(
            {
                "scenario_id": identifier,
                "elapsed_s": 1,
                "footprints": [list(target.exterior.coords)],
            }
        )
        + "\n"
    )
    monkeypatch.setattr(
        "sys.argv",
        ["benchmark", "score", "--suite", str(suite), "--updates", str(updates)],
    )
    main()
    assert json.loads(capsys.readouterr().out)["iou"] == 1


def test_websocket_exchange():
    import asyncio
    from contextlib import suppress

    from src import benchmark
    from src.benchmark import listen
    from websockets.client import connect
    from websockets.server import serve as real_serve

    async def run():
        ready = asyncio.Future()

        class Server:
            async def __aenter__(self):
                self.server = await real_serve(self.handler, "127.0.0.1", 0)
                ready.set_result(self.server.sockets[0].getsockname()[1])
                return self.server

            async def __aexit__(self, *args):
                self.server.close()
                await self.server.wait_closed()

        def factory(handler, *args, **kwargs):
            wrapper = Server()
            wrapper.handler = handler
            return wrapper

        original = benchmark.serve
        benchmark.serve = factory
        task = asyncio.create_task(listen({"test": polygon(SQUARE)}, "127.0.0.1", 0))
        try:
            port = await asyncio.wait_for(ready, 5)
            async with connect(f"ws://127.0.0.1:{port}") as client:
                await client.send("bad json")
                assert "error" in json.loads(await client.recv())
                await client.send(
                    json.dumps(
                        {"scenario_id": "test", "elapsed_s": 1, "footprints": [SQUARE]}
                    )
                )
                assert json.loads(await client.recv())["coverage"] == 1
            async with connect(f"ws://127.0.0.1:{port}") as client:
                await client.send(
                    json.dumps(
                        {"scenario_id": "test", "elapsed_s": 0, "footprints": []}
                    )
                )
                assert json.loads(await client.recv())["coverage"] == 0
        finally:
            task.cancel()
            with suppress(asyncio.CancelledError):
                await task
            benchmark.serve = original

    asyncio.run(asyncio.wait_for(run(), 10))
