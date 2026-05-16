from __future__ import annotations

from pathlib import Path

from typer.testing import CliRunner

from cli import app
from models import Project


runner = CliRunner()


def test_list_projects_from_repo_root(fixture_repo: Path, monkeypatch) -> None:
    monkeypatch.chdir(fixture_repo)

    result = runner.invoke(app, ["list"], env={}, catch_exceptions=False, obj=None)

    assert result.exit_code == 0
    assert "camera" in result.stdout
    assert "gesture_control" in result.stdout


def test_info_shows_commands(fixture_repo: Path, monkeypatch) -> None:
    monkeypatch.chdir(fixture_repo)

    result = runner.invoke(app, ["info", "camera"])

    assert result.exit_code == 0
    assert "camera" in result.stdout
    assert "test:unit" in result.stdout


def test_run_executes_dynamic_command(fixture_repo: Path, monkeypatch) -> None:
    monkeypatch.chdir(fixture_repo)
    calls = []

    class FakeRunner:
        def run(self, project: Project, command_name: str, passthrough: list[str]) -> int:
            calls.append((project.name, command_name, passthrough))
            return 0

    monkeypatch.setattr("cli.CommandRunner", FakeRunner)

    result = runner.invoke(app, ["run", "camera", "test:unit"])

    assert result.exit_code == 0
    assert calls == [("camera", "test:unit", [])]


def test_run_supports_passthrough_args(fixture_repo: Path, monkeypatch) -> None:
    monkeypatch.chdir(fixture_repo)
    calls = []

    class FakeRunner:
        def run(self, project: Project, command_name: str, passthrough: list[str]) -> int:
            calls.append((project.name, command_name, passthrough))
            return 0

    monkeypatch.setattr("cli.CommandRunner", FakeRunner)

    result = runner.invoke(app, ["run", "camera", "test", "--", "--fix", "x y"])

    assert result.exit_code == 0
    assert calls == [("camera", "test", ["--fix", "x y"])]


def test_missing_command_lists_available_commands(fixture_repo: Path, monkeypatch) -> None:
    monkeypatch.chdir(fixture_repo)

    result = runner.invoke(app, ["run", "camera", "missing"])

    assert result.exit_code == 1
    assert "Available commands:" in result.stdout
    assert "test:unit" in result.stdout


def test_run_uses_command_picker_when_command_is_missing(
    fixture_repo: Path, monkeypatch
) -> None:
    monkeypatch.chdir(fixture_repo)
    monkeypatch.setattr("cli._pick_command", lambda commands: "test")
    calls = []

    class FakeRunner:
        def run(self, project: Project, command_name: str, passthrough: list[str]) -> int:
            calls.append((project.name, command_name, passthrough))
            return 0

    monkeypatch.setattr("cli.CommandRunner", FakeRunner)

    result = runner.invoke(app, ["run", "camera"])

    assert result.exit_code == 0
    assert calls == [("camera", "test", [])]


def test_up_uses_project_picker_when_project_is_missing(
    fixture_repo: Path, monkeypatch
) -> None:
    monkeypatch.chdir(fixture_repo)

    materialized = {}

    class FakeGit:
        def __init__(self, root: Path):
            self.root = root

        def materialize_paths(self, paths: list[str]) -> set[str]:
            materialized["paths"] = paths
            return set(paths)

    calls = []

    class FakeRunner:
        def run(self, project: Project, command_name: str, passthrough: list[str]) -> int:
            calls.append((project.name, command_name, passthrough))
            return 0

    monkeypatch.setattr("cli._pick_project", lambda registry: "gesture_control")
    monkeypatch.setattr("cli.GitAdapter", FakeGit)
    monkeypatch.setattr("cli.CommandRunner", FakeRunner)

    result = runner.invoke(app, ["up"])

    assert result.exit_code == 0
    assert materialized["paths"] == [
        "camera",
        "gesture_control",
        "mavlink_comm",
    ]
    assert calls == [
        ("camera", "setup", []),
        ("mavlink_comm", "setup", []),
        ("gesture_control", "setup", []),
    ]


def test_up_skips_existing_project_setup(fixture_repo: Path, monkeypatch) -> None:
    monkeypatch.chdir(fixture_repo)

    class FakeGit:
        def __init__(self, root: Path):
            self.root = root

        def materialize_paths(self, paths: list[str]) -> set[str]:
            return {"gesture_control"}

    calls = []

    class FakeRunner:
        def run(self, project: Project, command_name: str, passthrough: list[str]) -> int:
            calls.append(project.name)
            return 0

    monkeypatch.setattr("cli.GitAdapter", FakeGit)
    monkeypatch.setattr("cli.CommandRunner", FakeRunner)

    result = runner.invoke(app, ["up", "gesture_control"])

    assert result.exit_code == 0
    assert calls == ["gesture_control"]


