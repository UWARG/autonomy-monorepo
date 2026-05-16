from __future__ import annotations

from pathlib import Path

from typer.testing import CliRunner

from cli import _materialize_dependency_graph, app
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


def test_clone_uses_sparse_partial_clone(monkeypatch) -> None:
    calls = []

    class FakeGit:
        @classmethod
        def clone_sparse(cls, repository: str, destination: str | None) -> None:
            calls.append((repository, destination))

    monkeypatch.setattr("cli.GitAdapter", FakeGit)

    result = runner.invoke(
        app,
        [
            "clone",
            "git@github.com:warg/autonomy-monorepo.git",
            "autonomy-monorepo",
        ],
    )

    assert result.exit_code == 0
    assert calls == [
        ("git@github.com:warg/autonomy-monorepo.git", "autonomy-monorepo")
    ]
    assert "Only root files are checked out" in result.stdout


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


def test_ci_pr_runs_affected_project_pipeline(fixture_repo: Path, monkeypatch) -> None:
    monkeypatch.chdir(fixture_repo)
    calls = []

    class FakeGit:
        def __init__(self, root: Path):
            self.root = root

        def changed_files(self, base: str, *, merge_base: bool) -> list[Path]:
            assert base == "origin/feature-base"
            assert merge_base is True
            return [Path("camera/src/capture.py")]

    class FakeRunner:
        def run(self, project: Project, command_name: str, passthrough: list[str]) -> int:
            calls.append((project.name, command_name, passthrough))
            return 0

    monkeypatch.setattr("cli.GitAdapter", FakeGit)
    monkeypatch.setattr("cli.CommandRunner", FakeRunner)

    result = runner.invoke(app, ["ci", "pr", "--base", "origin/feature-base"])

    assert result.exit_code == 0
    assert calls == [("camera", "test", [])]


def test_ci_main_runs_affected_project_pipeline(fixture_repo: Path, monkeypatch) -> None:
    monkeypatch.chdir(fixture_repo)
    calls = []

    class FakeGit:
        def __init__(self, root: Path):
            self.root = root

        def changed_files(self, base: str, *, merge_base: bool) -> list[Path]:
            assert base == "abc123"
            assert merge_base is False
            return [Path("camera/src/capture.py")]

    class FakeRunner:
        def run(self, project: Project, command_name: str, passthrough: list[str]) -> int:
            calls.append((project.name, command_name, passthrough))
            return 0

    monkeypatch.setattr("cli.GitAdapter", FakeGit)
    monkeypatch.setattr("cli.CommandRunner", FakeRunner)

    result = runner.invoke(app, ["ci", "main", "--base", "abc123"])

    assert result.exit_code == 0
    assert calls == [("camera", "lint", []), ("camera", "test", [])]


def test_ci_skips_affected_projects_without_pipeline(
    fixture_repo: Path, monkeypatch
) -> None:
    monkeypatch.chdir(fixture_repo)
    calls = []

    class FakeGit:
        def __init__(self, root: Path):
            self.root = root

        def changed_files(self, base: str, *, merge_base: bool) -> list[Path]:
            return [Path("mavlink_comm/src/radio.py")]

    class FakeRunner:
        def run(self, project: Project, command_name: str, passthrough: list[str]) -> int:
            calls.append((project.name, command_name, passthrough))
            return 0

    monkeypatch.setattr("cli.GitAdapter", FakeGit)
    monkeypatch.setattr("cli.CommandRunner", FakeRunner)

    result = runner.invoke(app, ["ci", "pr"])

    assert result.exit_code == 0
    assert calls == []
    assert "No affected projects define [ci].pr" in result.stdout


def test_bare_command_runs_current_project_manifest_command(
    fixture_repo: Path, monkeypatch
) -> None:
    monkeypatch.chdir(fixture_repo / "camera")
    calls = []

    class FakeRunner:
        def run(self, project: Project, command_name: str, passthrough: list[str]) -> int:
            calls.append((project.name, command_name, passthrough))
            return 0

    monkeypatch.setattr("cli.CommandRunner", FakeRunner)

    result = runner.invoke(app, ["test", "--", "--fix"])

    assert result.exit_code == 0
    assert calls == [("camera", "test", ["--fix"])]


def test_bare_command_finds_project_manifest_from_nested_directory(
    fixture_repo: Path, monkeypatch
) -> None:
    nested = fixture_repo / "camera" / "src" / "camera"
    nested.mkdir(parents=True)
    monkeypatch.chdir(nested)
    calls = []

    class FakeRunner:
        def run(self, project: Project, command_name: str, passthrough: list[str]) -> int:
            calls.append((project.name, command_name, passthrough))
            return 0

    monkeypatch.setattr("cli.CommandRunner", FakeRunner)

    result = runner.invoke(app, ["test:unit"])

    assert result.exit_code == 0
    assert calls == [("camera", "test:unit", [])]


def test_unknown_bare_command_still_reports_no_such_command(
    fixture_repo: Path, monkeypatch
) -> None:
    monkeypatch.chdir(fixture_repo / "camera")

    result = runner.invoke(app, ["missing"])

    assert result.exit_code == 2
    assert "No such command 'missing'" in result.stdout


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


def test_up_project_picker_uses_root_registry_for_sparse_checkout(
    tmp_path: Path, monkeypatch
) -> None:
    (tmp_path / ".git").mkdir()
    (tmp_path / "projects.toml").write_text(
        """
[projects.camera]
path = "camera"

[projects.gesture_control]
path = "gesture_control"
""".strip()
        + "\n"
    )
    monkeypatch.chdir(tmp_path)

    selected_choices = []

    class FakeQuestion:
        def ask(self) -> str:
            return "gesture_control"

    def fake_select(message: str, choices: list[str]) -> FakeQuestion:
        assert message == "Select a project"
        selected_choices.extend(choices)
        return FakeQuestion()

    monkeypatch.setattr("cli.questionary.select", fake_select)

    from cli import _pick_project
    from registry import Registry

    assert _pick_project(Registry(tmp_path)) == "gesture_control"
    assert selected_choices == ["camera", "gesture_control"]


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


def test_up_reports_git_errors(fixture_repo: Path, monkeypatch) -> None:
    monkeypatch.chdir(fixture_repo)

    class FakeGit:
        def __init__(self, root: Path):
            self.root = root

        def materialize_paths(self, paths: list[str]) -> set[str]:
            from errors import GitError

            raise GitError("git sparse-checkout init failed: permission denied")

    monkeypatch.setattr("cli.GitAdapter", FakeGit)

    result = runner.invoke(app, ["up", "gesture_control"])

    assert result.exit_code == 1
    assert "git sparse-checkout init failed" in result.stdout


def test_up_materializes_requested_project_before_reading_dependencies(
    tmp_path: Path,
) -> None:
    (tmp_path / ".git").mkdir()
    (tmp_path / "projects.toml").write_text(
        """
[projects.camera]
path = "camera"

[projects.gesture_control]
path = "gesture_control"

[projects.mavlink_comm]
path = "mavlink_comm"
""".strip()
        + "\n"
    )
    calls = []

    class FakeGit:
        def materialize_paths(self, paths: list[str]) -> set[str]:
            calls.append(paths)
            for path in paths:
                project_dir = tmp_path / path
                project_dir.mkdir(exist_ok=True)
                manifest = project_dir / "warg.toml"
                if manifest.exists():
                    continue
                if path == "gesture_control":
                    manifest.write_text(
                        'name = "gesture_control"\n'
                        'depends_on = ["camera", "mavlink_comm"]\n'
                        "[commands]\n"
                        'setup = "echo setup-gesture"\n'
                    )
                elif path == "camera":
                    manifest.write_text(
                        'name = "camera"\n'
                        "depends_on = []\n"
                        "[commands]\n"
                        'setup = "echo setup-camera"\n'
                    )
                elif path == "mavlink_comm":
                    manifest.write_text(
                        'name = "mavlink_comm"\n'
                        "depends_on = []\n"
                        "[commands]\n"
                        'setup = "echo setup-mavlink"\n'
                    )
            return set(paths)

    paths, order, materialized = _materialize_dependency_graph(
        tmp_path, FakeGit(), "gesture_control"
    )

    assert calls == [
        ["gesture_control"],
        ["camera", "gesture_control", "mavlink_comm"],
    ]
    assert paths == ["camera", "gesture_control", "mavlink_comm"]
    assert [project.name for project in order] == [
        "camera",
        "mavlink_comm",
        "gesture_control",
    ]
    assert materialized == {"camera", "gesture_control", "mavlink_comm"}
