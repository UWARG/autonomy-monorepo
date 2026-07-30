from __future__ import annotations

from pathlib import Path

from typer.testing import CliRunner

from cli import _materialize_dependency_graph, app
from errors import GitError
from github_adapter import GitHubRepository
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


def test_doctor_prints_repository_access_diagnostics(
    fixture_repo: Path, monkeypatch
) -> None:
    monkeypatch.chdir(fixture_repo)
    roots = []

    class FakeGit:
        def __init__(self, root: Path | None):
            self.root = root
            roots.append(root)

        def repository_access_diagnostics(self) -> list[str]:
            return [
                "remote.origin.url: git@github.com:UWARG/autonomy-monorepo.git",
                "git ls-remote --exit-code origin HEAD: ok",
            ]

    monkeypatch.setattr("cli.GitAdapter", FakeGit)

    result = runner.invoke(app, ["doctor"])

    assert result.exit_code == 0
    assert roots == [fixture_repo]
    assert "Git repository access" in result.stdout
    assert "remote.origin.url" in result.stdout
    assert "git ls-remote --exit-code origin HEAD: ok" in result.stdout


def test_doctor_runs_outside_git_repo(tmp_path: Path, monkeypatch) -> None:
    monkeypatch.chdir(tmp_path)
    roots = []

    class FakeGit:
        def __init__(self, root: Path | None):
            self.root = root
            roots.append(root)

        def repository_access_diagnostics(self) -> list[str]:
            return [
                "Git repository: not found",
                "ssh -T -o BatchMode=yes git@github.com: ok",
            ]

    monkeypatch.setattr("cli.GitAdapter", FakeGit)

    result = runner.invoke(app, ["doctor"])

    assert result.exit_code == 0
    assert roots == [None]
    assert "Git repository: not found" in result.stdout
    assert "ssh -T -o BatchMode=yes git@github.com: ok" in result.stdout


def test_clone_uses_sparse_partial_clone(monkeypatch) -> None:
    calls = []

    class FakeGit:
        def __init__(self, root=None):
            self.root = root

        def materialize_paths(self, paths):
            return set(paths)

        @classmethod
        def clone(cls, repository: str, destination: str | None) -> None:
            raise AssertionError("unexpected full clone")

        @classmethod
        def clone_sparse(cls, repository: str, destination: str | None) -> Path:
            calls.append((repository, destination))

            return Path(destination or "autonomy-monorepo")
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
    assert calls == [("git@github.com:warg/autonomy-monorepo.git", "autonomy-monorepo")]
    assert "include_paths are checked out" in result.stdout


def test_clone_warns_when_repository_uses_https(monkeypatch) -> None:
    calls = []

    class FakeGit:
        def __init__(self, root=None):
            self.root = root

        def materialize_paths(self, paths):
            return set(paths)

        @classmethod
        def clone_sparse(cls, repository: str, destination: str | None) -> Path:
            calls.append((repository, destination))

            return Path(destination or "autonomy-monorepo")
    monkeypatch.setattr("cli.GitAdapter", FakeGit)

    result = runner.invoke(
        app,
        [
            "clone",
            "https://github.com/UWARG/autonomy-monorepo.git",
            "autonomy-monorepo",
        ],
    )

    assert result.exit_code == 0
    assert calls == [
        ("https://github.com/UWARG/autonomy-monorepo.git", "autonomy-monorepo")
    ]
    assert "unable to push" in result.stdout
    assert (
        "https://docs.github.com/en/authentication/connecting-to-github-with-ssh"
        in result.stdout
    )


def test_clone_falls_back_to_https_when_github_ssh_clone_fails(monkeypatch) -> None:
    calls = []

    class FakeGit:
        def __init__(self, root=None):
            self.root = root

        def materialize_paths(self, paths):
            return set(paths)

        @classmethod
        def clone_sparse(cls, repository: str, destination: str | None) -> Path:
            calls.append((repository, destination))
            if repository == "git@github.com:UWARG/autonomy-monorepo.git":
                raise GitError("SSH clone failed")

            return Path(destination or "autonomy-monorepo")
    monkeypatch.setattr("cli.GitAdapter", FakeGit)

    result = runner.invoke(
        app,
        [
            "clone",
            "git@github.com:UWARG/autonomy-monorepo.git",
            "autonomy-monorepo",
        ],
    )

    assert result.exit_code == 0
    assert calls == [
        ("git@github.com:UWARG/autonomy-monorepo.git", "autonomy-monorepo"),
        ("https://github.com/UWARG/autonomy-monorepo.git", "autonomy-monorepo"),
    ]
    assert "SSH clone failed. Retrying with HTTPS" in result.stdout
    assert "unable to push" in result.stdout


def test_clone_does_not_fallback_for_non_github_ssh_urls(monkeypatch) -> None:
    calls = []

    class FakeGit:
        def __init__(self, root=None):
            self.root = root

        def materialize_paths(self, paths):
            return set(paths)

        @classmethod
        def clone_sparse(cls, repository: str, destination: str | None) -> Path:
            calls.append((repository, destination))
            raise GitError("SSH clone failed")

            return Path(destination or "autonomy-monorepo")
    monkeypatch.setattr("cli.GitAdapter", FakeGit)

    result = runner.invoke(
        app,
        [
            "clone",
            "git@example.com:UWARG/autonomy-monorepo.git",
            "autonomy-monorepo",
        ],
    )

    assert result.exit_code == 1
    assert calls == [
        ("git@example.com:UWARG/autonomy-monorepo.git", "autonomy-monorepo")
    ]
    assert "Error:" in result.stdout


def test_clone_full_uses_normal_clone(monkeypatch) -> None:
    calls = []

    class FakeGit:
        def __init__(self, root=None):
            self.root = root

        def materialize_paths(self, paths):
            return set(paths)

        @classmethod
        def clone(cls, repository: str, destination: str | None) -> None:
            calls.append((repository, destination))

        @classmethod
        def clone_sparse(cls, repository: str, destination: str | None) -> Path:
            raise AssertionError("unexpected sparse clone")

            return Path(destination or "autonomy-monorepo")
    monkeypatch.setattr("cli.GitAdapter", FakeGit)

    result = runner.invoke(
        app,
        [
            "clone",
            "--full",
            "git@github.com:warg/autonomy-monorepo.git",
            "autonomy-monorepo",
        ],
    )

    assert result.exit_code == 0
    assert calls == [("git@github.com:warg/autonomy-monorepo.git", "autonomy-monorepo")]
    assert "Cloned full repository" in result.stdout
    assert "Only root files are checked out" not in result.stdout


def test_clone_full_falls_back_to_https_when_github_ssh_clone_fails(
    monkeypatch,
) -> None:
    calls = []

    class FakeGit:
        def __init__(self, root=None):
            self.root = root

        def materialize_paths(self, paths):
            return set(paths)

        @classmethod
        def clone(cls, repository: str, destination: str | None) -> None:
            calls.append((repository, destination))
            if repository == "git@github.com:UWARG/autonomy-monorepo.git":
                raise GitError("SSH clone failed")

        @classmethod
        def clone_sparse(cls, repository: str, destination: str | None) -> Path:
            raise AssertionError("unexpected sparse clone")

            return Path(destination or "autonomy-monorepo")
    monkeypatch.setattr("cli.GitAdapter", FakeGit)

    result = runner.invoke(
        app,
        [
            "clone",
            "--full",
            "git@github.com:UWARG/autonomy-monorepo.git",
            "autonomy-monorepo",
        ],
    )

    assert result.exit_code == 0
    assert calls == [
        ("git@github.com:UWARG/autonomy-monorepo.git", "autonomy-monorepo"),
        ("https://github.com/UWARG/autonomy-monorepo.git", "autonomy-monorepo"),
    ]
    assert "SSH clone failed. Retrying with HTTPS" in result.stdout


def test_clone_picks_repository_when_missing(monkeypatch) -> None:
    calls = []

    class FakeGit:
        def __init__(self, root=None):
            self.root = root

        def materialize_paths(self, paths):
            return set(paths)

        @classmethod
        def clone(cls, repository: str, destination: str | None) -> None:
            raise AssertionError("unexpected full clone")

        @classmethod
        def clone_sparse(cls, repository: str, destination: str | None) -> Path:
            calls.append((repository, destination))

            return Path(destination or "autonomy-monorepo")
    monkeypatch.setattr("cli.GitAdapter", FakeGit)
    monkeypatch.setattr(
        "cli._pick_repository",
        lambda organization, include_archived: "git@github.com:UWARG/autonomy.git",
    )

    result = runner.invoke(app, ["clone"])

    assert result.exit_code == 0
    assert calls == [("git@github.com:UWARG/autonomy.git", None)]


def test_clone_resolves_uwarg_repository_name(monkeypatch) -> None:
    calls = []

    class FakeGit:
        def __init__(self, root=None):
            self.root = root

        def materialize_paths(self, paths):
            return set(paths)

        @classmethod
        def clone(cls, repository: str, destination: str | None) -> None:
            raise AssertionError("unexpected full clone")

        @classmethod
        def clone_sparse(cls, repository: str, destination: str | None) -> Path:
            calls.append((repository, destination))

            return Path(destination or "autonomy-monorepo")
    class FakeGitHub:
        @classmethod
        def list_org_repositories(
            cls, organization: str, include_archived: bool = False
        ) -> list[GitHubRepository]:
            assert organization == "UWARG"
            assert include_archived is False
            return [
                GitHubRepository(
                    name="autonomy-monorepo",
                    ssh_url="git@github.com:UWARG/autonomy-monorepo.git",
                    url="https://github.com/UWARG/autonomy-monorepo",
                )
            ]

    monkeypatch.setattr("cli.GitAdapter", FakeGit)
    monkeypatch.setattr("cli.GitHubAdapter", FakeGitHub)

    result = runner.invoke(app, ["clone", "autonomy-monorepo"])

    assert result.exit_code == 0
    assert calls == [("git@github.com:UWARG/autonomy-monorepo.git", None)]


def test_repository_picker_uses_fuzzy_choices(monkeypatch) -> None:
    class FakeGitHub:
        @classmethod
        def list_org_repositories(
            cls, organization: str, include_archived: bool = False
        ) -> list[GitHubRepository]:
            return [
                GitHubRepository(
                    name="autonomy-monorepo",
                    ssh_url="git@github.com:UWARG/autonomy-monorepo.git",
                    url="https://github.com/UWARG/autonomy-monorepo",
                )
            ]

    class FakeFuzzy:
        def __init__(self, choice: str):
            self.choice = choice

        def execute(self) -> str:
            return self.choice

    def fake_fuzzy(message: str, choices: list[object]):
        assert message == "Select a UWARG repository"
        assert len(choices) == 1
        assert choices[0].name == "autonomy-monorepo"
        assert choices[0].value == "git@github.com:UWARG/autonomy-monorepo.git"
        return FakeFuzzy(choices[0].value)

    monkeypatch.setattr("cli.GitHubAdapter", FakeGitHub)
    monkeypatch.setattr("cli.inquirer.fuzzy", fake_fuzzy)

    from cli import _pick_repository

    assert _pick_repository("UWARG", False) == (
        "git@github.com:UWARG/autonomy-monorepo.git"
    )


def test_run_executes_dynamic_command(fixture_repo: Path, monkeypatch) -> None:
    monkeypatch.chdir(fixture_repo)
    calls = []

    class FakeRunner:
        def run(
            self, project: Project, command_name: str, passthrough: list[str]
        ) -> int:
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
        def run(
            self, project: Project, command_name: str, passthrough: list[str]
        ) -> int:
            calls.append((project.name, command_name, passthrough))
            return 0

    monkeypatch.setattr("cli.CommandRunner", FakeRunner)

    result = runner.invoke(app, ["run", "camera", "test", "--", "--fix", "x y"])

    assert result.exit_code == 0
    assert calls == [("camera", "test", ["--fix", "x y"])]


def test_missing_command_lists_available_commands(
    fixture_repo: Path, monkeypatch
) -> None:
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
        def run(
            self, project: Project, command_name: str, passthrough: list[str]
        ) -> int:
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
        def run(
            self, project: Project, command_name: str, passthrough: list[str]
        ) -> int:
            calls.append((project.name, command_name, passthrough))
            return 0

    monkeypatch.setattr("cli.GitAdapter", FakeGit)
    monkeypatch.setattr("cli.CommandRunner", FakeRunner)

    result = runner.invoke(app, ["ci", "pr", "--base", "origin/feature-base"])

    assert result.exit_code == 0
    assert calls == [("camera", "test", [])]


def test_ci_main_runs_affected_project_pipeline(
    fixture_repo: Path, monkeypatch
) -> None:
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
        def run(
            self, project: Project, command_name: str, passthrough: list[str]
        ) -> int:
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
        def run(
            self, project: Project, command_name: str, passthrough: list[str]
        ) -> int:
            calls.append((project.name, command_name, passthrough))
            return 0

    monkeypatch.setattr("cli.GitAdapter", FakeGit)
    monkeypatch.setattr("cli.CommandRunner", FakeRunner)

    result = runner.invoke(app, ["ci", "pr"])

    assert result.exit_code == 0
    assert calls == []
    assert "No affected projects define [ci].pr" in result.stdout


def test_unknown_top_level_command_reports_no_such_command(
    fixture_repo: Path, monkeypatch
) -> None:
    monkeypatch.chdir(fixture_repo / "camera")

    result = runner.invoke(app, ["missing"])

    assert result.exit_code == 2
    assert "No such command 'missing'" in result.output


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
        def run(
            self, project: Project, command_name: str, passthrough: list[str]
        ) -> int:
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
    (tmp_path / "projects.toml").write_text("""
[projects.camera]
path = "camera"

[projects.gesture_control]
path = "gesture_control"
""".strip() + "\n")
    monkeypatch.chdir(tmp_path)

    selected_choices = []

    class FakeFuzzy:
        def execute(self) -> str:
            return "gesture_control"

    def fake_fuzzy(message: str, choices: list[str]) -> FakeFuzzy:
        assert message == "Select a project"
        selected_choices.extend(choices)
        return FakeFuzzy()

    monkeypatch.setattr("cli.inquirer.fuzzy", fake_fuzzy)

    from cli import _pick_project
    from registry import Registry

    assert _pick_project(Registry(tmp_path)) == "gesture_control"
    assert selected_choices == ["camera", "gesture_control"]


def test_up_runs_existing_project_setup(fixture_repo: Path, monkeypatch) -> None:
    monkeypatch.chdir(fixture_repo)

    class FakeGit:
        def __init__(self, root: Path):
            self.root = root

        def materialize_paths(self, paths: list[str]) -> set[str]:
            return set()

    calls = []

    class FakeRunner:
        def run(
            self, project: Project, command_name: str, passthrough: list[str]
        ) -> int:
            calls.append(project.name)
            return 0

    monkeypatch.setattr("cli.GitAdapter", FakeGit)
    monkeypatch.setattr("cli.CommandRunner", FakeRunner)

    result = runner.invoke(app, ["up", "gesture_control"])

    assert result.exit_code == 0
    assert calls == ["camera", "mavlink_comm", "gesture_control"]


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


def test_down_removes_project_sparse_checkout_path(
    fixture_repo: Path, monkeypatch
) -> None:
    monkeypatch.chdir(fixture_repo)
    calls = []

    class FakeGit:
        def __init__(self, root: Path):
            self.root = root

        def unmaterialize_paths(self, paths: list[str]) -> set[str]:
            calls.append(paths)
            return set(paths)

    monkeypatch.setattr("cli.GitAdapter", FakeGit)

    result = runner.invoke(app, ["down", "gesture_control"])

    assert result.exit_code == 0
    assert calls == [["gesture_control"]]
    assert "Removed sparse checkout paths:" in result.stdout
    assert "gesture_control" in result.stdout


def test_down_can_include_dependencies(fixture_repo: Path, monkeypatch) -> None:
    monkeypatch.chdir(fixture_repo)
    calls = []

    class FakeGit:
        def __init__(self, root: Path):
            self.root = root

        def unmaterialize_paths(self, paths: list[str]) -> set[str]:
            calls.append(paths)
            return set(paths)

    monkeypatch.setattr("cli.GitAdapter", FakeGit)

    result = runner.invoke(app, ["down", "gesture_control", "--include-dependencies"])

    assert result.exit_code == 0
    assert calls == [["camera", "gesture_control", "mavlink_comm"]]


def test_down_removes_checked_out_dependents_with_warning(
    fixture_repo: Path, monkeypatch
) -> None:
    monkeypatch.chdir(fixture_repo)
    calls = []

    class FakeGit:
        def __init__(self, root: Path):
            self.root = root

        def unmaterialize_paths(self, paths: list[str]) -> set[str]:
            calls.append(paths)
            return set(paths)

    monkeypatch.setattr("cli.GitAdapter", FakeGit)

    result = runner.invoke(app, ["down", "camera"])

    assert result.exit_code == 0
    assert calls == [["camera", "gesture_control"]]
    assert "Warning:" in result.stdout
    assert (
        "Also unloaded projects that depend on camera: gesture_control" in result.stdout
    )
    assert "camera" in result.stdout
    assert "gesture_control" in result.stdout


def test_down_reports_when_project_is_not_checked_out(
    fixture_repo: Path, monkeypatch
) -> None:
    monkeypatch.chdir(fixture_repo)

    class FakeGit:
        def __init__(self, root: Path):
            self.root = root

        def unmaterialize_paths(self, paths: list[str]) -> set[str]:
            return set()

    monkeypatch.setattr("cli.GitAdapter", FakeGit)

    result = runner.invoke(app, ["down", "camera"])

    assert result.exit_code == 0
    assert "camera is not checked out" in result.stdout


def test_up_materializes_requested_project_before_reading_dependencies(
    tmp_path: Path,
) -> None:
    (tmp_path / ".git").mkdir()
    (tmp_path / "projects.toml").write_text("""
[projects.camera]
path = "camera"

[projects.gesture_control]
path = "gesture_control"

[projects.mavlink_comm]
path = "mavlink_comm"
""".strip() + "\n")
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


def test_up_materializes_project_extra_paths(tmp_path: Path) -> None:
    (tmp_path / ".git").mkdir()
    (tmp_path / "projects.toml").write_text(
        """
[projects.airside]
path = "airside"
extra_paths = ["shared/protos"]
""".strip()
        + "\n"
    )

    class FakeGit:
        def materialize_paths(self, paths: list[str]) -> set[str]:
            for path in paths:
                project_dir = tmp_path / path
                project_dir.mkdir(parents=True, exist_ok=True)
                manifest = project_dir / "warg.toml"
                if path == "airside" and not manifest.exists():
                    manifest.write_text(
                        'name = "airside"\n'
                        "depends_on = []\n"
                        "[commands]\n"
                        'setup = "echo setup-airside"\n'
                    )
            return set(paths)

    paths, order, materialized = _materialize_dependency_graph(
        tmp_path, FakeGit(), "airside"
    )

    assert "shared/protos" in paths
    assert "airside" in paths
    assert "shared/protos" in materialized