from __future__ import annotations

import subprocess
import urllib.error

import pytest

from github_adapter import GitHubAdapter, GitHubError


def test_lists_org_repositories_from_gh(monkeypatch) -> None:
    def fake_run(*args, **kwargs):
        assert args[0] == [
            "gh",
            "repo",
            "list",
            "UWARG",
            "--limit",
            "1000",
            "--json",
            "name,sshUrl,url,isArchived,updatedAt",
        ]
        return subprocess.CompletedProcess(
            args=args[0],
            returncode=0,
            stdout="""
[
  {
    "name": "zeta",
    "sshUrl": "git@github.com:UWARG/zeta.git",
    "url": "https://github.com/UWARG/zeta",
    "isArchived": false,
    "updatedAt": "2025-01-02T00:00:00Z"
  },
  {
    "name": "old",
    "sshUrl": "git@github.com:UWARG/old.git",
    "url": "https://github.com/UWARG/old",
    "isArchived": true,
    "updatedAt": "2025-01-03T00:00:00Z"
  },
  {
    "name": "alpha",
    "sshUrl": "git@github.com:UWARG/alpha.git",
    "url": "https://github.com/UWARG/alpha",
    "isArchived": false,
    "updatedAt": "2025-01-01T00:00:00Z"
  }
]
""",
            stderr="",
        )

    monkeypatch.setattr("github_adapter.subprocess.run", fake_run)

    repositories = GitHubAdapter.list_org_repositories("UWARG")

    assert [repository.name for repository in repositories] == ["zeta", "alpha"]
    assert repositories[0].ssh_url == "git@github.com:UWARG/zeta.git"
    assert repositories[0].updated_at == "2025-01-02T00:00:00Z"


def test_can_include_archived_repositories(monkeypatch) -> None:
    def fake_run(*args, **kwargs):
        return subprocess.CompletedProcess(
            args=args[0],
            returncode=0,
            stdout="""
[
  {
    "name": "old",
    "sshUrl": "git@github.com:UWARG/old.git",
    "url": "https://github.com/UWARG/old",
    "isArchived": true,
    "updatedAt": "2025-01-03T00:00:00Z"
  }
]
""",
            stderr="",
        )

    monkeypatch.setattr("github_adapter.subprocess.run", fake_run)

    repositories = GitHubAdapter.list_org_repositories("UWARG", include_archived=True)

    assert [repository.name for repository in repositories] == ["old"]


def test_falls_back_to_github_api_when_gh_is_missing(monkeypatch) -> None:
    def fake_run(*args, **kwargs):
        raise FileNotFoundError

    class FakeResponse:
        def __enter__(self):
            return self

        def __exit__(self, *args):
            return None

        def read(self) -> bytes:
            return b"""
[
  {
    "name": "alpha",
    "ssh_url": "git@github.com:UWARG/alpha.git",
    "html_url": "https://github.com/UWARG/alpha",
    "archived": false,
    "updated_at": "2025-01-01T00:00:00Z"
  }
]
"""

    def fake_urlopen(request, timeout):
        assert request.full_url == (
            "https://api.github.com/orgs/UWARG/repos?per_page=100&page=1"
        )
        assert timeout == 10
        return FakeResponse()

    monkeypatch.setattr("github_adapter.subprocess.run", fake_run)
    monkeypatch.setattr("github_adapter.urllib.request.urlopen", fake_urlopen)

    repositories = GitHubAdapter.list_org_repositories("UWARG")

    assert [repository.name for repository in repositories] == ["alpha"]
    assert repositories[0].ssh_url == "git@github.com:UWARG/alpha.git"


def test_reports_both_errors_when_gh_and_api_fail(monkeypatch) -> None:
    def fake_run(*args, **kwargs):
        return subprocess.CompletedProcess(
            args=args[0],
            returncode=1,
            stdout="",
            stderr="not authenticated",
        )

    def fake_urlopen(request, timeout):
        raise urllib.error.URLError("offline")

    monkeypatch.setattr("github_adapter.subprocess.run", fake_run)
    monkeypatch.setattr("github_adapter.urllib.request.urlopen", fake_urlopen)

    try:
        GitHubAdapter.list_org_repositories("UWARG")
    except Exception as error:
        message = str(error)
    else:
        raise AssertionError("Expected repository listing to fail.")

    assert "gh CLI" in message
    assert "not authenticated" in message
    assert "offline" in message


def _fork_run(monkeypatch, responses: dict[str, tuple[int, str]]) -> list[list[str]]:
    commands: list[list[str]] = []

    def fake_run(command, **kwargs):
        commands.append(command)
        for token, (returncode, stdout) in responses.items():
            if token in command:
                return subprocess.CompletedProcess(
                    args=command, returncode=returncode, stdout=stdout, stderr=""
                )
        raise AssertionError(f"unexpected gh command: {command}")

    monkeypatch.setattr("github_adapter.subprocess.run", fake_run)
    return commands


VIEW_JSON = """
{
  "name": "autonomy-bootcamp",
  "sshUrl": "git@github.com:student/autonomy-bootcamp.git",
  "url": "https://github.com/student/autonomy-bootcamp",
  "parent": {"name": "autonomy-bootcamp", "owner": {"login": "UWARG"}}
}
"""


def test_fork_repository_creates_and_resolves_fork(monkeypatch) -> None:
    commands = _fork_run(
        monkeypatch,
        {
            "user": (0, "student\n"),
            "fork": (0, "✓ Created fork student/autonomy-bootcamp\n"),
            "view": (0, VIEW_JSON),
        },
    )

    fork = GitHubAdapter.fork_repository("UWARG/autonomy-bootcamp")

    assert fork.ssh_url == "git@github.com:student/autonomy-bootcamp.git"
    assert ["gh", "repo", "view", "student/autonomy-bootcamp"] == commands[-1][:4]
    assert commands[1] == [
        "gh",
        "repo",
        "fork",
        "UWARG/autonomy-bootcamp",
        "--clone=false",
    ]


def test_fork_repository_reuses_an_existing_fork(monkeypatch) -> None:
    _fork_run(
        monkeypatch,
        {
            "user": (0, "student\n"),
            "fork": (0, "! student/autonomy-bootcamp already exists\n"),
            "view": (0, VIEW_JSON),
        },
    )

    fork = GitHubAdapter.fork_repository("UWARG/autonomy-bootcamp")

    assert fork.name == "autonomy-bootcamp"


def test_fork_repository_follows_a_renamed_fork(monkeypatch) -> None:
    renamed = VIEW_JSON.replace(
        '"name": "autonomy-bootcamp",\n  "sshUrl"',
        '"name": "autonomy-bootcamp-1",\n  "sshUrl"',
    )
    commands = _fork_run(
        monkeypatch,
        {
            "user": (0, "student\n"),
            "fork": (0, "✓ Created fork student/autonomy-bootcamp-1\n"),
            "view": (0, renamed),
        },
    )

    fork = GitHubAdapter.fork_repository("UWARG/autonomy-bootcamp")

    assert fork.name == "autonomy-bootcamp-1"
    assert commands[-1][3] == "student/autonomy-bootcamp-1"


def test_fork_repository_rejects_a_same_named_non_fork(monkeypatch) -> None:
    _fork_run(
        monkeypatch,
        {
            "user": (0, "student\n"),
            "fork": (0, "! student/autonomy-bootcamp already exists\n"),
            "view": (
                0,
                VIEW_JSON.replace(
                    '"parent": {"name": "autonomy-bootcamp", '
                    '"owner": {"login": "UWARG"}}',
                    '"parent": null',
                ),
            ),
        },
    )

    with pytest.raises(GitHubError, match="not a fork"):
        GitHubAdapter.fork_repository("UWARG/autonomy-bootcamp")


def test_fork_repository_reports_missing_gh(monkeypatch) -> None:
    def fake_run(command, **kwargs):
        raise FileNotFoundError

    monkeypatch.setattr("github_adapter.subprocess.run", fake_run)

    with pytest.raises(GitHubError, match="gh is not installed"):
        GitHubAdapter.fork_repository("UWARG/autonomy-bootcamp")


def test_fork_repository_reports_unauthenticated_gh(monkeypatch) -> None:
    def fake_run(command, **kwargs):
        return subprocess.CompletedProcess(
            args=command,
            returncode=1,
            stdout="",
            stderr="gh: To use GitHub CLI, run: gh auth login",
        )

    monkeypatch.setattr("github_adapter.subprocess.run", fake_run)

    with pytest.raises(GitHubError, match="gh auth login"):
        GitHubAdapter.fork_repository("UWARG/autonomy-bootcamp")
