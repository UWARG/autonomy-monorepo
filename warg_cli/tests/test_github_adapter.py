from __future__ import annotations

import subprocess

from github_adapter import GitHubAdapter


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
