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
            "name,sshUrl,url,isArchived",
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
    "isArchived": false
  },
  {
    "name": "old",
    "sshUrl": "git@github.com:UWARG/old.git",
    "url": "https://github.com/UWARG/old",
    "isArchived": true
  },
  {
    "name": "alpha",
    "sshUrl": "git@github.com:UWARG/alpha.git",
    "url": "https://github.com/UWARG/alpha",
    "isArchived": false
  }
]
""",
            stderr="",
        )

    monkeypatch.setattr("github_adapter.subprocess.run", fake_run)

    repositories = GitHubAdapter.list_org_repositories("UWARG")

    assert [repository.name for repository in repositories] == ["alpha", "zeta"]
    assert repositories[0].ssh_url == "git@github.com:UWARG/alpha.git"


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
    "isArchived": true
  }
]
""",
            stderr="",
        )

    monkeypatch.setattr("github_adapter.subprocess.run", fake_run)

    repositories = GitHubAdapter.list_org_repositories("UWARG", include_archived=True)

    assert [repository.name for repository in repositories] == ["old"]
