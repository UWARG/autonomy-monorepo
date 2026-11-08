from __future__ import annotations

import subprocess
from pathlib import Path

from git_adapter import GitAdapter


def make_git_repo(tmp_path: Path) -> Path:
    subprocess.run(["git", "init", "-q", str(tmp_path)], check=True)
    return tmp_path


def test_remote_url_is_none_when_remote_missing(tmp_path: Path) -> None:
    git = GitAdapter(make_git_repo(tmp_path))

    assert git.remote_url("origin") is None


def test_add_remote_if_absent_adds_then_reports_existing(tmp_path: Path) -> None:
    git = GitAdapter(make_git_repo(tmp_path))
    url = "git@github.com:student/autonomy-bootcamp.git"

    assert git.add_remote_if_absent("origin", url) == (True, url)
    assert git.remote_url("origin") == url
    assert git.add_remote_if_absent("origin", "git@github.com:other/repo.git") == (
        False,
        url,
    )
