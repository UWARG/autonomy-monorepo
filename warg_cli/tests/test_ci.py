from __future__ import annotations

from pathlib import Path

from ci import affected_projects
from registry import Registry


def test_affected_projects_include_dependents(fixture_repo: Path) -> None:
    registry = Registry(fixture_repo)

    projects = affected_projects(registry, [Path("camera/src/capture.py")])

    assert [project.name for project in projects] == ["camera", "gesture_control"]


def test_affected_projects_include_all_projects_when_registry_changes(
    fixture_repo: Path,
) -> None:
    registry = Registry(fixture_repo)

    projects = affected_projects(registry, [Path("projects.toml")])

    assert [project.name for project in projects] == [
        "camera",
        "gesture_control",
        "mavlink_comm",
    ]


def test_affected_projects_ignore_root_files(fixture_repo: Path) -> None:
    registry = Registry(fixture_repo)

    projects = affected_projects(registry, [Path("README.md")])

    assert projects == []
