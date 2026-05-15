from __future__ import annotations

from pathlib import Path

import pytest

from errors import DependencyError, ManifestError
from registry import Registry, find_repo_root


def test_discovers_top_level_manifests(fixture_repo: Path) -> None:
    registry = Registry(fixture_repo)

    assert sorted(registry.projects) == ["camera", "gesture_control", "mavlink_comm"]
    assert registry.get("camera").commands["test:unit"] == "echo unit-camera"


def test_resolves_dependency_order(fixture_repo: Path) -> None:
    registry = Registry(fixture_repo)

    assert [project.name for project in registry.dependency_order("gesture_control")] == [
        "camera",
        "mavlink_comm",
        "gesture_control",
    ]


def test_detects_dependency_cycles(tmp_path: Path) -> None:
    (tmp_path / ".git").mkdir()
    write_manifest(tmp_path, "a", 'name = "a"\ndepends_on = ["b"]\n')
    write_manifest(tmp_path, "b", 'name = "b"\ndepends_on = ["a"]\n')

    with pytest.raises(DependencyError, match="a -> b -> a"):
        Registry(tmp_path).dependency_order("a")


def test_reports_missing_dependencies(tmp_path: Path) -> None:
    (tmp_path / ".git").mkdir()
    write_manifest(tmp_path, "a", 'name = "a"\ndepends_on = ["missing"]\n')

    with pytest.raises(DependencyError, match="unknown project 'missing'"):
        Registry(tmp_path).dependency_order("a")


def test_sparse_paths_include_dependencies_and_root_files(fixture_repo: Path) -> None:
    registry = Registry(fixture_repo)

    assert registry.sparse_paths_for("gesture_control") == [
        "README.md",
        "camera",
        "gesture_control",
        "mavlink_comm",
        "warg_cli",
    ]


def test_find_repo_root_from_nested_path(fixture_repo: Path) -> None:
    nested = fixture_repo / "camera" / "src"
    nested.mkdir()

    assert find_repo_root(nested) == fixture_repo


def test_unknown_project_lists_available_projects(fixture_repo: Path) -> None:
    with pytest.raises(ManifestError, match="Available projects: camera"):
        Registry(fixture_repo).get("missing")


def write_manifest(root: Path, project: str, content: str) -> None:
    project_dir = root / project
    project_dir.mkdir()
    (project_dir / "warg.toml").write_text(content)
