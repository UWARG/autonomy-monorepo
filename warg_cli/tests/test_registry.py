from __future__ import annotations

from pathlib import Path

import pytest

from constants import PROJECT_MANIFEST_FILENAME, ROOT_REGISTRY_FILENAME
from errors import DependencyError, ManifestError
from registry import Registry, find_repo_root, find_repo_root_or_none


def test_discovers_top_level_manifests(fixture_repo: Path) -> None:
    registry = Registry(fixture_repo)

    assert sorted(registry.projects) == ["camera", "gesture_control", "mavlink_comm"]
    assert registry.get("camera").commands["test:unit"] == "echo unit-camera"
    assert registry.get("camera").ci["pr"] == ("test",)


def test_resolves_dependency_order(fixture_repo: Path) -> None:
    registry = Registry(fixture_repo)

    assert [
        project.name for project in registry.dependency_order("gesture_control")
    ] == [
        "camera",
        "mavlink_comm",
        "gesture_control",
    ]


def test_detects_dependency_cycles(tmp_path: Path) -> None:
    (tmp_path / ".git").mkdir()
    write_registry(tmp_path, {"a": "a", "b": "b"})
    write_manifest(tmp_path, "a", 'name = "a"\ndepends_on = ["b"]\n')
    write_manifest(tmp_path, "b", 'name = "b"\ndepends_on = ["a"]\n')

    with pytest.raises(DependencyError, match="a -> b -> a"):
        Registry(tmp_path).dependency_order("a")


def test_reports_missing_dependencies(tmp_path: Path) -> None:
    (tmp_path / ".git").mkdir()
    write_registry(tmp_path, {"a": "a"})
    write_manifest(tmp_path, "a", 'name = "a"\ndepends_on = ["missing"]\n')

    with pytest.raises(DependencyError, match="unknown project 'missing'"):
        Registry(tmp_path).dependency_order("a")


def test_sparse_paths_include_project_and_dependencies(fixture_repo: Path) -> None:
    registry = Registry(fixture_repo)

    assert registry.sparse_paths_for("gesture_control") == [
        "camera",
        "gesture_control",
        "mavlink_comm",
    ]


def test_find_repo_root_from_nested_path(fixture_repo: Path) -> None:
    nested = fixture_repo / "camera" / "src"
    nested.mkdir()

    assert find_repo_root(nested) == fixture_repo
    assert find_repo_root_or_none(nested) == fixture_repo


def test_find_repo_root_or_none_returns_none_outside_repo(tmp_path: Path) -> None:
    assert find_repo_root_or_none(tmp_path) is None


def test_unknown_project_lists_available_projects(fixture_repo: Path) -> None:
    with pytest.raises(ManifestError, match="Registered projects: camera"):
        Registry(fixture_repo).get("missing")


def test_ci_references_must_match_commands(tmp_path: Path) -> None:
    (tmp_path / ".git").mkdir()
    write_registry(tmp_path, {"a": "a"})
    write_manifest(
        tmp_path,
        "a",
        """
name = "a"
depends_on = []

[commands]
test = "echo test"

[ci]
pr = ["missing"]
""",
    )

    with pytest.raises(ManifestError, match="references missing command 'missing'"):
        Registry(tmp_path)


def write_registry(root: Path, projects: dict[str, str]) -> None:
    lines = []
    for name, path in projects.items():
        lines.append(f"[projects.{name}]")
        lines.append(f'path = "{path}"')
        lines.append("")
    (root / ROOT_REGISTRY_FILENAME).write_text("\n".join(lines))


def write_manifest(root: Path, project: str, content: str) -> None:
    project_dir = root / project
    project_dir.mkdir()
    (project_dir / PROJECT_MANIFEST_FILENAME).write_text(content)


def test_parses_extra_paths_for_project(tmp_path: Path) -> None:
    (tmp_path / ".git").mkdir()
    (tmp_path / ROOT_REGISTRY_FILENAME).write_text(
        """
[projects.airside]
path = "airside"
extra_paths = ["shared/protos"]
""".strip()
        + "\n"
    )

    registry = Registry(tmp_path)

    assert registry.entries["airside"].extra_paths == ("shared/protos",)


def test_extra_paths_defaults_to_empty(fixture_repo: Path) -> None:
    registry = Registry(fixture_repo)

    assert registry.entries["camera"].extra_paths == ()


def test_extra_paths_must_be_list_of_strings(tmp_path: Path) -> None:
    (tmp_path / ".git").mkdir()
    (tmp_path / ROOT_REGISTRY_FILENAME).write_text(
        """
[projects.a]
path = "a"
extra_paths = "not-a-list"
""".strip()
        + "\n"
    )

    with pytest.raises(ManifestError, match="extra_paths must be a list of strings"):
        Registry(tmp_path)


def test_extra_paths_must_be_repo_relative(tmp_path: Path) -> None:
    (tmp_path / ".git").mkdir()
    (tmp_path / ROOT_REGISTRY_FILENAME).write_text(
        """
[projects.a]
path = "a"
extra_paths = ["../escape"]
""".strip()
        + "\n"
    )

    with pytest.raises(ManifestError, match="extra_paths must be repo-relative"):
        Registry(tmp_path)