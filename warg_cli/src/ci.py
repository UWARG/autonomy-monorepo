from __future__ import annotations

from pathlib import Path

from errors import DependencyError
from models import Project
from registry import ROOT_REGISTRY, Registry
from runner import CommandRunner


def affected_projects(registry: Registry, changed_files: list[Path]) -> list[Project]:
    if any(path.parts and path.parts[0] == ROOT_REGISTRY for path in changed_files):
        return list(registry.projects.values())

    directly_changed = _directly_changed_projects(registry, changed_files)
    affected_names = _expand_dependents(registry, directly_changed)
    return _dependency_order_for_projects(registry, affected_names)


def run_ci_pipeline(
    registry: Registry,
    projects: list[Project],
    pipeline: str,
    runner: CommandRunner,
) -> int:
    for project in projects:
        command_names = project.ci.get(pipeline, ())
        for command_name in command_names:
            exit_code = runner.run(project, command_name, [])
            if exit_code != 0:
                return exit_code
    return 0


def _directly_changed_projects(
    registry: Registry, changed_files: list[Path]
) -> set[str]:
    changed: set[str] = set()
    project_paths = {
        name: Path(entry.path)
        for name, entry in registry.entries.items()
        if name in registry.projects
    }

    for changed_file in changed_files:
        for name, project_path in project_paths.items():
            if _is_relative_to(changed_file, project_path):
                changed.add(name)
    return changed


def _expand_dependents(registry: Registry, project_names: set[str]) -> set[str]:
    affected = set(project_names)
    changed = True
    while changed:
        changed = False
        for project in registry.projects.values():
            if project.name in affected:
                continue
            if any(dependency in affected for dependency in project.depends_on):
                affected.add(project.name)
                changed = True
    return affected


def _dependency_order_for_projects(
    registry: Registry, project_names: set[str]
) -> list[Project]:
    ordered: list[Project] = []
    seen: set[str] = set()

    for name in registry.projects:
        if name not in project_names:
            continue
        for project in registry.dependency_order(name):
            if project.name in project_names and project.name not in seen:
                seen.add(project.name)
                ordered.append(project)

    missing = project_names - seen
    if missing:
        raise DependencyError(
            f"Could not resolve affected projects: {', '.join(missing)}."
        )
    return ordered


def _is_relative_to(path: Path, parent: Path) -> bool:
    try:
        path.relative_to(parent)
    except ValueError:
        return False
    return True
