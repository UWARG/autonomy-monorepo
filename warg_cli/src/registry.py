from __future__ import annotations

from pathlib import Path
from typing import Any

try:
    import tomllib
except ModuleNotFoundError:  # pragma: no cover - exercised on Python < 3.11
    import tomli as tomllib  # type: ignore[no-redef]

from errors import DependencyError, ManifestError
from models import Project


ROOT_INCLUDE_PATHS = ("README.md", ".gitignore", "warg_cli")


def find_repo_root(start: Path | None = None) -> Path:
    current = (start or Path.cwd()).resolve()
    for path in (current, *current.parents):
        if (path / ".git").exists():
            return path
    raise ManifestError("Could not find a Git repository root.")


class Registry:
    def __init__(self, root: Path):
        self.root = root.resolve()
        self.projects = self._discover()

    def _discover(self) -> dict[str, Project]:
        projects: dict[str, Project] = {}
        for manifest in sorted(self.root.glob("*/warg.toml")):
            project = self._load_project(manifest)
            if project.name in projects:
                other = projects[project.name].path
                raise ManifestError(
                    f"Duplicate project name '{project.name}' in {other} and {project.path}."
                )
            projects[project.name] = project
        return projects

    def _load_project(self, manifest: Path) -> Project:
        with manifest.open("rb") as file:
            data = tomllib.load(file)

        name = _expect_string(data, "name", manifest)
        language = data.get("language")
        if language is not None and not isinstance(language, str):
            raise ManifestError(f"{manifest}: 'language' must be a string.")

        description = data.get("description", "")
        if not isinstance(description, str):
            raise ManifestError(f"{manifest}: 'description' must be a string.")

        depends_on = data.get("depends_on", [])
        if not isinstance(depends_on, list) or not all(
            isinstance(item, str) for item in depends_on
        ):
            raise ManifestError(f"{manifest}: 'depends_on' must be a list of strings.")

        commands = data.get("commands", {})
        if not isinstance(commands, dict) or not all(
            isinstance(key, str) and isinstance(value, str)
            for key, value in commands.items()
        ):
            raise ManifestError(f"{manifest}: '[commands]' must map strings to strings.")

        return Project(
            name=name,
            path=manifest.parent,
            language=language,
            description=description,
            depends_on=tuple(depends_on),
            commands=dict(sorted(commands.items())),
        )

    def get(self, name: str) -> Project:
        try:
            return self.projects[name]
        except KeyError as error:
            available = ", ".join(sorted(self.projects)) or "none"
            raise ManifestError(
                f"Unknown project '{name}'. Available projects: {available}."
            ) from error

    def dependency_order(self, name: str) -> list[Project]:
        order: list[Project] = []
        visiting: list[str] = []
        visited: set[str] = set()

        def visit(project_name: str) -> None:
            if project_name in visited:
                return
            if project_name in visiting:
                cycle = " -> ".join([*visiting, project_name])
                raise DependencyError(f"Dependency cycle detected: {cycle}.")

            project = self.get(project_name)
            visiting.append(project_name)
            for dependency in project.depends_on:
                if dependency not in self.projects:
                    raise DependencyError(
                        f"Project '{project_name}' depends on unknown project "
                        f"'{dependency}'."
                    )
                visit(dependency)
            visiting.pop()
            visited.add(project_name)
            order.append(project)

        visit(name)
        return order

    def sparse_paths_for(self, name: str) -> list[str]:
        project_paths = [project.relative_path for project in self.dependency_order(name)]
        root_paths = [path for path in ROOT_INCLUDE_PATHS if (self.root / path).exists()]
        return sorted({*root_paths, *project_paths})


def _expect_string(data: dict[str, Any], key: str, manifest: Path) -> str:
    value = data.get(key)
    if not isinstance(value, str) or not value:
        raise ManifestError(f"{manifest}: '{key}' must be a non-empty string.")
    return value
