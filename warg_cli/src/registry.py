from __future__ import annotations

from pathlib import Path
from typing import Any

try:
    import tomllib
except ModuleNotFoundError:  # pragma: no cover - exercised on Python < 3.11
    import tomli as tomllib  # type: ignore[no-redef]

from errors import DependencyError, ManifestError
from models import Project, ProjectEntry


ROOT_REGISTRY = "projects.toml"


def find_repo_root(start: Path | None = None) -> Path:
    current = (start or Path.cwd()).resolve()
    for path in (current, *current.parents):
        if (path / ".git").exists():
            return path
    raise ManifestError("Could not find a Git repository root.")


class Registry:
    def __init__(self, root: Path):
        self.root = root.resolve()
        self.entries = self._load_entries()
        self.projects = self._load_materialized_projects()

    def _load_entries(self) -> dict[str, ProjectEntry]:
        registry_path = self.root / ROOT_REGISTRY
        if not registry_path.exists():
            raise ManifestError(f"Missing root project registry: {ROOT_REGISTRY}.")

        with registry_path.open("rb") as file:
            data = tomllib.load(file)

        projects = data.get("projects")
        if not isinstance(projects, dict) or not projects:
            raise ManifestError(f"{ROOT_REGISTRY}: '[projects]' must not be empty.")

        entries: dict[str, ProjectEntry] = {}
        for name, metadata in projects.items():
            if not isinstance(name, str) or not name:
                raise ManifestError(f"{ROOT_REGISTRY}: project names must be strings.")
            if not isinstance(metadata, dict):
                raise ManifestError(f"{ROOT_REGISTRY}: project '{name}' must be a table.")
            path = metadata.get("path")
            if not isinstance(path, str) or not path:
                raise ManifestError(
                    f"{ROOT_REGISTRY}: project '{name}' must define a non-empty path."
                )
            if Path(path).is_absolute() or ".." in Path(path).parts:
                raise ManifestError(
                    f"{ROOT_REGISTRY}: project '{name}' path must be repo-relative."
                )
            if name in entries:
                raise ManifestError(f"{ROOT_REGISTRY}: duplicate project '{name}'.")
            entries[name] = ProjectEntry(name=name, path=path)

        return dict(sorted(entries.items()))

    def _load_materialized_projects(self) -> dict[str, Project]:
        projects: dict[str, Project] = {}
        for entry in self.entries.values():
            manifest = self.root / entry.path / "warg.toml"
            if not manifest.exists():
                continue
            project = self._load_project(manifest)
            if project.name != entry.name:
                raise ManifestError(
                    f"{manifest}: project name '{project.name}' does not match "
                    f"registry name '{entry.name}'."
                )
            projects[entry.name] = project
        return projects

    def _load_project(self, manifest: Path) -> Project:
        with manifest.open("rb") as file:
            data = tomllib.load(file)

        name = _expect_string(data, "name", manifest)
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

        ci = data.get("ci", {})
        if not isinstance(ci, dict) or not all(
            isinstance(key, str)
            and isinstance(value, list)
            and all(isinstance(item, str) for item in value)
            for key, value in ci.items()
        ):
            raise ManifestError(
                f"{manifest}: '[ci]' must map pipeline names to lists of commands."
            )

        for pipeline, command_names in ci.items():
            for command_name in command_names:
                if command_name not in commands:
                    raise ManifestError(
                        f"{manifest}: [ci].{pipeline} references missing command "
                        f"'{command_name}'."
                    )

        return Project(
            name=name,
            path=manifest.parent,
            description=description,
            depends_on=tuple(depends_on),
            commands=dict(sorted(commands.items())),
            ci={
                pipeline: tuple(command_names)
                for pipeline, command_names in sorted(ci.items())
            },
        )

    def get(self, name: str) -> Project:
        if name not in self.entries:
            available = ", ".join(sorted(self.entries)) or "none"
            raise ManifestError(
                f"Unknown project '{name}'. Registered projects: {available}."
            )
        try:
            return self.projects[name]
        except KeyError as error:
            raise ManifestError(
                f"Project '{name}' is registered but not checked out. "
                f"Run 'warg up {name}' to materialize it."
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
        return sorted(project_paths)


def _expect_string(data: dict[str, Any], key: str, manifest: Path) -> str:
    value = data.get(key)
    if not isinstance(value, str) or not value:
        raise ManifestError(f"{manifest}: '{key}' must be a non-empty string.")
    return value
