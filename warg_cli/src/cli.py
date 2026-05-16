from __future__ import annotations

from pathlib import Path
from typing import Optional

import click
import questionary
import typer
from rich.console import Console
from rich.table import Table
from typer.core import TyperGroup

from errors import WargError
from git_adapter import GitAdapter
from models import Project
from registry import Registry, find_repo_root, load_project_manifest
from runner import CommandRunner


class WargGroup(TyperGroup):
    def resolve_command(
        self, context: click.Context, args: list[str]
    ) -> tuple[str | None, click.Command | None, list[str]]:
        try:
            return super().resolve_command(context, args)
        except click.UsageError as error:
            exit_code = _run_current_project_command(args)
            if exit_code is None:
                raise error
            raise click.exceptions.Exit(exit_code) from None


app = typer.Typer(cls=WargGroup, no_args_is_help=True, add_completion=False)
console = Console()


@app.command("list")
def list_projects() -> None:
    """List discovered projects."""
    registry = _load_registry()
    table = Table(title="WARG projects")
    table.add_column("Project")
    table.add_column("Depends on")
    table.add_column("Description")

    for name, entry in registry.entries.items():
        project = registry.projects.get(name)
        table.add_row(
            name,
            ", ".join(project.depends_on) if project else "",
            project.description if project else f"Registered at {entry.path}",
        )
    console.print(table)


@app.command()
def info(project: str) -> None:
    """Show manifest metadata for a project."""
    registry = _load_registry()
    selected = registry.get(project)
    dependencies = registry.dependency_order(project)

    console.print(f"[bold]{selected.name}[/bold]")
    if selected.description:
        console.print(selected.description)

    console.print("Dependency order:")
    for dependency in dependencies:
        console.print(f"  - {dependency.name}")

    console.print("Commands:")
    for name in selected.commands:
        console.print(f"  - {name}")


@app.command()
def up(
    project: Optional[str] = typer.Argument(None),
    force: bool = typer.Option(False, "--force", help="Rerun setup commands."),
) -> None:
    """Materialize a project and its dependencies with Git sparse-checkout."""
    root = _load_repo_root()
    registry = Registry(root)
    project = project or _pick_project(registry)
    if not project:
        raise typer.Exit(1)

    try:
        git = GitAdapter(root)
        paths, setup_order, materialized = _materialize_dependency_graph(
            root, git, project
        )
    except WargError as error:
        console.print(f"[red]Error:[/red] {error}")
        raise typer.Exit(1) from error
    console.print("Sparse checkout paths:")
    for path in paths:
        console.print(f"  - {path}")

    runner = CommandRunner()
    for dependency in setup_order:
        should_setup = force or dependency.relative_path in materialized
        if "setup" in dependency.commands and should_setup:
            console.print(f"Running setup for [bold]{dependency.name}[/bold]")
            exit_code = runner.run(dependency, "setup", [])
            if exit_code != 0:
                raise typer.Exit(exit_code)


@app.command(
    context_settings={"allow_extra_args": True, "ignore_unknown_options": True}
)
def run(
    context: typer.Context,
    project: str,
) -> None:
    """Run a project-defined command from its warg.toml manifest."""
    registry = _load_registry()
    selected = registry.get(project)
    args = list(context.args)
    command = args.pop(0) if args else None
    command = command or _pick_command(selected.commands)
    if not command:
        raise typer.Exit(1)

    try:
        exit_code = CommandRunner().run(selected, command, args)
    except WargError as error:
        console.print(f"[red]Error:[/red] {error}")
        raise typer.Exit(1) from error
    if exit_code != 0:
        raise typer.Exit(exit_code)


def _load_registry() -> Registry:
    return Registry(_load_repo_root())


def _load_repo_root() -> Path:
    try:
        return find_repo_root()
    except WargError as error:
        console.print(f"[red]Error:[/red] {error}")
        raise typer.Exit(1) from error


def _run_current_project_command(args: list[str]) -> int | None:
    if not args:
        return None

    command, *passthrough = args
    if passthrough and passthrough[0] == "--":
        passthrough = passthrough[1:]
    project = _load_current_project()
    if project is None or command not in project.commands:
        return None

    try:
        return CommandRunner().run(project, command, passthrough)
    except WargError as error:
        console.print(f"[red]Error:[/red] {error}")
        return 1


def _load_current_project() -> Project | None:
    current = Path.cwd().resolve()
    for path in (current, *current.parents):
        manifest = path / "warg.toml"
        if manifest.exists():
            return load_project_manifest(manifest)
    return None


def _materialize_dependency_graph(
    root: Path, git: GitAdapter, project_name: str
) -> tuple[list[str], list[Project], set[str]]:
    requested_paths = {_path_for_project(root, project_name)}
    materialized: set[str] = set()

    while True:
        materialized.update(git.materialize_paths(sorted(requested_paths)))
        registry = Registry(root)
        order, discovered_paths = _discover_dependency_order(registry, project_name)
        missing_paths = discovered_paths - requested_paths
        if not missing_paths:
            paths = sorted(requested_paths)
            return paths, order, materialized
        requested_paths.update(missing_paths)


def _discover_dependency_order(
    registry: Registry, project_name: str
) -> tuple[list[Project], set[str]]:
    order: list[Project] = []
    requested_paths: set[str] = set()
    visiting: list[str] = []
    visited: set[str] = set()

    def visit(name: str) -> None:
        if name in visited:
            return
        if name in visiting:
            cycle = " -> ".join([*visiting, name])
            from errors import DependencyError

            raise DependencyError(f"Dependency cycle detected: {cycle}.")

        project = registry.get(name)
        requested_paths.add(project.relative_path)
        visiting.append(name)
        for dependency in project.depends_on:
            requested_paths.add(_entry_path(registry, dependency))
            if dependency in registry.projects:
                visit(dependency)
        visiting.pop()
        visited.add(name)
        order.append(project)

    visit(project_name)
    return order, requested_paths


def _path_for_project(root: Path, project_name: str) -> str:
    registry = Registry(root)
    return _entry_path(registry, project_name)


def _entry_path(registry: Registry, project_name: str) -> str:
    if project_name not in registry.entries:
        available = ", ".join(sorted(registry.entries)) or "none"
        from errors import DependencyError

        raise DependencyError(
            f"Unknown project '{project_name}'. Registered projects: {available}."
        )
    return registry.entries[project_name].path


def _pick_project(registry: Registry) -> str | None:
    if not registry.projects:
        console.print("No projects found.")
        return None
    return questionary.select(
        "Select a project",
        choices=sorted(registry.projects),
    ).ask()


def _pick_command(commands: dict[str, str]) -> str | None:
    if not commands:
        console.print("No commands found for this project.")
        return None
    return questionary.select(
        "Select a command",
        choices=sorted(commands),
    ).ask()


def main() -> None:
    try:
        app()
    except WargError as error:
        console.print(f"[red]Error:[/red] {error}")
        raise typer.Exit(1) from error


if __name__ == "__main__":
    main()
