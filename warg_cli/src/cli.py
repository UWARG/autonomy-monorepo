from __future__ import annotations

from pathlib import Path
from typing import Optional

import questionary
import typer
from rich.console import Console
from rich.table import Table
from typing_extensions import Annotated

from errors import WargError
from git_adapter import GitAdapter
from registry import Registry, find_repo_root
from runner import CommandRunner

app = typer.Typer(no_args_is_help=True, add_completion=False)
console = Console()


def _registry() -> Registry:
    return Registry(find_repo_root())


@app.command("list")
def list_projects() -> None:
    """List discovered projects."""
    registry = _load_registry()
    table = Table(title="WARG projects")
    table.add_column("Project")
    table.add_column("Language")
    table.add_column("Depends on")
    table.add_column("Description")

    for project in sorted(registry.projects.values(), key=lambda item: item.name):
        table.add_row(
            project.name,
            project.language or "",
            ", ".join(project.depends_on),
            project.description,
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
    if selected.language:
        console.print(f"Language: {selected.language}")

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
    registry = _load_registry()
    project = project or _pick_project(registry)
    if not project:
        raise typer.Exit(1)

    paths = registry.sparse_paths_for(project)
    materialized = GitAdapter(registry.root).materialize_paths(paths)
    console.print("Sparse checkout paths:")
    for path in paths:
        console.print(f"  - {path}")

    runner = CommandRunner()
    for dependency in registry.dependency_order(project):
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
    try:
        return _registry()
    except WargError as error:
        console.print(f"[red]Error:[/red] {error}")
        raise typer.Exit(1) from error


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
