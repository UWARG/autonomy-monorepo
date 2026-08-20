from __future__ import annotations

from importlib.metadata import version as package_version
from pathlib import Path
import re
from typing import Optional

from InquirerPy import inquirer
from InquirerPy.base.control import Choice
import typer
from rich.console import Console
from rich.markup import escape
from rich.table import Table

from ci import affected_projects, run_ci_pipeline
from doctor import CheckStatus, run_doctor
from errors import WargError
from errors import GitError
from git_adapter import GitAdapter
from github_adapter import GitHubAdapter
from models import Project
from registry import Registry, expand_dependents, find_repo_root, find_repo_root_or_none
from runner import CommandRunner

app = typer.Typer(no_args_is_help=True, add_completion=False)
ci_app = typer.Typer(no_args_is_help=True, add_completion=False)
app.add_typer(ci_app, name="ci")
console = Console()


@app.callback()
def root(
    version: Optional[bool] = typer.Option(
        None,
        "--version",
        "-v",
        callback=lambda value: _show_version(value),
        is_eager=True,
        help="Show the WARG CLI version and exit.",
    ),
) -> None:
    pass


@app.command()
def clone(
    repository: Optional[str] = typer.Argument(
        None,
        help=(
            "Repository URL, or a UWARG repository name. "
            "Omit to pick from a searchable UWARG repo list."
        ),
    ),
    destination: Optional[str] = typer.Argument(
        None, help="Directory to clone into. Defaults to Git's repository name."
    ),
    organization: str = typer.Option(
        "UWARG", "--org", help="GitHub organization to search when picking a repo."
    ),
    include_archived: bool = typer.Option(
        False, "--include-archived", help="Include archived repositories in the picker."
    ),
    full: bool = typer.Option(
        False,
        "--full",
        help="Perform a normal clone instead of a partial sparse checkout.",
    ),
) -> None:
    """Clone a WARG monorepo sparsely by default, or fully with --full."""
    try:
        repository = repository or _pick_repository(organization, include_archived)
        if not repository:
            raise typer.Exit(1)
        repository = _resolve_repository(repository, organization, include_archived)
        _clone_repository(repository, destination, full)
    except WargError as error:
        console.print(f"[red]Error:[/red] {error}")
        raise typer.Exit(1) from error

    if full:
        console.print("Cloned full repository.")
    else:
        console.print("Cloned repository with sparse checkout enabled.")
        console.print(
            "Root files and any configured include_paths are checked out. "
            "Run 'warg up <project>' next."
        )


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
def doctor(
    verbose: bool = typer.Option(
        False,
        "--verbose",
        help="Show the output of every command doctor runs.",
    ),
) -> None:
    """Check this machine for common WARG dev environment problems."""
    icons = {
        CheckStatus.OK: ("✓", "green"),
        CheckStatus.WARN: ("!", "yellow"),
        CheckStatus.FAIL: ("✗", "red"),
    }

    console.print(f"[bold]warg doctor[/bold] v{package_version('warg-cli')}")

    sections = run_doctor(find_repo_root_or_none())
    failed = False
    for section in sections:
        console.print(f"\n[bold]{section.title}[/bold]")
        for check in section.checks:
            icon, style = icons[check.status]
            console.print(f"  [{style}]{icon}[/{style}] {check.detail}")
            if check.fix and check.status != CheckStatus.OK:
                console.print(f"      [dim]fix: {check.fix}[/dim]")
            if verbose:
                for entry in check.transcript:
                    for line in entry.splitlines():
                        console.print(f"      [dim]{escape(line)}[/dim]")
            failed = failed or check.status == CheckStatus.FAIL

    if failed:
        console.print(
            "\n[red]Some checks failed.[/red] Fix the items marked ✗ and rerun "
            "'warg doctor'."
        )
        raise typer.Exit(1)
    console.print("\nNo blocking problems found.")


@app.command()
def up(
    project: Optional[str] = typer.Argument(None),
) -> None:
    """Materialize a project and its dependencies with Git sparse-checkout."""
    root = _load_repo_root()
    registry = Registry(root)
    project = project or _pick_project(registry)
    if not project:
        raise typer.Exit(1)

    try:
        git = GitAdapter(root)
        with console.status(f"Materializing [bold]{project}[/bold]..."):
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
        if "setup" in dependency.commands:
            console.print(f"Running setup for [bold]{dependency.name}[/bold]")
            exit_code = runner.run(dependency, "setup", [])
            if exit_code != 0:
                raise typer.Exit(exit_code)


@app.command("down")
def down(
    project: Optional[str] = typer.Argument(None),
    include_dependencies: bool = typer.Option(
        False,
        "--include-dependencies",
        "--deps",
        help="Also unload dependencies that are checked out.",
    ),
) -> None:
    """Remove a project from Git sparse-checkout."""
    root = _load_repo_root()
    registry = Registry(root)
    project = project or _pick_project(registry)
    if not project:
        raise typer.Exit(1)

    try:
        paths, dependents = _unload_paths(registry, project, include_dependencies)
        git = GitAdapter(root)
        with console.status(f"Unloading [bold]{project}[/bold]..."):
            removed = git.unmaterialize_paths(paths)
    except WargError as error:
        console.print(f"[red]Error:[/red] {error}")
        raise typer.Exit(1) from error

    if not removed:
        console.print(f"[bold]{project}[/bold] is not checked out.")
        return

    removed_dependents = sorted(
        dependent for dependent in dependents if dependent in removed
    )
    if removed_dependents:
        console.print(
            "[yellow]Warning:[/yellow] Also unloaded projects that depend on "
            f"[bold]{project}[/bold]: {', '.join(removed_dependents)}"
        )

    console.print("Removed sparse checkout paths:")
    for path in sorted(removed):
        console.print(f"  - {path}")


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


@ci_app.command("pr")
def ci_pr(
    base: str = typer.Option(
        "origin/main",
        "--base",
        help="Git ref to compare against with merge-base diff syntax.",
    ),
) -> None:
    """Run PR CI for projects affected by this branch."""
    _run_ci("pr", base=base, merge_base=True)


@ci_app.command("main")
def ci_main(
    base: str = typer.Option(
        "HEAD~1",
        "--base",
        help="Git ref or SHA to compare against with before/after diff syntax.",
    ),
) -> None:
    """Run main-branch CI for projects affected by this push."""
    _run_ci("main", base=base, merge_base=False)


def _load_registry() -> Registry:
    return Registry(_load_repo_root())


def _load_repo_root() -> Path:
    try:
        return find_repo_root()
    except WargError as error:
        console.print(f"[red]Error:[/red] {error}")
        raise typer.Exit(1) from error


def _show_version(version: bool | None) -> None:
    if not version:
        return
    console.print(package_version("warg-cli"))
    raise typer.Exit()


def _resolve_repository(
    repository: str, organization: str, include_archived: bool
) -> str:
    if _looks_like_repository_url(repository):
        return repository

    repositories = GitHubAdapter.list_org_repositories(organization, include_archived)
    for candidate in repositories:
        if candidate.name == repository:
            return candidate.ssh_url

    available = ", ".join(candidate.name for candidate in repositories) or "none"

    raise GitError(
        f"Unknown {organization} repository '{repository}'. Available repositories: {available}."
    )


def _looks_like_repository_url(repository: str) -> bool:
    return (
        "://" in repository
        or repository.startswith("git@")
        or repository.startswith("ssh://")
    )


def _warn_if_https_repository(repository: str) -> None:
    if not repository.startswith("https://github.com/"):
        return

    console.print(
        "[yellow]Warning:[/yellow] This repository is configured with an HTTPS "
        "remote, so you will be unable to push to it through the expected WARG "
        "SSH workflow. Set up GitHub SSH access and clone with the SSH URL instead: "
        "https://docs.github.com/en/authentication/connecting-to-github-with-ssh"
    )


def _clone_repository(repository: str, destination: str | None, full: bool) -> None:
    _warn_if_https_repository(repository)
    try:
        if full:
            with console.status("Cloning repository..."):
                GitAdapter.clone(repository, destination)
        else:
            with console.status("Cloning repository with sparse checkout..."):
                clone_dir = GitAdapter.clone_sparse(repository, destination)
                _materialize_always_paths(clone_dir)
    except GitError:
        fallback = _github_ssh_url_to_https(repository)
        if fallback is None:
            raise
        console.print(
            "[yellow]Warning:[/yellow] SSH clone failed. Retrying with HTTPS."
        )
        _warn_if_https_repository(fallback)
        if full:
            GitAdapter.clone(fallback, destination)
        else:
            clone_dir = GitAdapter.clone_sparse(fallback, destination)
            _materialize_always_paths(clone_dir)


def _materialize_always_paths(clone_dir: Path) -> None:
    try:
        include_paths = Registry(clone_dir).include_paths
    except WargError:
        return
    if not include_paths:
        return
    git = GitAdapter(clone_dir)
    git.materialize_paths(list(include_paths))


def _github_ssh_url_to_https(repository: str) -> str | None:
    match = re.fullmatch(r"git@github\.com:([^/]+)/(.+?)(?:\.git)?", repository)
    if not match:
        return None
    owner, name = match.groups()
    return f"https://github.com/{owner}/{name}.git"


def _run_ci(pipeline: str, *, base: str, merge_base: bool) -> None:
    root = _load_repo_root()
    try:
        registry = Registry(root)
        changed_files = GitAdapter(root).changed_files(base, merge_base=merge_base)
        projects = affected_projects(registry, changed_files)
        runnable_projects = [project for project in projects if pipeline in project.ci]
        if not runnable_projects:
            console.print(f"No affected projects define \\[ci].{pipeline}.")
            return

        console.print(f"Running [bold]{pipeline}[/bold] CI for affected projects:")
        for project in runnable_projects:
            commands = ", ".join(project.ci[pipeline])
            console.print(f"  - {project.name}: {commands}")

        exit_code = run_ci_pipeline(
            registry, runnable_projects, pipeline, CommandRunner()
        )
    except WargError as error:
        console.print(f"[red]Error:[/red] {error}")
        raise typer.Exit(1) from error
    if exit_code != 0:
        raise typer.Exit(exit_code)


def _materialize_dependency_graph(
    root: Path, git: GitAdapter, project_name: str
) -> tuple[list[str], list[Project], set[str]]:
    requested_paths = {_path_for_project(root, project_name)}
    materialized: set[str] = set()

    while True:
        materialized.update(git.materialize_paths(sorted(requested_paths)))
        registry = Registry(root)
        order, discovered_paths = _discover_dependency_order(registry, project_name)
        discovered_paths |= _extra_paths_for_order(registry, order)
        missing_paths = discovered_paths - requested_paths
        if not missing_paths:
            paths = sorted(requested_paths)
            return paths, order, materialized
        requested_paths.update(missing_paths)


def _extra_paths_for_order(registry: Registry, order: list[Project]) -> set[str]:
    extra: set[str] = set()
    for project in order:
        entry = registry.entries.get(project.name)
        if entry is not None:
            extra.update(entry.extra_paths)
    return extra


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


def _unload_paths(
    registry: Registry, project_name: str, include_dependencies: bool
) -> tuple[list[str], set[str]]:
    paths = {_entry_path(registry, project_name)}
    dependent_names = expand_dependents(registry, {project_name}) - {project_name}
    dependents = {
        registry.projects[name].relative_path
        for name in dependent_names
        if name in registry.projects
    }
    paths.update(dependents)
    if include_dependencies and project_name in registry.projects:
        paths.update(
            project.relative_path for project in registry.dependency_order(project_name)
        )
    return sorted(paths), dependents


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
    if not registry.entries:
        console.print("No projects found.")
        return None
    return inquirer.fuzzy(
        message="Select a project",
        choices=sorted(registry.entries),
    ).execute()


def _pick_repository(organization: str, include_archived: bool) -> str | None:
    repositories = GitHubAdapter.list_org_repositories(organization, include_archived)
    if not repositories:
        console.print(f"No repositories found in {organization}.")
        return None

    choices = [
        Choice(
            name=repository.name,
            value=repository.ssh_url,
        )
        for repository in repositories
    ]
    return inquirer.fuzzy(
        message=f"Select a {organization} repository",
        choices=choices,
    ).execute()


def _pick_command(commands: dict[str, str]) -> str | None:
    if not commands:
        console.print("No commands found for this project.")
        return None
    return inquirer.fuzzy(
        message="Select a command",
        choices=sorted(commands),
    ).execute()


def main() -> None:
    try:
        app()
    except WargError as error:
        console.print(f"[red]Error:[/red] {error}")
        raise typer.Exit(1) from error


if __name__ == "__main__":
    main()
