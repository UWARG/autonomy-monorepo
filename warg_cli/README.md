# WARG CLI

`warg-cli` provides the `warg` command for working with the WARG autonomy
monorepo. It reads the root `projects.toml`, loads each project's `warg.toml`,
resolves project dependencies, manages Git sparse-checkout paths, and runs
project-defined commands.

## Installation

Install from PyPI:

```bash
uv tool install warg-cli
```

## Commands

Clone the WARG monorepo with sparse checkout enabled:

```bash
warg clone
warg clone autonomy-monorepo
warg clone git@github.com:warg/autonomy-monorepo.git
```

Only root files such as `README.md` and `projects.toml` are checked out
initially. Project directories stay absent until they are materialized. When
called without a repository, `warg clone` opens a searchable list of repositories
in the UWARG GitHub organization, sorted by most recently updated first. You can
also pass a UWARG repository name instead of a full clone URL.

List registered projects:

```bash
warg list
```

Materialize a project and its dependencies:

```bash
warg up gesture_control
```

Inspect a project's manifest metadata:

```bash
warg info gesture_control
```

Run commands defined in a project's `warg.toml`:

```bash
warg run camera test
warg run camera test:unit
warg run mavlink_comm lint -- --fix
```

If `warg up` or `warg run` is called without a project or command where one can
be selected interactively, the CLI prompts for a choice.

## Project commands

The CLI does not hardcode command names such as `test`, `lint`, or `run`.
Projects define their own command surface in `warg.toml`:

```toml
[commands]
setup = "uv sync"
test = "uv run pytest"
run = "uv run python -m gesture_control"
lint = "uv run ruff check ."
```

`warg up <project>` runs `setup` for newly materialized projects and their
dependencies. Use `warg up <project> --force` to rerun setup commands.

## Development

From the monorepo root directory:

```bash
warg run warg_cli setup
warg run warg_cli test
warg run warg_cli run -- --help
```

The package exposes the CLI entry point from `pyproject.toml`:

```toml
[project.scripts]
warg = "cli:app"
```
