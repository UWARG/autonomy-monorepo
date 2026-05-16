# WARG Autonomy Monorepo

This repository is organized as a flat monorepo. Each top-level directory is a
project with its own `warg.toml` manifest. Projects are deliberately registered
in the root `projects.toml` file.

The `warg` CLI reads `projects.toml`, resolves project dependencies, manages Git
sparse-checkout paths, and runs project-defined commands.

## Project registry

Register every project in the root `projects.toml`:

```toml
[projects.camera]
path = "camera"

[projects.mavlink_comm]
path = "mavlink_comm"

[projects.gesture_control]
path = "gesture_control"
```

## Project manifests

Each project should include a `warg.toml`:

```toml
name = "gesture_control"
description = "Gesture control pipeline for camera-based command input."

depends_on = ["camera", "mavlink_comm"]

[commands]
setup = "uv sync"
test = "uv run pytest"
"test:unit" = "uv run pytest tests/unit"
"test:integration" = "uv run pytest tests/integration"
run = "uv run python -m gesture_control"
lint = "uv run ruff check ."
```

Commands are intentionally project-defined, similar to `scripts` in
`package.json`. The CLI does not hardcode project commands like `test` or `lint`.

## CLI examples

```bash
warg clone
warg clone autonomy-monorepo
warg clone git@github.com:warg/autonomy-monorepo.git
warg list
warg up gesture_control
warg info gesture_control
warg run camera test
warg run camera test:unit
warg run mavlink_comm lint -- --fix
```

`warg clone` uses Git sparse checkout and partial clone support so only root
files such as `README.md` and `projects.toml` are checked out initially. Project
directories stay absent until you materialize one with `warg up <project>`. When
called without a repository, `warg clone` opens a searchable list of repositories
in the UWARG GitHub organization. You can also pass a UWARG repository name
instead of a full clone URL.
