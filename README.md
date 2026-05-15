# WARG Autonomy Monorepo

This repository is organized as a flat monorepo. Each top-level directory is a
project with its own `warg.toml` manifest.

The `warg` CLI discovers those manifests, resolves project dependencies, manages
Git sparse-checkout paths, and runs project-defined commands.

Sparse checkouts always keep the `warg_cli` project available so the developer
tooling remains present while individual autonomy projects are materialized.

## Project manifests

Each project should include a `warg.toml`:

```toml
name = "gesture_control"
language = "python"
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
warg list
warg up gesture_control
warg info gesture_control
warg run camera test
warg run camera test:unit
warg run mavlink_comm lint -- --fix
```
