# WARG Autonomy Monorepo

This repository is organized as a flat monorepo for WARG autonomy projects.
Each top-level directory is a project, and every project is registered from the
root `projects.toml` file.

## Layout

```text
.
├── README.md
├── projects.toml
└── <project>/
    ├── warg.toml
    └── ...
```

Projects own their local source, tests, dependencies, and commands. The root of
the repo owns only shared repository documentation and the project registry.

## Project registry

Register every project in `projects.toml`:

```toml
[projects.camera]
path = "camera"

[projects.mavlink_comm]
path = "mavlink_comm"

[projects.gesture_control]
path = "gesture_control"
```

The registry is intentionally explicit so dependency resolution and sparse
checkout paths stay predictable.

## Project manifests

Each project should include a `warg.toml` manifest:

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

Commands are project-defined, similar to `scripts` in `package.json`. Keep
project-specific setup and workflows in the project manifest rather than in the
root README.

## Projects

- `warg_cli`: developer CLI for materializing projects, inspecting manifests,
  and running project-defined commands. See [warg_cli/README.md](warg_cli/README.md).

- `kernel`: scheduler and state-management library for building airside autonomy projects. See [kernel/README.md](kernel/README.md).

-  `IMS`: Intelligent Monitoring System, a dashboard displaying MAVLink message feed, drone state, and live camera feed.

-  `Camera`: A Hardware Abstraction Layer for cameras. Normalizes frames from ArduCam, Oak-D, and simulation. 

- `Utils`: Shared enums and dataclasses used across all Airside modules. 

- `Mav_comms`: Wrapper around pymavlink.

- `airside`: ROS 2 Humble workspace for the airside architecture. See [airside/README.md](airside/README.md).
