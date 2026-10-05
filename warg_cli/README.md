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

Fork, clone, and configure the autonomy bootcamp repository:

```bash
warg bootcamp
warg bootcamp my-bootcamp
```

This forks `UWARG/autonomy-bootcamp` into your account with the `gh` CLI,
clones the fork, and sets `origin` to your fork and `upstream` to the original
repository. It needs `gh` installed and logged in. See the
[bootcamp repository](https://github.com/UWARG/autonomy-bootcamp) for details.

Check the machine for common dev environment problems:

```bash
warg doctor
warg doctor --verbose
```

Doctor checks the tools the CLI depends on (Git, uv, Docker, gh), SSH access to
GitHub, and the current clone's remote, Git identity, and checked-out projects.
Each result is marked ok, warn, or fail, with a suggested fix for anything that
isn't ok. The command exits non-zero only when a check fails. Run it from inside
a clone to include the repository checks; elsewhere they are skipped.
`--verbose` prints the output of every command doctor ran.

List registered projects:

```bash
warg list
```

Materialize a project and its dependencies:

```bash
warg up gesture_control
```

Unload a project from sparse checkout:

```bash
warg down gesture_control
```

Projects that depend on the unloaded project are unloaded too, with a warning.
Pass `--include-dependencies` to also unload checked-out dependencies.

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

`warg up <project>` runs `setup` for the project and its dependencies every
time, keeping already-materialized projects in sync with their current setup
commands.

## Startup commands

On a Raspberry Pi or any other Linux machine with systemd, a project can run
its commands as background services that start at boot. List them under
`[startup]` in `warg.toml`:

```toml
[commands]
run = "uv run python -m gesture_control"

[startup]
commands = ["run"]
restart = "on-failure"  # optional: "on-failure" (default), "always", or "no"
```

Each entry must name a command from `[commands]`. If a service needs different
flags than you use during development, give it its own command, such as
`"run:drone" = "uv run python -m gesture_control --headless"`.

Checking a project out does not turn its startup commands on. Each machine
opts in through its own config file, `~/.config/warg/startup.toml`, which lives
outside the repo:

```toml
projects = ["gesture_control"]

[environment]            # optional: extra variables for every service
ROS_DOMAIN_ID = "7"
```

Then, from inside the clone:

```bash
warg up gesture_control
warg startup sync
```

`sync` creates a systemd user service for each startup command of the listed
projects, starts them, and enables them at boot. It is safe to rerun: it
installs what is missing, restarts services whose definition changed, and
removes services that are no longer listed. Nothing resyncs on its own, so the
installed services keep running as they were while you switch branches or pull.
Rerun `warg startup sync` to apply a change to `[startup]` or to the machine
config.

```bash
warg startup list        # startup commands and whether each service is running
warg startup sync        # install or update this machine's services
warg startup uninstall   # stop and remove every warg startup service
journalctl --user -u warg-gesture_control-run -f   # follow a service's logs
```

Each service runs `warg run <project> <command>` from the clone, as the user who
ran `sync`. Services do not inherit that shell's environment. `PATH` is always
`~/.local/bin` followed by the standard system directories, so an active venv
never leaks in. Anything else a command needs goes in `[environment]`, which can
also override `PATH`.

With `restart = "on-failure"`, systemd restarts a command 5 seconds after it
exits with an error; with `"always"`, after any exit. A service that fails 5
times within 60 seconds stops retrying and shows as `failed` in
`warg startup list`. Once the cause is fixed, bring it back with:

```bash
systemctl --user reset-failed warg-gesture_control-run
warg startup sync
```

Services are named after the project and command only, so they are shared by
every clone on the machine. Keep startup commands to one clone per user:
running `sync` from a second clone repoints the services at that clone.

Services start at boot without anyone logging in only when lingering is enabled
for that user. `sync` tries to enable it and, if it can't, prints the command to
run:

```bash
sudo loginctl enable-linger $USER
```

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
