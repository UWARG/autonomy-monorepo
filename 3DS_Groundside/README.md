# 3DS Groundside

> [!WARNING]
> This is a skeletal project and has no functionality yet.

Groundside operator tooling for 3DS autonomy.

## Layout

```text
3DS_Groundside/
├── src/           # Python package (import src.*)
├── tests/
├── pyproject.toml
└── warg.toml
```

## Commands

```bash
warg run 3DS_Groundside setup
warg run 3DS_Groundside test
warg run 3DS_Groundside lint
```

## Setpoint tester

`src/setpoint_tester` is a command prompt for flight tests that sends setpoints
and go-home to a drone's `setpoint_link` node over TCP (protocol: see "Setpoint
tester link" in `3ds/README.md`).

```bash
uv run python -m src.setpoint_tester --host <drone-ip>   # --port 7070 --timeout 3
```

| Command | Effect |
|---|---|
| `goto LAT LON ALT` | Fly to the point and photograph it (`ALT` in meters above launch). Commas are ignored, so coordinates pasted from a map work: `goto 43.4339, -80.5776, 10` |
| `home` | Return to the launch point and land (ends the mission) |
| `help` / `quit` | Show commands / exit (also Ctrl-D) |

Each command prints the drone's reply: `OK` (handed to the engine) or `REJECTED`
with the reason. `FAILED` means the link broke and says whether the command may
still have arrived; the tester reconnects on the next command, so re-send if
needed. Without `--host` it connects to `127.0.0.1`, i.e. SITL on this machine.
