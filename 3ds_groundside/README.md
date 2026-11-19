# 3DS Groundside

Groundside operator tooling for 3DS autonomy.

## Layout

```text
3ds_groundside/
├── src/           # Python package (import src.*)
├── tests/
├── pyproject.toml
└── warg.toml
```

## Commands

```bash
warg run 3ds_groundside setup
warg run 3ds_groundside test
warg run 3ds_groundside lint
```

## Airside rosbridge link

From this directory, run:

```bash
uv run python -m rosbridge_link status --watch
uv run python -m rosbridge_link send-setpoint 43.4339 -80.5776 10
uv run python -m rosbridge_link go-home
```

The default endpoint is `ws://10.42.3.2:9090`. Set
`THREEDS_ROSBRIDGE_URL` or pass `--url` before the subcommand to override it.
For local SITL, use `--url ws://127.0.0.1:9090`. If your workstation uses a
WebSocket proxy, add the airside address to `no_proxy` for the direct Wi-Fi link.

`status --watch` reports heartbeat loss after three seconds and reconnects.
A heartbeat confirms airside link health. Check the pilot's mission state
separately. Each command opens its own session, waits for airside heartbeats,
and sends one publish. The CLI reports unconfirmed delivery after a WebSocket
send. A connection failure during publish returns exit code 3 for unknown
delivery. Check airside state before manually repeating an uncertain command.

See the [adapter setup and bench checks](../3ds/README.md#dedicated-3ds-wi-fi-link).
