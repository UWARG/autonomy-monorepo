from __future__ import annotations

from pathlib import Path

import pytest

import startup
from constants import PROJECT_MANIFEST_FILENAME, ROOT_REGISTRY_FILENAME
from errors import StartupError
from registry import Registry
from startup import (
    SYNC_UNIT,
    UNIT_MARKER,
    StartupEntry,
    SystemdUser,
    install_sync_unit,
    render_command_unit,
    render_units,
    startup_entries,
    sync_units,
    uninstall_units,
)

CAMERA_UNIT = "warg-camera-test:unit.service"
GESTURE_UNIT = "warg-gesture_control-sim:replay.service"
GENERATED = f"{UNIT_MARKER}.\n"


def test_startup_entries_cover_checked_out_projects(startup_repo: Path) -> None:
    entries = startup_entries(Registry(startup_repo))

    assert entries == [
        StartupEntry("camera", "test:unit", "on-failure"),
        StartupEntry("gesture_control", "sim:replay", "always"),
    ]
    assert [entry.unit_name for entry in entries] == [CAMERA_UNIT, GESTURE_UNIT]


def test_unit_names_replace_unsafe_characters() -> None:
    entry = StartupEntry("SITL-Plus", "run sim/fast", "no")

    assert entry.unit_name == "warg-SITL-Plus-run_sim_fast.service"


def test_colliding_unit_names_are_rejected(tmp_path: Path) -> None:
    (tmp_path / ".git").mkdir()
    (tmp_path / ROOT_REGISTRY_FILENAME).write_text(
        '[projects.a]\npath = "a"\n\n[projects.a-b]\npath = "a-b"\n'
    )
    for project, command in [("a", "b-c"), ("a-b", "c")]:
        (tmp_path / project).mkdir()
        (tmp_path / project / PROJECT_MANIFEST_FILENAME).write_text(
            f'name = "{project}"\n\n[commands]\n"{command}" = "echo"\n\n'
            f'[startup]\ncommands = ["{command}"]\n'
        )

    with pytest.raises(StartupError, match="would both install warg-a-b-c.service"):
        startup_entries(Registry(tmp_path))


def test_command_unit_runs_warg_with_restart_policy_and_path() -> None:
    content = render_command_unit(
        StartupEntry("gesture_control", "sim:replay", "always"),
        root=Path("/home/pi/autonomy-monorepo"),
        warg="/home/pi/.local/bin/warg",
        path_env="/home/pi/.local/bin:/usr/bin",
    )

    assert content.startswith(UNIT_MARKER)
    lines = content.splitlines()
    assert "Description=WARG gesture_control: sim:replay" in lines
    assert f"After={SYNC_UNIT}" in lines
    assert "WorkingDirectory=/home/pi/autonomy-monorepo" in lines
    assert 'Environment="PATH=/home/pi/.local/bin:/usr/bin"' in lines
    assert 'Environment="PYTHONUNBUFFERED=1"' in lines
    assert "ExecStart=/home/pi/.local/bin/warg run gesture_control sim:replay" in lines
    assert "Restart=always" in lines
    assert "WantedBy=default.target" in lines


def test_command_unit_escapes_systemd_special_characters() -> None:
    content = render_command_unit(
        StartupEntry("camera", 'say "100%" $HOME', "no"),
        root=Path("/srv/my 50% repo"),
        warg="/opt/warg tools/warg",
        path_env='/a"b:/c%d',
    )

    lines = content.splitlines()
    assert "WorkingDirectory=/srv/my 50%% repo" in lines
    assert 'Environment="PATH=/a\\"b:/c%%d"' in lines
    assert (
        'ExecStart="/opt/warg tools/warg" run camera "say \\"100%%\\" $$HOME"' in lines
    )


def test_sync_writes_enables_and_starts_new_units(
    startup_repo: Path, fake_systemd
) -> None:
    units = render_units(Registry(startup_repo), warg="/bin/warg", path_env="/bin")

    result = sync_units(fake_systemd, units)

    assert result.added == [CAMERA_UNIT, GESTURE_UNIT]
    assert result.updated == result.removed == result.unchanged == []
    assert (fake_systemd.unit_dir / CAMERA_UNIT).read_text() == units[CAMERA_UNIT]
    assert fake_systemd.calls == [
        ("daemon-reload",),
        ("enable", CAMERA_UNIT, GESTURE_UNIT),
        ("start", "--no-block", CAMERA_UNIT, GESTURE_UNIT),
    ]


def test_sync_restarts_updated_units_and_removes_stale_ones(
    startup_repo: Path, fake_systemd
) -> None:
    registry = Registry(startup_repo)
    sync_units(fake_systemd, render_units(registry, warg="/bin/warg", path_env="/bin"))
    fake_systemd.write_unit("warg-old-run.service", GENERATED)
    fake_systemd.write_unit("warg-handwritten.service", "[Unit]\n")
    fake_systemd.calls.clear()

    units = render_units(registry, warg="/bin/warg", path_env="/usr/bin")
    del units[CAMERA_UNIT]
    result = sync_units(fake_systemd, units)

    assert result.updated == [GESTURE_UNIT]
    assert result.removed == [CAMERA_UNIT, "warg-old-run.service"]
    assert not (fake_systemd.unit_dir / CAMERA_UNIT).exists()
    assert (fake_systemd.unit_dir / "warg-handwritten.service").exists()
    assert fake_systemd.calls == [
        ("disable", "--now", CAMERA_UNIT, "warg-old-run.service"),
        ("daemon-reload",),
        ("enable", GESTURE_UNIT),
        ("try-restart", "--no-block", GESTURE_UNIT),
        ("start", "--no-block", GESTURE_UNIT),
    ]


def test_sync_skips_reload_when_nothing_changed(
    startup_repo: Path, fake_systemd
) -> None:
    units = render_units(Registry(startup_repo), warg="/bin/warg", path_env="/bin")
    sync_units(fake_systemd, units)
    fake_systemd.calls.clear()

    result = sync_units(fake_systemd, units)

    assert result.unchanged == [CAMERA_UNIT, GESTURE_UNIT]
    assert ("daemon-reload",) not in fake_systemd.calls


def test_sync_keeps_the_boot_sync_unit(fake_systemd) -> None:
    install_sync_unit(fake_systemd, GENERATED)

    result = sync_units(fake_systemd, {})

    assert result.removed == []
    assert (fake_systemd.unit_dir / SYNC_UNIT).exists()


def test_sync_refuses_to_overwrite_units_warg_did_not_write(fake_systemd) -> None:
    fake_systemd.write_unit(CAMERA_UNIT, "[Unit]\nDescription=mine\n")

    with pytest.raises(StartupError, match="was not generated by warg"):
        sync_units(fake_systemd, {CAMERA_UNIT: GENERATED})

    assert fake_systemd.calls == []


def test_uninstall_removes_only_generated_units(fake_systemd) -> None:
    install_sync_unit(fake_systemd, GENERATED)
    fake_systemd.write_unit(CAMERA_UNIT, GENERATED)
    fake_systemd.write_unit("warg-handwritten.service", "[Unit]\n")
    fake_systemd.calls.clear()

    removed = uninstall_units(fake_systemd)

    assert removed == [CAMERA_UNIT, SYNC_UNIT]
    assert [path.name for path in fake_systemd.unit_dir.iterdir()] == [
        "warg-handwritten.service"
    ]
    assert fake_systemd.calls == [
        ("disable", "--now", CAMERA_UNIT, SYNC_UNIT),
        ("daemon-reload",),
    ]


def test_systemd_requires_linux(monkeypatch) -> None:
    monkeypatch.setattr(startup.sys, "platform", "darwin")

    with pytest.raises(StartupError, match="need Linux"):
        SystemdUser().ensure_available()
