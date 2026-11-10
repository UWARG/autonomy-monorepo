from __future__ import annotations

from pathlib import Path

import pytest

import startup
from constants import PROJECT_MANIFEST_FILENAME, ROOT_REGISTRY_FILENAME
from errors import ManifestError, StartupError
from registry import Registry
from startup import (
    UNIT_MARKER,
    MachineConfig,
    StartupEntry,
    SystemdUser,
    load_machine_config,
    render_command_unit,
    render_units,
    service_environment,
    startup_entries,
    sync_units,
    uninstall_units,
)

CAMERA_UNIT = "warg-camera-test:unit.service"
GESTURE_UNIT = "warg-gesture_control-sim:replay.service"
GENERATED = f"{UNIT_MARKER}.\n"
BOTH = MachineConfig(projects=("camera", "gesture_control"))


def test_startup_entries_cover_the_given_projects(startup_repo: Path) -> None:
    entries = startup_entries(Registry(startup_repo), ["camera", "gesture_control"])

    assert entries == [
        StartupEntry("camera", "test:unit", "on-failure"),
        StartupEntry("gesture_control", "sim:replay", "always"),
    ]
    assert [entry.unit_name for entry in entries] == [CAMERA_UNIT, GESTURE_UNIT]


def test_checked_out_projects_stay_off_until_the_machine_enables_them(
    startup_repo: Path,
) -> None:
    config = MachineConfig(projects=("gesture_control",))

    units = render_units(Registry(startup_repo), config, warg="/bin/warg")

    assert list(units) == [GESTURE_UNIT]


def test_enabling_an_unknown_project_is_rejected(startup_repo: Path) -> None:
    config = MachineConfig(projects=("missing",))

    with pytest.raises(ManifestError, match="Unknown project 'missing'"):
        render_units(Registry(startup_repo), config, warg="/bin/warg")


def test_machine_config_lists_projects_and_environment(tmp_path: Path) -> None:
    path = tmp_path / "startup.toml"
    path.write_text(
        'projects = ["camera", "camera"]\n\n[environment]\nROS_DOMAIN_ID = "7"\n'
    )

    config = load_machine_config(path)

    assert config.projects == ("camera",)
    assert config.environment == {"ROS_DOMAIN_ID": "7"}


def test_missing_machine_config_explains_how_to_create_it(tmp_path: Path) -> None:
    path = tmp_path / "startup.toml"

    with pytest.raises(StartupError, match="Missing startup config"):
        load_machine_config(path)

    assert load_machine_config(path, missing_ok=True) == MachineConfig()


@pytest.mark.parametrize(
    ("content", "message"),
    [
        ('projects = "camera"', "must be a list of project names"),
        ('project = ["camera"]', "unknown key\\(s\\): project"),
        ("[environment]\nJOBS = 4", "must map names to strings"),
        ('[environment]\n"MY VAR" = "1"', "not a valid environment variable name"),
        ("projects = [", "startup.toml: "),
    ],
)
def test_rejects_invalid_machine_config(
    tmp_path: Path, content: str, message: str
) -> None:
    path = tmp_path / "startup.toml"
    path.write_text(content + "\n")

    with pytest.raises(StartupError, match=message):
        load_machine_config(path)


def test_service_path_ignores_the_calling_shell(monkeypatch) -> None:
    monkeypatch.setenv("PATH", "/repo/.venv/bin:/usr/bin")
    monkeypatch.setattr(startup.Path, "home", lambda: Path("/home/pi"))

    environment = service_environment(MachineConfig())

    assert environment["PATH"] == (
        "/home/pi/.local/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin"
        ":/sbin:/bin"
    )


def test_machine_environment_overrides_the_defaults() -> None:
    config = MachineConfig(environment={"PATH": "/opt/ros/bin", "ROS_DOMAIN_ID": "7"})

    environment = service_environment(config)

    assert environment["PATH"] == "/opt/ros/bin"
    assert environment["ROS_DOMAIN_ID"] == "7"
    assert environment["PYTHONUNBUFFERED"] == "1"


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
        startup_entries(Registry(tmp_path), ["a", "a-b"])


def test_command_unit_runs_warg_with_restart_policy_and_environment() -> None:
    content = render_command_unit(
        StartupEntry("gesture_control", "sim:replay", "always"),
        root=Path("/home/pi/autonomy-monorepo"),
        warg="/home/pi/.local/bin/warg",
        environment={"PATH": "/home/pi/.local/bin:/usr/bin", "PYTHONUNBUFFERED": "1"},
    )

    assert content.startswith(UNIT_MARKER)
    lines = content.splitlines()
    assert "Description=WARG gesture_control: sim:replay" in lines
    assert "WorkingDirectory=/home/pi/autonomy-monorepo" in lines
    assert 'Environment="PATH=/home/pi/.local/bin:/usr/bin"' in lines
    assert 'Environment="PYTHONUNBUFFERED=1"' in lines
    assert "ExecStart=/home/pi/.local/bin/warg run gesture_control sim:replay" in lines
    assert "Restart=always" in lines
    assert "WantedBy=default.target" in lines


def test_restarting_units_give_up_after_repeated_failures() -> None:
    content = render_command_unit(
        StartupEntry("camera", "run", "on-failure"),
        root=Path("/repo"),
        warg="/bin/warg",
        environment={},
    )

    unit_section, service_section = content.split("[Service]")
    assert "StartLimitIntervalSec=60" in unit_section.splitlines()
    assert "StartLimitBurst=5" in unit_section.splitlines()
    assert "RestartSec=5" in service_section.splitlines()


def test_units_that_never_restart_have_no_start_limit() -> None:
    content = render_command_unit(
        StartupEntry("camera", "run", "no"),
        root=Path("/repo"),
        warg="/bin/warg",
        environment={},
    )

    assert "Restart=no" in content.splitlines()
    assert "StartLimit" not in content
    assert "RestartSec" not in content


def test_command_unit_escapes_systemd_special_characters() -> None:
    content = render_command_unit(
        StartupEntry("camera", 'say "100%" $HOME', "no"),
        root=Path("/srv/my 50% repo"),
        warg="/opt/warg tools/warg",
        environment={"PATH": '/a"b:/c%d'},
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
    units = render_units(Registry(startup_repo), BOTH, warg="/bin/warg")

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
    sync_units(fake_systemd, render_units(registry, BOTH, warg="/bin/warg"))
    fake_systemd.write_unit("warg-old-run.service", GENERATED)
    fake_systemd.write_unit("warg-handwritten.service", "[Unit]\n")
    fake_systemd.calls.clear()

    changed = MachineConfig(
        projects=("gesture_control",), environment={"ROS_DOMAIN_ID": "7"}
    )
    units = render_units(registry, changed, warg="/bin/warg")
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
    units = render_units(Registry(startup_repo), BOTH, warg="/bin/warg")
    sync_units(fake_systemd, units)
    fake_systemd.calls.clear()

    result = sync_units(fake_systemd, units)

    assert result.unchanged == [CAMERA_UNIT, GESTURE_UNIT]
    assert ("daemon-reload",) not in fake_systemd.calls


def test_sync_refuses_to_overwrite_units_warg_did_not_write(fake_systemd) -> None:
    fake_systemd.write_unit(CAMERA_UNIT, "[Unit]\nDescription=mine\n")

    with pytest.raises(StartupError, match="was not generated by warg"):
        sync_units(fake_systemd, {CAMERA_UNIT: GENERATED})

    assert fake_systemd.calls == []


def test_uninstall_removes_only_generated_units(fake_systemd) -> None:
    fake_systemd.write_unit(GESTURE_UNIT, GENERATED)
    fake_systemd.write_unit(CAMERA_UNIT, GENERATED)
    fake_systemd.write_unit("warg-handwritten.service", "[Unit]\n")
    fake_systemd.calls.clear()

    removed = uninstall_units(fake_systemd)

    assert removed == [CAMERA_UNIT, GESTURE_UNIT]
    assert [path.name for path in fake_systemd.unit_dir.iterdir()] == [
        "warg-handwritten.service"
    ]
    assert fake_systemd.calls == [
        ("disable", "--now", CAMERA_UNIT, GESTURE_UNIT),
        ("daemon-reload",),
    ]


def test_systemd_requires_linux(monkeypatch) -> None:
    monkeypatch.setattr(startup.sys, "platform", "darwin")

    with pytest.raises(StartupError, match="need Linux"):
        SystemdUser().ensure_available()
