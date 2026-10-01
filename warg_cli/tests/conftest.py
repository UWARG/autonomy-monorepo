from __future__ import annotations

import subprocess
from pathlib import Path

import pytest

from constants import PROJECT_MANIFEST_FILENAME, ROOT_REGISTRY_FILENAME
from startup import SystemdUser


class FakeSystemd(SystemdUser):
    def __init__(self, unit_dir: Path):
        super().__init__(unit_dir)
        self.calls: list[tuple[str, ...]] = []
        self.linger = True
        self.linger_requests = 0

    def ensure_available(self) -> None:
        pass

    def linger_enabled(self) -> bool:
        return self.linger

    def enable_linger(self) -> bool:
        self.linger_requests += 1
        return False

    def _systemctl(
        self, *args: str, check: bool = True
    ) -> subprocess.CompletedProcess[str]:
        self.calls.append(args)
        stdout = "active\n" if args[0] == "is-active" else ""
        return subprocess.CompletedProcess(
            ["systemctl", "--user", *args], 0, stdout, ""
        )


@pytest.fixture()
def fake_systemd(tmp_path: Path) -> FakeSystemd:
    return FakeSystemd(tmp_path / "systemd-units")


@pytest.fixture()
def startup_repo(fixture_repo: Path) -> Path:
    for project, section in [
        ("camera", 'commands = ["test:unit"]'),
        ("gesture_control", 'commands = ["sim:replay"]\nrestart = "always"'),
    ]:
        manifest = fixture_repo / project / PROJECT_MANIFEST_FILENAME
        manifest.write_text(manifest.read_text() + f"\n[startup]\n{section}\n")
    return fixture_repo


@pytest.fixture()
def fixture_repo(tmp_path: Path) -> Path:
    (tmp_path / ".git").mkdir()
    (tmp_path / "README.md").write_text("# Fixture\n")
    (tmp_path / ROOT_REGISTRY_FILENAME).write_text(
        """
[projects.camera]
path = "camera"

[projects.gesture_control]
path = "gesture_control"

[projects.mavlink_comm]
path = "mavlink_comm"
""".strip()
        + "\n"
    )

    write_manifest(
        tmp_path,
        "camera",
        """
name = "camera"
description = "Camera project."
depends_on = []

[commands]
setup = "echo setup-camera"
lint = "echo lint-camera"
test = "echo test-camera"
"test:unit" = "echo unit-camera"

[ci]
pr = ["test"]
main = ["lint", "test"]
""",
    )
    write_manifest(
        tmp_path,
        "mavlink_comm",
        """
name = "mavlink_comm"
depends_on = []

[commands]
setup = "echo setup-mavlink"
lint = "echo lint-mavlink"
""",
    )
    write_manifest(
        tmp_path,
        "gesture_control",
        """
name = "gesture_control"
depends_on = ["camera", "mavlink_comm"]

[commands]
setup = "echo setup-gesture"
"sim:replay" = "echo replay"
""",
    )

    return tmp_path


def write_manifest(root: Path, project: str, content: str) -> None:
    project_dir = root / project
    project_dir.mkdir()
    (project_dir / PROJECT_MANIFEST_FILENAME).write_text(content.strip() + "\n")
