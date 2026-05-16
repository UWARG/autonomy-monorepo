from __future__ import annotations

from pathlib import Path

import pytest


@pytest.fixture()
def fixture_repo(tmp_path: Path) -> Path:
    (tmp_path / ".git").mkdir()
    (tmp_path / "README.md").write_text("# Fixture\n")
    (tmp_path / "projects.toml").write_text(
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
language = "python"
description = "Camera project."
depends_on = []

[commands]
setup = "echo setup-camera"
test = "echo test-camera"
"test:unit" = "echo unit-camera"
""",
    )
    write_manifest(
        tmp_path,
        "mavlink_comm",
        """
name = "mavlink_comm"
language = "python"
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
language = "python"
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
    (project_dir / "warg.toml").write_text(content.strip() + "\n")
