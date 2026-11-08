from __future__ import annotations

import re
import subprocess
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from typing import Optional

try:
    import tomllib
except ModuleNotFoundError:  # pragma: no cover - exercised on Python < 3.11
    import tomli as tomllib  # type: ignore[no-redef]

from constants import PROJECT_MANIFEST_FILENAME, ROOT_REGISTRY_FILENAME

MIN_GIT_VERSION = (2, 36)


class CheckStatus(str, Enum):
    OK = "ok"
    WARN = "warn"
    FAIL = "fail"


@dataclass
class Probe:
    command: list[str]
    code: Optional[int]
    output: str
    error: Optional[str] = None

    @property
    def transcript(self) -> str:
        body = self.error or self.output or "(no output)"
        return f"$ {' '.join(self.command)}\n{body}"


@dataclass
class Check:
    status: CheckStatus
    detail: str
    fix: Optional[str] = None
    transcript: list[str] = field(default_factory=list)


@dataclass
class Section:
    title: str
    checks: list[Check]


def run_probe(
    command: list[str], *, cwd: Optional[Path] = None, timeout: int = 10
) -> Probe:
    try:
        result = subprocess.run(
            command,
            cwd=cwd,
            check=False,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=timeout,
        )
    except FileNotFoundError:
        return Probe(command, None, "", error="command not found")
    except subprocess.TimeoutExpired:
        return Probe(command, None, "", error=f"timed out after {timeout} seconds")

    output = (result.stdout.strip() + "\n" + result.stderr.strip()).strip()
    return Probe(command, result.returncode, output)


def run_doctor(root: Optional[Path]) -> list[Section]:
    return [
        Section("Tools", [_git_check(), _uv_check(), *_docker_checks(), _gh_check()]),
        Section("GitHub access", [_ssh_key_check(), _github_ssh_check()]),
        Section("Repository", _repository_checks(root)),
    ]


def _first_line(value: str) -> str:
    return next((line.strip() for line in value.splitlines() if line.strip()), "")


def _git_check() -> Check:
    probe = run_probe(["git", "--version"])
    if probe.code != 0:
        return Check(
            CheckStatus.FAIL,
            "Git is not installed.",
            fix="Install Git: https://git-scm.com/downloads",
            transcript=[probe.transcript],
        )
    match = re.search(r"(\d+)\.(\d+)", probe.output)
    version = (int(match.group(1)), int(match.group(2))) if match else None
    if version and version < MIN_GIT_VERSION:
        minimum = ".".join(str(part) for part in MIN_GIT_VERSION)
        return Check(
            CheckStatus.WARN,
            f"{_first_line(probe.output)} is older than {minimum}, which is "
            "known to mishandle sparse checkout.",
            fix=f"Upgrade Git to {minimum} or newer.",
            transcript=[probe.transcript],
        )
    return Check(
        CheckStatus.OK, _first_line(probe.output), transcript=[probe.transcript]
    )


def _uv_check() -> Check:
    probe = run_probe(["uv", "--version"])
    if probe.code != 0:
        return Check(
            CheckStatus.FAIL,
            "uv is not installed. Every project's setup runs through it.",
            fix=(
                "curl -LsSf https://astral.sh/uv/install.sh | sh "
                "(see https://docs.astral.sh/uv/)"
            ),
            transcript=[probe.transcript],
        )
    return Check(
        CheckStatus.OK, _first_line(probe.output), transcript=[probe.transcript]
    )


def _docker_checks() -> list[Check]:
    probe = run_probe(["docker", "--version"])
    if probe.code != 0:
        return [
            Check(
                CheckStatus.WARN,
                "Docker is not installed. The airside project needs it.",
                fix="Install Docker Desktop: https://docs.docker.com/get-docker/",
                transcript=[probe.transcript],
            )
        ]

    checks = [
        Check(CheckStatus.OK, _first_line(probe.output), transcript=[probe.transcript])
    ]

    compose = run_probe(["docker", "compose", "version"])
    if compose.code != 0:
        checks.append(
            Check(
                CheckStatus.WARN,
                "Docker Compose v2 is not available. Airside commands run "
                "'docker compose'.",
                fix=(
                    "Install Docker Desktop, or the docker-compose-plugin "
                    "package on Linux."
                ),
                transcript=[compose.transcript],
            )
        )
    else:
        checks.append(
            Check(
                CheckStatus.OK,
                _first_line(compose.output),
                transcript=[compose.transcript],
            )
        )

    info = run_probe(["docker", "info"])
    if info.code != 0:
        checks.append(
            Check(
                CheckStatus.WARN,
                "Docker is installed but the daemon is not running.",
                fix=(
                    "Start Docker Desktop, or run 'sudo systemctl start docker' "
                    "on Linux."
                ),
                transcript=[info.transcript],
            )
        )
    else:
        checks.append(
            Check(
                CheckStatus.OK,
                "Docker daemon is running.",
                transcript=[info.transcript],
            )
        )
    return checks


def _gh_check() -> Check:
    probe = run_probe(["gh", "--version"])
    if probe.code != 0:
        return Check(
            CheckStatus.WARN,
            "The gh CLI is not installed, so the 'warg clone' repo picker falls "
            "back to the GitHub API.",
            fix="Install gh: https://cli.github.com",
            transcript=[probe.transcript],
        )
    return Check(
        CheckStatus.OK, _first_line(probe.output), transcript=[probe.transcript]
    )


def _ssh_key_check() -> Check:
    ssh_dir = Path.home() / ".ssh"
    found = sorted(path.name for path in ssh_dir.glob("*.pub"))
    transcript = [
        f"looked for *.pub files in {ssh_dir}\nfound: {', '.join(found) or '(none)'}"
    ]
    if not found:
        return Check(
            CheckStatus.WARN,
            f"No SSH key found in {ssh_dir}.",
            fix=(
                "Generate one with 'ssh-keygen -t ed25519 -C \"you@example.com\"' "
                "and add it to GitHub: "
                "https://docs.github.com/en/authentication/connecting-to-github-with-ssh"
            ),
            transcript=transcript,
        )
    return Check(
        CheckStatus.OK, f"SSH key found: {ssh_dir / found[0]}", transcript=transcript
    )


def _github_ssh_check() -> Check:
    def authenticated(probe: Probe) -> bool:
        return probe.code == 0 or "successfully authenticated" in probe.output

    probe = run_probe(
        [
            "ssh",
            "-T",
            "-o",
            "BatchMode=yes",
            "-o",
            "ConnectTimeout=5",
            "git@github.com",
        ]
    )
    if authenticated(probe):
        match = re.search(r"Hi ([^!\s]+)!", probe.output)
        username = f" as {match.group(1)}" if match else ""
        return Check(
            CheckStatus.OK,
            f"Authenticated to GitHub over SSH{username}.",
            transcript=[probe.transcript],
        )

    fallback = run_probe(
        [
            "ssh",
            "-T",
            "-p",
            "443",
            "-o",
            "BatchMode=yes",
            "-o",
            "ConnectTimeout=5",
            "git@ssh.github.com",
        ]
    )
    if authenticated(fallback):
        return Check(
            CheckStatus.WARN,
            "Port 22 to GitHub is blocked on this network, but SSH over port "
            "443 works.",
            fix=(
                "Add to ~/.ssh/config: 'Host github.com' / "
                "'Hostname ssh.github.com' / 'Port 443'"
            ),
            transcript=[probe.transcript, fallback.transcript],
        )
    return Check(
        CheckStatus.FAIL,
        "Could not authenticate to GitHub over SSH. Cloning and pushing will fail.",
        fix=(
            "Add your SSH key to your GitHub account: "
            "https://docs.github.com/en/authentication/connecting-to-github-with-ssh"
        ),
        transcript=[probe.transcript, fallback.transcript],
    )


def _repository_checks(root: Optional[Path]) -> list[Check]:
    if root is None:
        return [
            Check(
                CheckStatus.WARN,
                "Not inside a monorepo clone, so the repository checks were skipped.",
                fix="Clone one with 'warg clone', or run doctor from inside it.",
            )
        ]

    return [
        _remote_check(root),
        _clone_type_check(root),
        _identity_check(),
        _projects_check(root),
    ]


def _remote_check(root: Path) -> Check:
    probe = run_probe(["git", "config", "--get", "remote.origin.url"], cwd=root)
    url = _first_line(probe.output)
    if probe.code != 0 or not url:
        return Check(
            CheckStatus.WARN,
            "No 'origin' remote is configured.",
            fix="Reclone with 'warg clone'.",
            transcript=[probe.transcript],
        )
    if url.startswith("https://"):
        return Check(
            CheckStatus.WARN,
            f"Remote 'origin' uses HTTPS ({url}), so pushing will fail without "
            "a token.",
            fix=(
                "Set up SSH access and reclone with 'warg clone': "
                "https://docs.github.com/en/authentication/connecting-to-github-with-ssh"
            ),
            transcript=[probe.transcript],
        )
    return Check(
        CheckStatus.OK,
        f"Remote 'origin' uses SSH ({url}).",
        transcript=[probe.transcript],
    )


def _clone_type_check(root: Path) -> Check:
    probe = run_probe(
        ["git", "config", "--get", "remote.origin.partialclonefilter"], cwd=root
    )
    if probe.code == 0 and _first_line(probe.output):
        return Check(
            CheckStatus.OK,
            "Partial clone with sparse checkout is configured.",
            transcript=[probe.transcript],
        )
    return Check(
        CheckStatus.OK,
        "Full clone with no partial-clone filter. That works, but 'warg clone' "
        "downloads less.",
        transcript=[probe.transcript],
    )


def _identity_check() -> Check:
    name = run_probe(["git", "config", "--get", "user.name"])
    email = run_probe(["git", "config", "--get", "user.email"])
    transcript = [name.transcript, email.transcript]
    missing = []
    if name.code != 0 or not _first_line(name.output):
        missing.append("user.name")
    if email.code != 0 or not _first_line(email.output):
        missing.append("user.email")
    if missing:
        return Check(
            CheckStatus.FAIL,
            f"Git identity is not set: {', '.join(missing)}. Commits will fail.",
            fix=(
                'git config --global user.name "Your Name" && '
                'git config --global user.email "you@example.com"'
            ),
            transcript=transcript,
        )
    return Check(
        CheckStatus.OK,
        f"Git identity: {_first_line(name.output)} <{_first_line(email.output)}>.",
        transcript=transcript,
    )


def _projects_check(root: Path) -> Check:
    registry_path = root / ROOT_REGISTRY_FILENAME
    if not registry_path.exists():
        return Check(
            CheckStatus.WARN,
            f"{ROOT_REGISTRY_FILENAME} is missing from the repository root. Is "
            "this the monorepo?",
        )
    try:
        with registry_path.open("rb") as file:
            data = tomllib.load(file)
        entries = data.get("projects", {})
        paths = {
            name: metadata.get("path", "")
            for name, metadata in entries.items()
            if isinstance(metadata, dict)
        }
    except tomllib.TOMLDecodeError as error:
        return Check(
            CheckStatus.WARN, f"Could not parse {ROOT_REGISTRY_FILENAME}: {error}"
        )

    checked_out = sorted(
        name
        for name, path in paths.items()
        if path and (root / path / PROJECT_MANIFEST_FILENAME).exists()
    )
    transcript = [f"checked out: {', '.join(checked_out) or '(none)'}"]
    return Check(
        CheckStatus.OK,
        f"{len(checked_out)} of {len(paths)} registered projects checked out. "
        "Run 'warg up <project>' to add more.",
        transcript=transcript,
    )
