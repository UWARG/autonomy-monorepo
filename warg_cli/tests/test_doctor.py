from __future__ import annotations

from pathlib import Path

import doctor
from doctor import CheckStatus, Probe, run_doctor

GITHUB_AUTH_OUTPUT = (
    "Hi octocat! You've been successfully authenticated, but GitHub does not "
    "provide shell access."
)
SSH_COMMAND = [
    "ssh",
    "-T",
    "-o",
    "BatchMode=yes",
    "-o",
    "ConnectTimeout=5",
    "git@github.com",
]
SSH_443_COMMAND = [
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


def install_fakes(
    monkeypatch,
    tmp_path: Path,
    responses: dict[tuple[str, ...], Probe],
    *,
    with_ssh_key: bool = True,
) -> None:
    def fake_run_probe(command: list[str], **kwargs) -> Probe:
        key = tuple(command)
        if key in responses:
            return responses[key]
        return Probe(command, None, "", error="command not found")

    monkeypatch.setattr(doctor, "run_probe", fake_run_probe)

    ssh_dir = tmp_path / ".ssh"
    ssh_dir.mkdir()
    if with_ssh_key:
        (ssh_dir / "id_ed25519.pub").write_text("ssh-ed25519 AAAA\n")
    monkeypatch.setattr(Path, "home", lambda: tmp_path)


def default_responses() -> dict[tuple[str, ...], Probe]:
    def ok(command: list[str], output: str) -> Probe:
        return Probe(command, 0, output)

    return {
        ("git", "--version"): ok(["git", "--version"], "git version 2.44.0"),
        ("uv", "--version"): ok(["uv", "--version"], "uv 0.5.24"),
        ("docker", "--version"): ok(["docker", "--version"], "Docker version 27.0.3"),
        ("docker", "compose", "version"): ok(
            ["docker", "compose", "version"], "Docker Compose version v2.28.1"
        ),
        ("docker", "info"): ok(["docker", "info"], "Server Version: 27.0.3"),
        ("gh", "--version"): ok(["gh", "--version"], "gh version 2.52.0"),
        tuple(SSH_COMMAND): Probe(SSH_COMMAND, 1, GITHUB_AUTH_OUTPUT),
        ("git", "config", "--get", "remote.origin.url"): ok(
            ["git", "config", "--get", "remote.origin.url"],
            "git@github.com:UWARG/autonomy-monorepo.git",
        ),
        ("git", "config", "--get", "remote.origin.partialclonefilter"): ok(
            ["git", "config", "--get", "remote.origin.partialclonefilter"],
            "blob:none",
        ),
        ("git", "config", "--get", "user.name"): ok(
            ["git", "config", "--get", "user.name"], "First Year"
        ),
        ("git", "config", "--get", "user.email"): ok(
            ["git", "config", "--get", "user.email"], "fy@example.com"
        ),
    }


def make_repo(tmp_path: Path) -> Path:
    root = tmp_path / "repo"
    (root / "camera").mkdir(parents=True)
    (root / "projects.toml").write_text(
        '[projects.camera]\npath = "camera"\n\n[projects.gnc]\npath = "gnc"\n'
    )
    (root / "camera" / "warg.toml").write_text('name = "camera"\n')
    return root


def checks_by_section(sections) -> dict[str, list]:
    return {section.title: section.checks for section in sections}


def test_all_checks_pass_in_healthy_environment(tmp_path: Path, monkeypatch) -> None:
    install_fakes(monkeypatch, tmp_path, default_responses())

    sections = run_doctor(make_repo(tmp_path))

    checks = [check for section in sections for check in section.checks]
    assert all(check.status == CheckStatus.OK for check in checks)

    by_section = checks_by_section(sections)
    details = [check.detail for check in by_section["GitHub access"]]
    assert any("octocat" in detail for detail in details)
    repo_details = [check.detail for check in by_section["Repository"]]
    assert any("1 of 2 registered projects" in detail for detail in repo_details)


def test_missing_uv_fails_with_install_hint(tmp_path: Path, monkeypatch) -> None:
    responses = default_responses()
    del responses[("uv", "--version")]
    install_fakes(monkeypatch, tmp_path, responses)

    sections = run_doctor(make_repo(tmp_path))

    tools = checks_by_section(sections)["Tools"]
    uv_check = next(check for check in tools if "uv" in check.detail)
    assert uv_check.status == CheckStatus.FAIL
    assert "astral.sh/uv" in uv_check.fix


def test_old_git_version_warns(tmp_path: Path, monkeypatch) -> None:
    responses = default_responses()
    responses[("git", "--version")] = Probe(
        ["git", "--version"], 0, "git version 2.25.1"
    )
    install_fakes(monkeypatch, tmp_path, responses)

    sections = run_doctor(make_repo(tmp_path))

    git_check = checks_by_section(sections)["Tools"][0]
    assert git_check.status == CheckStatus.WARN
    assert "sparse checkout" in git_check.detail


def test_missing_docker_collapses_to_single_warning(
    tmp_path: Path, monkeypatch
) -> None:
    responses = default_responses()
    for key in [
        ("docker", "--version"),
        ("docker", "compose", "version"),
        ("docker", "info"),
    ]:
        del responses[key]
    install_fakes(monkeypatch, tmp_path, responses)

    sections = run_doctor(make_repo(tmp_path))

    tools = checks_by_section(sections)["Tools"]
    docker_checks = [check for check in tools if "Docker" in check.detail]
    assert len(docker_checks) == 1
    assert docker_checks[0].status == CheckStatus.WARN


def test_stopped_docker_daemon_warns(tmp_path: Path, monkeypatch) -> None:
    responses = default_responses()
    responses[("docker", "info")] = Probe(
        ["docker", "info"], 1, "Cannot connect to the Docker daemon"
    )
    install_fakes(monkeypatch, tmp_path, responses)

    sections = run_doctor(make_repo(tmp_path))

    tools = checks_by_section(sections)["Tools"]
    daemon_check = next(check for check in tools if "daemon" in check.detail)
    assert daemon_check.status == CheckStatus.WARN
    assert "Docker Desktop" in daemon_check.fix


def test_missing_ssh_key_warns_with_keygen_hint(tmp_path: Path, monkeypatch) -> None:
    install_fakes(monkeypatch, tmp_path, default_responses(), with_ssh_key=False)

    sections = run_doctor(make_repo(tmp_path))

    key_check = checks_by_section(sections)["GitHub access"][0]
    assert key_check.status == CheckStatus.WARN
    assert "ssh-keygen" in key_check.fix


def test_blocked_port_22_suggests_ssh_over_443(tmp_path: Path, monkeypatch) -> None:
    responses = default_responses()
    responses[tuple(SSH_COMMAND)] = Probe(
        SSH_COMMAND,
        255,
        "ssh: connect to host github.com port 22: Connection timed out",
    )
    responses[tuple(SSH_443_COMMAND)] = Probe(SSH_443_COMMAND, 1, GITHUB_AUTH_OUTPUT)
    install_fakes(monkeypatch, tmp_path, responses)

    sections = run_doctor(make_repo(tmp_path))

    auth_check = checks_by_section(sections)["GitHub access"][1]
    assert auth_check.status == CheckStatus.WARN
    assert "port 443" in auth_check.detail
    assert "ssh.github.com" in auth_check.fix


def test_unreachable_github_fails(tmp_path: Path, monkeypatch) -> None:
    responses = default_responses()
    responses[tuple(SSH_COMMAND)] = Probe(
        SSH_COMMAND, 255, "git@github.com: Permission denied (publickey)."
    )
    install_fakes(monkeypatch, tmp_path, responses)

    sections = run_doctor(make_repo(tmp_path))

    auth_check = checks_by_section(sections)["GitHub access"][1]
    assert auth_check.status == CheckStatus.FAIL
    assert "docs.github.com" in auth_check.fix


def test_https_remote_warns(tmp_path: Path, monkeypatch) -> None:
    responses = default_responses()
    responses[("git", "config", "--get", "remote.origin.url")] = Probe(
        ["git", "config", "--get", "remote.origin.url"],
        0,
        "https://github.com/UWARG/autonomy-monorepo.git",
    )
    install_fakes(monkeypatch, tmp_path, responses)

    sections = run_doctor(make_repo(tmp_path))

    remote_check = checks_by_section(sections)["Repository"][0]
    assert remote_check.status == CheckStatus.WARN
    assert "HTTPS" in remote_check.detail


def test_missing_git_identity_fails(tmp_path: Path, monkeypatch) -> None:
    responses = default_responses()
    responses[("git", "config", "--get", "user.name")] = Probe(
        ["git", "config", "--get", "user.name"], 1, ""
    )
    install_fakes(monkeypatch, tmp_path, responses)

    sections = run_doctor(make_repo(tmp_path))

    repo_checks = checks_by_section(sections)["Repository"]
    identity_check = next(check for check in repo_checks if "identity" in check.detail)
    assert identity_check.status == CheckStatus.FAIL
    assert "user.name" in identity_check.detail
    assert "git config --global" in identity_check.fix


def test_outside_repository_skips_repo_checks(tmp_path: Path, monkeypatch) -> None:
    install_fakes(monkeypatch, tmp_path, default_responses())

    sections = run_doctor(None)

    repo_checks = checks_by_section(sections)["Repository"]
    assert len(repo_checks) == 1
    assert repo_checks[0].status == CheckStatus.WARN
    assert "warg clone" in repo_checks[0].fix
