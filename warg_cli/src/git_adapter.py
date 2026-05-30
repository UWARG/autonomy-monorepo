from __future__ import annotations

import os
import subprocess
from pathlib import Path

from errors import GitError


class GitAdapter:
    def __init__(self, root: Path | None):
        self.root = root

    @classmethod
    def clone(cls, repository: str, destination: str | None = None) -> None:
        command = ["git", "clone", repository]
        if destination:
            command.append(destination)

        cls._run_clone_command(command)

    @classmethod
    def clone_sparse(cls, repository: str, destination: str | None = None) -> None:
        command = ["git", "clone", "--filter=blob:none", "--sparse"]
        command.append(repository)
        if destination:
            command.append(destination)

        cls._run_clone_command(command)

    @staticmethod
    def _run_clone_command(command: list[str]) -> None:
        result = subprocess.run(
            command,
            check=False,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        if result.returncode != 0:
            message = result.stderr.strip() or result.stdout.strip()
            raise GitError(f"{' '.join(command)} failed: {message}")

    def sparse_checkout_enabled(self) -> bool:
        result = self._run("config", "--bool", "core.sparseCheckout", check=False)
        return result.returncode == 0 and result.stdout.strip() == "true"

    def enable_sparse_checkout(self) -> None:
        self._run("sparse-checkout", "init", "--cone")

    def set_sparse_paths(self, paths: list[str]) -> None:
        self._run("sparse-checkout", "set", *paths)

    def current_sparse_paths(self) -> set[str]:
        result = self._run("sparse-checkout", "list", check=False)
        if result.returncode != 0:
            return set()
        return {line.strip() for line in result.stdout.splitlines() if line.strip()}

    def materialize_paths(self, paths: list[str]) -> set[str]:
        before = self.current_sparse_paths()
        if not self.sparse_checkout_enabled():
            self.enable_sparse_checkout()
        desired = sorted({*before, *paths})
        self.set_sparse_paths(desired)
        return set(desired) - before

    def unmaterialize_paths(self, paths: list[str]) -> set[str]:
        before = self.current_sparse_paths()
        if not before:
            return set()
        desired = sorted(before - set(paths))
        self.set_sparse_paths(desired)
        return before - set(desired)

    def changed_files(self, base: str, *, merge_base: bool) -> list[Path]:
        separator = "..." if merge_base else ".."
        result = self._run("diff", "--name-only", f"{base}{separator}HEAD")
        return [
            Path(line.strip()) for line in result.stdout.splitlines() if line.strip()
        ]

    def repository_access_diagnostics(self) -> list[str]:
        lines = []
        if self.root is None:
            lines.append("Git repository: not found")

        else:
            remote_url = self._config_value("remote.origin.url") or "(not set)"
            promisor = self._config_value("remote.origin.promisor") or "(not set)"
            partial_filter = (
                self._config_value("remote.origin.partialclonefilter") or "(not set)"
            )
            core_ssh_command = self._config_value("core.sshCommand") or "(not set)"
            lines += [
                f"Git repository: {self.root}",
                f"remote.origin.url: {remote_url}",
                f"remote.origin.promisor: {promisor}",
                f"remote.origin.partialclonefilter: {partial_filter}",
                f"core.sshCommand: {core_ssh_command}",
            ]

        lines += [
            f"GIT_SSH_COMMAND: {_env_value('GIT_SSH_COMMAND')}",
            f"SSH_AUTH_SOCK: {_env_value('SSH_AUTH_SOCK')}",
        ]

        lines.append(
            self._probe(
                "ssh -T -o BatchMode=yes git@github.com",
                ["ssh", "-T", "-o", "BatchMode=yes", "git@github.com"],
                cwd=None,
                success_text="successfully authenticated",
            )
        )

        return lines

    def _run(self, *args: str, check: bool = True) -> subprocess.CompletedProcess[str]:
        result = subprocess.run(
            ["git", *args],
            cwd=self.root,
            check=False,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        if check and result.returncode != 0:
            command = " ".join(["git", *args])
            message = result.stderr.strip() or result.stdout.strip()
            raise GitError(f"{command} failed: {message}")
        return result

    def _config_value(self, key: str) -> str | None:
        result = self._run("config", "--get", key, check=False)
        if result.returncode != 0:
            return None
        value = result.stdout.strip()
        return value or None

    def _probe(
        self,
        label: str,
        command: list[str],
        *,
        cwd: Path | None,
        success_text: str | None = None,
    ) -> str:
        try:
            result = subprocess.run(
                command,
                cwd=cwd,
                check=False,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=10,
            )
        except FileNotFoundError:
            return f"{label}: command not found"
        except subprocess.TimeoutExpired:
            return f"{label}: timed out after 10 seconds"

        output = _single_line(result.stderr.strip() or result.stdout.strip())
        if result.returncode == 0 or (success_text and success_text in output):
            suffix = f" ({output})" if output else ""
            return f"{label}: ok{suffix}"
        suffix = f": {output}" if output else ""
        return f"{label}: failed with exit code {result.returncode}{suffix}"


def _single_line(value: str) -> str:
    return " | ".join(line.strip() for line in value.splitlines() if line.strip())


def _env_value(name: str) -> str:
    return os.environ.get(name) or "(not set)"
