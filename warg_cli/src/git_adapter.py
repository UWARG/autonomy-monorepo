from __future__ import annotations

import subprocess
from pathlib import Path

from errors import GitError


class GitAdapter:
    def __init__(self, root: Path):
        self.root = root

    @classmethod
    def clone_sparse(cls, repository: str, destination: str | None = None) -> None:
        command = ["git", "clone", "--filter=blob:none", "--sparse"]
        command.append(repository)
        if destination:
            command.append(destination)

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

    def _run(
        self, *args: str, check: bool = True
    ) -> subprocess.CompletedProcess[str]:
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
