from __future__ import annotations

import json
import subprocess
from dataclasses import dataclass

from errors import WargError


class GitHubError(WargError):
    """Raised when GitHub repository discovery cannot be completed."""


@dataclass(frozen=True)
class GitHubRepository:
    name: str
    ssh_url: str
    url: str


class GitHubAdapter:
    @classmethod
    def list_org_repositories(
        cls, organization: str, include_archived: bool = False
    ) -> list[GitHubRepository]:
        result = subprocess.run(
            [
                "gh",
                "repo",
                "list",
                organization,
                "--limit",
                "1000",
                "--json",
                "name,sshUrl,url,isArchived",
            ],
            check=False,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        if result.returncode != 0:
            message = result.stderr.strip() or result.stdout.strip()
            raise GitHubError(f"gh repo list {organization} failed: {message}")

        try:
            records = json.loads(result.stdout)
        except json.JSONDecodeError as error:
            raise GitHubError("gh returned invalid repository data.") from error

        repositories = []
        for record in records:
            if record.get("isArchived") and not include_archived:
                continue
            name = record.get("name")
            ssh_url = record.get("sshUrl")
            url = record.get("url")
            if not name or not ssh_url or not url:
                continue
            repositories.append(GitHubRepository(name=name, ssh_url=ssh_url, url=url))
        return sorted(repositories, key=lambda repository: repository.name.lower())
