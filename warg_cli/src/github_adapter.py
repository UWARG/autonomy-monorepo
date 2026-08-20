from __future__ import annotations

import json
import re
import subprocess
import urllib.error
import urllib.request
from dataclasses import dataclass

from errors import WargError


class GitHubError(WargError):
    """Raised when GitHub repository discovery cannot be completed."""


@dataclass(frozen=True)
class GitHubRepository:
    name: str
    ssh_url: str
    url: str
    updated_at: str = ""


class GitHubAdapter:
    @classmethod
    def list_org_repositories(
        cls, organization: str, include_archived: bool = False
    ) -> list[GitHubRepository]:
        try:
            return cls._list_org_repositories_with_gh(organization, include_archived)
        except GitHubError as gh_error:
            try:
                return cls._list_org_repositories_with_api(
                    organization, include_archived
                )
            except GitHubError as api_error:
                raise GitHubError(
                    "Could not list GitHub repositories with the gh CLI or "
                    f"GitHub API. gh error: {gh_error}. API error: {api_error}"
                ) from api_error

    @classmethod
    def _list_org_repositories_with_gh(
        cls, organization: str, include_archived: bool
    ) -> list[GitHubRepository]:
        result = cls._run_gh(
            [
                "gh",
                "repo",
                "list",
                organization,
                "--limit",
                "1000",
                "--json",
                "name,sshUrl,url,isArchived,updatedAt",
            ]
        )

        return cls._repositories_from_json(
            result.stdout,
            include_archived=include_archived,
            field_names={
                "ssh_url": "sshUrl",
                "url": "url",
                "archived": "isArchived",
                "updated_at": "updatedAt",
            },
            invalid_message="gh returned invalid repository data.",
        )

    @classmethod
    def _list_org_repositories_with_api(
        cls, organization: str, include_archived: bool
    ) -> list[GitHubRepository]:
        repositories: list[GitHubRepository] = []
        page = 1
        while True:
            url = (
                f"https://api.github.com/orgs/{organization}/repos"
                f"?per_page=100&page={page}"
            )
            request = urllib.request.Request(
                url,
                headers={
                    "Accept": "application/vnd.github+json",
                    "User-Agent": "warg-cli",
                },
            )
            try:
                with urllib.request.urlopen(request, timeout=10) as response:
                    body = response.read().decode("utf-8")
            except urllib.error.URLError as error:
                raise GitHubError(f"GitHub API request failed: {error}") from error

            try:
                records = json.loads(body)
            except json.JSONDecodeError as error:
                raise GitHubError(
                    "GitHub API returned invalid repository data."
                ) from error
            if not isinstance(records, list):
                raise GitHubError("GitHub API returned invalid repository data.")

            page_repositories = cls._repositories_from_records(
                records,
                include_archived=include_archived,
                field_names={
                    "ssh_url": "ssh_url",
                    "url": "html_url",
                    "archived": "archived",
                    "updated_at": "updated_at",
                },
            )
            if not records:
                break
            repositories.extend(page_repositories)
            if len(records) < 100:
                break
            page += 1

        return cls._sort_repositories(repositories)

    @classmethod
    def _repositories_from_json(
        cls,
        payload: str,
        *,
        include_archived: bool,
        field_names: dict[str, str],
        invalid_message: str,
    ) -> list[GitHubRepository]:
        try:
            records = json.loads(payload)
        except json.JSONDecodeError as error:
            raise GitHubError(invalid_message) from error
        if not isinstance(records, list):
            raise GitHubError(invalid_message)

        return cls._repositories_from_records(
            records,
            include_archived=include_archived,
            field_names=field_names,
        )

    @classmethod
    def _repositories_from_records(
        cls,
        records: list[object],
        *,
        include_archived: bool,
        field_names: dict[str, str],
    ) -> list[GitHubRepository]:
        repositories = []
        for record in records:
            if not isinstance(record, dict):
                continue
            if record.get(field_names["archived"]) and not include_archived:
                continue
            name = record.get("name")
            ssh_url = record.get(field_names["ssh_url"])
            url = record.get(field_names["url"])
            updated_at = record.get(field_names["updated_at"]) or ""
            if not name or not ssh_url or not url:
                continue
            repositories.append(
                GitHubRepository(
                    name=name,
                    ssh_url=ssh_url,
                    url=url,
                    updated_at=updated_at,
                )
            )
        return cls._sort_repositories(repositories)

    @classmethod
    def _sort_repositories(
        cls, repositories: list[GitHubRepository]
    ) -> list[GitHubRepository]:
        return sorted(
            sorted(repositories, key=lambda repository: repository.name.lower()),
            key=lambda repository: repository.updated_at,
            reverse=True,
        )

    @classmethod
    def current_user(cls) -> str:
        result = cls._run_gh(["gh", "api", "user", "--jq", ".login"])
        login = result.stdout.strip()
        if not login:
            raise GitHubError("gh did not report an authenticated GitHub user.")
        return login

    @classmethod
    def fork_repository(cls, upstream: str) -> GitHubRepository:
        login = cls.current_user()
        output = cls._run_gh(
            ["gh", "repo", "fork", upstream, "--clone=false"],
            merge_stderr=True,
        ).stdout
        target = cls._fork_slug_from_output(output, upstream, login)
        return cls._resolve_fork(target, upstream)

    @classmethod
    def _fork_slug_from_output(cls, output: str, upstream: str, login: str) -> str:
        upstream_name = upstream.rsplit("/", 1)[-1]
        for match in re.finditer(r"\b([\w.-]+/[\w.-]+)\b", output):
            slug = match.group(1)
            if slug.lower() != upstream.lower() and slug.startswith(f"{login}/"):
                return slug
        return f"{login}/{upstream_name}"

    @classmethod
    def _resolve_fork(cls, target: str, upstream: str) -> GitHubRepository:
        result = cls._run_gh(
            ["gh", "repo", "view", target, "--json", "name,sshUrl,url,parent"]
        )
        try:
            record = json.loads(result.stdout)
        except json.JSONDecodeError as error:
            raise GitHubError(f"gh returned invalid data for {target}.") from error
        if not isinstance(record, dict):
            raise GitHubError(f"gh returned invalid data for {target}.")

        parent = record.get("parent") or {}
        parent_slug = ""
        if isinstance(parent, dict):
            owner = parent.get("owner") or {}
            owner_login = owner.get("login") if isinstance(owner, dict) else None
            if owner_login and parent.get("name"):
                parent_slug = f"{owner_login}/{parent['name']}"
        if parent_slug.lower() != upstream.lower():
            raise GitHubError(
                f"{target} exists but is not a fork of {upstream}. Rename or delete "
                "it, then try again."
            )

        name = record.get("name")
        ssh_url = record.get("sshUrl")
        url = record.get("url")
        if not name or not ssh_url or not url:
            raise GitHubError(f"gh returned incomplete data for {target}.")
        return GitHubRepository(name=name, ssh_url=ssh_url, url=url)

    @classmethod
    def _run_gh(
        cls, command: list[str], *, merge_stderr: bool = False
    ) -> subprocess.CompletedProcess[str]:
        try:
            result = subprocess.run(
                command,
                check=False,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT if merge_stderr else subprocess.PIPE,
            )
        except FileNotFoundError as error:
            raise GitHubError("gh is not installed.") from error
        if result.returncode != 0:
            message = (result.stderr or "").strip() or result.stdout.strip()
            raise GitHubError(f"{' '.join(command)} failed: {message}")
        return result
