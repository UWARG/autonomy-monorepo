from __future__ import annotations

import subprocess

from errors import CommandError
from models import Project


class CommandRunner:
    def run(self, project: Project, command_name: str, passthrough: list[str]) -> int:
        try:
            command = project.commands[command_name]
        except KeyError as error:
            available = ", ".join(project.commands) or "none"
            raise CommandError(
                f"Project '{project.name}' has no command '{command_name}'. "
                f"Available commands: {available}."
            ) from error

        if passthrough:
            command = " ".join([command, *(_shell_quote(arg) for arg in passthrough)])

        result = subprocess.run(command, cwd=project.path, shell=True)
        return result.returncode


def _shell_quote(value: str) -> str:
    if not value:
        return "''"
    safe = set(
        "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_+-=.,:/@%"
    )
    if all(char in safe for char in value):
        return value
    return "'" + value.replace("'", "'\"'\"'") + "'"
