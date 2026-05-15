from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path


@dataclass(frozen=True)
class Project:
    name: str
    path: Path
    language: str | None = None
    description: str = ""
    depends_on: tuple[str, ...] = field(default_factory=tuple)
    commands: dict[str, str] = field(default_factory=dict)

    @property
    def relative_path(self) -> str:
        return self.path.name

