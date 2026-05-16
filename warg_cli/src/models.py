from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path


@dataclass(frozen=True)
class ProjectEntry:
    name: str
    path: str


@dataclass(frozen=True)
class Project:
    name: str
    path: Path
    description: str = ""
    depends_on: tuple[str, ...] = field(default_factory=tuple)
    commands: dict[str, str] = field(default_factory=dict)
    ci: dict[str, tuple[str, ...]] = field(default_factory=dict)

    @property
    def relative_path(self) -> str:
        return self.path.name
