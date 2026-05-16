class WargError(Exception):
    """Base class for user-facing CLI errors."""


class ManifestError(WargError):
    """Raised when project manifests are missing or invalid."""


class DependencyError(WargError):
    """Raised when project dependencies cannot be resolved."""


class CommandError(WargError):
    """Raised when a manifest command cannot be run."""


class GitError(WargError):
    """Raised when a Git operation cannot be completed."""
