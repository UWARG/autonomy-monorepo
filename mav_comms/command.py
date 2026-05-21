from abc import ABC, abstractmethod

from .connection import MavConnection


class MavCommand(ABC):
    def __init__(self, connection: MavConnection) -> None:
        self.connection = connection

    @abstractmethod
    def execute(self) -> bool:
        """Execute the command, returns True on success."""
        pass

class ChangeAltitudeCommand(MavCommand):
    def __init__(self, connection: MavConnection, altitude: float) -> None:
        super().__init__(connection)
        self.altitude = altitude

    def execute(self) -> bool:
        """Change the drone's target altitude."""


class ChangeYawCommand(MavCommand):
    def __init__(self, connection: MavConnection, yaw: float) -> None:
        super().__init__(connection)
        self.yaw = yaw

    def execute(self) -> bool:
        """Change the drone's yaw."""
