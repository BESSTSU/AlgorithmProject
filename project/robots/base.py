"""Interface every robot backend implements. Commands are cells, not wheel speeds."""
from abc import ABC, abstractmethod

from core.grid_map import Cell


class RobotInterface(ABC):
    # The battery is simulated in Python for every backend (SPEC 10.2). The
    # executor stores the current level here after each tick.
    battery: int | None = None

    @abstractmethod
    def get_cell(self) -> Cell:
        """Cell the robot is currently in."""

    @abstractmethod
    def go_to_cell(self, cell: Cell) -> None:
        """Start moving to an adjacent cell. Does not block."""

    @abstractmethod
    def is_arrived(self) -> bool:
        """True once the last go_to_cell target has been reached."""

    def get_battery(self) -> int | None:
        """Current battery level, as last set by the executor."""
        return self.battery

    @abstractmethod
    def stop(self) -> None:
        """Stop moving immediately."""
