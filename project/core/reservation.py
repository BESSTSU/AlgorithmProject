"""Space-time reservation table for paths that have already been planned.

A reserved cell keeps one time step of clearance on both sides: no other robot
may enter a cell in the step its occupant leaves it, or leave it in the step the
next occupant arrives. This is the rule the executor enforces (a robot enters a
cell only once the previous one is really out of it), so a plan made with this
table runs exactly as planned when nothing is delayed. The clearance also rules
out two robots swapping cells, so no separate edge reservation is needed.
"""
import math

from core.grid_map import Cell


class ReservationTable:
    def __init__(self):
        self._blocked: set[tuple[Cell, int]] = set()  # (cell, t) closed to other robots
        self._last_time: dict[Cell, int] = {}         # cell -> last blocked time, parking aside
        # cell -> time from which a finished robot occupies it for good.
        self._parked: dict[Cell, int] = {}
        # Start cells of robots that have not been planned yet.
        self._starts: set[Cell] = set()

    def reserve_path(self, path: list[Cell]) -> None:
        """Reserve every (cell, t) of the path with its clearance, and the final cell for good."""
        for t, cell in enumerate(path):
            for blocked_t in (t - 1, t, t + 1):
                self._blocked.add((cell, blocked_t))
            self._last_time[cell] = max(self._last_time.get(cell, -1), t + 1)
        self._parked[path[-1]] = len(path) - 1

    def reserve_start(self, cell: Cell) -> None:
        """Keep others off a start cell at t = 0 and 1: its robot cannot be out of it sooner."""
        self._starts.add(cell)

    def release_start(self, cell: Cell) -> None:
        self._starts.discard(cell)

    def is_free(self, cell: Cell, t: int) -> bool:
        if (cell, t) in self._blocked:
            return False
        if t <= 1 and cell in self._starts:
            return False
        parked_from = self._parked.get(cell)
        return parked_from is None or t < parked_from

    def last_reserved(self, cell: Cell) -> float:
        """Last time `cell` is closed to others: -1 if never, inf if a robot parks there."""
        if cell in self._parked:
            return math.inf
        last = self._last_time.get(cell, -1)
        return max(last, 1) if cell in self._starts else last

    def last_time(self) -> int:
        """Last time any moving robot holds a reservation (parked robots aside)."""
        return max(self._last_time.values(), default=0)
