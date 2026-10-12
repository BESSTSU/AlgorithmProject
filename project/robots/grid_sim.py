"""Headless grid simulator with discrete ticks and optional random delays.

A move normally takes one tick. With probability `delay_prob` it takes
1..max_extra_steps ticks longer. While a robot moves it occupies both its source
and its target cell, and any cell occupied by two robots counts as a collision.
That is stricter than the planner's conflict rules: following directly behind
another robot into the cell it is still leaving is a collision here.
"""
import random

from core.grid_map import Cell, GridMap
from robots.base import RobotInterface


class GridSimRobot(RobotInterface):
    def __init__(self, sim: "GridSim", cell: Cell):
        self._sim = sim
        self._cell = cell
        self._target = None  # cell being moved to, if any
        self._remaining = 0  # ticks until the move completes

    def get_cell(self) -> Cell:
        return self._cell

    def go_to_cell(self, cell: Cell) -> None:
        r, c = self._cell
        if abs(cell[0] - r) + abs(cell[1] - c) != 1 or not self._sim.grid.is_passable(cell):
            raise ValueError(f"cannot move from {self._cell} to {cell}")
        self._target = cell
        self._remaining = 1 + self._sim.extra_delay()

    def is_arrived(self) -> bool:
        return self._target is None

    def stop(self) -> None:
        self._target = None
        self._remaining = 0

    def occupied(self) -> set[Cell]:
        return {self._cell} if self._target is None else {self._cell, self._target}

    def advance(self) -> None:
        if self._target is None:
            return
        self._remaining -= 1
        if self._remaining == 0:
            self._cell, self._target = self._target, None


class GridSim:
    def __init__(self, grid: GridMap, starts: dict[str, Cell], delay_prob: float,
                 max_extra_steps: int, rng: random.Random):
        self.grid = grid
        self.robots = {name: GridSimRobot(self, cell) for name, cell in starts.items()}
        self.collisions = 0  # (cell, tick) pairs in which two robots overlapped
        self._delay_prob = delay_prob
        self._max_extra_steps = max_extra_steps
        self._rng = rng

    def extra_delay(self) -> int:
        if self._rng.random() < self._delay_prob:
            return self._rng.randint(1, self._max_extra_steps)
        return 0

    def step(self) -> None:
        """Check for collisions during this tick, then advance every move by one tick."""
        occupied = set()
        for robot in self.robots.values():
            cells = robot.occupied()
            self.collisions += len(cells & occupied)
            occupied |= cells
        for robot in self.robots.values():
            robot.advance()
