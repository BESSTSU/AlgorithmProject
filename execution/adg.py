"""Action Dependency Graph: the order of moves in a plan, without its clock.

Each robot's time-indexed path is compressed into visits (a cell and the planned
time span spent on it). Move k takes the robot from visit k to visit k + 1; waits
disappear, since they only existed to enforce an order. A move into a cell
depends on the move by which the previous robot planned on that cell leaves it,
and that move has to be completed first. So robots pass every cell in the
planned order, however fast or slow each of them is.
"""
from collections import defaultdict
from dataclasses import dataclass

from core.grid_map import Cell


@dataclass(frozen=True)
class Visit:
    cell: Cell
    t_in: int   # planned arrival time
    t_out: int  # planned time of the last step spent on the cell


def _visits(path: list[Cell]) -> list[Visit]:
    visits = []
    t_in = 0
    for t in range(1, len(path) + 1):
        if t == len(path) or path[t] != path[t_in]:
            visits.append(Visit(path[t_in], t_in, t - 1))
            t_in = t
    return visits


class ActionDependencyGraph:
    def __init__(self, paths: dict[str, list[Cell]]):
        self.visits = {name: _visits(path) for name, path in paths.items()}
        # (robot, move index) -> moves of other robots that must be completed first
        self._deps: dict[tuple[str, int], list[tuple[str, int]]] = defaultdict(list)

        by_cell = defaultdict(list)
        for name, visits in self.visits.items():
            for k, visit in enumerate(visits):
                by_cell[visit.cell].append((visit.t_in, name, k))
        for cell, entries in by_cell.items():
            entries.sort()
            for (_, prev_name, prev_k), (_, name, k) in zip(entries, entries[1:]):
                if prev_name == name:
                    continue
                if prev_k == self.n_moves(prev_name):
                    raise ValueError(f"plan sends {name} onto {cell} after {prev_name} parked there")
                # Move k - 1 enters visit k; move prev_k leaves visit prev_k.
                self._deps[(name, k - 1)].append((prev_name, prev_k))

    def n_moves(self, name: str) -> int:
        return len(self.visits[name]) - 1

    def dependencies(self, name: str, k: int) -> list[tuple[str, int]]:
        """Moves (robot, index) of other robots that must be completed before move k starts."""
        return self._deps.get((name, k), [])

    def can_start(self, name: str, k: int, completed: dict[str, int]) -> bool:
        """True if move k of `name` may start, given how many moves each robot has completed."""
        return all(completed[other] > index for other, index in self.dependencies(name, k))
