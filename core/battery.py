"""Battery model: the single place that defines how one time step changes the battery."""
from dataclasses import dataclass

from core.grid_map import CHARGER, Cell, GridMap


@dataclass(frozen=True)
class BatteryModel:
    max: int
    move_cost: int
    wait_cost: int
    charge_rate: int
    min_final: int  # minimum level required when the last task is done
    reserve: int    # minimum level the planner keeps at every step (margin for delays)

    def next_level(self, grid: GridMap, level: int, a: Cell, b: Cell,
                   allow_charging: bool = True) -> int:
        """Battery level after one step from cell a to cell b (a == b means wait).

        Waiting on a charger cell charges instead of costing energy. A negative
        result means the step is infeasible.
        """
        if a != b:
            return level - self.move_cost
        if allow_charging and grid.symbol(a) == CHARGER:
            return min(level + self.charge_rate, self.max)
        return level - self.wait_cost

    def trace(self, grid: GridMap, path: list[Cell], start_level: int,
              allow_charging: bool = True) -> list[int]:
        """Battery level at every time step of the path (trace[t])."""
        levels = [start_level]
        for a, b in zip(path, path[1:]):
            levels.append(self.next_level(grid, levels[-1], a, b, allow_charging))
        return levels
