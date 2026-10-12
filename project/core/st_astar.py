"""Space-time A* for a single robot with battery and an ordered task list.

The search state is (cell, t, battery, k), where k is the index of the next
task. Because k is part of the state, the planner decides when and how long to
charge while looking at the whole task sequence.
"""
import heapq
import itertools
from dataclasses import replace

from core.battery import BatteryModel
from core.grid_map import WALL, Cell, GridMap
from core.reservation import ReservationTable

FAIL_BATTERY = "battery"  # a path exists within t_max, but not with the available energy
FAIL_HORIZON = "horizon"  # a path exists, but it needs more time than t_max
FAIL_NO_PATH = "no_path"  # the way is blocked for good


def manhattan(a: Cell, b: Cell) -> int:
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def plan_path(
    grid: GridMap,
    start: Cell,
    tasks: list[Cell],
    battery_start: int,
    battery: BatteryModel,
    t_max: int,
    reservations: ReservationTable | None = None,
    allow_charging: bool = True,
    t_start: int = 0,
    park: bool = True,
) -> list[Cell] | None:
    """Fastest path that visits `tasks` in order, as a list of cells per time step.

    The robot is on `start` at time `t_start`, so path[i] is its cell at time
    t_start + i. The path ends on the last task cell with at least
    `battery.min_final` left. With `park` it ends only once no other robot
    reserves that cell afterwards, so the robot can stay there; without it the
    path ends on arrival (a segment that another one continues). The battery
    never drops below `battery.reserve`. Returns None if no such path exists
    within t_max.
    """
    if not tasks:
        raise ValueError("a robot needs at least one task")
    for name, cell in [("start", start)] + [(f"task {i}", c) for i, c in enumerate(tasks)]:
        if not grid.is_passable(cell):
            raise ValueError(f"{name} {cell} is not a passable cell")
    if reservations is None:
        reservations = ReservationTable()

    n = len(tasks)
    final = tasks[-1]
    goal_free_after = reservations.last_reserved(final)
    # remaining[k]: distance still to travel after reaching tasks[k].
    remaining = [0] * n
    for i in range(n - 2, -1, -1):
        remaining[i] = remaining[i + 1] + manhattan(tasks[i], tasks[i + 1])

    def heuristic(cell, k):
        if k == n:
            return manhattan(cell, final)
        return manhattan(cell, tasks[k]) + remaining[k]

    def advance(cell, k):
        while k < n and cell == tasks[k]:
            k += 1
        return k

    start_state = (start, t_start, battery_start, advance(start, 0))
    h0 = heuristic(start, start_state[3])
    tie = itertools.count()  # keeps heap ordering deterministic
    open_heap = [(t_start + h0, h0, next(tie), start_state)]
    # g == t for every state, so the first time a state is generated its g is
    # already optimal and it never needs updating. States are keyed by the full
    # (cell, t, battery, k), never (row, col) alone, so waits stay possible.
    came_from = {start_state: None}
    # Dominance pruning: at the same (cell, t, k), a state with more battery can
    # do everything a state with less can, so only the highest level is expanded.
    best_level = {(start, t_start, start_state[3]): battery_start}

    while open_heap:
        _, _, _, state = heapq.heappop(open_heap)
        cell, t, level, k = state
        if level < best_level[(cell, t, k)]:
            continue  # a better state for this (cell, t, k) was found after this one was queued
        if (k == n and cell == final and level >= battery.min_final
                and (not park or t > goal_free_after)):
            return _reconstruct(came_from, state)
        if t >= t_max:
            continue
        for nxt in [cell] + grid.neighbors(cell):  # wait (or charge) + 4-connected moves
            if not reservations.is_free(nxt, t + 1):
                continue
            new_level = battery.next_level(grid, level, cell, nxt, allow_charging)
            if new_level < battery.reserve:
                continue
            new_k = advance(nxt, k)
            if best_level.get((nxt, t + 1, new_k), -1) >= new_level:
                continue
            best_level[(nxt, t + 1, new_k)] = new_level
            child = (nxt, t + 1, new_level, new_k)
            came_from[child] = state
            h = heuristic(nxt, new_k)
            heapq.heappush(open_heap, (t + 1 + h, h, next(tie), child))
    return None


def failure_reason(
    grid: GridMap,
    start: Cell,
    tasks: list[Cell],
    battery: BatteryModel,
    t_max: int,
    reservations: ReservationTable | None = None,
    t_start: int = 0,
    park: bool = True,
) -> str:
    """Why plan_path found nothing for this query: FAIL_BATTERY, FAIL_HORIZON or FAIL_NO_PATH.

    Repeats the search with energy ignored. If that finds a path within t_max,
    only the battery stood in the way. Otherwise it searches once more with a
    horizon that outlasts every moving robot: a path then means t_max was too
    short, no path means the way is blocked for good.
    """
    unlimited = replace(battery, move_cost=0, wait_cost=0, min_final=0, reserve=0)

    def reachable(horizon):
        return plan_path(grid, start, tasks, 0, unlimited, horizon, reservations,
                         allow_charging=False, t_start=t_start, park=park) is not None

    if reachable(t_max):
        return FAIL_BATTERY
    # Once every moving robot has parked the world is static, and each task is
    # then at most one step per passable cell away.
    passable = sum(ch != WALL for row in grid.rows for ch in row)
    settled = max(t_start, reservations.last_time() if reservations else 0)
    long_horizon = settled + 1 + passable * len(tasks)
    if long_horizon > t_max and reachable(long_horizon):
        return FAIL_HORIZON
    return FAIL_NO_PATH


def _reconstruct(came_from, state) -> list[Cell]:
    path = []
    while state is not None:
        path.append(state[0])
        state = came_from[state]
    return path[::-1]
