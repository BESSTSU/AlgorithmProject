from core.grid_map import CHARGER
from core.st_astar import manhattan, plan_path


def plan_single(grid, start, goal, battery, t_max, reservations=None):
    """Plan one goal from a full battery, for tests that are not about the battery."""
    return plan_path(grid, start, [goal], battery.max, battery, t_max, reservations)


def assert_valid_path(grid, path, start, goal):
    assert path[0] == start
    assert path[-1] == goal
    for cell in path:
        assert grid.is_passable(cell), f"path enters blocked cell {cell}"
    for a, b in zip(path, path[1:]):
        assert manhattan(a, b) <= 1, f"illegal jump {a} -> {b}"


def charging_times(grid, path):
    """Time steps t at which the robot charges (waits on a charger cell from t to t + 1)."""
    return [
        t for t, (a, b) in enumerate(zip(path, path[1:]))
        if a == b and grid.symbol(a) == CHARGER
    ]


def find_conflicts(paths):
    """Vertex and swap conflicts between paths. A robot stays on its last cell after its path ends."""
    def at(path, t):
        return path[min(t, len(path) - 1)]

    names = list(paths)
    horizon = max(len(p) for p in paths.values())
    conflicts = []
    for t in range(horizon):
        for i, a in enumerate(names):
            for b in names[i + 1:]:
                pa, pb = paths[a], paths[b]
                if at(pa, t) == at(pb, t):
                    conflicts.append(f"vertex {a}/{b} at {at(pa, t)} t={t}")
                elif at(pa, t) == at(pb, t + 1) and at(pb, t) == at(pa, t + 1):
                    conflicts.append(f"swap {a}/{b} between t={t} and t={t + 1}")
    return conflicts
