"""Where the grid lies in a metric frame: the CoppeliaSim world or the SLAM map."""
import math
from dataclasses import dataclass

from core.grid_map import Cell


@dataclass(frozen=True)
class GridFrame:
    """Converts between grid cells and metric (x, y) positions.

    (x, y) is the position of the grid's bottom-left corner in the metric frame
    and yaw is the direction in which columns increase. Rows increase the other
    way along the second axis, so row 0 is the top row.
    """
    height: int        # number of grid rows
    cell_size: float   # meters
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0   # radians

    def to_metric(self, cell: Cell) -> tuple[float, float]:
        """Metric position of the center of a cell."""
        row, col = cell
        along = (col + 0.5) * self.cell_size
        across = (self.height - row - 0.5) * self.cell_size
        cos, sin = math.cos(self.yaw), math.sin(self.yaw)
        return self.x + along * cos - across * sin, self.y + along * sin + across * cos

    def to_cell(self, position: tuple[float, float]) -> Cell:
        """Cell that contains a metric position."""
        dx, dy = position[0] - self.x, position[1] - self.y
        cos, sin = math.cos(self.yaw), math.sin(self.yaw)
        along, across = dx * cos + dy * sin, -dx * sin + dy * cos
        return (self.height - 1 - math.floor(across / self.cell_size),
                math.floor(along / self.cell_size))

    def to_grid(self, position: tuple[float, float]) -> tuple[float, float]:
        """Fractional (row, col) of a metric position: cell centers are whole numbers."""
        dx, dy = position[0] - self.x, position[1] - self.y
        cos, sin = math.cos(self.yaw), math.sin(self.yaw)
        along, across = dx * cos + dy * sin, -dx * sin + dy * cos
        return self.height - 0.5 - across / self.cell_size, along / self.cell_size - 0.5


def fit_grid_frame(height: int, cell_size: float,
                   samples: list[tuple[Cell, tuple[float, float]]]) -> tuple[GridFrame, list[float]]:
    """Find where the grid lies in a metric frame from measured cell centers.

    Each sample pairs a cell with the measured (x, y) of a robot standing on that
    cell's center, for example its pose in the SLAM map. At least two different
    cells are needed. Returns the best-fitting GridFrame (least squares, rotation
    and translation only) and each sample's distance from where the fit puts it.
    Large distances mean a misplaced robot or a cell size that does not match.
    """
    if len({cell for cell, _ in samples}) < 2:
        raise ValueError("at least two different cells are needed")
    flat = GridFrame(height, cell_size)
    grid_points = [flat.to_metric(cell) for cell, _ in samples]
    measured = [xy for _, xy in samples]
    n = len(samples)
    gx, gy = sum(p[0] for p in grid_points) / n, sum(p[1] for p in grid_points) / n
    mx, my = sum(p[0] for p in measured) / n, sum(p[1] for p in measured) / n
    cross = dot = 0.0
    for (ax, ay), (bx, by) in zip(grid_points, measured):
        ax, ay, bx, by = ax - gx, ay - gy, bx - mx, by - my
        cross += ax * by - ay * bx
        dot += ax * bx + ay * by
    yaw = math.atan2(cross, dot)
    cos, sin = math.cos(yaw), math.sin(yaw)
    frame = GridFrame(height, cell_size, mx - (gx * cos - gy * sin), my - (gx * sin + gy * cos), yaw)
    errors = [math.dist(frame.to_metric(cell), xy) for cell, xy in samples]
    return frame, errors
