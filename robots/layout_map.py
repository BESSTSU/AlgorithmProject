"""Draw an occupancy map for nav2 (PGM + YAML) straight from the grid map file.

Used instead of a SLAM map: the arena is built exactly to its layout, so the walls
are known. The grid's bottom-left corner lands on the map frame origin, which makes
`ros2.grid_origin` all zeros.

    python -m robots.layout_map            # writes ros2.layout_map.out + .pgm/.yaml

Walls are drawn on every edge between a passable cell and a non-passable one (or
the outside), and on every thin wall listed in the map file. Passable cells are
free, everything else is unknown.
"""
import argparse
from pathlib import Path

import numpy as np
import yaml

from core.grid_map import GridMap

# Pixel values understood by nav2 map_server in trinary mode.
FREE_PX, OCCUPIED_PX, UNKNOWN_PX = 254, 0, 205

_SIDES = {(-1, 0): "top", (1, 0): "bottom", (0, -1): "left", (0, 1): "right"}


def wall_segments(grid: GridMap, cell_size: float) -> list[tuple[tuple[float, float], tuple[float, float]]]:
    """Wall segments in meters, grid bottom-left corner at (0, 0), y up (row 0 on top)."""
    blocked = {frozenset(edge) for edge in grid.edge_walls}
    segments = set()
    for r in range(grid.height):
        for c in range(grid.width):
            if not grid.is_passable((r, c)):
                continue
            x0, x1 = c * cell_size, (c + 1) * cell_size
            y0, y1 = (grid.height - r - 1) * cell_size, (grid.height - r) * cell_size
            for (dr, dc), side in _SIDES.items():
                n = (r + dr, c + dc)
                if grid.is_passable(n) and frozenset(((r, c), n)) not in blocked:
                    continue
                segments.add({"top": ((x0, y1), (x1, y1)), "bottom": ((x0, y0), (x1, y0)),
                              "left": ((x0, y0), (x0, y1)), "right": ((x1, y0), (x1, y1))}[side])
    return sorted(segments)


def render(grid: GridMap, cell_size: float, resolution: float, wall_thickness: float,
           margin: float) -> tuple[np.ndarray, list[float]]:
    """Image (row 0 = top, as in a PGM file) and the map origin [x, y, yaw] of its bottom-left pixel."""
    width = round((grid.width * cell_size + 2 * margin) / resolution)
    height = round((grid.height * cell_size + 2 * margin) / resolution)
    img = np.full((height, width), UNKNOWN_PX, dtype=np.uint8)

    def px_range(lo: float, hi: float) -> slice:
        a = int(np.floor((lo + margin) / resolution + 1e-9))
        b = int(np.ceil((hi + margin) / resolution - 1e-9))
        return slice(a, max(b, a + 1))

    def fill(x0, x1, y0, y1, value):
        xs, ys = px_range(x0, x1), px_range(y0, y1)
        img[height - ys.stop:height - ys.start, xs] = value  # flip y: image row 0 is the top

    for r in range(grid.height):
        for c in range(grid.width):
            if grid.is_passable((r, c)):
                y0 = (grid.height - r - 1) * cell_size
                fill(c * cell_size, (c + 1) * cell_size, y0, y0 + cell_size, FREE_PX)
    h = wall_thickness / 2
    for (xa, ya), (xb, yb) in wall_segments(grid, cell_size):
        # Extend each segment by half a thickness so corners close.
        fill(min(xa, xb) - h, max(xa, xb) + h, min(ya, yb) - h, max(ya, yb) + h, OCCUPIED_PX)
    return img, [-margin, -margin, 0.0]


def write(out: Path, img: np.ndarray, resolution: float, origin: list[float]) -> None:
    out.parent.mkdir(parents=True, exist_ok=True)
    pgm = out.with_suffix(".pgm")
    with open(pgm, "wb") as f:
        f.write(f"P5\n{img.shape[1]} {img.shape[0]}\n255\n".encode())
        f.write(img.tobytes())
    meta = {"image": pgm.name, "mode": "trinary", "resolution": resolution, "origin": origin,
            "negate": 0, "occupied_thresh": 0.65, "free_thresh": 0.25}
    out.with_suffix(".yaml").write_text(yaml.safe_dump(meta, sort_keys=False), encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description="Draw a nav2 map from the grid map file.")
    parser.add_argument("--config", default="config.yaml")
    args = parser.parse_args()

    config_path = Path(args.config)
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    cfg = config["ros2"]["layout_map"]
    grid = GridMap.from_file(config_path.parent / config["map"])
    img, origin = render(grid, config["cell_size"], cfg["resolution"], cfg["wall_thickness"], cfg["margin"])
    out = config_path.parent / cfg["out"]
    write(out, img, cfg["resolution"], origin)
    print(f"wrote {out.with_suffix('.pgm')} ({img.shape[1]}x{img.shape[0]} px) and {out.with_suffix('.yaml')}")
    print(f"map {config['map']}, cell size {config['cell_size']} m; grid_origin is (0, 0, 0) in this map")


if __name__ == "__main__":
    main()
