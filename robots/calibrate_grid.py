"""Find `ros2.grid_origin`: where the planner's grid lies in the SLAM map.

Stand a robot on the center of a few known cells and note its (x, y) in the map
frame each time, for example from
    ros2 run tf2_ros tf2_echo map base_footprint
Then pass every measurement as row,col:x,y

    python -m robots.calibrate_grid 2,0:0.02,-0.01 2,5:1.76,0.03 0,0:0.01,0.71

Two cells are the minimum. Cells far apart, and not all on one line, give the
best fit. The result is printed ready to paste into config.yaml.
"""
import argparse
import math
from pathlib import Path

import yaml

from core.grid_map import Cell, GridMap
from robots.grid_frame import fit_grid_frame


def parse_sample(text: str) -> tuple[Cell, tuple[float, float]]:
    """'row,col:x,y' -> ((row, col), (x, y))"""
    try:
        cell_text, xy_text = text.split(":")
        row, col = (int(v) for v in cell_text.split(","))
        x, y = (float(v) for v in xy_text.split(","))
    except ValueError:
        raise argparse.ArgumentTypeError(f"expected row,col:x,y but got {text!r}") from None
    return (row, col), (x, y)


def main():
    parser = argparse.ArgumentParser(description="Fit the grid's position in the SLAM map.")
    parser.add_argument("samples", nargs="+", type=parse_sample, metavar="row,col:x,y")
    parser.add_argument("--config", default="config.yaml")
    args = parser.parse_args()

    config_path = Path(args.config)
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    grid = GridMap.from_file(config_path.parent / config["map"])
    for cell, _ in args.samples:
        if not grid.is_passable(cell):
            parser.error(f"cell {cell} is not a free cell of {config['map']}")
    frame, errors = fit_grid_frame(grid.height, config["cell_size"], args.samples)

    print(f"map {config['map']}, cell size {config['cell_size']} m, {len(args.samples)} samples")
    for (cell, xy), error in zip(args.samples, errors):
        fitted = frame.to_metric(cell)
        print(f"  cell {cell}: measured ({xy[0]:.3f}, {xy[1]:.3f})"
              f"  fitted ({fitted[0]:.3f}, {fitted[1]:.3f})  off by {error * 100:.1f} cm")
    print(f"largest error {max(errors) * 100:.1f} cm; grid rotated {math.degrees(frame.yaw):.1f} degrees")
    print("\nPaste into config.yaml under ros2:")
    print(f"  grid_origin: {{x: {frame.x:.4f}, y: {frame.y:.4f}, yaw: {frame.yaw:.4f}}}")


if __name__ == "__main__":
    main()
