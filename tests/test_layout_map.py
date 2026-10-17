"""The nav2 map drawn from the grid layout must put walls exactly on the cell edges."""
import pytest
import yaml

from robots.grid_frame import GridFrame
from robots.layout_map import FREE_PX, OCCUPIED_PX, UNKNOWN_PX, render, wall_segments, write


@pytest.fixture(scope="module")
def drawn(arena_map, config):
    cfg = config["ros2"]["layout_map"]
    img, origin = render(arena_map, config["cell_size"], cfg["resolution"], cfg["wall_thickness"], cfg["margin"])
    return img, origin, cfg["resolution"]


def pixel(drawn, x, y):
    img, (ox, oy, _), res = drawn
    col = int((x - ox) / res)
    row = img.shape[0] - 1 - int((y - oy) / res)
    return img[row, col]


def test_cell_centers_free_outside_unknown(drawn, arena_map, config):
    frame = GridFrame(arena_map.height, config["cell_size"])  # grid_origin is zero for this map
    for r in range(arena_map.height):
        for c in range(arena_map.width):
            expected = FREE_PX if arena_map.is_passable((r, c)) else UNKNOWN_PX
            assert pixel(drawn, *frame.to_metric((r, c))) == expected, (r, c)
    assert pixel(drawn, -0.1, -0.1) == UNKNOWN_PX  # margin


def test_walls_on_edges(drawn, arena_map, config):
    cs = config["cell_size"]
    frame = GridFrame(arena_map.height, cs)
    x20, y20 = frame.to_metric((2, 0))
    assert pixel(drawn, x20 - cs / 2, y20) == OCCUPIED_PX   # outer wall left of (2,0)
    assert pixel(drawn, x20, y20 + cs / 2) == OCCUPIED_PX   # thin wall (1,0)-(2,0)
    assert pixel(drawn, x20 + cs / 2, y20) == FREE_PX       # open edge (2,0)-(2,1)
    x11, y11 = frame.to_metric((1, 1))
    assert pixel(drawn, x11, y11 - cs / 2) == FREE_PX       # open edge (1,1)-(2,1)
    x24, y24 = frame.to_metric((2, 4))
    assert pixel(drawn, x24, y24 - cs / 2) == OCCUPIED_PX   # thin wall (2,4)-(3,4)


def test_wall_count(arena_map, config):
    # Outer edges: room A 6, niche (1,4) 3, corridor row 2 9, room B 6; plus the 2 thin walls.
    assert len(wall_segments(arena_map, config["cell_size"])) == 26


def test_written_yaml(tmp_path, drawn):
    img, origin, res = drawn
    write(tmp_path / "arena", img, res, origin)
    meta = yaml.safe_load((tmp_path / "arena.yaml").read_text())
    assert meta["image"] == "arena.pgm" and meta["resolution"] == res and meta["origin"] == origin
    data = (tmp_path / "arena.pgm").read_bytes()
    header = f"P5\n{img.shape[1]} {img.shape[0]}\n255\n".encode()
    assert data.startswith(header) and len(data) == len(header) + img.size
