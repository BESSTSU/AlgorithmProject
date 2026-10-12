"""Grid map loaded from a text file. Cells are (row, col) with the origin at the top-left."""
from pathlib import Path

WALL = "#"
FREE = "."
DOOR = "D"
CHARGER = "C"
# Any other uppercase letter (A, B, ...) marks a cell inside that room.

Cell = tuple[int, int]

_DIRECTIONS = ((-1, 0), (1, 0), (0, -1), (0, 1))  # N, S, W, E


class GridMap:
    def __init__(self, lines: list[str]):
        rows = [line.rstrip() for line in lines if line.strip()]
        if not rows:
            raise ValueError("map is empty")
        width = len(rows[0])
        for r, row in enumerate(rows):
            if len(row) != width:
                raise ValueError(f"row {r} has width {len(row)}, expected {width}")
            for c, ch in enumerate(row):
                if ch not in (WALL, FREE) and not ch.isupper():
                    raise ValueError(f"unknown map symbol {ch!r} at {(r, c)}")
        self.rows = rows
        self.height = len(rows)
        self.width = width

    @classmethod
    def from_file(cls, path) -> "GridMap":
        return cls(Path(path).read_text(encoding="utf-8").splitlines())

    def symbol(self, cell: Cell) -> str:
        r, c = cell
        return self.rows[r][c]

    def in_bounds(self, cell: Cell) -> bool:
        r, c = cell
        return 0 <= r < self.height and 0 <= c < self.width

    def is_passable(self, cell: Cell) -> bool:
        return self.in_bounds(cell) and self.symbol(cell) != WALL

    def neighbors(self, cell: Cell) -> list[Cell]:
        """Passable 4-connected neighbors, in a fixed order (N, S, W, E)."""
        r, c = cell
        candidates = [(r + dr, c + dc) for dr, dc in _DIRECTIONS]
        return [n for n in candidates if self.is_passable(n)]

    def is_side_pocket(self, cell: Cell) -> bool:
        """A dead-end corridor cell that branches sideways off a through-passage, like (4,6).

        The end of a straight corridor, like (3,0), is not a side pocket.
        """
        if not self.in_bounds(cell) or self.symbol(cell) != FREE:
            return False
        nbrs = self.neighbors(cell)
        if len(nbrs) != 1:
            return False
        (nr, nc), (r, c) = nbrs[0], cell
        dr, dc = nr - r, nc - c
        # The two cells beside the neighbor, perpendicular to the branch direction.
        return self.is_passable((nr + dc, nc + dr)) and self.is_passable((nr - dc, nc - dr))

    def room_cells(self) -> list[Cell]:
        """All cells inside rooms (any room letter), in row-major order."""
        return [
            (r, c)
            for r, row in enumerate(self.rows)
            for c, ch in enumerate(row)
            if ch not in (WALL, FREE, DOOR, CHARGER)
        ]

    def cells_with(self, symbol: str) -> list[Cell]:
        """All cells marked with `symbol`, in row-major order."""
        return [
            (r, c)
            for r, row in enumerate(self.rows)
            for c, ch in enumerate(row)
            if ch == symbol
        ]
