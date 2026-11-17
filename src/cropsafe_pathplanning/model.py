"""Grid and field model shared by the search algorithms."""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Iterable

Position = tuple[int, int]


@dataclass
class Field:
    """A rectangular four-connected field.

    Only protected and obstacle cells are forbidden to watering. Crop-row
    cells remain irrigable. This field model has no start or goal coordinates.
    """

    height: int
    width: int
    crop_rows: set[Position] = field(default_factory=set)
    protected: set[Position] = field(default_factory=set)
    obstacles: set[Position] = field(default_factory=set)
    risk_penalty: float = 2.0

    def __post_init__(self) -> None:
        if self.height <= 0 or self.width <= 0:
            raise ValueError("Field dimensions must be positive")
        if not math.isfinite(self.risk_penalty) or self.risk_penalty < 0:
            raise ValueError("risk_penalty must be finite and nonnegative")
        for position in self.crop_rows | self.protected | self.obstacles:
            if not self.in_bounds(position):
                raise ValueError(f"Position outside field: {position}")

    @property
    def blocked(self) -> set[Position]:
        return self.protected | self.obstacles

    @property
    def irrigable(self) -> set[Position]:
        """All field cells that should receive water."""
        return {
            (row, col)
            for row in range(self.height)
            for col in range(self.width)
            if (row, col) not in self.blocked
        }

    def in_bounds(self, position: Position) -> bool:
        row, col = position
        return 0 <= row < self.height and 0 <= col < self.width

    def neighbors(self, position: Position) -> Iterable[Position]:
        row, col = position
        # Fixed order makes runs deterministic and easier to compare.
        for candidate in ((row - 1, col), (row, col + 1), (row + 1, col), (row, col - 1)):
            if self.in_bounds(candidate) and candidate not in self.blocked:
                yield candidate

    def step_cost(self, destination: Position) -> float:
        if destination in self.blocked:
            raise ValueError(f"Cannot enter blocked cell {destination}")
        adjacent_protected = any(
            (destination[0] + dr, destination[1] + dc) in self.protected
            for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1))
        )
        return 1.0 + (self.risk_penalty if adjacent_protected else 0.0)

    def with_obstacles(self, obstacles: set[Position]) -> "Field":
        return Field(
            height=self.height,
            width=self.width,
            crop_rows=set(self.crop_rows),
            protected=set(self.protected),
            obstacles=set(obstacles),
            risk_penalty=self.risk_penalty,
        )


def demo_field() -> Field:
    """Return the fixed 12x18 scenario used by the command-line demo.

    Returns the complete field geometry used by the watering demo.
    """
    from .field_scenario import make_field

    return make_field()
