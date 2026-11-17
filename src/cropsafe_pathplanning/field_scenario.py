"""Shared field scenario for the AI-CropSafe Path Planning project.

This module exposes the shared watering field configuration and factories:

- ``FIELD_CONFIG``  — a plain dictionary describing every parameter of the demo
  field, usable by any module without importing the full ``Field`` dataclass.
- ``make_field_map()``  — constructs a ``FieldMap`` with protected and obstacle
  cells blocked.
- ``make_field()``  — constructs the ``Field`` dataclass used by the coverage
  planner.
- ``print_field_ascii()``  — prints a labelled ASCII map to stdout for quick
  visual verification.

Loading the scenario
--------------------
Any module in the project can get the shared configuration with::

    from cropsafe_pathplanning.field_scenario import (
        FIELD_CONFIG,
        make_field,
        make_field_map,
       print_field_ascii,
    )

Both ``make_field()`` and ``make_field_map()`` read from ``FIELD_CONFIG`` so
the single source of truth is never duplicated.

Coordinate convention
---------------------
Every position is a ``(row, col)`` tuple with 0-based indexing.
Rows increase **downward**; columns increase **rightward**.

Movement rules
--------------
Four-directional (cardinal) movement only: UP (-1,0), DOWN (+1,0),
LEFT (0,-1), RIGHT (0,+1).  Diagonal steps are **not** permitted.
This is confirmed by ``Field.neighbors()`` and ``FieldMap.get_neighbors()``.

Watering rules
--------------
- Crop rows are irrigable.
- Protected cells and obstacles are excluded from watering.
- Every other field cell must be visited by the complete coverage tour.

Formal problem summary
----------------------
- Coverage objective: visit every cell except protected cells and obstacles.
- Crop rows are irrigable and must be included in the coverage.
- Actions: move up, right, down, or left through irrigable cells.
- Completion: every irrigable cell has been visited by the watering tour.
"""

from __future__ import annotations

from typing import Any

# ---------------------------------------------------------------------------
# Single source of truth for the shared 12 × 18 demo field
# ---------------------------------------------------------------------------

FIELD_CONFIG: dict[str, Any] = {
    # Grid dimensions
    "height": 12,
    "width": 18,
    # Crop rows remain irrigable as part of full-field watering coverage.
    # Band A: row 3, cols 1–16
    # Band B: row 8, cols 1–16
    "crop_rows": frozenset(
        (r, c) for r in (3, 8) for c in range(1, 17)
    ),
    # Protected cells — excluded from watering.
    # Band A: row 3, cols 2–15
    # Band B: row 8, cols 2–15
    "protected": frozenset(
        (r, c) for r in (3, 8) for c in range(2, 16)
    ),
    # Sixteen obstacles distributed in small clusters across the field.
    # Keep gaps between clusters so all irrigable regions remain connected.
    "obstacles": frozenset({
        (1, 4), (1, 5), (2, 5),
        (4, 8), (5, 5), (5, 8), (5, 12),
        (6, 5), (6, 8), (6, 12),
        (9, 4), (9, 14), (10, 4), (10, 5), (10, 13), (10, 14),
    }),
    # Retained for the standalone point-to-point A* exercise.
    "risk_penalty": 2.0,
}


# ---------------------------------------------------------------------------
# Factory helpers
# ---------------------------------------------------------------------------

def make_field():  # type: ignore[return]
    """Return a ``Field`` dataclass populated from ``FIELD_CONFIG``.

    Use this with the full-field coverage planner.

    Returns
    -------
    model.Field
        The 12 × 18 field with crop, protected, and obstacle metadata.
    """
    from cropsafe_pathplanning.model import Field

    cfg = FIELD_CONFIG
    return Field(
        height=cfg["height"],
        width=cfg["width"],
        crop_rows=set(cfg["crop_rows"]),
        protected=set(cfg["protected"]),
        obstacles=set(cfg["obstacles"]),
        risk_penalty=cfg["risk_penalty"],
    )


def make_field_map():
    """Return a ``FieldMap`` object populated from ``FIELD_CONFIG``.

    Use this with the BFS / DFS (``bfs_dfs.py``) and GBFS (``gbfs.py``)
    modules that consume the ``types.FieldMap`` interface.

    Returns
    -------
    types.FieldMap
        The 12 × 18 demo field with protected cells and obstacles blocked.
    """
    from cropsafe_pathplanning.types import FieldMap

    cfg = FIELD_CONFIG
    blocked = set(cfg["protected"]) | set(cfg["obstacles"])
    return FieldMap(
        height=cfg["height"],
        width=cfg["width"],
        blocked_cells=blocked,
    )


# ---------------------------------------------------------------------------
# ASCII visualisation
# ---------------------------------------------------------------------------

# Symbol legend
_SYMBOLS: dict[str, str] = {
    "safe": ".",
    "crop": "W",
    "protected": "P",
    "obstacle": "X",
    "path": "*",
    "expanded": "·",
}

SYMBOL_LEGEND: str = """\
Symbol legend
  .  Irrigable field cell
  P  Protected cell (excluded from watering)
  X  Known obstacle (excluded from watering)
  *  Cell included in a rendered route / coverage set
"""


def render_ascii(
    *,
    path: list[tuple[int, int]] | None = None,
    expanded: list[tuple[int, int]] | None = None,
) -> str:
    """Return a labelled ASCII string of the demo field.

    Parameters
    ----------
    path:
        Optional list of ``(row, col)`` positions to mark with ``*``.
    expanded:
        Optional list of ``(row, col)`` positions to mark with ``·``.

    Returns
    -------
    str
        Multi-line string ready to be printed.
    """
    cfg = FIELD_CONFIG
    h, w = cfg["height"], cfg["width"]
    protected: frozenset[tuple[int, int]] = cfg["protected"]
    obstacles: frozenset[tuple[int, int]] = cfg["obstacles"]

    path_set = set(path or [])
    expanded_set = set(expanded or [])

    lines: list[str] = []
    # Column header
    col_hdr = "     " + " ".join(f"{c:2d}" for c in range(w))
    lines.append(col_hdr)
    lines.append("   +" + "---" * w + "+")

    for r in range(h):
        row_chars: list[str] = []
        for c in range(w):
            pos = (r, c)
            if pos in protected:
                ch = " P "
            elif pos in obstacles:
                ch = " X "
            elif pos in path_set:
                ch = " * "
            elif pos in expanded_set:
                ch = " · "
            else:
                ch = " . "
            row_chars.append(ch)
        lines.append(f"{r:2d} |" + "".join(row_chars) + "|")

    lines.append("   +" + "---" * w + "+")
    return "\n".join(lines)


def print_field_ascii(**kwargs: Any) -> None:
    """Print the demo field ASCII map to stdout.

    Accepts the same keyword arguments as :func:`render_ascii`.
    """
    print(render_ascii(**kwargs))
    print()
    print(SYMBOL_LEGEND)


# ---------------------------------------------------------------------------
# Quick self-test when run as a script
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    print("Watering field (P = protected, X = obstacle; all other cells irrigable)")
    print_field_ascii()
    print(f"Irrigable cells: {len(make_field().irrigable)}")
