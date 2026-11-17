# Field Environment and Watering-Coverage Objective

The demonstration models a rectangular field as a 12 × 18 grid. Coordinates use zero-based `(row, column)` indexing, with rows increasing downward and columns increasing to the right.

## Cell rules

| Cell class | Coverage rule |
|---|---|
| Ordinary field cell | Irrigated |
| Crop row | Irrigated |
| Protected cell | Never entered or irrigated |
| Obstacle | Never entered or irrigated |

Thus `irrigable = all grid cells − protected − obstacles`. Crop rows are descriptive field metadata; they do not make a cell forbidden. The sample has 216 total cells, 28 protected cells, and 16 obstacle cells, so the coverage objective is 172 irrigable cells.

## Coverage planning

The main program does not request or show a start or destination. It picks the first irrigable cell internally as a tour launch point. The completion condition is that every irrigable cell has been visited. A single continuous tour requires the irrigable region to be connected; otherwise the planner raises a clear error instead of claiming complete coverage.

Legal movement is four-directional (up, right, down, left). BFS, DFS, Greedy Best-First, and A* each produce an exploration order; the robot follows legal routes between the cells in that order. Repeated visits are allowed because the robot may need to cross cells already watered to reach the next area.

## Output

The command line prints a grid using `W` for watered cells, `P` for protected cells, and `X` for obstacles. The generated SVG shows the same coverage mask without route lines or endpoint markers. A CSV compares coverage completeness, route movement count, expanded cells, and runtime for each algorithm.

## Shared sample geometry

- Crop metadata: rows 3 and 8, columns 1–16; these cells are irrigated.
- Protected cells: rows 3 and 8, columns 2–15; these cells are excluded.
- Obstacles: `(1,4)`, `(1,5)`, `(2,5)`, `(4,8)`, `(5,5)`, `(5,8)`, `(5,12)`, `(6,5)`, `(6,8)`, `(6,12)`, `(9,4)`, `(9,14)`, `(10,4)`, `(10,5)`, `(10,13)`, and `(10,14)`.
- All 16 obstacles are included in the field from the beginning and remain excluded from watering.
