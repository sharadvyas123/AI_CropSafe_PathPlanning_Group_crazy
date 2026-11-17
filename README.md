# AI-Based Crop-Safe Field Watering

A grid simulation for planning a robot tour that waters the complete irrigable field. The demo does not ask for a start point or a destination. It automatically plans coverage over all cells except protected areas and obstacles.

## Watering rules

- Every cell in the rectangular field is eligible for watering unless it is marked protected or as an obstacle.
- Crop-row cells are irrigable and must be covered.
- Protected cells and obstacle cells are never watered or entered.
- A plan is complete when its route has visited every eligible cell.
- A continuous tour requires the eligible cells to form one connected region. The planner reports an error if obstacles split the field into disconnected regions.

The sample is a 12 × 18 grid with 28 protected cells and 16 obstacles. The obstacle coordinates are listed in `FIELD_CONFIG["obstacles"]` in `src/cropsafe_pathplanning/field_scenario.py`. To change the obstacle layout, edit that set and rerun the demo. `Field.blocked` combines protected cells and obstacles, and `Field.irrigable` subtracts those blocked cells from the whole grid; the coverage planners then route only through irrigable cells.

The first eligible cell in row-major order is selected internally as the tour's launch cell. No user-supplied start or goal is required, printed, or shown in the coverage map.

## Repository layout

```text
src/cropsafe_pathplanning/
├── coverage.py          # BFS, DFS, Greedy, and A* coverage tours
├── field_scenario.py    # Shared field layout
├── model.py             # Field and irrigable-cell rules
├── visualize.py         # SVG coverage map
└── __main__.py          # CLI and CSV output
docs/                    # Algorithm and field notes
results/
├── watering_coverage.csv
└── screenshots/watering_coverage.svg
tests/                   # Project checks
```

## Setup

Python 3.10 or newer is recommended. The project uses only the Python standard library.

```bash
python -m venv .venv
```

Activate the environment on macOS/Linux:

```bash
source .venv/bin/activate
```

Activate on Windows PowerShell:

```powershell
.venv\Scripts\Activate.ps1
```

Install the project if desired:

```bash
python -m pip install -e .
```

## Run the presentation demo

From the repository root:

```bash
PYTHONPATH=src python -m cropsafe_pathplanning
```

The command runs BFS, DFS, Greedy Best-First, and A* as coverage planners. It prints a comparison and a text map (`W` watered, `P` protected, `X` obstacle), then writes:

- `results/watering_coverage.csv` — coverage, tour length, expanded cells, and runtime for each algorithm.
- `results/screenshots/watering_coverage.svg` — one field coverage map, with no start/goal markers.

## What the algorithms do

Each algorithm explores the graph of irrigable cells using its frontier rule, and the robot follows legal connecting routes through that order. BFS explores breadth-first, DFS explores depth-first, Greedy prioritizes progress toward the far side of the field, and A* combines accumulated traversal distance with that sweep heuristic. All four must cover the same eligible cell set; their movement counts and runtimes can differ.

## Assignment background

The original point-to-point search implementations remain available in their modules for the algorithm exercises, but the main project demo and its reported results now use the full-field watering objective described above.
