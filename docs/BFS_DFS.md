# BFS and DFS — Member 2 (Dushyant)

> **Scope:** This document describes the standalone point-to-point search APIs. The project demo uses the full-field coverage planners in `src/cropsafe_pathplanning/coverage.py`; it does not request or display start/goal points.

> [!IMPORTANT]
> Both algorithms use the **shared interfaces** from `cropsafe_pathplanning.types`
> (`FieldMap`, `SearchResult`, `CARDINAL_DIRECTIONS`) and are compatible with the
> `Field` model defined by Hitesh (Member 1).

---

## Table of Contents

1. [Algorithm Overview](#algorithm-overview)
2. [Implementation — Where to Find It](#implementation--where-to-find-it)
3. [Public API](#public-api)
4. [Shared Result Interface](#shared-result-interface)
5. [BFS — Shortest Path Guarantee](#bfs--shortest-path-guarantee)
6. [DFS — Completeness Without Optimality](#dfs--completeness-without-optimality)
7. [Comparison Table](#comparison-table)
8. [Reproducible Examples](#reproducible-examples)
   - [Example 1 — Reachable Goal (open grid)](#example-1--reachable-goal-open-grid)
   - [Example 2 — Obstacle Avoidance (vertical wall)](#example-2--obstacle-avoidance-vertical-wall)
   - [Example 3 — Unreachable Goal (enclosed cell)](#example-3--unreachable-goal-enclosed-cell)
   - [Example 4 — Demo Field Scenario](#example-4--demo-field-scenario)
9. [Design Decisions](#design-decisions)
10. [Running the Tests](#running-the-tests)

---

## Algorithm Overview

| Property | BFS | DFS |
|---|---|---|
| Frontier structure | FIFO queue (`collections.deque`) | LIFO stack (`list`) |
| Completeness (finite graphs) | ✅ Yes | ✅ Yes |
| Shortest path (uniform costs) | ✅ Guaranteed | ❌ Not guaranteed |
| Memory usage | O(b^d) — wide | O(b·m) — narrow |
| Typical use case | Shortest step count matters | Existence of path matters, memory is tight |

> [!NOTE]
> *b* = branching factor (≤ 4 for 4-connected grid), *d* = depth of shallowest goal,
> *m* = maximum depth of the search tree.

---

## Implementation — Where to Find It

```
src/cropsafe_pathplanning/bfs_dfs.py   ← BFS and DFS implementations
tests/test_bfs_dfs.py                   ← 25 automated tests
docs/BFS_DFS.md                         ← this document
```

---

## Public API

Both functions share the same signature pattern:

```python
from cropsafe_pathplanning.bfs_dfs import breadth_first_search, depth_first_search
from cropsafe_pathplanning.types import FieldMap

result = breadth_first_search(
    field,          # FieldMap | list[list] | duck-typed object
    start,          # (row, col) int tuple
    goal,           # (row, col) int tuple
    *,
    movement_directions = CARDINAL_DIRECTIONS,  # 4-directional by default
    cost_fn = None,                             # optional (from, to) → float
    raise_on_invalid = False,                   # True → raise on bad input
)
```

**Accepted `field` types:**

| Type | Behaviour |
|---|---|
| `FieldMap` | Used directly, **never mutated** |
| `list[list[int\|str\|bool]]` | Forwarded to `FieldMap.from_matrix` |
| Any object with `height`, `width`, `is_traversable` | Converted to `FieldMap` |

---

## Shared Result Interface

Both functions return a `SearchResult` (a `dict` subclass) with these keys:

```python
{
    "path":           list[tuple[int, int]],   # (row, col) from start → goal; [] on failure
    "cost":           int | float,             # total step cost; float("inf") on failure
    "expanded_nodes": list[tuple[int, int]],   # nodes popped from frontier, in order
    "reached_goal":   bool,                    # True iff goal was reached
}
```

Property accessors (`.path`, `.cost`, `.expanded_nodes`, `.reached_goal`) mirror the dict values for convenience.

> [!WARNING]
> A **failed search** always returns `path=[]` and `cost=float("inf")`.
> Never treat a zero-length path as success.

---

## BFS — Shortest Path Guarantee

BFS explores the grid **level by level**: all cells one step from start are
discovered before any cell two steps away, and so on.

```
Frontier at each level (4×4 open grid, start = (0,0)):
  Level 0: {(0,0)}
  Level 1: {(1,0), (0,1)}
  Level 2: {(2,0), (1,1), (0,2)}
  ...
```

**Why BFS is optimal under uniform costs:**

- Every path of length *k* is fully explored before any path of length *k + 1*.
- The first time the goal is dequeued it was reached via the shortest route.
- With equal step costs (cost = 1 per move) the step count equals the path
  cost, so BFS is simultaneously step-optimal and cost-optimal.

> [!IMPORTANT]
> BFS **does not optimise for weighted costs**.  If some moves cost more than
> others (e.g., the `risk_penalty` near crop rows), use **A\*** instead — it
> handles variable-cost edges and still guarantees the cheapest path.

---

## DFS — Completeness Without Optimality

DFS always goes as deep as possible before backtracking.  It is implemented
with an explicit LIFO stack (not recursion, so there is no Python recursion
limit concern on large grids).

```
Stack evolution (4×4 open grid, start = (0,0)):
  Push (0,0)
  Pop  (0,0) → push neighbours: ... (0,1) goes in first; (1,0) goes on top
  Pop  (1,0) → push deeper ...
  ... keeps diving down column 0, then backtracks
```

**Why DFS does not guarantee the shortest path:**

- DFS commits to the first explored branch regardless of its length.
- On an open grid from (0,0) to (4,4), BFS returns a path of exactly 8 steps
  (the Manhattan distance); DFS may return a path of 16 or more steps.
- The returned path is valid and obstacle-free, but it was not chosen for
  its brevity.

> [!TIP]
> DFS is useful when you only need to know *whether* a path exists, or when
> memory is severely constrained (frontier size is bounded by path depth, not
> the entire wavefront).

---

## Comparison Table

```
Grid (5×5 open, start=(0,0), goal=(4,4)):

    0  1  2  3  4
 0 [S][·][·][·][·]      · = expanded by BFS
 1 [·][·][·][·][·]      * = on BFS shortest path
 2 [·][·][·][·][·]
 3 [·][·][·][·][*]
 4 [·][·][·][*][G]

BFS path  : (0,0)→(1,0)→(2,0)→(3,0)→(4,0)→(4,1)→(4,2)→(4,3)→(4,4)   [8 steps]
DFS path  : varies (may zigzag through most of the grid)                 [≥8 steps]
```

---

## Reproducible Examples

All examples can be copy-pasted into a Python shell.  Add the `src/` directory
to `PYTHONPATH` first, or run from inside the project root with:

```bash
PYTHONPATH=src python3
```

### Example 1 — Reachable Goal (open grid)

```python
from cropsafe_pathplanning.bfs_dfs import breadth_first_search, depth_first_search

grid = [
    [0, 0, 0, 0],
    [0, 0, 0, 0],
    [0, 0, 0, 0],
    [0, 0, 0, 0],
]
start, goal = (0, 0), (3, 3)

bfs = breadth_first_search(grid, start, goal)
dfs = depth_first_search(grid, start, goal)

print("BFS reached goal:", bfs.reached_goal)   # True
print("BFS path:", bfs.path)                   # 7-cell path (6 steps)
print("BFS cost:", bfs.cost)                   # 6

print("DFS reached goal:", dfs.reached_goal)   # True
print("DFS path:", dfs.path)                   # valid but possibly longer
print("DFS cost:", dfs.cost)                   # >= 6
```

**Expected output (BFS optimal path):**
```
BFS path: [(0,0), (1,0), (2,0), (3,0), (3,1), (3,2), (3,3)]
BFS cost: 6
```

---

### Example 2 — Obstacle Avoidance (vertical wall)

```python
# Vertical wall in column 2 with a gap at row 3
grid = [
    [0, 0, 1, 0, 0],
    [0, 0, 1, 0, 0],
    [0, 0, 1, 0, 0],
    [0, 0, 0, 0, 0],  # gap
    [0, 0, 1, 0, 0],
]
start, goal = (1, 1), (1, 4)

bfs = breadth_first_search(grid, start, goal)
print("BFS path:", bfs.path)
# Must detour through row 3 to pass the wall

# Verify no path cell is blocked
blocked = {(0,2),(1,2),(2,2),(4,2)}
assert all(c not in blocked for c in bfs.path), "Path crossed a wall!"
print("No blocked cells in path ✓")
```

---

### Example 3 — Unreachable Goal (enclosed cell)

```python
# (3,3) is surrounded by blocked cells — unreachable
grid = [
    [0, 0, 0, 0],
    [0, 0, 0, 0],
    [0, 0, 1, 1],
    [0, 0, 1, 0],   # (3,3) is walled off
]
start, goal = (0, 0), (3, 3)

import math

bfs = breadth_first_search(grid, start, goal)
dfs = depth_first_search(grid, start, goal)

for label, res in [("BFS", bfs), ("DFS", dfs)]:
    print(f"{label} reached_goal: {res.reached_goal}")   # False
    print(f"{label} path:         {res.path}")            # []
    print(f"{label} cost:         {res.cost}")            # inf
    assert not res.reached_goal
    assert res.path == []
    assert math.isinf(res.cost)

print("Both correctly detected unreachable goal ✓")
```

---

### Example 4 — Demo Field Scenario

Uses Hitesh's shared `demo_field()` (12×18 grid, blocked crop rows, and obstacles):

```python
from cropsafe_pathplanning.model import demo_field
from cropsafe_pathplanning.types import FieldMap
from cropsafe_pathplanning.bfs_dfs import breadth_first_search, depth_first_search

demo = demo_field()
field_map = FieldMap(
    height=demo.height,
    width=demo.width,
    blocked_cells=set(demo.blocked),   # obstacles ∪ protected cells
)

bfs = breadth_first_search(field_map, demo.start, demo.goal)
dfs = depth_first_search(field_map, demo.start, demo.goal)

print(f"BFS: reached={bfs.reached_goal}, steps={len(bfs.path)-1}, expanded={len(bfs.expanded_nodes)}")
print(f"DFS: reached={dfs.reached_goal}, steps={len(dfs.path)-1}, expanded={len(dfs.expanded_nodes)}")

# Visualise BFS path
print(field_map.render(path=bfs.path, start=demo.start, goal=demo.goal))
```

> [!NOTE]
> `demo.blocked` = `demo.protected | demo.obstacles` as defined in `model.py`.
> The `FieldMap` passed here is a **new, independent object** — the original
> `demo` Field is not modified.

---

## Design Decisions

| Decision | Rationale |
|---|---|
| `FieldMap` **not mutated** | Algorithms only read `is_traversable`, `is_within_bounds`, and `get_neighbors`; the `blocked_cells` set is never touched. |
| `parent` dict initialised with `{start: None}` | Avoids a separate `visited` set; doubles as the "discovered" record and the path-reconstruction map. |
| Neighbours reversed before push (DFS) | Ensures the first direction in `CARDINAL_DIRECTIONS` (Up) is examined first, keeping results deterministic and consistent with BFS order. |
| `start == goal` handled as a special case | Returns `path=[start]`, `cost=0`, `expanded_nodes=[start]` immediately without entering the main loop. |
| `raise_on_invalid=False` default | Matches the GBFS convention in `gbfs.py` so callers can choose between exception raising and silent failure results. |
| `cost_fn` parameter | Lets callers plug in the `Field.step_cost` function for weighted-cost scenarios while keeping BFS/DFS independent of the `Field` class. |

---

## Running the Tests

```bash
# From the project root
PYTHONPATH=src python3 -m unittest tests.test_bfs_dfs -v
```

**Expected:**
```
Ran 25 tests in 0.003s
OK
```

Tests cover:
- Shared result-format contract (dict, keys, types)
- Simple reachable and unreachable grids
- Obstacle avoidance (path never enters blocked cells)
- BFS optimality (path length = Manhattan distance on open grid)
- DFS non-optimality (path length ≥ BFS path length)
- Invalid inputs (out-of-bounds, blocked endpoints)
- Start == goal edge case
- Reproducibility across 5 consecutive runs
- Expanded-nodes ordering, bounds, and uniqueness
- No field mutation
- Path traceability (every step is a legal cardinal move)
- Custom `cost_fn` integration
- Demo field (Hitesh's shared scenario) — reachable and unreachable variants

