# Greedy Best-First Search (GBFS) and Manhattan Distance Heuristic

> **Scope:** This document describes the standalone point-to-point GBFS API. The main project demo uses a separate GBFS coverage strategy that visits the whole irrigable field without user-specified endpoints; see `src/cropsafe_pathplanning/coverage.py`.

> **Project:** AI-Based Crop-Safe Path Planning  
> **Module:** Greedy Best-First Search & Informed Heuristic Design  
> **Author:** Ricky (Member 3)  
> **Target Python Version:** 3.10+  

---

## 1. Overview: What is Greedy Best-First Search?

**Greedy Best-First Search (GBFS)** is an informed (heuristic) search algorithm designed for pathfinding and state-space exploration. Unlike uninformed algorithms (such as Breadth-First Search or Depth-First Search) that blindly search through nodes, GBFS utilizes domain-specific knowledge in the form of a **heuristic evaluation function**, $h(n)$.

At each step, GBFS expands the frontier node that appears closest to the goal state according to the evaluation function:

$$f(n) = h(n)$$

Where:
- $n$: The candidate node under evaluation.
- $h(n)$: The estimated distance or cost from node $n$ to the goal.

### Operational Characteristics in Agricultural Robotics:
In an agricultural setting (e.g., an autonomous farm vehicle navigating crop rows in a potato field), GBFS attempts to drive the robot directly toward the designated destination by greedily selecting adjacent grid cells that minimize estimated distance to the goal.

---

## 2. Priority Queue & Deterministic Tie-Breaking

GBFS maintains an **open set (frontier)** organized as a min-priority queue using Python's standard library `heapq`.

### Priority Queue Mechanics:
1. When a node $n$ is discovered, its heuristic value $h(n)$ is computed.
2. The tuple `(h_cost, insertion_counter, node)` is pushed onto the min-heap.
3. In each iteration, the algorithm pops the entry with the minimum $h(n)$ value.

### Deterministic Tie-Breaking:
If multiple discovered cells have the exact same heuristic value $h(n)$, an arbitrary tie-breaker can lead to unpredictable, non-reproducible path choices across repeated runs.

To ensure **strict reproducibility**, our implementation introduces an incrementing monotonic integer counter (`itertools.count()`):
```python
heapq.heappush(open_heap, (h_score, next(counter), neighbor_coordinate))
```
This guarantees:
1. **First-In, First-Out (FIFO)** tie-breaking among equidistant candidates.
2. Coordinates are never compared directly by Python's `heapq`, preventing coordinate comparison overhead.
3. Neighbor expansions occur in a fixed directional order (Up, Down, Left, Right).

---

## 3. Manhattan Distance Heuristic Calculation

For a two-dimensional grid where positions are specified as `(row, column)` coordinates, the **Manhattan Distance** (also known as $L_1$ norm or taxicab distance) is defined as:

$$h(n) = |r_n - r_{goal}| + |c_n - c_{goal}|$$

### Properties:
1. **Zero at Goal:** $h(goal) = |r_{goal} - r_{goal}| + |c_{goal} - c_{goal}| = 0$.
2. **Nonnegativity:** For all real/integer coordinates, $h(n) \ge 0$.
3. **Symmetry:** $h(u, v) = h(v, u)$.
4. **Triangle Inequality:** $h(u, w) \le h(u, v) + h(v, w)$.

---

## 4. Why Manhattan Distance Fits Four-Directional Movement

In our potato-field simulation, the robot moves strictly in **four orthogonal directions**:
- $\text{Up}: (-1, 0)$
- $\text{Down}: (1, 0)$
- $\text{Left}: (0, -1)$
- $\text{Right}: (0, 1)$

Each legal cardinal move alters either the row index by $\pm 1$ or the column index by $\pm 1$, never both at once. 

Under a uniform cost model (each step costs 1 unit), the minimum number of orthogonal transitions required to move from $(r_1, c_1)$ to $(r_2, c_2)$ on an unobstructed grid is mathematically bounded below by:

$$\Delta r + \Delta c = |r_1 - r_2| + |c_1 - c_2|$$

Hence, Manhattan distance represents the **exact shortest path distance on an obstacle-free grid**. In search theory:
- It is **admissible** (it never overestimates the actual cost to reach the goal).
- It is **consistent (monotonic)**, satisfying $h(n) \le c(n, a, n') + h(n')$.

---

## 5. Effect of Permitting Diagonal Movement

If the robot's movement rules were modified to permit diagonal transitions, Manhattan distance would **overestimate** the true path cost, losing admissibility.

The appropriate heuristic depends on the diagonal movement cost model:

### 1. Chebyshev Distance ($L_\infty$ Metric)
If diagonal steps cost the same as cardinal steps ($c = 1$ for all 8 directions):
$$h_{\text{Chebyshev}}(n) = \max(|r_n - r_{goal}|, |c_n - c_{goal}|)$$
Because one diagonal step decrements both row and column differences simultaneously, the minimum steps required is the maximum of the two coordinate offsets.

### 2. Octile Distance
If diagonal steps reflect true Euclidean geometry with cost $\sqrt{2} \approx 1.414$ while cardinal steps cost $1$:
Let $\Delta r = |r_n - r_{goal}|$ and $\Delta c = |c_n - c_{goal}|$.
$$h_{\text{Octile}}(n) = (\sqrt{2} - 1) \cdot \min(\Delta r, \Delta c) + \max(\Delta r, \Delta c)$$
Here, the agent takes $\min(\Delta r, \Delta c)$ diagonal steps followed by $|\Delta r - \Delta c|$ straight cardinal steps.

### 3. Euclidean Distance ($L_2$ Metric)
If movement is continuous or unconstrained by grid discretization:
$$h_{\text{Euclidean}}(n) = \sqrt{(r_n - r_{goal})^2 + (c_n - c_{goal})^2}$$

---

## 6. Why GBFS is NOT Guaranteed to Find a Least-Cost (Optimal) Route

One of the most crucial theoretical distinctions in AI search algorithms is the difference between **Greedy Best-First Search** and **A\* Search**:

| Property | Greedy Best-First Search (GBFS) | A\* Search |
|---|---|---|
| **Evaluation Function** | $f(n) = h(n)$ | $f(n) = g(n) + h(n)$ |
| **Path Cost Considered?** | **No.** Completely ignores $g(n)$ (distance traveled so far). | **Yes.** Balances cost accumulated $g(n)$ with estimated remaining cost $h(n)$. |
| **Optimality Guarantee** | **Not optimal.** Can return substantially suboptimal paths. | **Optimal**, provided $h(n)$ is admissible. |
| **Primary Advantage** | Aggressively rushes toward the goal; frequently expands fewer nodes in open space. | Guarantees finding the minimum-cost route. |

### Intuitive Cause of Suboptimality:
GBFS is susceptible to **local minima and deceptive heuristic gradients**:
- When encountering an obstacle or concave barrier (such as a long crop row or U-shaped ditch), GBFS prioritizes cells that minimize straight-line distance to the goal.
- It will plunge deeply into dead-ends or long winding detours simply because the intermediate cells appear closer to the goal coordinates, entirely disregarding the high cost of the detour already traveled.
- In contrast, A* would notice $g(n)$ rising along the detour and switch to a shorter alternative branch.

---

## 7. Obstacle Handling and Unreachable Goals

### 1. Obstacle and Protected Crop Avoidance
- All grid cells labeled as obstacles (`1`, `'#'`, `'X'`) or protected potato crop zones (`'C'`) are marked non-traversable in `FieldMap`.
- When expanding neighbors, GBFS checks `field.is_traversable(neighbor)`. Blocked cells are never enqueued into the open set, preventing the robot from ever entering forbidden territory.

### 2. Input Validation (Rejecting Invalid Endpoints)
- If `start` or `goal` is out of bounds or located on a blocked cell, the search rejects the query cleanly.
- When `raise_on_invalid=True`, a descriptive `ValueError` is raised.
- When `raise_on_invalid=False`, the search returns a structured failure result:
  ```python
  {
      "path": [],
      "cost": float("inf"),
      "expanded_nodes": [],
      "reached_goal": False,
  }
  ```

### 3. Unreachable Goals (Disconnected Search Spaces)
- If the goal is completely surrounded by obstacles or lies in an inaccessible partition of the field:
  - GBFS systematically explores all reachable traversable cells.
  - The `visited` set prevents infinite loops and duplicate node evaluations.
  - When the priority queue becomes empty without discovering the goal, GBFS terminates gracefully and returns:
    - `reached_goal = False`
    - `path = []`
    - `cost = float('inf')`
    - `expanded_nodes`: Complete ordered record of all cells expanded during the attempt.

---

## 8. Shared Result Interface

The algorithm returns a `SearchResult` dictionary conforming to the project schema:

```python
{
    "path": [(0, 0), (1, 0), (2, 0), ...],  # Ordered coordinates from start to goal
    "cost": 14,                             # Total cost (unit cost: len(path) - 1)
    "expanded_nodes": [(0, 0), (1, 0), ...], # Nodes in chronological expansion order
    "reached_goal": True                    # Boolean flag
}
```

For unreachable goals:
```python
{
    "path": [],
    "cost": float("inf"),
    "expanded_nodes": [...],
    "reached_goal": False
}
```

For `start == goal`:
```python
{
    "path": [start],
    "cost": 0,
    "expanded_nodes": [start],
    "reached_goal": True
}
```

---

## 9. How to Run the Implementation and Tests

### Running the Automated Tests
From the project root directory:

Using Python's standard `unittest` module:
```bash
python3 -m unittest discover -s tests -v
```

Or using `pytest`:
```bash
python3 -m pytest tests -v
```

### Running the Demonstration Scenarios
To run Example A (reachable field) and Example B (unreachable field):
```bash
PYTHONPATH=src python3 -m cropsafe_pathplanning
```
or
```bash
PYTHONPATH=src python3 -m cropsafe_pathplanning.demo
```
