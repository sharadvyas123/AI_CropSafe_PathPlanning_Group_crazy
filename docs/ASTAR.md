# A* Search Algorithm and Crop-Safe Route Cost Model

> **Scope:** This is the standalone point-to-point A* exercise. The main project demo applies A* to full-field watering coverage and has no user-specified start or goal; see `docs/FIELD_ENVIRONMENT.md` and `src/cropsafe_pathplanning/coverage.py`.

> **Project:** AI-Based Crop-Safe Path Planning  
> **Module:** A* Search Algorithm & Agricultural Proximity Cost Modeling  
> **Author:** Ashutosh (Member 4)  
> **Target Python Version:** 3.10+  

---

## 1. Overview: What is A* Search?

**A\* Search** is an informed, best-first graph search algorithm that finds the least-cost path from a starting coordinate to a target goal coordinate on a weighted grid. It combines the rigorous cost-tracking of Dijkstra's algorithm with the goal-directed heuristic acceleration of Greedy Best-First Search.

At each step, A* selects and expands the node $n$ from the frontier that minimizes the evaluation function:

$$f(n) = g(n) + h(n)$$

Where:
- $g(n)$: The **exact accumulated cost** incurred along the path from the starting position to node $n$.
- $h(n)$: The **admissible heuristic estimate** of the remaining cost from node $n$ to the goal.
- $f(n)$: The **estimated total cost** of the cheapest solution passing through node $n$.

### Comparative Search Spectrum:

| Algorithm | Evaluation Function $f(n)$ | Optimality Guarantee | Characteristic Behavior |
| :--- | :--- | :--- | :--- |
| **Breadth-First Search (BFS)** | $f(n) = \text{depth}(n)$ | Step-optimal (unweighted only) | Uniform level-by-level expansion; blind to risk weights. |
| **Depth-First Search (DFS)** | None (LIFO stack) | Not optimal | Explores deep paths rapidly; frequently finds inefficient routes. |
| **Greedy Best-First Search** | $f(n) = h(n)$ | Not cost-optimal | Chases goal greedily; easily trapped in high-cost proximity zones. |
| **A\* Search** | $f(n) = g(n) + h(n)$ | **Cost-optimal** (with admissible $h$) | Balances distance traveled with distance remaining to minimize total path cost. |

---

## 2. Crop-Safe Route Cost Model

### 2.1 Agricultural Problem Context
In autonomous agricultural operations (e.g., spraying, harvesting, or soil sampling in potato fields), driving directly through or too close to crop beds causes:
1. **Direct Crop Destruction**: Crushing potato plants, damaging foliage, and breaking irrigation lines.
2. **Soil Compaction**: Compacting the root zone reduces tuber yields.
3. **Chemical / Mechanical Disturbance**: Operating heavy equipment immediately adjacent to sensitive plants increases wear and inadvertent contact.

To mitigate this, our model establishes:
- **Forbidden Zones**: Both permanent obstacles and protected crop rows are strictly impassable.
- **Safety Buffer Zone (Crop Clearance)**: Traversable soil cells 4-directionally adjacent to protected crop rows incur an explicit, configurable **risk penalty surcharge**.

### 2.2 Mathematical Formulation

Let the agricultural field be represented as a grid $V \subset \mathbb{Z}^2$. The cells are partitioned into:
- Traversable free cells: $V_{\text{free}}$
- Blocked obstacle cells: $V_{\text{obs}}$
- Protected crop cells: $V_{\text{crop}}$ (strictly impassable)

A cell $v = (r, c) \in V_{\text{free}}$ belongs to the **Crop Buffer Zone** if it is 4-directionally adjacent to any protected crop cell:

$$\text{is\_adjacent\_to\_protected}(v, V_{\text{crop}}) \iff \exists (dr, dc) \in \{(-1, 0), (1, 0), (0, -1), (0, 1)\} \text{ s.t. } (r + dr, c + dc) \in V_{\text{crop}}$$

The transition cost $c(u, v)$ for entering neighbor cell $v$ from cell $u$ is defined as:

$$c(u, v) = \begin{cases} \infty & \text{if } v \in V_{\text{obs}} \cup V_{\text{crop}} \\ c_{\text{base}} + \text{risk\_penalty} & \text{if } \text{is\_adjacent\_to\_protected}(v, V_{\text{crop}}) \\ c_{\text{base}} & \text{otherwise} \end{cases}$$

Where:
- $c_{\text{base}} = 1.0$: Baseline cost of traversing one soil grid unit.
- $\text{risk\_penalty} \ge 0.0$: Configurable penalty (default $2.0$) penalizing proximity to crops.

The total cost of a path $\pi = \langle v_0, v_1, \dots, v_k \rangle$ is:

$$C(\pi) = \sum_{i=1}^{k} c(v_{i-1}, v_i)$$

---

## 3. Heuristic Design: Admissibility and Consistency

For 4-directional cardinal movement on a 2D grid, the heuristic evaluation function is the **Manhattan Distance**:

$$h(n) = |r_n - r_{\text{goal}}| + |c_n - c_{\text{goal}}|$$

### 3.1 Proof of Admissibility ($h(n) \le h^*(n)$)
A heuristic is **admissible** if it never overestimates the true remaining cost $h^*(n)$ to reach the goal.

1. On a 4-connected grid, every cardinal step moves by at most 1 grid unit along either the row or column axis.
2. Therefore, the minimum number of steps required to reach the goal from $(r_n, c_n)$ in the absence of any obstacles is exactly $|r_n - r_{\text{goal}}| + |c_n - c_{\text{goal}}|$.
3. In our cost model, the transition cost of any valid step satisfies:
   $$c(u, v) \ge c_{\text{base}} = 1.0 \quad (\text{since } \text{risk\_penalty} \ge 0)$$
4. Any valid route to the goal requires at least $h(n)$ steps, each costing at least $1.0$:
   $$h^*(n) \ge \text{steps} \times c_{\text{base}} \ge h(n) \times 1.0 = h(n)$$
5. **Conclusion:** $h(n) \le h^*(n)$ holds for all nodes. The heuristic is strictly **admissible**.

### 3.2 Proof of Consistency / Monotonicity ($h(n) \le c(n, n') + h(n')$)
A heuristic is **consistent (monotonic)** if for every node $n$ and every neighbor $n'$ generated by an action, the estimated cost of reaching the goal from $n$ is no greater than the step cost of getting to $n'$ plus the estimated cost of reaching the goal from $n'$.

1. By the triangle inequality of the $L_1$ metric:
   $$|h(n) - h(n')| \le |r_n - r_{n'}| + |c_n - c_{n'}|$$
2. Since $n'$ is a 4-directional neighbor of $n$, $|r_n - r_{n'}| + |c_n - c_{n'}| = 1$. Thus:
   $$h(n) - h(n') \le 1$$
3. Since every step cost satisfies $c(n, n') \ge 1.0 + \text{penalty} \ge 1.0$:
   $$h(n) - h(n') \le 1 \le c(n, n') \implies h(n) \le c(n, n') + h(n')$$
4. **Conclusion:** $h(n)$ is strictly **consistent**.

### 3.3 Theoretical Guarantee
Because $h(n)$ is consistent:
- The first time A* expands any node $n$, the accumulated cost $g(n)$ is guaranteed to be optimal.
- No closed node ever needs to be reopened.
- A* is guaranteed to return the **cost-optimal path** without exhaustive state-space expansion.

---

## 4. Priority Queue and Deterministic Tie-Breaking

Our implementation uses Python's standard library `heapq` (binary min-heap).

```python
frontier: list[tuple[float, int, Coordinate]] = [(start_h, serial, start)]
```

### Deterministic Tie-Breaker (`serial`):
When two nodes have identical $f$-values ($f(n_1) = f(n_2)$):
- Comparing coordinates directly (`(r1, c1) < (r2, c2)`) introduces arbitrary spatial bias and tuple-comparison overhead.
- An auto-incrementing monotonic integer counter `serial` ensures **FIFO (First-In, First-Out)** expansion among equal-$f$ nodes, ensuring **100% reproducible search traces** across different runs and platforms.

---

## 5. Architectural Bridge: `ExpandedNodeList` & Dual Interfaces

To resolve the team interface mismatch identified in `TEAM_TASKS.md`:
- `model.Field` and the baseline evaluation runner expected an integer count (`expanded_total += result.expanded_nodes`).
- Teammate test suites (`test_bfs_dfs.py`, `test_gbfs.py`) expected `result.expanded_nodes` to be a `list` of `(row, col)` tuples.

We designed `ExpandedNodeList(list)` in `astar.py`:

```python
class ExpandedNodeList(list):
    def __int__(self) -> int: return len(self)
    def __add__(self, other: Any) -> Any:
        if isinstance(other, int): return len(self) + other
        return super().__add__(other)
    def __radd__(self, other: Any) -> Any:
        if isinstance(other, int): return other + len(self)
        return other + list(self)
    def __format__(self, format_spec: str) -> str:
        return format(len(self), format_spec)
```

**Benefits:**
- `isinstance(result.expanded_nodes, list)` evaluates to `True`.
- `expanded_total += result.expanded_nodes` increments an integer cleanly without error.
- `{result.expanded_nodes:>10}` prints the integer count in output comparison tables.

---

## 6. Case Study: Safety Penalty Tradeoff and Suboptimality of GBFS

### 6.1 Safety Penalty Divergence Test
In `tests/test_astar.py` (`test_safety_penalty_changes_route`):
- Grid: $5 \times 7$, Start: `(2, 0)`, Goal: `(2, 6)`, Protected crop: `(2, 3)`.
- When $\text{risk\_penalty} = 0$:
  - A* takes the direct path skirting immediately above/below the crop.
  - Path length: **8 steps**, Total cost: **8.0**.
- When $\text{risk\_penalty} = 3.0$:
  - Skirting the crop would cost $1 + 1 + (1+3) + (1+3) + \dots > 10$.
  - A* routes wide around the crop clearance zone.
  - Path length: **10 steps**, Total cost: **10.0**.
  - A* willingly accepts 2 extra spatial steps to achieve a lower overall cost.

### 6.2 A* Cost-Optimality vs Greedy Best-First Search
In `tests/test_astar.py` (`test_astar_cost_optimality_vs_gbfs`):
- Protected crop row placed between start and goal with high risk penalty ($\text{risk\_penalty} = 5.0$).
- **GBFS ($f = h$)**: Ignorant of step costs, GBFS rushes straight into the crop buffer cells because they minimize Manhattan distance. Resulting path cost exceeds **25.0**.
- **A\* ($f = g + h$)**: Correctly evaluates the accumulated $g(n)$ risk penalties and chooses the safe detour around the buffer. Resulting path cost: **7.0**.

---

## 7. Verification and Acceptance Evidence

The test suite in [`tests/test_astar.py`](../tests/test_astar.py) covers the A* behavior and public API:

| Test Method | Description | Status |
| :--- | :--- | :--- |
| `test_astar_reaches_goal` | Validates path start, goal, and step length on an open field. | Pass |
| `test_astar_avoids_blocked_cells` | Verifies path is strictly disjoint from obstacles and crops. | Pass |
| `test_safety_penalty_changes_route` | Demonstrates wider safe route selection when penalty increases. | Pass |
| `test_astar_reports_no_path` | Confirms empty path and infinite cost for walled-off goals. | Pass |
| `test_astar_path_cost` | Verifies path cost matches the sum of step transition costs. | Pass |
| `test_search_result_shared_dict_contract` | Validates shared dictionary interface and property accessors. | Pass |
| `test_expanded_nodes_dual_nature` | Confirms dual collection/integer arithmetic behavior of `ExpandedNodeList`. | Pass |
| `test_start_equals_goal` | Verifies 1-node zero-cost path when start and goal coincide. | Pass |
| `test_invalid_endpoints_graceful_handling` | Confirms graceful failure when `raise_on_invalid=False`. | Pass |
| `test_invalid_endpoints_raise_exceptions` | Confirms ValueError/TypeError when `raise_on_invalid=True`. | Pass |
| `test_reproducibility` | Confirms identical paths and node expansions across runs. | Pass |
| `test_no_field_mutation` | Verifies field blocked cells are never mutated during search. | Pass |
| `test_path_traceability_and_bounds` | Ensures every step is in-bounds and a valid 4-directional move. | Pass |
| `test_crop_safe_cost_functions_standalone` | Tests `crop_safe_step_cost` and `make_crop_safe_cost_fn`. | Pass |
| `test_astar_cost_optimality_vs_gbfs` | Contrasts A* cost-optimality with GBFS proximity trap. | Pass |
| `test_interoperability_fieldmap_and_field` | Confirms identical optimal costs on `Field` and `FieldMap`. | Pass |
| `test_crop_safe_cost_rejects_invalid_penalty_and_base` | Rejects values that invalidate the Manhattan cost assumptions. | Pass |

Run the test suite using:
```bash
python -m unittest tests/test_astar.py -v
```
