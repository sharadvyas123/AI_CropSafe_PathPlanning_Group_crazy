"""Full-field watering coverage plans with no user-specified endpoints."""

from __future__ import annotations

import heapq
from collections import deque
from dataclasses import dataclass
from time import perf_counter

from .model import Field, Position


@dataclass
class CoveragePlan:
    algorithm: str
    route: list[Position]
    watered_cells: set[Position]
    irrigable_cells: set[Position]
    expanded_nodes: int
    elapsed_seconds: float

    @property
    def complete(self) -> bool:
        return self.watered_cells == self.irrigable_cells

    @property
    def coverage_percent(self) -> float:
        if not self.irrigable_cells:
            return 100.0
        return 100.0 * len(self.watered_cells) / len(self.irrigable_cells)

    @property
    def movement_steps(self) -> int:
        return max(0, len(self.route) - 1)


def _neighbors(field: Field, pos: Position):
    row, col = pos
    for nxt in ((row - 1, col), (row, col + 1), (row + 1, col), (row, col - 1)):
        if field.in_bounds(nxt) and nxt not in field.blocked:
            yield nxt


def _shortest_route(field: Field, start: Position, goal: Position) -> list[Position]:
    """Return a shortest legal connector between two irrigable cells."""
    parents: dict[Position, Position | None] = {start: None}
    queue = deque([start])
    while queue:
        current = queue.popleft()
        if current == goal:
            break
        for nxt in _neighbors(field, current):
            if nxt not in parents:
                parents[nxt] = current
                queue.append(nxt)
    if goal not in parents:
        raise ValueError("The irrigable field has disconnected regions; one continuous tour cannot cover all cells")
    path = []
    cursor: Position | None = goal
    while cursor is not None:
        path.append(cursor)
        cursor = parents[cursor]
    return list(reversed(path))


def _coverage_order(field: Field, algorithm: str, seed: Position) -> tuple[list[Position], int]:
    """Explore the irrigable-cell graph using each algorithm's frontier rule."""
    visited = {seed}
    order: list[Position] = []
    expanded = 0
    if algorithm in {"BFS", "DFS"}:
        frontier = deque([seed])
        while frontier:
            current = frontier.popleft() if algorithm == "BFS" else frontier.pop()
            order.append(current)
            expanded += 1
            neighbors = list(_neighbors(field, current))
            if algorithm == "DFS":
                neighbors.reverse()
            for nxt in neighbors:
                if nxt not in visited:
                    visited.add(nxt)
                    frontier.append(nxt)
        return order, expanded

    # GBFS prioritizes cells near the opposite corner for a sweeping coverage
    # wave. A* adds accumulated distance, balancing sweep progress and travel.
    target = (field.height - 1, field.width - 1)
    g_score = {seed: 0}
    heap: list[tuple[float, float, int, Position]] = []
    serial = 0
    h0 = abs(seed[0] - target[0]) + abs(seed[1] - target[1])
    heapq.heappush(heap, ((h0 if algorithm == "Greedy" else h0), h0, serial, seed))
    discovered = {seed}
    while heap:
        _, _, _, current = heapq.heappop(heap)
        order.append(current)
        expanded += 1
        for nxt in _neighbors(field, current):
            if nxt in discovered:
                continue
            discovered.add(nxt)
            g_score[nxt] = g_score[current] + 1
            h = abs(nxt[0] - target[0]) + abs(nxt[1] - target[1])
            priority = h if algorithm == "Greedy" else g_score[nxt] + h
            serial += 1
            heapq.heappush(heap, (priority, h, serial, nxt))
    return order, expanded


def plan_watering(field: Field, algorithm: str = "A*") -> CoveragePlan:
    """Create a continuous tour that waters every non-protected, non-obstacle cell.

    The first irrigable cell in row-major order is selected internally as the
    tour's launch cell. Callers provide no start or goal. A connected irrigable
    field is required for one continuous robot tour.
    """
    normalized = algorithm.strip().lower()
    names = {"bfs": "BFS", "dfs": "DFS", "greedy": "Greedy", "gbfs": "Greedy", "a*": "A*", "astar": "A*"}
    if normalized not in names:
        raise ValueError("algorithm must be BFS, DFS, Greedy, or A*")
    name = names[normalized]
    started = perf_counter()
    irrigable = field.irrigable
    if not irrigable:
        return CoveragePlan(name, [], set(), set(), 0, perf_counter() - started)
    seed = min(irrigable)
    order, expanded = _coverage_order(field, name, seed)
    if set(order) != irrigable:
        raise ValueError("The irrigable field has disconnected regions; one continuous tour cannot cover all cells")

    route = [seed]
    current = seed
    for target in order[1:]:
        connector = _shortest_route(field, current, target)
        route.extend(connector[1:])
        current = target
    elapsed = perf_counter() - started
    watered = set(route) & irrigable
    return CoveragePlan(name, route, watered, irrigable, expanded, elapsed)


def evaluate_coverage(field: Field) -> list[CoveragePlan]:
    """Run the four assignment algorithms as complete-field coverage planners."""
    return [plan_watering(field, name) for name in ("BFS", "DFS", "Greedy", "A*")]
