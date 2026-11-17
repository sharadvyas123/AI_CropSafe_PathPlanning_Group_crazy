
"""CropSafe field watering: full-area coverage planning for agricultural robotics.

The main demo applies BFS, DFS, Greedy Best-First, and A* to cover all irrigable
field cells without entering protected or obstacle cells. Standalone point-to-
point search APIs are also retained for the assignment's algorithm exercises.
"""

from cropsafe_pathplanning.gbfs import (
    greedy_best_first_search,
    validate_endpoints,
)
from cropsafe_pathplanning.heuristic import manhattan_distance
from cropsafe_pathplanning.types import (
    CARDINAL_DIRECTIONS,
    Coordinate,
    FieldMap,
    SearchResult,
)

__version__ = "0.1.0"

__all__ = [
    "greedy_best_first_search",
    "validate_endpoints",
    "manhattan_distance",
    "FieldMap",
    "SearchResult",
    "Coordinate",
    "CARDINAL_DIRECTIONS",
]

from cropsafe_pathplanning.bfs_dfs import (
    breadth_first_search,
    depth_first_search,
)

__all__ += [
    "breadth_first_search",
    "depth_first_search",
]

from cropsafe_pathplanning.astar import (
    a_star_search,
    crop_safe_step_cost,
    make_crop_safe_cost_fn,
)

__all__ += [
    "a_star_search",
    "crop_safe_step_cost",
    "make_crop_safe_cost_fn",
]
