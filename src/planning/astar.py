"""A* and Dijkstra on the inflated occupancy grid (8-connected)."""
import heapq
import itertools
import math
import time
from dataclasses import dataclass

SQRT2 = math.sqrt(2)
MOVES = [(-1, 0, 1), (1, 0, 1), (0, -1, 1), (0, 1, 1),
         (-1, -1, SQRT2), (-1, 1, SQRT2), (1, -1, SQRT2), (1, 1, SQRT2)]


@dataclass
class PlanResult:
    path: list        # list of (row, col), or None if no path exists
    cost: float       # path cost in grid cells
    expanded: int     # nodes expanded (lower = more efficient)
    time_ms: float


def search(grid, start, goal, use_heuristic=True):
    """A* when use_heuristic=True, Dijkstra when False."""
    t0 = time.perf_counter()
    if not grid.is_free(*start) or not grid.is_free(*goal):
        return PlanResult(None, math.inf, 0, 0.0)

    def h(cell):
        if not use_heuristic:
            return 0.0
        return math.hypot(cell[0] - goal[0], cell[1] - goal[1])  # Euclidean, admissible

    counter = itertools.count()               # tie-breaker for the heap
    open_heap = [(h(start), next(counter), 0.0, start)]
    best_g = {start: 0.0}
    came_from = {}
    expanded = 0

    while open_heap:
        _, _, g, cur = heapq.heappop(open_heap)
        if g > best_g.get(cur, math.inf):
            continue                          # stale heap entry
        expanded += 1
        if cur == goal:
            path = [cur]
            while cur in came_from:
                cur = came_from[cur]
                path.append(cur)
            path.reverse()
            return PlanResult(path, g, expanded, (time.perf_counter() - t0) * 1000)

        for dr, dc, step in MOVES:
            nr, nc = cur[0] + dr, cur[1] + dc
            if not grid.is_free(nr, nc):
                continue
            # no cutting corners through blocked cells on diagonal moves
            if dr != 0 and dc != 0 and not (grid.is_free(cur[0] + dr, cur[1]) and
                                            grid.is_free(cur[0], cur[1] + dc)):
                continue
            ng = g + step
            if ng < best_g.get((nr, nc), math.inf):
                best_g[(nr, nc)] = ng
                came_from[(nr, nc)] = cur
                heapq.heappush(open_heap, (ng + h((nr, nc)), next(counter), ng, (nr, nc)))

    return PlanResult(None, math.inf, expanded, (time.perf_counter() - t0) * 1000)