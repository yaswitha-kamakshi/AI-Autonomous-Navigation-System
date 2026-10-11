"""Planning helpers: plan from the car's position, detect blocked paths."""
import math
from src.planning.astar import search
from src.planning.smoothing import smooth


def nearest_free(grid, cell, max_r=8):
    """If `cell` is inside an obstacle margin, find the closest free cell."""
    if grid.is_free(*cell):
        return cell
    best, best_d = None, float("inf")
    for dr in range(-max_r, max_r + 1):
        for dc in range(-max_r, max_r + 1):
            r, c = cell[0] + dr, cell[1] + dc
            if grid.is_free(r, c) and math.hypot(dr, dc) < best_d:
                best, best_d = (r, c), math.hypot(dr, dc)
    return best


def plan_route(grid, start_xy, goal_xy):
    """Return a smoothed list of world (x, y) waypoints, or None if there is no route."""
    start = nearest_free(grid, grid.world_to_cell(*start_xy))
    goal = nearest_free(grid, grid.world_to_cell(*goal_xy))
    if start is None or goal is None:
        return None
    result = search(grid, start, goal)
    if result.path is None:
        return None
    cells = smooth(grid, result.path)
    return [grid.cell_to_world(r, c) for r, c in cells]


def path_blocked(grid, path, start_idx, car_xy, lookahead_px):
    """True if the next `lookahead_px` of the path now crosses an obstacle."""
    travelled, prev = 0.0, None
    for p in path[start_idx:]:
        if prev is not None:
            travelled += math.hypot(p[0] - prev[0], p[1] - prev[1])
        if travelled > lookahead_px:
            break
        prev = p
        if math.hypot(p[0] - car_xy[0], p[1] - car_xy[1]) < 22:
            continue   # ignore points right under the car
        if not grid.is_free(*grid.world_to_cell(*p)):
            return True
    return False