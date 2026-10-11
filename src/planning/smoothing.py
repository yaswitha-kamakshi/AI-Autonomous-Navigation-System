"""Path smoothing by line-of-sight shortcutting."""


def line_of_sight(grid, a, b):
    """True if the straight line between cells a and b stays in free space."""
    (r0, c0), (r1, c1) = a, b
    n = max(abs(r1 - r0), abs(c1 - c0)) * 2 + 1
    for i in range(n + 1):
        t = i / n
        r = int(round(r0 + (r1 - r0) * t))
        c = int(round(c0 + (c1 - c0) * t))
        if not grid.is_free(r, c):
            return False
    return True


def smooth(grid, path):
    """Remove unnecessary waypoints so the route is shorter and less jagged."""
    if path is None or len(path) < 3:
        return path
    out = [path[0]]
    i = 0
    while i < len(path) - 1:
        j = len(path) - 1
        while j > i + 1 and not line_of_sight(grid, path[i], path[j]):
            j -= 1
        out.append(path[j])
        i = j
    return out