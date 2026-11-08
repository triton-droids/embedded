"""ROS-independent 8-connected A*: metres in, cell-centre path out.

Unknown cells are blocked. A circular footprint is conservatively inflated
against occupied cell squares and map boundaries. Rotated maps are rejected.
"""
import heapq
import math

import numpy as np


def plan(data, resolution, origin, start, goal, radius=0.25,
         occupied_threshold=50, max_expansions=100000):
    """Return [(x, y), ...], or [] for blocked/unreachable/budget exhaustion.

    Invalid map/arguments raise ValueError; endpoints resolve to cell centres.
    Cost is geometric path length; the Euclidean heuristic is admissible.
    """
    grid = np.asarray(data)
    values = (resolution, *origin, *start, *goal, radius)
    if (grid.ndim != 2 or not grid.size or grid.size > 250000
            or not np.isfinite(grid).all() or np.any(grid < -1) or np.any(grid > 100)
            or not all(math.isfinite(v) for v in values)
            or resolution <= 0 or radius < 0 or not 1 <= occupied_threshold <= 100
            or max_expansions < 1):
        raise ValueError('Invalid grid, geometry, threshold or search budget')
    height, width = grid.shape
    if radius * 2 > min(height, width) * resolution:
        return []
    blocked = (grid < 0) | (grid >= occupied_threshold)
    inflated = blocked.copy()
    # Include overlap with each occupied square, not only its centre.
    reach = min(math.ceil(radius / resolution + 0.5), max(height, width))
    for dy in range(-reach, reach + 1):
        for dx in range(-reach, reach + 1):
            distance = math.hypot(max(abs(dx)-0.5, 0), max(abs(dy)-0.5, 0))
            if distance > radius / resolution:
                continue
            y0, y1 = max(0, dy), min(height, height + dy)
            x0, x1 = max(0, dx), min(width, width + dx)
            if y1 > y0 and x1 > x0:
                inflated[y0:y1, x0:x1] |= blocked[y0-dy:y1-dy, x0-dx:x1-dx]
    margin = radius / resolution
    inflated |= ((np.arange(width)[None, :] + 0.5 < margin)
                 | (width - np.arange(width)[None, :] - 0.5 < margin)
                 | (np.arange(height)[:, None] + 0.5 < margin)
                 | (height - np.arange(height)[:, None] - 0.5 < margin))

    def cell(point):
        return (math.floor((point[0]-origin[0])/resolution),
                math.floor((point[1]-origin[1])/resolution))

    def free(p):
        x, y = p
        return 0 <= x < width and 0 <= y < height and not inflated[y, x]

    source, target = cell(start), cell(goal)
    if not free(source) or not free(target):
        return []
    costs, parent = {source: 0.0}, {}
    queue = [(math.dist(source, target), 0.0, source)]
    expanded = 0
    while queue and expanded < max_expansions:
        _, cost, current = heapq.heappop(queue)
        if cost != costs[current]:
            continue
        if current == target:
            route = [current]
            while route[-1] != source:
                route.append(parent[route[-1]])
            return [(origin[0]+(x+0.5)*resolution, origin[1]+(y+0.5)*resolution)
                    for x, y in reversed(route)]
        expanded += 1
        x, y = current
        for dx, dy in ((1,0), (-1,0), (0,1), (0,-1),
                       (1,1), (1,-1), (-1,1), (-1,-1)):
            nxt = x+dx, y+dy
            if not free(nxt):
                continue
            if dx and dy and (not free((x+dx,y)) or not free((x,y+dy))):
                continue
            candidate = cost + math.hypot(dx, dy)
            if candidate < costs.get(nxt, math.inf):
                costs[nxt], parent[nxt] = candidate, current
                heapq.heappush(queue, (candidate+math.dist(nxt,target), candidate, nxt))
    return []
