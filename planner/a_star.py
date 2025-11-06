# planner/a_star.py
import heapq
import math

def grid_from_obstacles(W, H, cell, rects):
    cols, rows = W // cell, H // cell
    grid = [[0]*cols for _ in range(rows)]
    # inflate obstacles by 1 cell (safety)
    inflate = cell
    for r in rects:
        x0 = max(0, (r.left - inflate)//cell)
        y0 = max(0, (r.top  - inflate)//cell)
        x1 = min(cols-1, (r.right + inflate)//cell)
        y1 = min(rows-1, (r.bottom+ inflate)//cell)
        for gy in range(y0, y1+1):
            for gx in range(x0, x1+1):
                grid[gy][gx] = 1
    return grid, cols, rows

def _h(a, b):  # octile distance
    dx, dy = abs(a[0]-b[0]), abs(a[1]-b[1])
    return (dx+dy) + (math.sqrt(2)-2)*min(dx, dy)

def a_star(grid, start, goal):
    rows, cols = len(grid), len(grid[0])
    def nbrs(gx, gy):
        for dx, dy in [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(1,-1),(-1,1),(1,1)]:
            nx, ny = gx+dx, gy+dy
            if 0 <= nx < cols and 0 <= ny < rows and grid[ny][nx] == 0:
                yield nx, ny, (math.sqrt(2) if dx and dy else 1.0)
    openh = []
    g = {start: 0.0}
    came = {}
    heapq.heappush(openh, (0.0, start))
    seen = set()
    while openh:
        _, cur = heapq.heappop(openh)
        if cur in seen: 
            continue
        seen.add(cur)
        if cur == goal:
            # reconstruct
            path = [cur]
            while cur in came:
                cur = came[cur]
                path.append(cur)
            path.reverse()
            return path
        for nx, ny, w in nbrs(*cur):
            nc = (nx, ny)
            ng = g[cur] + w
            if ng < g.get(nc, 1e18):
                g[nc] = ng
                came[nc] = cur
                f = ng + _h(nc, goal)
                heapq.heappush(openh, (f, nc))
    return None

def grid_to_world(path_cells, cell):
    return [(gx*cell + cell/2.0, gy*cell + cell/2.0) for gx, gy in path_cells]
