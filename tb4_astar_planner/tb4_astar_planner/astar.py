import heapq
from math import sqrt

def is_free_occ(occ: int, occ_thresh: int = 50, unknown_is_free: bool = False) -> bool:
    # OccupancyGrid: -1 unknown, 0 free, 100 occupied
    if occ < 0:
        return unknown_is_free
    return occ < occ_thresh

def neighbors(r, c, H, W, allow_diag=True):
    if allow_diag:
        steps = [(-1,0),(1,0),(0,-1),(0,1), (-1,-1),(-1,1),(1,-1),(1,1)]
    else:
        steps = [(-1,0),(1,0),(0,-1),(0,1)]
    for dr, dc in steps:
        nr = r + dr
        nc = c + dc
        if 0 <= nr < H and 0 <= nc < W:
            yield nr, nc

def heuristic(a, b, allow_diag=True):
    (r1, c1) = a
    (r2, c2) = b
    dr = abs(r1 - r2)
    dc = abs(c1 - c2)
    if not allow_diag:
        return dr + dc
    D, D2 = 1.0, sqrt(2.0)
    return D * (dr + dc) + (D2 - 2 * D) * min(dr, dc)

def move_cost(a, b):
    (r1, c1) = a
    (r2, c2) = b
    return sqrt(2.0) if (r1 != r2 and c1 != c2) else 1.0

def astar_occ_grid(occ_grid_2d, start_rc, goal_rc, allow_diag=True, occ_thresh=50, unknown_is_free=False):
    H, W = occ_grid_2d.shape

    sr, sc = start_rc
    gr, gc = goal_rc

    if not (0 <= sr < H and 0 <= sc < W):
        raise ValueError("Start outside map bounds")
    if not (0 <= gr < H and 0 <= gc < W):
        raise ValueError("Goal outside map bounds")

    if not is_free_occ(int(occ_grid_2d[sr, sc]), occ_thresh, unknown_is_free):
        raise ValueError(f"Start not free: occ={occ_grid_2d[sr, sc]}")
    if not is_free_occ(int(occ_grid_2d[gr, gc]), occ_thresh, unknown_is_free):
        raise ValueError(f"Goal not free: occ={occ_grid_2d[gr, gc]}")

    open_heap = []
    heapq.heappush(open_heap, (0.0, (sr, sc)))

    came_from = {}
    g = {(sr, sc): 0.0}
    f0 = heuristic((sr, sc), (gr, gc), allow_diag)
    f = {(sr, sc): f0}

    closed = set()

    while open_heap:
        _, current = heapq.heappop(open_heap)
        if current in closed:
            continue
        closed.add(current)

        if current == (gr, gc):
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path.reverse()
            return path

        cr, cc = current
        for nr, nc in neighbors(cr, cc, H, W, allow_diag):
            if not is_free_occ(int(occ_grid_2d[nr, nc]), occ_thresh, unknown_is_free):
                continue

            nxt = (nr, nc)
            tentative_g = g[current] + move_cost(current, nxt)

            if nxt not in g or tentative_g < g[nxt]:
                came_from[nxt] = current
                g[nxt] = tentative_g
                f[nxt] = tentative_g + heuristic(nxt, (gr, gc), allow_diag)
                heapq.heappush(open_heap, (f[nxt], nxt))

    return None
