"""Render animated GIFs of the three planners in this repo on a shared test map.

The search logic below is a line-by-line Python port of the C++ nodes
(a_star_planner, hybrid_a_star_cpp, rrt_planner), using the same parameters,
neighbour/primitive ordering, collision checks and termination conditions.
Only the map and the start/goal poses are made up.

    pip install numpy matplotlib pillow
    python docs/visualize_planners.py
"""

import heapq
import itertools
import math
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.animation import FuncAnimation, PillowWriter  # noqa: E402
from matplotlib.collections import LineCollection  # noqa: E402
from matplotlib.patches import Polygon  # noqa: E402

OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "images")

# ---------------------------------------------------------------- test map --
RES = 0.05            # m / cell
W, H = 200, 120       # 10 m x 6 m
START = (1.2, 1.2, math.pi / 2)   # x, y, yaw (facing up the left corridor)
GOAL = (8.8, 4.8)

COLORS = {
    "free": "#ffffff",
    "wall": "#2b2f36",
    "closed": "#b9d4f0",
    "open": "#f4b26a",
    "expanded": "#e0565b",
    "tree": "#3a9a5b",
    "sample": "#e0565b",
    "path": "#1f5fbf",
    "reverse": "#c23a8c",
    "start": "#2e9e4f",
    "goal": "#d64545",
}


def build_map():
    grid = np.zeros((H, W), dtype=np.int8)

    def block(x0, y0, x1, y1):
        grid[int(y0 / RES):int(y1 / RES), int(x0 / RES):int(x1 / RES)] = 100

    block(0, 0, 10, 0.1)       # borders
    block(0, 5.9, 10, 6)
    block(0, 0, 0.1, 6)
    block(9.9, 0, 10, 6)
    block(3.2, 0, 3.4, 3.6)    # wall from the bottom
    block(6.5, 2.4, 6.7, 6)    # wall from the top
    block(4.6, 3.9, 5.3, 4.5)  # small pillar between the walls
    return grid


def world_to_grid(x, y):
    return int(x / RES), int(y / RES)


def grid_to_world(gx, gy):
    return gx * RES, gy * RES


# -------------------------------------------------------------------- A* --
def a_star(grid, start, goal, allow_diagonal=True):
    """Port of AStarPlanner::aStar. Returns (path, per-expansion event log)."""
    def heuristic(a, b):  # "euclidean" (the default heuristic_type)
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def is_valid(x, y):
        return 0 <= x < W and 0 <= y < H and grid[y, x] == 0

    counter = itertools.count()
    best_g = {start: 0.0}
    parent = {start: None}
    closed = np.zeros((H, W), dtype=bool)
    open_list = [(heuristic(start, goal), next(counter), 0.0, start)]
    events = []  # (closed_cell, [newly opened/improved cells])

    while open_list:
        _, _, g, cur = heapq.heappop(open_list)
        cx, cy = cur
        if closed[cy, cx]:
            continue
        closed[cy, cx] = True
        opened = []

        if cur == goal:
            events.append((cur, opened))
            path = []
            while cur is not None:
                path.append(grid_to_world(*cur))
                cur = parent[cur]
            return path[::-1], events

        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                if not allow_diagonal and abs(dx) + abs(dy) > 1:
                    continue
                nx, ny = cx + dx, cy + dy
                if not is_valid(nx, ny) or closed[ny, nx]:
                    continue
                new_g = g + (1.0 if dx == 0 or dy == 0 else 1.414)
                if (nx, ny) not in best_g or new_g < best_g[(nx, ny)]:
                    best_g[(nx, ny)] = new_g
                    parent[(nx, ny)] = cur
                    heapq.heappush(open_list, (new_g + heuristic((nx, ny), goal),
                                               next(counter), new_g, (nx, ny)))
                    opened.append((nx, ny))
        events.append((cur, opened))
    return [], events


# ------------------------------------------------------------ Hybrid A* --
MAX_STEER = 0.34
STEP = 0.1
WHEELBASE = 0.3302
CAR_WIDTH = 0.2032
HA_GOAL_TOL = 0.2


def footprint(x, y, th):
    hw = CAR_WIDTH / 2
    pts = [(0, -hw), (0, hw), (WHEELBASE, hw), (WHEELBASE, -hw)]
    return [(x + px * math.cos(th) - py * math.sin(th),
             y + px * math.sin(th) + py * math.cos(th)) for px, py in pts]


def hybrid_a_star(grid, start, goal):
    """Port of HybridAStarPlanner::hybridAStar."""
    def is_collision(x, y, th):
        for px, py in footprint(x, y, th):
            mx, my = int(px / RES), int(py / RES)
            if mx < 0 or my < 0 or mx >= W or my >= H:
                return True
            if grid[my, mx] > 50 or grid[my, mx] == -1:
                return True
        return False

    def key(x, y, th):  # getKey: 0.1 m / 0.1 rad bins, truncated like static_cast<int>
        return (int(x * 10), int(y * 10), int(th * 10))

    def h(x, y):
        return math.hypot(x - goal[0], y - goal[1])

    counter = itertools.count()
    # node = (x, y, theta, g, parent_node, direction)
    root = (start[0], start[1], start[2], 0.0, None, 1.0)
    open_list = [(h(start[0], start[1]), next(counter), root)]
    closed = {}
    expanded = [(start[0], start[1])]   # mirrors expanded_nodes_
    frames_at = []                      # len(expanded) after each pop

    while open_list:
        f, _, node = heapq.heappop(open_list)
        x, y, th, g, _, _ = node
        frames_at.append(len(expanded))

        k = key(x, y, th)
        if k in closed and closed[k] <= f:
            continue
        closed[k] = f

        if math.hypot(x - goal[0], y - goal[1]) < HA_GOAL_TOL:
            path = []
            while node is not None:
                path.append(node)
                node = node[4]
            return path[::-1], expanded, frames_at

        for steer in (-MAX_STEER, 0.0, MAX_STEER):
            for d in (1.0, -1.0):
                nth = th + (d * STEP / WHEELBASE) * math.tan(steer)
                nx = x + d * STEP * math.cos(th)
                ny = y + d * STEP * math.sin(th)
                if is_collision(nx, ny, nth):
                    continue
                ng = g + STEP
                nk = key(nx, ny, nth)
                if nk not in closed or ng < closed[nk]:
                    heapq.heappush(open_list, (ng + h(nx, ny), next(counter),
                                               (nx, ny, nth, ng, node, d)))
                    expanded.append((nx, ny))
    return [], expanded, frames_at


# ------------------------------------------------------------------- RRT --
RRT_STEP = 0.1
RRT_TOL = 0.15
RRT_MAX_ITER = 5000
RRT_BIAS = 0.65


def bresenham(x0, y0, x1, y1):
    pts = []
    dx, sx = abs(x1 - x0), (1 if x0 < x1 else -1)
    dy, sy = -abs(y1 - y0), (1 if y0 < y1 else -1)
    err = dx + dy
    while True:
        pts.append((x0, y0))
        if x0 == x1 and y0 == y1:
            break
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            x0 += sx
        if e2 <= dx:
            err += dx
            y0 += sy
    return pts


def rrt(grid, start, goal, seed):
    """Port of RRTPlanner::plan."""
    rng = np.random.default_rng(seed)

    def collides(a, b):
        for cx, cy in bresenham(*world_to_grid(*a), *world_to_grid(*b)):
            if cx < 0 or cx >= W or cy < 0 or cy >= H or grid[cy, cx] > 50:
                return True
        return False

    nodes = [start]
    parents = [-1]
    log = []  # (sample, nearest_idx, new_idx or None)

    for _ in range(RRT_MAX_ITER):
        if rng.integers(100) < RRT_BIAS * 100:
            sample = goal
        else:
            sample = (rng.random() * W * RES, rng.random() * H * RES)

        pts = np.asarray(nodes)
        nearest = int(np.argmin(np.hypot(pts[:, 0] - sample[0], pts[:, 1] - sample[1])))
        nx, ny = nodes[nearest]
        dx, dy = sample[0] - nx, sample[1] - ny
        dist = math.hypot(dx, dy)
        new = sample if dist <= RRT_STEP else (nx + dx / dist * RRT_STEP, ny + dy / dist * RRT_STEP)

        if collides(nodes[nearest], new):
            log.append((sample, nearest, None))
            continue
        nodes.append(new)
        parents.append(nearest)
        log.append((sample, nearest, len(nodes) - 1))

        if math.hypot(new[0] - goal[0], new[1] - goal[1]) <= RRT_TOL:
            path, i = [], len(nodes) - 1
            while i != -1:
                path.append(nodes[i])
                i = parents[i]
            return path[::-1], nodes, parents, log
    return [], nodes, parents, log


# ------------------------------------------------------------- rendering --
def hex_rgb(c):
    c = c.lstrip("#")
    return np.array([int(c[i:i + 2], 16) for i in (0, 2, 4)], dtype=np.uint8)


def base_axes(grid, title, subtitle):
    fig, ax = plt.subplots(figsize=(8, 5.2), dpi=80)
    fig.subplots_adjust(left=0.02, right=0.98, bottom=0.03, top=0.86)
    rgb = np.where(grid[..., None] > 50, hex_rgb(COLORS["wall"]), hex_rgb(COLORS["free"]))
    img = ax.imshow(rgb, origin="lower", extent=[0, W * RES, 0, H * RES],
                    interpolation="nearest")
    ax.set_xticks([])
    ax.set_yticks([])
    for s in ax.spines.values():
        s.set_visible(False)
    fig.text(0.02, 0.945, title, fontsize=15, weight="bold", color="#1d2128")
    sub = fig.text(0.02, 0.895, subtitle, fontsize=10.5, color="#555b66")
    ax.plot(*START[:2], "o", ms=9, color=COLORS["start"], zorder=6)
    ax.plot(*GOAL, "*", ms=15, color=COLORS["goal"], zorder=6)
    return fig, ax, img, rgb, sub


def frame_indices(total, n_frames):
    if total <= n_frames:
        return list(range(1, total + 1))
    return sorted({max(1, round(total * (i / n_frames) ** 1.0)) for i in range(1, n_frames + 1)})


def save(anim, name, fps=15):
    os.makedirs(OUT_DIR, exist_ok=True)
    path = os.path.join(OUT_DIR, name)
    anim.save(path, writer=PillowWriter(fps=fps))
    print(f"wrote {path} ({os.path.getsize(path) / 1e3:.0f} kB)")


def render_a_star(grid):
    start = world_to_grid(*START[:2])
    goal = world_to_grid(*GOAL)
    path, events = a_star(grid, start, goal)
    print(f"A*: {len(events)} expansions, path {len(path)} cells")

    fig, ax, img, rgb, sub = base_axes(
        grid, "A*  (a_star_planner)",
        "8-connected grid search, f = g + Euclidean h.  "
        "orange = open list, blue = closed list")
    line, = ax.plot([], [], color=COLORS["path"], lw=3, zorder=5)
    state = rgb.copy()
    closed_c, open_c = hex_rgb(COLORS["closed"]), hex_rgb(COLORS["open"])
    stops = frame_indices(len(events), 70)
    hold = 25
    progress = {"i": 0}

    def update(fi):
        if fi < len(stops):
            upto = stops[fi]
            for cell, opened in events[progress["i"]:upto]:
                for ox, oy in opened:
                    state[oy, ox] = open_c
                state[cell[1], cell[0]] = closed_c
            progress["i"] = upto
            img.set_data(state)
            sub.set_text(f"8-connected grid search, f = g + Euclidean h.  "
                         f"expanded {upto} / {len(events)} cells")
        else:
            xs, ys = zip(*[(x + RES / 2, y + RES / 2) for x, y in path])
            line.set_data(xs, ys)
            sub.set_text(f"Path found: {len(path)} cells after {len(events)} expansions "
                         f"(orange = open, blue = closed)")
        return img, line, sub

    save(FuncAnimation(fig, update, frames=len(stops) + hold, blit=False), "a_star.gif")
    plt.close(fig)
    return path


def car_patch(x, y, th, color):
    return Polygon(footprint(x, y, th), closed=True, fc=color, ec="#1d2128",
                   lw=0.6, alpha=0.85, zorder=7)


def render_hybrid(grid):
    path, expanded, frames_at = hybrid_a_star(grid, START, GOAL)
    print(f"Hybrid A*: {len(frames_at)} pops, {len(expanded)} pushed states, "
          f"path {len(path)} poses")

    fig, ax, img, rgb, sub = base_axes(
        grid, "Hybrid A*  (hybrid_a_star_cpp)",
        "Searches (x, y, θ) with 6 bicycle-model primitives: steer {-0.34, 0, +0.34} × {fwd, rev}")
    pts = np.asarray(expanded)
    scat = ax.scatter([], [], s=3, c=COLORS["expanded"], alpha=0.35, lw=0, zorder=3)
    segs = LineCollection([], lw=3, zorder=5)
    ax.add_collection(segs)
    stops = frame_indices(len(frames_at), 70)
    hold = 30

    def update(fi):
        if fi < len(stops):
            n = frames_at[stops[fi] - 1]
            scat.set_offsets(pts[:n])
            sub.set_text(f"Searches (x, y, θ) with 6 bicycle-model primitives.  "
                         f"{n} states pushed so far")
        elif fi == len(stops):
            lines, colors = [], []
            for a, b in zip(path, path[1:]):
                lines.append([(a[0], a[1]), (b[0], b[1])])
                colors.append(COLORS["path"] if b[5] > 0 else COLORS["reverse"])
            segs.set_segments(lines)
            segs.set_color(colors)
            for node in path[::12] + [path[-1]]:
                ax.add_patch(car_patch(node[0], node[1], node[2], "#f2f4f7"))
            n_rev = sum(1 for n in path[1:] if n[5] < 0)
            note = f", {n_rev} in reverse (pink)" if n_rev else ""
            sub.set_text(f"Path found: {len(path)} poses of 0.1 m{note}. "
                         f"Rectangles = car footprint used for collision checks")
        return scat, segs, sub

    save(FuncAnimation(fig, update, frames=len(stops) + hold, blit=False), "hybrid_a_star.gif")
    plt.close(fig)
    return path


def render_rrt(grid, seed=7):
    path, nodes, parents, log = rrt(grid, START[:2], GOAL, seed)
    print(f"RRT (seed {seed}): {len(log)} iterations, {len(nodes)} nodes, "
          f"path {len(path)} nodes")

    fig, ax, img, rgb, sub = base_axes(
        grid, "RRT  (rrt_planner)",
        "Sample (65% goal-biased), extend nearest node 0.1 m, keep it if the edge is collision-free")
    edges = LineCollection([], lw=1.2, color=COLORS["tree"], zorder=3)
    ax.add_collection(edges)
    samples = ax.scatter([], [], s=6, c=COLORS["sample"], alpha=0.35, lw=0, zorder=2)
    probe, = ax.plot([], [], color="#888", lw=0.8, ls="--", zorder=4)
    line, = ax.plot([], [], color=COLORS["path"], lw=3, zorder=5)
    stops = frame_indices(len(log), 80)
    hold = 25
    rand_samples = np.asarray([s for s, _, _ in log if s != GOAL] or [(np.nan, np.nan)])
    rand_upto = np.cumsum([s != GOAL for s, _, _ in log])

    def update(fi):
        if fi < len(stops):
            it = stops[fi]
            segs = [[nodes[parents[n]], nodes[n]] for _, _, n in log[:it] if n is not None]
            edges.set_segments(segs)
            samples.set_offsets(rand_samples[:rand_upto[it - 1]])
            s, near, _ = log[it - 1]
            probe.set_data([nodes[near][0], s[0]], [nodes[near][1], s[1]])
            rejected = sum(1 for _, _, n in log[:it] if n is None)
            sub.set_text(f"iteration {it}: {len(segs) + 1} nodes, {rejected} extensions "
                         f"rejected by collision check.  red dots = random samples")
        else:
            probe.set_data([], [])
            xs, ys = zip(*path) if path else ([], [])
            line.set_data(xs, ys)
            sub.set_text(f"Path found after {len(log)} iterations "
                         f"({len(nodes)} nodes). Not smoothed or optimised."
                         if path else f"No path within {RRT_MAX_ITER} iterations")
        return edges, samples, probe, line, sub

    save(FuncAnimation(fig, update, frames=len(stops) + hold, blit=False), "rrt.gif")
    plt.close(fig)
    return path


def render_comparison(grid, a_path, h_path, r_path):
    fig, ax, *_ = base_axes(grid, "Same map, three planners", "")
    fig.texts[-1].remove()
    fig.subplots_adjust(top=0.9)
    off = RES / 2
    ax.plot([p[0] + off for p in a_path], [p[1] + off for p in a_path],
            lw=2.5, color=COLORS["path"], label="A* (grid, 8-connected)")
    ax.plot([p[0] for p in h_path], [p[1] for p in h_path],
            lw=2.5, color="#d9822b", label="Hybrid A* (kinematically feasible)")
    ax.plot([p[0] for p in r_path], [p[1] for p in r_path],
            lw=2.0, color=COLORS["tree"], label="RRT (random, unsmoothed)")
    ax.legend(loc="lower right", frameon=True, fontsize=9.5)
    os.makedirs(OUT_DIR, exist_ok=True)
    out = os.path.join(OUT_DIR, "comparison.png")
    fig.savefig(out)
    plt.close(fig)
    print(f"wrote {out}")


if __name__ == "__main__":
    g = build_map()
    a = render_a_star(g)
    hy = render_hybrid(g)
    r = render_rrt(g)
    render_comparison(g, a, hy, r)
