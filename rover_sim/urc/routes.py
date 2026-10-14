"""Route analysis on a Heightfield: the easiest drivable route between two
points, how steep a hill is from each side, and the rim of a hilltop.

Used for the Autonomy world's ground truth (rule 1.e.xv: "not every approach
to the target is navigable"): the judges' easy route to a hilltop post, the
table of approach grades around it and the rim its boulders line.
"""
import heapq
import math

import cv2
import numpy as np

from . import terrain

GRADE_BASE = 4.0  # [m] grades are measured over this span (a few rover lengths): metre-scale DEM noise is not a slope
CROSS_SLOPE_MARGIN = 8.0  # [deg] a route may cross a side slope this much steeper than the grade it climbs
NEIGHBOURS = [(dr, dc) for dr in (-1, 0, 1) for dc in (-1, 0, 1) if (dr, dc) != (0, 0)]


def easy_route(hf, start, goal, max_slope, res=2.0, margin=250.0):
    """The least-effort route from start to goal (layout x, y) that never climbs
    or descends steeper than max_slope [deg].

    Dijkstra over 8 neighbours on a `res`-metre grid of the smoothed terrain,
    searched within `margin` metres of the start-goal box. A step is allowed if
    its grade is at most max_slope and the ground under it at most
    CROSS_SLOPE_MARGIN steeper; it costs its length times (1 + grade /
    max_slope), so gentle detours beat steep shortcuts. Returns (polyline,
    max grade along it [deg] over GRADE_BASE on the full terrain), or None if
    there is no such route.
    """
    k = max(1, int(round(res / hf.res)))
    step = k * hf.res
    z = cv2.GaussianBlur(hf.z, (0, 0), k / 2)[::k, ::k]  # the rover feels the terrain at about this scale
    X, Y = (a[::k, ::k] for a in hf.grid())
    cells = [_cell(X, Y, p) for p in (start, goal)]
    (r0, c0), (r1, c1) = cells
    pad = int(math.ceil(margin / step))
    rows = slice(max(min(r0, r1) - pad, 0), min(max(r0, r1) + pad + 1, z.shape[0]))
    cols = slice(max(min(c0, c1) - pad, 0), min(max(c0, c1) + pad + 1, z.shape[1]))
    z, X, Y = z[rows, cols], X[rows, cols], Y[rows, cols]
    gy, gx = np.gradient(z, step)
    ground_ok = np.degrees(np.arctan(np.hypot(gx, gy))) <= max_slope + CROSS_SLOPE_MARGIN
    h, w = z.shape
    edges = []  # per neighbour: flat-index offset and step cost (inf where not allowed)
    for dr, dc in NEIGHBOURS:
        length = step * math.hypot(dr, dc)
        grade = np.degrees(np.arctan(np.abs(_neighbour(z, dr, dc, np.nan) - z) / length))
        ok = (grade <= max_slope) & ground_ok & _neighbour(ground_ok, dr, dc, False)  # NaN compares False
        cost = np.where(ok, length * (1 + grade / max_slope), np.inf)
        edges.append((dr * w + dc, cost.ravel().tolist()))
    source = (r0 - rows.start) * w + (c0 - cols.start)
    target = (r1 - rows.start) * w + (c1 - cols.start)
    dist = [math.inf] * (h * w)
    prev = [-1] * (h * w)
    dist[source] = 0.0
    queue = [(0.0, source)]
    while queue:
        d, i = heapq.heappop(queue)
        if i == target:
            break
        if d > dist[i]:
            continue
        for offset, cost in edges:
            if cost[i] == math.inf:  # also every step off the window
                continue
            nd, j = d + cost[i], i + offset
            if nd < dist[j]:
                dist[j] = nd
                prev[j] = i
                heapq.heappush(queue, (nd, j))
    if math.isinf(dist[target]):
        return None
    chain = [target]
    while chain[-1] != source:
        chain.append(prev[chain[-1]])
    pts = np.array([(X.flat[i], Y.flat[i]) for i in reversed(chain)])
    pts[0], pts[-1] = start, goal
    # Straight runs of grid steps become one segment (within a quarter cell).
    keep = cv2.approxPolyDP(pts.astype(np.float32).reshape(-1, 1, 2), step / 4, False).reshape(-1, 2)
    path = [tuple(map(float, p)) for p in keep]
    return path, hf.max_grade_along(path, base=GRADE_BASE)


def approach_grades(hf, summit, bearings, r0, r1, base=GRADE_BASE):
    """Steepest grade [deg] met driving straight up to `summit` (layout x, y)
    from each compass bearing [deg, 0 = north, 90 = east], between r0 and r1
    metres out."""
    out = []
    for bearing in bearings:
        dx, dy = math.sin(math.radians(bearing)), math.cos(math.radians(bearing))
        line = [(summit[0] + r1 * dx, summit[1] + r1 * dy), (summit[0] + r0 * dx, summit[1] + r0 * dy)]
        out.append(hf.max_grade_along(line, base=base))
    return out


def rim(hf, summit, max_slope, keep=None, keep_width=0.0):
    """The rim of the hilltop at `summit` (layout x, y): a closed layout
    polyline round the ground reached from it on slopes up to max_slope
    [deg] (over GRADE_BASE), widened to keep_width either side of the path
    `keep` from where it first reaches that ground (an easy route along a
    narrow crest). Returns (polyline, that first point of `keep`, or None)."""
    z = cv2.GaussianBlur(hf.z, (0, 0), GRADE_BASE / 2 / hf.res)
    gy, gx = np.gradient(z, hf.res)
    gentle = (np.degrees(np.arctan(np.hypot(gx, gy))) <= max_slope).astype(np.uint8)
    west, north = hf.center[0] - hf.size / 2, hf.center[1] + hf.size / 2

    def cell(p):
        """Fractional (column, row) of layout points p (..., 2)."""
        p = np.asarray(p, float)
        return np.stack([(p[..., 0] - west) / hf.res, (north - p[..., 1]) / hf.res], axis=-1)

    _, label = cv2.connectedComponents(gentle, connectivity=8)
    c, r = np.round(cell(summit)).astype(int)
    if not gentle[r, c]:
        raise ValueError(f"the ground at {summit} is steeper than {max_slope} deg")
    top = (label == label[r, c]).astype(np.uint8)
    entry = None
    if keep is not None:
        pts = terrain.resample(keep, hf.res / 2)
        cols, rows = np.round(cell(pts)).astype(int).T
        on_top = top[rows, cols]
        if not on_top.any():
            raise ValueError("the path never reaches the hilltop")
        first = int(np.argmax(on_top))
        entry = tuple(map(float, pts[first]))
        cv2.polylines(top, [np.round(cell(pts[first:]) * 16).astype(np.int32)], False, 1,
                      thickness=int(round(2 * keep_width / hf.res)), lineType=cv2.LINE_8, shift=4)
    contours, _ = cv2.findContours(top, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    outline = cv2.approxPolyDP(max(contours, key=cv2.contourArea), 0.5, True)[:, 0, :] * hf.res
    path = [(west + float(u), north - float(v)) for u, v in outline]
    return path + path[:1], entry


def _neighbour(a, dr, dc, fill):
    """a[r + dr, c + dc] at every (r, c); `fill` where that is off the grid."""
    out = np.full_like(a, fill)
    h, w = a.shape
    out[max(-dr, 0):h - max(dr, 0), max(-dc, 0):w - max(dc, 0)] = a[max(dr, 0):h - max(-dr, 0), max(dc, 0):w - max(-dc, 0)]
    return out


def _cell(X, Y, p):
    """Grid (row, col) nearest to layout point p."""
    return int(np.argmin(np.abs(Y[:, 0] - p[1]))), int(np.argmin(np.abs(X[0] - p[0])))
