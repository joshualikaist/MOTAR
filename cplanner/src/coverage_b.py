"""Alternative-B coverage core, copied VERBATIM from P0 v7 `certify_b.py` (and unchanged since the S0/S1/S2
prototype, `prototype.py:70-140`): `point_segment`, `crosses`, `segment_box_distance`, `_coverage_B`.

P0-R does not edit these functions; `tests/check_coverage_ast.py` compares their AST with v3-v7. Source:
/home/fair/motar_tmp/p0_v7_20261029/src/certify_b.py (sha256 88a3acf9987e53df40e5884cad4b26d384952790c2985b9308ece64a28dcbb5a).
"""
import math


def point_segment(p, a, b):
    dx = b[0] - a[0]; dy = b[1] - a[1]; den = dx * dx + dy * dy
    t = max(0., min(1., ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / den)) if den else 0.
    return math.hypot(p[0] - a[0] - t * dx, p[1] - a[1] - t * dy)


def crosses(a, b, box, strict=False):
    xmin, ymin, xmax, ymax = box
    lo, hi = 0., 1.
    for x, d, mn, mx in [(a[0], b[0] - a[0], xmin, xmax), (a[1], b[1] - a[1], ymin, ymax)]:
        if abs(d) < 1e-15:
            if x < mn or x > mx:
                return False
        else:
            t0, t1 = sorted(((mn - x) / d, (mx - x) / d)); lo = max(lo, t0); hi = min(hi, t1)
            if lo > hi:
                return False
    if not strict:
        return True
    t = (lo + hi) / 2; x = a[0] + t * (b[0] - a[0]); y = a[1] + t * (b[1] - a[1])
    return xmin + 1e-9 < x < xmax - 1e-9 and ymin + 1e-9 < y < ymax - 1e-9


def segment_box_distance(a, b, box):
    if crosses(a, b, box):
        return 0.
    x0, y0, x1, y1 = box
    vals = [point_segment(p, a, b) for p in [(x0, y0), (x0, y1), (x1, y0), (x1, y1)]]
    vals += [math.hypot(max(x0 - p[0], 0, p[0] - x1), max(y0 - p[1], 0, p[1] - y1)) for p in (a, b)]
    return min(vals)


def _coverage_B(path, free, surfaces, shapes, radius, resolution, max_nodes):
    """Verbatim method-B branch of prototype.certify after evidence extraction."""
    a, b = path
    if any(point_segment(p, a, b) <= radius + 1e-9 for p in surfaces):
        return {'certified': False, 'reason': 'OBSERVED_SURFACE_INTERSECTION', 'nodes': 0}
    if not free:
        return {'certified': False, 'reason': 'NO_VALID_RAYS', 'nodes': 0}
    total = 0
    for hx, hy in shapes:
        reach = radius
        stack = [(min(a[0], b[0]) - reach - hx, min(a[1], b[1]) - reach - hy,
                  max(a[0], b[0]) + reach + hx, max(a[1], b[1]) + reach + hy, 0)]
        nodes = 0
        while stack:
            x0, y0, x1, y1, dep = stack.pop(); nodes += 1; total += 1
            if nodes > max_nodes:
                return {'certified': False, 'reason': 'NODE_BUDGET_UNKNOWN', 'nodes': total}
            if segment_box_distance(a, b, (x0 - hx, y0 - hy, x1 + hx, y1 + hy)) > reach + 1e-9:
                continue
            cx = (x0 + x1) / 2; cy = (y0 + y1) / 2; ex = (x1 - x0) / 2; ey = (y1 - y0) / 2
            covered = hx > ex and hy > ey and any(
                crosses(s, e, (cx - hx + ex, cy - hy + ey, cx + hx - ex, cy + hy - ey), True) for s, e in free)
            if covered:
                continue
            if max(x1 - x0, y1 - y0) <= resolution or dep >= 24:
                return {'certified': False, 'reason': 'UNRESOLVED_CENTER_CELL', 'nodes': total,
                        'shape_halfsize': [hx, hy], 'cell': [x0, y0, x1, y1]}
            if ex >= ey:
                stack += [(x0, y0, cx, y1, dep + 1), (cx, y0, x1, y1, dep + 1)]
            else:
                stack += [(x0, y0, x1, cy, dep + 1), (x0, cy, x1, y1, dep + 1)]
    return {'certified': True, 'reason': 'ALL_COLLIDING_CENTER_CELLS_EXCLUDED', 'nodes': total}
