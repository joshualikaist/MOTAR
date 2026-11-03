"""Independent review harness (scratch only). Shares NO code with the prototype's oracle.py.

Geometry is computed from polygon edges (rectangles with optional yaw), exact segment-segment
distance and edge-parametric ray casting. Used only to falsify the prototype's certificate.
"""
import json
import math
import random

CATALOG_JSON = __import__('os').path.join(__import__('os').path.dirname(__file__), '..', 'geometry_results.json')


def catalog():
    g = json.load(open(CATALOG_JSON))
    return [(b['size_m'][0] / 2, b['size_m'][1] / 2) for b in g['bars']], g['quad_any_attitude_ball_m']


def rect_poly(cx, cy, hx, hy, yaw=0.0):
    c, s = math.cos(yaw), math.sin(yaw)
    pts = [(-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy)]
    return [(cx + c * x - s * y, cy + s * x + c * y) for x, y in pts]


def _cross(o, a, b):
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])


def point_in_poly(p, poly):
    sgn = [_cross(poly[i], poly[(i + 1) % 4], p) for i in range(4)]
    return all(v >= 0 for v in sgn) or all(v <= 0 for v in sgn)


def seg_seg_dist(p1, p2, q1, q2):
    d1, d2 = _cross(q1, q2, p1), _cross(q1, q2, p2)
    d3, d4 = _cross(p1, p2, q1), _cross(p1, p2, q2)
    if (d1 > 0) != (d2 > 0) and (d3 > 0) != (d4 > 0) and d1 != 0 and d2 != 0 and d3 != 0 and d4 != 0:
        return 0.0

    def pt_seg(p, a, b):
        ax, ay = b[0] - a[0], b[1] - a[1]
        den = ax * ax + ay * ay
        t = 0.0 if den == 0 else max(0.0, min(1.0, ((p[0] - a[0]) * ax + (p[1] - a[1]) * ay) / den))
        return math.hypot(p[0] - a[0] - t * ax, p[1] - a[1] - t * ay)
    return min(pt_seg(p1, q1, q2), pt_seg(p2, q1, q2), pt_seg(q1, p1, p2), pt_seg(q2, p1, p2))


def clearance(path, polys, radius):
    """min distance(path segment, any polygon) - radius; 0 distance if overlapping."""
    a, b = path
    best = math.inf
    for poly in polys:
        if point_in_poly(a, poly) or point_in_poly(b, poly):
            return -radius
        for i in range(4):
            best = min(best, seg_seg_dist(a, b, poly[i], poly[(i + 1) % 4]))
    return best - radius


def ray_poly(origin, d, poly):
    """Smallest t >= 0 where origin + t d meets the polygon boundary (inf if none)."""
    best = math.inf
    for i in range(4):
        p, q = poly[i], poly[(i + 1) % 4]
        ex, ey = q[0] - p[0], q[1] - p[1]
        den = d[0] * ey - d[1] * ex
        if abs(den) < 1e-15:
            continue
        wx, wy = p[0] - origin[0], p[1] - origin[1]
        t = (wx * ey - wy * ex) / den
        u = (wx * d[1] - wy * d[0]) / den
        if t >= 0 and -1e-12 <= u <= 1 + 1e-12:
            best = min(best, t)
    return best


def scan(polys, *, true_origin=(0.0, 0.0), reported_origin=(0.0, 0.0), range_bias=0.0, range_sigma=0.0,
         rng=None, n=72, far=12.0, min_range=0.2):
    """Horizontal exact scan from true_origin; reported to the certificate from reported_origin."""
    out = []
    for i in range(n):
        ang = math.radians(i * 360.0 / n)
        d = (math.cos(ang), math.sin(ang))
        dist = min([far] + [ray_poly(true_origin, d, p) for p in polys])
        hit = dist < far
        if hit:
            dist = dist + range_bias + (rng.gauss(0, range_sigma) if range_sigma else 0.0)
            dist = min(max(dist, min_range), far - 1e-9)
        out.append({'origin': list(reported_origin), 'direction': list(d), 'range': dist if hit else far,
                    'status': 'HIT' if hit else 'NO_HIT', 'valid_exact': True, 'candidate': False, 'timestamp': 0.0})
    return out


def random_scene(rng, shapes, *, n_bars, yaw_mode='axis', keep_out=0.3, extent=3.0, yaw_max=math.pi / 4):
    polys, rects = [], []
    tries = 0
    while len(polys) < n_bars and tries < 500:
        tries += 1
        hx, hy = rng.choice(shapes)
        cx, cy = rng.uniform(-extent, extent), rng.uniform(-extent, extent)
        yaw = 0.0 if yaw_mode == 'axis' else rng.uniform(-yaw_max, yaw_max)
        poly = rect_poly(cx, cy, hx, hy, yaw)
        # pursuer start must be outside every bar with a margin
        if clearance(((0.0, 0.0), (0.0, 0.0)), [poly], 0.0) <= keep_out:
            continue
        polys.append(poly)
        rects.append((cx, cy, hx, hy, yaw))
    return polys, rects


def random_path(rng, lo=0.3, hi=2.5):
    ang = rng.uniform(-math.pi, math.pi)
    length = rng.uniform(lo, hi)
    return ((0.0, 0.0), (length * math.cos(ang), length * math.sin(ang)))
