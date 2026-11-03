"""Offline harness for the P0-R suites: independent 3-D ray caster, packet builder, clearance oracle and camera
simulator. Shares no code with src/ (the suites call only the public API of `core`). Ground truth stays here."""
import json
import math
import os
import pathlib
import sys

import numpy as np

W = pathlib.Path(__file__).resolve().parents[1]
SRC = pathlib.Path(os.environ.get("P0R_SRC", W / "src"))
sys.path.insert(0, str(SRC))
sys.path.insert(0, str(W / "snapshot" / "c_review"))
import core  # noqa: E402
import indep as I  # noqa: E402

CATALOG, BALL = I.catalog()
R_PLAN, TARGET_R, MIN_R, MAX_R, DT = 0.45, 0.20, 0.2, 12.0, 0.1
CAPTURE = 0.5


# ---------------------------------------------------------------- objects (ground truth)
def bar(cx, cy, hx, hy):
    return {"kind": "bar", "type": "box", "c": (cx, cy), "h": (hx, hy), "z": (0.0, 3.0)}


def noncat_box(cx, cy, hx, hy, z=(0.0, 3.0)):
    return {"kind": "noncat", "type": "box", "c": (cx, cy), "h": (hx, hy), "z": z}


def pole(cx, cy, r, z=(0.0, 3.0)):
    return {"kind": "noncat", "type": "pole", "c": (cx, cy), "r": r, "z": z}


def target(cx, cy, cz=1.0, r=TARGET_R):
    return {"kind": "target", "type": "sphere", "c": (cx, cy, cz), "r": r}


# ---------------------------------------------------------------- beams (recomputed from the MOTAR formula)
def beam_dirs(layout):
    if layout == "P0R-72x1":
        rows, az = [0.0], [math.radians(5.0 * j) for j in range(72)]
    else:   # warp_lidar.py:79-84 with hmin -175, hmax 180, vmin -10, vmax 20, 72 x 4
        rows = [math.radians(20.0 - 30.0 * i / 3) for i in range(4)]
        az = [math.radians(180.0 - 355.0 * j / 71) for j in range(72)]
    return np.array([[(math.cos(a) * math.cos(e), math.sin(a) * math.cos(e), math.sin(e)) for a in az] for e in rows])


def _box_t(o, d, ob):
    lo, hi = 0.0, math.inf
    bounds = [(ob["c"][0] - ob["h"][0], ob["c"][0] + ob["h"][0]), (ob["c"][1] - ob["h"][1], ob["c"][1] + ob["h"][1]),
              ob["z"]]
    inside = True
    for k in range(3):
        mn, mx = bounds[k]
        if not mn <= o[k] <= mx:
            inside = False
        if abs(d[k]) < 1e-15:
            if o[k] < mn or o[k] > mx:
                return math.inf
        else:
            t0, t1 = sorted(((mn - o[k]) / d[k], (mx - o[k]) / d[k]))
            lo, hi = max(lo, t0), min(hi, t1)
            if lo > hi:
                return math.inf
    return 0.0 if inside else lo


def _sphere_t(o, d, ob):
    c, r = ob["c"], ob["r"]
    oc = [o[k] - c[k] for k in range(3)]
    b = sum(oc[k] * d[k] for k in range(3))
    cc = sum(x * x for x in oc) - r * r
    disc = b * b - cc
    if disc < 0:
        return math.inf
    t = -b - math.sqrt(disc)
    return t if t >= 0 else (0.0 if cc <= 0 else math.inf)


def _pole_t(o, d, ob):
    hx, hy = d[0], d[1]
    a = hx * hx + hy * hy
    if a < 1e-18:
        return math.inf
    ox, oy = o[0] - ob["c"][0], o[1] - ob["c"][1]
    b = ox * hx + oy * hy
    cc = ox * ox + oy * oy - ob["r"] ** 2
    disc = b * b - a * cc
    if disc < 0:
        return math.inf
    t = (-b - math.sqrt(disc)) / a
    if t < 0:
        return 0.0 if cc <= 0 else math.inf
    z = o[2] + d[2] * t
    return t if ob["z"][0] <= z <= ob["z"][1] else math.inf


def cast(o, d, objs):
    """(nearest t, index of the object) along unit 3-D direction d from o."""
    best, who = math.inf, None
    for i, ob in enumerate(objs):
        t = {"box": _box_t, "sphere": _sphere_t, "pole": _pole_t}[ob["type"]](o, d, ob)
        if t < best:
            best, who = t, i
    return best, who


def scan(objs, pose, layout="P0R-72x1"):
    """Exact nearest-hit scan with explicit status: NEAR_CLIPPED (< 0.2, range 0), HIT, NO_HIT (12.0)."""
    x, y, z, yaw = pose
    D = beam_dirs(layout)
    c, s = math.cos(yaw), math.sin(yaw)
    rng = np.zeros(D.shape[:2])
    st = np.zeros(D.shape[:2], np.int8)
    who = np.full(D.shape[:2], -1)
    for i in range(D.shape[0]):
        for j in range(D.shape[1]):
            d = D[i, j]
            wd = (c * d[0] - s * d[1], s * d[0] + c * d[1], d[2])
            t, k = cast((x, y, z), wd, objs)
            if t < MIN_R:
                rng[i, j], st[i, j] = 0.0, core.NEAR_CLIPPED
            elif t <= MAX_R:
                rng[i, j], st[i, j] = t, core.HIT
            else:
                rng[i, j], st[i, j] = 12.0, core.NO_HIT
            who[i, j] = -1 if t > MAX_R else k
    return rng, st, who


def header(step, pose, *, env=0, episode=0, health=2, noise=0):
    hi = np.array([env, episode, step, health, noise], np.int64)
    hf = np.array([step * DT, pose[0], pose[1], pose[2], pose[3]], np.float64)
    return hi, hf


# ---------------------------------------------------------------- oracle
def _pt_seg(p, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    den = dx * dx + dy * dy
    t = 0.0 if den == 0 else max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / den))
    return math.hypot(p[0] - a[0] - t * dx, p[1] - a[1] - t * dy)


def clearance_by_kind(path, objs, z_vehicle=1.0, radius=R_PLAN, band=0.202):
    """Min (distance - radius) from the 2-D segment to each kind's footprint, for objects overlapping the vehicle's
    altitude band; +inf if none."""
    out = {"bar": math.inf, "noncat": math.inf, "target": math.inf}
    for ob in objs:
        if ob["type"] == "sphere":
            dz = abs(ob["c"][2] - z_vehicle)
            if dz >= ob["r"] + band:
                continue
            d = _pt_seg(ob["c"][:2], *path) - ob["r"] - radius
        else:
            if ob["z"][1] < z_vehicle - band or ob["z"][0] > z_vehicle + band:
                continue
            if ob["type"] == "box":
                d = I.clearance(path, [I.rect_poly(ob["c"][0], ob["c"][1], ob["h"][0], ob["h"][1])], radius)
            else:
                d = _pt_seg(ob["c"], *path) - ob["r"] - radius
        out[ob["kind"]] = min(out[ob["kind"]], d)
    return out


def classify(cert, path, objs, z_vehicle=1.0):
    c = clearance_by_kind(path, objs, z_vehicle)
    tgt = [ob for ob in objs if ob["kind"] == "target"]
    terminal = any(_pt_seg(t["c"][:2], *path) < R_PLAN + t["r"] or
                   math.hypot(path[1][0] - t["c"][0], path[1][1] - t["c"][1]) < CAPTURE for t in tgt)
    if cert["static_bar_certified"]:
        if c["bar"] <= 0:
            return "FALSE_CERT"
        if c["noncat"] <= 0 or c["target"] <= 0:
            return "OUT_OF_SCOPE"
        return "VALID"
    if c["bar"] <= 0 or c["noncat"] <= 0:          # a non-target object within R justifies the refusal by itself
        return "JUSTIFIED_REFUSAL"
    if terminal:
        return "TERMINAL_UNCERTIFIED"
    if c["target"] <= 0:
        return "JUSTIFIED_REFUSAL"
    return "DEADLOCK_OR_EVIDENCE_LIMIT"


# ---------------------------------------------------------------- camera simulator (labels only)
def _rot(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw),
                              math.sin(yaw))
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def camera(t, pose, point_world, *, ext=(0.10, 0.0, 0.03, 0.0, 0.0, 0.0), sigma_r=0.05, sigma_b=0.005,
           range_err=0.0, bearing_err=0.0, visible=1.0):
    """A camera observation of `point_world` from the camera at `ext` in the yaw-level sensor frame."""
    x, y, z, yaw = pose
    p = np.asarray(point_world, float) - np.array([x, y, z])
    p = _rot(0, 0, -yaw) @ p
    q = _rot(*ext[3:]).T @ (p - np.array(ext[:3]))
    rng = float(np.linalg.norm(q))
    u = q / rng
    if bearing_err:
        u = _rot(0.0, 0.0, bearing_err) @ u
    return np.array([t, visible, *ext, *u, rng + range_err, sigma_r, sigma_b], np.float64)


def dump(path, obj):
    pathlib.Path(path).write_text(json.dumps(obj, indent=1, default=str) + "\n")
