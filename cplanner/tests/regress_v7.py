"""R2 rule 3: the 4,000 nominal falsification trials and the 960 real-layout trials of v3-v7, regenerated with the
same RNG call sequences as v7 `tests/regress_boundary.py`, through the P0-R array boundary. Compared trial by trial
with v7 run 8 (`regress_boundary.json`) and with the independent oracle. usage: regress_v7.py OUT_JSON"""
import hashlib
import json
import math
import random
import sys
import time

import numpy as np

from harness import I, R_PLAN, BALL, CATALOG, W, core, dump

V7 = json.load(open("/home/fair/motar_tmp/p0_v7_20261029/results/run8/regress_boundary.json"))
FZ = json.load(open(W / "snapshot/c_review/fz_nominal.json"))
MODES = (("criterion_synthetic_assurance", "SYNTH-72x1"), ("no_nearest_hit_assurance", "SYNTH-72x1-NOASSURE"))
OUT = sys.argv[1]


def beam(i):
    a = math.radians(i * 360.0 / 72)
    return (math.cos(a), math.sin(a))


def packet(polys):
    """The v7 synthetic scan (I.ray_poly, exact nearest hit) as P0-R arrays."""
    r = np.zeros((1, 72))
    s = np.zeros((1, 72), np.int8)
    for i in range(72):
        d = min([math.inf] + [I.ray_poly((0.0, 0.0), beam(i), p) for p in polys])
        if d < 0.2:
            r[0, i], s[0, i] = 0.0, core.NEAR_CLIPPED
        elif d <= 12.0:
            r[0, i], s[0, i] = d, core.HIT
        else:
            r[0, i], s[0, i] = 12.0, core.NO_HIT
    return r, s


def new_cert(polys, path, cfg):
    r, s = packet(polys)
    c = core.CertifierCore(cfg, 0, 0)
    t0 = time.perf_counter()
    rep = c.step(r, s, np.array([0, 0, 0, 2, 0], np.int64), np.array([0.0, 0.0, 0.0, 1.0, 0.0]))
    t1 = time.perf_counter()
    out = c.certify(np.array([path], np.float64), 0)[0]
    t2 = time.perf_counter()
    assert rep["accepted"] and not c.exceptions
    return {"certified": out["static_bar_certified"], "reason": out["reason"], "nodes": out.get("nodes"),
            "ms_step": 1e3 * (t1 - t0), "ms_certify": 1e3 * (t2 - t1)}


def boundary_bar(path, rng):
    """Verbatim from v7 regress_boundary.py (itself verbatim from snapshot/c_review/falsify.py, nominal)."""
    hx, hy = rng.choice(CATALOG)
    a, b = path
    t = rng.uniform(0.2, 1.0)
    p = (a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]))
    L = math.hypot(b[0] - a[0], b[1] - a[1])
    nx, ny = -(b[1] - a[1]) / L, (b[0] - a[0]) / L
    if rng.random() < 0.5:
        nx, ny = -nx, -ny
    want = R_PLAN + rng.uniform(-0.08, 0.08)
    lo, hi = 0.0, 3.0
    for _ in range(60):
        mid = (lo + hi) / 2
        poly = I.rect_poly(p[0] + nx * mid, p[1] + ny * mid, hx, hy, 0.0)
        if I.clearance(path, [poly], 0.0) < want:
            lo = mid
        else:
            hi = mid
    return I.rect_poly(p[0] + nx * hi, p[1] + ny * hi, hx, hy, 0.0)


def summarise(rows, mode, v7rows):
    ms = sorted(r[mode]["ms_certify"] for r in rows)
    q = lambda p: ms[min(len(ms) - 1, int(p * len(ms)))]  # noqa: E731
    cert = [r for r in rows if r[mode]["certified"]]
    return {"trials": len(rows), "certified": len(cert), "plan_feasible": sum(r["plan"] > 1e-9 for r in rows),
            "FALSE_CERT_plan": sum(r["plan"] <= 1e-9 for r in cert),
            "FALSE_CERT_physical": sum(r["phys"] <= 1e-9 for r in cert),
            "same_flag_as_v7": sum(r[mode]["certified"] == v[mode]["certified"] for r, v in zip(rows, v7rows)),
            "same_reason_as_v7": sum(r[mode]["reason"] == v[mode]["reason"] for r, v in zip(rows, v7rows)),
            "same_nodes_as_v7": sum(r[mode]["nodes"] == v[mode]["nodes"] for r, v in zip(rows, v7rows)
                                    if r[mode]["certified"] or r[mode]["reason"] in
                                    ("UNRESOLVED_CENTER_CELL", "NODE_BUDGET_UNKNOWN")),
            "ms_certify": {"p50": q(.5), "p95": q(.95), "p99": q(.99), "max": ms[-1],
                           "over_100ms": sum(x > 100 for x in ms)}}


t_start = time.time()
rng = random.Random(FZ["summary"]["seed"])
rows, mismatch = [], 0
for k in range(4000):
    path = I.random_path(rng)
    polys, _ = I.random_scene(rng, CATALOG, n_bars=rng.randint(1, 8), yaw_mode="axis", yaw_max=math.pi / 4)
    if k % 2 == 1:
        b = boundary_bar(path, rng)
        if I.clearance(((0.0, 0.0), (0.0, 0.0)), [b], 0.0) > 0.3:
            polys = polys + [b]
    o = FZ["rows"][k]
    plan, phys = I.clearance(path, polys, R_PLAN), I.clearance(path, polys, BALL)
    mismatch += plan != o["plan_clearance"] or phys != o["phys_clearance"] or len(polys) != o["n_bars"]
    row = {"k": k, "plan": plan, "phys": phys, "path": path}
    for mode, cfg in MODES:
        row[mode] = new_cert(polys, path, cfg)
    rows.append(row)
v7n = V7["nominal"]["rows"]
nominal = {"input_identity_mismatches": mismatch, "modes": {m: summarise(rows, m, v7n) for m, _ in MODES}}
print("nominal", json.dumps(nominal)[:1500], flush=True)

sys.path.insert(0, "/home/fair/workspaces/aerial_gym_ws/src/MOTAR-mt-p1")
from motar_eval import occlusion_trace as ot  # noqa: E402
REC = "/home/fair/workspaces/aerial_gym_ws/src/aerial_gym_simulator/results/b1r_classical_baseline_v1/A/F_SHADOW/"
rng = random.Random(20261027)
real = {}
for bars in (70, 205):
    tr = ot.read_trace(REC + "E2_obstacle_aware__%dbars__seed4301" % bars)
    frames = tr["frames"]
    f0 = frames[frames["frame"] == 0]
    look = ot.reset_lookup(tr["layouts"])
    rr = []
    for x in f0[:120]:
        key = (int(x["env"]), int(x["episode"]))
        if key not in look:
            continue
        c, h, _ = ot.layout_arrays(tr["layouts"], look[key])
        o = (float(x["p_pos"][0]), float(x["p_pos"][1]))
        polys = [I.rect_poly(float(cx) - o[0], float(cy) - o[1], float(hx), float(hy))
                 for (cx, cy, _), (hx, hy, _) in zip(c, h)]
        I.scan(polys)                                   # keeps the v7 call sequence (no RNG use)
        for _ in range(4):
            path = I.random_path(rng, 0.3, 2.0)
            row = {"env_episode": list(key), "plan": I.clearance(path, polys, R_PLAN),
                   "phys": I.clearance(path, polys, BALL)}
            for mode, cfg in MODES:
                row[mode] = new_cert(polys, path, cfg)
            rr.append(row)
    v7r = V7["real_layouts"][str(bars)]["rows"]
    same_input = len(rr) == len(v7r) and all(abs(a["plan"] - b["plan"]) == 0 for a, b in zip(rr, v7r))
    real[str(bars)] = {"inputs_identical_to_v7": same_input, "modes": {m: summarise(rr, m, v7r) for m, _ in MODES},
                       "rows": rr}
    print("real", bars, json.dumps({k: v for k, v in real[str(bars)].items() if k != "rows"})[:1500], flush=True)

allm = [nominal["modes"][m] for m, _ in MODES] + [v["modes"][m] for v in real.values() for m, _ in MODES]
checks = {
    "nominal_inputs_identical_to_preserved": mismatch == 0,
    "real_inputs_identical_to_v7": all(v["inputs_identical_to_v7"] for v in real.values()),
    "zero_false_certification": all(s["FALSE_CERT_plan"] == 0 for s in allm),
    "flags_identical_to_v7": all(s["same_flag_as_v7"] == s["trials"] for s in allm),
    "reasons_identical_to_v7": all(s["same_reason_as_v7"] == s["trials"] for s in allm),
}
dump(OUT, {"checks": checks, "pass": all(checks.values()), "nominal": dict(nominal, rows=rows), "real_layouts": real,
           "wall_s": time.time() - t_start,
           "v7_reference_sha256": hashlib.sha256(
               open("/home/fair/motar_tmp/p0_v7_20261029/results/run8/regress_boundary.json", "rb").read()).hexdigest()})
print(json.dumps({"checks": checks, "wall_s": time.time() - t_start}, indent=1))
