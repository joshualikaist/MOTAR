"""Direction-bound soundness fuzz for the C-Planner certifier (docs/plans/prereg_cplanner_dirbound_v1.md, gate D1).

The M2 V1 FAIL showed that MOTAR's kernel casts float32 rays that differ from the certifier's float64 manifest
directions by about 1e-7 rad, so a declared status can be true along the cast ray but false along the manifest. The
configuration MOTAR-72x4-RUNTIME-DIR declares a lateral bound eps (registry.direction_error_m). This suite builds the
most permissive scan that the bound allows: for every beam it casts the manifest direction and RING x 2 directions on
cones of half-angle alpha and alpha / 2 around it (independent harness ray caster), and declares the largest
nearest-hit distance found (NO_HIT if any of them misses within 12 m). The certifier receives that scan as if cast
along the manifest. Ground truth is the exact 2-D clearance of each candidate segment to every bar (harness.classify).

Parts (same scenes):
  A  MOTAR-72x4-RUNTIME-DIR (verbatim coverage), alpha = ALPHA_IN = 0.9 eps / 12 m  -> FALSE_CERT must be 0
  B  MOTAR-72x4-RUNTIME-DIR-NATIVE, alpha = ALPHA_OUT = 100 ALPHA_IN               -> FALSE_CERT > 0 (power control)
  C  MOTAR-72x4-RUNTIME-NATIVE (eps = 0), alpha = ALPHA_IN                         -> reported
  D  exact scan (alpha = 0) through MOTAR-72x4-RUNTIME-NATIVE and -DIR-NATIVE      -> FALSE_CERT 0; certified counts
                                                                                       reported (cost of eps)
  E  part A's packets through MOTAR-72x4-RUNTIME-DIR-NATIVE                        -> results identical to A
Candidates per scene: 8 random headings and lengths, and 8 segments aimed to pass a bar footprint at R plus a
uniform offset in [-0.05, 0.05] m (near-boundary candidates, where a small unsoundness would show).
usage: check_direction_bound.py OUT_JSON N_SCENES WORKERS
"""
import concurrent.futures as cf
import json
import math
import multiprocessing as mp
import pathlib
import random
import sys
import time

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import harness as H                                   # noqa: E402  (puts src/ on sys.path)
import core                                           # noqa: E402
import coverage_native as CN                          # noqa: E402
import registry as REG                                # noqa: E402

SAMPLE_SEED = 20261108
EPS = REG.CONFIGS["MOTAR-72x4-RUNTIME-DIR"].direction_error_m
ALPHA_IN = 0.9 * EPS / REG.CONFIGS["MOTAR-72x4-RUNTIME-DIR"].max_range
ALPHA_OUT = 100.0 * ALPHA_IN
RING = 8
CFG = {"A": "MOTAR-72x4-RUNTIME-DIR", "B": "MOTAR-72x4-RUNTIME-DIR-NATIVE", "C": "MOTAR-72x4-RUNTIME-NATIVE",
       "D0": "MOTAR-72x4-RUNTIME-NATIVE", "Ddir": "MOTAR-72x4-RUNTIME-DIR-NATIVE", "E": "MOTAR-72x4-RUNTIME-DIR-NATIVE"}


def _basis(d):
    a = np.array([0.0, 0.0, 1.0]) if abs(d[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
    u = np.cross(d, a)
    u /= np.linalg.norm(u)
    return u, np.cross(d, u)


def perm_scan(objs, pose, alpha):
    """(ranges, status) of the most permissive declaration within the cone of half-angle alpha (alpha 0: exact)."""
    x, y, z, yaw = pose
    D = H.beam_dirs("MOTAR-72x4")
    c, s = math.cos(yaw), math.sin(yaw)
    rng = np.zeros(D.shape[:2])
    st = np.zeros(D.shape[:2], np.int8)
    for i in range(D.shape[0]):
        for j in range(D.shape[1]):
            d0 = D[i, j]
            d = np.array((c * d0[0] - s * d0[1], s * d0[0] + c * d0[1], d0[2]))
            cand = [d]
            if alpha > 0:
                u, v = _basis(d)
                for frac in (1.0, 0.5):
                    ta = math.tan(alpha * frac)
                    for m in range(RING):
                        th = 2 * math.pi * m / RING
                        w = d + ta * (math.cos(th) * u + math.sin(th) * v)
                        cand.append(w / np.linalg.norm(w))
            t = max(H.cast((x, y, z), tuple(w), objs)[0] for w in cand)
            if t < H.MIN_R:
                rng[i, j], st[i, j] = 0.0, core.NEAR_CLIPPED
            elif t <= H.MAX_R:
                rng[i, j], st[i, j] = t, core.HIT
            else:
                rng[i, j], st[i, j] = 12.0, core.NO_HIT
    return rng, st


def scene(i):
    r = random.Random(SAMPLE_SEED * 1000003 + i)
    pose = (r.uniform(8.0, 32.0), r.uniform(8.0, 32.0), r.uniform(0.3, 2.7), r.uniform(-math.pi, math.pi))
    objs = []
    for _ in range(r.randint(1, 10)):
        hx, hy = H.CATALOG[r.randrange(len(H.CATALOG))]
        for _try in range(20):
            ang, dist = r.uniform(-math.pi, math.pi), r.uniform(1.0, 7.0)
            cx, cy = pose[0] + dist * math.cos(ang), pose[1] + dist * math.sin(ang)
            if abs(cx - pose[0]) > hx + 0.3 or abs(cy - pose[1]) > hy + 0.3:
                if all(abs(cx - o["c"][0]) > hx + o["h"][0] or abs(cy - o["c"][1]) > hy + o["h"][1] for o in objs):
                    objs.append(H.bar(cx, cy, hx, hy))
                    break
    paths = []
    for _ in range(8):
        h, L = r.uniform(-math.pi, math.pi), r.uniform(0.5, 2.5)
        paths.append(((pose[0], pose[1]), (pose[0] + L * math.cos(h), pose[1] + L * math.sin(h))))
    for _ in range(8):
        if not objs:
            h, L = r.uniform(-math.pi, math.pi), r.uniform(0.5, 2.5)
            paths.append(((pose[0], pose[1]), (pose[0] + L * math.cos(h), pose[1] + L * math.sin(h))))
            continue
        ob = objs[r.randrange(len(objs))]
        side = r.choice((-1.0, 1.0))
        off = H.R_PLAN + r.uniform(-0.05, 0.05)
        # aim at a point beside the bar's nearest corner region, offset by R (+/- 5 cm) perpendicular to the sight line
        vx, vy = ob["c"][0] - pose[0], ob["c"][1] - pose[1]
        n = math.hypot(vx, vy)
        px, py = -vy / n, vx / n
        ext = max(ob["h"]) + off
        tx, ty = ob["c"][0] + side * px * ext, ob["c"][1] + side * py * ext
        L = min(2.5, math.hypot(tx - pose[0], ty - pose[1]))
        h = math.atan2(ty - pose[1], tx - pose[0])
        paths.append(((pose[0], pose[1]), (pose[0] + L * math.cos(h), pose[1] + L * math.sin(h))))
    return pose, objs, paths


def certify(cfg, rs, pose, paths, scene_id):
    c = core.CertifierCore(cfg, 0, scene_id)
    rep = c.step(rs[0], rs[1], *H.header(1, pose, episode=scene_id, health=core.HEALTH_RUNTIME))
    if not rep.get("accepted"):
        return rep, None
    return rep, c.certify(np.array(paths, dtype=np.float64), rep["step"])


def strip_cfg(d):
    return {k: v for k, v in d.items() if k != "config"}


def run_chunk(ids):
    st = {p: {"scenes": 0, "accepted": 0, "segments": 0, "certified": 0, "false_cert": 0, "classes": {},
              "reasons": {}} for p in ("A", "B", "C", "D0", "Ddir")}
    st["E"] = {"segments": 0, "mismatch": 0}
    ex = []
    for i in ids:
        pose, objs, paths = scene(i)
        scans = {"in": perm_scan(objs, pose, ALPHA_IN), "out": perm_scan(objs, pose, ALPHA_OUT),
                 "exact": perm_scan(objs, pose, 0.0)}
        use = {"A": "in", "B": "out", "C": "in", "D0": "exact", "Ddir": "exact"}
        res = {}
        for p, sk in use.items():
            rep, out = certify(CFG[p], scans[sk], pose, paths, i)
            s = st[p]
            s["scenes"] += 1
            res[p] = out
            if out is None:
                s["reasons"]["STEP_REFUSED_" + str(rep.get("reason"))] = s["reasons"].get(
                    "STEP_REFUSED_" + str(rep.get("reason")), 0) + 1
                continue
            s["accepted"] += 1
            for path, d in zip(paths, out):
                s["segments"] += 1
                s["reasons"][d["reason"]] = s["reasons"].get(d["reason"], 0) + 1
                cls = H.classify(d, path, objs, pose[2])
                s["classes"][cls] = s["classes"].get(cls, 0) + 1
                s["certified"] += int(d["static_bar_certified"])
                if cls == "FALSE_CERT":
                    s["false_cert"] += 1
                    if len(ex) < 30:
                        ex.append({"part": p, "scene": i, "path": path, "reason": d["reason"],
                                   "bar_clearance_minus_R": H.clearance_by_kind(path, objs, pose[2])["bar"]})
        rep, outE = certify(CFG["E"], scans["in"], pose, paths, i)
        if res["A"] is not None or outE is not None:
            a = [strip_cfg(d) for d in (res["A"] or [])]
            b = [strip_cfg(d) for d in (outE or [])]
            st["E"]["segments"] += len(paths)
            st["E"]["mismatch"] += int(json.dumps(a, sort_keys=True, default=float)
                                       != json.dumps(b, sort_keys=True, default=float))
    return st, ex


def merge(parts):
    out, ex = None, []
    for st, e in parts:
        ex += e
        if out is None:
            out = st
            continue
        for p, s in st.items():
            for k, v in s.items():
                if isinstance(v, dict):
                    for kk, vv in v.items():
                        out[p][k][kk] = out[p][k].get(kk, 0) + vv
                else:
                    out[p][k] += v
    return out, ex[:30]


def main():
    out = pathlib.Path(sys.argv[1])
    n, workers = int(sys.argv[2]), int(sys.argv[3])
    if out.exists():
        raise SystemExit("refusing to overwrite " + str(out))
    t0 = time.time()
    lib_ok = CN.load() is not None
    chunks = [list(range(k, n, workers * 4)) for k in range(workers * 4)]
    chunks = [c for c in chunks if c]
    failure, st, ex = None, None, []
    try:
        if not lib_ok:
            raise RuntimeError("native coverage library not loadable: " + CN.path())
        with cf.ProcessPoolExecutor(max_workers=workers, mp_context=mp.get_context("spawn")) as pool:
            st, ex = merge(list(pool.map(run_chunk, chunks)))
    except Exception as e:
        failure = type(e).__name__ + ": " + str(e)[:300]
    checks = {"library_loaded": lib_ok, "completed": failure is None and st is not None
              and all(st[p]["scenes"] == n for p in ("A", "B", "C", "D0", "Ddir"))}
    if checks["completed"]:
        checks.update({
            "A_zero_false_cert": st["A"]["false_cert"] == 0,
            "A_nonvacuous_certified": st["A"]["certified"] > 0 and st["A"]["accepted"] == n,
            "B_power_control_detects": st["B"]["false_cert"] > 0,
            "D_exact_zero_false_cert": st["D0"]["false_cert"] == 0 and st["Ddir"]["false_cert"] == 0,
            "D_nonvacuous_certified": st["D0"]["certified"] > 0 and st["Ddir"]["certified"] > 0,
            "E_native_identical": st["E"]["mismatch"] == 0 and st["E"]["segments"] == 16 * n,
        })
    res = {"gate": "C-Planner direction bound D1", "pass": all(checks.values()), "checks": checks, "stats": st,
           "examples": ex, "eps_m": EPS, "alpha_in_rad": ALPHA_IN, "alpha_out_rad": ALPHA_OUT, "ring": RING,
           "n_scenes": n, "sample_seed": SAMPLE_SEED, "library": CN.path(), "failure": failure,
           "wall_s": time.time() - t0}
    with out.open("x") as fh:
        json.dump(res, fh, indent=1, default=float)
        fh.write("\n")
    print(json.dumps({"pass": res["pass"], "checks": checks}, indent=1))
    return 0 if res["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
