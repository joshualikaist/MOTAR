"""Gate D1-F: full-bound direction adversary for the direction-bound repair (docs/plans/prereg_cplanner_dirbound_d1f_v1.md).

Closes review I1 of the repair (D1 had no power at the scale of eps). The construction follows the independent
reviewer's probe P1 (audit/probes/p1_full_eps.py of the repair review); the code here is the gate's own.

Ground truth and casting are local, sharing no code with `src/`: a vectorised slab caster for axis-aligned 0-3 m
boxes (no ground plane, as in B1-R) and an exact 2-D segment-rectangle distance.
Adversary for every beam with manifest world direction d and declared origin o:
  probed rays  o + e0 + t w,  w = normalize(12 d + e12 - e0),  e0, e12 in {0, +-a u, +-a v, +-a (u+v)/sqrt2,
  +-a (u-v)/sqrt2}, with u the horizontal normal of d, v = d x u and a = 0.99 eps.
  This gives 81 straight rays per beam, each within a of o + t d for every t in [0, 12] (checked; reported as
  max_family_offset_m), covering origin offsets, tilts and pivots.
  The declaration is the largest nearest-hit distance over the family: NO_HIT if any ray misses within 12 m, the raw
  distance as the HIT range, NEAR_CLIPPED below 0.2 m.
Scenes: 1-6 catalogue bars 0.8-4 m from the vehicle.
  Heights: 40 % in [0.3, 2.7] m, 20 % in [2.95, 2.999], 10 % in [2.999, 2.99999], 20 % in [0.001, 0.05] and
  10 % in [0.00001, 0.001] m.
  Candidates: 4 random segments, plus up to 12 aimed by bisection on the heading so that their exact clearance to a
  random bar is R - delta, delta ~ U(-0.3 mm, 1.5 mm). Most of the aimed ones must therefore be refused.
Parts (the same adversarial scans):
  DIR    MOTAR-72x4-RUNTIME-DIR-NATIVE (repaired, eps 1 mm)   -> FALSE_CERT must be 0, certified > 0
  ZERO   MOTAR-72x4-RUNTIME-NATIVE (eps 0, unrepaired)       -> FALSE_CERT > 0 overall and at heights in [0.3, 2.7]
  EXACT  manifest-only scans through both                    -> 0 FALSE_CERT each, certified > 0
  VERB   MOTAR-72x4-RUNTIME-DIR (verbatim) on every 20th scene         -> results identical to DIR
usage: check_direction_bound_full.py OUT_JSON N_SCENES WORKERS SEED0
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
sys.path.insert(0, str(HERE.parent / "src"))
import core                                           # noqa: E402
import coverage_native as CN                          # noqa: E402
import registry as REG                                # noqa: E402

R = 0.45
EPS = REG.CONFIGS["MOTAR-72x4-RUNTIME-DIR"].direction_error_m
A = 0.99 * EPS
CAT = list(REG.BAR_HALF_SIZES)
CFG = {"DIR": "MOTAR-72x4-RUNTIME-DIR-NATIVE", "ZERO": "MOTAR-72x4-RUNTIME-NATIVE", "VERB": "MOTAR-72x4-RUNTIME-DIR"}
MAN = np.array([[(math.cos(math.radians(180 - 5 * j)) * math.cos(math.radians(e)),
                  math.sin(math.radians(180 - 5 * j)) * math.cos(math.radians(e)), math.sin(math.radians(e)))
                 for j in range(72)] for e in (20.0, 10.0, 0.0, -10.0)]).reshape(-1, 3)


def cast(O, D, lo, hi):
    """Nearest t >= 0 of rays (N,3)/(N,3) against closed boxes (M,3)/(M,3); inf if none."""
    if len(lo) == 0:
        return np.full(len(O), np.inf)
    O, D = O[:, None, :], D[:, None, :]
    with np.errstate(divide="ignore", invalid="ignore"):
        t1, t2 = (lo[None] - O) / D, (hi[None] - O) / D
    par = np.abs(D) < 1e-15
    inslab = (O >= lo[None]) & (O <= hi[None])
    tmn = np.where(par, np.where(inslab, -np.inf, np.inf), np.minimum(t1, t2))
    tmx = np.where(par, np.where(inslab, np.inf, -np.inf), np.maximum(t1, t2))
    tmin, tmax = tmn.max(2), tmx.min(2)
    ok = tmax >= np.maximum(tmin, 0.0)
    return np.where(ok, np.maximum(tmin, 0.0), np.inf).min(1)


def family(d):
    u = np.array([-d[1], d[0], 0.0])
    u /= np.linalg.norm(u)
    v = np.cross(d, u)
    s2 = 1.0 / math.sqrt(2.0)
    return [np.zeros(3), A * u, -A * u, A * v, -A * v, A * s2 * (u + v), -A * s2 * (u + v), A * s2 * (u - v),
            -A * s2 * (u - v)]


def scan(o, yaw, lo, hi, adversarial):
    c, s = math.cos(yaw), math.sin(yaw)
    W = MAN @ np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]]).T
    rng, st = np.zeros(288), np.zeros(288, np.int8)
    maxoff = 0.0
    for b in range(288):
        d = W[b]
        if adversarial:
            F = family(d)
            O, D = [], []
            for e0 in F:
                for e12 in F:
                    w = 12.0 * d + e12 - e0
                    w /= np.linalg.norm(w)
                    O.append(o + e0)
                    D.append(w)
                    for t in (0.0, 3.0, 6.0, 12.0):
                        maxoff = max(maxoff, float(np.linalg.norm(o + e0 + t * w - (o + t * d))))
            O, D = np.array(O), np.array(D)
        else:
            O, D = o[None, :], d[None, :]
        t = cast(O, D, lo, hi).max()
        if not t <= 12.0:
            rng[b], st[b] = 12.0, core.NO_HIT
        elif t < 0.2:
            rng[b], st[b] = 0.0, core.NEAR_CLIPPED
        else:
            rng[b], st[b] = t, core.HIT
    return rng.reshape(4, 72), st.reshape(4, 72), maxoff


def seg_rect(a, b, cx, cy, hx, hy):
    x0, x1, y0, y1 = cx - hx, cx + hx, cy - hy, cy + hy
    lo_, hi_ = 0.0, 1.0
    for p, dd, mn, mx in ((a[0], b[0] - a[0], x0, x1), (a[1], b[1] - a[1], y0, y1)):
        if abs(dd) < 1e-15:
            if p < mn or p > mx:
                break
        else:
            t0, t1 = sorted(((mn - p) / dd, (mx - p) / dd))
            lo_, hi_ = max(lo_, t0), min(hi_, t1)
            if lo_ > hi_:
                break
    else:
        return 0.0

    def pseg(px, py):
        dx, dy = b[0] - a[0], b[1] - a[1]
        den = dx * dx + dy * dy
        t = 0.0 if den == 0 else max(0.0, min(1.0, ((px - a[0]) * dx + (py - a[1]) * dy) / den))
        return math.hypot(px - a[0] - t * dx, py - a[1] - t * dy)
    vals = [pseg(x, y) for x in (x0, x1) for y in (y0, y1)]
    vals += [math.hypot(max(x0 - p[0], 0.0, p[0] - x1), max(y0 - p[1], 0.0, p[1] - y1)) for p in (a, b)]
    return min(vals)


def clearance(seg, bars):
    return min(seg_rect(seg[0], seg[1], *bb) for bb in bars) if bars else math.inf


def scene(seed0, i):
    r = random.Random(seed0 * 1000003 + i)
    u = r.random()
    z = (r.uniform(0.3, 2.7) if u < 0.4 else r.uniform(2.95, 2.999) if u < 0.6 else r.uniform(2.999, 2.99999)
         if u < 0.7 else r.uniform(0.001, 0.05) if u < 0.9 else r.uniform(0.00001, 0.001))
    pose = (r.uniform(8, 32), r.uniform(8, 32), z, r.uniform(-math.pi, math.pi))
    bars = []
    for _ in range(r.randint(1, 6)):
        hx, hy = CAT[r.randrange(len(CAT))]
        for _t in range(30):
            ang, dist = r.uniform(-math.pi, math.pi), r.uniform(0.8, 4.0)
            cx, cy = pose[0] + dist * math.cos(ang), pose[1] + dist * math.sin(ang)
            if (abs(cx - pose[0]) > hx + 0.3 or abs(cy - pose[1]) > hy + 0.3) and all(
                    abs(cx - b[0]) > hx + b[2] + 0.01 or abs(cy - b[1]) > hy + b[3] + 0.01 for b in bars):
                bars.append((cx, cy, hx, hy))
                break
    p = (pose[0], pose[1])
    segs, aimed = [], []
    for _ in range(4):
        h, L = r.uniform(-math.pi, math.pi), r.uniform(0.5, 3.0)
        segs.append((p, (p[0] + L * math.cos(h), p[1] + L * math.sin(h))))
        aimed.append(None)
    for _ in range(12):
        if not bars:
            break
        bb = bars[r.randrange(len(bars))]
        delta = r.uniform(-0.0003, 0.0015)
        want = R - delta
        bc = math.atan2(bb[1] - p[1], bb[0] - p[0])
        L = min(4.0, max(0.5, math.hypot(bb[0] - p[0], bb[1] - p[1]) + r.uniform(-0.3, 0.6)))
        side = r.choice((-1.0, 1.0))
        f = lambda th: seg_rect(p, (p[0] + L * math.cos(th), p[1] + L * math.sin(th)), *bb)  # noqa: E731
        lo_, hi_ = bc, bc + side * math.pi / 2
        if not (f(lo_) <= want <= f(hi_)):
            continue
        for _k in range(80):
            mid = 0.5 * (lo_ + hi_)
            if f(mid) <= want:
                lo_ = mid
            else:
                hi_ = mid
        segs.append((p, (p[0] + L * math.cos(lo_), p[1] + L * math.sin(lo_))))
        aimed.append(delta)
    return pose, bars, segs, aimed


def certify(cfg, rs, pose, segs, i):
    c = core.CertifierCore(cfg, 0, i)
    hi = np.array([0, i, 1, core.HEALTH_RUNTIME, 0], np.int64)
    hf = np.array([0.1, pose[0], pose[1], pose[2], pose[3]], np.float64)
    rep = c.step(rs[0], rs[1], hi, hf)
    if not rep.get("accepted"):
        return rep, None
    return rep, c.certify(np.array(segs, dtype=np.float64), rep["step"])


def run_chunk(args):
    seed0, ids = args
    parts = ("DIR", "ZERO", "EXACT_DIR", "EXACT_ZERO")
    st = {p: {"scenes": 0, "accepted": 0, "segments": 0, "aimed_segments": 0, "certified": 0, "false_cert": 0,
              "false_cert_mid_height": 0, "false_cert_aimed": 0} for p in parts}
    st["VERB"] = {"scenes": 0, "segments": 0, "mismatch": 0}
    st["max_family_offset_m"] = 0.0
    ex = []
    for i in ids:
        pose, bars, segs, aimed = scene(seed0, i)
        lo = np.array([[b[0] - b[2], b[1] - b[3], 0.0] for b in bars])
        hi = np.array([[b[0] + b[2], b[1] + b[3], 3.0] for b in bars])
        o = np.array(pose[:3])
        adv = scan(o, pose[3], lo, hi, True)
        exact = scan(o, pose[3], lo, hi, False)
        st["max_family_offset_m"] = max(st["max_family_offset_m"], adv[2])
        clr = [clearance(sg, bars) for sg in segs]
        mid = 0.3 <= pose[2] <= 2.7
        outs = {}
        for part, cfg, sc in (("DIR", CFG["DIR"], adv), ("ZERO", CFG["ZERO"], adv),
                              ("EXACT_DIR", CFG["DIR"], exact), ("EXACT_ZERO", CFG["ZERO"], exact)):
            s = st[part]
            s["scenes"] += 1
            rep, out = certify(cfg, sc[:2], pose, segs, i)
            outs[part] = out
            if out is None:
                continue
            s["accepted"] += 1
            for seg, d, cl, am in zip(segs, out, clr, aimed):
                s["segments"] += 1
                s["aimed_segments"] += int(am is not None)
                s["certified"] += int(d["static_bar_certified"])
                if d["static_bar_certified"] and cl <= R:
                    s["false_cert"] += 1
                    s["false_cert_mid_height"] += int(mid)
                    s["false_cert_aimed"] += int(am is not None)
                    if len(ex) < 30:
                        ex.append({"part": part, "scene": i, "z": pose[2], "clearance_minus_R_m": cl - R,
                                   "aimed_delta_m": am})
        if i % 20 == 0 and outs["DIR"] is not None:                 # verbatim identity subset (VERB_N scenes total)
            _, outv = certify(CFG["VERB"], adv[:2], pose, segs, i)
            st["VERB"]["scenes"] += 1
            st["VERB"]["segments"] += len(segs)
            a = [{k: v for k, v in d.items() if k != "config"} for d in outs["DIR"]]
            b = [{k: v for k, v in d.items() if k != "config"} for d in (outv or [])]
            st["VERB"]["mismatch"] += int(json.dumps(a, sort_keys=True, default=float)
                                          != json.dumps(b, sort_keys=True, default=float))
    return st, ex


def merge(results):
    out, ex = None, []
    for st, e in results:
        ex += e
        if out is None:
            out = st
            continue
        for k, v in st.items():
            if k == "max_family_offset_m":
                out[k] = max(out[k], v)
            else:
                for kk, vv in v.items():
                    out[k][kk] += vv
    return out, ex[:30]


def main():
    out = pathlib.Path(sys.argv[1])
    n, workers, seed0 = int(sys.argv[2]), int(sys.argv[3]), int(sys.argv[4])
    if out.exists():
        raise SystemExit("refusing to overwrite " + str(out))
    t0 = time.time()
    lib_ok = CN.load() is not None
    chunks = [(seed0, list(range(k, n, workers * 4))) for k in range(workers * 4)]
    chunks = [c for c in chunks if c[1]]
    failure, st, ex = None, None, []
    try:
        if not lib_ok:
            raise RuntimeError("native coverage library not loadable: " + CN.path())
        with cf.ProcessPoolExecutor(max_workers=workers, mp_context=mp.get_context("spawn")) as pool:
            st, ex = merge(list(pool.map(run_chunk, chunks)))
    except Exception as e:
        failure = type(e).__name__ + ": " + str(e)[:300]
    checks = {"library_loaded": lib_ok,
              "completed": failure is None and st is not None and all(st[p]["scenes"] == n for p in
                                                                        ("DIR", "ZERO", "EXACT_DIR", "EXACT_ZERO"))}
    if checks["completed"]:
        checks.update({
            "adversary_within_bound": st["max_family_offset_m"] <= EPS,
            "all_steps_accepted": all(st[p]["accepted"] == n for p in ("DIR", "ZERO", "EXACT_DIR", "EXACT_ZERO")),
            "DIR_zero_false_cert": st["DIR"]["false_cert"] == 0,
            "DIR_nonvacuous_certified": st["DIR"]["certified"] > 0,
            "ZERO_detected_at_eps_scale": st["ZERO"]["false_cert"] > 0,
            "ZERO_detected_at_ordinary_heights": st["ZERO"]["false_cert_mid_height"] > 0,
            "EXACT_zero_false_cert": st["EXACT_DIR"]["false_cert"] == 0 and st["EXACT_ZERO"]["false_cert"] == 0,
            "EXACT_nonvacuous_certified": st["EXACT_DIR"]["certified"] > 0 and st["EXACT_ZERO"]["certified"] > 0,
            "VERB_identical": st["VERB"]["scenes"] > 0 and st["VERB"]["mismatch"] == 0,
        })
    res = {"gate": "C-Planner direction bound D1-F (full-bound adversary)", "pass": all(checks.values()),
           "checks": checks, "stats": st, "examples": ex, "eps_m": EPS, "a_m": A, "n_scenes": n, "seed0": seed0,
           "configs": CFG, "library": CN.path(), "failure": failure, "wall_s": time.time() - t0}
    with out.open("x") as fh:
        json.dump(res, fh, indent=1, default=float)
        fh.write("\n")
    print(json.dumps({"pass": res["pass"], "checks": checks}, indent=1))
    return 0 if res["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
