"""M3 equivalence of the native coverage port (cplanner/native/coverage_b.c via src/coverage_native.py) with the
verbatim `coverage_b._coverage_B`. Every comparison is of whole results (JSON of the result dictionary, or the bit
pattern of a float), never of the certified flag alone.

Parts:
  H  bit identity of the CPython 3.8 hypot reproduction and of point_segment / segment_box_distance / crosses on
     random and adversarial inputs (zeros, signed zeros, tiny and huge magnitudes, axis-parallel and degenerate
     segments, touching boxes, inf and nan for hypot);
  T  threshold probes at the code's own constants (strict-crossing margin 1e-9 on every box edge, the parallel test
     |d| < 1e-15, the surface rule <= R + 1e-9 at exact distances), with true and false outcomes on each side;
  S  synthetic scenes (harness ray caster; P0R-72x1 and MOTAR-72x4 alternating; sensor height in [0.3, 2.7] m;
     0-8 catalog bars; a target sphere), 16 segments each, full catalog, at the qualified configuration
     (R 0.45, resolution and node budget of SYNTH-72x1) and at two stress settings (node budget 60; resolution
     0.004 m) that exercise NODE_BUDGET_UNKNOWN and UNRESOLVED_CENTER_CELL, plus two segments with no free rays and
     no surfaces (NO_VALID_RAYS) and four segments with the free rays but no surfaces;
  R  CPU-replayed B1-R packets (MOTAR-72x4-RUNTIME through the certifier core), the 16 G-alpha candidates at every
     decision step of the sampled episodes;
  C  on the same packets, two certifier cores (MOTAR-72x4-RUNTIME and MOTAR-72x4-RUNTIME-NATIVE) must return identical
     step reports and identical certify results (whole dictionaries except the `config` name).
PASS iff every comparison is identical, every part ran to completion, and the positive controls hold: certified,
OBSERVED_SURFACE_INTERSECTION, UNRESOLVED_CENTER_CELL, NODE_BUDGET_UNKNOWN and NO_VALID_RAYS each occur at least
once in S, and R
has at least one certified and one refused segment.
usage: m3_native_equivalence.py OUT_JSON N_SCENES WORKERS N_EPISODES_PER_CELL CELL_DIR [CELL_DIR ...]
"""
import concurrent.futures as cf
import json
import math
import multiprocessing as mp
import pathlib
import random
import struct
import sys
import time

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "motar"))
sys.path.insert(0, str(HERE.parents[1]))
from harness import CATALOG, bar, core, scan, target, header   # noqa: E402  (puts src/ on sys.path)
import coverage_b as CB                                         # noqa: E402
import coverage_native as CN                                    # noqa: E402
import registry as REG                                          # noqa: E402

BASE_H, BASE_S = 4_000_000, 5_000_000
NATIVE_CONFIG = "MOTAR-72x4-RUNTIME-NATIVE"
K = 16
SAMPLE_SEED = 20261106


def bits(x):
    return struct.pack("<d", x)


def same(a, b):
    return bits(a) == bits(b) or (math.isnan(a) and math.isnan(b))


def part_h(n):
    lib = CN.load()
    rng = random.Random(BASE_H)
    specials = [0.0, -0.0, 5e-324, -5e-324, 1e-300, 1e300, 1.7976931348623157e308, 1e-15, 1e-9, 1.0, -1.0,
                0.1, 3.0, float("inf"), float("-inf"), float("nan")]

    def num():
        k = rng.random()
        if k < 0.1:
            return rng.choice(specials)
        if k < 0.4:
            return rng.uniform(-10, 10)
        if k < 0.7:
            return rng.uniform(-1, 1) * 10 ** rng.uniform(-12, 3)
        a = rng.uniform(-10, 10)
        return a - (a + rng.uniform(-1e-6, 1e-6))          # cancellation residue
    st = {"hypot": 0, "hypot_mismatch": 0, "point_segment": 0, "ps_mismatch": 0, "sbd": 0, "sbd_mismatch": 0,
          "crosses": 0, "crosses_mismatch": 0, "examples": []}
    for _ in range(n):
        x, y = num(), num()
        st["hypot"] += 1
        if not same(lib.cov_py_hypot(x, y), math.hypot(x, y)):
            st["hypot_mismatch"] += 1
            if len(st["examples"]) < 10:
                st["examples"].append(["hypot", repr(x), repr(y)])

    def fin():
        k = rng.random()
        if k < 0.15:
            return rng.choice([0.0, -0.0, 1.0, -1.0, 0.5, 2.0])
        if k < 0.25:
            return round(rng.uniform(-5, 5), 1)               # lattice values: exact ties and touching boxes
        return rng.uniform(-5, 5)
    for _ in range(n // 4):
        a = (fin(), fin())
        r = rng.random()
        if r < 0.15:
            b = a                                             # zero-length segment
        elif r < 0.3:
            b = (a[0], fin())                                 # vertical
        elif r < 0.45:
            b = (fin(), a[1])                                 # horizontal
        elif r < 0.5:
            b = (a[0] + rng.uniform(-1e-16, 1e-16), fin())    # |d| < 1e-15 branch
        else:
            b = (fin(), fin())
        p = (fin(), fin())
        x0, y0 = fin(), fin()
        box = (x0, y0, x0 + abs(fin()), y0 + abs(fin()))
        st["point_segment"] += 1
        if not same(lib.cov_point_segment(p[0], p[1], a[0], a[1], b[0], b[1]), CB.point_segment(p, a, b)):
            st["ps_mismatch"] += 1
            if len(st["examples"]) < 10:
                st["examples"].append(["point_segment", p, a, b])
        st["sbd"] += 1
        if not same(lib.cov_segment_box_distance(a[0], a[1], b[0], b[1], *box), CB.segment_box_distance(a, b, box)):
            st["sbd_mismatch"] += 1
            if len(st["examples"]) < 10:
                st["examples"].append(["segment_box_distance", a, b, box])
        for strict in (False, True):
            st["crosses"] += 1
            if bool(lib.cov_crosses(a[0], a[1], b[0], b[1], *box, int(strict))) != bool(CB.crosses(a, b, box, strict)):
                st["crosses_mismatch"] += 1
                if len(st["examples"]) < 10:
                    st["examples"].append(["crosses", a, b, box, strict])
    return st


def part_t():
    """Threshold probes built at the code's own constants: the strict-crossing margin 1e-9 on all four box edges, the
    parallel test |d| < 1e-15, the surface rule <= R + 1e-9 (exact distances: a point straight above a horizontal
    segment has point_segment == |dy| exactly) and the prune rule through small segments. Whole results compared."""
    lib = CN.load()
    st = {"strict": 0, "strict_mismatch": 0, "strict_true": 0, "strict_false": 0, "parallel": 0,
          "parallel_mismatch": 0, "surface": 0, "surface_mismatch": 0, "surface_hit": 0, "surface_miss": 0,
          "examples": []}
    deltas = [0.0, 5e-11, 1e-10, 5e-10, 9.9e-10, 1e-9, 1.01e-9, 1.5e-9, 2e-9, 1e-8]
    rng = random.Random(BASE_H + 1)
    for _ in range(400):
        x0, y0 = round(rng.uniform(-3, 3), 2), round(rng.uniform(-3, 3), 2)
        box = (x0, y0, x0 + round(rng.uniform(0.05, 1.0), 2), y0 + round(rng.uniform(0.05, 1.0), 2))
        xm, ym = (box[0] + box[2]) / 2, (box[1] + box[3]) / 2
        for d in deltas:
            for sgn in (1.0, -1.0):
                segs = [((box[0] - 1.0, box[1] + sgn * d), (box[2] + 1.0, box[1] + sgn * d)),
                        ((box[0] - 1.0, box[3] - sgn * d), (box[2] + 1.0, box[3] - sgn * d)),
                        ((box[0] + sgn * d, box[1] - 1.0), (box[0] + sgn * d, box[3] + 1.0)),
                        ((box[2] - sgn * d, box[1] - 1.0), (box[2] - sgn * d, box[3] + 1.0)),
                        ((xm, box[1] - 1.0), (xm + sgn * d, box[3] + 1.0))]
                for a, b in segs:
                    for strict in (True, False):
                        py = bool(CB.crosses(a, b, box, strict))
                        st["strict"] += 1
                        st["strict_true" if py else "strict_false"] += strict
                        if bool(lib.cov_crosses(a[0], a[1], b[0], b[1], *box, int(strict))) != py:
                            st["strict_mismatch"] += 1
                            if len(st["examples"]) < 10:
                                st["examples"].append(["crosses", a, b, box, strict])
        for dd in (0.0, 5e-16, 9.99e-16, 1e-15, 1.0000001e-15, 2e-15, 1e-14):
            for a, b in (((xm, box[1] - 1.0), (xm + dd, box[3] + 1.0)), ((box[0] - 1.0, ym), (box[2] + 1.0, ym + dd)),
                         ((box[0] - 1e-3, box[1] - 1.0), (box[0] - 1e-3 + dd, box[3] + 1.0))):
                for strict in (True, False):
                    st["parallel"] += 1
                    if bool(lib.cov_crosses(a[0], a[1], b[0], b[1], *box, int(strict))) != bool(
                            CB.crosses(a, b, box, strict)):
                        st["parallel_mismatch"] += 1
    R = REG.CONFIGS["SYNTH-72x1"].plan_radius
    res, budget = REG.CONFIGS["SYNTH-72x1"].resolution, REG.CONFIGS["SYNTH-72x1"].max_nodes
    edge = R + 1e-9
    ups = [edge, math.nextafter(edge, 1.0) if hasattr(math, "nextafter") else edge * (1 + 2 ** -52),
           edge * (1 - 2 ** -53), R, edge + 1e-12, edge - 1e-12]
    free = [((-3.0, -2.0), (3.0, -2.0)), ((-3.0, 2.0), (3.0, 2.0))]
    for y in ups:
        for sgn in (1.0, -1.0):
            seg = ((0.0, 0.0), (1.0, 0.0))
            surf = [(0.5, sgn * y)]
            rv = CB._coverage_B(seg, free, surf, REG.BAR_HALF_SIZES[:2], R, res, budget)
            rn = CN.coverage_B_native(seg, free, surf, REG.BAR_HALF_SIZES[:2], R, res, budget)
            st["surface"] += 1
            st["surface_hit" if rv["reason"] == "OBSERVED_SURFACE_INTERSECTION" else "surface_miss"] += 1
            if json.dumps(rv, sort_keys=True) != json.dumps(rn, sort_keys=True):
                st["surface_mismatch"] += 1
                if len(st["examples"]) < 10:
                    st["examples"].append(["surface", y, rv, rn])
    return st


def scene(seed):
    rng = random.Random(BASE_S + seed)
    layout = ("P0R-72x1", "MOTAR-72x4")[seed % 2]
    z = rng.uniform(0.3, 2.7)
    pose = (rng.uniform(-1, 1), rng.uniform(-1, 1), z, rng.uniform(-math.pi, math.pi))
    objs = []
    for _ in range(rng.randint(0, 8)):
        hx, hy = rng.choice(CATALOG)
        r, b = rng.uniform(0.8, 6.0), rng.uniform(-math.pi, math.pi)
        ob = bar(pose[0] + r * math.cos(b), pose[1] + r * math.sin(b), hx, hy)
        dx = max(abs(pose[0] - ob["c"][0]) - hx, 0.0)
        dy = max(abs(pose[1] - ob["c"][1]) - hy, 0.0)
        if math.hypot(dx, dy) > 0.35:
            objs.append(ob)
    ta, td = rng.uniform(-math.pi, math.pi), rng.uniform(1.5, 6.0)
    objs.append(target(pose[0] + td * math.cos(ta), pose[1] + td * math.sin(ta), z))
    segs = []
    for i in range(K):
        if i < 4 and len(objs) > 1:
            ob = objs[rng.randrange(len(objs) - 1)]
            ang = math.atan2(ob["c"][1] - pose[1], ob["c"][0] - pose[0]) + rng.uniform(-0.4, 0.4)
        else:
            ang = rng.uniform(-math.pi, math.pi)
        L = rng.choice([0.0, rng.uniform(0.3, 3.0), rng.uniform(0.3, 3.0), rng.uniform(0.3, 3.0)])
        segs.append(((pose[0], pose[1]), (pose[0] + L * math.cos(ang), pose[1] + L * math.sin(ang))))
    return layout, pose, objs, segs


def compare(rec, segs, free, hits, settings, tag):
    for name, (R, res, budget) in settings.items():
        for sg in segs:
            t0 = time.perf_counter()
            rv = CB._coverage_B(sg, free, hits, REG.BAR_HALF_SIZES, R, res, budget)
            t1 = time.perf_counter()
            rn = CN.coverage_B_native(sg, free, hits, REG.BAR_HALF_SIZES, R, res, budget)
            t2 = time.perf_counter()
            rec["segments"] += 1
            rec["ms_V"] += 1e3 * (t1 - t0)
            rec["ms_N"] += 1e3 * (t2 - t1)
            key = name + ":" + rv["reason"]
            rec["reasons"][key] = rec["reasons"].get(key, 0) + 1
            if json.dumps(rv, sort_keys=True) != json.dumps(rn, sort_keys=True):
                rec["mismatch"] += 1
                if len(rec["examples"]) < 10:
                    rec["examples"].append({"tag": tag, "setting": name, "seg": sg, "V": rv, "N": rn})


def part_s(seeds):
    cfg = REG.CONFIGS["SYNTH-72x1"]
    settings = {"qualified": (cfg.plan_radius, cfg.resolution, cfg.max_nodes),
                "budget60": (cfg.plan_radius, cfg.resolution, 60),
                "res0.004": (cfg.plan_radius, 0.004, cfg.max_nodes)}
    rec = {"scenes": 0, "skipped_incomplete": 0, "segments": 0, "mismatch": 0, "ms_V": 0.0, "ms_N": 0.0,
           "reasons": {}, "examples": []}
    for seed in seeds:
        layout, pose, objs, segs = scene(seed)
        r, s, _ = scan(objs, pose, layout)
        c = core.CertifierCore("SYNTH-72x1" if layout == "P0R-72x1" else "SYNTH-72x4", 0, 0)
        rep = c.step(r, s, *header(1, pose))
        cur = c._current                                    # test-side read of the core's own packet snapshot
        rec["scenes"] += 1
        if not rep.get("accepted") or cur is None or not cur[2]:
            rec["skipped_incomplete"] += 1
            continue
        compare(rec, segs, list(cur[3]), list(cur[4]), settings, "S%d" % seed)
        compare(rec, segs[:2], [], [], {"nofree": settings["qualified"]}, "S%d-nofree" % seed)
        compare(rec, segs[4:8], list(cur[3]), [], {"nosurf": settings["qualified"]}, "S%d-nosurf" % seed)
    return rec


def part_r(items):
    import adapter as AD
    import replay as RP
    import galpha_replay as GA
    from motar_eval import occlusion_trace as OT
    cfg = REG.CONFIGS[GA.CONFIG]
    settings = {"qualified": (cfg.plan_radius, cfg.resolution, cfg.max_nodes)}
    rec = {"episodes": 0, "decision_steps": 0, "segments": 0, "mismatch": 0, "ms_V": 0.0, "ms_N": 0.0,
           "reasons": {}, "examples": [], "core_steps": 0, "core_step_mismatch": 0, "core_segments": 0,
           "core_certified": 0, "core_certify_mismatch": 0}
    for it in items:
        tr = OT.read_trace(it["cell"])
        fr = OT.episode_frames(tr["frames"])[(it["env"], it["episode"])]
        fr = fr[np.argsort(fr["frame"], kind="stable")]
        look = OT.reset_lookup(tr["layouts"])
        c, h, q = (np.array(a, float) for a in OT.layout_arrays(tr["layouts"], look[(it["env"], it["episode"])]))
        sc = RP.Scene(c, h, q)
        cc = core.CertifierCore(GA.CONFIG, it["env"], it["episode"])
        cn = core.CertifierCore(NATIVE_CONFIG, it["env"], it["episode"])
        for x in fr:
            k = int(x["frame"])
            p, tq, tp = (np.array(x[n], float) for n in ("p_pos", "p_quat", "t_pos"))
            yaw, sq = RP.sensor_yaw(tq)
            arrs = AD.packet(sc.scan(p.astype(np.float32), sq, tp.astype(np.float32)), k, it["env"], it["episode"],
                             p, yaw)
            rep = cc.step(*arrs)
            rep_n = cn.step(*arrs)
            rec["core_steps"] += 1
            if json.dumps(rep, sort_keys=True) != json.dumps(rep_n, sort_keys=True):
                rec["core_step_mismatch"] += 1
            if k % GA.CERT_EVERY or not rep.get("accepted") or math.hypot(*(tp[:2] - p[:2])) <= GA.TERMINAL_M:
                continue
            segs = GA.candidates(p, tp)[0]
            paths = np.array(segs, dtype=np.float64)
            cv, cnr = cc.certify(paths, rep["step"]), cn.certify(paths, rep_n["step"])
            strip = lambda L: [{kk: vv for kk, vv in d.items() if kk != "config"} for d in L]   # noqa: E731
            rec["core_segments"] += len(cv)
            rec["core_certified"] += sum(d["static_bar_certified"] for d in cv)
            if json.dumps(strip(cv), sort_keys=True) != json.dumps(strip(cnr), sort_keys=True) or \
                    any(d["config"] != NATIVE_CONFIG for d in cnr):
                rec["core_certify_mismatch"] += 1
            cur = cc._current
            if not cur[2]:
                continue
            rec["decision_steps"] += 1
            compare(rec, segs, list(cur[3]), list(cur[4]), settings,
                    "%s/%d/%d/%d" % (pathlib.Path(it["cell"]).name, it["env"], it["episode"], k))
        rec["episodes"] += 1
    return rec


def merge(parts):
    st = {}
    for p in parts:
        for k, v in p.items():
            if isinstance(v, dict):
                d = st.setdefault(k, {})
                for kk, vv in v.items():
                    d[kk] = d.get(kk, 0) + vv
            elif isinstance(v, list):
                st.setdefault(k, []).extend(v)
            else:
                st[k] = st.get(k, 0) + v
    if "examples" in st:
        st["examples"] = st["examples"][:10]
    return st


def main():
    out = pathlib.Path(sys.argv[1])
    n, workers, n_ep, cells = int(sys.argv[2]), int(sys.argv[3]), int(sys.argv[4]), sys.argv[5:]
    if out.exists():
        raise SystemExit("refusing to overwrite " + str(out))
    t0 = time.time()
    lib_ok = CN.load() is not None
    failure = None
    H = S = Rr = Tt = None
    if lib_ok:
        from motar_eval import occlusion_trace as OT
        rng = random.Random(SAMPLE_SEED)
        items = []
        for cell in cells:
            tr = OT.read_trace(cell)
            keys = sorted(k for k in OT.episode_frames(tr["frames"]) if k in OT.reset_lookup(tr["layouts"]))
            for env, ep in rng.sample(keys, min(n_ep, len(keys))):
                items.append({"cell": cell, "env": env, "episode": ep})
        seeds = list(range(n))
        s_chunks = [c for c in (seeds[i::workers * 4] for i in range(workers * 4)) if c]
        r_chunks = [c for c in (items[i::workers] for i in range(workers)) if c]
        try:
            with cf.ProcessPoolExecutor(max_workers=workers, mp_context=mp.get_context("spawn")) as ex:
                fh = ex.submit(part_h, 200_000)
                ft = ex.submit(part_t)
                fs = [ex.submit(part_s, c) for c in s_chunks]
                fr = [ex.submit(part_r, c) for c in r_chunks]
                H, S, Rr = fh.result(), merge([f.result() for f in fs]), merge([f.result() for f in fr])
                Tt = ft.result()
        except Exception as e:
            failure = type(e).__name__ + ": " + str(e)[:300]
    sr = (S or {}).get("reasons", {})
    checks = {
        "library_loaded": lib_ok,
        "completed": failure is None and H is not None and Tt is not None and S is not None and Rr is not None
                     and S.get("scenes") == n and Rr.get("episodes") == len(cells) * n_ep,
        "H_identical": bool(H) and H["hypot_mismatch"] == 0 and H["ps_mismatch"] == 0 and H["sbd_mismatch"] == 0
                       and H["crosses_mismatch"] == 0,
        "T_identical": bool(Tt) and Tt["strict_mismatch"] == 0 and Tt["parallel_mismatch"] == 0
                       and Tt["surface_mismatch"] == 0,
        "T_controls": bool(Tt) and Tt["strict_true"] > 0 and Tt["strict_false"] > 0 and Tt["surface_hit"] > 0
                      and Tt["surface_miss"] > 0,
        "S_identical": bool(S) and S.get("mismatch", 1) == 0 and S.get("segments", 0) > 0,
        "R_identical": bool(Rr) and Rr.get("mismatch", 1) == 0 and Rr.get("segments", 0) > 0,
        "C_core_identical": bool(Rr) and Rr.get("core_step_mismatch", 1) == 0 and Rr.get("core_certify_mismatch", 1) == 0
                            and Rr.get("core_segments", 0) > 0 and 0 < Rr.get("core_certified", 0) < Rr["core_segments"],
        "S_controls": all(any(k.endswith(":" + r) for k in sr) for r in (
            "ALL_COLLIDING_CENTER_CELLS_EXCLUDED", "OBSERVED_SURFACE_INTERSECTION", "UNRESOLVED_CENTER_CELL",
            "NODE_BUDGET_UNKNOWN", "NO_VALID_RAYS")),
        "R_controls": bool(Rr) and Rr.get("reasons", {}).get("qualified:ALL_COLLIDING_CENTER_CELLS_EXCLUDED", 0) > 0
                      and sum(v for k, v in Rr.get("reasons", {}).items()
                              if k != "qualified:ALL_COLLIDING_CENTER_CELLS_EXCLUDED") > 0,
    }
    res = {"gate": "M3 native coverage equivalence", "pass": all(checks.values()), "checks": checks,
           "H": H, "T": Tt, "S": S, "R": Rr, "n_scenes": n, "cells": cells, "episodes_per_cell": n_ep,
           "sample_seed": SAMPLE_SEED, "library": CN.path(), "failure": failure,
           "mean_ms_per_segment": {p: {v: (d["ms_" + v] / d["segments"] if d and d.get("segments") else None)
                                       for v in "VN"} for p, d in (("S", S), ("R", Rr))},
           "wall_s": time.time() - t0}
    with out.open("x") as fh:
        json.dump(res, fh, indent=1, default=float)
        fh.write("\n")
    print(json.dumps({k: res[k] for k in ("pass", "checks", "mean_ms_per_segment", "failure")}, indent=1,
                     default=float))
    return 0 if res["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
