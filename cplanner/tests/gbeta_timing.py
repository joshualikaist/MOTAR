"""M3 gate G-beta: per-decision compute of the certificate on CPU-replayed B1-R packets, one process, with the native
coverage configuration (docs/plans/prereg_cplanner_m3_v1.md; threshold in docs/plans/cplanner_charter_v1.md §3).

Per sampled episode, every recorded frame is replayed (not timed) and then, timed with time.perf_counter:
  adapter  AD.packet (normalised scan -> P0-R arrays)
  step     core.step on MOTAR-72x4-RUNTIME-NATIVE
  cert8    core.certify of the first 8 G-alpha candidate segments (offsets 0, +-20, 40 deg x 1 m, 2 m)
  cert16   core.certify of all 16 G-alpha candidate segments
at every decision step (even frame, accepted packet, target centre > 0.9 m), as in G-alpha. total_K = adapter + step +
certK of the same frame. One process pinned to cpu PIN_CPU (a P-core thread). Reading: PASS_K16 if
p95(total_16) <= 50 ms; else PASS_K8_ONLY if p95(total_8) <= 50 ms; else FAIL; FAIL_INCOMPLETE if any episode did
not finish, no decision step was timed or the pinning failed.
Reported, not gated: C-Planner skeleton act() time in OFF and FILTER modes on the same scans (belief = recorded target
position and velocity `t_vel`; information only: the timing input of a stand-in tracker, never evidence).
usage: gbeta_timing.py OUT_JSON EPISODES_PER_CELL CELL_DIR [CELL_DIR ...]
"""
import json
import math
import os
import pathlib
import random
import sys
import time

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
for _p in (HERE.parent / "planner", HERE.parent / "motar", HERE.parent / "src", HERE.parents[1]):
    sys.path.insert(0, str(_p))
import adapter as AD                                 # noqa: E402
import c_planner as CP                               # noqa: E402
import core                                          # noqa: E402
import coverage_native as CN                         # noqa: E402
import galpha_replay as GA                           # noqa: E402
import replay as RP                                  # noqa: E402
from motar_eval import occlusion_trace as OT         # noqa: E402

CONFIG = "MOTAR-72x4-RUNTIME-NATIVE"
PIN_CPU = 8                     # one P-core thread inside the allowed set (charter §7); "single core" for G-beta
SAMPLE_SEED = 20261107
LIMIT_MS = 50.0


def q(xs):
    if not xs:
        return None
    xs = sorted(xs)
    pick = lambda p: xs[min(len(xs) - 1, int(p * len(xs)))]  # noqa: E731
    return {"n": len(xs), "p50": pick(.5), "p95": pick(.95), "p99": pick(.99), "max": xs[-1]}


def main():
    out = pathlib.Path(sys.argv[1])
    n, cells = int(sys.argv[2]), sys.argv[3:]
    if out.exists():
        raise SystemExit("refusing to overwrite " + str(out))
    t_start = time.time()
    try:
        os.sched_setaffinity(0, {PIN_CPU})
    except OSError:
        pass
    affinity = sorted(os.sched_getaffinity(0))
    lib_ok = CN.load() is not None
    rng = random.Random(SAMPLE_SEED)
    items = []
    for cell in cells:
        tr = OT.read_trace(cell)
        keys = sorted(k for k in OT.episode_frames(tr["frames"]) if k in OT.reset_lookup(tr["layouts"]))
        for env, ep in rng.sample(keys, min(n, len(keys))):
            items.append((cell, env, ep))
    T = {"adapter": [], "step": [], "cert8": [], "cert16": [], "total8": [], "total16": [], "act_OFF": [],
         "act_FILTER": []}
    st = {"episodes": 0, "frames": 0, "decision_steps": 0, "certified8": 0, "certified16": 0, "segments16": 0,
          "reasons16": {}, "planner_counts": {"OFF": {}, "FILTER": {}}}
    failure = None
    try:
        if not lib_ok:
            raise RuntimeError("native coverage library not loadable: " + CN.path())
        for cell, env, ep in items:
            tr = OT.read_trace(cell)
            fr = OT.episode_frames(tr["frames"])[(env, ep)]
            fr = fr[np.argsort(fr["frame"], kind="stable")]
            look = OT.reset_lookup(tr["layouts"])
            c, h, qq = (np.array(a, float) for a in OT.layout_arrays(tr["layouts"], look[(env, ep)]))
            sc = RP.Scene(c, h, qq)
            cc = core.CertifierCore(CONFIG, env, ep)
            planners = {m: CP.CPlanner(env, ep, m, config=CONFIG) for m in CP.MODES}
            for x in fr:
                k = int(x["frame"])
                p, tq, tp = (np.array(x[nm], float) for nm in ("p_pos", "p_quat", "t_pos"))
                v_p, v_t = np.array(x["p_vel"], float), np.array(x["t_vel"], float)
                yaw, sq = RP.sensor_yaw(tq)
                norm = sc.scan(p.astype(np.float32), sq, tp.astype(np.float32))
                t0 = time.perf_counter()
                arrs = AD.packet(norm, k, env, ep, p, yaw)
                t1 = time.perf_counter()
                rep = cc.step(*arrs)
                t2 = time.perf_counter()
                T["adapter"].append(1e3 * (t1 - t0))
                T["step"].append(1e3 * (t2 - t1))
                st["frames"] += 1
                for m, pl in planners.items():
                    a0 = time.perf_counter()
                    pl.act(k, p, v_p, yaw, tp, v_t, True, norm)
                    T["act_" + m].append(1e3 * (time.perf_counter() - a0))
                if k % GA.CERT_EVERY or not rep.get("accepted") or math.hypot(*(tp[:2] - p[:2])) <= GA.TERMINAL_M:
                    continue
                paths = np.array(GA.candidates(p, tp)[0], dtype=np.float64)
                c0 = time.perf_counter()
                r8 = cc.certify(paths[:8], rep["step"])
                c1 = time.perf_counter()
                r16 = cc.certify(paths, rep["step"])
                c2 = time.perf_counter()
                T["cert8"].append(1e3 * (c1 - c0))
                T["cert16"].append(1e3 * (c2 - c1))
                base = 1e3 * (t2 - t0)
                T["total8"].append(base + 1e3 * (c1 - c0))
                T["total16"].append(base + 1e3 * (c2 - c1))
                st["decision_steps"] += 1
                st["certified8"] += sum(d["static_bar_certified"] for d in r8)
                st["certified16"] += sum(d["static_bar_certified"] for d in r16)
                st["segments16"] += len(r16)
                for d in r16:
                    st["reasons16"][d["reason"]] = st["reasons16"].get(d["reason"], 0) + 1
            for m, pl in planners.items():
                for kk, vv in pl.counts.items():
                    st["planner_counts"][m][kk] = st["planner_counts"][m].get(kk, 0) + vv
            st["episodes"] += 1
    except Exception as e:
        failure = type(e).__name__ + ": " + str(e)[:300]
    timing = {k: q(v) for k, v in T.items()}
    if affinity != [PIN_CPU]:
        failure = failure or "affinity %s, expected [%d]" % (affinity, PIN_CPU)
    if failure or st["episodes"] != len(items) or not T["total16"]:
        reading = "FAIL_INCOMPLETE"
    elif timing["total16"]["p95"] <= LIMIT_MS:
        reading = "PASS_K16"
    elif timing["total8"]["p95"] <= LIMIT_MS:
        reading = "PASS_K8_ONLY"
    else:
        reading = "FAIL"
    res = {"gate": "M3 G-beta", "reading": reading, "limit_ms": LIMIT_MS, "timing_ms": timing, "stats": st,
           "config": CONFIG, "library": CN.path(), "affinity": affinity, "sample_seed": SAMPLE_SEED, "episodes_per_cell": n,
           "cells": cells, "items": [[pathlib.Path(c).parent.name + "/" + pathlib.Path(c).name, e, p]
                                     for c, e, p in items],
           "failure": failure, "planner_params": {k: getattr(CP, k) for k in ("MAX_FILTER_TRIES", "N_POINTS")},
           "wall_s": time.time() - t_start}
    with out.open("x") as fh:
        json.dump(res, fh, indent=1, default=float)
        fh.write("\n")
    print(json.dumps({"reading": reading, "total16": timing["total16"], "total8": timing["total8"],
                      "step": timing["step"], "act_OFF": timing["act_OFF"], "act_FILTER": timing["act_FILTER"],
                      "failure": failure}, indent=1, default=float))
    return 0 if reading in ("PASS_K16", "PASS_K8_ONLY", "FAIL") else 1


if __name__ == "__main__":
    raise SystemExit(main())
