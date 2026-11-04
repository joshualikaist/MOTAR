"""M2 gate G-alpha: certifiability of the P0-R certificate on CPU-replayed B1-R trajectories
(docs/plans/prereg_cplanner_m2_v1.md; thresholds in docs/plans/cplanner_charter_v1.md §3).

Per sampled episode: every recorded frame is replayed (cplanner/motar/replay.py), converted by the adapter and ingested
by one `CertifierCore("MOTAR-72x4-RUNTIME", env, episode)` in frame order. At every CERT_EVERY-th frame whose target
centre is more than TERMINAL_M away, 16 candidate segments from the vehicle position are certified: headings at
OFFSETS_DEG from the bearing to the recorded target position, lengths 1 m and 2 m. A candidate is "progressing" if its
end point is at least PROGRESS_M closer to the target than the vehicle. The decision step counts as certifiable if at
least one progressing candidate is STATIC_BAR_CERTIFIED.

The recorded target position is used as the intercept proxy (the replay has no tracker mean); it is a measurement
device here, never a control input. Ground truth for FALSE_CERT: every active bar of the recorded layout
(axis-aligned, 0-3 m) with an exact 2-D segment-rectangle distance; FALSE_CERT if a certified segment comes within
R = 0.45 m of a bar.

usage: galpha_replay.py OUT_JSON EPISODES_PER_CELL WORKERS CELL_DIR [CELL_DIR ...]
"""
import concurrent.futures as cf
import hashlib
import json
import os
import math
import multiprocessing as mp
import pathlib
import random
import sys
import time

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "motar"))
sys.path.insert(0, str(HERE.parent / "src"))
sys.path.insert(0, str(HERE.parents[1]))
import adapter as AD                                 # noqa: E402
import core                                          # noqa: E402
import replay as RP                                  # noqa: E402
from motar_eval import occlusion_trace as OT         # noqa: E402

# The M2 gate ran MOTAR-72x4-RUNTIME; the direction-bound repair (docs/plans/prereg_cplanner_dirbound_v1.md) reruns
# this script on its configurations through CPLANNER_GALPHA_CONFIG (read here so spawned workers see it).
CONFIG = os.environ.get("CPLANNER_GALPHA_CONFIG", "MOTAR-72x4-RUNTIME")
if CONFIG not in ("MOTAR-72x4-RUNTIME", "MOTAR-72x4-RUNTIME-NATIVE", "MOTAR-72x4-RUNTIME-DIR",
                  "MOTAR-72x4-RUNTIME-DIR-NATIVE"):
    raise SystemExit("unsupported CPLANNER_GALPHA_CONFIG " + CONFIG)
R_PLAN = 0.45
TERMINAL_M = 0.9
CERT_EVERY = 2
OFFSETS_DEG = (0.0, 20.0, -20.0, 40.0, -40.0, 70.0, -70.0, 180.0)
LENGTHS = (1.0, 2.0)
PROGRESS_M = 0.1
SAMPLE_SEED = 20261105
THRESH = {"filter_min": 0.60, "monitor_min": 0.20}


def seg_rect_dist(a, b, cx, cy, hx, hy):
    x0, x1, y0, y1 = cx - hx, cx + hx, cy - hy, cy + hy
    lo, hi = 0.0, 1.0
    inside = True
    for p, d, mn, mx in ((a[0], b[0] - a[0], x0, x1), (a[1], b[1] - a[1], y0, y1)):
        if abs(d) < 1e-15:
            if p < mn or p > mx:
                inside = False
                break
        else:
            t0, t1 = sorted(((mn - p) / d, (mx - p) / d))
            lo, hi = max(lo, t0), min(hi, t1)
            if lo > hi:
                inside = False
                break
    if inside:
        return 0.0

    def pseg(px, py):
        dx, dy = b[0] - a[0], b[1] - a[1]
        den = dx * dx + dy * dy
        t = 0.0 if den == 0 else max(0.0, min(1.0, ((px - a[0]) * dx + (py - a[1]) * dy) / den))
        return math.hypot(px - a[0] - t * dx, py - a[1] - t * dy)
    vals = [pseg(x, y) for x in (x0, x1) for y in (y0, y1)]
    vals += [math.hypot(max(x0 - p[0], 0.0, p[0] - x1), max(y0 - p[1], 0.0, p[1] - y1)) for p in (a, b)]
    return min(vals)


def candidates(p, tgt):
    beta = math.atan2(tgt[1] - p[1], tgt[0] - p[0])
    d0 = math.hypot(tgt[0] - p[0], tgt[1] - p[1])
    segs, prog = [], []
    for off in OFFSETS_DEG:
        for L in LENGTHS:
            a = beta + math.radians(off)
            e = (p[0] + L * math.cos(a), p[1] + L * math.sin(a))
            segs.append(((p[0], p[1]), e))
            prog.append(math.hypot(tgt[0] - e[0], tgt[1] - e[1]) <= d0 - PROGRESS_M)
    return segs, prog


def run_episodes(items):
    st = {"episodes": 0, "frames": 0, "steps_accepted": 0, "steps_complete": 0, "decision_steps": 0,
          "certifiable_steps": 0, "segments": 0, "certified": 0, "certified_progressing": 0, "false_cert": 0,
          "reasons": {}, "by_cell_group": {}, "by_distance_bin": {}, "exceptions": 0, "ms_step": [],
          "ms_certify16": [], "examples": []}
    for it in items:
        tr = OT.read_trace(it["cell"])
        fr = OT.episode_frames(tr["frames"])[(it["env"], it["episode"])]
        fr = fr[np.argsort(fr["frame"], kind="stable")]
        look = OT.reset_lookup(tr["layouts"])
        c, h, q = OT.layout_arrays(tr["layouts"], look[(it["env"], it["episode"])])
        c, h, q = np.array(c, float), np.array(h, float), np.array(q, float)
        if np.abs(q - np.array([0, 0, 0, 1.0])).max() > 1e-6:
            raise ValueError("non-identity bar orientation; the axis-aligned oracle does not apply")
        scene = RP.Scene(c, h, q)
        cc = core.CertifierCore(CONFIG, it["env"], it["episode"])
        group = it["group"]
        g = st["by_cell_group"].setdefault(group, {"decision_steps": 0, "certifiable_steps": 0, "segments": 0,
                                                   "certified": 0})
        for x in fr:
            k = int(x["frame"])
            p, tq, tp = np.array(x["p_pos"], float), np.array(x["p_quat"], float), np.array(x["t_pos"], float)
            yaw, sq = RP.sensor_yaw(tq)
            norm = scene.scan(p.astype(np.float32), sq, tp.astype(np.float32))
            arrs = AD.packet(norm, k, it["env"], it["episode"], p, yaw)
            t0 = time.perf_counter()
            rep = cc.step(*arrs)
            st["ms_step"].append(1e3 * (time.perf_counter() - t0))
            st["frames"] += 1
            st["steps_accepted"] += bool(rep.get("accepted"))
            st["steps_complete"] += bool(rep.get("complete"))
            d_t = math.hypot(tp[0] - p[0], tp[1] - p[1])
            if k % CERT_EVERY or d_t <= TERMINAL_M or not rep.get("accepted"):
                continue
            segs, prog = candidates(p, tp)
            t0 = time.perf_counter()
            res = cc.certify(np.array(segs, np.float64), k)
            st["ms_certify16"].append(1e3 * (time.perf_counter() - t0))
            ok_any = False
            for s, pr, r in zip(segs, prog, res):
                st["segments"] += 1
                g["segments"] += 1
                st["reasons"][r["reason"]] = st["reasons"].get(r["reason"], 0) + 1
                if r["static_bar_certified"]:
                    st["certified"] += 1
                    g["certified"] += 1
                    st["certified_progressing"] += pr
                    ok_any = ok_any or pr
                    dmin = min(seg_rect_dist(s[0], s[1], c[b, 0], c[b, 1], h[b, 0], h[b, 1]) for b in range(len(c)))
                    if dmin <= R_PLAN:
                        st["false_cert"] += 1
                        if len(st["examples"]) < 20:
                            st["examples"].append({"cell": it["cell"], "env": it["env"], "episode": it["episode"],
                                                   "frame": k, "segment": s, "bar_clearance": dmin})
            st["decision_steps"] += 1
            st["certifiable_steps"] += ok_any
            g["decision_steps"] += 1
            g["certifiable_steps"] += ok_any
            b = "%d-%d" % (min(int(d_t // 2) * 2, 10), min(int(d_t // 2) * 2, 10) + 2)
            db = st["by_distance_bin"].setdefault(b, [0, 0])
            db[0] += 1
            db[1] += ok_any
        st["exceptions"] += len(cc.exceptions)
        st["episodes"] += 1
    return st


def q(xs):
    xs = sorted(xs)
    if not xs:
        return None
    pick = lambda p: xs[min(len(xs) - 1, int(p * len(xs)))]  # noqa: E731
    return {"n": len(xs), "p50": pick(.5), "p95": pick(.95), "p99": pick(.99), "max": xs[-1]}


def main():
    out = pathlib.Path(sys.argv[1])
    n, workers, cells = int(sys.argv[2]), int(sys.argv[3]), sys.argv[4:]
    if out.exists():
        raise SystemExit("refusing to overwrite " + str(out))
    t0 = time.time()
    rng = random.Random(SAMPLE_SEED)
    items = []
    for cell in cells:
        tr = OT.read_trace(cell)
        keys = sorted(OT.episode_frames(tr["frames"]).keys())
        look = OT.reset_lookup(tr["layouts"])
        keys = [k for k in keys if k in look]
        name = pathlib.Path(cell).name
        arm = pathlib.Path(cell).parent.name
        bars = int(name.split("__")[1].replace("bars", ""))
        for env, ep in rng.sample(keys, min(n, len(keys))):
            items.append({"cell": cell, "env": env, "episode": ep, "group": "%s:%d" % (arm, bars)})
    chunks = [items[i::workers * 4] for i in range(workers * 4)]
    chunks = [c for c in chunks if c]
    parts, failure = [], None
    try:
        with cf.ProcessPoolExecutor(max_workers=workers, mp_context=mp.get_context("spawn")) as ex:
            parts = list(ex.map(run_episodes, chunks))
    except Exception as e:
        failure = type(e).__name__ + ": " + str(e)[:300]
    st = {}
    for p in parts:
        for k, v in p.items():
            if isinstance(v, list):
                st.setdefault(k, []).extend(v)
            elif isinstance(v, dict):
                d = st.setdefault(k, {})
                for kk, vv in v.items():
                    if isinstance(vv, dict):
                        e = d.setdefault(kk, {})
                        for a, bb in vv.items():
                            e[a] = e.get(a, 0) + bb
                    elif isinstance(vv, list):
                        e = d.setdefault(kk, [0] * len(vv))
                        d[kk] = [x + y for x, y in zip(e, vv)]
                    else:
                        d[kk] = d.get(kk, 0) + vv
            else:
                st[k] = st.get(k, 0) + v
    st["examples"] = st.get("examples", [])[:20]
    ms_step, ms_cert = st.pop("ms_step", []), st.pop("ms_certify16", [])
    rate = st["certifiable_steps"] / st["decision_steps"] if st.get("decision_steps") else None
    if failure or rate is None:
        reading = "FAIL_INCOMPLETE"
    elif st["false_cert"]:
        reading = "STOP_FALSE_CERT"
    elif rate >= THRESH["filter_min"]:
        reading = "FILTER_ARM_RETAINED"
    elif rate >= THRESH["monitor_min"]:
        reading = "MONITOR_ONLY_PILOT"
    else:
        reading = "FILTER_PAUSED"
    res = {"gate": "M2 G-alpha", "reading": reading, "certifiable_rate": rate,
           "certifiable_rate_by_group": {k: (v["certifiable_steps"] / v["decision_steps"] if v["decision_steps"]
                                             else None) for k, v in st.get("by_cell_group", {}).items()},
           "stats": st, "timing_ms": {"step": q(ms_step), "certify16": q(ms_cert),
                                      "note": "single process per worker, 12 concurrent workers; G-beta is "
                                              "measured separately, alone"},
           "failure": failure, "cells": cells, "episodes_per_cell": n, "sample_seed": SAMPLE_SEED,
           "config": CONFIG, "offsets_deg": OFFSETS_DEG, "lengths_m": LENGTHS, "terminal_m": TERMINAL_M,
           "cert_every": CERT_EVERY, "progress_m": PROGRESS_M, "thresholds": THRESH,
           "kernel_file_sha256": hashlib.sha256(RP.KERNEL_FILE.read_bytes()).hexdigest(), "wall_s": time.time() - t0}
    with out.open("x") as fh:
        json.dump(res, fh, indent=1, default=float)
        fh.write("\n")
    print(json.dumps({k: res[k] for k in ("reading", "certifiable_rate", "certifiable_rate_by_group")}, indent=1))
    print(json.dumps({"false_cert": st.get("false_cert"), "decision_steps": st.get("decision_steps"),
                      "reasons": st.get("reasons"), "timing": res["timing_ms"], "wall_s": res["wall_s"]}, indent=1))
    return 0 if reading not in ("FAIL_INCOMPLETE", "STOP_FALSE_CERT") else 1


if __name__ == "__main__":
    raise SystemExit(main())
