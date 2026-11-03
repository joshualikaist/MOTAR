"""M2 gate V1: qualify the MOTAR 72x4 status mapping (docs/plans/prereg_cplanner_m2_v1.md).

For sampled B1-R frames, the scan replayed with MOTAR's own Warp kernel on CPU (cplanner/motar/replay.py) is compared,
beam by beam, with an independent float64 ray caster written here (oriented boxes from the recorded layout and the
0.20 m analytic target sphere; it shares no code with Warp, the replay or the certifier). The adapter
(cplanner/motar/adapter.py) is checked on the same beams.

usage: v1_hitbit.py OUT_JSON FRAMES_PER_CELL WORKERS CELL_DIR [CELL_DIR ...]
"""
import concurrent.futures as cf
import hashlib
import json
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
sys.path.insert(0, str(HERE.parents[1]))            # repository root, for motar_eval (read-only reader)
import adapter as AD                                 # noqa: E402
import registry as REG                               # noqa: E402
import replay as RP                                  # noqa: E402
from motar_eval import occlusion_trace as OT         # noqa: E402

TOL_RANGE = 5.0e-4          # |12 v - t_oracle| for HIT beams: half the adapter margin (dev max 5.2e-5 m)
BAND = 1.0e-4               # oracle hits within BAND of 0.2 or 12 m are reported as boundary cases, not violations
SAMPLE_SEED = 20261104


def quat_rotate_xyzw(q, v):
    x, y, z, w = (float(c) for c in q)
    u = np.array([x, y, z])
    return v + 2.0 * np.cross(u, np.cross(u, v) + w * v)


def oracle(origin, dirs, centers, halves, quats, target, target_r):
    """Nearest hit distance (float64, inf if none) and owner (-1 none, -2 target, b bar) per direction."""
    n = len(dirs)
    best = np.full(n, np.inf)
    who = np.full(n, -1, np.int64)
    for b, (c, h, q) in enumerate(zip(centers, halves, quats)):
        qi = (-q[0], -q[1], -q[2], q[3])
        o = quat_rotate_xyzw(qi, np.asarray(origin, float) - np.asarray(c, float))
        d = np.array([quat_rotate_xyzw(qi, x) for x in dirs]) if any(abs(float(a)) > 0 for a in q[:3]) else dirs
        with np.errstate(divide="ignore", invalid="ignore"):
            t1 = (-np.asarray(h, float) - o) / d
            t2 = (np.asarray(h, float) - o) / d
        lo = np.where(np.abs(d) < 1e-15, np.where(np.abs(o) <= np.asarray(h, float), -np.inf, np.inf),
                      np.minimum(t1, t2))
        hi = np.where(np.abs(d) < 1e-15, np.where(np.abs(o) <= np.asarray(h, float), np.inf, -np.inf),
                      np.maximum(t1, t2))
        tmin, tmax = lo.max(1), hi.min(1)
        ok = tmax >= np.maximum(tmin, 0.0)
        t = np.where(tmin >= 0, tmin, tmax)
        upd = ok & (t < best)
        best[upd], who[upd] = t[upd], b
    oc = np.asarray(origin, float) - np.asarray(target, float)
    bb = dirs @ oc
    cc = oc @ oc - target_r ** 2
    disc = bb * bb - cc
    sq = np.sqrt(np.maximum(disc, 0.0))
    t = np.where(-bb - sq >= 0, -bb - sq, -bb + sq)
    upd = (disc >= 0) & (t >= 0) & (t < best)
    best[upd], who[upd] = t[upd], -2
    return best, who


def frames_for(cell, n, rng):
    tr = OT.read_trace(cell)
    fr = tr["frames"]
    look = OT.reset_lookup(tr["layouts"])
    idx = sorted(rng.sample(range(len(fr)), min(n, len(fr))))
    out = []
    for k in idx:
        x = fr[k]
        key = (int(x["env"]), int(x["episode"]))
        if key not in look:
            continue
        c, h, q = OT.layout_arrays(tr["layouts"], look[key])
        out.append(dict(cell=str(cell), k=int(k), env=key[0], episode=key[1], frame=int(x["frame"]),
                        p=np.array(x["p_pos"], float), q=np.array(x["p_quat"], float), t=np.array(x["t_pos"], float),
                        c=np.array(c, float), h=np.array(h, float), qb=np.array(q, float)))
    return out


def check_frames(items):
    st = {"frames": 0, "beams": 0, "nonfinite": 0, "hit": 0, "no_hit": 0, "near": 0, "target_hits": 0,
          "v_nohit_but_oracle_hit": 0, "v_near_but_oracle_far": 0, "v_hit_range_mismatch": 0,
          "v_hit_but_oracle_none": 0, "v_oracle_hit_missed": 0, "band_cases": 0, "adapter_range_beyond_true": 0,
          "adapter_status_mismatch": 0, "max_abs_range_err_m": 0.0, "max_dir_angle_rad": 0.0,
          "max_lateral_offset_12m": 0.0, "bars_per_frame": [], "examples": []}
    man = REG.directions("MOTAR-72x4").reshape(-1, 3)
    rv = RP.ray_vectors().reshape(-1, 3).astype(np.float64)
    scenes = {}
    for it in items:
        key = (it["cell"], it["env"], it["episode"])
        if key not in scenes:
            scenes = {key: RP.Scene(it["c"], it["h"], it["qb"])}
        sc = scenes[key]
        yaw, sq = RP.sensor_yaw(it["q"])
        v = sc.scan(it["p"].astype(np.float32), sq, it["t"].astype(np.float32)).reshape(-1).astype(np.float64)
        cy, sy = math.cos(yaw), math.sin(yaw)
        rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1.0]])
        dirs = man @ rz.T                                            # what the certifier assumes
        kd = np.array([quat_rotate_xyzw(sq.astype(np.float64), r) for r in rv])
        kd /= np.linalg.norm(kd, axis=1, keepdims=True)              # what the kernel cast (float32 inputs)
        ang = np.arccos(np.clip((dirs * kd).sum(1), -1.0, 1.0))
        st["max_dir_angle_rad"] = max(st["max_dir_angle_rad"], float(ang.max()))
        st["max_lateral_offset_12m"] = max(st["max_lateral_offset_12m"], float(12.0 * np.sin(ang).max()))
        t, who = oracle(it["p"], dirs, it["c"], it["h"], it["qb"], it["t"], RP.TARGET_RADIUS)
        ranges, status = AD.derive(v.reshape(AD.SHAPE))
        ranges, status = ranges.reshape(-1), status.reshape(-1)
        st["frames"] += 1
        st["beams"] += len(v)
        st["bars_per_frame"].append(len(it["c"]))
        for b in range(len(v)):
            vb, tb = v[b], t[b]
            band = abs(tb - RP.MAX_RANGE) <= BAND or abs(tb - RP.MIN_RANGE) <= BAND
            if not np.isfinite(vb):
                st["nonfinite"] += 1
                continue
            bad = None
            if vb == 1.0:
                st["no_hit"] += 1
                if tb <= RP.MAX_RANGE and not band:
                    bad = "v_nohit_but_oracle_hit"
            elif vb == -1.0:
                st["near"] += 1
                if tb >= RP.MIN_RANGE and not band:
                    bad = "v_near_but_oracle_far"
            else:
                st["hit"] += 1
                st["target_hits"] += int(who[b] == -2)
                if not np.isfinite(tb):
                    bad = "v_hit_but_oracle_none"
                else:
                    err = abs(12.0 * vb - tb)
                    st["max_abs_range_err_m"] = max(st["max_abs_range_err_m"], err)
                    if err > TOL_RANGE:
                        bad = "v_hit_range_mismatch"
                    if status[b] == AD.HIT and ranges[b] > tb:
                        st["adapter_range_beyond_true"] += 1
            if np.isfinite(tb) and tb < RP.MAX_RANGE - BAND and tb > RP.MIN_RANGE + BAND and vb == 1.0:
                bad = bad or "v_oracle_hit_missed"
            want = AD.NO_HIT if vb == 1.0 else AD.NEAR_CLIPPED if vb == -1.0 else (
                AD.HIT if 12.0 * vb - AD.HIT_MARGIN_M >= AD.MIN_RANGE else AD.NEAR_CLIPPED)
            st["adapter_status_mismatch"] += int(status[b] != want)
            if band:
                st["band_cases"] += 1
            if bad:
                st[bad] += 1
                if len(st["examples"]) < 20:
                    st["examples"].append({"cell": it["cell"], "k": it["k"], "beam": b, "v": vb, "t_oracle": tb,
                                           "owner": int(who[b]), "kind": bad})
    return st


def main():
    out = pathlib.Path(sys.argv[1])
    n, workers, cells = int(sys.argv[2]), int(sys.argv[3]), sys.argv[4:]
    if out.exists():
        raise SystemExit("refusing to overwrite " + str(out))
    t0 = time.time()
    rng = random.Random(SAMPLE_SEED)
    items = []
    for c in cells:
        items += frames_for(c, n, rng)
    groups = {}
    for it in items:
        groups.setdefault((it["cell"], it["env"], it["episode"]), []).append(it)
    chunks = [[] for _ in range(workers * 4)]
    for i, g in enumerate(groups.values()):
        chunks[i % len(chunks)].extend(g)
    chunks = [c for c in chunks if c]
    parts, failure = [], None
    try:
        with cf.ProcessPoolExecutor(max_workers=workers, mp_context=mp.get_context("spawn")) as ex:
            parts = list(ex.map(check_frames, chunks))
    except Exception as e:
        failure = type(e).__name__ + ": " + str(e)[:300]
    st = {}
    for p in parts:
        for k, v in p.items():
            if k.startswith("max_"):
                st[k] = max(st.get(k, 0.0), v)
            elif isinstance(v, list):
                st.setdefault(k, []).extend(v)
            else:
                st[k] = st.get(k, 0) + v
    st["examples"] = st.get("examples", [])[:20]
    bpf = st.pop("bars_per_frame", [])
    st["bars_per_frame_min_max"] = [min(bpf), max(bpf)] if bpf else None
    viol = ("v_nohit_but_oracle_hit", "v_near_but_oracle_far", "v_hit_range_mismatch", "v_hit_but_oracle_none",
            "v_oracle_hit_missed", "nonfinite", "adapter_range_beyond_true", "adapter_status_mismatch")
    checks = {"completed": failure is None and st.get("frames", 0) == len(items),
              "zero_violations": failure is None and all(st.get(k, 1) == 0 for k in viol),
              "positive_hits": st.get("hit", 0) > 0, "positive_no_hits": st.get("no_hit", 0) > 0,
              "positive_target_hits": st.get("target_hits", 0) > 0,
              "range_error_below_half_margin": st.get("max_abs_range_err_m", 1.0) <= AD.HIT_MARGIN_M / 2}
    checks = {k: bool(v) for k, v in checks.items()}
    res = {"gate": "M2 V1", "pass": all(checks.values()), "checks": checks, "stats": st, "failure": failure,
           "cells": cells, "frames_per_cell": n, "sample_seed": SAMPLE_SEED, "tol_range_m": TOL_RANGE,
           "band_m": BAND, "hit_margin_m": AD.HIT_MARGIN_M,
           "kernel_file_sha256": hashlib.sha256(RP.KERNEL_FILE.read_bytes()).hexdigest(), "wall_s": time.time() - t0}
    with out.open("x") as fh:
        json.dump(res, fh, indent=1, default=float)
        fh.write("\n")
    print(json.dumps({"pass": res["pass"], "checks": checks, "frames": st.get("frames"), "beams": st.get("beams"),
                      "max_err": st.get("max_abs_range_err_m"), "max_ang": st.get("max_dir_angle_rad"),
                      "wall_s": res["wall_s"]}, indent=1))
    return 0 if res["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
