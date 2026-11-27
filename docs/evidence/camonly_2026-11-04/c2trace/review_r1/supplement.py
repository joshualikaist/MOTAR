"""Supplementary sensitivity for the G3 review (Opus 5.5): body-yaw vs vehicle-yaw, camera-eye origin CI, frame-0 yaw,
invalid A crashes. Read-only on inputs; writes argv[1]."""
import json
import math
import pathlib
import sys

import numpy as np

RUN = pathlib.Path("/home/fair/workspaces/aerial_gym_ws/src/aerial_gym_simulator/results/camonly_c2trace_v1")
CONDS = {"A": "full", "B_out": "no_lidar_out_fov", "C": "no_lidar"}
DENS = (70, 115, 160, 205)
H = math.radians(43.5)


def dt_of(s):
    return np.dtype([(f[0], f[1], tuple(f[2])) if len(f) == 3 else (f[0], f[1]) for f in s["frame_dtype"]])


def yaw(q):
    x, y, z, w = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    return np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def ang(a):
    return np.arctan2(np.sin(a), np.cos(a))


res = {"conditions": {}}
vals = {}
for name, cond in CONDS.items():
    rows = []
    dyaw_all, dyaw_f0 = [], []
    for d in DENS:
        cdir = RUN / cond / ("E2_obstacle_aware__%dbars__seed4501" % d)
        s = json.loads((cdir / "occlusion_trace_summary.json").read_text())
        fr = np.fromfile(cdir / "occlusion_frames.bin", dtype=dt_of(s))
        lay = np.load(cdir / "occlusion_layouts.npz")
        rl = {(int(r["env"]), int(r["episode"])): int(r["layout"]) for r in lay["resets"]}
        yv = yaw(fr["p_vehicle_quat"].astype(float))
        yb = yaw(fr["p_quat"].astype(float))
        dy = np.degrees(np.abs(ang(yv - yb)))
        dyaw_all.append(dy)
        dyaw_f0.append(dy[fr["frame"] == 0])
        key = fr["env"].astype(np.int64) * (1 << 32) + fr["episode"].astype(np.int64)
        crash = [json.loads(l) for l in open(cdir / "episodes.jsonl")]
        crash = [e for e in crash if e["outcome"] == "crash"]
        order = np.lexsort((fr["frame"], key))
        ks = key[order]
        for e in crash:
            k = e["env_index"] * (1 << 32) + e["env_episode_index"]
            lo, hi = np.searchsorted(ks, k, "left"), np.searchsorted(ks, k, "right")
            idx = order[lo:hi]
            last = idx[-1]
            li = rl[(e["env_index"], e["env_episode_index"])]
            a, b = int(lay["offsets"][li]), int(lay["offsets"][li + 1])
            c, h = lay["center"][a:b].astype(float), lay["half"][a:b].astype(float)
            p = fr["p_pos"][last].astype(float)
            q = np.clip(p[:2], c[:, :2] - h[:, :2], c[:, :2] + h[:, :2])
            off = q - p[:2]
            dist = np.hypot(off[:, 0], off[:, 1])
            j = int(np.argmin(dist))
            bv = ang(math.atan2(off[j, 1], off[j, 0]) - yv[last])
            bb = ang(math.atan2(off[j, 1], off[j, 0]) - yb[last])
            eye = p[:2] + 0.1 * np.array([math.cos(yb[last]), math.sin(yb[last])])
            be = ang(math.atan2(q[j, 1] - eye[1], q[j, 0] - eye[0]) - yb[last])
            rows.append({"out_v": abs(bv) > H, "out_b": abs(bb) > H, "out_eye_b": abs(be) > H,
                         "dist": float(dist[j]), "nframes": int(len(idx)), "density": d,
                         "speed": float(np.hypot(*fr["p_vel"][last][:2])),
                         "z": float(p[2]), "in_quota": e["in_quota"]})
    dya = np.concatenate(dyaw_all)
    dy0 = np.concatenate(dyaw_f0)
    n = len(rows)
    inval = [r for r in rows if r["dist"] > 0.6]
    res["conditions"][name] = {
        "n": n,
        "out_last_vehicle_yaw_pct": 100 * sum(r["out_v"] for r in rows) / n,
        "out_last_body_yaw_pct": 100 * sum(r["out_b"] for r in rows) / n,
        "out_last_body_yaw_camera_eye_pct": 100 * sum(r["out_eye_b"] for r in rows) / n,
        "flip_vehicle_vs_body": sum(r["out_v"] != r["out_b"] for r in rows),
        "yaw_gap_deg_all_frames": {"median": float(np.median(dya)), "p99": float(np.percentile(dya, 99)),
                                   "max": float(dya.max())},
        "yaw_gap_deg_frame0": {"median": float(np.median(dy0)), "max": float(dy0.max())},
        "invalid_gt0p6": [{"dist": round(r["dist"], 3), "nframes": r["nframes"], "density": r["density"],
                           "speed": round(r["speed"], 2), "z": round(r["z"], 2), "out": bool(r["out_v"])}
                          for r in inval][:20],
        "invalid_count": len(inval),
    }
    vals[name] = rows

rng = np.random.default_rng(20261105)


def boot(x, y):
    x, y = np.asarray(x, float), np.asarray(y, float)
    bx = rng.integers(0, len(x), size=(10000, len(x)))
    by = rng.integers(0, len(y), size=(10000, len(y)))
    dd = x[bx].mean(1) - y[by].mean(1)
    return [float(100 * np.percentile(dd, 2.5)), float(100 * np.percentile(dd, 97.5))]


for f in ("out_v", "out_b", "out_eye_b"):
    xc = [r[f] for r in vals["C"]]
    xa = [r[f] for r in vals["A"]]
    res["C-A_" + f] = {"delta": 100 * (np.mean(xc) - np.mean(xa)), "ci95": boot(xc, xa)}
pathlib.Path(sys.argv[1]).write_text(json.dumps(res, indent=1, default=float) + "\n")
print(json.dumps(res, indent=1, default=float))
