"""Camera-only G3 analysis (docs/plans/prereg_camonly_c2trace_v1.md, fixed before the run). CPU only.

For each condition (A = full, B_out = no_lidar_out_fov, C = no_lidar), pooled over 70/115/160/205 bars, seed 4501,
from the traced cells of tools/camonly_trace_run.py. Geometry in the yaw-only vehicle frame (x = camera axis), the
camera FOV = |bearing| <= 43.5 deg, bars = axis-aligned boxes spanning the vehicle's height (as
docs/evidence/camonly_2026-11-04/mechanism/f_trace_geometry.py):
  - crash episodes = episodes.jsonl outcome crash (all episodes with a trace, not only the quota);
  - contact bar = the bar nearest (surface distance) at the episode's last recorded frame (about 0.1 s before contact);
  - out_last = share of crash episodes whose contact bar is outside the FOV at that frame;
  - never_in_1s = share whose contact bar was outside the FOV in every one of the last 11 frames;
  - travel_out = share of moving frames (speed > 0.3 m/s) whose velocity bearing is outside the FOV.
Validity gate: in each condition at least 90 % of crash episodes have a last-frame surface distance <= 0.6 m.
Reading (first match): MEASUREMENT_INVALID; BLIND_SIDE_SUPPORTED if Delta = out_last(C) - out_last(A) >= 15 pp,
its 95 % CI lower bound > 0 and out_last(C) >= 60 %; BLIND_SIDE_NOT_SUPPORTED if the CI upper bound < 15 pp or
out_last(C) < 50 %; otherwise INCONCLUSIVE. CI: percentile bootstrap over crash episodes, resampled within each
condition, 10,000 resamples, seed 20261105. The B_out - A contrast is computed the same way and reported.
usage: camonly_c2trace_analyse.py MOTAR_TREE TRACE_RUN_DIR OUT_JSON
"""
import hashlib
import importlib.util
import json
import math
import pathlib
import sys

import numpy as np

TREE, RUN, OUT = (pathlib.Path(a).resolve() for a in sys.argv[1:4])
HALF = math.radians(87.0) / 2
MOVING = 0.3
CONDS = {"A": "full", "B_out": "no_lidar_out_fov", "C": "no_lidar"}
DENSITIES = (70, 115, 160, 205)
SEED = 4501
NBOOT, BSEED = 10000, 20261105


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, str(path))
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


OT = load("occlusion_trace_c2t_an", TREE / "motar_eval/occlusion_trace.py")


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def yaw_of(q):
    x, y, z, w = q[..., 0], q[..., 1], q[..., 2], q[..., 3]
    return np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def surface(p, c, h):
    span = np.abs(p[2] - c[:, 2]) <= h[:, 2]
    d = p[None, :2] - c[:, :2]
    cl = np.clip(d, -h[:, :2], h[:, :2])
    off = (c[:, :2] + cl) - p[None, :2]
    dist = np.hypot(off[:, 0], off[:, 1])
    dist[~span] = np.inf
    return dist, off


def bearing_to(off_row, yaw):
    return float(wrap(math.atan2(off_row[1], off_row[0]) - yaw))


def cell(cdir):
    tr = OT.read_trace(cdir)
    by_ep = OT.episode_frames(tr["frames"])
    look = OT.reset_lookup(tr["layouts"])
    outcome = {}
    with open(cdir / "episodes.jsonl") as f:
        for line in f:
            e = json.loads(line)
            outcome[(e["env_index"], e["env_episode_index"])] = e["outcome"]
    crashes, moving, moving_out = [], 0, 0
    for key, fr in by_ep.items():
        if key not in look or len(fr) == 0:
            continue
        c, h, _ = OT.layout_arrays(tr["layouts"], look[key])
        c, h = np.asarray(c, float), np.asarray(h, float)
        yaw = yaw_of(np.asarray(fr["p_vehicle_quat"], float))
        pos = np.asarray(fr["p_pos"], float)
        vel = np.asarray(fr["p_vel"], float)
        for i in range(len(fr)):
            spd = math.hypot(vel[i, 0], vel[i, 1])
            if spd > MOVING:
                moving += 1
                moving_out += abs(wrap(math.atan2(vel[i, 1], vel[i, 0]) - yaw[i])) > HALF
        if outcome.get(key) != "crash":
            continue
        dist, off = surface(pos[-1], c, h)
        j = int(np.argmin(dist))
        b_last = bearing_to(off[j], yaw[-1])
        never_in = True
        for k in range(max(0, len(fr) - 11), len(fr)):
            _, o2 = surface(pos[k], c, h)
            never_in &= abs(bearing_to(o2[j], yaw[k])) > HALF
        crashes.append({"dist": float(dist[j]), "out": abs(b_last) > HALF, "never_in_1s": bool(never_in),
                        "left": b_last > 0, "abs_bearing_deg": math.degrees(abs(b_last)), "frames": int(len(fr))})
    return crashes, moving, moving_out


def boot_diff(x, y, rng):
    x, y = np.asarray(x, float), np.asarray(y, float)
    bx = rng.integers(0, len(x), size=(NBOOT, len(x)))
    by = rng.integers(0, len(y), size=(NBOOT, len(y)))
    d = x[bx].mean(1) - y[by].mean(1)
    return [float(np.percentile(d, 2.5) * 100), float(np.percentile(d, 97.5) * 100)]


def main():
    if OUT.exists():
        raise SystemExit("refusing to overwrite " + str(OUT))
    res = {"analysis": "camonly G3 trace", "prereg": "docs/plans/prereg_camonly_c2trace_v1.md", "inputs_sha256": {},
           "conditions": {}}
    pooled = {}
    for name, cond in CONDS.items():
        allc, mv, mvo = [], 0, 0
        for d in DENSITIES:
            cdir = RUN / cond / ("E2_obstacle_aware__%dbars__seed%d" % (d, SEED))
            for fn in ("occlusion_frames.bin", "occlusion_layouts.npz", "episodes.jsonl"):
                res["inputs_sha256"]["%s/%d/%s" % (cond, d, fn)] = hashlib.sha256((cdir / fn).read_bytes()).hexdigest()
            cr, m, mo = cell(cdir)
            allc += cr
            mv += m
            mvo += mo
        n = len(allc)
        pooled[name] = allc
        res["conditions"][name] = {
            "condition": cond, "crash_episodes_traced": n,
            "valid_share_pct": 100 * sum(c["dist"] <= 0.6 for c in allc) / max(1, n),
            "out_last_pct": 100 * sum(c["out"] for c in allc) / max(1, n),
            "never_in_1s_pct": 100 * sum(c["never_in_1s"] for c in allc) / max(1, n),
            "left_pct": 100 * sum(c["left"] for c in allc) / max(1, n),
            "mean_abs_bearing_deg": sum(c["abs_bearing_deg"] for c in allc) / max(1, n),
            "travel_out_of_fov_pct": 100 * mvo / max(1, mv), "moving_frames": mv}
    rng = np.random.default_rng(BSEED)
    out = {k: [float(c["out"]) for c in v] for k, v in pooled.items()}
    contrasts = {}
    for x in ("C", "B_out"):
        delta = 100 * (np.mean(out[x]) - np.mean(out["A"]))
        contrasts[x + "-A_out_last_pp"] = {"delta": float(delta), "ci95": boot_diff(out[x], out["A"], rng)}
    res["contrasts"] = contrasts
    cA, cC = res["conditions"]["A"], res["conditions"]["C"]
    dC = contrasts["C-A_out_last_pp"]
    if any(v["valid_share_pct"] < 90.0 or v["crash_episodes_traced"] == 0 for v in res["conditions"].values()):
        reading = "MEASUREMENT_INVALID"
    elif dC["delta"] >= 15.0 and dC["ci95"][0] > 0 and cC["out_last_pct"] >= 60.0:
        reading = "BLIND_SIDE_SUPPORTED"
    elif dC["ci95"][1] < 15.0 or cC["out_last_pct"] < 50.0:
        reading = "BLIND_SIDE_NOT_SUPPORTED"
    else:
        reading = "INCONCLUSIVE"
    res["reading"] = reading
    OUT.write_text(json.dumps(res, indent=1, sort_keys=True) + "\n")
    print(json.dumps({"reading": reading, "contrasts": contrasts,
                      "out_last": {k: v["out_last_pct"] for k, v in res["conditions"].items()}}, indent=1))
    print(OUT, hashlib.sha256(OUT.read_bytes()).hexdigest())


if __name__ == "__main__":
    main()
