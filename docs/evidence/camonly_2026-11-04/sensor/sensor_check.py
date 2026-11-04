"""Camera-only study, sensor-level check on the CPU (plan "K5 revised"; review r2 M-r2-2). No policy, no GPU.

At Policy F's own recorded poses and bar layouts (B1-R A/F_SHADOW traces), render with MOTAR's own Warp kernels on
the CPU:
  - the 40 x 24 obstacle camera: `_render_obstacle_depth_kernel` copied byte for byte from navrl_detector.py, its ray
    table and its bilinear upsample to 160 x 90, exactly as `NavRLDetector.render_raw_rgbd`;
  - the 72 x 4 LiDAR: the C-Planner M2 replay (`cplanner/motar/replay.py`, MOTAR's LiDAR kernel).
For each of the 17 in-FOV bins, the camera value perception fuses (navrl_perception.py:1910-1921: one column at the
bin's bearing, minimum over rows, camera no-return at 10 m mapped to 12 m) is compared with an independent exact
range: a horizontal ray from the LiDAR origin at the bin's bearing against the axis-aligned bar boxes that span the
sensor height. Classified per bin (tolerance TOL m):
  phantom   camera < truth - TOL            (the camera reports something nearer than any bar on that bearing)
  miss      camera > truth + TOL, truth < 10 (a bar the camera range covers is reported farther or not at all)
  range     truth in [10, 12): beyond the camera, within the LiDAR
  match     otherwise
The same classification is repeated against the exact range along the bearing the camera column actually samples,
from the camera origin ("sampled"): it separates the angle-to-column offset (bin truth vs sampled truth) from the
render itself (40 x 24 rays and bilinear upsampling; camera vs sampled truth).
The LiDAR's horizontal row (elevation 0) is checked against the same truth as a validation of the truth caster
(it shoots the same ray, so it must agree). Also reported: the bearing each bin's camera column actually samples
(the linear angle-to-column map against the pinhole render).
usage: sensor_check.py CAMONLY_TREE CPLANNER_TREE OUT_JSON [frames_per_cell]
"""
import ast
import hashlib
import importlib.util
import json
import math
import pathlib
import random
import sys
import types

import numpy as np
import torch
import torch.nn.functional as Fnn

CAM_TREE, CP_TREE, OUT = (pathlib.Path(a).resolve() for a in sys.argv[1:4])
PER_CELL = int(sys.argv[4]) if len(sys.argv) > 4 else 300
HERE = pathlib.Path(__file__).resolve().parent
B1R = pathlib.Path("/home/fair/workspaces/aerial_gym_ws/src/aerial_gym_simulator/results/b1r_classical_baseline_v1")
CELLS = [B1R / "A" / "F_SHADOW" / ("E2_obstacle_aware__%dbars__seed4301" % b) for b in (70, 115, 160, 205)]
TOL = 0.3
OBS_W, OBS_H, CAM_W, CAM_H = 40, 24, 160, 90
HFOV, VFOV = math.radians(87.0), math.radians(58.0)
CAM_FAR, LIDAR_MAX = 10.0, 12.0
CAM_OFFSET = np.array([0.10, 0.0, 0.03])
SEED = 20261104


def sha(p):
    return hashlib.sha256(pathlib.Path(p).read_bytes()).hexdigest()


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, str(path))
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


# --- the obstacle-camera kernel, byte for byte (Warp needs a real source file for code generation)
DET = CAM_TREE / "aerial_gym/task/navrl_task/navrl_detector.py"
src = DET.read_text()
tree = ast.parse(src)
fn = [n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "_render_obstacle_depth_kernel"][0]
lines = src.splitlines(keepends=True)
start = min(d.lineno for d in fn.decorator_list) - 1
kernel_src = "".join(lines[start:fn.end_lineno])
kfile = HERE / "obstacle_kernel_extracted.py"
kfile.write_text("import warp as wp\n\n\n" + kernel_src)
import warp as wp                                               # noqa: E402
wp.init()
KM = load("obstacle_kernel_extracted", kfile)

# --- perception's own helpers (by path, with the stub packages its imports need)
for pkg in ("aerial_gym", "aerial_gym.task", "aerial_gym.task.navrl_task"):
    sys.modules.setdefault(pkg, types.ModuleType(pkg))
for name in ("navrl_corridor", "navrl_search_state"):
    load("aerial_gym.task.navrl_task." + name, CAM_TREE / "aerial_gym/task/navrl_task" / (name + ".py"))
import os                                                       # noqa: E402
os.environ["NAVRL_LIDAR_HBEAMS"], os.environ["NAVRL_LIDAR_VBEAMS"] = "72", "4"
PERC = load("navrl_perception_sensor_check", CAM_TREE / "aerial_gym/task/navrl_task/navrl_perception.py")
RP = load("cplanner_replay_sensor_check", CP_TREE / "cplanner/motar/replay.py")
RP.KERNEL_FILE = CAM_TREE / "aerial_gym/sensors/warp/warp_kernels/warp_lidar_kernels.py"
OT = load("occlusion_trace_sensor_check", CAM_TREE / "motar_eval/occlusion_trace.py")


def obstacle_rays():
    """navrl_detector.py's obstacle ray table (no FOV error), float32, normalised; (H, W, 3)."""
    uu = torch.arange(OBS_W, dtype=torch.float32)
    vv = torch.arange(OBS_H, dtype=torch.float32)
    vv, uu = torch.meshgrid(vv, uu, indexing="ij")
    fx = OBS_W / (2.0 * math.tan(HFOV / 2))
    fy = OBS_H / (2.0 * math.tan(VFOV / 2))
    cx, cy = (OBS_W - 1) * 0.5, (OBS_H - 1) * 0.5
    r = torch.stack([torch.ones_like(uu), -(uu - cx) / fx, -(vv - cy) / fy], dim=-1)
    return (r / r.norm(dim=-1, keepdim=True).clamp(min=1e-9)).numpy()


RAYS = wp.array(obstacle_rays(), dtype=wp.vec3, device="cpu")
BEARINGS = PERC.lidar_bin_bearings("cpu")                       # perception's bin bearings, [72]
INSIDE = (BEARINGS.abs() <= HFOV * 0.5)
CAM_U = ((HFOV * 0.5 - BEARINGS) / HFOV * (CAM_W - 1)).round().long().clamp(0, CAM_W - 1)


def sampled_bearing(u):
    """Bearing of upsampled column u: bilinear (align_corners=False) source position, pinhole angle."""
    xs = (u + 0.5) * OBS_W / CAM_W - 0.5
    fx = OBS_W / (2.0 * math.tan(HFOV / 2))
    return math.atan(-(xs - (OBS_W - 1) * 0.5) / fx)


def camera_bins(mesh_ids, origin, quat):
    out = wp.zeros((1, OBS_H, OBS_W), dtype=float, device="cpu")
    wp.launch(KM._render_obstacle_depth_kernel, dim=(1, OBS_H, OBS_W),
              inputs=[mesh_ids, wp.array(np.asarray(origin, np.float32).reshape(1, 3), dtype=wp.vec3, device="cpu"),
                      wp.array(np.asarray(quat, np.float32).reshape(1, 4), dtype=wp.quat, device="cpu"), RAYS,
                      float(CAM_FAR), out], device="cpu")
    d = torch.from_numpy(out.numpy())
    hi = Fnn.interpolate(d.unsqueeze(1), size=(CAM_H, CAM_W), mode="bilinear", align_corners=False).squeeze(1)
    hi = torch.nan_to_num(hi, nan=CAM_FAR, posinf=CAM_FAR, neginf=CAM_FAR).clamp(0.0, CAM_FAR)
    col_min = PERC.camera_no_return_to_lidar_range(hi.amin(dim=1), CAM_FAR, LIDAR_MAX).clamp(0.0, LIDAR_MAX)
    return col_min[0, CAM_U].numpy()                             # [72], meaningful where INSIDE


def truth_ranges(c, h, p, yaw, bearings=None):
    """Exact horizontal range from p along each bearing (vehicle frame; default: the bin bearings) to the nearest box
    spanning p's height (inf if none)."""
    bearings = BEARINGS.numpy().astype(np.float64) if bearings is None else np.asarray(bearings, np.float64)
    out = np.full(len(bearings), np.inf)
    span = np.abs(p[2] - c[:, 2]) <= h[:, 2]
    lo, hi = (c - h)[span, :2], (c + h)[span, :2]
    for j, b in enumerate(bearings):
        a = yaw + b
        dx, dy = math.cos(a), math.sin(a)
        with np.errstate(divide="ignore", invalid="ignore"):
            tx1, tx2 = (lo[:, 0] - p[0]) / dx, (hi[:, 0] - p[0]) / dx
            ty1, ty2 = (lo[:, 1] - p[1]) / dy, (hi[:, 1] - p[1]) / dy
        tmin = np.maximum(np.minimum(tx1, tx2), np.minimum(ty1, ty2))
        tmax = np.minimum(np.maximum(tx1, tx2), np.maximum(ty1, ty2))
        ok = (tmax >= np.maximum(tmin, 0.0))
        if ok.any():
            out[j] = float(np.maximum(tmin[ok], 0.0).min())
    return out


BANDS = [(0, 2), (2, 4), (4, 6), (6, 8), (8, 10), (10, 12), (12, None)]


def band_of(t):
    for lo, hi in BANDS:
        if hi is None or t < hi:
            return "%s-%s" % (lo, hi if hi is not None else "inf")


SAMPLED = np.array([sampled_bearing(int(CAM_U[j])) if bool(INSIDE[j]) else float(BEARINGS[j])
                    for j in range(len(BEARINGS))])


def classify(t, v):
    if t < CAM_FAR and v > t + TOL:
        return "miss"
    if CAM_FAR <= t < LIDAR_MAX:
        return "range" if v >= LIDAR_MAX * 0.995 else ("phantom" if v < t - TOL else "match")
    if v < min(t, LIDAR_MAX) - TOL:
        return "phantom"
    return "match"


def main():
    if OUT.exists():
        raise SystemExit("refusing to overwrite " + str(OUT))
    rng = random.Random(SEED)
    stats = {}
    val = {"lidar_row0_vs_truth_n": 0, "lidar_row0_agree": 0, "max_abs_err_m": 0.0}
    frames_done = 0
    for cell in CELLS:
        tr = OT.read_trace(cell)
        by_ep = OT.episode_frames(tr["frames"])
        look = OT.reset_lookup(tr["layouts"])
        keys = [k for k in sorted(by_ep) if k in look]
        bars = int(cell.name.split("__")[1].replace("bars", ""))
        picks = []
        for k in keys:
            for x in by_ep[k]:
                picks.append((k, x))
        rng.shuffle(picks)
        picks = picks[:PER_CELL]
        scenes = {}
        for k, x in picks:
            if k not in scenes:
                c, h, q = OT.layout_arrays(tr["layouts"], look[k])
                c, h, q = np.array(c, float), np.array(h, float), np.array(q, float)
                if np.abs(q - np.array([0, 0, 0, 1.0])).max() > 1e-6:
                    raise ValueError("non-identity bar orientation")
                scenes[k] = (RP.Scene(c, h, q), c, h)
            sc, c, h = scenes[k]
            p = np.array(x["p_pos"], float)
            yaw, sq = RP.sensor_yaw(np.array(x["p_quat"], float))
            far_target = (p + np.array([1e4, 1e4, 0.0])).astype(np.float32)
            raw, _ = sc.raw(p.astype(np.float32), sq, far_target)            # LiDAR slant range, metres, (4, 72)
            q4 = np.asarray(sq, float)
            cam_origin = p + RP._quat_rotate_xyzw(q4, CAM_OFFSET)
            cam = camera_bins(sc.ids, cam_origin, sq)
            truth = truth_ranges(c, h, p, yaw)
            truth_s = truth_ranges(c, h, cam_origin, yaw, SAMPLED)
            lidar0 = np.where(raw[2] > LIDAR_MAX, np.inf, raw[2])
            for j in range(len(BEARINGS)):
                if np.isfinite(truth[j]) and truth[j] < LIDAR_MAX * 0.99:
                    val["lidar_row0_vs_truth_n"] += 1
                    err = abs(lidar0[j] - truth[j]) if np.isfinite(lidar0[j]) else np.inf
                    val["lidar_row0_agree"] += int(err < 0.02)
                    if np.isfinite(err):
                        val["max_abs_err_m"] = max(val["max_abs_err_m"], float(err))
                if not bool(INSIDE[j]):
                    continue
                t = truth[j]
                v = float(cam[j])
                kind = classify(t, v)
                kind_s = classify(truth_s[j], v)
                s = stats.setdefault(str(bars), {}).setdefault(band_of(t), {"n": 0, "phantom": 0, "miss": 0,
                                                                            "range": 0, "match": 0,
                                                                            "phantom_depth_sum_m": 0.0,
                                                                            "sampled_phantom": 0, "sampled_miss": 0,
                                                                            "sampled_match": 0, "sampled_range": 0,
                                                                            "offset_changes_truth": 0})
                s["n"] += 1
                s[kind] += 1
                s["sampled_" + kind_s] += 1
                same = (np.isinf(t) and np.isinf(truth_s[j])) or abs(min(t, 99.0) - min(truth_s[j], 99.0)) <= TOL
                s["offset_changes_truth"] += int(not same)
                if kind == "phantom":
                    s["phantom_depth_sum_m"] += min(t, LIDAR_MAX) - v
            frames_done += 1
    pooled = {}
    for bars, bands in stats.items():
        for band, s in bands.items():
            ps = pooled.setdefault(band, {"n": 0, "phantom": 0, "miss": 0, "range": 0, "match": 0,
                                          "sampled_phantom": 0, "sampled_miss": 0, "sampled_match": 0,
                                          "sampled_range": 0, "offset_changes_truth": 0})
            for k in ps:
                ps[k] += s[k]
    rate = lambda s: {k + "_rate": (s[k] / s["n"] if s["n"] else None) for k in ("phantom", "miss", "range", "match",  # noqa
                                                                                "sampled_phantom", "sampled_miss",
                                                                                "sampled_match", "sampled_range",
                                                                                "offset_changes_truth")}
    offsets = [{"bin_bearing_deg": round(math.degrees(float(BEARINGS[j])), 3), "camera_u": int(CAM_U[j]),
                "sampled_bearing_deg": round(math.degrees(sampled_bearing(int(CAM_U[j]))), 3)}
               for j in range(len(BEARINGS)) if bool(INSIDE[j])]
    for o in offsets:
        o["offset_deg"] = round(o["sampled_bearing_deg"] - o["bin_bearing_deg"], 3)
    res = {"frames": frames_done, "per_cell": PER_CELL, "tolerance_m": TOL, "cells": [str(c) for c in CELLS],
           "in_fov_bins": int(INSIDE.sum()), "validation_lidar_row0": val,
           "pooled_by_truth_band": {b: {**s, **rate(s)} for b, s in pooled.items()},
           "by_density": {bars: {b: {**s, **rate(s)} for b, s in bands.items()} for bars, bands in stats.items()},
           "camera_column_bearing": offsets,
           "sources": {"navrl_detector.py": sha(DET), "kernel_extracted": sha(kfile),
                       "navrl_perception.py": sha(CAM_TREE / "aerial_gym/task/navrl_task/navrl_perception.py"),
                       "warp_lidar_kernels.py": sha(RP.KERNEL_FILE), "replay.py": sha(CP_TREE / "cplanner/motar/replay.py"),
                       "occlusion_trace.py": sha(CAM_TREE / "motar_eval/occlusion_trace.py")},
           "seed": SEED}
    OUT.write_text(json.dumps(res, indent=1) + "\n")
    print(json.dumps({"frames": frames_done, "validation": val,
                      "pooled": {b: {k: (round(v, 3) if isinstance(v, float) else v) for k, v in s.items()}
                                 for b, s in res["pooled_by_truth_band"].items()},
                      "max_offset_deg": max(abs(o["offset_deg"]) for o in offsets)}, indent=1))


if __name__ == "__main__":
    main()
