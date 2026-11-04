"""C-Planner direction-bound repair, gate D2: V1-v2, the MOTAR 72x4 status mapping checked along the rays the kernel
actually cast (docs/plans/prereg_cplanner_dirbound_v1.md).

The M2 V1 test cast its float64 oracle along the certifier's manifest directions; one beam in 691,200 disagreed
because the kernel's float32 ray differs from the manifest by ~1e-7 rad (M2 review I1). With the declared direction
bound eps (registry MOTAR-72x4-RUNTIME-DIR) the certifier needs: (K1) every declared status and range true along the
kernel's own ray against the recorded float64 layout, up to a geometric depth DELTA_GEOM (float32 mesh vertices);
(K2) the lateral offset between kernel ray and manifest ray, at 12 m plus the float32 origin rounding, at most
eps / 10; (K3) every disagreement along the manifest no deeper than eps inside the box; plus V1's range tolerance,
adapter mapping and positive controls.

The kernel ray is computed with the kernel's own float32 Warp operations (normalize, quat_rotate, normalize;
warp_lidar_kernels.py draw_optimized_kernel_range_segmentation_target) on the Warp CPU device; the scan itself comes
from cplanner/motar/replay.py as in M2.
usage: v1v2_kernelray.py OUT_JSON FRAMES_PER_CELL WORKERS CELL_DIR [CELL_DIR ...]
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
sys.path.insert(0, str(HERE))
import v1_hitbit as V                                # noqa: E402  (oracle, sampling, paths)
import adapter as AD                                 # noqa: E402
import registry as REG                               # noqa: E402
import replay as RP                                  # noqa: E402
import warp as wp                                    # noqa: E402

SAMPLE_SEED = 20261109
EPS = REG.CONFIGS["MOTAR-72x4-RUNTIME-DIR"].direction_error_m
DELTA_GEOM = 1.0e-5          # allowed depth of a kernel-ray disagreement inside a box (float32 vertex rounding)
LATERAL_MAX = EPS / 10.0     # K2 bound
TOL_RANGE = V.TOL_RANGE
BAND = V.BAND

@wp.kernel
def _kray_kernel(q: wp.array(dtype=wp.quat), rv: wp.array2d(dtype=wp.vec3), out: wp.array2d(dtype=wp.vec3)):
    i, j = wp.tid()
    d = wp.normalize(rv[i, j])
    out[i, j] = wp.normalize(wp.quat_rotate(q[0], d))


def kernel_rays(sensor_quat_xyzw):
    """(H*W, 3) float64 copy of the float32 world ray directions exactly as the LiDAR kernel computes them."""
    RP.kernels()                                      # wp.init()
    rv = wp.array(RP.ray_vectors(), dtype=wp.vec3, device=RP.DEVICE)
    q = wp.array(np.asarray(sensor_quat_xyzw, np.float32).reshape(1, 4), dtype=wp.quat, device=RP.DEVICE)
    out = wp.zeros((RP.H, RP.W), dtype=wp.vec3, device=RP.DEVICE)
    wp.launch(_kray_kernel, dim=(RP.H, RP.W), inputs=[q, rv, out], device=RP.DEVICE)
    return out.numpy().reshape(-1, 3).astype(np.float64)


def depth_inside(origin, d, c, h, q, t_hit):
    """Largest depth below the box surface reached by the ray o + t d (t >= 0) inside the oriented box (0 if it only
    touches): sampled on 65 points of the chord, starting from the entry at t_hit."""
    qi = (-q[0], -q[1], -q[2], q[3])
    o = V.quat_rotate_xyzw(qi, np.asarray(origin, float) - np.asarray(c, float))
    dl = V.quat_rotate_xyzw(qi, np.asarray(d, float))
    with np.errstate(divide="ignore", invalid="ignore"):
        t1, t2 = (-np.asarray(h) - o) / dl, (np.asarray(h) - o) / dl
    lo = np.where(np.abs(dl) < 1e-15, np.where(np.abs(o) <= h, -np.inf, np.inf), np.minimum(t1, t2)).max()
    hi = np.where(np.abs(dl) < 1e-15, np.where(np.abs(o) <= h, np.inf, -np.inf), np.maximum(t1, t2)).min()
    lo = max(lo, 0.0)
    if not hi >= lo:
        return 0.0
    ts = np.linspace(lo, hi, 65)
    pts = o[None, :] + ts[:, None] * dl[None, :]
    return float(max(0.0, (np.asarray(h)[None, :] - np.abs(pts)).min(1).max()))


def classify_beam(vb, tb):
    """V1's rule-1 kinds for value vb against oracle distance tb (None: consistent)."""
    band = abs(tb - RP.MAX_RANGE) <= BAND or abs(tb - RP.MIN_RANGE) <= BAND
    if vb == 1.0:
        return "nohit_but_oracle_hit" if (tb <= RP.MAX_RANGE and not band) else None
    if vb == -1.0:
        return "near_but_oracle_far" if (tb >= RP.MIN_RANGE and not band) else None
    return "hit_but_oracle_none" if not np.isfinite(tb) else None


def check_frames(items):
    st = {"frames": 0, "beams": 0, "nonfinite": 0, "hit": 0, "no_hit": 0, "near": 0, "target_hits": 0,
          "k_disagree": 0, "k_disagree_deep": 0, "k_max_depth_m": 0.0, "k_range_mismatch": 0,
          "k_max_abs_range_err_m": 0.0, "k_adapter_range_beyond_true": 0, "m_disagree": 0, "m_disagree_deep": 0,
          "m_max_depth_m": 0.0, "max_lateral_m": 0.0, "max_angle_rad": 0.0, "max_origin_round_m": 0.0,
          "adapter_status_mismatch": 0, "examples": []}
    man = REG.directions("MOTAR-72x4").reshape(-1, 3)
    scenes = {}
    for it in items:
        key = (it["cell"], it["env"], it["episode"])
        if key not in scenes:
            scenes = {key: RP.Scene(it["c"], it["h"], it["qb"])}
        sc = scenes[key]
        yaw, sq = RP.sensor_yaw(it["q"])
        p32 = it["p"].astype(np.float32)
        t32 = it["t"].astype(np.float32)
        v = sc.scan(p32, sq, t32).reshape(-1).astype(np.float64)
        cy, sy = math.cos(yaw), math.sin(yaw)
        dm = man @ np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1.0]]).T          # certifier's manifest rays
        dk = kernel_rays(sq)                                                     # kernel's float32 rays
        ok = p32.astype(np.float64)
        ang = np.arccos(np.clip((dm * dk).sum(1) / np.linalg.norm(dk, axis=1), -1.0, 1.0))
        origin_round = float(np.linalg.norm(ok - it["p"]))
        lateral = RP.MAX_RANGE * np.sin(ang) + origin_round
        st["max_lateral_m"] = max(st["max_lateral_m"], float(lateral.max()))
        st["max_angle_rad"] = max(st["max_angle_rad"], float(ang.max()))
        st["max_origin_round_m"] = max(st["max_origin_round_m"], origin_round)
        tk, wk = V.oracle(ok, dk, it["c"], it["h"], it["qb"], t32.astype(np.float64), RP.TARGET_RADIUS)
        tm, wm = V.oracle(it["p"], dm, it["c"], it["h"], it["qb"], it["t"], RP.TARGET_RADIUS)
        ranges, status = AD.derive(v.reshape(AD.SHAPE))
        ranges, status = ranges.reshape(-1), status.reshape(-1)
        st["frames"] += 1
        st["beams"] += len(v)
        for b in range(len(v)):
            vb = v[b]
            if not np.isfinite(vb):
                st["nonfinite"] += 1
                continue
            if vb == 1.0:
                st["no_hit"] += 1
            elif vb == -1.0:
                st["near"] += 1
            else:
                st["hit"] += 1
                st["target_hits"] += int(wk[b] == -2)
                if np.isfinite(tk[b]):
                    err = abs(12.0 * vb - tk[b])
                    st["k_max_abs_range_err_m"] = max(st["k_max_abs_range_err_m"], err)
                    st["k_range_mismatch"] += int(err > TOL_RANGE)
                    st["k_adapter_range_beyond_true"] += int(status[b] == AD.HIT and ranges[b] > tk[b])
            want = AD.NO_HIT if vb == 1.0 else AD.NEAR_CLIPPED if vb == -1.0 else (
                AD.HIT if 12.0 * vb - AD.HIT_MARGIN_M >= AD.MIN_RANGE else AD.NEAR_CLIPPED)
            st["adapter_status_mismatch"] += int(status[b] != want)
            for tag, t_, w_, o_, d_ in (("k", tk, wk, ok, dk), ("m", tm, wm, it["p"], dm)):
                kind = classify_beam(vb, t_[b])
                if kind is None:
                    continue
                own = int(w_[b])
                if own >= 0 and np.isfinite(t_[b]):
                    dep = depth_inside(o_, d_[b], it["c"][own], it["h"][own], it["qb"][own], t_[b])
                elif own == -2:                          # target sphere: radius minus the ray's closest approach
                    ctr = (t32.astype(np.float64) if tag == "k" else it["t"]) - np.asarray(o_, float)
                    dd = d_[b] / np.linalg.norm(d_[b])
                    dep = max(0.0, RP.TARGET_RADIUS - float(np.linalg.norm(ctr - (ctr @ dd) * dd)))
                else:
                    dep = math.inf                       # no object: no geometric excuse
                lim = DELTA_GEOM if tag == "k" else EPS
                st[tag + "_disagree"] += 1
                st[tag + "_max_depth_m"] = max(st[tag + "_max_depth_m"], dep)
                st[tag + "_disagree_deep"] += int(dep > lim)
                if len(st["examples"]) < 20:
                    st["examples"].append({"ray": tag, "cell": it["cell"], "k": it["k"], "beam": b, "v": vb,
                                           "t_oracle": float(t_[b]), "owner": own, "kind": kind, "depth_m": dep})
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
        items += V.frames_for(c, n, rng)
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
            if k.startswith("max_") or k.endswith("_max_depth_m") or k.startswith("k_max"):
                st[k] = max(st.get(k, 0.0), v)
            elif isinstance(v, list):
                st.setdefault(k, []).extend(v)
            else:
                st[k] = st.get(k, 0) + v
    st["examples"] = st.get("examples", [])[:20]
    g = lambda k, d=1: st.get(k, d)                                 # noqa: E731
    checks = {
        "completed": failure is None and g("frames", 0) == len(items),
        "K1_kernel_ray_truthful": failure is None and g("nonfinite") == 0 and g("k_disagree_deep") == 0
        and g("k_range_mismatch") == 0 and g("k_adapter_range_beyond_true") == 0,
        "K2_lateral_within_eps_over_10": failure is None and g("max_lateral_m", 1.0) <= LATERAL_MAX,
        "K3_manifest_disagreements_within_eps": failure is None and g("m_disagree_deep") == 0,
        "range_error_below_half_margin": g("k_max_abs_range_err_m", 1.0) <= AD.HIT_MARGIN_M / 2,
        "adapter_mapping_exact": g("adapter_status_mismatch") == 0,
        "positive_controls": g("hit", 0) > 0 and g("no_hit", 0) > 0 and g("target_hits", 0) > 0,
    }
    checks = {k: bool(v) for k, v in checks.items()}
    res = {"gate": "C-Planner direction bound D2 (V1-v2)", "pass": all(checks.values()), "checks": checks,
           "stats": st, "failure": failure, "cells": cells, "frames_per_cell": n, "sample_seed": SAMPLE_SEED,
           "eps_m": EPS, "delta_geom_m": DELTA_GEOM, "lateral_max_m": LATERAL_MAX, "tol_range_m": TOL_RANGE,
           "band_m": BAND, "hit_margin_m": AD.HIT_MARGIN_M,
           "kernel_file_sha256": hashlib.sha256(RP.KERNEL_FILE.read_bytes()).hexdigest(), "wall_s": time.time() - t0}
    with out.open("x") as fh:
        json.dump(res, fh, indent=1, default=float)
        fh.write("\n")
    print(json.dumps({"pass": res["pass"], "checks": checks, "beams": st.get("beams"),
                      "k_disagree": st.get("k_disagree"), "m_disagree": st.get("m_disagree"),
                      "max_lateral_m": st.get("max_lateral_m"), "wall_s": res["wall_s"]}, indent=1, default=float))
    return 0 if res["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
