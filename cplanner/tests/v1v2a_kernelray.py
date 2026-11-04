"""C-Planner direction-bound repair, gate D2-A1: V1-v2 with the disagreement direction taken into account
(docs/plans/prereg_cplanner_dirbound_v1_amendment1_2026-11-04.md; the original D2 is v1v2_kernelray.py, unchanged).

D2 failed on corner grazing in the conservative direction. The kernel's float32 ray hit a bar's vertical edge that
the float64 ray just misses, so a HIT was declared where the recorded layout shows free space. A conservative
declaration only removes free evidence or adds a surface, so it cannot make a certificate unsound. A permissive one
can. This version classifies every disagreement between a declaration and the oracle along a ray:
  PERMISSIVE  NO_HIT while the oracle hits within 12 m (outside V1's 0.1 mm range band), or a HIT range 12 v that
              overshoots the oracle distance by more than half the adapter margin (0.5 mm);
              allowed only if the ray is at most LIMIT deep inside the object it misses
  CONSERVATIVE  NEAR_CLIPPED while the oracle is beyond 0.2 m (outside the band), or a HIT whose oracle distance is
              longer than 12 v + 0.5 mm (or infinite); allowed only if the declared point (o + 12 v d; for
              NEAR_CLIPPED the nearest point of the first 0.2 m) lies within LIMIT of an object surface, i.e. the
              disagreement is grazing and not a phantom object
LIMIT is DELTA_GEOM = 1e-5 m along the kernel's ray (K1) and eps = 1e-3 m along the manifest (K3). K2, the adapter
mapping and the positive controls are unchanged. Rule controls per frame: one real bar HIT rewritten as NO_HIT and one
phantom HIT at 3 m on an empty beam must both be flagged unexcused. The original D2 checks are recomputed and reported, not gated.
usage: v1v2a_kernelray.py OUT_JSON FRAMES_PER_CELL WORKERS SAMPLE_SEED CELL_DIR [CELL_DIR ...]
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
import v1v2_kernelray as B                           # noqa: E402  (kernel rays, depth, original rules)
import v1_hitbit as V                                # noqa: E402
import adapter as AD                                 # noqa: E402
import registry as REG                               # noqa: E402
import replay as RP                                  # noqa: E402

EPS, DELTA_GEOM, LATERAL_MAX, TOL_RANGE, BAND = B.EPS, B.DELTA_GEOM, B.LATERAL_MAX, B.TOL_RANGE, B.BAND


def surface_distance(P, c, h, q, tgt, r_t):
    """Distance from point P to the nearest object surface (oriented boxes, target sphere)."""
    best = abs(float(np.linalg.norm(np.asarray(P, float) - tgt)) - r_t)
    for cb, hb, qb in zip(c, h, q):
        qi = (-qb[0], -qb[1], -qb[2], qb[3])
        loc = V.quat_rotate_xyzw(qi, np.asarray(P, float) - cb)
        out = np.maximum(np.abs(loc) - hb, 0.0)
        d = float(np.linalg.norm(out)) if out.any() else float((hb - np.abs(loc)).min())
        best = min(best, d)
    return best


def segment_surface_distance(o, d, t1, c, h, q, tgt, r_t, n=21):
    return min(surface_distance(o + t * d, c, h, q, tgt, r_t) for t in np.linspace(0.0, t1, n))


def classify(vb, tb, band_lo=None):
    """(direction, kind) of a disagreement between value vb and oracle distance tb, or None."""
    band = abs(tb - RP.MAX_RANGE) <= BAND or abs(tb - RP.MIN_RANGE) <= BAND
    if vb == 1.0:
        return ("PERMISSIVE", "nohit_but_oracle_hit") if (tb <= RP.MAX_RANGE and not band) else None
    if vb == -1.0:
        return ("CONSERVATIVE", "near_but_oracle_far") if (tb >= RP.MIN_RANGE and not band) else None
    r = 12.0 * vb
    if np.isfinite(tb) and r - tb > TOL_RANGE:
        return ("PERMISSIVE", "hit_range_beyond_oracle")
    if (not np.isfinite(tb)) or tb - r > TOL_RANGE:
        return ("CONSERVATIVE", "hit_short_of_oracle")
    return None


def check_frames(items):
    st = {"frames": 0, "beams": 0, "nonfinite": 0, "hit": 0, "no_hit": 0, "near": 0, "target_hits": 0,
          "adapter_status_mismatch": 0, "k_adapter_range_beyond_true": 0, "max_lateral_m": 0.0,
          "max_angle_rad": 0.0, "k_max_abs_range_err_m": 0.0, "k_max_permissive_overshoot_m": 0.0, "examples": []}
    for tag in ("k", "m"):
        for direc in ("PERMISSIVE", "CONSERVATIVE"):
            st["%s_%s" % (tag, direc)] = 0
            st["%s_%s_unexcused" % (tag, direc)] = 0
            st["%s_%s_max_excuse_m" % (tag, direc)] = 0.0
        st[tag + "_original_rule_disagree"] = 0
    st.update(control_permissive=0, control_permissive_flagged=0, control_conservative=0,
              control_conservative_flagged=0)
    man = REG.directions("MOTAR-72x4").reshape(-1, 3)
    scenes = {}
    for it in items:
        key = (it["cell"], it["env"], it["episode"])
        if key not in scenes:
            scenes = {key: RP.Scene(it["c"], it["h"], it["qb"])}
        sc = scenes[key]
        yaw, sq = RP.sensor_yaw(it["q"])
        p32, t32 = it["p"].astype(np.float32), it["t"].astype(np.float32)
        v = sc.scan(p32, sq, t32).reshape(-1).astype(np.float64)
        cy, sy = math.cos(yaw), math.sin(yaw)
        dm = man @ np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1.0]]).T
        dk = B.kernel_rays(sq)
        ok = p32.astype(np.float64)
        ang = np.arccos(np.clip((dm * dk).sum(1) / np.linalg.norm(dk, axis=1), -1.0, 1.0))
        st["max_lateral_m"] = max(st["max_lateral_m"],
                                  float((RP.MAX_RANGE * np.sin(ang) + float(np.linalg.norm(ok - it["p"]))).max()))
        st["max_angle_rad"] = max(st["max_angle_rad"], float(ang.max()))
        tk, wk = V.oracle(ok, dk, it["c"], it["h"], it["qb"], t32.astype(np.float64), RP.TARGET_RADIUS)
        tm, wm = V.oracle(it["p"], dm, it["c"], it["h"], it["qb"], it["t"], RP.TARGET_RADIUS)
        ranges, status = AD.derive(v.reshape(AD.SHAPE))
        ranges, status = ranges.reshape(-1), status.reshape(-1)
        st["frames"] += 1
        st["beams"] += len(v)
        # positive controls (non-vacuity), along the kernel ray: a HIT that really enters a bar at least 1 cm deep is
        # rewritten as NO_HIT (must be PERMISSIVE and unexcused); a NO_HIT beam whose oracle sees nothing gets a
        # phantom HIT at 3 m (must be CONSERVATIVE and unexcused)
        for b in range(len(v)):
            own = int(wk[b])
            if 0.0 < v[b] < 1.0 and own >= 0 and np.isfinite(tk[b]) and B.depth_inside(
                    ok, dk[b], it["c"][own], it["h"][own], it["qb"][own], tk[b]) > 0.01:
                st["control_permissive"] += 1
                cls = classify(1.0, tk[b])
                dep = B.depth_inside(ok, dk[b], it["c"][own], it["h"][own], it["qb"][own], tk[b])
                st["control_permissive_flagged"] += int(cls is not None and cls[0] == "PERMISSIVE" and dep > DELTA_GEOM)
                break
        for b in range(len(v)):
            if v[b] == 1.0 and not np.isfinite(tk[b]):
                st["control_conservative"] += 1
                cls = classify(3.0 / 12.0, tk[b])
                exc = surface_distance(ok + 3.0 * dk[b], it["c"], it["h"], it["qb"], t32.astype(np.float64),
                                       RP.TARGET_RADIUS)
                st["control_conservative_flagged"] += int(cls is not None and cls[0] == "CONSERVATIVE"
                                                          and exc > DELTA_GEOM)
                break
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
                    st["k_max_abs_range_err_m"] = max(st["k_max_abs_range_err_m"], abs(12.0 * vb - tk[b]))
                    st["k_max_permissive_overshoot_m"] = max(st["k_max_permissive_overshoot_m"], 12.0 * vb - tk[b])
                    st["k_adapter_range_beyond_true"] += int(status[b] == AD.HIT and ranges[b] > tk[b])
            want = AD.NO_HIT if vb == 1.0 else AD.NEAR_CLIPPED if vb == -1.0 else (
                AD.HIT if 12.0 * vb - AD.HIT_MARGIN_M >= AD.MIN_RANGE else AD.NEAR_CLIPPED)
            st["adapter_status_mismatch"] += int(status[b] != want)
            for tag, t_, w_, o_, d_, tgt in (("k", tk, wk, ok, dk, t32.astype(np.float64)),
                                             ("m", tm, wm, it["p"], dm, it["t"])):
                st[tag + "_original_rule_disagree"] += int(B.classify_beam(vb, t_[b]) is not None)
                cls = classify(vb, t_[b])
                if cls is None:
                    continue
                direc, kind = cls
                own = int(w_[b])
                if kind == "nohit_but_oracle_hit":
                    if own >= 0:
                        exc = B.depth_inside(o_, d_[b], it["c"][own], it["h"][own], it["qb"][own], t_[b])
                    else:
                        ctr = tgt - np.asarray(o_, float)
                        dd = d_[b] / np.linalg.norm(d_[b])
                        exc = max(0.0, RP.TARGET_RADIUS - float(np.linalg.norm(ctr - (ctr @ dd) * dd)))
                elif kind == "hit_range_beyond_oracle":
                    exc = float(12.0 * vb - t_[b])                    # overshoot: never excused below
                elif kind == "hit_short_of_oracle":
                    exc = surface_distance(np.asarray(o_, float) + 12.0 * vb * d_[b], it["c"], it["h"], it["qb"],
                                           tgt, RP.TARGET_RADIUS)
                else:                                               # near_but_oracle_far
                    exc = segment_surface_distance(np.asarray(o_, float), d_[b], RP.MIN_RANGE, it["c"], it["h"],
                                                   it["qb"], tgt, RP.TARGET_RADIUS)
                lim = DELTA_GEOM if tag == "k" else EPS
                unexcused = (kind == "hit_range_beyond_oracle") or exc > lim
                k = "%s_%s" % (tag, direc)
                st[k] += 1
                st[k + "_unexcused"] += int(unexcused)
                st[k + "_max_excuse_m"] = max(st[k + "_max_excuse_m"], exc)
                if len(st["examples"]) < 30:
                    st["examples"].append({"ray": tag, "cell": it["cell"], "k": it["k"], "beam": b, "v": vb,
                                           "t_oracle": float(t_[b]), "owner": own, "direction": direc, "kind": kind,
                                           "excuse_m": exc, "unexcused": bool(unexcused)})
    return st


def main():
    out = pathlib.Path(sys.argv[1])
    n, workers, seed, cells = int(sys.argv[2]), int(sys.argv[3]), int(sys.argv[4]), sys.argv[5:]
    if out.exists():
        raise SystemExit("refusing to overwrite " + str(out))
    t0 = time.time()
    rng = random.Random(seed)
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
            if k.startswith(("max_", "k_max")) or k.endswith("_max_excuse_m"):
                st[k] = max(st.get(k, -math.inf), v)
            elif isinstance(v, list):
                st.setdefault(k, []).extend(v)
            else:
                st[k] = st.get(k, 0) + v
    st["examples"] = st.get("examples", [])[:30]
    g = lambda k, d=1: st.get(k, d)                                 # noqa: E731
    checks = {
        "completed": failure is None and g("frames", 0) == len(items),
        "A_K1_kernel_ray_no_unexcused": failure is None and g("nonfinite") == 0
        and g("k_PERMISSIVE_unexcused") == 0 and g("k_CONSERVATIVE_unexcused") == 0
        and g("k_adapter_range_beyond_true") == 0,
        "K2_lateral_within_eps_over_10": failure is None and g("max_lateral_m", 1.0) <= LATERAL_MAX,
        "A_K3_manifest_no_unexcused": failure is None and g("m_PERMISSIVE_unexcused") == 0
        and g("m_CONSERVATIVE_unexcused") == 0,
        "A_permissive_overshoot_below_half_margin": g("k_max_permissive_overshoot_m", 1.0) <= AD.HIT_MARGIN_M / 2,
        "adapter_mapping_exact": g("adapter_status_mismatch") == 0,
        "positive_controls": g("hit", 0) > 0 and g("no_hit", 0) > 0 and g("target_hits", 0) > 0,
        "A_rule_controls_flagged": g("control_permissive", 0) > 0 and g("control_conservative", 0) > 0
        and g("control_permissive_flagged", 0) == g("control_permissive", 0)
        and g("control_conservative_flagged", 0) == g("control_conservative", 0),
    }
    checks = {k: bool(v) for k, v in checks.items()}
    res = {"gate": "C-Planner direction bound D2-A1 (V1-v2, direction-aware)", "pass": all(checks.values()),
           "checks": checks, "stats": st, "failure": failure, "cells": cells, "frames_per_cell": n,
           "sample_seed": seed, "eps_m": EPS, "delta_geom_m": DELTA_GEOM, "lateral_max_m": LATERAL_MAX,
           "tol_range_m": TOL_RANGE, "band_m": BAND, "hit_margin_m": AD.HIT_MARGIN_M,
           "kernel_file_sha256": hashlib.sha256(RP.KERNEL_FILE.read_bytes()).hexdigest(), "wall_s": time.time() - t0}
    with out.open("x") as fh:
        json.dump(res, fh, indent=1, default=float)
        fh.write("\n")
    print(json.dumps({"pass": res["pass"], "checks": checks, "beams": st.get("beams"),
                      "k_PERMISSIVE": st.get("k_PERMISSIVE"), "k_CONSERVATIVE": st.get("k_CONSERVATIVE"),
                      "m_PERMISSIVE": st.get("m_PERMISSIVE"), "m_CONSERVATIVE": st.get("m_CONSERVATIVE"),
                      "wall_s": res["wall_s"]}, indent=1, default=float))
    return 0 if res["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
