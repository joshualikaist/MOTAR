"""Independent recomputation for the G3 c2trace analysis review (reviewer: Opus 5.5). Read-only on inputs.

Own frame parser (dtype from each cell's summary), own grouping, own geometry; does not import the analysis tool or
motar_eval. Writes only to argv[1] (a JSON in the review directory).
"""
import hashlib
import json
import math
import pathlib
import sys

import numpy as np

RUN = pathlib.Path("/home/fair/workspaces/aerial_gym_ws/src/aerial_gym_simulator/results/camonly_c2trace_v1")
ANA = pathlib.Path("/home/fair/workspaces/aerial_gym_ws/src/MOTAR-camonly/docs/evidence/camonly_2026-11-04/c2trace/"
                   "c2trace_analysis.json")
OUT = pathlib.Path(sys.argv[1])
CONDS = {"A": "full", "B_out": "no_lidar_out_fov", "C": "no_lidar"}
DENS = (70, 115, 160, 205)
HALF_DEG = 43.5
CAM_T = 0.1  # camera translation along vehicle x (summary metadata)


def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()


def dtype_from_summary(s):
    fields = []
    for f in s["frame_dtype"]:
        if len(f) == 3:
            fields.append((f[0], f[1], tuple(f[2])))
        else:
            fields.append((f[0], f[1]))
    dt = np.dtype(fields)
    assert dt.itemsize == s["frame_itemsize"]
    return dt


def yaw_xyzw(q):
    x, y, z, w = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    return np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def ang(a):
    return np.arctan2(np.sin(a), np.cos(a))


def closest(p, c, h):
    """p [3]; c,h [B,3]. Horizontal surface distance (inf if box does not span p_z) and offset to closest point."""
    span = np.abs(p[2] - c[:, 2]) <= h[:, 2]
    q = np.minimum(np.maximum(p[:2], c[:, :2] - h[:, :2]), c[:, :2] + h[:, :2])
    off = q - p[:2]
    d = np.sqrt((off ** 2).sum(1))
    d = np.where(span, d, np.inf)
    return d, off, span


def cell(cdir, stats):
    s = json.loads((cdir / "occlusion_trace_summary.json").read_text())
    for fn, rec in s["files"].items():
        assert sha(cdir / fn) == rec, fn
    fr = np.frombuffer((cdir / "occlusion_frames.bin").read_bytes(), dtype=dtype_from_summary(s))
    assert len(fr) == s["frames"]
    lay = np.load(cdir / "occlusion_layouts.npz")
    center, half, quat, offs, resets = lay["center"], lay["half"], lay["quat"], lay["offsets"], lay["resets"]
    # bar orientation / shape audit
    stats["quat_nonidentity_rows"] += int((np.abs(quat - np.array([0, 0, 0, 1], np.float32)).max(1) > 1e-6).sum())
    stats["quat_rows"] += len(quat)
    stats["half_xy_min"] = min(stats["half_xy_min"], float(half[:, :2].min()))
    stats["half_xy_max"] = max(stats["half_xy_max"], float(half[:, :2].max()))
    stats["half_z_min"] = min(stats["half_z_min"], float(half[:, 2].min()))
    stats["half_z_max"] = max(stats["half_z_max"], float(half[:, 2].max()))
    stats["bar_zc_min"] = min(stats["bar_zc_min"], float(center[:, 2].min()))
    stats["bar_zc_max"] = max(stats["bar_zc_max"], float(center[:, 2].max()))
    rl = {}
    for r in resets:
        k = (int(r["env"]), int(r["episode"]))
        assert k not in rl
        rl[k] = int(r["layout"])
    # group frames by (env, episode) with a stable argsort on a combined key
    key = fr["env"].astype(np.int64) * (1 << 32) + fr["episode"].astype(np.int64)
    order = np.argsort(key, kind="stable")
    ks = key[order]
    cut = np.flatnonzero(np.diff(ks)) + 1
    starts = np.concatenate([[0], cut])
    ends = np.concatenate([cut, [len(ks)]])
    groups = {}
    for a, b in zip(starts, ends):
        idx = order[a:b]
        idx = idx[np.argsort(fr["frame"][idx], kind="stable")]
        groups[(int(ks[a] >> 32), int(ks[a] & 0xFFFFFFFF))] = idx
    ledger = [json.loads(l) for l in open(cdir / "episodes.jsonl")]
    kroc = {(e["env_index"], e["env_episode_index"]): e for e in map(json.loads, open(cdir / "kroc2027_episodes.jsonl"))}
    stats["ledger_rows"] += len(ledger)
    out = []
    # travel_out over all frames (as prereg: share of moving frames)
    vel = fr["p_vel"].astype(np.float64)
    yaw_all = yaw_xyzw(fr["p_vehicle_quat"].astype(np.float64))
    yaw_body = yaw_xyzw(fr["p_quat"].astype(np.float64))
    stats["max_abs_yaw_vehicle_minus_body_deg"] = max(stats["max_abs_yaw_vehicle_minus_body_deg"],
                                                      float(np.degrees(np.abs(ang(yaw_all - yaw_body))).max()))
    spd = np.hypot(vel[:, 0], vel[:, 1])
    mv = spd > 0.3
    vb = ang(np.arctan2(vel[:, 1], vel[:, 0]) - yaw_all)
    stats_cell = {"moving": int(mv.sum()), "moving_out": int((np.abs(vb[mv]) > math.radians(HALF_DEG)).sum())}
    # frames of episodes that are in the ledger only (sensitivity)
    led_keys = {(e["env_index"], e["env_episode_index"]) for e in ledger}
    mask_led = np.zeros(len(fr), bool)
    for k, idx in groups.items():
        if k in led_keys:
            mask_led[idx] = True
    stats_cell["moving_ledger_only"] = int((mv & mask_led).sum())
    stats_cell["moving_out_ledger_only"] = int((mv & mask_led & (np.abs(vb) > math.radians(HALF_DEG))).sum())
    for e in ledger:
        k = (e["env_index"], e["env_episode_index"])
        kr = kroc.get(k)
        if kr is None:
            stats["ledger_without_kroc"] += 1
        elif kr["outcome"] != e["outcome"]:
            stats["ledger_kroc_outcome_mismatch"] += 1
        if k not in groups:
            stats["ledger_without_frames"] += 1
            if e["outcome"] == "crash":
                stats["crash_without_frames"] += 1
            continue
        if k not in rl:
            stats["ledger_without_layout"] += 1
            if e["outcome"] == "crash":
                stats["crash_without_layout"] += 1
            continue
        idx = groups[k]
        fnums = fr["frame"][idx].astype(int)
        if not (fnums[0] == 0 and np.all(np.diff(fnums) == 1)):
            stats["episodes_noncontiguous_frames"] += 1
        if kr is not None and "actions" in kr and kr["actions"] != len(idx):
            stats["frames_ne_actions"] += 1
        if e["outcome"] == "capture":
            last = idx[-1]
            dtg = float(np.linalg.norm(fr["t_pos"][last].astype(float) - fr["p_pos"][last].astype(float)))
            stats["capture_last_target_dist"].append(dtg)
            continue
        if e["outcome"] != "crash":
            continue
        li = rl[k]
        c = center[offs[li]:offs[li + 1]].astype(np.float64)
        h = half[offs[li]:offs[li + 1]].astype(np.float64)
        pos = fr["p_pos"][idx].astype(np.float64)
        yw = yaw_all[idx]
        d, off, span = closest(pos[-1], c, h)
        if not span.any():
            stats["crash_no_spanning_bar"] += 1
        j = int(np.argmin(d))
        # second-nearest margin (ambiguity of the contact bar)
        ds = np.sort(d)
        second = float(ds[1]) if len(ds) > 1 else float("inf")
        b = float(ang(math.atan2(off[j, 1], off[j, 0]) - yw[-1]))
        # camera-eye bearing (0.1 m forward along yaw)
        eye = pos[-1, :2] + CAM_T * np.array([math.cos(yw[-1]), math.sin(yw[-1])])
        qpt = pos[-1, :2] + off[j]
        be = float(ang(math.atan2(qpt[1] - eye[1], qpt[0] - eye[0]) - yw[-1]))
        nin = True
        for kk in range(max(0, len(idx) - 11), len(idx)):
            _, o2, _ = closest(pos[kk], c, h)
            nin = nin and abs(float(ang(math.atan2(o2[j, 1], o2[j, 0]) - yw[kk]))) > math.radians(HALF_DEG)
        out.append({"key": k, "density": int(e["bars"]), "in_quota": bool(e["in_quota"]), "dist": float(d[j]),
                    "second": second, "bearing_deg": math.degrees(b), "eye_bearing_deg": math.degrees(be),
                    "out": abs(math.degrees(b)) > HALF_DEG, "eye_out": abs(math.degrees(be)) > HALF_DEG,
                    "never_in_1s": bool(nin), "nframes": int(len(idx)),
                    "short": len(idx) < 11})
    out.sort(key=lambda r: r["key"])
    return out, stats_cell


def boot(x, y, rng, n=10000):
    x, y = np.asarray(x, float), np.asarray(y, float)
    bx = rng.integers(0, len(x), size=(n, len(x)))
    by = rng.integers(0, len(y), size=(n, len(y)))
    d = x[bx].mean(1) - y[by].mean(1)
    return [float(np.percentile(d, 2.5) * 100), float(np.percentile(d, 97.5) * 100)]


def main():
    ana = json.loads(ANA.read_text())
    stats = {"quat_nonidentity_rows": 0, "quat_rows": 0, "half_xy_min": 9e9, "half_xy_max": -9e9, "half_z_min": 9e9,
             "half_z_max": -9e9, "bar_zc_min": 9e9, "bar_zc_max": -9e9, "ledger_rows": 0, "ledger_without_kroc": 0,
             "ledger_kroc_outcome_mismatch": 0, "ledger_without_frames": 0, "crash_without_frames": 0,
             "ledger_without_layout": 0, "crash_without_layout": 0, "episodes_noncontiguous_frames": 0,
             "frames_ne_actions": 0, "crash_no_spanning_bar": 0, "capture_last_target_dist": [],
             "max_abs_yaw_vehicle_minus_body_deg": 0.0}
    sha_mismatch = []
    res = {"conditions": {}, "per_density": {}}
    pooled = {}
    for name, cond in CONDS.items():
        allc, mv, mvo, mvl, mvlo = [], 0, 0, 0, 0
        for dd in DENS:
            cdir = RUN / cond / ("E2_obstacle_aware__%dbars__seed4501" % dd)
            for fn in ("occlusion_frames.bin", "occlusion_layouts.npz", "episodes.jsonl"):
                if ana["inputs_sha256"]["%s/%d/%s" % (cond, dd, fn)] != sha(cdir / fn):
                    sha_mismatch.append("%s/%d/%s" % (cond, dd, fn))
            cr, sc = cell(cdir, stats)
            allc += cr
            mv += sc["moving"]
            mvo += sc["moving_out"]
            mvl += sc["moving_ledger_only"]
            mvlo += sc["moving_out_ledger_only"]
            n = len(cr)
            res["per_density"]["%s/%d" % (name, dd)] = {
                "crash": n, "crash_quota": sum(c["in_quota"] for c in cr),
                "out_last_pct": 100 * sum(c["out"] for c in cr) / max(1, n),
                "out_last_quota_pct": 100 * sum(c["out"] for c in cr if c["in_quota"]) / max(1, sum(c["in_quota"] for c in cr)),
                "valid_pct": 100 * sum(c["dist"] <= 0.6 for c in cr) / max(1, n)}
        pooled[name] = allc
        n = len(allc)
        q = [c for c in allc if c["in_quota"]]
        v = [c for c in allc if c["dist"] <= 0.6]
        lt = [c for c in allc if not c["short"]]
        dists = np.array([c["dist"] for c in allc])
        marg = np.array([c["second"] - c["dist"] for c in allc])
        res["conditions"][name] = {
            "crash_episodes": n, "crash_quota": len(q),
            "valid_share_pct": 100 * float((dists <= 0.6).mean()),
            "out_last_pct": 100 * sum(c["out"] for c in allc) / n,
            "out_last_quota_only_pct": 100 * sum(c["out"] for c in q) / len(q),
            "out_last_valid_only_pct": 100 * sum(c["out"] for c in v) / len(v),
            "out_last_camera_eye_pct": 100 * sum(c["eye_out"] for c in allc) / n,
            "never_in_1s_pct": 100 * sum(c["never_in_1s"] for c in allc) / n,
            "never_in_1s_ge11frames_pct": 100 * sum(c["never_in_1s"] for c in lt) / max(1, len(lt)),
            "short_lt11_frames": n - len(lt),
            "left_pct": 100 * sum(c["bearing_deg"] > 0 for c in allc) / n,
            "mean_abs_bearing_deg": float(np.mean([abs(c["bearing_deg"]) for c in allc])),
            "travel_out_of_fov_pct": 100 * mvo / mv, "moving_frames": mv,
            "travel_out_ledger_eps_only_pct": 100 * mvlo / max(1, mvl),
            "last_dist_median_m": float(np.median(dists[np.isfinite(dists)])),
            "last_dist_p95_m": float(np.percentile(dists[np.isfinite(dists)], 95)),
            "last_dist_inf": int((~np.isfinite(dists)).sum()),
            "contact_ambiguous_second_within_0p1m": int((marg < 0.1).sum()),
            "out_last_pct_excluding_ambiguous": 100 * sum(c["out"] for c, m in zip(allc, marg) if m >= 0.1) / max(1, int((marg >= 0.1).sum())),
        }
    rng = np.random.default_rng(20261105)
    o = {k: [float(c["out"]) for c in v] for k, v in pooled.items()}
    res["contrasts_same_rng_as_tool"] = {}
    for x in ("C", "B_out"):
        res["contrasts_same_rng_as_tool"][x + "-A"] = {"delta": 100 * (np.mean(o[x]) - np.mean(o["A"])),
                                                       "ci95": boot(o[x], o["A"], rng)}
    rng2 = np.random.default_rng(12345)
    res["contrast_C-A_other_seed"] = boot(o["C"], o["A"], rng2)
    # quota-only contrast
    oq = {k: [float(c["out"]) for c in v if c["in_quota"]] for k, v in pooled.items()}
    rng3 = np.random.default_rng(20261105)
    res["contrast_C-A_quota_only"] = {"delta": 100 * (np.mean(oq["C"]) - np.mean(oq["A"])), "ci95": boot(oq["C"], oq["A"], rng3)}
    # cluster (cell) bootstrap: resample densities within condition, then episodes within the cell
    rng4 = np.random.default_rng(7)
    per = {k: {dd: np.array([float(c["out"]) for c in v if c["density"] == dd]) for dd in DENS} for k, v in pooled.items()}
    ds = []
    for _ in range(2000):
        m = {}
        for k in ("A", "C"):
            pick = rng4.integers(0, 4, 4)
            vals = np.concatenate([per[k][DENS[i]][rng4.integers(0, len(per[k][DENS[i]]), len(per[k][DENS[i]]))] for i in pick])
            m[k] = vals.mean()
        ds.append(100 * (m["C"] - m["A"]))
    res["contrast_C-A_two_stage_cell_bootstrap_ci95"] = [float(np.percentile(ds, 2.5)), float(np.percentile(ds, 97.5))]
    # density-stratified (matched density weights): mean of per-density deltas weighted by A's crash counts
    w = np.array([len(per["A"][dd]) for dd in DENS], float)
    dd_delta = np.array([100 * (per["C"][dd].mean() - per["A"][dd].mean()) for dd in DENS])
    res["contrast_C-A_per_density_pp"] = dict(zip(map(str, DENS), dd_delta.tolist()))
    res["contrast_C-A_density_weighted_by_A_pp"] = float((w * dd_delta).sum() / w.sum())
    cap = np.array(stats.pop("capture_last_target_dist"))
    stats["capture_last_frame_target_dist_m"] = {"n": int(len(cap)), "median": float(np.median(cap)),
                                                 "p99": float(np.percentile(cap, 99)), "max": float(cap.max())}
    res["audit_stats"] = stats
    res["inputs_sha_mismatch_vs_analysis_json"] = sha_mismatch
    # comparison with the JSON
    cmp = {}
    for k in CONDS:
        a, r = ana["conditions"][k], res["conditions"][k]
        cmp[k] = {f: [a[g], r[f]] for f, g in (("crash_episodes", "crash_episodes_traced"), ("valid_share_pct", "valid_share_pct"),
                                              ("out_last_pct", "out_last_pct"), ("never_in_1s_pct", "never_in_1s_pct"),
                                              ("left_pct", "left_pct"), ("mean_abs_bearing_deg", "mean_abs_bearing_deg"),
                                              ("travel_out_of_fov_pct", "travel_out_of_fov_pct"), ("moving_frames", "moving_frames"))}
    res["json_vs_recomputed"] = cmp
    res["json_contrasts"] = ana["contrasts"]
    res["json_reading"] = ana["reading"]
    OUT.write_text(json.dumps(res, indent=1, sort_keys=True, default=float) + "\n")
    print(json.dumps({"json_vs_recomputed": cmp, "contrasts": res["contrasts_same_rng_as_tool"],
                      "json_contrasts": ana["contrasts"]}, indent=1, default=float))


if __name__ == "__main__":
    main()
