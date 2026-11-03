"""R2' rule 5 (fixed 2026-11-04 before any gate measurement; docs/plans/cplanner_r2prime_rules_v1.md).

Part A, camera non-interference: N random scenes (P0R-72x1 / MOTAR-72x4 alternating), sensor height uniform in
[0.05, 2.95] m, 2 packets. Each packet is cast ONCE by the independent harness and the same arrays are fed to every
run: no camera, 7 camera variants, a fresh track, and a fresh track plus a truthful camera. The JSON of every
`certify` result must be identical across the camera runs; the track may only remove certificates.

Part B, temporal track monotonicity (non-vacuous, replaces the historical 2-packet check, R3 F6): N random
30-packet sequences (3.0 s). Static catalog bars, a moving target with a fresh active track and, in most sequences, a
non-catalog pole that stays put for 22-25 packets and then relocates. The static rule (N_STATIC, T_STATIC, D_SEP) is
therefore crossed and the vacated cells are seen free afterwards. Runs per sequence: base, base+camera, track,
track+camera. The suite fails as VACUOUS unless static cells, conflict cells and track-caused certificate losses
actually occur (thresholds in NON_VACUITY) and valid base certificates exist for both layouts.

Every certificate is classified by the independent oracle (tests/harness.py) with the ACTUAL vehicle height.
Per-seed SHA-256 digests of the certificate JSON of every run are written, so a duplicate run can be compared seed
by seed (hardware-fault / silent-corruption check). A worker that dies makes the suite fail (no Pool hang).

usage: noninterference_v2.py {A,B} OUT_JSON N WORKERS [SEED0]
"""
import collections
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

from harness import CATALOG, I, bar, camera, classify, core, header, noncat_box, pole, scan, target

LAYOUTS = ("P0R-72x1", "MOTAR-72x4")
CFG = {"P0R-72x1": "SYNTH-72x1", "MOTAR-72x4": "SYNTH-72x4"}
A_BASE, B_BASE = 1_000_000, 2_000_000
DT = 0.1
STEPS_A, K_A = 2, 4
STEPS_B = 30
CERT_STEPS_B = (5, 10, 15) + tuple(range(20, STEPS_B + 1))
VARIANTS = ("truthful_noisy", "biased_minus3sigma", "wrong_extrinsic", "false_positive_on_bar", "random_finite",
            "nonfinite", "malformed_array")
# Part B non-vacuity, fixed before the gate run (fractions of the N sequences of the run).
NON_VACUITY = {"sequences_with_static_min_frac": 0.25, "sequences_with_conflict_min_frac": 0.10,
               "sequences_with_track_loss_min_frac": 0.05}


def digest(obj):
    return hashlib.sha256(json.dumps(obj, sort_keys=True, default=str).encode()).hexdigest()


def clear_of_point(ob, p, margin):
    if ob["type"] == "box":
        dx = max(abs(p[0] - ob["c"][0]) - ob["h"][0], 0.0)
        dy = max(abs(p[1] - ob["c"][1]) - ob["h"][1], 0.0)
        return math.hypot(dx, dy) > margin
    return math.hypot(p[0] - ob["c"][0], p[1] - ob["c"][1]) - ob["r"] > margin


def clear_of_segment(ob, a, b, margin):
    if ob["type"] == "box":
        return I.clearance((a, b), [I.rect_poly(ob["c"][0], ob["c"][1], ob["h"][0], ob["h"][1])], 0.0) > margin
    dx, dy = b[0] - a[0], b[1] - a[1]
    den = dx * dx + dy * dy
    t = 0.0 if den == 0 else max(0.0, min(1.0, ((ob["c"][0] - a[0]) * dx + (ob["c"][1] - a[1]) * dy) / den))
    return math.hypot(ob["c"][0] - a[0] - t * dx, ob["c"][1] - a[1] - t * dy) - ob["r"] > margin


def ray_path(p, ang, length):
    return ((p[0], p[1]), (p[0] + length * math.cos(ang), p[1] + length * math.sin(ang)))


def bump(counter, key, n=1):
    counter[key] = counter.get(key, 0) + n


# ---------------------------------------------------------------------------------------------------- Part A
def scene_a(seed):
    rng = random.Random(A_BASE + seed)
    layout = LAYOUTS[seed % 2]
    z = rng.uniform(0.05, 2.95)
    pose = (rng.uniform(-2, 2), rng.uniform(-2, 2), z, rng.uniform(-math.pi, math.pi))
    objs = []
    for _ in range(rng.randint(0, 6)):
        hx, hy = rng.choice(CATALOG)
        ob = bar(pose[0] + rng.uniform(-5, 5), pose[1] + rng.uniform(-5, 5), hx, hy)
        if clear_of_point(ob, pose, 0.35):
            objs.append(ob)
    a, d = rng.uniform(-math.pi, math.pi), rng.uniform(1.0, 6.0)
    tz = min(2.75, max(0.25, z + rng.uniform(-0.3, 0.3)))
    tc = (pose[0] + d * math.cos(a), pose[1] + d * math.sin(a), tz)
    mode = rng.choice(["none", "same_bearing_front", "adjacent", "random"])
    extra = None
    if mode == "same_bearing_front":
        dd = d - rng.uniform(0.25, 0.9)
        extra = pole(pose[0] + dd * math.cos(a), pose[1] + dd * math.sin(a), rng.uniform(0.01, 0.06))
    elif mode == "adjacent":
        b = rng.uniform(-math.pi, math.pi)
        g = 0.2 + rng.uniform(0.05, 0.6)
        if rng.random() < 0.5:
            extra = pole(tc[0] + g * math.cos(b), tc[1] + g * math.sin(b), rng.uniform(0.01, 0.06))
        else:
            hx, hy = rng.choice(CATALOG)
            extra = bar(tc[0] + (g + hx) * math.cos(b), tc[1] + (g + hy) * math.sin(b), hx, hy)
    elif mode == "random":
        extra = noncat_box(pose[0] + rng.uniform(-4, 4), pose[1] + rng.uniform(-4, 4), rng.uniform(0.03, 0.15),
                           rng.uniform(0.03, 0.15))
    if extra is not None and clear_of_point(extra, pose, 0.35):
        objs.append(extra)
    vel = (rng.uniform(-1.5, 1.5), rng.uniform(-1.5, 1.5))
    paths = [ray_path(pose, rng.uniform(-math.pi, math.pi), rng.uniform(0.3, 3.0)) for _ in range(K_A - 2)]
    for short in (1.0, 0.3):
        paths.append(ray_path(pose, a, max(0.05, d - short)))
    return rng, layout, pose, objs, tc, vel, paths, mode


def camera_variant(variant, rng, t, pose, tc, objs):
    ext = (0.10, 0.0, 0.03, rng.uniform(-0.5, 0.5), rng.uniform(-0.5, 0.5), 0.0)
    sr, sb = rng.uniform(0.02, 0.7), rng.uniform(0.0005, 0.02)
    if variant == "truthful_noisy":
        return camera(t, pose, tc, ext=ext, sigma_r=sr, sigma_b=sb, range_err=rng.gauss(0, sr),
                      bearing_err=rng.gauss(0, sb))
    if variant == "biased_minus3sigma":
        return camera(t, pose, tc, ext=ext, sigma_r=sr, sigma_b=sb, range_err=-3 * sr)
    if variant == "wrong_extrinsic":
        c = camera(t, pose, tc, ext=ext, sigma_r=sr, sigma_b=sb)
        c[2:8] += np.array([0.3, -0.2, 0.1, 0.2, -0.2, 0.3])
        return c
    if variant == "false_positive_on_bar":
        bars = [o for o in objs if o["kind"] == "bar"] or objs
        p = (bars[0]["c"][0], bars[0]["c"][1], pose[2]) if bars else tc
        return camera(t, pose, p, ext=ext, sigma_r=0.01, sigma_b=0.001)
    if variant == "random_finite":
        c = np.array([rng.uniform(-50, 50) for _ in range(14)])
        c[0], c[1] = t, rng.choice([0.0, 1.0])
        return c
    if variant == "nonfinite":
        c = camera(t, pose, tc, ext=ext)
        c[rng.randrange(14)] = rng.choice([np.nan, np.inf, -np.inf])
        return c
    return camera(t, pose, tc, ext=ext).astype(np.float32)          # malformed_array


def run_fixed_pose(layout, pose, packets, paths, cams=None, tracks=None):
    c = core.CertifierCore(CFG[layout], 0, 0)
    reps, certs = [], []
    for k, (r, s) in enumerate(packets, start=1):
        reps.append(c.step(r, s, *header(k, pose), camera=None if cams is None else cams[k - 1],
                           track=None if tracks is None else tracks[k - 1]))
        certs.append(c.certify(paths, k))
    return reps, certs, c


def worker_a(seeds):
    st = {"scenes": 0, "segments": 0, "camera_diff": {}, "trackcam_diff": 0, "track_gain": 0, "track_loss": 0,
          "classes_by_layout": {}, "track_classes_by_layout": {}, "false_cert": 0, "exceptions": 0,
          "valid_by_layout": {}, "truthful_labelled_returns": 0, "mode_counts": {}, "examples": []}
    digests = {}
    for seed in seeds:
        rng, layout, pose, objs, tc0, vel, paths, mode = scene_a(seed)
        bump(st["mode_counts"], mode)
        P = np.array(paths, dtype=np.float64)
        scenes, packets, tcs = [], [], []
        for k in range(1, STEPS_A + 1):
            tc = (tc0[0] + vel[0] * DT * k, tc0[1] + vel[1] * DT * k, tc0[2])
            scene = objs + [target(*tc)]
            r, s, _ = scan(scene, pose, layout)
            scenes.append(scene)
            packets.append((r, s))
            tcs.append(tc)
        tracks = [np.array([1.0, k * DT, tcs[k - 1][0], tcs[k - 1][1], 0.2]) for k in range(1, STEPS_A + 1)]
        _, base, cb = run_fixed_pose(layout, pose, packets, P)
        record = {"base": base}
        st["exceptions"] += len(cb.exceptions)
        for v in VARIANTS:
            cams = [camera_variant(v, rng, k * DT, pose, tcs[k - 1], objs) for k in range(1, STEPS_A + 1)]
            reps, out, cc = run_fixed_pose(layout, pose, packets, P, cams=cams)
            st["exceptions"] += len(cc.exceptions)
            record[v] = out
            if v == "truthful_noisy":
                st["truthful_labelled_returns"] += sum(int(x.get("labelled_returns", 0)) for x in reps)
            if json.dumps(out, sort_keys=True) != json.dumps(base, sort_keys=True):
                bump(st["camera_diff"], v)
                if len(st["examples"]) < 10:
                    st["examples"].append({"seed": seed, "kind": "camera_diff", "variant": v})
        _, tr, ct = run_fixed_pose(layout, pose, packets, P, tracks=tracks)
        cams = [camera_variant("truthful_noisy", rng, k * DT, pose, tcs[k - 1], objs) for k in range(1, STEPS_A + 1)]
        _, trc, ctc = run_fixed_pose(layout, pose, packets, P, cams=cams, tracks=tracks)
        st["exceptions"] += len(ct.exceptions) + len(ctc.exceptions)
        record["track"], record["track_camera"] = tr, trc
        if json.dumps(trc, sort_keys=True) != json.dumps(tr, sort_keys=True):
            st["trackcam_diff"] += 1
        for k in range(STEPS_A):
            for p, x0, x1 in zip(paths, base[k], tr[k]):
                st["segments"] += 1
                g = (not x0["static_bar_certified"]) and x1["static_bar_certified"]
                st["track_gain"] += g
                st["track_loss"] += x0["static_bar_certified"] and not x1["static_bar_certified"]
                c0 = classify(x0, p, scenes[k], z_vehicle=pose[2])
                c1 = classify(x1, p, scenes[k], z_vehicle=pose[2])
                bump(st["classes_by_layout"], layout + ":" + c0)
                bump(st["track_classes_by_layout"], layout + ":" + c1)
                if c0 == "VALID":
                    bump(st["valid_by_layout"], layout)
                for cl, run in ((c0, "base"), (c1, "track")):
                    if cl == "FALSE_CERT":
                        st["false_cert"] += 1
                        if len(st["examples"]) < 10:
                            st["examples"].append({"seed": seed, "kind": "FALSE_CERT", "run": run, "path": p,
                                                   "z": pose[2], "layout": layout, "reason": x0["reason"]})
                if g and len(st["examples"]) < 10:
                    st["examples"].append({"seed": seed, "kind": "track_gain", "path": p})
        digests[seed] = digest(record)
        st["scenes"] += 1
    return st, digests


def checks_a(st, n):
    return {
        "all_scenes_completed": st["scenes"] == n,
        "camera_outputs_identical": sum(st["camera_diff"].values()) == 0,
        "track_plus_camera_equals_track": st["trackcam_diff"] == 0,
        "track_never_adds_certificate": st["track_gain"] == 0,
        "zero_false_certification": st["false_cert"] == 0,
        "no_internal_exceptions": st["exceptions"] == 0,
        "positive_control_valid_certificates_each_layout": all(st["valid_by_layout"].get(L, 0) > 0 for L in LAYOUTS),
        "positive_control_truthful_camera_labels_active": st["truthful_labelled_returns"] > 0,
    }


# ---------------------------------------------------------------------------------------------------- Part B
def sequence_b(seed):
    rng = random.Random(B_BASE + seed)
    layout = LAYOUTS[seed % 2]
    z = rng.uniform(0.3, 2.7)
    x0, y0, yaw = rng.uniform(-1, 1), rng.uniform(-1, 1), rng.uniform(-math.pi, math.pi)
    hover = rng.random() < 0.5
    if hover:
        v = (0.0, 0.0)
    else:
        sp, hd = rng.uniform(0.05, 0.2), rng.uniform(-math.pi, math.pi)
        v = (sp * math.cos(hd), sp * math.sin(hd))
    poses = [(x0 + v[0] * DT * k, y0 + v[1] * DT * k, z, yaw) for k in range(1, STEPS_B + 1)]
    ta, tb = poses[0][:2], poses[-1][:2]
    bars = []
    for _ in range(rng.randint(0, 5)):
        hx, hy = rng.choice(CATALOG)
        r_, b_ = rng.uniform(1.5, 6.0), rng.uniform(-math.pi, math.pi)
        ob = bar(x0 + r_ * math.cos(b_), y0 + r_ * math.sin(b_), hx, hy)
        if clear_of_segment(ob, ta, tb, 0.35):
            bars.append(ob)
    pole_a = pole_b = None
    s_move = None
    if rng.random() < 0.7:
        pr = rng.uniform(0.03, 0.08)
        b_ = rng.uniform(-math.pi, math.pi)
        r_ = rng.uniform(1.0, 3.0)
        cand_a = pole(x0 + r_ * math.cos(b_), y0 + r_ * math.sin(b_), pr)
        b2 = b_ + rng.choice([-1.0, 1.0]) * rng.uniform(1.0, 2.0)
        r2 = rng.uniform(1.0, 3.0)
        cand_b = pole(x0 + r2 * math.cos(b2), y0 + r2 * math.sin(b2), pr)
        s_move = rng.randint(22, 25)
        ok = all(clear_of_segment(c, ta, tb, 0.35) for c in (cand_a, cand_b)) and all(
            clear_of_point(bo, c["c"], c["r"] + 0.05) for bo in bars for c in (cand_a, cand_b))
        if ok:
            pole_a, pole_b = cand_a, cand_b
    ta_, td = rng.uniform(-math.pi, math.pi), rng.uniform(2.5, 6.0)
    tz = min(2.75, max(0.25, z + rng.uniform(-0.3, 0.3)))
    t0 = (x0 + td * math.cos(ta_), y0 + td * math.sin(ta_), tz)
    tv = (rng.uniform(-1.0, 1.0), rng.uniform(-1.0, 1.0))
    rand_dirs = [(rng.uniform(-math.pi, math.pi), rng.uniform(0.3, 3.0)) for _ in range(3)]
    cam_noise = [(rng.uniform(0.02, 0.3), rng.uniform(0.0005, 0.01)) for _ in range(STEPS_B)]
    cam_err = [(rng.gauss(0, 1), rng.gauss(0, 1)) for _ in range(STEPS_B)]
    return dict(layout=layout, z=z, poses=poses, bars=bars, pole_a=pole_a, pole_b=pole_b, s_move=s_move, t0=t0,
                tv=tv, rand_dirs=rand_dirs, cam_noise=cam_noise, cam_err=cam_err, hover=hover)


def paths_b(sq, k):
    p = sq["poses"][k - 1]
    tc = target_at(sq, k)
    out = []
    if sq["pole_a"] is not None:
        pc = sq["pole_a"]["c"]
        d = math.hypot(pc[0] - p[0], pc[1] - p[1])
        out.append(ray_path(p, math.atan2(pc[1] - p[1], pc[0] - p[0]), d + 0.5))
    else:
        out.append(ray_path(p, *sq["rand_dirs"][2]))
    dt_ = math.hypot(tc[0] - p[0], tc[1] - p[1])
    out.append(ray_path(p, math.atan2(tc[1] - p[1], tc[0] - p[0]), max(0.05, dt_ - 1.0)))
    out += [ray_path(p, *sq["rand_dirs"][i]) for i in range(2)]
    return out


def target_at(sq, k):
    return (sq["t0"][0] + sq["tv"][0] * DT * k, sq["t0"][1] + sq["tv"][1] * DT * k, sq["t0"][2])


def scene_b(sq, k):
    objs = list(sq["bars"])
    if sq["pole_a"] is not None:
        objs.append(sq["pole_a"] if k <= sq["s_move"] else sq["pole_b"])
    return objs + [target(*target_at(sq, k))]


def worker_b(seeds):
    runs = ("base", "camera", "track", "track_camera")
    st = {"sequences": 0, "sequences_by_layout": {}, "segments": 0, "track_gain": 0, "track_loss": 0,
          "track_loss_reasons": {}, "camera_diff": 0, "trackcam_diff": 0, "false_cert": {r: 0 for r in runs},
          "classes_base_by_layout": {}, "classes_track_by_layout": {}, "valid_base_by_layout": {}, "exceptions": 0,
          "sequences_with_pole": 0, "sequences_with_static": 0, "sequences_with_conflict": 0,
          "sequences_with_track_loss": 0, "baseline_static_cells_total": 0, "track_static_cells_total": 0,
          "labelled_returns_camera_run": 0, "hover_sequences": 0, "examples": []}
    digests = {}
    for seed in seeds:
        sq = sequence_b(seed)
        layout, z = sq["layout"], sq["z"]
        bump(st["sequences_by_layout"], layout)
        st["sequences_with_pole"] += sq["pole_a"] is not None
        st["hover_sequences"] += sq["hover"]
        cores = {r: core.CertifierCore(CFG[layout], 0, 0) for r in runs}
        record = {r: [] for r in runs}
        lost_here = False
        for k in range(1, STEPS_B + 1):
            pose = sq["poses"][k - 1]
            scene = scene_b(sq, k)
            rr, ss, _ = scan(scene, pose, layout)
            tc = target_at(sq, k)
            sr, sb = sq["cam_noise"][k - 1]
            er, eb = sq["cam_err"][k - 1]
            cam = camera(k * DT, pose, tc, sigma_r=sr, sigma_b=sb, range_err=er * sr, bearing_err=eb * sb)
            trk = np.array([1.0, k * DT, tc[0], tc[1], 0.2])
            reps = {}
            for r in runs:
                reps[r] = cores[r].step(rr, ss, *header(k, pose), camera=cam if "camera" in r else None,
                                        track=trk if r.startswith("track") else None)
            st["labelled_returns_camera_run"] += int(reps["camera"].get("labelled_returns", 0))
            if k not in CERT_STEPS_B:
                continue
            paths = paths_b(sq, k)
            P = np.array(paths, dtype=np.float64)
            res = {r: cores[r].certify(P, k) for r in runs}
            for r in runs:
                record[r].append(res[r])
            st["camera_diff"] += json.dumps(res["camera"], sort_keys=True) != json.dumps(res["base"], sort_keys=True)
            st["trackcam_diff"] += (json.dumps(res["track_camera"], sort_keys=True)
                                    != json.dumps(res["track"], sort_keys=True))
            for i, p in enumerate(paths):
                st["segments"] += 1
                b0, b1 = res["base"][i]["static_bar_certified"], res["track"][i]["static_bar_certified"]
                if not b0 and b1:
                    st["track_gain"] += 1
                    if len(st["examples"]) < 10:
                        st["examples"].append({"seed": seed, "kind": "track_gain", "step": k, "path": i})
                if b0 and not b1:
                    st["track_loss"] += 1
                    lost_here = True
                    bump(st["track_loss_reasons"], res["track"][i]["reason"])
                for r in runs:
                    cl = classify(res[r][i], p, scene, z_vehicle=z)
                    if r == "base":
                        bump(st["classes_base_by_layout"], layout + ":" + cl)
                        if cl == "VALID":
                            bump(st["valid_base_by_layout"], layout)
                    elif r == "track":
                        bump(st["classes_track_by_layout"], layout + ":" + cl)
                    if cl == "FALSE_CERT":
                        st["false_cert"][r] += 1
                        if len(st["examples"]) < 10:
                            st["examples"].append({"seed": seed, "kind": "FALSE_CERT", "run": r, "step": k,
                                                   "path": p, "z": z, "layout": layout})
        bstat = int(np.count_nonzero(cores["base"].map.static))
        tstat = int(np.count_nonzero(cores["track"].map.static))
        tconf = int(np.count_nonzero(cores["track"].map.conflict))
        st["baseline_static_cells_total"] += bstat
        st["track_static_cells_total"] += tstat
        st["sequences_with_static"] += tstat > 0
        st["sequences_with_conflict"] += tconf > 0
        st["sequences_with_track_loss"] += lost_here
        st["exceptions"] += sum(len(c.exceptions) for c in cores.values())
        digests[seed] = digest(record)
        st["sequences"] += 1
    return st, digests


def checks_b(st, n):
    nv = NON_VACUITY
    return {
        "all_sequences_completed": st["sequences"] == n,
        "track_never_adds_certificate": st["track_gain"] == 0,
        "camera_outputs_identical": st["camera_diff"] == 0 and st["trackcam_diff"] == 0,
        "zero_false_certification_all_runs": sum(st["false_cert"].values()) == 0,
        "no_internal_exceptions": st["exceptions"] == 0,
        "static_rule_needs_track": st["baseline_static_cells_total"] == 0,
        "non_vacuous_static": st["sequences_with_static"] >= nv["sequences_with_static_min_frac"] * n,
        "non_vacuous_conflict": st["sequences_with_conflict"] >= nv["sequences_with_conflict_min_frac"] * n,
        "non_vacuous_track_loss": st["sequences_with_track_loss"] >= nv["sequences_with_track_loss_min_frac"] * n,
        "positive_control_valid_base_certificates_each_layout": all(
            st["valid_base_by_layout"].get(L, 0) > 0 for L in LAYOUTS),
        "positive_control_camera_labels_active": st["labelled_returns_camera_run"] > 0,
    }


# ---------------------------------------------------------------------------------------------------- driver
def merge(parts):
    out = {}
    for p in parts:
        for k, v in p.items():
            if isinstance(v, bool) or isinstance(v, int):
                out[k] = out.get(k, 0) + int(v)
            elif isinstance(v, dict):
                d = out.setdefault(k, {})
                for kk, vv in v.items():
                    d[kk] = d.get(kk, 0) + vv
            else:
                out.setdefault(k, []).extend(v)
    out["examples"] = out.get("examples", [])[:20]
    return out


def main():
    part, out_path, n, workers = sys.argv[1], pathlib.Path(sys.argv[2]), int(sys.argv[3]), int(sys.argv[4])
    seed0 = int(sys.argv[5]) if len(sys.argv) > 5 else 0
    if part not in ("A", "B") or n < 1 or not 1 <= workers <= 16:
        raise SystemExit("usage: noninterference_v2.py {A,B} OUT_JSON N WORKERS [SEED0]")
    if out_path.exists():
        raise SystemExit("refusing to overwrite " + str(out_path))
    t0 = time.time()
    seeds = list(range(seed0, seed0 + n))
    chunks = [seeds[i::workers * 8] for i in range(workers * 8)]
    chunks = [c for c in chunks if c]
    fn = worker_a if part == "A" else worker_b
    parts, digests, failure = [], {}, None
    try:
        with cf.ProcessPoolExecutor(max_workers=workers, mp_context=mp.get_context("fork")) as ex:
            for st, dg in ex.map(fn, chunks):
                parts.append(st)
                digests.update(dg)
    except Exception as e:                                  # BrokenProcessPool on a dead worker, or any error
        failure = type(e).__name__ + ": " + str(e)[:300]
    st = merge(parts) if parts else {}
    if failure is None:
        checks = checks_a(st, n) if part == "A" else checks_b(st, n)
    else:
        checks = {"suite_completed_without_worker_failure": False}
    result = {"part": part, "rule": "R2' rule 5" + part, "pass": bool(checks) and all(checks.values()),
              "checks": checks, "failure": failure, "stats": st, "n": n, "seed0": seed0, "workers": workers,
              "steps": STEPS_A if part == "A" else STEPS_B,
              "certify_steps": None if part == "A" else list(CERT_STEPS_B),
              "non_vacuity": NON_VACUITY if part == "B" else None,
              "digests": {str(s): digests[s] for s in sorted(digests)},
              "digest_of_digests": digest([digests[s] for s in sorted(digests)]),
              "wall_s": time.time() - t0}
    with out_path.open("x") as fh:
        json.dump(result, fh, indent=1, default=str)
        fh.write("\n")
    print(json.dumps({"pass": result["pass"], "checks": checks, "failure": failure, "n": n,
                      "wall_s": result["wall_s"]}, indent=1))
    return 0 if result["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
