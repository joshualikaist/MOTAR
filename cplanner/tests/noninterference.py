"""R2 rule 5: camera non-interference and track monotonicity on 20,000 random scenes (72x1 and 72x4), with the oracle
classification of every certificate. usage: noninterference.py OUT_JSON [N_SCENES] [WORKERS]"""
import json
import math
import multiprocessing as mp
import random
import sys
import time

import numpy as np

from harness import CATALOG, bar, camera, classify, core, dump, header, noncat_box, pole, scan, target

N = int(sys.argv[2]) if len(sys.argv) > 2 else 20000
WORKERS = int(sys.argv[3]) if len(sys.argv) > 3 else 20
STEPS, K = 2, 4
VARIANTS = ("truthful_noisy", "biased_minus3sigma", "wrong_extrinsic", "false_positive_on_bar", "random_finite",
            "nonfinite", "malformed_array")


def _clear_of_sensor(ob, pose, margin=0.35):
    x, y = pose[0], pose[1]
    if ob["type"] == "box":
        dx = max(abs(x - ob["c"][0]) - ob["h"][0], 0.0)
        dy = max(abs(y - ob["c"][1]) - ob["h"][1], 0.0)
        return math.hypot(dx, dy) > margin
    return math.hypot(x - ob["c"][0], y - ob["c"][1]) - ob["r"] > margin


def make_scene(seed):
    rng = random.Random(seed)
    layout = "P0R-72x1" if seed % 2 == 0 else "MOTAR-72x4"
    pose = (rng.uniform(-2, 2), rng.uniform(-2, 2), 1.0, rng.uniform(-math.pi, math.pi))
    objs = []
    for _ in range(rng.randint(0, 6)):
        hx, hy = rng.choice(CATALOG)
        ob = bar(pose[0] + rng.uniform(-5, 5), pose[1] + rng.uniform(-5, 5), hx, hy)
        if _clear_of_sensor(ob, pose):
            objs.append(ob)
    a, d = rng.uniform(-math.pi, math.pi), rng.uniform(1.0, 6.0)
    tc = (pose[0] + d * math.cos(a), pose[1] + d * math.sin(a), rng.uniform(0.8, 1.2))
    tgt = target(*tc)
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
    if extra is not None and _clear_of_sensor(extra, pose):
        objs.append(extra)
    vel = (rng.uniform(-1.5, 1.5), rng.uniform(-1.5, 1.5))
    paths = []
    for _ in range(K - 2):
        ang, L = rng.uniform(-math.pi, math.pi), rng.uniform(0.3, 3.0)
        paths.append(((pose[0], pose[1]), (pose[0] + L * math.cos(ang), pose[1] + L * math.sin(ang))))
    for short in (1.0, 0.3):            # towards the target, ending `short` m before its centre
        L = max(0.05, d - short)
        paths.append(((pose[0], pose[1]), (pose[0] + L * math.cos(a), pose[1] + L * math.sin(a))))
    return rng, layout, pose, objs, tgt, vel, paths, mode


def camera_for(variant, rng, t, pose, tc, objs):
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
        p = (bars[0]["c"][0], bars[0]["c"][1], 1.0) if bars else tc
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


def run_seq(layout, pose, objs, tgt, vel, paths, cams=None, track=False):
    cfg = "SYNTH-72x1" if layout == "P0R-72x1" else "SYNTH-72x4"
    c = core.CertifierCore(cfg, 0, 0)
    out = []
    for k in range(1, STEPS + 1):
        tc = (tgt["c"][0] + vel[0] * 0.1 * k, tgt["c"][1] + vel[1] * 0.1 * k, tgt["c"][2])
        scene = objs + [target(*tc)]
        r, s, _ = scan(scene, pose, layout)
        trk = np.array([1.0, k * 0.1, tc[0], tc[1], 0.2]) if track else None
        rep = c.step(r, s, *header(k, pose), camera=None if cams is None else cams[k - 1], track=trk)
        res = c.certify(np.array(paths, float), k)
        out.append((rep["accepted"], json.dumps(res, sort_keys=True), scene))
    return out, c


def worker(seeds):
    stats = {"scenes": 0, "segments": 0, "camera_diff": 0, "track_gain": 0, "track_loss": 0, "trackcam_diff": 0,
             "classes": {}, "classes_by_layout": {}, "exceptions": 0, "false_cert_examples": [],
             "camera_diff_examples": [], "out_of_scope_examples": [], "mode_counts": {}}
    for seed in seeds:
        rng, layout, pose, objs, tgt, vel, paths, mode = make_scene(seed)
        stats["mode_counts"][mode] = stats["mode_counts"].get(mode, 0) + 1
        base, cb = run_seq(layout, pose, objs, tgt, vel, paths)
        stats["exceptions"] += len(cb.exceptions)
        for v in VARIANTS:
            cams = [camera_for(v, rng, k * 0.1, pose, (tgt["c"][0] + vel[0] * 0.1 * k, tgt["c"][1] + vel[1] * 0.1 * k,
                                                      tgt["c"][2]), objs) for k in range(1, STEPS + 1)]
            o, cc = run_seq(layout, pose, objs, tgt, vel, paths, cams=cams)
            stats["exceptions"] += len(cc.exceptions)
            if [x[1] for x in o] != [x[1] for x in base]:
                stats["camera_diff"] += 1
                if len(stats["camera_diff_examples"]) < 5:
                    stats["camera_diff_examples"].append({"seed": seed, "variant": v})
        tr, _ = run_seq(layout, pose, objs, tgt, vel, paths, track=True)
        cams = [camera_for("truthful_noisy", rng, k * 0.1, pose, tgt["c"], objs) for k in range(1, STEPS + 1)]
        trc, _ = run_seq(layout, pose, objs, tgt, vel, paths, cams=cams, track=True)
        if [x[1] for x in trc] != [x[1] for x in tr]:
            stats["trackcam_diff"] += 1
        for (a0, j0, scene), (a1, j1, _) in zip(base, tr):
            r0, r1 = json.loads(j0), json.loads(j1)
            for p, x0, x1 in zip(paths, r0, r1):
                stats["segments"] += 1
                stats["track_gain"] += (not x0["static_bar_certified"]) and x1["static_bar_certified"]
                stats["track_loss"] += x0["static_bar_certified"] and not x1["static_bar_certified"]
                cl = classify(x0, p, scene)
                stats["classes"][cl] = stats["classes"].get(cl, 0) + 1
                kk = layout + ":" + cl
                stats["classes_by_layout"][kk] = stats["classes_by_layout"].get(kk, 0) + 1
                ex = {"seed": seed, "layout": layout, "path": p, "reason": x0["reason"], "mode": mode}
                if cl == "FALSE_CERT" and len(stats["false_cert_examples"]) < 20:
                    stats["false_cert_examples"].append(ex)
                if cl == "OUT_OF_SCOPE" and len(stats["out_of_scope_examples"]) < 20:
                    stats["out_of_scope_examples"].append(ex)
        stats["scenes"] += 1
    return stats


def merge(parts):
    out = {}
    for p in parts:
        for k, v in p.items():
            if isinstance(v, int):
                out[k] = out.get(k, 0) + v
            elif isinstance(v, dict):
                d = out.setdefault(k, {})
                for kk, vv in v.items():
                    d[kk] = d.get(kk, 0) + vv
            else:
                out.setdefault(k, []).extend(v)
    return out


if __name__ == "__main__":
    t0 = time.time()
    seeds = list(range(N))
    chunks = [seeds[i::WORKERS * 4] for i in range(WORKERS * 4)]
    with mp.get_context("fork").Pool(WORKERS) as pool:
        parts = pool.map(worker, chunks)
    st = merge(parts)
    checks = {"camera_outputs_identical": st["camera_diff"] == 0, "track_never_adds_certificate": st["track_gain"] == 0,
              "track_plus_camera_equals_track": st["trackcam_diff"] == 0,
              "zero_false_certification": st["classes"].get("FALSE_CERT", 0) == 0, "no_internal_exceptions":
              st["exceptions"] == 0}
    dump(sys.argv[1], {"checks": checks, "pass": all(checks.values()), "stats": st, "n_scenes": N, "steps": STEPS,
                       "segments_per_step": K, "variants": VARIANTS, "wall_s": time.time() - t0})
    print(json.dumps({"checks": checks, "classes": st["classes"], "by_layout": st["classes_by_layout"],
                      "camera_diff": st["camera_diff"], "track_gain": st["track_gain"],
                      "track_loss": st["track_loss"], "wall_s": time.time() - t0}, indent=1))
