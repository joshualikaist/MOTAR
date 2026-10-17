#!/usr/bin/env python3
"""EQ-1B flight envelope — speed, tilt and stopping margins (recorded OCC-P2 states) and the riskcap law (CPU).

    python -B tools/audits/eq1_flight_envelope.py [--out docs/evidence/.../eq1b_envelope.json] [--workers 12]

Validation only; raw OCC-P2 results are read, never written.

B2 speed and tilt. Frame k of the occlusion trace is the state after k completed actions (10 Hz),
recorded when its observation is consumed; the terminating step is not recorded, and the episode
resets after a crash. Classes per quota episode, declared before execution:
  normal          every frame of capture/timeout episodes, and crash-episode frames > 1 s before the end;
  crash_final_1s  the last 10 recorded frames of a crash episode (the approach to the contact);
  pre_crash_last  the last recorded frame of a crash episode (<= 0.1 s before termination);
  during/after contact: NOT_RECORDED (no inner-step or post-reset state exists).
Tilt is the angle between the body z axis and world z from p_quat (x, y, z, w).

B2b recorded stopping margin. From each frame's position and horizontal velocity, the free travel
distance D along the velocity to the first bar is an exact ray cast against each recorded box
inflated by the conservative body radius (Minkowski box + disc). Margin = D - (v t_r + v^2 / 2a):
a = 2.9609 m/s^2 (riskcap's declared brake) and a = g tan 45 deg (an optimistic physical bound at the
tilt limit, motor and attitude lag ignored). Negative means straight-line braking from that state
cannot stop before contact. Nearest-surface distance below the optimistic body radius in a frame that
did not end the episode would contradict the contact model (consistency check).

B5 riskcap law. Production `apply_speed_governor` with the receipt's governor settings over a
clearance x requested-speed grid, against the independent stopping inequality
clearance - hard_margin >= v t_r + v^2 / 2a. Gate: nonnegative executed stopping margin everywhere.
"""

from __future__ import annotations

import argparse
from concurrent.futures import ProcessPoolExecutor
import json
import math
from pathlib import Path
import sys

import numpy as np
import torch

sys.path.insert(0, str(Path(__file__).resolve().parent))
import eq1_common as common  # noqa: E402
from eq1_scene_feasibility import R_CONS, R_OPT, cell_dirs  # noqa: E402

G = 9.80665
BRAKE = float(common.GOVERNOR["brake_mps2"])
BRAKE_PHYS = G * math.tan(math.radians(45.0))
REACTION = float(common.GOVERNOR["reaction_s"])
FINAL_FRAMES = 10
TILT_LIMIT = 45.0
FREE_CAP = float(common.GOVERNOR["free_speed_cap_mps"])
BINS = {"speed_xy": np.arange(0, 8.0001, 0.01), "speed_z": np.arange(0, 6.0001, 0.01),
        "tilt_deg": np.arange(0, 180.0001, 0.1), "margin_gov_m": np.arange(-6, 20.0001, 0.01),
        "margin_phys_m": np.arange(-6, 20.0001, 0.01), "free_travel_m": np.arange(0, 60.0001, 0.01),
        "nearest_surface_m": np.arange(0, 30.0001, 0.01)}
CLASSES = ("normal", "crash_final_1s", "pre_crash_last")
RECEIPT_GOVERNOR_ENV = {
    "NAVRL_SPEED_GOVERNOR": "riskcap", "NAVRL_SPEED_GOVERNOR_BRAKE_MPS2": "2.9608856678",
    "NAVRL_SPEED_GOVERNOR_FIXED_MPS": "2.0", "NAVRL_SPEED_GOVERNOR_FREE_MPS": "3.53553390593",
    "NAVRL_SPEED_GOVERNOR_HALF_WIDTH_M": "0.45", "NAVRL_SPEED_GOVERNOR_MARGIN_M": "0.45",
    "NAVRL_SPEED_GOVERNOR_REACTION_S": "0.1", "NAVRL_SPEED_GOVERNOR_RELEASE_M": "5.0",
    "NAVRL_SPEED_GOVERNOR_SLOW_M": "3.0", "NAVRL_SPEED_GOVERNOR_TTC_S": "1.2"}


# ------------------------------------------------------------------ independent geometry
def tilt_deg(q):
    return np.degrees(np.arccos(np.clip(1.0 - 2.0 * (q[:, 0] ** 2 + q[:, 1] ** 2), -1.0, 1.0)))


def _ray_aabb(px, py, dx, dy, cx, cy, hx, hy):
    """Entry distance of rays [N,1] into boxes [1,B]; 0 when the origin is inside; inf on a miss."""
    with np.errstate(divide="ignore", invalid="ignore"):
        inv_x = np.where(np.abs(dx) > 1e-12, 1.0 / dx, np.inf)
        inv_y = np.where(np.abs(dy) > 1e-12, 1.0 / dy, np.inf)
        tx1, tx2 = (cx - hx - px) * inv_x, (cx + hx - px) * inv_x
        ty1, ty2 = (cy - hy - py) * inv_y, (cy + hy - py) * inv_y
    slab_x = np.abs(dx) > 1e-12
    slab_y = np.abs(dy) > 1e-12
    inx = np.abs(px - cx) <= hx
    iny = np.abs(py - cy) <= hy
    lo_x = np.where(slab_x, np.minimum(tx1, tx2), np.where(inx, -np.inf, np.inf))
    hi_x = np.where(slab_x, np.maximum(tx1, tx2), np.where(inx, np.inf, -np.inf))
    lo_y = np.where(slab_y, np.minimum(ty1, ty2), np.where(iny, -np.inf, np.inf))
    hi_y = np.where(slab_y, np.maximum(ty1, ty2), np.where(iny, np.inf, -np.inf))
    t0, t1 = np.maximum(lo_x, lo_y), np.minimum(hi_x, hi_y)
    hit = (t1 >= np.maximum(t0, 0.0))
    return np.where(hit, np.maximum(t0, 0.0), np.inf)


def _ray_circle(px, py, dx, dy, cx, cy, r):
    ox, oy = px - cx, py - cy
    b = ox * dx + oy * dy
    c = ox * ox + oy * oy - r * r
    disc = b * b - c
    t = -b - np.sqrt(np.maximum(disc, 0.0))
    return np.where(c <= 0, 0.0, np.where((disc >= 0) & (t >= 0), t, np.inf))


def free_travel(pos, vel, centres, halves, r, chunk=1024):
    """Distance along the horizontal velocity to the first box inflated by a disc of radius r (exact)."""
    n = len(pos)
    out = np.full(n, np.inf)
    speed = np.linalg.norm(vel, axis=1)
    d = vel / np.maximum(speed, 1e-12)[:, None]
    cx, cy = centres[None, :, 0], centres[None, :, 1]
    hx, hy = halves[None, :, 0], halves[None, :, 1]
    for a in range(0, n, chunk):
        b = min(n, a + chunk)
        px, py = pos[a:b, 0:1], pos[a:b, 1:2]
        dx, dy = d[a:b, 0:1], d[a:b, 1:2]
        t = np.minimum(_ray_aabb(px, py, dx, dy, cx, cy, hx + r, hy), _ray_aabb(px, py, dx, dy, cx, cy, hx, hy + r))
        for sx in (-1, 1):
            for sy in (-1, 1):
                t = np.minimum(t, _ray_circle(px, py, dx, dy, cx + sx * hx, cy + sy * hy, r))
        out[a:b] = t.min(axis=1)
    out[speed < 1e-6] = np.inf
    return out


def nearest_surface(pos, centres, halves, chunk=2048):
    out = np.empty(len(pos))
    for a in range(0, len(pos), chunk):
        p = pos[a:a + chunk, None, :]
        q = np.maximum(np.abs(p - centres[None]) - halves[None], 0.0)
        out[a:a + chunk] = np.sqrt((q ** 2).sum(axis=2)).min(axis=1)
    return out


def stopping_distance(v, a):
    return v * REACTION + v * v / (2.0 * a)


# ------------------------------------------------------------------ per cell
def cell_envelope(cell_dir):
    from motar_eval import occlusion_trace as ot

    common.assert_in_tree(ot)
    cell_dir = Path(cell_dir)
    trace = ot.read_trace(cell_dir)
    layouts, frames = trace["layouts"], trace["frames"]
    lookup = ot.reset_lookup(layouts)
    eps = ot.episode_frames(frames)
    rows = [json.loads(l) for l in open(cell_dir / "episodes.jsonl")]
    hist = {c: {k: np.zeros(len(v) - 1, dtype=np.int64) for k, v in BINS.items()} for c in CLASSES}
    counts = {c: {"frames": 0, "tilt_over_45": 0, "speed_xy_over_free_cap": 0, "speed_xy_over_2p5": 0,
                  "moving_frames": 0, "margin_gov_negative": 0, "margin_phys_negative": 0,
                  "nearest_below_optimistic_body": 0} for c in CLASSES}
    maxima = {c: {"tilt_deg": [0.0, None], "speed_xy": [0.0, None]} for c in CLASSES}
    for r in rows:
        if not r["in_quota"]:
            continue
        key = (r["env_index"], r["env_episode_index"])
        ef = eps.get(key)
        if ef is None or key not in lookup:
            continue
        ef = ef[np.argsort(ef["frame"])]
        n = len(ef)
        label = np.full(n, "normal", dtype=object)
        if r["outcome"] == "crash":
            label[max(0, n - FINAL_FRAMES):] = "crash_final_1s"
        centres, halves, _ = ot.layout_arrays(layouts, lookup[key])
        c2, h2 = centres[:, :2].astype(np.float64), halves[:, :2].astype(np.float64)
        pos = ef["p_pos"][:, :2].astype(np.float64)
        vel = ef["p_vel"][:, :2].astype(np.float64)
        v = np.linalg.norm(vel, axis=1)
        values = {"speed_xy": v, "speed_z": np.abs(ef["p_vel"][:, 2].astype(np.float64)),
                  "tilt_deg": tilt_deg(ef["p_quat"].astype(np.float64)),
                  "nearest_surface_m": nearest_surface(pos, c2, h2)}
        travel = free_travel(pos, vel, c2, h2, R_CONS)
        values["free_travel_m"] = travel
        values["margin_gov_m"] = travel - stopping_distance(v, BRAKE)
        values["margin_phys_m"] = travel - stopping_distance(v, BRAKE_PHYS)
        groups = {"normal": label == "normal", "crash_final_1s": label == "crash_final_1s",
                  "pre_crash_last": np.zeros(n, dtype=bool)}
        if r["outcome"] == "crash":
            groups["pre_crash_last"][n - 1] = True
        for cname, m in groups.items():
            if not m.any():
                continue
            moving = m & (v > 0.1)
            for k, edges in BINS.items():
                sel = moving if k in ("margin_gov_m", "margin_phys_m", "free_travel_m") else m
                x = np.clip(values[k][sel], edges[0], edges[-1] - 1e-9)
                hist[cname][k] += np.histogram(x, bins=edges)[0]
            c = counts[cname]
            c["frames"] += int(m.sum())
            c["moving_frames"] += int(moving.sum())
            c["tilt_over_45"] += int((values["tilt_deg"][m] > TILT_LIMIT).sum())
            c["speed_xy_over_free_cap"] += int((v[m] > FREE_CAP * 1.01).sum())
            c["speed_xy_over_2p5"] += int((v[m] > 2.5).sum())
            c["margin_gov_negative"] += int((values["margin_gov_m"][moving] < 0).sum())
            c["margin_phys_negative"] += int((values["margin_phys_m"][moving] < 0).sum())
            c["nearest_below_optimistic_body"] += int((values["nearest_surface_m"][m] < R_OPT).sum())
            for k in ("tilt_deg", "speed_xy"):
                i = int(np.argmax(np.where(m, values[k], -np.inf)))
                if values[k][i] > maxima[cname][k][0]:
                    maxima[cname][k] = [float(values[k][i]), {"cell": cell_dir.name, "env": key[0], "episode": key[1],
                                                               "frame": int(ef["frame"][i]), "outcome": r["outcome"]}]
    return {"cell": cell_dir.name, "group": cell_dir.parent.parent.name, "bars": int(rows[0]["bars"]),
            "hist": {c: {k: v.tolist() for k, v in h.items()} for c, h in hist.items()},
            "counts": counts, "maxima": maxima}


def quantiles(h, edges, qs=(0.01, 0.05, 0.5, 0.95, 0.99, 0.999)):
    h = np.asarray(h)
    total = h.sum()
    if total == 0:
        return {}
    cdf = np.cumsum(h) / total
    return {"q%g" % (q * 100): float(edges[int(np.searchsorted(cdf, q)) + 1]) for q in qs}


def aggregate(cells):
    out = {}
    for c in cells:
        for key in ("%s/%d" % (c["group"], c["bars"]), "%s/all" % c["group"], "all"):
            a = out.setdefault(key, {cl: {"hist": {k: np.zeros(len(v) - 1, dtype=np.int64) for k, v in BINS.items()},
                                          "counts": {}, "maxima": {}} for cl in CLASSES})
            for cl in CLASSES:
                for k in BINS:
                    a[cl]["hist"][k] += np.asarray(c["hist"][cl][k])
                for k, v in c["counts"][cl].items():
                    a[cl]["counts"][k] = a[cl]["counts"].get(k, 0) + v
                for k, (val, where) in c["maxima"][cl].items():
                    if val > a[cl]["maxima"].get(k, [0.0, None])[0]:
                        a[cl]["maxima"][k] = [val, where]
    report = {}
    for key, a in out.items():
        report[key] = {}
        for cl in CLASSES:
            cnt = a[cl]["counts"]
            report[key][cl] = {"counts": cnt, "maxima": a[cl]["maxima"],
                               "quantiles": {k: quantiles(a[cl]["hist"][k], BINS[k]) for k in BINS},
                               "fractions": {k: (cnt[k] / cnt["moving_frames"] if k.startswith("margin") else cnt[k] / cnt["frames"])
                                             if cnt.get("frames") else None
                                             for k in ("tilt_over_45", "speed_xy_over_free_cap", "speed_xy_over_2p5",
                                                       "margin_gov_negative", "margin_phys_negative",
                                                       "nearest_below_optimistic_body")}}
    return report


def case_envelope(workers=12, root=common.OCC_P2):
    with ProcessPoolExecutor(max_workers=workers) as pool:
        cells = list(pool.map(cell_envelope, cell_dirs(root)))
    report = aggregate(cells)
    normal = report["all"]["normal"]["counts"]
    return {"name": "B2 speed / tilt / recorded stopping margin (OCC-P2 quota episodes)", "root": str(root),
            "cells": len(cells), "classes": list(CLASSES) + ["during_contact: NOT_RECORDED", "after_contact: NOT_RECORDED"],
            "body_radius_for_travel_m": R_CONS, "brake_mps2": {"riskcap_declared": BRAKE, "physical_45deg_bound": BRAKE_PHYS},
            "reaction_s": REACTION, "histogram_bin_note": "quantiles are bin upper edges (0.01 m, 0.01 m/s, 0.1 deg)",
            "by_group_density": report,
            "tilt_limit_in_normal_flight": common.verdict(normal["tilt_over_45"] == 0,
                                                          "%d of %d normal frames above %.0f deg" % (
                                                              normal["tilt_over_45"], normal["frames"], TILT_LIMIT)),
            "contact_model_consistency": common.verdict(
                report["all"]["normal"]["counts"]["nearest_below_optimistic_body"] == 0,
                "no surviving frame closer to a bar than the optimistic body radius")}


# ------------------------------------------------------------------ B5 governor
def safe_speed(clearance, margin, a=BRAKE, tr=REACTION):
    usable = max(clearance - margin, 0.0)
    return math.sqrt((a * tr) ** 2 + 2 * a * usable) - a * tr


def case_governor():
    with common.production():
        return _case_governor(common.load_production("aerial_gym.task.navrl_task.speed_governor"))


def _case_governor(sg):
    rows, violations = [], []
    cfg = sg.SpeedGovernorConfig.from_environ(RECEIPT_GOVERNOR_ENV)
    stop_env = dict(RECEIPT_GOVERNOR_ENV, NAVRL_SPEED_GOVERNOR="stopcap")
    stop_cfg = sg.SpeedGovernorConfig.from_environ(stop_env)
    clear = torch.arange(0.0, 12.0001, 0.05, dtype=torch.float64)
    worst = {}
    for req in (1.0, 2.0, 2.5, FREE_CAP):
        cmd = torch.stack([torch.full_like(clear, req), torch.zeros_like(clear)], dim=1)
        for name, c in (("riskcap", cfg), ("stopcap", stop_cfg)):
            _, info = sg.apply_speed_governor(cmd, clear, c)
            ex = info["executed_speed_mps"].numpy()
            margin = info["stopping_margin_executed_m"].numpy()
            indep = np.array([max(x - c.hard_margin_m, 0.0) for x in clear.numpy()]) - stopping_distance(ex, c.brake_mps2)
            fidelity = float(np.abs(indep - margin).max())
            neg = clear.numpy()[margin < -1e-9]
            rows.append({"law": name, "requested_mps": req, "fidelity_max_abs_m": fidelity,
                         "negative_margin_clearance_m": [float(neg.min()), float(neg.max())] if len(neg) else None,
                         "min_margin_m": float(margin.min())})
            if name == "riskcap" and len(neg):
                violations.append(rows[-1])
            w = worst.setdefault(name, [math.inf, None])
            i = int(np.argmin(margin))
            if margin[i] < w[0]:
                worst[name] = [float(margin[i]), {"clearance_m": float(clear[i]), "requested_mps": req,
                                                  "executed_mps": float(ex[i])}]
    _, info = sg.apply_speed_governor(torch.tensor([[2.0, 0.0]], dtype=torch.float64),
                                      torch.tensor([0.1], dtype=torch.float64), cfg)
    required = cfg.hard_margin_m + stopping_distance(2.0, cfg.brake_mps2)
    return {"name": "B5 riskcap braking law (production apply_speed_governor, receipt settings)",
            "config": {"mode": cfg.mode, "fixed_cap_mps": cfg.fixed_cap_mps, "free_speed_cap_mps": cfg.free_speed_cap_mps,
                       "hard_margin_m": cfg.hard_margin_m, "brake_mps2": cfg.brake_mps2, "reaction_s": cfg.reaction_s,
                       "slow_m": cfg.slow_distance_m, "release_m": cfg.release_distance_m},
            "counterexample": {"clearance_m": 0.1, "requested_mps": 2.0,
                               "executed_mps": float(info["executed_speed_mps"][0]),
                               "stopping_margin_executed_m": float(info["stopping_margin_executed_m"][0]),
                               "clearance_needed_at_2mps_m": required},
            "independent_safe_speed_at_0.1m_mps": safe_speed(0.1, cfg.hard_margin_m),
            "grid": rows, "worst": worst,
            "implementation_fidelity": common.verdict(max(r["fidelity_max_abs_m"] for r in rows) < 1e-9,
                                                      "production margin equals the independent inequality"),
            "riskcap_nonnegative_margin": common.verdict(not violations, "2 m/s floor: negative below %.4f m" % required),
            "stopcap_nonnegative_margin": common.verdict(all(r["min_margin_m"] >= -1e-9 for r in rows if r["law"] == "stopcap"),
                                                         "existing production mode; never trained or evaluated with Policy F")}


def run(workers=12):
    return {"schema": "motar.eq1b-envelope.v1", "package": "EQ-1B flight envelope and braking (CPU, validation only)",
            "source": common.source_binding(["aerial_gym/task/navrl_task/speed_governor.py", "motar_eval/occlusion_trace.py"]),
            "cases": [case_governor(), case_envelope(workers)]}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--out")
    parser.add_argument("--workers", type=int, default=12)
    args = parser.parse_args(argv)
    record = run(args.workers)
    if args.out:
        common.write_record(args.out, record)
    g, e = record["cases"]
    print(json.dumps(g["counterexample"]), g["riskcap_nonnegative_margin"], g["stopcap_nonnegative_margin"])
    for cl in CLASSES:
        a = e["by_group_density"]["all"][cl]
        print(cl, a["counts"], json.dumps(a["fractions"]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
