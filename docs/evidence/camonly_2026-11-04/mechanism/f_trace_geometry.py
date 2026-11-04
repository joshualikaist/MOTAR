"""Camera-only study, crash-mechanism geometry on Policy F's own full-LiDAR trajectories (EXPLORATORY, post hoc).
CPU only; numpy on recorded traces; no policy, no rendering.

Input: B1-R A/F_SHADOW occlusion traces (Policy F f7022139 flying with full LiDAR and riskcap; seed 4301; 70/115/160/
205 bars; E2, A3), read with MOTAR's own reader (motar_eval/occlusion_trace.py, checks both SHA-256 values).
Per recorded frame (state after k actions; dt 0.1 s), in the yaw-only vehicle frame (x forward = camera axis):
  - travel bearing  = bearing of the horizontal velocity; "outside" means |bearing| > HFOV/2 = 43.5 deg;
  - nearest bar     = smallest horizontal surface distance to an axis-aligned bar box that spans the vehicle's height,
                      bearing = direction to the box's closest point;
  - threats         = bars with surface distance < 2 m and closing speed > 0 (velocity component toward the closest
                      point), and the subset with time-to-contact (distance / closing speed) < 1 s ("imminent");
                      a "blind-imminent" frame has imminent bars and all of them outside the FOV.
Crash episodes (episodes.jsonl outcome == crash): the bar nearest at the last recorded frame (about 0.1 s before
contact) is taken as the contacted bar; its bearing is followed back 0.5 s and 1.0 s.
usage: f_trace_geometry.py MOTAR_TREE OUT_JSON
"""
import collections
import hashlib
import importlib.util
import json
import math
import pathlib
import sys

import numpy as np

TREE, OUT = pathlib.Path(sys.argv[1]).resolve(), pathlib.Path(sys.argv[2])
B1R = pathlib.Path("/home/fair/workspaces/aerial_gym_ws/src/aerial_gym_simulator/results/b1r_classical_baseline_v1")
CELLS = [B1R / "A" / "F_SHADOW" / ("E2_obstacle_aware__%dbars__seed4301" % b) for b in (70, 115, 160, 205)]
HALF = math.radians(87.0) / 2
MOVING = 0.3          # m/s; bearing of travel only when moving
BINS = list(range(-180, 181, 15))


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, str(path))
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


OT = load("occlusion_trace_mech", TREE / "motar_eval/occlusion_trace.py")


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def yaw_of(q):                       # xyzw
    x, y, z, w = q[..., 0], q[..., 1], q[..., 2], q[..., 3]
    return np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def surface(p, c, h):
    """Horizontal surface distance and closest-point offset from p to every box spanning p's height."""
    span = np.abs(p[2] - c[:, 2]) <= h[:, 2]
    d = p[None, :2] - c[:, :2]
    cl = np.clip(d, -h[:, :2], h[:, :2])          # closest point offset from center
    off = (c[:, :2] + cl) - p[None, :2]            # p -> closest point
    dist = np.hypot(off[:, 0], off[:, 1])
    dist[~span] = np.inf
    return dist, off


def main():
    if OUT.exists():
        raise SystemExit("refusing to overwrite " + str(OUT))
    res = {"analysis": "Policy F full-LiDAR trajectory geometry vs the camera FOV", "status": "EXPLORATORY_POST_HOC",
           "hfov_deg": 87.0, "inputs_sha256": {}, "cells": {}, "pooled": {}}
    pooled = collections.defaultdict(float)
    hist_pool = np.zeros(len(BINS) - 1)
    crash_pool = collections.defaultdict(float)
    for cell in CELLS:
        bars = int(cell.name.split("__")[1].replace("bars", ""))
        for fn in ("occlusion_frames.bin", "occlusion_layouts.npz", "episodes.jsonl"):
            res["inputs_sha256"]["%d/%s" % (bars, fn)] = hashlib.sha256((cell / fn).read_bytes()).hexdigest()
        tr = OT.read_trace(cell)
        by_ep = OT.episode_frames(tr["frames"])
        look = OT.reset_lookup(tr["layouts"])
        outcome = {}
        with open(cell / "episodes.jsonl") as f:
            for line in f:
                e = json.loads(line)
                outcome[(e["env_index"], e["env_episode_index"])] = e["outcome"]
        st = collections.defaultdict(float)
        hist = np.zeros(len(BINS) - 1)
        cr = collections.defaultdict(float)
        cache = {}
        for key, fr in by_ep.items():
            if key not in look:
                st["episodes_without_layout"] += 1
                continue
            li = look[key]
            if li not in cache:
                c, h, _ = OT.layout_arrays(tr["layouts"], li)
                cache = {li: (np.asarray(c, float), np.asarray(h, float))}
            c, h = cache[li]
            st["episodes"] += 1
            yaw = yaw_of(np.asarray(fr["p_vehicle_quat"], float))
            pos = np.asarray(fr["p_pos"], float)
            vel = np.asarray(fr["p_vel"], float)
            near_idx = []
            near_brg = []
            blind_frames = 0
            for i in range(len(fr)):
                p, v = pos[i], vel[i]
                spd = math.hypot(v[0], v[1])
                st["frames"] += 1
                if spd > MOVING:
                    b = wrap(math.atan2(v[1], v[0]) - yaw[i])
                    st["moving"] += 1
                    st["moving_speed_sum"] += spd
                    out = abs(b) > HALF
                    st["moving_out"] += out
                    st["moving_out_speedw"] += spd * out
                    st["moving_abs_brg_sum"] += abs(math.degrees(b))
                    st["moving_left"] += b > 0
                    hist += np.histogram([math.degrees(b)], bins=BINS)[0]
                dist, off = surface(p, c, h)
                j = int(np.argmin(dist))
                dj = float(dist[j])
                bj = float(wrap(math.atan2(off[j, 1], off[j, 0]) - yaw[i])) if np.isfinite(dj) else float("nan")
                near_idx.append(j)
                near_brg.append(bj)
                for R in (1.0, 2.0, 3.0):
                    if dj < R:
                        st["near%.0f" % R] += 1
                        st["near%.0f_out" % R] += abs(bj) > HALF
                # threats: closing within 2 m
                m = dist < 2.0
                if m.any():
                    u = off[m] / np.maximum(dist[m, None], 1e-9)
                    closing = u @ v[:2]
                    brg = wrap(np.arctan2(off[m, 1], off[m, 0]) - yaw[i])
                    thr = closing > 0.0
                    st["threat_bar_frames"] += float(thr.sum())
                    st["threat_bar_frames_out"] += float((thr & (np.abs(brg) > HALF)).sum())
                    ttc1 = thr & (dist[m] / np.maximum(closing, 1e-9) < 1.0)
                    st["ttc1_bar_frames"] += float(ttc1.sum())
                    st["ttc1_bar_frames_out"] += float((ttc1 & (np.abs(brg) > HALF)).sum())
                    st["frames_with_ttc1"] += float(ttc1.any())
                    only_out = bool(ttc1.any() and not (ttc1 & (np.abs(brg) <= HALF)).any())
                    st["frames_with_ttc1_only_out"] += float(only_out)
                    blind_frames += int(only_out)
            st["episodes_with_blind_imminent"] += blind_frames > 0
            st["blind_imminent_frames_per_episode_sum"] += blind_frames
            if outcome.get(key) == "crash" and len(fr) >= 1:
                cr["contact_bar_left"] += near_brg[-1] > 0
                cr["crash_episodes"] += 1
                j = near_idx[-1]
                p_last = pos[-1]
                dist, off = surface(p_last, c, h)
                cr["last_frame_dist_sum"] += float(dist[j])
                b0 = abs(near_brg[-1])
                cr["contact_bar_out_at_last"] += b0 > HALF
                cr["contact_bar_abs_brg_sum"] += math.degrees(b0)
                for lag, name in ((5, "0p5s"), (10, "1p0s")):
                    if len(fr) > lag:
                        k = len(fr) - 1 - lag
                        d2, o2 = surface(pos[k], c, h)
                        bb = abs(float(wrap(math.atan2(o2[j, 1], o2[j, 0]) - yaw[k])))
                        cr["n_" + name] += 1
                        cr["contact_bar_out_" + name] += bb > HALF
                        cr["contact_bar_dist_sum_" + name] += float(d2[j])
                # ever in FOV during the last 1 s
                ks = range(max(0, len(fr) - 11), len(fr))
                ever_in = False
                for k in ks:
                    d2, o2 = surface(pos[k], c, h)
                    bb = abs(float(wrap(math.atan2(o2[j, 1], o2[j, 0]) - yaw[k])))
                    ever_in |= bb <= HALF
                cr["contact_bar_never_in_fov_last1s"] += not ever_in
        def summ(s, hh, cc):
            o = {
                "episodes": s["episodes"], "frames": s["frames"], "moving_frames": s["moving"],
                "travel_out_of_fov_pct": 100 * s["moving_out"] / max(1, s["moving"]),
                "travel_out_of_fov_speed_weighted_pct": 100 * s["moving_out_speedw"] / max(1e-9, s["moving_speed_sum"]),
                "travel_mean_abs_bearing_deg": s["moving_abs_brg_sum"] / max(1, s["moving"]),
                "travel_left_pct": 100 * s["moving_left"] / max(1, s["moving"]),
                "mean_speed_mps": s["moving_speed_sum"] / max(1, s["moving"]),
                "travel_bearing_hist_15deg": {"%d..%d" % (BINS[i], BINS[i + 1]): int(hh[i]) for i in range(len(hh))},
            }
            for R in (1, 2, 3):
                o["nearest_bar_within_%dm_frames_pct" % R] = 100 * s["near%d" % R] / max(1, s["frames"])
                o["nearest_bar_within_%dm_out_of_fov_pct" % R] = 100 * s["near%d_out" % R] / max(1, s["near%d" % R])
            o["closing_bars_within_2m_out_of_fov_pct"] = 100 * s["threat_bar_frames_out"] / max(1, s["threat_bar_frames"])
            o["ttc_lt_1s_bars_out_of_fov_pct"] = 100 * s["ttc1_bar_frames_out"] / max(1, s["ttc1_bar_frames"])
            o["frames_with_ttc_lt_1s_pct"] = 100 * s["frames_with_ttc1"] / max(1, s["frames"])
            o["frames_with_ttc_lt_1s_only_out_of_fov_pct_of_those"] = (
                100 * s["frames_with_ttc1_only_out"] / max(1, s["frames_with_ttc1"]))
            n = cc["crash_episodes"]
            o["episodes_with_ge1_frame_all_ttc_lt_1s_bars_out_of_fov_pct"] = (
                100 * s["episodes_with_blind_imminent"] / max(1, s["episodes"]))
            o["frames_all_ttc_lt_1s_bars_out_of_fov_per_episode_mean"] = (
                s["blind_imminent_frames_per_episode_sum"] / max(1, s["episodes"]))
            o["crash"] = {
                "contact_bar_on_left_pct": 100 * cc["contact_bar_left"] / max(1, cc["crash_episodes"]),
                "episodes": n,
                "contact_bar_out_of_fov_at_last_frame_pct": 100 * cc["contact_bar_out_at_last"] / max(1, n),
                "contact_bar_mean_abs_bearing_deg_at_last_frame": cc["contact_bar_abs_brg_sum"] / max(1, n),
                "last_frame_mean_surface_dist_m": cc["last_frame_dist_sum"] / max(1, n),
                "contact_bar_out_of_fov_0p5s_before_pct": 100 * cc["contact_bar_out_0p5s"] / max(1, cc["n_0p5s"]),
                "contact_bar_out_of_fov_1p0s_before_pct": 100 * cc["contact_bar_out_1p0s"] / max(1, cc["n_1p0s"]),
                "contact_bar_never_in_fov_in_last_1s_pct": 100 * cc["contact_bar_never_in_fov_last1s"] / max(1, n),
            }
            return o
        res["cells"][str(bars)] = summ(st, hist, cr)
        for k, v in st.items():
            pooled[k] += v
        hist_pool += hist
        for k, v in cr.items():
            crash_pool[k] += v
    res["pooled"] = summ(pooled, hist_pool, crash_pool)
    OUT.write_text(json.dumps(res, indent=1, sort_keys=True) + "\n")
    print(OUT, hashlib.sha256(OUT.read_bytes()).hexdigest())


if __name__ == "__main__":
    main()
