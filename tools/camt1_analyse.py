"""Analysis of CAM-T1 (docs/plans/prereg_camonly_t1_2026-11-12.md). CPU only; every rule below is fixed before the run.

Stage A (descriptive, no hypothesis): where CAM-K2 v3's crashes are relative to its camera. For every crash episode
(primary scope: the quota rows, outcome crash) of the 24 traced cells (tools/camt1_run.py trace):
  frames      the trace's frames of the episode (motar_eval/occlusion_trace.read_trace, both SHA-256 checked). Frame k
              is the state after k actions; the last recorded frame L is the state from which the terminal (contact)
              action was taken, one control step (dt = the trace's dt_s, 0.1 s) before the contact was registered.
  contact bar the bar with the smallest horizontal surface distance from the vehicle centre at frame L, among bars
              whose vertical extent contains the vehicle's height (as G3, tools/camonly_c2trace_analyse.py); AMBIGUOUS
              when the second-nearest bar's distance is within 0.1 m of it; VALID when that distance is <= 0.6 m;
              the bar nearest to the position extrapolated by one step (p_L + v_L dt) is recorded as an agreement
              check.
  camera      yaw-level (the sim renders it with robot_vehicle_orientation): heading = yaw of p_vehicle_quat; eye =
              centre + R(yaw) x the trace's camera translation (0.10, 0, 0.03) m.
  in FOV      (primary, origin "eye", test "footprint") some part of the bar's rectangular footprint lies inside the
              horizontal FOV |bearing| <= half-FOV (default 33 deg, a parameter), the bar's vertical extent meets the
              vertical FOV |elevation| <= vfov/2 (default 27 deg) at the horizontal distance r of its nearest
              footprint point, and r <= the camera's obstacle range (10 m, camera_obstacle_max_range); an eye inside
              the footprint counts as in view. Geometric FOV only: occlusion by other bars is NOT_MEASURED.
              (Test "nearest_point": the bearing of the nearest footprint point only; G3 used it from the centre.)
  windows     the frames with index >= L - round(T / dt) for T = 0.5, 1 and 2 s (6, 11 and 21 frames when complete).
  categories  (mutually exclusive) IN_VIEW: in FOV at L; LEFT_FOV: not in FOV at L, in FOV at some earlier frame of
              the 1 s window; BLIND: in FOV in no frame of the 1 s window. EARLY flag: contact within 1 s of the
              start, i.e. (L + 1) x dt <= 1.0 s (the 1 s window is then truncated); EARLY crashes stay in their
              category and are also reported separately.
  also        contact bearing (nearest footprint point from the origin, relative to the heading) and side (LEFT if
              > 0); velocity bearing at L and of the mean vehicle-frame velocity over the 1 s window (moving if the
              speed > 0.3 m/s); speed at L; roll and pitch at L (p_quat); time since start (L + 1) x dt; the share of
              the 1 s window with a valid camera detection of the target (CAMERA_VALID) and with the target
              geometrically visible (GEO_VISIBLE); eye distance below the depth model's MinZ 0.4 m at L.
Shares (review r1 F2): the category shares and every interval are computed over VALID crash episodes (the contacted
  bar identified); the non-VALID ones (contact distance > 0.6 m, e.g. out-of-bounds crashes) are counted and split
  separately, and the all-crash shares are kept as a sensitivity.
Statistics: per policy seed (V3s1, V3s2) and pooled. Point estimates are pooled ratios (episode-weighted).
  Primary CI: cluster bootstrap with the cell as cluster (a density x evaluation-seed run of one policy), resampled
  within density (the densities are fixed by design), 10,000 resamples, seed 20,261,112, percentile 95 %. Why the
  cell: episodes of one cell share its process, scene stream and synchronized resets, and a cell is the unit that was
  replicated independently; an episode-level bootstrap (G3's) treats correlated crashes as independent and is too
  narrow. With 3 cells per density per seed the percentile interval is approximate; the env-within-cell cluster
  bootstrap (128 clusters per cell) is reported as a within-cell sensitivity, and the episode bootstrap for
  comparability with G3.
Reading: TRACE_BINDING_FAIL (the trace run is not a finished 24-cell run on one commit, a cell fails the runner's
checks again, a reference differs from CAM-K2-E3's manifest, the identity stage beside it is not IDENTITY_PASS on the
same commit, or the chain's G3 positive control at this script's bytes is not POSITIVE_CONTROL_PASS); MEASUREMENT_INVALID
with its reasons (per policy seed: no crash episode; fewer than 90 % VALID; review r1 F1: trace coverage of the quota
crash rows, actions_match or frames_contiguous below 0.99; or a geometry the analysis does not support, such as rotated
obstacles); otherwise DESCRIBED, carrying the trace run's NONINTERFERENCE_PASS or TRACE_INTERFERES
(the latter: the traces describe these runs only). Sensitivity profiles (reported, not read): "eye_nearest_point"
(primary with the nearest-point test), "g3_geometry_33" (origin centre, nearest point, no vertical or range test,
half-FOV 33) and "all_episodes" (primary, every traced episode). Guidance for a later retraining (not a gate): see guidance().

Stage B (gated): each arm (cap25, cap20, d1) against the native V3 cells of CAM-K2-E3, paired by cell, per policy
seed s (6 blocks: 115 / 205 bars x 3 seeds) and pooled (weights 1/2). Quota rates; median capture time of the quota
capture episodes (kroc2027 actions x rl_dt_s). Intervals as CAM-K2-E3 (tools/camk2v3e_analyse.py): the stratified
percentile bootstrap (seeds resampled within density, 10,000 resamples, seed 20,261,108) and the stratified t
interval (df 4). Gate per arm and seed: CAM-K2-E3's gate_d on the crash contrast (MATERIAL_REDUCTION, SMALL_REDUCTION
with the 2 pp floor, WORSE, NEGLIGIBLE inside +-2 pp, NO_DETECTED_CHANGE, UNRESOLVED, CRASH_TO_TIMEOUT); the arm's
reading is the label both seeds share, else SEEDS_DISAGREE; the pooled label is reported. Controls on CAM-K2-E's
treated cells at the same 6 blocks (each must read as required, else CONTROL_FAIL and no label): K1s - K2s WORSE and
K2s - K1s SMALL_ or MATERIAL_REDUCTION for both seeds; K2s1 - K2s2 neither MATERIAL_REDUCTION nor WORSE. Bindings
(EVAL_BINDING_FAIL otherwise): identity IDENTITY_PASS, control CONTROL_PASS and envelope COMPLETE on one commit with the planned order and
36 cells; each envelope cell passes the runner's checks again; d1s1 and cap20s1 at 205 bars / 4501 equal the control
stage's d1_smoke and cap20_smoke byte for byte (determinism); the native cells at CAM-K2-E3's manifest SHA-256. On
CONTROL_FAIL no gate label is written (review r1 F8); the contrasts' numbers are kept.
usage: camt1_analyse.py stage-a TRACE_DIR OUT_JSON --g3-control JSON [--e3-raw DIR] [--half-fov-deg H] [--vfov-deg V]
       camt1_analyse.py g3-control G3_RUN_DIR OUT_JSON [--half-fov-deg H]
       camt1_analyse.py stage-b CONTROL_DIR ENVELOPE_DIR OUT_JSON [--e3-raw DIR] [--camk2e-raw DIR]
       camt1_analyse.py controls OUT_JSON [--camk2e-raw DIR]
"""
import argparse
import hashlib
import json
import math
import pathlib
import random
import statistics
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(HERE))
import camt1_run as RUN                                     # noqa: E402  (plans, arm settings, cell checks)
import camk2v3e_analyse as X3                               # noqa: E402  (gate_d, CAM-K2-E reference helpers)

A2, E1, A, K = X3.A2, X3.A2.E1, X3.A, X3.K
CAMK2E_RAW = pathlib.Path.home() / "motar_tmp" / "camk2e_20261108" / "raw"
G3_JSON = ROOT / "docs/evidence/camonly_2026-11-04/c2trace/c2trace_analysis.json"
G3_CONDS = {"A": "full", "B_out": "no_lidar_out_fov", "C": "no_lidar"}
WINDOWS_S = (0.5, 1.0, 2.0)
VALID_M, AMBIGUOUS_M, MOVING_MPS, MINZ_M = 0.6, 0.1, 0.3, 0.4
VALID_FLOOR = 0.90
BOOT, BOOT_SEED = 10000, 20261112
CATEGORIES = ("BLIND", "LEFT_FOV", "IN_VIEW")
PROFILES = {
    "primary": {"half_fov_deg": 33.0, "vhalf_deg": 27.0, "origin": "eye", "fov_test": "footprint",
                "max_range_m": 10.0, "scope": "quota"},
    "eye_nearest_point": {"half_fov_deg": 33.0, "vhalf_deg": 27.0, "origin": "eye", "fov_test": "nearest_point",
                          "max_range_m": 10.0, "scope": "quota"},
    "g3_geometry_33": {"half_fov_deg": 33.0, "vhalf_deg": None, "origin": "centre", "fov_test": "nearest_point",
                       "max_range_m": math.inf, "scope": "quota"},
    "all_episodes": {"half_fov_deg": 33.0, "vhalf_deg": 27.0, "origin": "eye", "fov_test": "footprint",
                     "max_range_m": 10.0, "scope": "all"},
    "g3": {"half_fov_deg": 43.5, "vhalf_deg": None, "origin": "centre", "fov_test": "nearest_point",
           "max_range_m": math.inf, "scope": "all"},
}
# The per-episode quantities that get a cluster interval (proportions and means).
CI_METRICS = ("BLIND", "LEFT_FOV", "IN_VIEW", "EARLY", "left_side", "seen_0p5s", "seen_1s", "seen_2s",
              "vel_out_of_fov_at_contact", "vel_out_of_fov_1s", "ambiguous", "valid", "speed_mps",
              "target_camera_valid_share_1s", "below_minz")


def wrap(a):
    return (np.asarray(a, dtype=float) + np.pi) % (2 * np.pi) - np.pi


def yaw_of(q):
    q = np.asarray(q, dtype=float)
    x, y, z, w = q[..., 0], q[..., 1], q[..., 2], q[..., 3]
    return np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def roll_pitch(q):
    x, y, z, w = (float(v) for v in q)
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    return math.degrees(roll), math.degrees(pitch)


def surface(p, c, h):
    """G3's horizontal surface distance from p to every axis-aligned bar spanning p's height (inf otherwise)."""
    span = np.abs(p[2] - c[:, 2]) <= h[:, 2]
    d = p[None, :2] - c[:, :2]
    cl = np.clip(d, -h[:, :2], h[:, :2])
    off = (c[:, :2] + cl) - p[None, :2]
    dist = np.hypot(off[:, 0], off[:, 1])
    dist[~span] = np.inf
    return dist, off


def origins(pos, yaw, cam_t, origin):
    """[W, 2] horizontal origin and [W] height of the bearing origin (the camera eye or the vehicle centre)."""
    pos = np.asarray(pos, dtype=float)
    if origin == "centre":
        return pos[:, :2].copy(), pos[:, 2].copy()
    if origin != "eye":
        raise ValueError("origin must be eye or centre")
    tx, ty, tz = (float(v) for v in cam_t)
    c, s = np.cos(yaw), np.sin(yaw)
    o = pos[:, :2] + np.stack([c * tx - s * ty, s * tx + c * ty], axis=1)
    return o, pos[:, 2] + tz


def bar_view(o, oz, yaw, c, h, prof):
    """Per frame: (in FOV, horizontal distance r to the nearest footprint point, bearing of that point) for one bar
    (centre c [3], half extents h [3]) seen from origins o [W, 2] at heights oz [W] with headings yaw [W]."""
    H = math.radians(prof["half_fov_deg"])
    q = np.clip(o, c[None, :2] - h[None, :2], c[None, :2] + h[None, :2])
    off = q - o
    r = np.hypot(off[:, 0], off[:, 1])
    bearing = wrap(np.arctan2(off[:, 1], off[:, 0]) - yaw)
    inside = r <= 0.0
    if prof["fov_test"] == "nearest_point":
        in_h = np.abs(bearing) <= H                            # G3's arithmetic, kept exactly (no inside case)
    elif prof["fov_test"] == "footprint":
        corners = np.array([[sx * h[0], sy * h[1]] for sx in (-1, 1) for sy in (-1, 1)]) + c[None, :2]
        ang = np.arctan2(corners[None, :, 1] - o[:, None, 1], corners[None, :, 0] - o[:, None, 0])
        dc = np.arctan2(c[1] - o[:, 1], c[0] - o[:, 0])
        rel = wrap(ang - dc[:, None])
        lo, hi = rel.min(axis=1), rel.max(axis=1)
        bc = wrap(dc - yaw)
        in_h = np.zeros(len(o), dtype=bool)
        for shift in (-2 * np.pi, 0.0, 2 * np.pi):
            in_h |= (bc + lo + shift <= H) & (bc + hi + shift >= -H)
        in_h |= inside
        bearing = np.where(inside, np.nan, bearing)
    else:
        raise ValueError("fov_test must be footprint or nearest_point")
    if prof["vhalf_deg"] is None:
        in_v = np.ones(len(o), dtype=bool)
    else:
        V = math.radians(prof["vhalf_deg"])
        rr = np.maximum(r, 1e-6)
        el_lo = np.arctan2(c[2] - h[2] - oz, rr)
        el_hi = np.arctan2(c[2] + h[2] - oz, rr)
        in_v = ((el_lo <= V) & (el_hi >= -V)) | inside
    in_range = r <= prof["max_range_m"]
    return in_h & in_v & in_range, r, bearing


def window_mask(frame_idx, last, seconds, dt):
    return frame_idx >= last - int(round(seconds / dt))


def crash_episode(fr, c, h, prof, cam_t, dt, ot, actions=None):
    """All per-episode quantities of one crash episode (fr: its trace frames sorted by frame index)."""
    pos = np.asarray(fr["p_pos"], dtype=float)
    vel = np.asarray(fr["p_vel"], dtype=float)
    yaw = yaw_of(fr["p_vehicle_quat"])
    idx = np.asarray(fr["frame"], dtype=int)
    last = int(idx[-1])
    dist, _off = surface(pos[-1], c, h)
    order = np.argsort(dist)
    j = int(order[0])
    d1 = float(dist[j])
    d2 = float(dist[order[1]]) if len(order) > 1 else math.inf
    dist_x, _ = surface(pos[-1] + np.r_[vel[-1, :2] * dt, 0.0], c, h)
    o, oz = origins(pos, yaw, cam_t, prof["origin"])
    seen, r, bearing = bar_view(o, oz, yaw, c[j], h[j], prof)
    in_last = bool(seen[-1])
    w1 = window_mask(idx, last, 1.0, dt)
    before = w1 & (idx < last)
    if in_last:
        cat = "IN_VIEW"
    elif bool(np.any(seen[before])):
        cat = "LEFT_FOV"
    else:
        cat = "BLIND"
    t_contact = (last + 1) * dt
    H = math.radians(prof["half_fov_deg"])
    v_body = np.stack([np.cos(-yaw) * vel[:, 0] - np.sin(-yaw) * vel[:, 1],
                       np.sin(-yaw) * vel[:, 0] + np.cos(-yaw) * vel[:, 1]], axis=1)
    speed = np.hypot(vel[:, 0], vel[:, 1])
    vb_last = float(math.atan2(v_body[-1, 1], v_body[-1, 0])) if speed[-1] > MOVING_MPS else math.nan
    mean_v = v_body[w1].mean(axis=0)
    vb_1s = float(math.atan2(mean_v[1], mean_v[0])) if float(np.hypot(*mean_v)) > MOVING_MPS else math.nan
    roll, pitch = roll_pitch(fr["p_quat"][-1])
    b_last = float(bearing[-1])
    out = {"category": cat, "EARLY": bool(t_contact <= 1.0 + 1e-9), "contact_bar": j, "dist_m": d1,
           "second_dist_m": d2, "ambiguous": bool(d2 - d1 <= AMBIGUOUS_M), "valid": bool(d1 <= VALID_M),
           "extrapolated_bar_agrees": bool(int(np.argmin(dist_x)) == j), "frames": int(len(fr)),
           "last_frame": last, "frames_contiguous": bool(np.all(np.diff(idx) == 1)) and int(idx[0]) == 0,
           "t_contact_s": t_contact, "actions_match": None if actions is None else bool(actions == last + 1),
           "in_fov_at_contact": in_last, "contact_bearing_deg": math.degrees(b_last) if not math.isnan(b_last)
           else None, "left_side": None if math.isnan(b_last) else bool(b_last > 0),
           "eye_dist_at_contact_m": float(r[-1]), "below_minz": bool(r[-1] < MINZ_M),
           "speed_mps": float(speed[-1]), "speed3_mps": float(np.linalg.norm(vel[-1])),
           "vel_bearing_at_contact_deg": None if math.isnan(vb_last) else math.degrees(vb_last),
           "vel_bearing_1s_deg": None if math.isnan(vb_1s) else math.degrees(vb_1s),
           "vel_out_of_fov_at_contact": None if math.isnan(vb_last) else bool(abs(vb_last) > H),
           "vel_out_of_fov_1s": None if math.isnan(vb_1s) else bool(abs(vb_1s) > H),
           "roll_deg": roll, "pitch_deg": pitch,
           "target_camera_valid_share_1s": float(np.mean(ot.flag(fr[w1], ot.CAMERA_VALID))),
           "target_geo_visible_any_1s": bool(np.any(ot.flag(fr[w1], ot.GEO_VISIBLE)))}
    for T in WINDOWS_S:
        key = "seen_%ss" % ("0p5" if T == 0.5 else int(T))
        out[key] = bool(np.any(seen[window_mask(idx, last, T, dt)]))
    for cat_name in CATEGORIES:
        out[cat_name] = cat == cat_name
    return out


def cell_episodes(cell_dir, prof, ot, cam_t=None):
    """The crash episodes of one traced cell under a profile, plus coverage counts."""
    cell_dir = pathlib.Path(cell_dir)
    tr = ot.read_trace(cell_dir)
    meta = tr["summary"].get("metadata") or {}
    dt = float(meta.get("dt_s"))
    cam_t = cam_t if cam_t is not None else (meta.get("camera") or {}).get("translation_m")
    by_ep = ot.episode_frames(tr["frames"])
    look = ot.reset_lookup(tr["layouts"])
    quats = tr["layouts"]["quat"]
    if len(quats) and not np.allclose(quats, np.array([0, 0, 0, 1.0]), atol=1e-6):
        raise ValueError("%s: non-identity obstacle orientation; the axis-aligned geometry does not apply" % cell_dir)
    rows = K.read_jsonl(cell_dir / "episodes.jsonl")
    actions = {}
    kp = cell_dir / "kroc2027_episodes.jsonl"
    if kp.is_file():
        actions = {(r["env_index"], r["env_episode_index"]): r.get("actions") for r in K.read_jsonl(kp)}
    eps, missing = [], 0
    for row in rows:
        if row.get("outcome") != "crash" or (prof["scope"] == "quota" and not row.get("in_quota")):
            continue
        key = (row["env_index"], row["env_episode_index"])
        fr = by_ep.get(key)
        if fr is None or key not in look or len(fr) == 0:
            missing += 1
            continue
        c, h, _q = ot.layout_arrays(tr["layouts"], look[key])
        e = crash_episode(fr, np.asarray(c, float), np.asarray(h, float), prof, cam_t, dt, ot, actions.get(key))
        e.update(env=key[0], env_episode=key[1], in_quota=bool(row.get("in_quota")))
        eps.append(e)
    return eps, {"crash_rows_without_trace": missing, "dt_s": dt, "camera": meta.get("camera")}


def _cluster_sums(values, clusters):
    keys = sorted(set(clusters))
    pos = {k: i for i, k in enumerate(keys)}
    s, n = np.zeros(len(keys)), np.zeros(len(keys))
    for v, k in zip(values, clusters):
        if v is None or (isinstance(v, float) and math.isnan(v)):
            continue
        s[pos[k]] += float(v)
        n[pos[k]] += 1
    return keys, s, n


def cluster_ci(values, clusters, strata, rng, boot=BOOT):
    """Percentile 95 % interval of the pooled mean (sum / count over the resampled clusters), clusters resampled with
    replacement within their stratum. clusters / strata: one label per value (a cluster lies in one stratum)."""
    keys, s, n = _cluster_sums(values, clusters)
    stratum = {}
    for k, st in zip(clusters, strata):
        stratum.setdefault(k, st)
    groups = {}
    for i, k in enumerate(keys):
        groups.setdefault(stratum[k], []).append(i)
    tot_s, tot_n = np.zeros(boot), np.zeros(boot)
    for st in sorted(groups):
        g = np.array(groups[st])
        pick = g[rng.integers(0, len(g), size=(boot, len(g)))]
        tot_s += s[pick].sum(axis=1)
        tot_n += n[pick].sum(axis=1)
    est = tot_s / np.where(tot_n > 0, tot_n, np.nan)
    est = est[~np.isnan(est)]
    if len(est) == 0:
        return None
    return [float(np.percentile(est, 2.5)), float(np.percentile(est, 97.5))]


def summarize(eps, rng, keys=CI_METRICS):
    """Pooled means and three intervals per metric: cell clusters within density (primary), env clusters within
    cell, and episodes (G3's)."""
    out = {"crash_episodes": len(eps)}
    if not eps:
        return out
    cells = [(e["density"], e["policy_seed"], e["seed"]) for e in eps]
    dens = [e["density"] for e in eps]
    envs = [(e["density"], e["policy_seed"], e["seed"], e["env"]) for e in eps]
    for k in keys:
        vals = [None if e.get(k) is None else float(e[k]) for e in eps]
        ok = [v for v in vals if v is not None]
        out[k] = {"mean": sum(ok) / len(ok) if ok else None, "n": len(ok),
                  "ci95_cell_within_density": cluster_ci(vals, cells, dens, rng),
                  "ci95_env_within_cell": cluster_ci(vals, envs, cells, rng),
                  "ci95_episode": cluster_ci(vals, list(range(len(vals))), [0] * len(vals), rng)}
    for k in ("contact_bearing_deg", "speed_mps", "roll_deg", "pitch_deg", "t_contact_s", "dist_m",
              "vel_bearing_1s_deg"):
        ok = [e[k] for e in eps if e.get(k) is not None]
        out["quantiles_" + k] = ({"p10": float(np.percentile(ok, 10)), "p50": float(np.percentile(ok, 50)),
                                  "p90": float(np.percentile(ok, 90))} if ok else None)
    out["abs_contact_bearing_deg_median"] = (float(np.median([abs(e["contact_bearing_deg"]) for e in eps
                                                              if e["contact_bearing_deg"] is not None]))
                                             if any(e["contact_bearing_deg"] is not None for e in eps) else None)
    out["category_counts"] = {c: sum(1 for e in eps if e["category"] == c) for c in CATEGORIES}
    out["early_by_category"] = {c: sum(1 for e in eps if e["category"] == c and e["EARLY"]) for c in CATEGORIES}
    out["extrapolated_bar_agrees_share"] = sum(e["extrapolated_bar_agrees"] for e in eps) / len(eps)
    am = [e["actions_match"] for e in eps if e["actions_match"] is not None]
    out["actions_match_share"] = sum(am) / len(am) if am else None
    out["frames_contiguous_share"] = sum(e["frames_contiguous"] for e in eps) / len(eps)
    return out


def dominant(pooled):
    """The category with the largest pooled share, if CLEAR (its primary interval's lower end above both other
    categories' upper ends), else None."""
    shares = {c: pooled[c]["mean"] for c in CATEGORIES}
    top = max(shares, key=shares.get)
    lo = (pooled[top]["ci95_cell_within_density"] or [0, 0])[0]
    clear = all(lo > (pooled[c]["ci95_cell_within_density"] or [1, 1])[1] for c in CATEGORIES if c != top)
    return (top if clear else None), shares


def guidance(pooled, pooled_nearest=None):
    """Retraining guidance from the pooled categories (not a gate). The footprint test (primary) and the nearest-point
    test (eye_nearest_point) disagree most for wide bars beside the camera, so a direction is named only when both
    give the same CLEAR dominant category; otherwise DEFINITION_SENSITIVE (or MIXED when the primary has none)."""
    top, shares = dominant(pooled)
    top_n, shares_n = dominant(pooled_nearest) if pooled_nearest is not None else (top, None)
    text = {"BLIND": "out-of-FOV obstacle memory, or a wider depth FOV (80 deg)",
            "LEFT_FOV": "short-term memory of obstacles that were seen and left the view",
            "IN_VIEW": "depth fidelity near the bar (MinZ, the validity rule) or speed (see Stage B)"}
    if top is None:
        dom, direction = "MIXED", "no single dominant category; no single lever indicated"
    elif top_n != top:
        dom, direction = "DEFINITION_SENSITIVE", ("the dominant category depends on the in-FOV definition "
                                                  "(primary %s, nearest point %s); no single lever indicated"
                                                  % (top, top_n))
    else:
        dom, direction = top, text[top]
    g = {"dominant": dom, "shares": shares, "shares_nearest_point": shares_n, "direction": direction}
    early = pooled["EARLY"]["mean"]
    if early is not None and early >= 0.2:
        g["early_note"] = "EARLY crashes are %.1f %%: initial placement, not only the policy" % (100 * early)
    return g


def g3_control(run_dir, out, half_fov_deg=43.5):
    """The positive control: the G3 profile (origin centre, nearest point, no vertical or range test, every traced
    episode) at half-FOV 43.5 deg must reproduce G3's recorded out_last and never_in_1s exactly; at 33 deg (negative
    control) out_last must differ from the record."""
    ot = RUN.load_ot()
    rec = json.loads(G3_JSON.read_text())["conditions"]
    res = {"analysis": "CAM-T1 positive control on G3", "g3_record": str(G3_JSON.relative_to(ROOT)),
           "g3_record_sha256": hashlib.sha256(G3_JSON.read_bytes()).hexdigest(), "conditions": {}}
    ok = True
    for half, label in ((half_fov_deg, "at_half_fov"), (33.0, "negative_half_fov_33")):
        prof = dict(PROFILES["g3"], half_fov_deg=half)
        for name, cond in G3_CONDS.items():
            eps = []
            for d in (70, 115, 160, 205):
                e, _cov = cell_episodes(pathlib.Path(run_dir) / cond / (RUN.CELL % (d, 4501)), prof, ot)
                eps += e
            n = len(eps)
            got = {"crash_episodes_traced": n, "out_last_pct": 100 * sum(not e["IN_VIEW"] for e in eps) / n,
                   "never_in_1s_pct": 100 * sum(e["BLIND"] for e in eps) / n,
                   "valid_share_pct": 100 * sum(e["valid"] for e in eps) / n,
                   "left_pct": 100 * sum(bool(e["left_side"]) for e in eps) / n}
            r = res["conditions"].setdefault(name, {})
            r[label] = got
            if label == "at_half_fov":
                match = {k: abs(got[k] - rec[name][k]) < 1e-9 for k in got}
                r["recorded"] = {k: rec[name][k] for k in got}
                r["matches_record"] = match
                ok &= all(match.values())
            else:
                r["negative_differs"] = got["out_last_pct"] != rec[name]["out_last_pct"]
                ok &= r["negative_differs"]
    res["half_fov_deg"] = half_fov_deg
    res["script_sha256"] = E1.sha(__file__)
    res["reading"] = "POSITIVE_CONTROL_PASS" if ok and half_fov_deg == 43.5 else "POSITIVE_CONTROL_FAIL"
    _write(out, res)
    return res


def _write(out, res):
    with pathlib.Path(out).open("x") as fh:
        json.dump(res, fh, indent=1, default=str, sort_keys=True)
        fh.write("\n")


def _report(path):
    """A run report, or {} when it is missing or unreadable (every binding then fails closed)."""
    try:
        r = json.loads(pathlib.Path(path).read_text())
        return r if isinstance(r, dict) else {}
    except (OSError, ValueError):
        return {}


def trace_binding(trace_dir, e3_raw):
    """The trace run is finished (24 cells in plan order, one commit), each cell passes the runner's checks again,
    and the references are at CAM-K2-E3's manifest."""
    rep = _report(pathlib.Path(trace_dir) / RUN.REPORT["trace"])
    plan = RUN.trace_plan()
    b = {"trace_reading_finished": rep.get("reading") in RUN.FINISHED_TRACE,
         "cells_in_plan_order": [(c.get("arm"), c.get("density"), c.get("seed")) for c in rep.get("cells", [])]
         == [tuple(x) for x in plan],
         "e3_references_at_manifest": not RUN.e3_problems(e3_raw),
         "script_equals_run_commit": E1.file_matches_commit(__file__, "tools/camt1_analyse.py", rep.get("commit"))}
    ot = RUN.load_ot()
    checks_ok = True
    for arm, d, s in plan:
        cell = pathlib.Path(trace_dir) / arm / (RUN.CELL % (d, s))
        try:
            checks_ok &= not (A2.R2.receipt_problems(cell, "K2s1", s, rep.get("committed_sha256") or {})
                              or RUN.V3.validity_problems(cell, True, rep.get("validity_sha256"))
                              or RUN.camt1_problems(cell, arm, RUN.reference_dir(e3_raw, arm, d, s), ot))
        except (OSError, ValueError, KeyError, TypeError):
            checks_ok = False
    b["cells_pass_the_runner_checks_again"] = bool(checks_ok)
    return b, rep


CONSISTENCY_FLOOR = 0.99


def consistency_problems(eps, coverage):
    """Review r1 F1 (and the VALID floor): per policy seed, MEASUREMENT_INVALID reasons when the measurement itself is
    not trustworthy. eps: every traced quota crash episode of the primary profile (VALID or not, with policy_seed);
    coverage: {cell label: {"policy_seed", "crash_episodes", "crash_rows_without_trace"}}."""
    reasons = []
    for ps in (1, 2):
        e = [x for x in eps if x["policy_seed"] == ps]
        cov = [c for c in coverage.values() if c["policy_seed"] == ps]
        traced = sum(c["crash_episodes"] for c in cov)
        rows = traced + sum(c["crash_rows_without_trace"] for c in cov)
        name = "V3s%d" % ps
        if not e:
            reasons.append("%s: no crash episode" % name)
            continue
        if rows == 0 or traced / rows < CONSISTENCY_FLOOR:
            reasons.append("%s: trace coverage %d / %d quota crash rows < %.2f" % (name, traced, rows,
                                                                                CONSISTENCY_FLOOR))
        am = [x["actions_match"] for x in e]
        if any(a is None for a in am) or sum(am) / len(am) < CONSISTENCY_FLOOR:
            reasons.append("%s: actions_match share below %.2f or not recorded" % (name, CONSISTENCY_FLOOR))
        if sum(x["frames_contiguous"] for x in e) / len(e) < CONSISTENCY_FLOOR:
            reasons.append("%s: frames_contiguous share below %.2f" % (name, CONSISTENCY_FLOOR))
        if sum(x["valid"] for x in e) / len(e) < VALID_FLOOR:
            reasons.append("%s: VALID share below %.2f" % (name, VALID_FLOOR))
    return reasons


def split_summaries(eps_all, rng):
    """Review r1 F2: the category shares and their intervals over VALID crash episodes (the contacted bar identified);
    the non-VALID episodes are counted and split separately, and the all-episode shares kept as a sensitivity."""
    valid = [e for e in eps_all if e["valid"]]
    invalid = [e for e in eps_all if not e["valid"]]
    r = {"pooled": summarize(valid, rng)}
    for ps in (1, 2):
        r["V3s%d" % ps] = summarize([e for e in valid if e["policy_seed"] == ps], rng)
    r["invalid_not_in_shares"] = {
        "V3s%d" % ps: {"count": sum(1 for e in invalid if e["policy_seed"] == ps),
                       "categories": {c: sum(1 for e in invalid if e["policy_seed"] == ps and e["category"] == c)
                                      for c in CATEGORIES},
                       "dist_m_quantiles": ([float(np.percentile([e["dist_m"] for e in invalid
                                                                  if e["policy_seed"] == ps], q)) for q in (10, 50, 90)]
                                            if any(e["policy_seed"] == ps for e in invalid) else None)}
        for ps in (1, 2)}
    r["sensitivity_all_crashes"] = {"pooled": {c: (sum(e[c] for e in eps_all) / len(eps_all) if eps_all else None)
                                               for c in CATEGORIES + ("EARLY",)}, "crash_episodes": len(eps_all)}
    r["per_density_counts_valid"] = {"V3s%d/%d" % (ps, d): {c: sum(1 for e in valid if e["policy_seed"] == ps
                                                                  and e["density"] == d and e[c])
                                                           for c in CATEGORIES + ("EARLY",)}
                                     for ps in (1, 2) for d in RUN.TRACE_DENSITIES}
    r["per_density_invalid"] = {"V3s%d/%d" % (ps, d): sum(1 for e in invalid if e["policy_seed"] == ps
                                                          and e["density"] == d)
                                for ps in (1, 2) for d in RUN.TRACE_DENSITIES}
    return r


def stage_a_binding_extra(trace_dir, rep, g3_control_json):
    """Review r1 F4 / F7: the identity stage read IDENTITY_PASS on the trace run's commit (one report beside the trace
    folder), and the chain's G3 positive control at this script's bytes read POSITIVE_CONTROL_PASS."""
    ids = sorted(pathlib.Path(trace_dir).parent.glob("*/" + RUN.REPORT["identity"]))
    ident = _report(ids[0]) if len(ids) == 1 else {}
    pc = _report(g3_control_json) if g3_control_json else {}
    return {"identity_pass_same_commit": ident.get("reading") == "IDENTITY_PASS"
            and bool(rep.get("commit")) and ident.get("commit") == rep.get("commit"),
            "g3_positive_control_pass": pc.get("reading") == "POSITIVE_CONTROL_PASS"
            and pc.get("half_fov_deg") == 43.5 and pc.get("script_sha256") == E1.sha(__file__)}


def stage_a(trace_dir, out, e3_raw, half_fov_deg=33.0, vfov_deg=54.0, g3_control_json=None):
    trace_dir = pathlib.Path(trace_dir).resolve()          # the receipts record resolved cell paths
    e3_raw = pathlib.Path(e3_raw).resolve()
    binding, rep = trace_binding(trace_dir, e3_raw)
    binding.update(stage_a_binding_extra(trace_dir, rep, g3_control_json))
    res = {"analysis": "CAM-T1 Stage A", "prereg": RUN.PREREG, "binding": binding, "commit": rep.get("commit"),
           "trace_reading": rep.get("reading"), "script_sha256": E1.sha(__file__)}
    if not all(binding.values()):
        res["reading"] = "TRACE_BINDING_FAIL"
        _write(out, res)
        return res
    ot = RUN.load_ot()
    profiles = {k: dict(v) for k, v in PROFILES.items() if k != "g3"}
    for k in ("primary", "all_episodes", "eye_nearest_point"):
        profiles[k].update(half_fov_deg=half_fov_deg, vhalf_deg=vfov_deg / 2.0)
    profiles["g3_geometry_33"]["half_fov_deg"] = half_fov_deg
    res["profiles"] = {k: {kk: (None if vv == math.inf else vv) for kk, vv in v.items()} for k, v in profiles.items()}
    results, coverage, eps_by = {}, {}, {}
    try:
        for name, prof in profiles.items():
            rng = np.random.default_rng(BOOT_SEED)
            eps_all = []
            for arm, d, s in RUN.trace_plan():
                ps = RUN.arm_spec(arm)["policy_seed"]
                eps, cov = cell_episodes(trace_dir / arm / (RUN.CELL % (d, s)), prof, ot)
                for e in eps:
                    e.update(density=d, seed=s, policy_seed=ps)
                eps_all += eps
                coverage.setdefault(name, {})["%s/%d/%d" % (arm, d, s)] = {"crash_episodes": len(eps),
                                                                          "policy_seed": ps, **cov}
            eps_by[name] = eps_all
            results[name] = split_summaries(eps_all, rng)
    except (ValueError, OSError, KeyError, TypeError) as e:  # review r1 F10, r2 N6: write a reading, never exit
        res.update(reading="MEASUREMENT_INVALID", reasons=["geometry or trace unsupported: %s" % e])
        _write(out, res)
        print(json.dumps({"reading": res["reading"], "reasons": res["reasons"]}, indent=1))
        return res
    res["results"], res["coverage"] = results, coverage
    reasons = consistency_problems(eps_by["primary"], coverage["primary"])
    prim = results["primary"]
    res["reasons"] = reasons
    res["reading"] = "MEASUREMENT_INVALID" if reasons else "DESCRIBED"
    res["guidance_not_a_gate"] = None if reasons else guidance(prim["pooled"],
                                                              results["eye_nearest_point"]["pooled"])
    res["bootstrap"] = {"resamples": BOOT, "seed": BOOT_SEED, "primary": "cells within density",
                        "shares_over": "VALID crash episodes"}
    _write(out, res)
    print(json.dumps({"reading": res["reading"], "reasons": reasons, "trace_reading": res["trace_reading"],
                      "primary_pooled_valid": {c: prim["pooled"][c]["mean"] for c in CATEGORIES + ("EARLY",)}
                      if not reasons else None}, indent=1))
    return res


# ------------------------------------------------------------------------------------------------- Stage B
def cell_rates(cell_dir):
    """Quota rates (gr_analyse.rates) and the median capture time of the quota capture episodes, in seconds."""
    cell_dir = pathlib.Path(cell_dir)
    out = {"dir": str(cell_dir), **A.rates(cell_dir)}
    kr = {(r["env_index"], r["env_episode_index"]): r for r in K.read_jsonl(cell_dir / "kroc2027_episodes.jsonl")}
    t = [kr[(r["env_index"], r["env_episode_index"])]["actions"] * kr[(r["env_index"], r["env_episode_index"])]["rl_dt_s"]
         for r in K.read_jsonl(cell_dir / "episodes.jsonl") if r.get("in_quota") and r.get("outcome") == "capture"]
    out["median_capture_s"] = float(statistics.median(t)) if t else None
    cm = [r["pursuer_commanded_speed_max_mps"] for r in kr.values()]
    out["commanded_max_mps"] = max(cm) if cm else None
    out["share_commanded_over_2p5"] = sum(c > 2.5 + RUN.CAP_TOLERANCE_MPS for c in cm) / len(cm) if cm else None
    out["share_commanded_over_2p0"] = sum(c > 2.0 + RUN.CAP_TOLERANCE_MPS for c in cm) / len(cm) if cm else None
    act = [r["pursuer_actual_speed_mean_mps"] for r in kr.values() if r.get("pursuer_actual_speed_mean_mps") is not None]
    out["mean_actual_speed_mps"] = sum(act) / len(act) if act else None
    return out


def contrast_seconds(cells, weights, blocks, rng):
    """A2.contrast for a metric already in seconds (median capture time): the same stratified bootstrap and t."""
    xs = [sum(w * cells[k][b]["median_capture_s"] for k, w in weights.items()) for b in blocks]
    t, df = A2.strat_t(xs, blocks)
    return {"mean_s": sum(xs) / len(xs), "ci95_s": E1.strat_ci(xs, blocks, rng), "t95_s": t, "t_df": df,
            "blocks_s": xs}


def stage_b_blocks():
    return [(d, s) for d in RUN.ENVELOPE_DENSITIES for s in RUN.SEEDS]


def gate_controls(k2e, blocks):
    """CAM-K2-E3's gate_d rule on CAM-K2-E's treated cells at the Stage B blocks: the known-bad K1s - K2s must read
    WORSE, K2s - K1s SMALL_ or MATERIAL_REDUCTION, and the null K2s1 - K2s2 neither MATERIAL_REDUCTION nor WORSE.
    One generator, fixed order."""
    cells = {**X3.load_reference(k2e, blocks), **X3.load_camk2e_k1(k2e, blocks)}
    rng = random.Random(A2.BOOT_SEED)
    defs = []
    for s in (1, 2):
        defs.append(("K1s%d_minus_K2s%d" % (s, s), {"K2E_K1s%d" % s: 1, "V2s%d" % s: -1}, ("WORSE",)))
        defs.append(("K2s%d_minus_K1s%d" % (s, s), {"V2s%d" % s: 1, "K2E_K1s%d" % s: -1},
                     ("SMALL_REDUCTION", "MATERIAL_REDUCTION")))
    defs.append(("K2s1_minus_K2s2_null", {"V2s1": 1, "V2s2": -1}, None))
    out = {}
    for name, w, need in defs:
        c = {m: A2.contrast(cells, w, blocks, m, rng) for m in A2.METRICS}
        label = X3.gate_d(c["crash"]["ci95_pp"], c["crash"]["t95_pp"], c["crash"]["mean_pp"], c["timeout"]["mean_pp"])
        ok = label in need if need else label not in ("MATERIAL_REDUCTION", "WORSE")
        out[name] = {"label": label, "ok": ok, "required": list(need) if need else
                     "not MATERIAL_REDUCTION and not WORSE", "crash": c["crash"]}
    manifest = X3.camk2e_manifest_ok(k2e, blocks, arms=("K1s1", "K1s2", "K2s1", "K2s2"))
    return out, manifest


def stage_b(control_dir, envelope_dir, out, e3_raw, k2e):
    control_dir, envelope_dir = pathlib.Path(control_dir).resolve(), pathlib.Path(envelope_dir).resolve()
    e3_raw = pathlib.Path(e3_raw).resolve()
    ctrl = _report(control_dir / RUN.REPORT["control"])
    env = _report(envelope_dir / RUN.REPORT["envelope"])
    plan = RUN.envelope_plan()
    blocks = stage_b_blocks()
    binding = {"control_pass": ctrl.get("reading") == "CONTROL_PASS",
               "same_commit": ctrl.get("commit") == env.get("commit"),
               "envelope_complete": env.get("reading") == "COMPLETE" and env.get("cells_expected") == len(plan) == 36,
               "order_as_planned": [(c.get("arm"), c.get("density"), c.get("seed")) for c in env.get("cells", [])]
               == [tuple(x) for x in plan],
               "e3_references_at_manifest": not RUN.e3_problems(e3_raw),
               "script_equals_run_commit": E1.file_matches_commit(__file__, "tools/camt1_analyse.py", env.get("commit"))}
    ids = sorted(control_dir.parent.glob("*/" + RUN.REPORT["identity"]))
    ident = _report(ids[0]) if len(ids) == 1 else {}
    binding["identity_pass_same_commit"] = (ident.get("reading") == "IDENTITY_PASS" and bool(env.get("commit"))
                                            and ident.get("commit") == env.get("commit"))    # review r1 F4
    ot = RUN.load_ot()
    checks_ok = True
    cells = {}
    for arm, d, s in plan:
        cd = envelope_dir / arm / (RUN.CELL % (d, s))
        try:
            checks_ok &= not (A2.R2.receipt_problems(cd, "K2s1", s, env.get("committed_sha256") or {})
                              or RUN.V3.validity_problems(cd, True, env.get("validity_sha256"))
                              or RUN.camt1_problems(cd, arm, RUN.reference_dir(e3_raw, arm, d, s), ot))
            cells.setdefault(arm, {})[(d, s)] = cell_rates(cd)
        except (OSError, ValueError, KeyError, TypeError):
            checks_ok = False
    binding["cells_pass_the_runner_checks_again"] = bool(checks_ok)
    for env_arm, ctrl_arm in (("d1s1", "d1_smoke"), ("cap20s1", "cap20_smoke")):
        a = envelope_dir / env_arm / (RUN.CELL % (205, 4501))
        b = control_dir / ctrl_arm / (RUN.CELL % (205, 4501))
        try:
            binding["determinism_%s_equals_%s" % (env_arm, ctrl_arm)] = RUN.ledgers_equal(a, b)["pass"]
        except OSError:
            binding["determinism_%s_equals_%s" % (env_arm, ctrl_arm)] = False
    for ps in (1, 2):
        cells["native_s%d" % ps] = {b: cell_rates(RUN.reference_dir(e3_raw, "V3s%d" % ps, *b)) for b in blocks}
    res = {"analysis": "CAM-T1 Stage B", "prereg": RUN.PREREG, "binding": binding, "commit": env.get("commit"),
           "script_sha256": E1.sha(__file__)}
    if not all(binding.values()) or any(sorted(cells.get(a, {})) != blocks for a in RUN.ENVELOPE_ARMS):
        res["reading"] = "EVAL_BINDING_FAIL"
        _write(out, res)
        return res
    controls, manifest = gate_controls(k2e, blocks)
    ctrl_ok = manifest and all(c["ok"] for c in controls.values())
    res.update(controls=controls, controls_ok=ctrl_ok, camk2e_manifest_ok=manifest)
    rng = random.Random(A2.BOOT_SEED)
    contrasts, labels = {}, {}
    for kind in ("cap25", "cap20", "d1"):
        defs = {"s1": {kind + "s1": 1, "native_s1": -1}, "s2": {kind + "s2": 1, "native_s2": -1},
                "pooled": {kind + "s1": 0.5, kind + "s2": 0.5, "native_s1": -0.5, "native_s2": -0.5}}
        contrasts[kind], labels[kind] = {}, {}
        for name, w in defs.items():
            c = {m: A2.contrast(cells, w, blocks, m, rng) for m in A2.METRICS}
            c["median_capture_time"] = contrast_seconds(cells, w, blocks, rng)
            contrasts[kind][name] = c
            labels[kind][name] = X3.gate_d(c["crash"]["ci95_pp"], c["crash"]["t95_pp"], c["crash"]["mean_pp"],
                                           c["timeout"]["mean_pp"])
        labels[kind]["reading"] = E1.lineage(labels[kind]["s1"], labels[kind]["s2"]) if ctrl_ok else None
    res.update(reading="REPORTED" if ctrl_ok else "CONTROL_FAIL", labels=labels if ctrl_ok else None,
               contrasts=contrasts,
               rates_pct={a: {m: 100.0 * sum(v[m] for v in c.values()) / len(c) for m in A2.METRICS}
                          for a, c in cells.items()},
               speed={a: {k: [c[b][k] for b in blocks] for k in ("commanded_max_mps", "share_commanded_over_2p5",
                                                                  "share_commanded_over_2p0",
                                                                  "mean_actual_speed_mps", "median_capture_s")}
                      for a, c in cells.items()},
               blocks=[list(b) for b in blocks],
               bootstrap={"resamples": A.BOOT, "seed": A2.BOOT_SEED, "scheme": "seeds within density", "t_df": 4})
    _write(out, res)
    print(json.dumps({"reading": res["reading"], "labels": res["labels"],
                      "controls": {k: v["label"] for k, v in controls.items()}}, indent=1))
    return res


def main(argv=None):
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)
    a = sub.add_parser("stage-a")
    a.add_argument("trace_dir")
    a.add_argument("out")
    a.add_argument("--e3-raw", default=str(RUN.E3_RAW))
    a.add_argument("--half-fov-deg", type=float, default=33.0)
    a.add_argument("--vfov-deg", type=float, default=54.0)
    a.add_argument("--g3-control", required=True, help="the chain's g3-control JSON at this script (review r1 F7)")
    g = sub.add_parser("g3-control")
    g.add_argument("g3_run_dir")
    g.add_argument("out")
    g.add_argument("--half-fov-deg", type=float, default=43.5)
    b = sub.add_parser("stage-b")
    b.add_argument("control_dir")
    b.add_argument("envelope_dir")
    b.add_argument("out")
    b.add_argument("--e3-raw", default=str(RUN.E3_RAW))
    b.add_argument("--camk2e-raw", default=str(CAMK2E_RAW))
    c = sub.add_parser("controls")
    c.add_argument("out")
    c.add_argument("--camk2e-raw", default=str(CAMK2E_RAW))
    args = ap.parse_args(argv)
    if pathlib.Path(args.out).exists():
        raise SystemExit("refusing to overwrite " + args.out)
    if args.cmd == "stage-a":
        res = stage_a(args.trace_dir, args.out, pathlib.Path(args.e3_raw), args.half_fov_deg, args.vfov_deg,
                      args.g3_control)
        return 0 if res["reading"] == "DESCRIBED" else 1
    if args.cmd == "g3-control":
        res = g3_control(args.g3_run_dir, args.out, args.half_fov_deg)
        print(json.dumps({"reading": res["reading"], **{k: {kk: vv for kk, vv in v.items()
                                                           if kk in ("at_half_fov", "recorded", "negative_half_fov_33")}
                                                        for k, v in res["conditions"].items()}}, indent=1))
        return 0 if res["reading"] == "POSITIVE_CONTROL_PASS" else 1
    if args.cmd == "stage-b":
        res = stage_b(args.control_dir, args.envelope_dir, args.out, pathlib.Path(args.e3_raw),
                      pathlib.Path(args.camk2e_raw))
        return 0 if res["reading"] == "REPORTED" else 1
    k2e = pathlib.Path(args.camk2e_raw)
    controls, manifest = gate_controls(k2e, stage_b_blocks())
    res = {"analysis": "CAM-T1 Stage B controls on existing data", "controls": controls,
           "camk2e_manifest_ok": manifest, "ok": manifest and all(c["ok"] for c in controls.values())}
    _write(args.out, res)
    print(json.dumps({k: (v["label"], round(v["crash"]["mean_pp"], 2)) for k, v in controls.items()}, indent=1))
    return 0 if res["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
