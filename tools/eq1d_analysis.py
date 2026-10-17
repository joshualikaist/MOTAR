#!/usr/bin/env python3
"""EQ-1D analysis (CPU): every preregistered gate from the raw D1/D2/D3 outputs against independent references.

    python -B tools/eq1d_analysis.py --run DIR --out docs/evidence/.../eq1d_analysis.json

Never imports the simulator or production sensor/physics code. References: closed-form motor
update and exact first-order lag (D1), dt self-convergence, URDF mass, an oriented-box separating-axis
test for contact, the riskcap stopping model; the EQ-1A analytic camera/LiDAR geometry
(`tools/audits/eq1_geometry_reference.py`, pinhole rays, ray/sphere, ray/box, sub-pixel silhouettes)
and the OCC offline line-of-sight indicators (D2); bitwise comparison records (D3). The gate texts are
those of `docs/plans/eq1d_qualification_v1.json`. D1-K1's geometric contact is SAT separation
<= 0.004 m, the sum of both shapes' PhysX contact offsets (2 x 0.002 m), the same 0.004 m as the
gate's window.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools/audits"))
import eq1_geometry_reference as geo  # noqa: E402  (independent analytic references)

PLAN = json.loads((ROOT / "docs/plans/eq1d_qualification_v1.json").read_text())
TAU = 0.04
MASS = 0.225 + 4 * 0.00625
G = 9.81
BRAKE, REACTION = 2.9608856678, 0.1
BOX_HALF = np.array([0.14, 0.14, 0.04])
CONTACT_SEP = 0.004
FORCE_THR = 0.05
D1_RUNS = ("d1_dt10", "d1_dt5", "d1_dt2p5")


def status(ok, note=""):
    return {"status": "PASS" if ok else "FAIL", "note": note}


def not_tested(note):
    return {"status": "NOT_TESTED", "note": note}


# ------------------------------------------------------------------ geometry helpers
def quat_to_rot(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def tilt_deg(q):
    q = np.asarray(q, dtype=np.float64)
    return np.degrees(np.arccos(np.clip(1 - 2 * (q[..., 0] ** 2 + q[..., 1] ** 2), -1, 1)))


def quat_angle_deg(q1, q2):
    d = np.abs(np.sum(np.asarray(q1, np.float64) * np.asarray(q2, np.float64), axis=-1))
    return np.degrees(2 * np.arccos(np.clip(d, -1, 1)))


def sat_separation(c1, r1, h1, c2, h2):
    """Largest separating-axis gap between an oriented box (centre c1, rotation r1, half h1) and an
    axis-aligned box (c2, h2); <= 0 means overlap. Exact for face contact, a lower bound otherwise."""
    axes = [r1[:, i] for i in range(3)] + [np.eye(3)[i] for i in range(3)]
    for i in range(3):
        for j in range(3):
            a = np.cross(r1[:, i], np.eye(3)[j])
            n = np.linalg.norm(a)
            if n > 1e-9:
                axes.append(a / n)
    d = c2 - c1
    best = -math.inf
    for a in axes:
        p1 = sum(h1[i] * abs(r1[:, i] @ a) for i in range(3))
        p2 = sum(h2[i] * abs(a[i]) for i in range(3))
        best = max(best, abs(d @ a) - p1 - p2)
    return best


# ------------------------------------------------------------------ D1
def production_gain(dt, tau=TAU):
    h = dt / (dt + tau)
    return h + h * h / 2 + h ** 3 / 6 + h ** 4 / 24


def load_d1(run):
    out = {}
    for rid in D1_RUNS:
        d = Path(run) / rid
        if (d / "d1_substeps.npz").exists():
            z = np.load(d / "d1_substeps.npz")
            out[rid] = ({k: z[k] for k in z.files}, json.loads((d / "d1_meta.json").read_text()))
    return out


def analyse_d1(run):
    runs = load_d1(run)
    fixtures = PLAN["D1"]["fixtures"]
    res = {"runs": sorted(runs), "gates": {}, "per_dt": {}, "per_fixture": {}}
    if len(runs) != 3:
        for g in PLAN["D1"]["gates"]:
            res["gates"][g] = not_tested("D1 runs present: %s" % sorted(runs))
        return res
    t1_ok, m1_worst, m2_worst = True, 0.0, 0.0
    boundaries = {}
    for rid, (z, meta) in runs.items():
        dt, n_sub = meta["dt_s"], meta["substeps_per_action"]
        ai = z["action_index"]
        counts = np.bincount(ai, minlength=meta["actions"])
        cmd = z["command"]
        const = all(np.array_equal(cmd[ai == a], np.repeat(cmd[ai == a][:1], (ai == a).sum(), axis=0))
                    for a in range(meta["actions"]))
        period_ok = abs(n_sub * dt - PLAN["D1"]["action_period_s"]) <= 1e-9
        ok = bool((counts == n_sub).all() and const and period_ok)
        t1_ok &= ok
        # Motor fidelity and intended-lag agreement (RPM domain; k_T cancels).
        thrust = z["motor_thrust"].astype(np.float64)
        ref = np.clip(z["motor_ref_thrust"].astype(np.float64), 0.0, meta["max_thrust"])
        prev = np.concatenate([z["initial_motor_thrust"][None].astype(np.float64), thrust[:-1]], axis=0)
        w, wr = np.sqrt(prev), np.sqrt(ref)
        pred = (w + production_gain(dt) * (wr - w)) ** 2
        err = np.abs(pred - thrust) / (1e-4 * np.abs(thrust) + 1e-6)
        m1 = float(err.max())
        m1_worst = max(m1_worst, m1)
        sel = np.abs(wr - w) > 0.01 * np.maximum(w, 1e-6)
        g_obs = (np.sqrt(thrust) - w)[sel] / (wr - w)[sel]
        exact = 1 - math.exp(-dt / TAU)
        m2 = float(np.max(np.abs(g_obs / exact - 1))) if sel.any() else 0.0
        m2_worst = max(m2_worst, m2)
        res["per_dt"][rid] = {"dt_s": dt, "substeps_per_action": n_sub, "T1_ok": ok,
                              "motor_fidelity_max_normalised_error": m1, "motor_gain_substeps_compared": int(sel.sum()),
                              "motor_gain_max_rel_dev": m2, "motor_gain_production_closed_form": production_gain(dt),
                              "motor_gain_exact_lag": exact}
        idx = np.arange(meta["actions"]) * n_sub + n_sub - 1
        boundaries[rid] = (z["position"][idx].astype(np.float64), z["orientation_xyzw"][idx].astype(np.float64))
    res["gates"]["D1-T1"] = status(t1_ok)
    res["gates"]["D1-M1"] = status(m1_worst <= 1.0, "max |pred - logged| / (1e-4|T| + 1e-6 N) = %.4g" % m1_worst)
    res["gates"]["D1-M2"] = status(m2_worst <= 0.005, "max relative gain deviation %.4g (tolerance 0.005)" % m2_worst)
    res["gates"]["D1-M2"]["known_cpu_fails"] = "EQ-1B M1 (not classical RK4) and M2 (order 3 < 3.5) remain FAIL"
    # Convergence (non-contact fixtures).
    (p10, q10), (p5, q5), (p25, q25) = boundaries["d1_dt10"], boundaries["d1_dt5"], boundaries["d1_dt2p5"]
    c_rows, c_ok = [], True
    for i, f in enumerate(fixtures):
        if "contact" in f:
            continue
        e105 = float(np.linalg.norm(p10[:, i] - p5[:, i], axis=1).max())
        e525 = float(np.linalg.norm(p5[:, i] - p25[:, i], axis=1).max())
        a105 = float(quat_angle_deg(q10[:, i], q5[:, i]).max())
        a525 = float(quat_angle_deg(q5[:, i], q25[:, i]).max())
        ok = e525 <= 0.01 and a525 <= 0.5 and (e105 <= 1e-6 or e525 < e105)
        c_ok &= ok
        c_rows.append({"fixture": i, "name": f["name"], "pos_10v5_m": e105, "pos_5v2p5_m": e525, "att_10v5_deg": a105,
                       "att_5v2p5_deg": a525, "status": "PASS" if ok else "FAIL"})
    res["per_fixture"]["convergence"] = c_rows
    res["gates"]["D1-C1"] = status(c_ok, "%d of %d fixtures pass" % (sum(r["status"] == "PASS" for r in c_rows), len(c_rows)))
    # Hover, velocity, braking, tilt per dt.
    h_rows, v_rows, b_rows, a_rows = [], [], [], []
    for rid, (z, meta) in runs.items():
        dt, n_sub = meta["dt_s"], meta["substeps_per_action"]
        last2 = slice(int(round(-2.0 / dt)), None)
        total = z["motor_thrust"][last2, 0].sum(axis=1).astype(np.float64)
        h_rows.append({"run": rid, "mean_total_thrust_N": float(total.mean()), "mg_N": MASS * G,
                       "rel_err": float(total.mean() / (MASS * G) - 1),
                       "hover_z_drift_m": float(z["position"][-1, 0, 2] - z["initial_robot_position"][0, 2])})
        for i in (1, 2, 3, 4, 7):
            cmd = np.array(fixtures[i]["segments"][-1][1][:2], dtype=np.float64)
            v = z["linvel_w"][int(round(-1.0 / dt)):, i, :2].astype(np.float64).mean(axis=0)
            v_rows.append({"run": rid, "fixture": i, "command": cmd.tolist(), "mean_v": v.tolist(),
                           "rel_err": float(np.linalg.norm(v - cmd) / np.linalg.norm(cmd))})
        for i in (13, 14, 15):
            k0 = int(round(fixtures[i]["brake_at_s"] / dt)) - 1
            vxy = np.linalg.norm(z["linvel_w"][:, i, :2].astype(np.float64), axis=1)
            after = np.nonzero(vxy[k0:] < 0.05)[0]
            vb = float(vxy[k0])
            model = vb * REACTION + vb * vb / (2 * BRAKE)
            if len(after):
                k1 = k0 + int(after[0])
                dist = float(np.linalg.norm(z["position"][k1, i, :2].astype(np.float64) - z["position"][k0, i, :2]))
                tstop = (k1 - k0) * dt
            else:
                dist, tstop = None, None
            b_rows.append({"run": rid, "fixture": i, "speed_at_brake_mps": vb, "stopping_distance_m": dist,
                           "stopping_time_s": tstop, "riskcap_model_m": model,
                           "mean_decel_mps2": (vb / tstop) if tstop else None,
                           "status": "PASS" if dist is not None and dist <= model else "FAIL"})
        tilt = tilt_deg(z["orientation_xyzw"])
        q = z["orientation_xyzw"].astype(np.float64)
        yaw = np.degrees(np.unwrap(np.arctan2(2 * (q[..., 3] * q[..., 2] + q[..., 0] * q[..., 1]),
                                              1 - 2 * (q[..., 1] ** 2 + q[..., 2] ** 2)), axis=0))
        for i, f in enumerate(fixtures):
            commanded_yaw = any(abs(seg[1][3]) > 0 for seg in f.get("segments", []))
            a_rows.append({"run": rid, "fixture": i, "max_tilt_deg": float(tilt[:, i].max()),
                           "yaw_change_deg": float(yaw[-1, i] - yaw[0, i]), "yaw_rate_commanded": commanded_yaw})
    res["per_fixture"].update(hover=h_rows, velocity=v_rows, braking=b_rows, tilt=a_rows)
    res["gates"]["D1-H1"] = status(all(abs(r["rel_err"]) <= 0.01 for r in h_rows))
    res["gates"]["D1-V1"] = status(all(r["rel_err"] <= 0.05 for r in v_rows),
                                   "max rel err %.4f" % max(r["rel_err"] for r in v_rows))
    res["gates"]["D1-B1"] = status(all(r["status"] == "PASS" for r in b_rows))
    max_tilt = max(r["max_tilt_deg"] for r in a_rows)
    res["gates"]["D1-A1"] = status(max_tilt <= 45.0, "max tilt %.2f deg" % max_tilt)
    drift = [r for r in a_rows if not r["yaw_rate_commanded"] and "contact" not in fixtures[r["fixture"]]]
    res["yaw_drift_without_yaw_command"] = {"status": "CHARACTERISATION",
                                            "max_abs_deg": max(abs(r["yaw_change_deg"]) for r in drift),
                                            "worst": max(drift, key=lambda r: abs(r["yaw_change_deg"]))}
    # Contact.
    k_rows, k_ok, k_any = [], True, False
    for rid, (z, meta) in runs.items():
        dt, n_sub = meta["dt_s"], meta["substeps_per_action"]
        for i, f in enumerate(fixtures):
            if "contact" not in f:
                continue
            bar = meta["contact_bars"][str(i)]
            c2, h2 = np.array(bar["centre"]), np.array(bar["half"])
            pos, quat = z["position"][:, i].astype(np.float64), z["orientation_xyzw"][:, i].astype(np.float64)
            sep = np.array([sat_separation(pos[k], quat_to_rot(quat[k]), BOX_HALF, c2, h2) for k in range(len(pos))])
            force = np.linalg.norm(z["contact_force"][:, i].astype(np.float64), axis=1)
            kg = np.nonzero(sep <= CONTACT_SEP)[0]
            kc = np.nonzero(force > FORCE_THR)[0]
            row = {"run": rid, "fixture": i, "name": f["name"], "min_separation_m": float(sep.min())}
            if len(kc) == 0 and len(kg) == 0:
                row["status"] = "NOT_TESTED"
                row["note"] = "no contact occurred"
            elif len(kc) == 0 or len(kg) == 0:
                k_any = True
                row["status"] = "FAIL"
                row["note"] = "force %s, geometric %s" % (kc[:1].tolist(), kg[:1].tolist())
                k_ok = False
            else:
                k_any = True
                kg0, kc0 = int(kg[0]), int(kc[0])
                v = float(np.linalg.norm(z["linvel_w"][max(kg0 - 1, 0), i].astype(np.float64)))
                lo = kg0 - 1 - math.ceil(CONTACT_SEP / max(v * dt, 1e-9))
                ok = lo <= kc0 <= kg0 + 1
                k_ok &= ok
                a_c = int(z["action_index"][kc0])
                after = min(kc0 + int(round(0.5 / dt)), len(pos) - 1)
                row.update(status="PASS" if ok else "FAIL", first_geometric_substep=kg0, first_force_substep=kc0,
                           window=[lo, kg0 + 1], t_contact_s=(kc0 + 1) * dt,
                           speed_before_contact_mps=float(np.linalg.norm(z["linvel_w"][max(kc0 - 1, 0), i])),
                           speed_at_contact_mps=float(np.linalg.norm(z["linvel_w"][kc0, i])),
                           speed_0p5s_after_mps=float(np.linalg.norm(z["linvel_w"][after, i])),
                           tilt_at_contact_deg=float(tilt_deg(quat[kc0])), tilt_0p5s_after_deg=float(tilt_deg(quat[after])),
                           max_tilt_after_deg=float(tilt_deg(quat[kc0:]).max()),
                           peak_force_N=float(force.max()),
                           crash_detection_delay_s=float((a_c + 1) * PLAN["D1"]["action_period_s"] - (kc0 + 1) * dt))
            k_rows.append(row)
    res["per_fixture"]["contact"] = k_rows
    res["gates"]["D1-K1"] = status(k_ok, "") if k_any else not_tested("no contact in any fixture")
    if any(r["status"] == "NOT_TESTED" for r in k_rows):
        res["gates"]["D1-K1"]["note"] = "fixtures without contact: %s" % sorted({r["fixture"] for r in k_rows
                                                                                 if r["status"] == "NOT_TESTED"})
    spread = {}
    for i in (20, 21, 22, 23):
        ts = [r.get("t_contact_s") for r in k_rows if r["fixture"] == i and r.get("t_contact_s") is not None]
        spread[str(i)] = (max(ts) - min(ts)) if len(ts) > 1 else None
    res["gates"]["D1-K2"] = {"status": "CHARACTERISATION", "first_contact_time_spread_s": spread}
    return res


# ------------------------------------------------------------------ D2 references
CAM_W, CAM_H = 160, 90
OBS_W, OBS_H = 40, 24
HFOV, VFOV = math.radians(87.0), math.radians(58.0)


def obstacle_ray(u, v):
    fx = OBS_W / (2 * math.tan(HFOV / 2))
    fy = OBS_H / (2 * math.tan(VFOV / 2))
    d = np.array([1.0, -(u - (OBS_W - 1) / 2) / fx, -(v - (OBS_H - 1) / 2) / fy])
    return d / np.linalg.norm(d)


def boxes_rel(layout_env, drone):
    out = []
    for b in layout_env:
        c, h = np.array(b["centre"]) - drone, np.array(b["half"])
        out.append((c - h, c + h))
    return out


def first_hit(eye, d, target, radius, boxes, far):
    """(object, t): object 'T' target, 'B' bar or None; target only if no bar is nearer."""
    tt = geo.ray_sphere(eye, d, target, radius) if target is not None else math.inf
    tb = min((geo.ray_box(eye, d, lo, hi) for lo, hi in boxes), default=math.inf)
    if tt < tb and tt < far:
        return "T", tt
    if tb < far:
        return "B", tb
    return None, far


def lidar_rays():
    az = [math.radians(180.0 - 355.0 * j / 71.0) for j in range(72)]
    el = [math.radians(20.0 - 30.0 * i / 3.0) for i in range(4)]
    return az, el


def lidar_dir(a, e):
    return np.array([math.cos(a) * math.cos(e), math.sin(a) * math.cos(e), math.sin(e)])


def analyse_d2(run):
    d = Path(run) / "d2_scenes"
    res = {"gates": {}, "per_scene": []}
    if not (d / "d2_renders.npz").exists():
        for g in PLAN["D2"]["gates"]:
            res["gates"][g] = not_tested("D2 output missing")
        return res
    z = np.load(d / "d2_renders.npz")
    meta = json.loads((d / "d2_meta.json").read_text())
    from motar_eval import occlusion_geometry as og

    scenes = PLAN["D2"]["scenes"]
    drone = z["drone_env_m"].astype(np.float64)
    targets = z["target_world"].astype(np.float64)
    present = z["target_present"]
    eye_off = np.array(geo.CAM_T)
    r1 = {"mask_mismatch": 0, "robust_pixels": 0, "depth_max_err": 0.0, "depth_over_1e-4": 0, "worst": None}
    r2 = {"robust_rays": 0, "max_err": 0.0}
    r3 = {"robust_pixels": 0, "within_1e-4": 0, "over_0p05": 0, "max_err": 0.0}
    l1 = {"robust_beams": 0, "max_err": 0.0, "mismatch": 0, "worst": None}
    f1_errs, a1_rows, o1_rows, s1_rows, n1_rows, n2_rows = [], [], [], [], [], []
    for i, s in enumerate(scenes):
        rel_t = targets[i] - drone[i]
        t_eye = rel_t if present[i] else None
        boxes = boxes_rel(meta["layout"][i], drone[i])
        mask = z["level_scene__target_mask"][i]
        tdep = z["level_scene__target_depth"][i]
        depth = z["level_scene__depth"][i]
        odep = z["level_scene__obstacle_depth_native"][i]
        lidar = z["level_scene__lidar_m"][i].reshape(4, 72)
        # R1 / R3 per pixel (pixel-centre rays, +-0.01 px robustness).
        for v in range(CAM_H):
            for u in range(CAM_W):
                verdicts = set()
                for du, dv in ((0, 0), (0.01, 0), (-0.01, 0), (0, 0.01), (0, -0.01)):
                    obj, _ = first_hit(eye_off, geo.pixel_ray(u + du, v + dv), t_eye, 0.15, boxes, 20.0)
                    verdicts.add(obj)
                if len(verdicts) != 1:
                    continue
                obj, t = first_hit(eye_off, geo.pixel_ray(u, v), t_eye, 0.15, boxes, 20.0)
                r1["robust_pixels"] += 1
                r1["mask_mismatch"] += int((obj == "T") != bool(mask[v, u]))
                if obj == "T" and mask[v, u]:
                    e_t = abs(float(tdep[v, u]) - t)
                    r1["depth_over_1e-4"] += int(e_t > 1e-4)
                    if e_t > r1["depth_max_err"]:
                        r1["depth_max_err"] = e_t
                        r1["worst"] = {"scene": s["id"], "pixel": [u, v], "range_m": t,
                                       "target_distance_m": float(np.linalg.norm(rel_t - eye_off))}
                ref = t if obj == "T" else min(t if obj == "B" else 10.0, 10.0)
                e = abs(float(depth[v, u]) - ref)
                r3["robust_pixels"] += 1
                r3["within_1e-4"] += int(e <= 1e-4)
                r3["over_0p05"] += int(e > 0.05)
                r3["max_err"] = max(r3["max_err"], e)
        # R2 native obstacle grid.
        for v in range(OBS_H):
            for u in range(OBS_W):
                hits = set()
                for du, dv in ((0, 0), (0.01, 0), (-0.01, 0), (0, 0.01), (0, -0.01)):
                    obj, _ = first_hit(eye_off, obstacle_ray(u + du, v + dv), None, 0.0, boxes, 10.0)
                    hits.add(obj)
                if len(hits) != 1:
                    continue
                _, t = first_hit(eye_off, obstacle_ray(u, v), None, 0.0, boxes, 10.0)
                r2["robust_rays"] += 1
                r2["max_err"] = max(r2["max_err"], abs(float(odep[v, u]) - t))
        # L1 LiDAR (body origin, yaw-only, target r = 0.20).
        az, el = lidar_rays()
        origin = np.zeros(3)
        for a_i, e in enumerate(el):
            for j, a in enumerate(az):
                hits = set()
                for da, de in ((0, 0), (1e-5, 0), (-1e-5, 0), (0, 1e-5), (0, -1e-5)):
                    obj, _ = first_hit(origin, lidar_dir(a + da, e + de), t_eye, 0.20, boxes, 12.0)
                    hits.add(obj)
                if len(hits) != 1:
                    continue
                _, t = first_hit(origin, lidar_dir(a, e), t_eye, 0.20, boxes, 12.0)
                err = abs(float(lidar[a_i, j]) - t)
                l1["robust_beams"] += 1
                if err > l1["max_err"]:
                    l1["max_err"] = err
                    l1["worst"] = {"scene": s["id"], "row": a_i, "beam": j, "range_m": t}
                l1["mismatch"] += int(err > 1e-4)
        # F1 fusion: production fused scan (row 2, 0 deg) vs analytic body-origin horizontal bar range.
        fused = z["level_scene__observation"][i][:288].reshape(4, 72).astype(np.float64) * 12.0
        if boxes:
            for j, a in enumerate(az):
                if abs(a) > HFOV / 2:
                    continue
                obj, t = first_hit(origin, lidar_dir(a, 0.0), None, 0.0, boxes, 12.0)
                f1_errs.append({"scene": s["id"], "beam_deg": math.degrees(a), "fused_m": float(fused[2, j]),
                                "analytic_m": t, "lidar_row0deg_m": float(lidar[2, j]), "error_m": float(fused[2, j]) - t})
        # A1 camera measurement.
        if s["id"] in ("S00", "S01", "S02", "S03", "S22") and z["level_scene__det_visible"][i]:
            mv = z["level_scene__det_measurement_vehicle"][i].astype(np.float64)
            true_dir = (rel_t - eye_off) / np.linalg.norm(rel_t - eye_off)
            meas_dir = (mv - eye_off) / np.linalg.norm(mv - eye_off)
            ang = math.acos(min(1.0, float(true_dir @ meas_dir)))
            a1_rows.append({"scene": s["id"], "direction_error_px": ang * geo.FX,
                            "range_error_m": float(np.linalg.norm(mv - eye_off) - np.linalg.norm(rel_t - eye_off))})
        # O1 / S1 classification.
        if present[i]:
            sil = geo.silhouette(rel_t, boxes, sub=8)
            alone_px = int(z["level_alone__target_mask"][i].sum())
            rendered_px = int(mask.sum())
            out_of_range = float(np.linalg.norm(rel_t - eye_off)) - 0.15 >= 20.0
            ref_label = "OUT_OF_RANGE" if out_of_range and sil["in_fov"] else geo.reference_label(sil)
            centres = np.array([(lo + hi) / 2 for lo, hi in boxes]).reshape(-1, 3)
            halves = np.array([(hi - lo) / 2 for lo, hi in boxes]).reshape(-1, 3)
            los = og.line_of_sight(eye_off[None], rel_t[None], centres, halves, radius=0.15, rim_count=8)
            in_frustum = (rel_t[0] > 0 and abs(math.atan2(rel_t[1] - eye_off[1], rel_t[0] - eye_off[0])) <= HFOV / 2
                          and abs(math.atan2(rel_t[2] - eye_off[2], rel_t[0] - eye_off[0])) <= VFOV / 2
                          and float(np.linalg.norm(rel_t - eye_off)) <= 20.0)
            cur = {"in_fov": bool(in_frustum), "visible_pixels": rendered_px, "centre_blocked": bool(los["center_blocked"][0])}
            cur_label = geo.current_label(cur, bool(los["geometric_full_occlusion"][0]))
            o1_rows.append({"scene": s["id"], "reference": ref_label, "current": cur_label, "agree": ref_label == cur_label,
                            "rendered_pixels": rendered_px, "alone_rendered_pixels": alone_px,
                            "analytic_alone_pixel_centres": sil["alone_pixels"], "analytic_visible_area_px": sil["visible_area_px"],
                            "analytic_alone_area_px": sil["alone_area_px"]})
            if not boxes:
                s1_rows.append({"scene": s["id"], "rendered": alone_px, "analytic": sil["alone_pixels"],
                                "detector_visible": bool(z["level_alone__det_visible"][i]),
                                "ok": alone_px == sil["alone_pixels"] and bool(z["level_alone__det_visible"][i]) == (alone_px >= 2)})
        else:
            n1_rows.append({"scene": s["id"], "detector_visible": bool(z["level_scene__det_visible"][i])})
            if s.get("association_beam_deg") is not None:
                n2_rows.append({"scene": s["id"], "beam_deg": s["association_beam_deg"],
                                "accepted": int(z["level_scene__assoc_accept"][i])})
        res["per_scene"].append({"scene": s["id"], "bars": len(boxes)})
    res["R1"], res["R2"], res["R3"], res["L1"] = r1, r2, r3, l1
    res["gates"]["D2-R1"] = status(r1["mask_mismatch"] == 0 and r1["depth_max_err"] <= 1e-4,
                                   "%d robust pixels, %d mask mismatches, max depth err %.3g m" % (
                                       r1["robust_pixels"], r1["mask_mismatch"], r1["depth_max_err"]))
    res["gates"]["D2-R2"] = status(r2["max_err"] <= 1e-4, "%d robust rays, max err %.3g m" % (r2["robust_rays"], r2["max_err"]))
    res["gates"]["D2-R3"] = status(r3["within_1e-4"] == r3["robust_pixels"],
                                   "%d / %d robust pixels within 1e-4 m; %d over 0.05 m; max %.3g m" % (
                                       r3["within_1e-4"], r3["robust_pixels"], r3["over_0p05"], r3["max_err"]))
    res["gates"]["D2-L1"] = status(l1["mismatch"] == 0, "%d robust beams, %d over 1e-4 m, max %.3g m" % (
        l1["robust_beams"], l1["mismatch"], l1["max_err"]))
    y_ok = all(np.array_equal(z["%s_%s__%s" % (att, mode, f)], z["level_%s__%s" % (mode, f)])
               for att in ("roll15", "pitch15") for mode in ("scene", "alone")
               for f in ("rgb", "depth", "target_mask", "lidar_m"))
    res["gates"]["D2-Y1"] = status(y_ok, "synthetic yaw-only contract; hardware realism NOT_TESTED (a body-fixed "
                                         "camera at 15 deg pitch would shift the image by fy*tan(15 deg) = %.1f px)" % (
                                             geo.FY * math.tan(math.radians(15))))
    f_max = max((abs(r["error_m"]) for r in f1_errs), default=0.0)
    res["F1_rows"] = f1_errs
    res["gates"]["D2-F1"] = status(f_max <= 1e-4, "%d in-FOV beams in bar scenes, max |error| %.4f m" % (len(f1_errs), f_max))
    res["A1_rows"] = a1_rows
    res["gates"]["D2-A1"] = {"direction": status(all(r["direction_error_px"] <= 0.5 for r in a1_rows) and len(a1_rows) == 5),
                             "range": status(all(abs(r["range_error_m"]) <= 1e-4 for r in a1_rows) and len(a1_rows) == 5)}
    res["O1_rows"] = o1_rows
    dis = [r["scene"] for r in o1_rows if not r["agree"]]
    zero_not_full = [r["scene"] for r in o1_rows
                     if r["rendered_pixels"] == 0 and r["current"] == "FULL" and r["reference"] != "FULL"]
    res["gates"]["D2-O1"] = status(not dis, "disagreements %s; zero rendered pixels read as FULL while the "
                                            "reference is not FULL: %s" % (dis, zero_not_full))
    res["S1_rows"] = s1_rows
    res["gates"]["D2-S1"] = status(all(r["ok"] for r in s1_rows))
    res["N1_rows"], res["N2_rows"] = n1_rows, n2_rows
    res["gates"]["D2-N1"] = status(not any(r["detector_visible"] for r in n1_rows))
    res["gates"]["D2-N2"] = status(not any(r["accepted"] == 1 for r in n2_rows),
                                   "%d of %d target-absent bar returns accepted as the target" % (
                                       sum(r["accepted"] == 1 for r in n2_rows), len(n2_rows)))
    res["gates"]["D2-T1"] = status(all(c["capture_step_counter"] == c["delivery_step_counter"] for c in meta["counters"]))
    return res


# ------------------------------------------------------------------ D3
def analyse_d3(run):
    res = {"gates": {}, "per_n": {}}
    metas = {}
    for p in PLAN["run"]["processes"]:
        if p["batch"] != "D3":
            continue
        f = Path(run) / p["id"] / "d3_meta.json"
        if f.exists():
            metas[p["id"]] = json.loads(f.read_text())
    if len(metas) != 4:
        for g in PLAN["D3"]["gates"]:
            res["gates"][g] = not_tested("D3 processes present: %s" % sorted(metas))
        return res
    agg = {g: True for g in ("D3-F1", "D3-F2", "D3-F3", "D3-F4", "D3-P1", "D3-R1", "D3-T1", "D3-S1")}
    for pid, m in metas.items():
        row = {"num_envs": m["num_envs"], "branch_steps": [], "mismatches": []}
        for r in m["branch_log"]:
            row["branch_steps"].append([r["step"], r["reason"]])
            for kind, gate in (("B_all", "D3-F1"), ("B_perm", "D3-F2"), ("B_subset", "D3-F3")):
                b = r["branches"][kind]
                ok = all(b["rows_equal"]) and all(b["actions_equal"])
                if r["reason"] == "reset_boundary":
                    agg["D3-F4"] &= ok
                else:
                    agg[gate] &= ok
                if not ok:
                    row["mismatches"].append({"step": r["step"], "branch": kind, "columns": b["differing_columns"],
                                              "max_abs_diff": b["max_abs_diff"]})
            pc = r["branches"]["P_control"]
            if pc["in_view_envs"] > 0:
                agg["D3-P1"] &= pc["changed_envs"] > 0
        row["reset_branches"] = sum(1 for r in m["branch_log"] if r["reason"] == "reset_boundary")
        if row["reset_branches"] == 0:
            agg["D3-F4"] = agg["D3-F4"] and False
        frozen = PLAN["frozen"]["rms_digest_sha256"]
        agg["D3-R1"] &= (m["rms_digest_start"] == frozen and m["rms_digest_end"] == frozen
                         and not m["model_training_flag"] and not m["rms_training_flag"])
        # Delivery latency: per step, camera render and observe share the step counter of that step.
        by_step = {}
        for kind, step, counter in m["events"]:
            by_step.setdefault(step, {}).setdefault(kind, []).append(counter)
        lat_ok, checked = True, 0
        for step, ev in by_step.items():
            if step < 0 or "observe" not in ev:
                continue
            cam = ev.get("camera_render", [])
            lid = ev.get("lidar_render", [])
            obs = ev["observe"]
            checked += 1
            if not cam or not lid or cam[-1] != obs[-1] or lid[-1] != obs[-1]:
                lat_ok = False
        row["latency_steps_checked"] = checked
        agg["D3-T1"] &= lat_ok and checked > 0
        agg["D3-S1"] &= m["actor_width"] == 898 and all(str(ROOT) in v for v in m["origins"].values())
        row.update(rms_start=m["rms_digest_start"], rms_end=m["rms_digest_end"], tf32_matmul=m["tf32_matmul"],
                   loop_wall_s=m["loop_wall_s"])
        res["per_n"][pid] = row
    for g, ok in agg.items():
        res["gates"][g] = status(ok)
    res["gates"]["D3-B1"] = status(all(agg[g] for g in ("D3-F1", "D3-F2", "D3-F3", "D3-F4", "D3-R1")),
                                   "N = %s" % sorted(m["num_envs"] for m in metas.values()))
    return res


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--run", required=True)
    parser.add_argument("--out")
    args = parser.parse_args(argv)
    run = Path(args.run)
    report = json.loads((run / "run_report.json").read_text()) if (run / "run_report.json").exists() else {}
    record = {"schema": "motar.eq1d-analysis.v1", "run": str(run), "run_status": report.get("status"),
              "D1": analyse_d1(run), "D2": analyse_d2(run), "D3": analyse_d3(run)}
    summary = {}
    for batch in ("D1", "D2", "D3"):
        for g, v in record[batch]["gates"].items():
            summary[g] = v["status"] if "status" in v else {k: x["status"] for k, x in v.items()}
    record["summary"] = summary
    if args.out:
        Path(args.out).write_text(json.dumps(record, indent=1, sort_keys=True, default=float) + "\n")
    print(json.dumps(summary, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
