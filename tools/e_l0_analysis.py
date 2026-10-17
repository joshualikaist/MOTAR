#!/usr/bin/env python3
"""E-L0 analysis (CPU): every preregistered gate of run EL0-R1 against independent oracles.

    python -B tools/e_l0_analysis.py --run DIR --out docs/evidence/.../e_l0_analysis.json

E-L0's labels are never used to validate themselves. Oracles: per-step digests of the other arms
(N gates); pixel-centre analytic geometry from the EQ-1A references with the class rules
re-implemented here (L1); the analytic LiDAR first hit on every row of the beam the association
used (L2); the analytic visible count for camera provenance (L3); the independent EQ-1D substep
recorder and the SAT contact window (C1); the launcher's own per-step outcome flags (C2); the
offline OCC line of sight from the recorded trace layouts (C3); the launcher's action and tracker
logs (T1, T2).
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
for p in (str(ROOT), str(ROOT / "tools"), str(ROOT / "tools/audits")):
    if p not in sys.path:
        sys.path.insert(0, p)
import eq1_geometry_reference as geo  # noqa: E402
import eq1d_analysis as d1ref  # noqa: E402  (SAT separation, EQ-1D D1-K1 window)

PLAN = json.loads((ROOT / "docs/plans/e_l0_evaluation_lineage_v1.json").read_text())
EQ1D = json.loads((ROOT / "docs/plans/eq1d_qualification_v1.json").read_text())
CLASSES = ("NONE", "RANGE_EXIT", "FOV_EXIT", "SCALE_LOSS", "FULL", "CENTRE_ONLY", "PARTIAL", "VISIBLE")
DIGEST_KEYS = ("obs", "action", "pursuer", "target", "done", "captured", "crashed", "resets", "reward",
               "rgb", "depth", "lidar", "semantic", "tracker")
GATE_KEYS = {"E0-N1": ("obs",), "E0-N2": ("action",), "E0-N3": ("pursuer", "target"),
             "E0-N4": ("done", "captured", "crashed", "resets"), "E0-N6": ("rgb", "depth", "lidar", "semantic", "tracker")}
CAM_R, LIDAR_R, FAR, LIDAR_MAX = 0.15, 0.20, 20.0, 12.0
HFOV, VFOV = math.radians(87.0), math.radians(58.0)


def status(ok, note=""):
    return {"status": "PASS" if ok else "FAIL", "note": note}


def not_tested(note):
    return {"status": "NOT_TESTED", "note": note}


def read_jsonl(path):
    with open(path) as fh:
        return [json.loads(line) for line in fh]


# ------------------------------------------------------------------ V3 non-interference
def analyse_noninterference(run):
    groups = {}
    for spec in PLAN["run"]["processes"]:
        if spec["batch"] == "V3":
            groups.setdefault("%s_n%d" % (spec["config"], spec["num_envs"]), {})[spec["arm"]] = spec["id"]
    res, gates = {"groups": {}}, {g: True for g in GATE_KEYS}
    rms_ok, tested = True, 0
    for name, arms in groups.items():
        paths = {arm: Path(run) / pid / "v3_steps.jsonl" for arm, pid in arms.items()}
        if not all(p.exists() for p in paths.values()):
            res["groups"][name] = "MISSING"
            for g in gates:
                gates[g] = False
            continue
        tested += 1
        logs = {arm: read_jsonl(p) for arm, p in paths.items()}
        row = {}
        for a, b in (("P2", "P0"), ("P2", "P1"), ("P1", "P0")):
            la, lb = logs[a], logs[b]
            diff = {k: sum(x.get(k) != y.get(k) for x, y in zip(la, lb)) for k in DIGEST_KEYS}
            row["%s_vs_%s" % (a, b)] = {"steps": min(len(la), len(lb)), "length_equal": len(la) == len(lb),
                                        "differing_steps": diff}
            for g, keys in GATE_KEYS.items():
                gates[g] &= len(la) == len(lb) and all(diff[k] == 0 for k in keys)
        for arm, pid in arms.items():
            meta = json.loads((Path(run) / pid / "v3_meta.json").read_text())
            frozen = EQ1D["frozen"]["rms_digest_sha256"]
            rms_ok &= meta["rms_digest_start"] == frozen and meta["rms_digest_end"] == frozen and \
                not meta["model_training_flag"]
            row.setdefault("arm_state", {})[arm] = meta["arm_state"]
        res["groups"][name] = row
    out = {g: status(ok and tested == len(groups)) for g, ok in gates.items()}
    out["E0-N5"] = status(rms_ok and tested == len(groups))
    return res, out


# ------------------------------------------------------------------ V1 label oracles
def oracle_counts(eye, target, boxes):
    alone = visible = 0
    for v in range(geo.CAM_H):
        for u in range(geo.CAM_W):
            d = geo.pixel_ray(u, v)
            t = geo.ray_sphere(eye, d, target, CAM_R)
            if not math.isfinite(t) or t >= FAR:
                continue
            alone += 1
            if not any(geo.ray_box(eye, d, lo, hi) < t for lo, hi in boxes):
                visible += 1
    return alone, visible


def oracle_class(alone, visible, in_frustum, centre_range, blocked):
    """Class rules of the E-L0 contract, implemented independently of motar_eval.e_l0."""
    if alone == 0:
        if centre_range - CAM_R >= FAR:
            return "RANGE_EXIT"
        return "FOV_EXIT" if not in_frustum else "SCALE_LOSS"
    if visible == 0:
        return "FULL"
    if visible < alone:
        return "CENTRE_ONLY" if blocked else "PARTIAL"
    return "VISIBLE"


def lidar_first_hit(origin, az, el, target, boxes):
    d = np.array([math.cos(az) * math.cos(el), math.sin(az) * math.cos(el), math.sin(el)])
    tt = geo.ray_sphere(origin, d, target, LIDAR_R) if target is not None else math.inf
    tb = min((geo.ray_box(origin, d, lo, hi) for lo, hi in boxes), default=math.inf)
    if min(tt, tb) >= LIDAR_MAX:
        return None, LIDAR_MAX
    return ("TARGET", tt) if tt < tb else ("BAR", tb)


def analyse_v1(run):
    d = Path(run) / "v1_scenes"
    if not (d / "v1_labels.npz").exists():
        return {}, {g: not_tested("V1 missing") for g in ("E0-L1", "E0-L2", "E0-L3")}
    z = np.load(d / "v1_labels.npz")
    meta = json.loads((d / "v1_meta.json").read_text())
    rows, l1_ok, l2_ok, l3_ok = [], True, True, True
    az_table = [math.radians(180.0 - 355.0 * j / 71.0) for j in range(72)]
    el_table = [math.radians(20.0 - 30.0 * i / 3.0) for i in range(4)]
    expected_assoc = {s["id"]: s["oracle"].split(":")[0] for s in PLAN["V1"]["extra_scenes"]}
    for i, sid in enumerate(meta["scenes"]):
        drone = z["drone"][i].astype(np.float64)
        present = bool(z["target_present"][i])
        rel_t = z["target_world"][i].astype(np.float64) - drone
        boxes = [(np.array(b["centre"]) - drone - np.array(b["half"]), np.array(b["centre"]) - drone + np.array(b["half"]))
                 for b in meta["layout"][i]]
        eye = np.array(geo.CAM_T)
        row = {"scene": sid, "present": present}
        cls = CLASSES[int(z["cls"][i])]
        if present:
            alone, visible = oracle_counts(eye, rel_t, boxes)
            rr = rel_t - eye
            dist = float(np.linalg.norm(rr))
            in_fr = rr[0] > 0 and abs(rr[1]) <= rr[0] * math.tan(HFOV / 2) and abs(rr[2]) <= rr[0] * math.tan(VFOV / 2) \
                and dist <= FAR
            blocked = any(geo.ray_box(eye, rr / dist, lo, hi) < dist for lo, hi in boxes)
            ocls = oracle_class(alone, visible, in_fr, dist, blocked)
            ok = (ocls == cls and alone == int(z["alone_px"][i]) and visible == int(z["visible_px"][i])
                  and bool(blocked) == bool(z["centre_blocked"][i]) and bool(in_fr) == bool(z["centre_in_frustum"][i]))
            l1_ok &= ok
            row.update(e_l0_class=cls, oracle_class=ocls, e_l0_alone=int(z["alone_px"][i]), oracle_alone=alone,
                       e_l0_visible=int(z["visible_px"][i]), oracle_visible=visible, centre_blocked=bool(z["centre_blocked"][i]),
                       oracle_centre_blocked=bool(blocked), label_ok=ok)
            # Camera provenance oracle (analytic visible count, production detection flag).
            cv = bool(z["camera_visible"][i])
            ocp = ("TRUE_TARGET" if cv and visible >= 1 else "FALSE_DETECTION" if cv else
                   "MISSED" if visible >= int(meta["min_pixels"]) else "NONE")
        else:
            cv = bool(z["camera_visible"][i])
            ocp = "FALSE_DETECTION" if cv else "NONE"
        ecp = ("NONE", "TRUE_TARGET", "FALSE_DETECTION", "MISSED")[int(z["camera_prov"][i])]
        l3_ok &= ecp == ocp and ecp != "FALSE_DETECTION"
        row.update(camera_prov=ecp, oracle_camera_prov=ocp)
        # LiDAR provenance oracle on the beam the association used.
        label = ("NONE", "CORRECT", "FALSE_ASSOCIATION")[int(z["assoc_label"][i])]
        valid = bool(z["assoc_valid"][i])
        if valid:
            beam = int(z["assoc_beam"][i])
            hits = [lidar_first_hit(np.zeros(3), az_table[beam], e, rel_t if present else None, boxes) for e in el_table]
            obj, rng = min(hits, key=lambda h: h[1])
            olabel = "CORRECT" if obj == "TARGET" else "FALSE_ASSOCIATION"
            row.update(oracle_first_hit=obj, oracle_range_m=rng, e_l0_surface_m=float(z["assoc_surface"][i]))
        else:
            olabel = "NONE"
        ok2 = label == olabel and bool(z["assoc_consistent"][i])
        if not present and valid:
            ok2 &= label == "FALSE_ASSOCIATION"
        if sid in expected_assoc:
            row["planned_oracle"] = expected_assoc[sid]
        l2_ok &= ok2
        row.update(assoc_valid=valid, e_l0_lidar_prov=label, oracle_lidar_prov=olabel)
        rows.append(row)
    gates = {"E0-L1": status(l1_ok, "%d target scenes" % sum(r["present"] for r in rows)),
             "E0-L2": status(l2_ok, "%d accepted associations" % sum(r["assoc_valid"] for r in rows)),
             "E0-L3": status(l3_ok)}
    return {"rows": rows}, gates


# ------------------------------------------------------------------ V2 contact
def analyse_v2(run):
    d = Path(run) / "v2_contact"
    if not (d / "v2_contact.npz").exists():
        return {}, {"E0-C1": not_tested("V2 missing")}
    z = np.load(d / "v2_contact.npz")
    meta = json.loads((d / "v2_meta.json").read_text())
    n_sub, thr = meta["substeps_per_action"], meta["threshold_n"]
    force = np.linalg.norm(z["contact_force"].astype(np.float64), axis=2)   # [S, N]
    hit = force > thr
    actions = z["e_l0_count"].shape[0]
    rows, ok = [], True
    for i in range(force.shape[1]):
        per = hit[: actions * n_sub, i].reshape(actions, n_sub)
        count = per.sum(axis=1)
        first = np.where(per.any(axis=1), per.argmax(axis=1), -1)
        eq_count = bool(np.array_equal(count, z["e_l0_count"][:, i]))
        eq_first = bool(np.array_equal(first, z["e_l0_first"][:, i]))
        row = {"fixture": i, "counts_equal": eq_count, "first_equal": eq_first, "contact_substeps": int(count.sum())}
        bar = meta["contact_bars"].get(str(i))
        if bar is None:
            row["non_contact_zero"] = int(count.sum()) == 0
            ok &= row["non_contact_zero"]
        elif count.sum() > 0:
            pos = z["position"][:, i].astype(np.float64)
            quat = z["orientation_xyzw"][:, i].astype(np.float64)
            sep = np.array([d1ref.sat_separation(pos[k], d1ref.quat_to_rot(quat[k]), d1ref.BOX_HALF,
                                                 np.array(bar["centre"]), np.array(bar["half"])) for k in range(len(pos))])
            kg = int(np.nonzero(sep <= d1ref.CONTACT_SEP)[0][0]) if (sep <= d1ref.CONTACT_SEP).any() else None
            a0 = int(np.nonzero(z["e_l0_count"][:, i])[0][0])
            kc = a0 * n_sub + int(z["e_l0_first"][a0, i])
            v = float(np.linalg.norm(z["linvel_w"][max((kg or 1) - 1, 0), i]))
            lo = (kg - 1 - math.ceil(d1ref.CONTACT_SEP / max(v * 0.01, 1e-9))) if kg is not None else None
            row.update(e_l0_first_substep=kc, geometric_substep=kg,
                       in_window=kg is not None and lo <= kc <= kg + 1)
            ok &= row["in_window"]
        else:
            row["note"] = "no contact (as EQ-1D fixture 22)"
        ok &= eq_count and eq_first
        rows.append(row)
    return {"rows": rows}, {"E0-C1": status(ok)}


# ------------------------------------------------------------------ V3 E-L0 records
def analyse_v3_records(run):
    from motar_eval import e_l0 as el0
    from motar_eval import occlusion_geometry as og
    from motar_eval import occlusion_trace as ot
    from motar_eval import geometry as g

    import torch

    gates = {k: True for k in ("E0-C2", "E0-C3", "E0-T1", "E0-T2")}
    res = {}
    any_p2 = False
    for spec in PLAN["run"]["processes"]:
        if spec["batch"] != "V3" or spec["arm"] != "P2":
            continue
        d = Path(run) / spec["id"]
        if not (d / "e_l0_summary.json").exists():
            for k in gates:
                gates[k] = False
            continue
        any_p2 = True
        r = el0.read_frames(d)
        frames, episodes, steps = r["frames"], r["episodes"], r["steps"]
        tr = np.load(d / "v3_trajectory.npz")
        logs = read_jsonl(d / "v3_steps.jsonl")
        n = spec["num_envs"]
        row = {"frames": len(frames), "episodes": len(episodes)}
        # T1: counters and actions.
        counters_ok = all(s["staged_counter"] == s["consumed_counter"] for s in steps)
        act = frames["action"].reshape(-1, n, 4)
        actions_ok = act.shape[0] == tr["action"].shape[0] and np.array_equal(act.astype(np.float32),
                                                                             tr["action"].astype(np.float32))
        gates["E0-T1"] &= counters_ok and actions_ok
        row.update(counters_aligned=counters_ok, actions_equal=actions_ok)
        # T2: tracker age and expiries.
        age = frames["track_age_s"].reshape(-1, n)
        launcher_age = tr["track_age"][: age.shape[0]].astype(np.float32)
        age_ok = np.array_equal(age.astype(np.float32), launcher_age)
        exp = frames["expired"].reshape(-1, n).astype(bool)
        active = tr["track_active"].astype(bool)
        bad_exp = 0
        for k, e in zip(*np.nonzero(exp)):
            reset_prev = k >= 1 and int(e) in {x for rr in logs[k - 1]["resets"] for x in rr}
            if not (k >= 1 and active[k - 1, e] and not active[k, e] and not reset_prev):
                bad_exp += 1
        gates["E0-T2"] &= age_ok and bad_exp == 0
        row.update(track_age_equal=age_ok, expiries=int(exp.sum()), expiries_inconsistent=bad_exp)
        # C2: outcomes against the launcher's per-step flags.
        launcher = {e: [] for e in range(n)}
        for k in range(tr["done"].shape[0]):
            for e in np.nonzero(tr["done"][k])[0]:
                outcome = "capture" if tr["captured"][k, e] else "crash" if tr["crashed"][k, e] else "timeout"
                launcher[int(e)].append(outcome)
        mine = {e: [x["outcome"] for x in sorted([x for x in episodes if x["env_index"] == e],
                                                 key=lambda x: x["env_episode_index"])] for e in range(n)}
        outcomes_ok = mine == launcher
        cf_ok = all((x["collision_free_capture"] == (x["outcome"] == "capture" and x["contact_substeps"] == 0)) and
                    (x["capture_with_contact"] == (x["outcome"] == "capture" and x["contact_substeps"] > 0)) for x in episodes)
        cause_ok = all(x["crash_causes"] for x in episodes if x["outcome"] == "crash")
        abort_ok = all(x["safe_abort"] == "NOT_IMPLEMENTED" for x in episodes)
        gates["E0-C2"] &= outcomes_ok and cf_ok and cause_ok and abort_ok
        row.update(outcomes_equal=outcomes_ok, collision_free_consistent=cf_ok, crash_causes_present=cause_ok,
                   safe_abort_not_implemented=abort_ok,
                   outcome_counts={o: sum(x["outcome"] == o for x in episodes) for o in ("capture", "crash", "timeout")},
                   capture_with_contact=sum(x["capture_with_contact"] for x in episodes),
                   collision_free_capture=sum(x["collision_free_capture"] for x in episodes),
                   crash_causes={c: sum(c in x["crash_causes"] for x in episodes) for c in
                                 ("contact", "out_of_bounds", "below_height", "above_height", "target_contact")},
                   lidar_correct=int((frames["lidar_prov"] == 1).sum()), lidar_false=int((frames["lidar_prov"] == 2).sum()),
                   camera_false_detections=int((frames["camera_prov"] == 2).sum()),
                   class_frames={c: int((frames["cls"] == k).sum()) for k, c in enumerate(CLASSES) if (frames["cls"] == k).any()},
                   visible_outside_alone_max=int(frames["visible_outside_alone_px"].max()))
        # C3: online centre ray versus offline OCC LOS on in-frustum frames.
        trace = ot.read_trace(d)
        tf, layouts = trace["frames"], trace["layouts"]
        lookup = ot.reset_lookup(layouts)
        key = {(int(a), int(b), int(c)): k for k, (a, b, c) in enumerate(zip(frames["env"], frames["episode"], frames["frame"]))}
        agree = total = 0
        by_layout = {}
        for rec in tf:
            k = key.get((int(rec["env"]), int(rec["episode"]), int(rec["frame"])))
            if k is None or not frames["centre_in_frustum"][k]:
                continue
            by_layout.setdefault(lookup[(int(rec["env"]), int(rec["episode"]))], []).append((rec, k))
        for lay, items in by_layout.items():
            centres, halves, quats = ot.layout_arrays(layouts, lay)
            pos = torch.tensor(np.stack([x[0]["p_pos"] for x in items]), dtype=torch.float64)
            vq = torch.tensor(np.stack([x[0]["p_vehicle_quat"] for x in items]), dtype=torch.float64)
            eye = g.camera_eye(pos, vq, (0.10, 0.0, 0.03)).numpy()
            tgt = np.stack([x[0]["t_pos"] for x in items]).astype(np.float64)
            los = og.line_of_sight(eye, tgt, centres.astype(np.float64), halves.astype(np.float64), quats, radius=CAM_R,
                                   rim_count=0)
            online = np.array([bool(frames["centre_blocked"][x[1]]) for x in items])
            agree += int((los["center_blocked"] == online).sum())
            total += len(items)
        frac = agree / total if total else 0.0
        gates["E0-C3"] &= total > 0 and frac >= 0.995
        row.update(centre_ray_frames=total, centre_ray_agreement=frac)
        res[spec["id"]] = row
    if not any_p2:
        return res, {k: not_tested("no P2 records") for k in gates}
    return res, {k: status(ok) for k, ok in gates.items()}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--run", required=True)
    parser.add_argument("--out")
    args = parser.parse_args(argv)
    run = Path(args.run)
    report = json.loads((run / "run_report.json").read_text()) if (run / "run_report.json").exists() else {}
    ni, ni_g = analyse_noninterference(run)
    v1, v1_g = analyse_v1(run)
    v2, v2_g = analyse_v2(run)
    v3, v3_g = analyse_v3_records(run)
    gates = {**ni_g, **v1_g, **v2_g, **v3_g}
    record = {"schema": "motar.e-l0-analysis.v1", "run": str(run), "run_status": report.get("status"),
              "noninterference": ni, "V1": v1, "V2": v2, "V3": v3, "gates": gates,
              "summary": {k: v["status"] for k, v in sorted(gates.items())}}
    if args.out:
        Path(args.out).write_text(json.dumps(record, indent=1, sort_keys=True, default=float) + "\n")
    print(json.dumps(record["summary"], indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
