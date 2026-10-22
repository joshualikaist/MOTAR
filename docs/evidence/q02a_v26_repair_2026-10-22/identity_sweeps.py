"""Identity-gate sweeps for Q02A-v2.6 and the v2.5 baseline on identical scenes (CPU, read-only on the repository).

Scene generators follow the Opus final review of v2.5 (docs/evidence/q02a_v25_opus_final_review_2026-10-22/probes:
p11_corner_cls.py, p12_random_cls.py, p8b_boxtarget.py, tl.py): 0.15 m sphere target pasted where it is the nearest
surface, bars as (yawed) boxes 0.4-0.8 m wide, 2 m / 3 m tall seen from 1 m altitude; box target = the physical-dynamics
oriented box (0.14/0.14/0.06 m half extents). Truth (evaluation only) = true target pixels on envelope rays inside the
track's range interval. Each scene is fitted once; each track is decided from that fit exactly as `estimate` does
(evidence._footprint_decision), and every ~300th track is cross-checked against the full `estimate` (mismatch -> abort).

Usage: python -B identity_sweeps.py <family> <v26|v25> <out.json> [seed] [n_scenes]
families: corner_z2, corner_z3 (fixed grid), random (seeded), box (seeded, box target, SPHERE declared).
"""
import collections, importlib, json, math, subprocess, sys, time
from pathlib import Path
from types import SimpleNamespace

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT)); sys.path.insert(0, str(ROOT / "docs/evidence/q02a_v25_repair_2026-10-22"))
import identity_attack_v24_families as M                         # noqa: E402  (scene machinery, committed)
from motar_eval.q02 import evidence as ev, geometry as geo, validation as va   # noqa: E402

A, CAM, LID, CAM_O, P, T = M.A, M.CAM, M.LID, M.CAM_O, M.P, M.A.t
Z2, Z3 = M.Z2, M.Z3
NR, PR, LD = A.NRAYS, A.RAYS, A.LDIRS
HE = np.array([0.14, 0.14, 0.06])
SUP = "SUPPORTED_BY_FOOTPRINT"


def load_v25(scratch):
    pkg = Path(scratch) / "q02v25"; pkg.mkdir(parents=True, exist_ok=True)
    for n in ("__init__", "contracts", "geometry", "validation", "evidence"):
        (pkg / (n + ".py")).write_text(subprocess.check_output(["git", "-C", str(ROOT), "show", "5fc64a3:motar_eval/q02/%s.py" % n], text=True))
    sys.path.insert(0, str(scratch))
    return importlib.import_module("q02v25.geometry"), importlib.import_module("q02v25.evidence"), importlib.import_module("q02v25.validation")


def obox_hits(origin, dirs, boxes):
    t_all = np.full(dirs.shape[:-1], np.inf); o = np.asarray(origin, float)
    for cx, cy, hx, hy, z0, z1, yw in boxes:
        c, s = math.cos(yw), math.sin(yw)
        rot = np.array([[c, s, 0], [-s, c, 0], [0, 0, 1.0]])
        ol = rot @ (o - np.array([cx, cy, 0.0])); dl = dirs @ rot.T
        lo, hi = np.array([-hx, -hy, z0]), np.array([hx, hy, z1])
        with np.errstate(divide="ignore", invalid="ignore"):
            t1, t2 = (lo - ol) / dl, (hi - ol) / dl
        tmin, tmax = np.max(np.minimum(t1, t2), axis=-1), np.min(np.maximum(t1, t2), axis=-1)
        t_all = np.minimum(t_all, np.where((tmax >= np.maximum(tmin, 0.0)) & (tmin > 0.0), tmin, np.inf))
    return t_all


def obb_t(origin, dirs, centre, yaw, pitch=0.0):
    c, s = math.cos(yaw), math.sin(yaw); rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])
    cp, sp = math.cos(pitch), math.sin(pitch); ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rot = rz @ ry; ol = rot.T @ (np.asarray(origin, float) - centre); dl = dirs @ rot
    with np.errstate(divide="ignore", invalid="ignore"):
        t1, t2 = (-HE - ol) / dl, (HE - ol) / dl
    tmin, tmax = np.max(np.minimum(t1, t2), axis=-1), np.min(np.maximum(t1, t2), axis=-1)
    return np.where(tmax >= np.maximum(tmin, 0.0), np.where(tmin >= 0, tmin, tmax), np.inf)


def scene(target_v, boxes, box_target=None):
    native = np.minimum(obox_hits(CAM_O, NR, boxes), 10.0)
    depth = np.clip(T.AV @ native @ T.AH.T, 0.0, 10.0)
    tt = obb_t(CAM_O, PR, target_v, *box_target) if box_target else geo.sphere_first_hit(CAM_O, PR, target_v, CAM.target_radius)
    paste = np.isfinite(tt) & (tt < obox_hits(CAM_O, PR, boxes)) & (tt <= CAM.max_camera_range)
    depth = np.where(paste, tt, depth)
    tl = np.minimum(obox_hits(np.zeros(3), LD, boxes), geo.sphere_first_hit(np.zeros(3), LD, target_v, LID.target_proxy_radius_m))
    lidar = np.where(tl < 0.2, 0.0, np.where(tl >= 12.0, 12.0, tl))
    return depth.astype(np.float32), lidar.astype(np.float32), paste


class Judge:
    def __init__(self, mode, scratch):
        self.mode = mode
        if mode == "v25":
            self.geo, self.ev, self.va = load_v25(scratch)
        else:
            self.geo, self.ev, self.va = geo, ev, va

    def cache(self, target_v, boxes, yaw=0.0, box_target=None):
        depth, lidar, paste = scene(np.asarray(target_v, float), boxes, box_target)
        inp0 = T.make(target_v=tuple(target_v), detected=True, depth=depth, lidar=lidar.reshape(-1), yaw=yaw)
        rep = self.va.validate(inp0, CAM, LID)
        if rep.depth_pasted is None:
            return None
        fit = self.geo.fit_target_sphere(inp0.depth, rep.depth_pasted, CAM, CAM.target_radius)
        return dict(target_v=np.asarray(target_v, float), yaw=yaw, depth=depth, lidar=lidar, paste=paste, rep=rep, fit=fit,
                    d=np.asarray(inp0.depth, float), inp0=inp0, fit_status=fit[2] if self.mode == "v25" else fit.status)

    def decide(self, sc, track_v, cov):
        """Footprint identity_status for a flagged frame (only SUPPORTED vs not is used for the counts)."""
        world = geo.vehicle_to_world(np.asarray(track_v, float), T.POS, sc["yaw"])
        ns = SimpleNamespace(vehicle_quat=T.yaw_quat(sc["yaw"]), track_mean=np.r_[world, 0, 0, 0], track_cov=np.diag([cov] * 3 + [0.1] * 3),
                             position=T.POS, depth=sc["inp0"].depth)
        g = self.ev._belief_geometry(ns, CAM, LID, P)
        if not (g["sigma_r"] <= P.max_range_sigma_m and g["extra"] <= P.max_envelope_extra_m):
            return "NOT_TESTABLE_GATES", g
        if self.mode == "v25":
            c, mask, st, _ = sc["fit"]
            tgt = self.ev._pasted_in_footprint(ns, sc["rep"], CAM, P, g, mask=mask) if st == "OK" else (0, 0, 0)
            return (SUP if tgt[1] > 0 else ("RANGE_CONTRADICTED" if tgt[0] > 0 else "OTHER")), g
        dec = self.ev._footprint_decision(ns, sc["rep"], CAM, P, g, sc["fit"], (0, 0, 0))
        return (dec[2] if dec[0] is not None else "OTHER"), g


def truth(sc, g):
    v, u = np.nonzero(sc["paste"]); rays = geo.pixel_rays(u, v, CAM)
    inside, entry, exit_, _ = geo.ray_envelope(CAM_O, rays, g["p_v"], g["env_cam"])
    dd = sc["d"][v, u]; lo, hi = entry - P.sigma_k * g["sigma_r"] - P.range_tol_m, exit_ + P.sigma_k * g["sigma_r"] + P.range_tol_m
    at = inside & (dd >= lo) & (dd <= hi)
    gap = float(np.min(np.where(dd < lo, lo - dd, dd - hi)[inside])) if (not at.any() and inside.any()) else None
    tv = sc["target_v"]; rng_ = float(np.linalg.norm(tv - CAM_O)); dr = (tv - CAM_O) / rng_
    ci, ce, cx, _ = geo.ray_envelope(CAM_O, dr[None, :], g["p_v"], g["env_cam"])
    centre_in = bool(ci[0] and ce[0] - P.sigma_k * g["sigma_r"] - P.range_tol_m <= rng_ <= cx[0] + P.sigma_k * g["sigma_r"] + P.range_tol_m)
    return int(at.sum()), gap, centre_in


def score_track(J, sc, trk, cov, o, rng, check):
    status, g = J.decide(sc, trk, cov)
    t_at, gap, centre_in = truth(sc, g)
    tv = sc["target_v"]; dr = (tv - CAM_O) / np.linalg.norm(tv - CAM_O)
    err = trk - tv; er = float(err @ dr); el = float(np.linalg.norm(err - er * dr))
    sup = status == SUP
    o["tracks"] += 1; o["status_" + status] += 1
    if sup:
        o["supported"] += 1
        if t_at == 0:
            o["FALSE_SUPPORT"] += 1
            cls = "CENTRE_IN" if centre_in else ("GAP_LE_5CM" if (gap is not None and gap <= 0.05) else "MATERIAL")
            o["FALSE_" + cls] += 1
    if abs(er) <= 0.3 and el <= 0.15 and t_at > 0:
        o["correct_tracks_with_truth"] += 1
        o["correct_supported" if sup else "INCORRECT_REJECTION"] += 1
        if not sup:
            o["rejected_as_" + status] += 1
    if check and J.mode == "v26":
        r = ev.estimate(T.make(target_v=tuple(trk), detected=True, depth=sc["depth"], lidar=sc["lidar"].reshape(-1), cov_pos=cov, yaw=sc["yaw"]))
        assert (r.identity_status == SUP) == sup, (r.identity_status, status)
        o["crosschecks"] += 1
    return o


def sweep(J, sc, o, rng, lats=(0.0, 0.2)):
    tv = sc["target_v"]; R = float(np.linalg.norm(tv - CAM_O)); dr = (tv - CAM_O) / R
    ey = M.unit(np.cross([0, 0, 1.0], dr)); ez = np.cross(dr, ey)
    for rr in np.arange(0.4, 12.01, 0.1):
        for la in lats:
            ang = rng.uniform(0, 2 * math.pi)
            trk = CAM_O + rr * dr + la * (math.cos(ang) * ey + math.sin(ang) * ez)
            for cov in (0.0025, 0.01, 0.04):
                score_track(J, sc, trk, cov, o, rng, check=rng.random() < 0.0033)
    for _ in range(40):                       # tracker-like decisions: range N(0, 1 m), lateral N(0, 0.15 m)
        trk = tv + float(rng.normal(0, 1.0)) * dr + (rng.normal(0, 0.15, 2) @ np.stack([ey, ez]))
        if np.linalg.norm(trk - CAM_O) < 0.3:
            continue
        sub = collections.Counter(); score_track(J, sc, trk, float(rng.choice([0.0025, 0.01, 0.04])), sub, rng, check=False)
        for k, v in sub.items():
            o["TL_" + k] += v


def corner(J, pool, out):
    rng = np.random.default_rng(21 if pool is Z2 else 22); o = collections.Counter()
    for R in ((1.0, 1.5, 2.0, 3.0, 4.0, 6.0, 8.0) if pool is Z2 else (4.0, 5.0, 6.0, 8.0, 10.0)):
        for d_f in (0.4, 0.7, 1.0, 1.5, 2.5, 3.5):
            for a in (-0.85, -0.7, -0.4, -0.2, 0.0, 0.4):
                for b in (-0.85, -0.7, -0.4, -0.2, 0.0, 0.4):
                    gm = M.front_bar_corner(R, d_f, a, b, pool=pool)
                    if gm is None:
                        continue
                    tv, boxes0, _ = gm
                    sc = J.cache(tv, [bx + (0.0,) for bx in boxes0])
                    if sc is None or sc["paste"].sum() < 2:
                        continue
                    o["scenes"] += 1; o["fit_" + str(sc["fit_status"])] += 1
                    before = o["FALSE_MATERIAL"]; sweep(J, sc, o, rng)
                    o["scenes_with_material_false"] += o["FALSE_MATERIAL"] > before
    return o


def random_scenes(J, seed, n):
    rng = np.random.default_rng(7000 + seed); o = collections.Counter(); i = 0
    def ok_box(b, tv):
        cx, cy, hx, hy, z0, z1, yw = b; rad = math.hypot(hx, hy)
        return not (math.hypot(tv[0] - cx, tv[1] - cy) < rad + 0.2 and z0 - 0.2 < tv[2] < z1 + 0.2) and math.hypot(cx, cy) >= rad + 0.4
    while o["scenes"] < n and i < 20 * n:
        i += 1
        R = float(rng.uniform(0.6, 12.0)); az = math.radians(rng.uniform(-38, 38)); el = math.radians(rng.uniform(-20, 20))
        dr = np.array([math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el)]); tv = CAM_O + R * dr
        if not (-0.8 <= tv[2] <= 2.0):
            continue
        boxes = []; hor = np.array([dr[0], dr[1], 0.0]); hor /= np.linalg.norm(hor); side = np.array([-hor[1], hor[0], 0.0])
        for _ in range(int(rng.choice([0, 1, 2], p=[0.2, 0.6, 0.2]))):
            s = float(rng.uniform(0.4, max(0.5, R - 0.3))); hx, hy = float(rng.uniform(0.2, 0.4)), float(rng.uniform(0.2, 0.4))
            yw = float(rng.uniform(0, math.pi / 2)); re = 0.5 * (hx + hy)
            lat = float(rng.choice([-1, 1])) * (re + float(rng.normal(0.0, 0.08)) * s / max(R, 0.5))
            pool = Z2 if rng.random() < 0.5 else Z3; ctr = CAM_O + s * hor + lat * side + re * hor
            b = (float(ctr[0]), float(ctr[1]), hx, hy, pool[0], pool[1], yw)
            if ok_box(b, tv):
                boxes.append(b)
        for _ in range(int(rng.choice([0, 1, 2, 3], p=[0.2, 0.4, 0.3, 0.1]))):
            if boxes and rng.random() < 0.3:
                cx, cy, hx, hy, z0, z1, yw = boxes[int(rng.integers(len(boxes)))]
                ang = rng.uniform(0, 2 * math.pi); dd = float(rng.uniform(0.2, 0.4))
                b = (cx + dd * math.cos(ang), cy + dd * math.sin(ang), float(rng.uniform(0.2, 0.4)), float(rng.uniform(0.2, 0.4)), z0, z1, float(rng.uniform(0, math.pi / 2)))
            else:
                s = float(rng.uniform(0.5, min(11.0, R + 3.0))); lat = float(rng.uniform(-1.2, 1.2)); ctr = CAM_O + s * hor + lat * side
                pool = Z2 if rng.random() < 0.5 else Z3
                b = (float(ctr[0]), float(ctr[1]), float(rng.uniform(0.2, 0.4)), float(rng.uniform(0.2, 0.4)), pool[0], pool[1], float(rng.uniform(0, math.pi / 2)))
            if ok_box(b, tv):
                boxes.append(b)
        yaw = float(rng.uniform(-math.pi, math.pi))
        sc = J.cache(tv, boxes, yaw)
        if sc is None or sc["paste"].sum() < 2:
            continue
        o["scenes"] += 1; o["fit_" + str(sc["fit_status"])] += 1
        before = o["FALSE_MATERIAL"]; sweep(J, sc, o, rng)
        o["scenes_with_material_false"] += o["FALSE_MATERIAL"] > before
    return o


def box_targets(J, seed, n):
    rng = np.random.default_rng(seed); o = collections.Counter()
    for _ in range(n):
        R = float(rng.uniform(0.6, 9.0)); az = math.radians(rng.uniform(-30, 30)); el = math.radians(rng.uniform(-15, 15))
        dr = np.array([math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el)]); tv = CAM_O + R * dr
        hor = np.array([dr[0], dr[1], 0]); hor /= np.linalg.norm(hor); side = np.array([-hor[1], hor[0], 0]); boxes = []
        if rng.random() < 0.7:
            s = float(rng.uniform(0.4, max(0.5, R - 0.3))); h = float(rng.uniform(0.2, 0.4))
            ctr = CAM_O + (s + h) * hor + float(rng.choice([-1, 1])) * (h + rng.normal(0, 0.06) * s / R) * side
            boxes.append((ctr[0], ctr[1], h, h, -1.0, float(rng.choice([1.0, 2.0])), 0.0))
        if rng.random() < 0.5:
            ctr = CAM_O + (R + float(rng.uniform(0.4, 2.0))) * hor + float(rng.uniform(-0.3, 0.3)) * side
            boxes.append((ctr[0], ctr[1], 0.3, 0.3, -1.0, 2.0, 0.0))
        boxes = [b for b in boxes if not (abs(b[0] - tv[0]) < b[2] + 0.25 and abs(b[1] - tv[1]) < b[3] + 0.25)]
        sc = J.cache(tv, boxes, 0.0, box_target=(float(rng.uniform(0, math.pi)), float(rng.uniform(-0.3, 0.3))))
        if sc is None or sc["paste"].sum() < 2:
            continue
        o["scenes"] += 1; o["fit_" + str(sc["fit_status"])] += 1
        before = o["FALSE_MATERIAL"]; sweep(J, sc, o, rng)
        o["scenes_with_material_false"] += o["FALSE_MATERIAL"] > before
    return o


if __name__ == "__main__":
    fam, mode, out = sys.argv[1], sys.argv[2], sys.argv[3]
    seed = int(sys.argv[4]) if len(sys.argv) > 4 else 0; n = int(sys.argv[5]) if len(sys.argv) > 5 else 0
    J = Judge(mode, Path(out).parent / ("pkg_" + mode)); t0 = time.time()
    o = (corner(J, Z2, out) if fam == "corner_z2" else corner(J, Z3, out) if fam == "corner_z3" else
         random_scenes(J, seed, n) if fam == "random" else box_targets(J, seed, n))
    res = dict(family=fam, mode=mode, seed=seed, n=n, counts=dict(sorted(o.items())), seconds=round(time.time() - t0, 1))
    Path(out).write_text(json.dumps(res, indent=1) + "\n"); print(json.dumps(res))
