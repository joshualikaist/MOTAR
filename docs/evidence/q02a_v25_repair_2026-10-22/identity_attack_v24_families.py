"""Attack families written by the v2.4 confirmation review (Fable 5.1, scratch attack_v24.py), committed unchanged
except this docstring and the usage line; run here against the current module (Q02A-v2.5). Read-only on the repository.

Reuses the committed scene machinery (docs/evidence/q02a_v24_repair_2026-10-22/identity_attack.py: box bars
0.4-0.8 m footprint, 2 m / 3 m tall seen from 1 m altitude, 0.15 m sphere pasted where it is the nearest surface).

Truth per case (full-res paste pixels on rays through the camera envelope): true_at / true_near / true_far.
FALSE SUPPORTED  = SUPPORTED_BY_FOOTPRINT with true_at == 0 (and paste >= 2 px, else the oracle flag would not be set).
LOST SUPPORTED   = true_at > 0 and not SUPPORTED (on correct tracks this is the loss; on wrong tracks fail-safe).

Usage: python -B docs/evidence/q02a_v25_repair_2026-10-22/identity_attack_v24_families.py <family> <out.json>
"""
import sys, json, math, collections, time
import numpy as np

DOCS = "/home/fair/workspaces/aerial_gym_ws/src/MOTAR-mt-p1/docs/evidence/q02a_v24_repair_2026-10-22"
sys.path.insert(0, DOCS)
import identity_attack as A                      # inserts ROOT and tests into sys.path, imports test_q02_evidence as t
from motar_eval.q02 import geometry as geo, validation as va, evidence as ev

P, CAM, LID, CAM_O = A.P, A.CAM, A.LID, A.CAM_O
Z2, Z3 = A.Z2, A.Z3
HALF_VFOV, HALF_HFOV = math.radians(29.0), math.radians(43.5)


def unit(v):
    v = np.asarray(v, float)
    return v / np.linalg.norm(v)


def run(target_v, boxes=(), cyls=(), track_v=None, cov_pos=0.01, yaw=0.0):
    target_v = np.asarray(target_v, float)
    track_v = target_v if track_v is None else np.asarray(track_v, float)
    depth, lidar, paste, native = A.scene(target_v, boxes, cyls)
    inp = A.t.make(target_v=tuple(track_v), detected=True, depth=depth, lidar=lidar.reshape(-1), cov_pos=cov_pos, yaw=yaw)
    r = ev.estimate(inp)
    rep = va.validate(inp, CAM, LID)
    g = ev._belief_geometry(inp, CAM, LID, P)
    d = np.asarray(inp.depth, float)
    k, tol = P.sigma_k, P.range_tol_m

    def counts(mask):
        v, u = np.nonzero(mask)
        if v.size == 0:
            return 0, 0, 0, 0
        rays = geo.pixel_rays(u, v, CAM)
        inside, entry, exit_, _ = geo.ray_envelope(CAM_O, rays, g["p_v"], g["env_cam"])
        dd = d[v, u]
        lo, hi = entry - k * g["sigma_r"] - tol, exit_ + k * g["sigma_r"] + tol
        return int(inside.sum()), int((inside & (dd >= lo) & (dd <= hi)).sum()), int((inside & (dd < lo)).sum()), int((inside & (dd > hi)).sum())

    t_in, t_at, t_near, t_far = counts(paste)
    f = rep.depth_pasted if rep.depth_pasted is not None else np.zeros_like(paste)
    fn_in, fn_at, fn_near, fn_far = counts(f & ~paste)
    err = track_v - target_v
    n = unit(target_v - CAM_O)
    return dict(hyp=r.hypothesis, ident=r.identity_status, reasons=list(r.reasons), od=rep.status("obstacle_depth"),
                paste=int(paste.sum()), true_in=t_in, true_at=t_at, true_near=t_near, true_far=t_far,
                fnp=int((f & ~paste).sum()), fnp_in=fn_in, fnp_at=fn_at, fnp_near=fn_near, fnp_far=fn_far,
                pasted_in=r.pasted_pixels_in_footprint, pasted_at=r.pasted_pixels_at_track_range,
                pasted_near=r.pasted_pixels_nearer_than_track, range_m=float(g["range"]), sigma_r=float(g["sigma_r"]),
                err_range=float(err @ n), err_lat=float(np.linalg.norm(err - (err @ n) * n)), target_range=float(np.linalg.norm(target_v)))


def front_bar_corner(R, d_f, a, b, pool=Z2, half=0.3, side=-1):
    """Front bar whose near face is d_f before the target range; its top corner (x_f, 0, z1) is offset from the
    target direction by (a, b) target radii in angle. side=-1: bar extends to y<0 (image right). Returns
    (target_v, boxes, dir) or None if infeasible."""
    x_f = R - d_f
    if x_f < 0.35:
        return None
    corner = np.array([x_f, 0.0, pool[1]])
    c = unit(corner - CAM_O)
    th = CAM.target_radius / R
    ey = unit(np.cross([0, 0, 1.0], c)); ez = np.cross(c, ey)       # ey ~ +y (image left), ez ~ up
    dr = unit(c + a * th * ey + b * th * ez)
    tv = CAM_O + R * dr
    el = math.asin(dr[2]); az = math.atan2(dr[1], dr[0])
    if abs(el) > math.radians(26.0) or abs(az) > math.radians(40.0):
        return None
    boxes = [(x_f + half, side * half, half, half, pool[0], pool[1])]
    return tv, boxes, dr


def track_at_face(tv, dr, x_f, delta):
    """Track on the target's direction at the depth of the plane x = x_f (bar near face) + delta."""
    rt = (x_f - CAM_O[0]) / dr[0] + delta
    return CAM_O + rt * dr


def family(name):
    if name == "M1_near_corner":        # wrong NEAR track at a front bar's top corner that hides part of the target
        for R in (1.5, 2.0, 3.0, 4.0, 6.0, 8.0):
            for d_f in (0.6, 1.0, 1.5, 2.5):
                for a in (-0.7, -0.4, 0.0, 0.4, 0.7):
                    for b in (-0.7, -0.4, 0.0, 0.4, 0.7):
                        geom = front_bar_corner(R, d_f, a, b)
                        if geom is None:
                            continue
                        tv, boxes, dr = geom
                        for delta in (-0.3, 0.0, 0.3):
                            for cov in (0.01, 0.04):
                                yield dict(R=R, d_f=d_f, a=a, b=b, delta=delta, cov=cov), dict(
                                    target_v=tv, boxes=boxes, track_v=track_at_face(tv, dr, R - d_f, delta), cov_pos=cov)
    elif name == "M1b_near_corner_far":  # same, small far targets 10-15 m with the bar face inside 10 m
        for R in (10.0, 12.0, 15.0):
            for x_f in (6.0, 8.0, 9.5):
                for a in (-0.5, 0.0, 0.5):
                    for b in (-0.5, 0.0, 0.5):
                        geom = front_bar_corner(R, R - x_f, a, b)
                        if geom is None:
                            continue
                        tv, boxes, dr = geom
                        for delta in (-0.3, 0.0, 0.3):
                            for cov in (0.01, 0.04):
                                yield dict(R=R, x_f=x_f, a=a, b=b, delta=delta, cov=cov), dict(
                                    target_v=tv, boxes=boxes, track_v=track_at_face(tv, dr, x_f, delta), cov_pos=cov)
    elif name == "M2_near_vedge":        # wrong NEAR track at a front bar's vertical edge (sliver)
        for R in (1.5, 2.0, 3.0, 4.0, 6.0, 8.0):
            for d_f in (0.6, 1.0, 1.5, 2.5):
                x_f = R - d_f
                if x_f < 0.35:
                    continue
                for cover in (0.2, 0.4, 0.5, 0.6, 0.8, 0.9):
                    yt = (1 - 2 * cover) * CAM.target_radius        # edge plane y=0 through the camera: covered part y<0
                    tv = np.array([R, yt, CAM_O[2]])
                    dr = unit(tv - CAM_O)
                    boxes = [(x_f + 0.3, -0.3, 0.3, 0.3, Z3[0], Z3[1])]
                    for delta in (-0.3, 0.0, 0.3):
                        for cov in (0.01, 0.04):
                            yield dict(R=R, d_f=d_f, cover=cover, delta=delta, cov=cov), dict(
                                target_v=tv, boxes=boxes, track_v=track_at_face(tv, dr, x_f, delta), cov_pos=cov)
    elif name == "M3_near_hedge":        # wrong NEAR track at a 2 m bar's top edge (lower part of the target hidden)
        for R in (2.0, 3.0, 4.0, 6.0, 8.0):
            for d_f in (0.6, 1.0, 1.5, 2.5):
                x_f = R - d_f
                edge = math.atan2(Z2[1] - CAM_O[2], x_f - CAM_O[0])
                if edge > math.radians(26.0):
                    continue
                th = CAM.target_radius / R
                for cover in (0.2, 0.4, 0.5, 0.6, 0.8, 0.9):
                    ang = edge - (2 * cover - 1) * th
                    dr = unit([math.cos(ang), 0.0, math.sin(ang)])
                    tv = CAM_O + R * dr
                    boxes = [(x_f + 0.4, 0.0, 0.4, 0.4, Z2[0], Z2[1])]
                    for delta in (-0.3, 0.0, 0.3):
                        for cov in (0.01, 0.04):
                            yield dict(R=R, d_f=d_f, cover=cover, delta=delta, cov=cov), dict(
                                target_v=tv, boxes=boxes, track_v=track_at_face(tv, dr, x_f, delta), cov_pos=cov)
    elif name == "M4_far_diag":          # bar behind; track too far AND diagonally offset toward a box corner
        for R in (1.5, 2.0, 3.0, 5.0, 8.0):
            for dR in (0.6, 1.0, 1.5):
                boxes = [A.bar_box(R + dR + 0.3, 0.0, 0.3, Z3)]
                for er in (0.9, 1.2, 1.5):
                    for el, ez in ((0, 0), (0.15, 0.15), (0.25, 0.25), (0.35, 0.35), (0.25, -0.25), (0.4, 0.2), (0.0, 0.35)):
                        for cov in (0.0025, 0.01):
                            tv = np.array([R, 0.0, 0.0])
                            yield dict(R=R, dR=dR, er=er, el=el, ez=ez, cov=cov), dict(
                                target_v=tv, boxes=boxes, track_v=tv + np.array([er, el, ez]), cov_pos=cov)
    elif name == "M5_border_far":        # target clipped at the image border, bar behind, track too far
        for R in (2.0, 4.0, 8.0):
            for bearing, elev in ((42.0, 0.0), (43.0, 0.0), (-42.5, 0.0), (0.0, 28.0), (0.0, -28.0), (40.0, 26.0), (-41.0, -26.5)):
                b, e = math.radians(bearing), math.radians(elev)
                dr = np.array([math.cos(e) * math.cos(b), math.cos(e) * math.sin(b), math.sin(e)])
                tv = CAM_O + R * dr
                for dR in (0.6, 1.2):
                    s = (R + dR) / R
                    bx = CAM_O + (R + dR) * dr
                    boxes = [(bx[0] + 0.3 * dr[0], bx[1] + 0.3 * dr[1], 0.3, 0.3, Z3[0], Z3[1])]
                    for er in (0.9, 1.3):
                        for cov in (0.01, 0.04):
                            yield dict(R=R, bearing=bearing, elev=elev, dR=dR, er=er, cov=cov), dict(
                                target_v=tv, boxes=boxes, track_v=tv + er * dr, cov_pos=cov)
    elif name == "M6_two_behind":        # two bars behind at different ranges split the box; track at either
        for R in (2.0, 3.0, 5.0):
            for d1, d2 in ((0.6, 1.5), (1.0, 2.5), (0.6, 3.0)):
                boxes = [(R + d1 + 0.3, 0.3 + 0.0, 0.3, 0.3, Z3[0], Z3[1]), (R + d2 + 0.3, -0.3, 0.3, 0.3, Z3[0], Z3[1])]
                for er in (d1, d2, d1 + 0.3, d2 + 0.3):
                    for el in (0.0, 0.2, -0.2, 0.35):
                        tv = np.array([R, 0.0, 0.0])
                        yield dict(R=R, d1=d1, d2=d2, er=er, el=el), dict(target_v=tv, boxes=boxes, track_v=tv + np.array([er, el, 0.0]))
    elif name == "M7_small_far_behind":  # 2-6 px paste at 8.5-9.6 m with a bar behind inside 10 m; track too far
        for R in (8.5, 9.0, 9.3, 9.6):
            for dR in (0.2, 0.3, 0.5):
                if R + dR > 9.95:
                    continue
                boxes = [A.bar_box(R + dR + 0.3, 0.0, 0.3, Z3)]
                for er in (0.9, 1.2, 1.5, 2.0):
                    for el in (0.0, 0.2, 0.35):
                        for cov in (0.01, 0.04):
                            tv = np.array([R, 0.0, 0.0])
                            yield dict(R=R, dR=dR, er=er, el=el, cov=cov), dict(target_v=tv, boxes=boxes, track_v=tv + np.array([er, el, 0.0]), cov_pos=cov)
    elif name == "L5_corner_correct":    # CORRECT track; a front bar's top corner hides part of the target
        for R in (1.0, 1.5, 2.0, 3.0, 4.0, 6.0, 8.0):
            for d_f in (0.4, 0.7, 1.0, 1.5, 2.5):
                for a in (-0.7, -0.4, -0.2, 0.0, 0.2, 0.4, 0.7):
                    for b in (-0.7, -0.4, -0.2, 0.0, 0.2, 0.4, 0.7):
                        geom = front_bar_corner(R, d_f, a, b)
                        if geom is None:
                            continue
                        tv, boxes, dr = geom
                        for cov in (0.01, 0.04):
                            yield dict(R=R, d_f=d_f, a=a, b=b, cov=cov, shift=0.0), dict(target_v=tv, boxes=boxes, cov_pos=cov)
                        yield dict(R=R, d_f=d_f, a=a, b=b, cov=0.01, shift=0.3), dict(target_v=tv, boxes=boxes, track_v=tv + 0.3 * dr, cov_pos=0.01)
    elif name == "L6_close_front":       # CORRECT track at 0.45-1.5 m with a front bar edge/corner inside the box
        for R in (0.45, 0.6, 0.8, 1.0, 1.2, 1.5):
            for d_f in (0.2, 0.3, 0.5):
                x_f = R - d_f
                if x_f < 0.2:
                    continue
                for cover in (0.3, 0.6):
                    yt = (1 - 2 * cover) * CAM.target_radius
                    tv = np.array([R, yt, CAM_O[2]])
                    boxes = [(x_f + 0.3, -0.3, 0.3, 0.3, Z3[0], Z3[1])]
                    yield dict(R=R, d_f=d_f, kind="vedge", cover=cover), dict(target_v=tv, boxes=boxes)
                for a, b in ((0.0, 0.0), (0.4, 0.4), (-0.3, 0.3)):
                    geom = front_bar_corner(R, d_f, a, b)
                    if geom is None:
                        continue
                    tv, boxes, dr = geom
                    yield dict(R=R, d_f=d_f, kind="corner", a=a, b=b), dict(target_v=tv, boxes=boxes)
    elif name.startswith("RND"):         # randomized plausible scenes; prior stated in the report
        seed = int(name.split("_")[1]) if "_" in name else 0
        rng = np.random.default_rng(1000 + seed)
        for i in range(1500):
            R = float(rng.uniform(0.6, 9.5))
            az = float(rng.uniform(-35, 35)); el = float(rng.uniform(-18, 18))
            dr = np.array([math.cos(math.radians(el)) * math.cos(math.radians(az)), math.cos(math.radians(el)) * math.sin(math.radians(az)), math.sin(math.radians(el))])
            tv = CAM_O + R * dr
            if not (-0.8 <= tv[2] <= 2.0):
                continue
            nb = int(rng.choice([0, 1, 2, 3], p=[0.15, 0.4, 0.3, 0.15]))
            boxes = []
            tries = 0
            while len(boxes) < nb and tries < 50:
                tries += 1
                s = float(rng.uniform(0.4, min(9.6, R + 3.0)))
                lat = float(rng.uniform(-1.0, 1.0))
                ctr = CAM_O + s * np.array([dr[0], dr[1], 0.0]) / max(math.hypot(dr[0], dr[1]), 1e-6) + lat * np.array([-dr[1], dr[0], 0.0]) / max(math.hypot(dr[0], dr[1]), 1e-6)
                hx, hy = float(rng.uniform(0.2, 0.4)), float(rng.uniform(0.2, 0.4))
                pool = Z2 if rng.random() < 0.5 else Z3
                # non-interpenetrating: target sphere and vehicle clear of the box
                if abs(tv[0] - ctr[0]) < hx + 0.2 and abs(tv[1] - ctr[1]) < hy + 0.2 and pool[0] - 0.2 < tv[2] < pool[1] + 0.2:
                    continue
                if abs(ctr[0]) < hx + 0.5 and abs(ctr[1]) < hy + 0.5:
                    continue
                boxes.append((ctr[0], ctr[1], hx, hy, pool[0], pool[1]))
            mode = rng.choice(["tracker", "assoc"], p=[0.7, 0.3])
            if mode == "assoc" and boxes:
                tb = A.box_hits(CAM_O, dr[None, :], boxes)[0]
                if np.isfinite(tb) and tb < 10.0:
                    rt = tb + float(rng.uniform(-0.2, 0.2))
                    track = CAM_O + rt * dr + np.array([0.0, rng.normal(0, 0.1), rng.normal(0, 0.1)])
                else:
                    mode = "tracker"
            if mode == "tracker":
                err = np.array([rng.normal(0, 0.5), rng.normal(0, 0.25), rng.normal(0, 0.25)])
                err = np.clip(err, -2.0, 2.0)
                track = tv + err
            cov = float(rng.choice([0.0025, 0.01, 0.04]))
            yaw = float(rng.uniform(-math.pi, math.pi))
            yield dict(i=i, R=R, az=az, el=el, nb=len(boxes), mode=str(mode), cov=cov), dict(target_v=tv, boxes=boxes, track_v=track, cov_pos=cov, yaw=yaw)
    else:
        raise SystemExit("unknown family " + name)


def main(name, out):
    t0 = time.time(); rows = []; cnt = collections.Counter()
    for label, kw in family(name):
        res = run(**kw); row = dict(label, **res); rows.append(row)
        cnt["cases"] += 1
        if res["paste"] < 2:
            cnt["no_flag_paste<2"] += 1
            continue
        cnt["flagged_cases"] += 1
        sup = res["ident"] == "SUPPORTED_BY_FOOTPRINT"
        cnt["supported"] += sup
        cnt["true_at>0"] += res["true_at"] > 0
        correct = abs(res["err_range"]) <= 0.3 and res["err_lat"] <= 0.15
        cnt["correct_tracks"] += correct
        if sup and res["true_at"] == 0:
            cnt["FALSE_SUPPORTED"] += 1
            cnt["FALSE_SUPPORTED_near_track" if res["err_range"] < 0 else "FALSE_SUPPORTED_far_track"] += 1
        if (not sup) and res["true_at"] > 0:
            cnt["LOST_correct" if correct else "LOST_wrong_track_failsafe"] += 1
        cnt["ident " + res["ident"]] += 1
    summary = dict(family=name, counts=dict(cnt), seconds=round(time.time() - t0, 1))
    print(json.dumps(summary))
    fs = [r for r in rows if r["ident"] == "SUPPORTED_BY_FOOTPRINT" and r["true_at"] == 0 and r["paste"] >= 2]
    ls = [r for r in rows if r["ident"] != "SUPPORTED_BY_FOOTPRINT" and r["true_at"] > 0 and r["paste"] >= 2
          and abs(r["err_range"]) <= 0.3 and r["err_lat"] <= 0.15]
    for r in fs[:15]:
        print("  FALSE", {k: v for k, v in r.items() if k not in ("reasons",)})
    for r in ls[:15]:
        print("  LOST ", {k: v for k, v in r.items() if k not in ("reasons",)})
    json.dump(dict(summary=summary, false_supported=fs, lost_correct=ls, rows=rows), open(out, "w"), indent=0, default=float)


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
