"""Identity-gate attack families written by the v2.3 final review (Fable 5.1, scratch attack_c1.py), committed
unchanged except this docstring and the output path note; run here against the current module (Q02A-v2.4). Read-only on the repo.

Production-shaped scenes: 0.15 m sphere target; bars as axis-aligned BOXES (footprint 0.4-0.8 m, height 2 m or 3 m,
vehicle at 1.0 m altitude => bar z in [-1, +1] or [-1, +2] in the vehicle frame) or infinite cylinders (test harness).
Camera depth = native 40x24 obstacle render (10 m no-hit) bilinear to 160x90, clamp [0, 10], visible target pixels
pasted with exact ray depth (<= 20 m). LiDAR = first hit, < 0.2 -> 0, >= 12 -> 12.

Truth for identity: true paste pixels whose full-res ray passes the camera envelope and whose depth lies in the track's
range interval ("true_at"). FALSE SUPPORTED = identity SUPPORTED_BY_FOOTPRINT with true_at == 0.
LOST SUPPORTED = correct track, true_at > 0, identity != SUPPORTED_BY_FOOTPRINT.

Usage: python -B docs/evidence/q02a_v24_repair_2026-10-22/identity_attack.py <family> <out.json>
"""
import sys, json, math, collections, itertools, time
import numpy as np
ROOT = "/home/fair/workspaces/aerial_gym_ws/src/MOTAR-mt-p1"
sys.path.insert(0, ROOT); sys.path.insert(0, ROOT + "/tests")
import test_q02_evidence as t
from motar_eval.q02 import geometry as geo, validation as va, evidence as ev

P = ev.EvidenceParams()
CAM, LID, PROF = t.CAM, t.LID, t.PROF
CAM_O = np.array(CAM.camera_offset)
VV, UU = np.mgrid[0:90, 0:160]
RAYS = geo.pixel_rays(UU, VV, CAM)
NRAYS = geo.native_camera_rays(CAM, PROF.obstacle_native_wh)
LDIRS = geo.lidar_directions(LID)
Z2, Z3 = (-1.0, 1.0), (-1.0, 2.0)      # 2 m / 3 m bar pools seen from 1.0 m flight altitude


def box_hits(origin, dirs, boxes):
    """Axis-aligned boxes (cx, cy, hx, hy, z0, z1) in the vehicle frame; first entry distance, inf if missed."""
    t_all = np.full(dirs.shape[:-1], np.inf)
    o = np.asarray(origin, float)
    for cx, cy, hx, hy, z0, z1 in boxes:
        lo = np.array([cx - hx, cy - hy, z0]); hi = np.array([cx + hx, cy + hy, z1])
        with np.errstate(divide="ignore", invalid="ignore"):
            t1 = (lo - o) / dirs; t2 = (hi - o) / dirs
        tmin = np.max(np.minimum(t1, t2), axis=-1); tmax = np.min(np.maximum(t1, t2), axis=-1)
        hit = (tmax >= np.maximum(tmin, 0.0)) & (tmin > 0.0)
        t_all = np.minimum(t_all, np.where(hit, tmin, np.inf))
    return t_all


def cyl_hits(origin, dirs, cyls):
    """Vertical cylinders (cx, cy, r, z0, z1) with a top/bottom cap; z0/z1 = None -> infinite (test harness)."""
    t_all = np.full(dirs.shape[:-1], np.inf)
    o = np.asarray(origin, float)
    for cx, cy, r, z0, z1 in cyls:
        ox, oy = o[0] - cx, o[1] - cy
        a = dirs[..., 0] ** 2 + dirs[..., 1] ** 2
        b = 2 * (ox * dirs[..., 0] + oy * dirs[..., 1])
        cc = ox * ox + oy * oy - r * r
        disc = b * b - 4 * a * cc
        with np.errstate(invalid="ignore", divide="ignore"):
            th = (-b - np.sqrt(np.clip(disc, 0, None))) / (2 * a)
        side = np.where((disc >= 0) & (th > 0), th, np.inf)
        if z0 is not None:
            z = o[2] + side * dirs[..., 2]
            side = np.where((z >= z0) & (z <= z1), side, np.inf)
            for zc in (z0, z1):
                with np.errstate(divide="ignore", invalid="ignore"):
                    tc = (zc - o[2]) / dirs[..., 2]
                px, py = o[0] + tc * dirs[..., 0] - cx, o[1] + tc * dirs[..., 1] - cy
                cap = np.where((tc > 0) & (px * px + py * py <= r * r), tc, np.inf)
                side = np.minimum(side, cap)
        t_all = np.minimum(t_all, side)
    return t_all


def obstacles(origin, dirs, boxes, cyls):
    return np.minimum(box_hits(origin, dirs, boxes), cyl_hits(origin, dirs, cyls))


def scene(target_v, boxes=(), cyls=()):
    native = np.minimum(obstacles(CAM_O, NRAYS, boxes, cyls), PROF.obstacle_far_m)
    depth = np.clip(t.AV @ native @ t.AH.T, 0.0, PROF.obstacle_far_m)
    paste = np.zeros(depth.shape, bool)
    if target_v is not None:
        tt = geo.sphere_first_hit(CAM_O, RAYS, target_v, CAM.target_radius)
        paste = np.isfinite(tt) & (tt < obstacles(CAM_O, RAYS, boxes, cyls)) & (tt <= CAM.max_camera_range)
        depth = np.where(paste, tt, depth)
    tl = obstacles(np.zeros(3), LDIRS, boxes, cyls)
    if target_v is not None:
        tl = np.minimum(tl, geo.sphere_first_hit(np.zeros(3), LDIRS, target_v, LID.target_proxy_radius_m))
    lidar = np.where(tl < LID.min_range_m, 0.0, np.where(tl >= LID.max_range_m, LID.max_range_m, tl))
    return depth.astype(np.float32), lidar.astype(np.float32), paste, native


def run(target_v, boxes=(), cyls=(), shift=(0., 0., 0.), cov_pos=0.01, yaw=0.0, detected=True):
    target_v = np.asarray(target_v, float)
    depth, lidar, paste, native = scene(target_v, boxes, cyls)
    track_v = target_v + np.asarray(shift, float)
    inp = t.make(target_v=tuple(track_v), detected=detected, depth=depth, lidar=lidar.reshape(-1), cov_pos=cov_pos, yaw=yaw)
    r = ev.estimate(inp)
    rep = va.validate(inp, CAM, LID)
    g = ev._belief_geometry(inp, CAM, LID, P)
    d = np.asarray(inp.depth, float)
    k, tol = P.sigma_k, P.range_tol_m

    def at_range(mask):
        v, u = np.nonzero(mask)
        if v.size == 0:
            return 0, 0
        rays = geo.pixel_rays(u, v, CAM)
        inside, entry, exit_, _ = geo.ray_envelope(CAM_O, rays, g["p_v"], g["env_cam"])
        dd = d[v, u]
        at = inside & (dd >= entry - k * g["sigma_r"] - tol) & (dd <= exit_ + k * g["sigma_r"] + tol)
        return int(inside.sum()), int(at.sum())

    true_in, true_at = at_range(paste)
    f = rep.depth_pasted if rep.depth_pasted is not None else np.zeros_like(paste)
    fb_in, fb_at = at_range(f & ~paste)        # foreign pixels that are NOT target paste (obstacle/fill content)
    return dict(hyp=r.hypothesis, ident=r.identity_status, reasons=list(r.reasons), od=rep.status("obstacle_depth"),
                paste=int(paste.sum()), true_in=true_in, true_at=true_at, pasted_in=r.pasted_pixels_in_footprint,
                pasted_at=r.pasted_pixels_at_track_range, foreign_nonpaste=int((f & ~paste).sum()),
                fnp_in=fb_in, fnp_at=fb_at, range_m=float(g["range"]), sigma_r=float(g["sigma_r"]))


def bar_box(x, y, half=0.3, z=Z3):
    return (x, y, half, half, z[0], z[1])


def family(name):
    """Yield (label dict, kwargs for run)."""
    if name == "W1_behind":          # bar directly behind the target; track range error toward the bar (+ lateral)
        for R in (0.7, 1.0, 1.5, 2.0, 3.0, 5.0, 8.0):
            for dR in (0.5, 0.8, 1.0, 1.3, 1.6, 2.0):
                for yb in (0.0, 0.35):
                    for kind in ("box3", "box2", "cyl"):
                        bars = dict(boxes=[bar_box(R + dR + 0.3, yb, 0.3, Z3 if kind == "box3" else Z2)]) if kind != "cyl" \
                            else dict(cyls=[(R + dR + 0.3, yb, 0.3, None, None)])
                        for er in (0.3, 0.5, 0.7, 0.9, 1.1, 1.3, 1.5):
                            for el in (0.0, 0.3):
                                yield dict(R=R, dR=dR, yb=yb, kind=kind, er=er, el=el), dict(target_v=(R, 0., 0.), shift=(er, el, 0.), **bars)
    elif name == "W2_front_behind":  # front bar hides part of the target (sliver); bar behind; range errors both ways
        for R in (1.0, 2.0, 3.0, 5.0, 8.0):
            for cover in (0.5, 0.8):
                xf = 0.6 * R
                yf = (0.15 - 2 * 0.15 * cover) * 0.6 - 0.3       # front bar edge cuts the sphere (box half 0.3, right side)
                for dR in (0.6, 1.0, 1.5):
                    boxes = [bar_box(xf, yf, 0.3, Z3), bar_box(R + dR + 0.3, 0.0, 0.3, Z3)]
                    for er in (-1.5, -1.0, -0.5, 0.5, 1.0, 1.5):
                        for el in (-0.3, 0.0, 0.3):
                            yield dict(R=R, cover=cover, dR=dR, er=er, el=el), dict(target_v=(R, 0., 0.), shift=(er, el, 0.), boxes=boxes)
    elif name == "W3_two_beside":    # bars both sides at the target range with small gaps; wrong track sideways / in range
        for R in (2.0, 3.0, 5.0, 8.0):
            for gap in (0.03, 0.05, 0.1, 0.2, 0.4):
                for dRb in (0.0, 0.5, 1.0, -0.5):
                    boxes = [bar_box(R + dRb, 0.15 + gap + 0.3, 0.3, Z3), bar_box(R + dRb, -(0.15 + gap + 0.3), 0.3, Z3)]
                    for er in (0.0, 0.5, 1.0):
                        for el in (-0.7, -0.5, -0.3, 0.3, 0.5, 0.7):
                            yield dict(R=R, gap=gap, dRb=dRb, er=er, el=el), dict(target_v=(R, 0., 0.), shift=(er, el, 0.), boxes=boxes)
    elif name == "W4_top_corner":    # target near a bar's top corner (2 m pool, top at z=+1), bar behind; range/vertical errors
        for R in (2.0, 3.0, 5.0, 8.0):
            for dR in (0.5, 1.0, 1.5):
                for yb in (0.0, 0.3, 0.45):
                    for zt in (0.7, 0.9, 1.0, 1.1, 1.3):
                        boxes = [bar_box(R + dR + 0.3, yb, 0.3, Z2)]
                        for er in (0.5, 1.0, 1.5):
                            for ez in (-0.3, 0.0, 0.3):
                                yield dict(R=R, dR=dR, yb=yb, zt=zt, er=er, ez=ez), dict(target_v=(R, 0., zt), shift=(er, 0., ez), boxes=boxes)
    elif name == "W5_yaw_elev_border":  # yaw, elevation, image borders, bar behind
        for R in (2.0, 4.0, 8.0):
            for yaw in (0.0, 0.4, -0.8):
                for bearing in (0.0, 25.0, 40.0):
                    for elev in (0.0, 15.0, 25.0):
                        b, e = math.radians(bearing), math.radians(elev)
                        tv = (R * math.cos(e) * math.cos(b), R * math.cos(e) * math.sin(b), R * math.sin(e))
                        for dR in (0.6, 1.2):
                            s = 1 + dR / R
                            boxes = [bar_box(tv[0] * s + 0.3, tv[1] * s, 0.3, Z3)]
                            for er in (0.5, 1.0, 1.5):
                                n = np.array(tv) / R
                                yield dict(R=R, yaw=yaw, bearing=bearing, elev=elev, dR=dR, er=er), dict(target_v=tv, shift=tuple(n * er), boxes=boxes, yaw=yaw)
    elif name == "W6_sigma":        # bar behind, wider belief (cov 0.04) and smaller (0.0025); range errors
        for R in (1.0, 2.0, 4.0, 8.0):
            for dR in (0.6, 1.0, 1.5):
                for cov in (0.0025, 0.04):
                    boxes = [bar_box(R + dR + 0.3, 0.0, 0.3, Z3)]
                    for er in (0.3, 0.5, 0.8, 1.0, 1.3, 1.5):
                        for el in (0.0, 0.3, 0.6):
                            yield dict(R=R, dR=dR, cov=cov, er=er, el=el), dict(target_v=(R, 0., 0.), shift=(er, el, 0.), boxes=boxes, cov_pos=cov)
    elif name == "L1_slivers":       # correct track, target partly hidden by a front bar (vertical edge, top edge), 2 covs
        for R in (1.0, 2.0, 4.0, 8.0, 12.0):
            for xf_frac in (0.5, 0.8):
                xf = xf_frac * R
                for cover in (0.3, 0.5, 0.7, 0.85, 0.95):
                    yf = (0.15 - 2 * 0.15 * cover) * xf_frac - 0.3
                    for cov in (0.01, 0.04):
                        yield dict(R=R, xf=xf_frac, cover=cover, edge="side", cov=cov), dict(target_v=(R, 0., 0.), boxes=[bar_box(xf, yf, 0.3, Z3)], cov_pos=cov)
                        zt = 1.0 + (0.15 - 2 * 0.15 * cover) * xf_frac       # 2 m bar top at z=+1 hides the lower part
                        yield dict(R=R, xf=xf_frac, cover=cover, edge="top", cov=cov), dict(target_v=(R, 0., zt), boxes=[bar_box(xf, 0.0, 0.3, Z2)], cov_pos=cov)
                        # thin cylinder (0.05/0.1 m) through the middle
                        for rc in (0.05, 0.1):
                            yield dict(R=R, xf=xf_frac, cover=cover, edge="thin%.2f" % rc, cov=cov), dict(target_v=(R, 0., 0.), cyls=[(xf, (0.15 - 2 * 0.15 * cover) * xf_frac, rc, None, None)], cov_pos=cov)
    elif name == "L2_far":           # correct track, 10-19 m (paste depth > 10 m), lateral/vertical offsets, with/without bars behind/beside
        for R in (9.9, 10.0, 10.001, 10.05, 10.2, 10.5, 11.0, 12.0, 14.0, 16.0, 18.0, 19.0):
            for lat in (0.0, 0.5, 2.0):
                for z in (0.0, 0.5):
                    for bars in ("none", "beside", "behind"):
                        boxes = [] if bars == "none" else ([bar_box(R, lat + 0.6, 0.3, Z3)] if bars == "beside" else [bar_box(min(R + 1.3, 30), lat, 0.3, Z3)])
                        for cov in (0.01, 0.04):
                            yield dict(R=R, lat=lat, z=z, bars=bars, cov=cov), dict(target_v=(R, lat, z), boxes=boxes, cov_pos=cov)
    elif name == "L3_border":        # correct track near the image borders
        for R in (1.0, 2.0, 4.0, 8.0):
            for bearing in (38.0, 40.0, 41.5, 42.5, 43.0, -40.0, -42.5):
                for elev in (0.0, 24.0, 27.0, 28.5, -27.0):
                    b, e = math.radians(bearing), math.radians(elev)
                    tv = (R * math.cos(e) * math.cos(b), R * math.cos(e) * math.sin(b), R * math.sin(e))
                    for cov in (0.01, 0.04):
                        yield dict(R=R, bearing=bearing, elev=elev, cov=cov), dict(target_v=tv, cov_pos=cov)
    elif name == "L4_close":         # correct track, 0.45-1.2 m, offsets, with a bar behind or beside
        for R in (0.45, 0.5, 0.55, 0.6, 0.7, 0.8, 0.9, 1.0, 1.2):
            for lat in (0.0, 0.1, 0.25):
                for z in (0.0, 0.15):
                    for bars in ("none", "behind1", "beside"):
                        boxes = [] if bars == "none" else ([bar_box(R + 1.3, 0.0, 0.3, Z3)] if bars == "behind1" else [bar_box(R + 0.2, 0.15 + 0.3 + 0.3, 0.3, Z3)])
                        for cov in (0.01, 0.04):
                            yield dict(R=R, lat=lat, z=z, bars=bars, cov=cov), dict(target_v=(R, lat, z), boxes=boxes, cov_pos=cov)
    else:
        raise SystemExit("unknown family " + name)


def main(name, out):
    t0 = time.time(); rows = []; cnt = collections.Counter()
    for label, kw in family(name):
        res = run(**kw); row = dict(label, **res); rows.append(row)
        cnt["cases"] += 1
        sup = res["ident"] == "SUPPORTED_BY_FOOTPRINT"
        cnt["supported"] += sup
        cnt["true_at>0"] += res["true_at"] > 0
        if sup and res["true_at"] == 0:
            cnt["FALSE_SUPPORTED"] += 1
        if (not sup) and res["true_at"] > 0:
            cnt["LOST_SUPPORTED"] += 1
        if res["fnp_at"] > 0:
            cnt["foreign_nonpaste_at_range>0"] += 1
        cnt["ident " + res["ident"]] += 1
    summary = dict(family=name, counts=dict(cnt), seconds=round(time.time() - t0, 1))
    print(json.dumps(summary))
    fs = [r for r in rows if r["ident"] == "SUPPORTED_BY_FOOTPRINT" and r["true_at"] == 0]
    ls = [r for r in rows if r["ident"] != "SUPPORTED_BY_FOOTPRINT" and r["true_at"] > 0]
    for r in fs[:25]:
        print("  FALSE", {k: v for k, v in r.items() if k != "reasons"})
    for r in ls[:25]:
        print("  LOST ", {k: v for k, v in r.items() if k != "reasons"})
    json.dump(dict(summary=summary, false_supported=fs, lost_supported=ls, rows=rows), open(out, "w"), indent=0, default=float)


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
