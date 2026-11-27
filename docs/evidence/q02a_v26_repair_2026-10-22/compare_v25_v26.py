"""Q02-A v2.5 (5fc64a3) vs v2.6 on identical inputs: the earlier counterexamples, the R/D/C/E/F cases, the Opus-review
G1 (blend-vetoed corner) and G2 (box-shaped target) cases
and the Q01-V2 frames.

Usage: python -B docs/evidence/q02a_v26_repair_2026-10-22/compare_v25_v26.py <scratch dir> <Q01 run root> <out.json>
v2.5 is extracted with `git show 5fc64a3:...` into <scratch dir> (outside the repository) and imported under
another package name. Inputs come from the production-faithful test scene. Read-only on the Q01 raw.
"""
import importlib, json, math, subprocess, sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT)); sys.path.insert(0, str(ROOT / "tests"))


def load_base(scratch):
    pkg = Path(scratch) / "q02v25"
    pkg.mkdir(parents=True, exist_ok=True)
    for name in ("__init__", "contracts", "geometry", "validation", "evidence"):
        src = subprocess.check_output(["git", "-C", str(ROOT), "show", "5fc64a3:motar_eval/q02/%s.py" % name], text=True)
        (pkg / ("%s.py" % name)).write_text(src)
    sys.path.insert(0, str(Path(scratch)))
    return importlib.import_module("q02v25.evidence")


def summary(rec):
    return dict(hypothesis=rec.hypothesis, reasons=list(rec.reasons), identity=rec.identity_status,
                pasted_in_footprint=rec.pasted_pixels_in_footprint,
                pasted_at_track_range=getattr(rec, "pasted_pixels_at_track_range", None),
                pasted_nearer_than_track=getattr(rec, "pasted_pixels_nearer_than_track", None),
                target_at_track_range=getattr(rec, "target_pixels_at_track_range", None),
                geometry_support=getattr(rec, "geometry_support", None), identity_verdict=getattr(rec, "identity_verdict", None),
                sphere_fit_status=getattr(rec, "sphere_fit_status", None),
                lidar_extended=rec.lidar.foreground_extended, near_warning=rec.near_obstacle_warning,
                problems=list(rec.validation_problems))


def constant_paste(t, mask_fn, value):
    depth = np.clip(t.AV @ np.full((24, 40), 10.0) @ t.AH.T, 0.0, 10.0)
    vv, uu = np.mgrid[0:90, 0:160]
    depth[mask_fn(vv, uu)] = value
    return depth


def main(scratch, run_root, out):
    import test_q02_evidence as t
    from motar_eval.q02 import contracts as c, evidence as new
    base = load_base(scratch)
    side = ((4 * math.cos(math.radians(8)), 4 * math.sin(math.radians(8)), .30),)
    far = t.make(target_v=(10.5, 0., 0.), target_in_camera=False)
    far_mean = np.array(far.track_mean); far_mean[0] += 1.0
    near = t.make(target_v=(3., 0., 0.))
    near_mean = np.array(near.track_mean); near_mean[0] += 5.0
    cases = {
        "F1 empty scene, belief 12 m": dict(target_v=(12., 0., 0.), render_target=False),
        "F1 empty scene, belief 10 m": dict(target_v=(10., 0., 0.), render_target=False),
        "F1 empty scene, belief 9 m": dict(target_v=(9., 0., 0.), render_target=False),
        "F1 bar 8.6 m before belief 9.6 m": dict(target_v=(9.6, 0., 0.), render_target=False, bars=((8.6, 0., .3),)),
        "F1 upsampling blend beside LOS": dict(render_target=False, bars=((4., 0.5, .3),)),
        "F2 side bar, clear LOS": dict(render_target=False, bars=side),
        "F2 bar crossing LOS": dict(render_target=False, bars=((4., 0., .3),)),
        "F2 bar behind belief": dict(render_target=False, bars=((9.5, 0., .3),)),
        "F3 depth all zero": dict(render_target=False, depth=np.zeros((90, 160))),
        "F3 LiDAR all zero": dict(render_target=False, lidar=np.zeros(288)),
        "F4 thin bar across envelope": dict(render_target=False, bars=((4., 0.12, .08),)),
        "F6 flag with nothing at track": dict(detected=True, render_target=False),
        "visible target, flag set": dict(detected=True),
        "R1 constant disc r40 paste, no flag": dict(render_target=False,
                                                    depth=constant_paste(t, lambda v, u: (v - 45) ** 2 + (u - 80) ** 2 < 1600, 2.0)),
        "R1 disc r60 across top/bottom, no flag": dict(render_target=False,
                                                       depth=constant_paste(t, lambda v, u: (v - 45) ** 2 + (u - 80) ** 2 < 3600, 2.0)),
        "R2 F5 far case": dict(target_v=(10.5, 0., 0.), target_in_camera=False, track_mean=far_mean),
        "R2 F5 far case + bar 90 deg left at 10.3 m": dict(target_v=(10.5, 0., 0.), target_in_camera=False,
                                                           track_mean=far_mean, bars=((0., 10.3, .3),)),
        "R2 F5 far case + bar behind vehicle at 10.3 m": dict(target_v=(10.5, 0., 0.), target_in_camera=False,
                                                              track_mean=far_mean, bars=((-10.3, 0., .3),)),
        "R2 F5 far case, real bar on LOS": dict(target_v=(10.5, 0., 0.), render_target=False, track_mean=far_mean,
                                                bars=((9.5, 0., .3),)),
        "R3 target pasted 3 m, belief 8 m, flag": dict(target_v=(3., 0., 0.), detected=True, track_mean=near_mean),
        "R3 no target, bar r0.15 at 8 m, flag": dict(render_target=False, bars=((8., 0., .15),), detected=True),
        "R3 no target, bar r0.4 at 8 m, flag": dict(render_target=False, bars=((8., 0., .4),), detected=True),
    }
    for rng_ in (0.30, 0.35, 0.40, 0.45, 0.50, 0.60):
        cases["R4 target %.2f m, flag" % rng_] = dict(target_v=(rng_, 0., 0.), detected=True)
    for rng_ in (0.265, 0.28, 0.30):
        cases["D1 target %.3f m (contact range), flag" % rng_] = dict(target_v=(rng_, 0., 0.), detected=True)
        cases["D1 target %.3f m (contact range), no flag" % rng_] = dict(target_v=(rng_, 0., 0.))
    for rng_ in (10.0, 9.8, 9.5, 6.0):
        side = np.array(t.make(target_v=(rng_, 0., 0.)).track_mean); side[1] += 0.5
        cases["D2 target %.1f m, track 0.5 m beside, flag" % rng_] = dict(target_v=(rng_, 0., 0.), detected=True, track_mean=side)
    for rng_, gap in ((8.0, 0.1), (5.0, 0.2), (3.0, 0.3)):
        bars = ((rng_, 0.15 + gap + 0.3, 0.3),)
        cases["C1 target %.0f m, bar %.1f m beside, correct track, flag" % (rng_, gap)] = dict(target_v=(rng_, 0., 0.), detected=True, bars=bars)
        for err in (0.5, 0.7):
            onbar = np.array(t.make(target_v=(rng_, 0., 0.), bars=bars).track_mean); onbar[1] += err
            cases["C1 target %.0f m, bar %.1f m beside, track %.1f m off, flag" % (rng_, gap, err)] = dict(
                target_v=(rng_, 0., 0.), detected=True, bars=bars, track_mean=onbar)
    for rng_, gap in ((3.0, 1.0), (2.0, 0.6), (8.0, 1.0)):
        bars = ((rng_ + gap + 0.3, 0.0, 0.3),)
        cases["E1 target %.0f m, bar %.1f m behind, correct track, flag" % (rng_, gap)] = dict(target_v=(rng_, 0., 0.), detected=True, bars=bars)
        for err in (0.5, 1.0):
            far = np.array(t.make(target_v=(rng_, 0., 0.), bars=bars).track_mean); far[0] += err
            cases["E1 target %.0f m, bar %.1f m behind, track %.1f m too far, flag" % (rng_, gap, err)] = dict(
                target_v=(rng_, 0., 0.), detected=True, bars=bars, track_mean=far)
    cam_o = np.array(t.CAM.camera_offset); x_f, half = 2.5, 0.3
    corner = np.array([x_f, 0.0, 1.0]); cdir = (corner - cam_o) / np.linalg.norm(corner - cam_o)
    ey = np.cross([0, 0, 1.0], cdir); ey /= np.linalg.norm(ey); ez = np.cross(cdir, ey)
    dr = cdir - 0.7 * (t.CAM.target_radius / 4.0) * (ey + ez); dr /= np.linalg.norm(dr)
    tvf = cam_o + 4.0 * dr; fbox = [(x_f + half, -half, half, half, -1.0, 1.0)]
    fdepth, flidar = t.box_scene(tvf, fbox)
    cases["F1 target 4 m, front 2 m bar corner in silhouette, correct track, flag"] = dict(target_v=tuple(tvf), detected=True, depth=fdepth, lidar=flidar)
    for delta in (-0.3, 0.0, 0.3):
        face = cam_o + ((x_f - cam_o[0]) / dr[0] + delta) * dr
        cases["F1 same, track at the bar face %+.1f m, flag" % delta] = dict(target_v=tuple(face), detected=True, depth=fdepth, lidar=flidar)
    gtv, gdr, gdepth, glidar = t._front_corner_scene(3.0, 0.4, -0.7, -0.7)
    cases["G1 corner fixture, correct track, flag"] = dict(target_v=tuple(gtv), detected=True, depth=gdepth, lidar=glidar)
    for rt, cv in ((6.3, 0.04), (6.6, 0.01), (7.3, 0.01)):
        cases["G1 corner fixture, track %.1f m on the target ray, flag" % rt] = dict(target_v=tuple(cam_o + rt * gdr), detected=True,
                                                                                   depth=gdepth, lidar=glidar, cov_pos=cv)
    for bc in ((4.0, 0.2, 0.0), (2.0, 0.0, 0.1)):
        cases["G2 box-shaped target at %s (SPHERE declared), correct track, flag" % (bc,)] = dict(target_v=bc, detected=True,
                                                                                                 depth=t.box_target_scene(bc))
    cases["D7 flag, belief age 5 s"] = dict(detected=True, age=5.0)
    cases["D7 flag, belief sigma 2 m"] = dict(detected=True, cov=np.diag([4.0, 4.0, 4.0, .1, .1, .1]))
    results = {}
    for name, kw in cases.items():
        inp = t.make(**kw)
        results[name] = dict(v25=summary(base.estimate(inp)), v26=summary(new.estimate(inp)))
    frames = {}
    run_root = Path(run_root) / "S1_ON"
    rows = [json.loads(l) for l in (run_root / "sensors.jsonl").read_text().splitlines()]
    ck = lambda tt: c.Clock(tt["capture_sim_s"], tt["delivery_sim_s"], tt["provenance"], tt["frame"], tt["depth_convention"])
    for i in range(3):
        z = dict(np.load(run_root / ("sensor_%03d.npz" % i), allow_pickle=False))
        for label, flag in (("recorded_flag", None), ("camera_flag_cleared", False)):
            v = dict(z)
            if flag is not None:
                v["camera"] = np.zeros_like(v["camera"])
            inp = c.EvidenceInput.from_arrays(v, ck(rows[i]["camera_time"]), ck(rows[i]["lidar_time"]),
                                              rows[i]["camera_time"]["decision_sim_s"], tuple(rows[i]["calibration"]))
            frames["frame%d %s" % (i, label)] = dict(v25=summary(base.estimate(inp)), v26=summary(new.estimate(inp)))
    Path(out).write_text(json.dumps(dict(scope="SMOKE_AND_COUNTEREXAMPLES_NOT_ACCURACY", synthetic=results,
                                         q01_frames=frames), indent=1) + "\n")
    for k, r in list(results.items()) + list(frames.items()):
        same = "  " if (r["v25"]["hypothesis"], r["v25"]["identity"]) == (r["v26"]["hypothesis"], r["v26"]["identity"]) else "* "
        print("%s%-46s v2.5 %-18s %-22s -> v2.6 %-18s %-22s %s" % (same, k, r["v25"]["hypothesis"], r["v25"]["identity"],
              r["v26"]["hypothesis"], r["v26"]["identity"], ",".join(r["v26"]["reasons"])[:90]))


if __name__ == "__main__":
    main(*sys.argv[1:4])
