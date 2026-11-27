"""Q02-A v2 (87cced6) vs v2.1 on identical inputs: the v1->v2 counterexamples, the re-review R1-R4 cases
and the Q01-V2 frames.

Usage: python -B docs/evidence/q02a_v21_repair_2026-10-22/compare_v2_v21.py <scratch dir> <Q01 run root> <out.json>
v2 is extracted with `git show 87cced6:...` into <scratch dir> (outside the repository) and imported under
another package name. Inputs come from the production-faithful test scene. Read-only on the Q01 raw.
"""
import importlib, json, math, subprocess, sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT)); sys.path.insert(0, str(ROOT / "tests"))


def load_v2(scratch):
    pkg = Path(scratch) / "q02v2"
    pkg.mkdir(parents=True, exist_ok=True)
    for name in ("__init__", "contracts", "geometry", "validation", "evidence"):
        src = subprocess.check_output(["git", "-C", str(ROOT), "show", "87cced6:motar_eval/q02/%s.py" % name], text=True)
        (pkg / ("%s.py" % name)).write_text(src)
    sys.path.insert(0, str(Path(scratch)))
    return importlib.import_module("q02v2.evidence")


def summary(rec):
    return dict(hypothesis=rec.hypothesis, reasons=list(rec.reasons), identity=rec.identity_status,
                pasted_in_footprint=rec.pasted_pixels_in_footprint,
                pasted_at_track_range=getattr(rec, "pasted_pixels_at_track_range", None),
                lidar_extended=rec.lidar.foreground_extended, problems=list(rec.validation_problems))


def constant_paste(t, mask_fn, value):
    depth = np.clip(t.AV @ np.full((24, 40), 10.0) @ t.AH.T, 0.0, 10.0)
    vv, uu = np.mgrid[0:90, 0:160]
    depth[mask_fn(vv, uu)] = value
    return depth


def main(scratch, run_root, out):
    import test_q02_evidence as t
    from motar_eval.q02 import contracts as c, evidence as v21
    v2 = load_v2(scratch)
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
    results = {}
    for name, kw in cases.items():
        inp = t.make(**kw)
        results[name] = dict(v2=summary(v2.estimate(inp)), v21=summary(v21.estimate(inp)))
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
            frames["frame%d %s" % (i, label)] = dict(v2=summary(v2.estimate(inp)), v21=summary(v21.estimate(inp)))
    Path(out).write_text(json.dumps(dict(scope="SMOKE_AND_COUNTEREXAMPLES_NOT_ACCURACY", synthetic=results,
                                         q01_frames=frames), indent=1) + "\n")
    for k, r in list(results.items()) + list(frames.items()):
        same = "  " if (r["v2"]["hypothesis"], r["v2"]["identity"]) == (r["v21"]["hypothesis"], r["v21"]["identity"]) else "* "
        print("%s%-46s v2 %-18s %-22s -> v2.1 %-18s %-22s %s" % (same, k, r["v2"]["hypothesis"], r["v2"]["identity"],
              r["v21"]["hypothesis"], r["v21"]["identity"], ",".join(r["v21"]["reasons"])[:90]))


if __name__ == "__main__":
    main(*sys.argv[1:4])
