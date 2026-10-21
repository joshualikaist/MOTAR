"""Q02-A v1 (a10be96) vs v2 on identical inputs: the Fable F1-F6 counterexamples and the Q01-V2 frames.

Usage: python -B docs/evidence/q02a_v2_repair_2026-10-21/compare_v1_v2.py <scratch dir> <Q01 run root> <out.json>
v1 is extracted with `git show a10be96:...` into <scratch dir> (outside the repository) and imported under
another package name. Inputs come from the v2 production-faithful test scene. Read-only on the Q01 raw.
"""
import importlib, json, math, subprocess, sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT)); sys.path.insert(0, str(ROOT / "tests"))


def load_v1(scratch):
    pkg = Path(scratch) / "q02v1"
    pkg.mkdir(parents=True, exist_ok=True)
    for name in ("__init__", "contracts", "geometry", "validation", "evidence"):
        src = subprocess.check_output(["git", "-C", str(ROOT), "show", "a10be96:motar_eval/q02/%s.py" % name], text=True)
        (pkg / ("%s.py" % name)).write_text(src)
    sys.path.insert(0, str(Path(scratch)))
    return importlib.import_module("q02v1.evidence")


def summary(rec, v):
    out = dict(hypothesis=rec.hypothesis, reasons=list(rec.reasons))
    if v == 2:
        out.update(identity=rec.identity_status, camera=rec.camera.__dict__, lidar=rec.lidar.__dict__,
                   pasted=rec.pasted_pixels_in_footprint, near_warning=rec.near_obstacle_warning)
    return out


def main(scratch, run_root, out):
    import test_q02_evidence as t
    from motar_eval.q02 import contracts as c, evidence as v2
    v1 = load_v1(scratch)
    side = ((4 * math.cos(math.radians(8)), 4 * math.sin(math.radians(8)), .30),)
    cases = {
        "F1 empty scene, belief 12 m": dict(target_v=(12., 0., 0.), render_target=False),
        "F1 empty scene, belief 10 m": dict(target_v=(10., 0., 0.), render_target=False),
        "F1 empty scene, belief 9 m": dict(target_v=(9., 0., 0.), render_target=False),
        "F1 bar 8.6 m before belief 9.6 m": dict(target_v=(9.6, 0., 0.), render_target=False, bars=((8.6, 0., .3),)),
        "F1 upsampling blend beside LOS": dict(render_target=False, bars=((4., 0.5, .3),)),
        "F2 side bar, clear LOS (Fable case)": dict(render_target=False, bars=side),
        "F2 bar crossing LOS": dict(render_target=False, bars=((4., 0., .3),)),
        "F2 bar behind belief": dict(render_target=False, bars=((9.5, 0., .3),)),
        "F3 depth all zero": dict(render_target=False, depth=np.zeros((90, 160))),
        "F3 LiDAR all zero": dict(render_target=False, lidar=np.zeros(288)),
        "F4 thin bar across envelope": dict(render_target=False, bars=((4., 0.12, .08),)),
        "F6 flag with nothing at track": dict(detected=True, render_target=False),
        "visible target, flag set": dict(detected=True),
    }
    results = {}
    for name, kw in cases.items():
        inp = t.make(**kw)
        results[name] = dict(v1=summary(v1.estimate(inp), 1), v2=summary(v2.estimate(inp), 2))
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
            frames["frame%d %s" % (i, label)] = dict(v1=summary(v1.estimate(inp), 1), v2=summary(v2.estimate(inp), 2))
    Path(out).write_text(json.dumps(dict(scope="SMOKE_AND_COUNTEREXAMPLES_NOT_ACCURACY", synthetic=results,
                                         q01_frames=frames), indent=1) + "\n")
    for k, r in list(results.items()) + list(frames.items()):
        print("%-40s v1 %-18s -> v2 %-18s %s" % (k, r["v1"]["hypothesis"], r["v2"]["hypothesis"], ",".join(r["v2"]["reasons"])[:80]))


if __name__ == "__main__":
    main(*sys.argv[1:4])
