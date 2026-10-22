"""Reviewer probes (read-only on the repo). Per-scene worst-case wrong-track test for Q02A-v2.6."""
import sys, math, json, collections, time
from types import SimpleNamespace
import numpy as np
W = "/home/fair/workspaces/aerial_gym_ws/src/MOTAR-mt-p1"
sys.path.insert(0, W + "/docs/evidence/q02a_v26_repair_2026-10-22")
import identity_sweeps as IS
from motar_eval.q02 import evidence as ev, geometry as geo, validation as va
assert ev.__file__.startswith(W + "/"), ev.__file__
M, T, CAM, LID, CAM_O, P = IS.M, IS.T, IS.CAM, IS.LID, IS.CAM_O, IS.P
Z2, Z3 = IS.Z2, IS.Z3
J = IS.Judge("v26", None)
SUP = "SUPPORTED_BY_FOOTPRINT"

def gfor(sc, track_v, cov):
    world = geo.vehicle_to_world(np.asarray(track_v, float), T.POS, sc["yaw"])
    ns = SimpleNamespace(vehicle_quat=T.yaw_quat(sc["yaw"]), track_mean=np.r_[world, 0, 0, 0],
                         track_cov=np.diag([cov] * 3 + [0.1] * 3), position=T.POS, depth=sc["inp0"].depth)
    return ev._belief_geometry(ns, CAM, LID, P)

def full(sc, track_v, cov):
    return ev.estimate(T.make(target_v=tuple(track_v), detected=True, depth=sc["depth"], lidar=sc["lidar"].reshape(-1),
                              cov_pos=cov, yaw=sc["yaw"]))

def analyse(sc, o, keep, tag):
    """Scene-level: fit status; whether the accepted sphere is the true target; worst-case wrong track at its centre."""
    fit = sc["fit"]; tv = sc["target_v"]; paste = sc["paste"]
    o["scenes"] += 1; o["fit_" + fit.status] += 1
    npaste = int(paste.sum()); o["paste_lt10"] += npaste < 10
    if fit.status != geo.SPHERE_OK:
        return
    cerr = float(np.linalg.norm(fit.centre - tv)); tin = int((fit.inlier_mask & paste).sum())
    if cerr <= 0.05:
        o["ok_true"] += 1; return
    o["ok_offset"] += 1
    if tin == 0: o["ok_spurious_no_true_inlier"] += 1
    # worst-case wrong track: at the accepted sphere's centre, three covariances
    worst = False
    for cov in (0.0025, 0.01, 0.04):
        r = full(sc, fit.centre, cov)
        if r.identity_status == SUP:
            g = gfor(sc, fit.centre, cov); t_at, gap, cin = IS.truth(sc, g)
            if t_at == 0 and not cin and not (gap is not None and gap <= 0.05):
                worst = True
    rec = dict(tag=tag, worst=worst, npaste=npaste, cerr=cerr, n_in=fit.inliers, tin=tin, unexpl=fit.farther_unexplained,
               rival=fit.rival_inliers, band=fit.band_pixels, R=float(np.linalg.norm(tv - CAM_O)))
    keep.append(rec)
    if worst:
        o["WORST_FALSE_SUPPORT_SCENES"] += 1
    if False:
        keep.append(dict(tag=tag, npaste=npaste, cerr=cerr, n_in=fit.inliers, tin=tin, unexpl=fit.farther_unexplained,
                         rival=fit.rival_inliers, band=fit.band_pixels, R=float(np.linalg.norm(tv - CAM_O))))
