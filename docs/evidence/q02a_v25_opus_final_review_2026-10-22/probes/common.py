import sys, math, json, collections, time
import numpy as np
sys.path.insert(0, "/home/fair/workspaces/aerial_gym_ws/src/MOTAR-mt-p1/docs/evidence/q02a_v25_repair_2026-10-22")
import identity_attack_v24_families as M
A = M.A
from motar_eval.q02 import geometry as geo, validation as va, evidence as ev
CAM, LID, CAM_O, P = M.CAM, M.LID, M.CAM_O, M.P
Z2, Z3 = M.Z2, M.Z3
assert ev.__file__.startswith("/home/fair/workspaces/aerial_gym_ws/src/MOTAR-mt-p1/"), ev.__file__

def detail(target_v, boxes=(), cyls=(), track_v=None, cov_pos=0.01, yaw=0.0):
    """M.run plus the fitted sphere: centre distance to the true centre, mask overlap with true paste."""
    res = M.run(target_v, boxes=boxes, cyls=cyls, track_v=track_v, cov_pos=cov_pos, yaw=yaw)
    depth, lidar, paste, native = A.scene(np.asarray(target_v, float), boxes, cyls)
    inp = A.t.make(target_v=tuple(np.asarray(target_v if track_v is None else track_v, float)), detected=True, depth=depth, lidar=lidar.reshape(-1), cov_pos=cov_pos, yaw=yaw)
    rep = va.validate(inp, CAM, LID)
    if rep.depth_pasted is None:
        res.update(fit="NO_REPORT"); return res
    c, mask, st, nin = geo.fit_target_sphere(inp.depth, rep.depth_pasted, CAM, CAM.target_radius)
    res.update(fit=st, fit_inliers=int(nin), cand=int(rep.depth_pasted.sum()), cand_true=int((rep.depth_pasted & paste).sum()),
               mask=int(mask.sum()), mask_true=int((mask & paste).sum()), mask_false=int((mask & ~paste).sum()),
               centre_err=float(np.linalg.norm(c - np.asarray(target_v, float))) if c is not None else None)
    return res

def is_false(r):
    return r["paste"] >= 2 and r["ident"] == "SUPPORTED_BY_FOOTPRINT" and r["true_at"] == 0
def is_correct(r):
    return abs(r["err_range"]) <= 0.3 and r["err_lat"] <= 0.15
