#!/usr/bin/env python3
"""Independent float64 reference for S-L1D online measurement validity and evaluator classes (CPU).

NumPy only; never imports production code. Camera rays and first hits come from the committed
S-L1A oracle; frames follow S-L1B (camera = vehicle origin + translation, yaw-only, R = I unless
given). The evaluator reference mirrors the E-L0 rules (target-alone support, visible support,
exact centre ray) on analytic geometry; the online reference sees only depth, the detection flag
and a predicted centre with its covariance.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import s_l1a_reference as ref  # noqa: E402

CONTRACT = "MOTAR_S_L1D_FLOAT64_REFERENCE_V1"
CAMERA_TRANSLATION = np.array([0.10, 0.0, 0.03])
RADIUS = 0.15
FAR = 20.0
TAU_FLOOR = 0.05
ONLINE = ("NO_TRACK", "DETECTED", "PRED_RANGE_EXIT", "PRED_FOV_EXIT", "PRED_SCALE_LOSS",
          "OCCLUSION_FULL_EVIDENCE", "OCCLUSION_CENTRE_EVIDENCE", "OCCLUSION_PARTIAL_EVIDENCE", "UNEXPLAINED_LOSS")
EVALUATOR = ("NONE", "RANGE_EXIT", "FOV_EXIT", "SCALE_LOSS", "FULL", "CENTRE_ONLY", "PARTIAL", "VISIBLE")


def rays(width=160, height=90, hfov_deg=87., vfov_deg=58.):
    return ref.normalized_pixel_rays(width, height, hfov_deg, vfov_deg)


def sphere_hits(origin, directions, centre, radius=RADIUS, far=FAR):
    """Per-ray first hit distance on the sphere (inf when missed or at/after far)."""
    oc = origin - np.asarray(centre, dtype=np.float64)
    b = directions @ oc
    c = oc @ oc - radius * radius
    disc = b * b - c
    root = np.sqrt(np.clip(disc, 0, None))
    t = -b - root
    t = np.where(t < 0, -b + root, t)
    ok = (disc >= 0) & (t >= 0) & (t < far)
    return np.where(ok, t, np.inf)


def predicted_support(centre_body, *, translation=CAMERA_TRANSLATION, width=160, height=90, hfov_deg=87., vfov_deg=58.):
    r = rays(width, height, hfov_deg, vfov_deg)
    t = np.asarray(translation, dtype=np.float64)
    c = np.asarray(centre_body, dtype=np.float64)
    hit = sphere_hits(t, r.reshape(-1, 3), c).reshape(height, width)
    rel = c - t
    x, y, z = rel
    in_frustum = bool(x > 0 and abs(y) <= x * math.tan(math.radians(hfov_deg) / 2)
                      and abs(z) <= x * math.tan(math.radians(vfov_deg) / 2) and np.linalg.norm(rel) <= FAR)
    fx = width / (2 * math.tan(math.radians(hfov_deg) / 2))
    fy = height / (2 * math.tan(math.radians(vfov_deg) / 2))
    if x > 0:
        u = (width - 1) / 2 - fx * y / x
        v = (height - 1) / 2 - fy * z / x
        pixel = (int(np.clip(np.rint(v), 0, height - 1)), int(np.clip(np.rint(u), 0, width - 1)))
    else:
        pixel = None
    return {"mask": np.isfinite(hit), "hit_m": hit, "in_frustum": in_frustum, "centre_range_m": float(np.linalg.norm(rel)),
            "centre_pixel": pixel, "unit": rel / np.linalg.norm(rel)}


def ambiguous_support_pixels(centre_body, *, translation=CAMERA_TRANSLATION, eps_m=1e-6, width=160, height=90,
                             hfov_deg=87., vfov_deg=58.):
    """Pixel rays whose closest approach to the sphere centre is within eps of the radius (forward)."""
    r = rays(width, height, hfov_deg, vfov_deg).reshape(-1, 3)
    rel = np.asarray(centre_body, dtype=np.float64) - np.asarray(translation, dtype=np.float64)
    along = r @ rel
    closest = np.linalg.norm(rel[None] - along[:, None] * r, axis=1)
    return int(((np.abs(closest - RADIUS) <= eps_m) & (along > 0)).sum())


def online_class(depth, detected, active, centre_body, cov_pos, *, min_pixels=2, translation=CAMERA_TRANSLATION):
    """Sensor-inferable state from delivered depth, the detection flag and the track's own belief."""
    if not active:
        return {"class": "NO_TRACK"}
    if detected:
        return {"class": "DETECTED"}
    s = predicted_support(centre_body, translation=translation)
    support = int(s["mask"].sum())
    u = s["unit"]
    tau = TAU_FLOOR + 2 * math.sqrt(max(float(u @ np.asarray(cov_pos, dtype=np.float64) @ u), 0.))
    d = np.asarray(depth, dtype=np.float64)
    occluded = s["mask"] & (d < s["hit_m"] - tau)
    n_occ = int(occluded.sum())
    centre_occ = bool(s["centre_pixel"] is not None and s["mask"][s["centre_pixel"]] and occluded[s["centre_pixel"]])
    if support == 0 and s["centre_range_m"] - RADIUS >= FAR:
        cls = "PRED_RANGE_EXIT"
    elif support == 0 and not s["in_frustum"]:
        cls = "PRED_FOV_EXIT"
    elif support < min_pixels:
        cls = "PRED_SCALE_LOSS"
    elif n_occ == support:
        cls = "OCCLUSION_FULL_EVIDENCE"
    elif n_occ > 0 and centre_occ:
        cls = "OCCLUSION_CENTRE_EVIDENCE"
    elif n_occ > 0:
        cls = "OCCLUSION_PARTIAL_EVIDENCE"
    else:
        cls = "UNEXPLAINED_LOSS"
    return {"class": cls, "support": support, "occluded": n_occ, "centre_occluded": centre_occ, "tau_m": tau}


def evaluator_class(box_centers, box_half_extents, target_body, *, translation=CAMERA_TRANSLATION, obstacle_far=10.):
    """E-L0 rule on analytic geometry: alone support, visible support, exact centre ray."""
    t = np.asarray(translation, dtype=np.float64)
    s = predicted_support(target_body, translation=translation)
    alone = int(s["mask"].sum())
    render = ref.render_reference(box_centers, box_half_extents, sphere_center=target_body, sphere_radius=RADIUS,
                                  origin=tuple(t), far_m=FAR)
    visible = int((render["hit_class"] == ref.SPHERE).sum())
    # Exact centre ray: does a box intersect the camera->centre segment before the sphere surface?
    direction = s["unit"]
    sphere_t = sphere_hits(t, direction[None], target_body)[0]
    box_t = ref.first_hits(t, direction[None], box_centers, box_half_extents, far_m=FAR)["depth_m"][0] if len(box_centers) else np.inf
    centre_blocked = bool(box_t < sphere_t)
    pixel_blocked = None
    if s["centre_pixel"] is not None:
        pixel_blocked = bool(render["hit_class"][s["centre_pixel"]] == ref.BOX and s["mask"][s["centre_pixel"]])
    if alone == 0:
        cls = "RANGE_EXIT" if s["centre_range_m"] - RADIUS >= FAR else ("FOV_EXIT" if not s["in_frustum"] else "SCALE_LOSS")
    elif visible == 0:
        cls = "FULL"
    elif visible < alone:
        cls = "CENTRE_ONLY" if centre_blocked else "PARTIAL"
    else:
        cls = "VISIBLE"
    return {"class": cls, "alone_px": alone, "visible_px": visible, "centre_blocked": centre_blocked,
            "centre_pixel_blocked": pixel_blocked, "centre_robust": pixel_blocked is not None and pixel_blocked == centre_blocked,
            "render_depth": render["depth_m"], "render_class": render["hit_class"]}


def delivered_depth(render, obstacle_far=10.):
    """Production depth composition: obstacle first hits (far 10) with the target overlaid (far 20)."""
    d = np.where(render["render_class"] == ref.BOX, np.minimum(render["render_depth"], obstacle_far), obstacle_far)
    return np.where(render["render_class"] == ref.SPHERE, render["render_depth"], d)


def self_test():
    checks = {}
    clear = evaluator_class([], [], [5., 0., 0.])
    checks["clear_visible"] = clear["class"] == "VISIBLE" and clear["alone_px"] > 2
    full = evaluator_class([[3.3, 0., .5]], [[.3, .6, 1.5]], [6., 0., 0.])
    checks["full_occlusion"] = full["class"] == "FULL"
    online = online_class(delivered_depth(full), False, True, [6., 0., 0.], np.eye(3) * 1e-6)
    checks["online_full_evidence"] = online["class"] == "OCCLUSION_FULL_EVIDENCE"
    checks["online_clear_unexplained"] = online_class(delivered_depth(clear), False, True, [5., 0., 0.], np.eye(3) * 1e-6)["class"] == "UNEXPLAINED_LOSS"
    checks["fov_exit"] = online_class(np.full((90, 160), 10.), False, True, [0., 5., 0.], np.eye(3) * 1e-6)["class"] == "PRED_FOV_EXIT"
    checks["range_exit"] = online_class(np.full((90, 160), 10.), False, True, [24., 0., 0.], np.eye(3) * 1e-6)["class"] == "PRED_RANGE_EXIT"
    return checks


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--self-test", action="store_true")
    parser.parse_args(argv)
    checks = self_test()
    print(json.dumps({"contract": CONTRACT, "checks": checks}, indent=2))
    return 0 if all(checks.values()) else 1


if __name__ == "__main__":
    sys.exit(main())
