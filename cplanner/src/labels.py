"""P0-R target label layer: camera output as LABELS only.

The camera tells the planner where the target probably is (goal setting, prediction, reacquisition in P1/G1). It
never touches LiDAR evidence: this layer owns its own array, the occupancy map and the certificate never import or
read it, and labelling a return does not change its occupancy or its surface veto. A label says "this occupied cell
was probably the target at step k"; after LABEL_TTL_S it is no longer reported, and the cell is reported as the
occupancy map says (OBSERVED_HIT until free evidence resolves it), never FREE.

Camera array (float64, 14): timestamp, visible (0/1), camera extrinsic in the yaw-level LiDAR frame (x, y, z, roll,
pitch, yaw; R = Rz Ry Rx, as v7), unit target direction in the camera frame (3), range, range_sigma, bearing_sigma.
"""
import math

import numpy as np

from evidence_map import WIDTH

LABEL_TTL_S = 1.0
CAMERA_SYNC_S = 0.05
GATE_BASE = 0.20
GATE_K = 3.0


def rot(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw),
                              math.sin(yaw))
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


class TargetLabelLayer:
    def __init__(self, bounds, max_range):
        lo, hi = bounds
        self.lo = float(lo)
        self.n = int(round((hi - lo) / WIDTH))
        self.label_step = np.full((self.n, self.n), -1, np.int64)
        self.label_t = np.full((self.n, self.n), -np.inf)
        self.max_range = max_range
        self.estimate = None

    def update(self, step, t, origin, yaw, hits3d, camera):
        """Label this packet's HIT points near the camera's target estimate. Returns a report; changes only this
        layer. `camera` is None or the core's private copy of a (14,) float64 array."""
        if camera is None:
            return {"camera": "ABSENT", "labelled_returns": 0, "target_estimate_world": None}
        c = camera
        if not np.isfinite(c).all():
            return {"camera": "IGNORED_NONFINITE", "labelled_returns": 0, "target_estimate_world": None}
        if abs(c[0] - t) > CAMERA_SYNC_S or c[1] not in (0.0, 1.0):
            return {"camera": "IGNORED_NOT_SYNCHRONISED_OR_MALFORMED", "labelled_returns": 0,
                    "target_estimate_world": None}
        if c[1] == 0.0:
            return {"camera": "TARGET_NOT_VISIBLE", "labelled_returns": 0, "target_estimate_world": None}
        u, rng, sr, sb = c[8:11], c[11], c[12], c[13]
        if abs(math.sqrt(float(u @ u)) - 1.0) > 1e-6 or not (0.0 < rng <= self.max_range) or sr < 0 or sb < 0:
            return {"camera": "IGNORED_MALFORMED_RANGE_EVIDENCE", "labelled_returns": 0,
                    "target_estimate_world": None}
        in_sensor = c[2:5] + rot(*c[5:8]) @ (u * rng)
        centre = np.asarray(origin) + rot(0.0, 0.0, yaw) @ in_sensor
        gate = GATE_BASE + GATE_K * (sr + rng * sb)
        n = 0
        for p in hits3d:
            if math.dist(p, centre) <= gate:
                i, j = (int(math.floor((p[k] - self.lo) / WIDTH)) for k in (0, 1))
                if 0 <= i < self.n and 0 <= j < self.n:
                    self.label_step[i, j] = step
                    self.label_t[i, j] = t
                    n += 1
        self.estimate = (step, tuple(float(x) for x in centre))
        return {"camera": "USED_FOR_LABELS_ONLY", "labelled_returns": n,
                "target_estimate_world": tuple(float(x) for x in centre), "label_gate_m": float(gate)}

    def labelled(self, flat_id, occ_last, now_t):
        """True if the cell's latest occupancy evidence (step occ_last) was labelled and the label is still fresh."""
        return (occ_last >= 0 and self.label_step.flat[flat_id] == occ_last
                and now_t - self.label_t.flat[flat_id] <= LABEL_TTL_S)
