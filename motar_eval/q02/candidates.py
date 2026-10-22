"""Module D v0: reacquisition-candidate feasibility metadata (CPU, pure). Not a planner.

Generation (a fixed viewpoint lattice around the current position) is separate from evaluation;
there is no selection. Path safety and information potential are separate questions. A candidate
is FEASIBLE only if its straight path is SAFE in the observed-free map (UNKNOWN space never
certifies a path) AND it satisfies the declared motion envelope AND its line of sight to the
belief mean is not BLOCKED by observed occupancy and is within camera range. The view itself is an
ordinal class BLOCKED < UNKNOWN < CLEAR: looking into unknown space is a possible information
gain, not a certified one. No candidate is ever
turned into an action here: Frozen F's (1,4) command path is untouched, and no trajectory
follower exists yet (Q02-A audit).
"""
from dataclasses import dataclass
import math

import numpy as np

from . import localmap as lm

FEASIBLE = "FEASIBLE_OBSERVED_FREE"
INFEASIBLE_DYNAMICS, INFEASIBLE_BLOCKED = "INFEASIBLE_MOTION_ENVELOPE", "INFEASIBLE_OBSERVED_OCCUPANCY"
UNCERTIFIED, NO_VIEW = "UNCERTIFIED_UNKNOWN_SPACE", "NO_VIEW_OF_BELIEF"
NO_FEASIBLE_PLAN = "NO_FEASIBLE_PLAN"
VIEW_ORDER = dict(BLOCKED=0, UNKNOWN=1, CLEAR=2)


@dataclass(frozen=True)
class CandidateParams:
    """Declared priors, not qualified against the plant."""
    bearings_deg: tuple = (-60.0, -30.0, 0.0, 30.0, 60.0, 90.0, -90.0, 180.0)
    distances_m: tuple = (1.0, 2.0)
    speed_cap_mps: float = 1.5           # h_r3 arc cap; below Frozen F's 2.5 m/s scale
    yaw_rate_cap: float = 3.0            # rad/s, Frozen F yaw_rate_max
    max_travel_s: float = 3.0
    body_clearance_m: float = 0.45       # riskcap half-width; hard radius 0.14 m is lower
    camera_range_m: float = 20.0
    camera_hfov_rad: float = math.radians(87.0)
    los_clearance_m: float = 0.15        # camera target radius


@dataclass(frozen=True)
class CandidateRecord:
    index: int
    kind: str
    viewpoint_xy: tuple
    status: str
    reasons: tuple
    path_unknown_fraction: float
    travel_time_s: float
    yaw_change_rad: float
    view_from_candidate: str            # CLEAR / UNKNOWN / BLOCKED line of sight to the belief mean
    view_from_current: str
    visibility_gain: int                # ordinal class change (-2..+2) vs the current view; not a probability
    handover_feasible: bool


def generate(position_xy, yaw, params=CandidateParams()):
    out = []
    for b in params.bearings_deg:
        for d in params.distances_m:
            a = yaw + math.radians(b)
            out.append(("VIEWPOINT_b%+04.0f_d%.1f" % (b, d), (position_xy[0] + d * math.cos(a), position_xy[1] + d * math.sin(a))))
    return tuple(out)


def _los(m, a_xy, b_xy, clearance):
    status, unknown, _ = lm.segment_status(m, a_xy, b_xy, clearance)
    return "BLOCKED" if status == lm.BLOCKED else ("CLEAR" if status == lm.SAFE else "UNKNOWN")


def evaluate(m, position_xy, yaw, belief, candidates, now_s, params=CandidateParams()):
    """Metadata for every candidate plus the summary NO_FEASIBLE_PLAN when none qualifies."""
    if not belief.valid:
        return (), NO_FEASIBLE_PLAN, ("BELIEF_INVALID",) + tuple(belief.reasons)
    goal = np.asarray(belief.mean[:2], float)
    # The LOS stops short of the belief by the LiDAR target proxy (0.20 m) plus the LOS clearance and
    # a 0.15 m guard, so the target's own first hits (kept as occupancy, G4-R5 B04) do not block it.
    # Off-axis target hits can still block a lateral LOS: the open motar-3p3.13 limitation.
    def los_from(p):
        p = np.asarray(p, float); d = goal - p; r = float(np.linalg.norm(d))
        end = p + d * max(r - (0.20 + params.los_clearance_m + 0.15), 0.0) / max(r, 1e-9)
        return _los(m, p, end, params.los_clearance_m)
    current = los_from(position_xy)
    records = []
    for i, (kind, vp) in enumerate(candidates):
        reasons = []
        path, unknown, path_reasons = lm.segment_status(m, position_xy, vp, params.body_clearance_m, now_s=now_s)
        reasons += list(path_reasons)
        dist = float(np.hypot(vp[0] - position_xy[0], vp[1] - position_xy[1]))
        face = math.atan2(goal[1] - vp[1], goal[0] - vp[0])
        dyaw = abs(math.atan2(math.sin(face - yaw), math.cos(face - yaw)))
        travel = max(dist / params.speed_cap_mps, dyaw / params.yaw_rate_cap)
        view = los_from(vp)
        in_range = float(np.linalg.norm(goal - np.asarray(vp))) <= params.camera_range_m
        handover = view != "BLOCKED" and in_range
        if travel > params.max_travel_s:
            status = INFEASIBLE_DYNAMICS; reasons.append("TRAVEL_TIME_ABOVE_ENVELOPE")
        elif path == lm.BLOCKED:
            status = INFEASIBLE_BLOCKED
        elif path == lm.UNCERTIFIED:
            status = UNCERTIFIED
        elif not handover:
            status = NO_VIEW; reasons.append("BELIEF_NOT_VIEWABLE_FROM_CANDIDATE")
        else:
            status = FEASIBLE
        gain = VIEW_ORDER[view] - VIEW_ORDER[current]
        records.append(CandidateRecord(i, kind, tuple(map(float, vp)), status, tuple(reasons), float(unknown),
                                       float(travel), float(dyaw), view, current, gain, bool(handover)))
    feasible = sum(r.status == FEASIBLE for r in records)
    return tuple(records), (None if feasible else NO_FEASIBLE_PLAN), ()
