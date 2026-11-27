"""Module B helper: forecasts strictly beyond the tracker's own prediction (CPU, pure).

The production tracker predicts once per observe and then corrects, so its delivered mean and
covariance already describe the decision time. A forecast from that state to the same time is the
identity; a forecast to an earlier time is refused; a second forecast over an interval already
covered is refused by `BeliefSnapshot.time_s` bookkeeping.
"""
from dataclasses import dataclass
import math

import numpy as np

from .history import TRACKER_MEMORY_S, cv_process_noise, cv_transition


@dataclass(frozen=True)
class BeliefSnapshot:
    mean: tuple
    cov: tuple
    time_s: float
    source: str           # TRACKER_DELIVERED or FORECAST_FROM_TRACKER
    valid: bool
    reasons: tuple


def from_tracker(inp, max_sigma_pos_m=3.0):
    mean, cov = np.asarray(inp.track_mean, float), np.asarray(inp.track_cov, float)
    reasons = []
    if not inp.track_active:
        reasons.append("TRACK_INACTIVE")
    if not (np.isfinite(mean).all() and np.isfinite(cov).all()):
        reasons.append("NONFINITE_TRACK")
    elif math.sqrt(max(np.trace(cov[:3, :3]), 0.0)) > max_sigma_pos_m:
        reasons.append("POSITION_UNCERTAINTY_TOO_LARGE")
    if inp.track_age_s > TRACKER_MEMORY_S:
        reasons.append("TRACK_AGE_BEYOND_MEMORY")
    return BeliefSnapshot(tuple(map(float, mean)), tuple(tuple(map(float, r)) for r in cov), float(inp.decision_s),
                          "TRACKER_DELIVERED", not reasons, tuple(reasons))


def forecast(snapshot, target_time_s, dt=0.1):
    horizon = round(target_time_s - snapshot.time_s, 9)
    if horizon < 0:
        raise ValueError("FORECAST_INTO_THE_PAST")
    if horizon == 0:
        return snapshot  # no second prediction of an interval the tracker already covered
    if not snapshot.valid:
        raise ValueError("FORECAST_FROM_INVALID_BELIEF")
    steps = int(round(horizon / dt))
    if abs(steps * dt - horizon) > 1e-9:
        raise ValueError("HORIZON_NOT_A_MULTIPLE_OF_TRACKER_DT")
    f, q = cv_transition(dt), cv_process_noise(dt)
    x, p = np.asarray(snapshot.mean), np.asarray(snapshot.cov)
    for _ in range(steps):
        x, p = f @ x, f @ p @ f.T + q
    return BeliefSnapshot(tuple(map(float, x)), tuple(tuple(map(float, r)) for r in p), float(target_time_s),
                          "FORECAST_FROM_TRACKER", True, ())
