"""Occlusion intervals after first acquisition, with censoring and competing outcomes (pure numpy).

An interval is a maximal run of frames where an indicator is true, counted only after the episode's
first fused acquisition. It ends either when the indicator turns false (``restored``) or because the
episode ended first; then it is right-censored and its end cause is the task outcome (``capture``,
``crash``, ``timeout``) or ``unknown`` (run shutdown/reset). A censored interval is never a short
interval: its duration is a lower bound.

* Kaplan-Meier treats every non-``restored`` end as censoring. That assumes censoring independent of
  the remaining occlusion time, which capture and crash need not satisfy (an occluded pursuer may be
  more likely to crash). Report it with the cumulative incidences, never alone.
* Aalen-Johansen cumulative incidence splits the ends by cause (restored / capture / crash /
  timeout); ``unknown`` ends are censoring.

Frame ``k`` is the state after ``k`` completed actions; times are ``k * dt``. The terminal state
(after the last action) is ``terminal_frame``; no observation is consumed there.
"""

from __future__ import annotations

from typing import Dict, List, Sequence

import numpy as np

CAUSES = ("restored", "capture", "crash", "timeout")
THRESHOLDS_S = (0.5, 1.0, 2.0, 3.0, 5.0)


def first_true(values) -> int:
    idx = np.flatnonzero(np.asarray(values, dtype=bool))
    return int(idx[0]) if len(idx) else -1


def episode_intervals(indicator, frame_index, acquisition_frame: int, terminal_frame: int, dt: float,
                      outcome) -> List[Dict]:
    """Intervals of ``indicator`` on consumed frames strictly after ``acquisition_frame``.

    ``frame_index`` must be strictly increasing and contiguous; a gap is an error (missing data must
    not shorten an interval). ``outcome`` is capture/crash/timeout or None (censored, unknown).
    """
    indicator = np.asarray(indicator, dtype=bool)
    frames = np.asarray(frame_index, dtype=np.int64)
    if len(frames) and np.any(np.diff(frames) != 1):
        raise ValueError("frame indices must be contiguous")
    if acquisition_frame < 0:
        return []
    if outcome not in CAUSES[1:] and outcome is not None:
        raise ValueError("unknown outcome %r" % (outcome,))
    if len(frames) and terminal_frame <= frames[-1]:
        raise ValueError("terminal frame must follow the last consumed frame")
    keep = frames > acquisition_frame
    active, frames = indicator[keep], frames[keep]
    out = []
    i = 0
    while i < len(active):
        if not active[i]:
            i += 1
            continue
        j = i
        while j + 1 < len(active) and active[j + 1]:
            j += 1
        start = int(frames[i])
        if j + 1 < len(active):
            end, cause = int(frames[j + 1]), "restored"
        else:
            end, cause = int(terminal_frame), outcome or "unknown"
        out.append({"start_frame": start, "end_frame": end, "start_s": start * dt,
                    "duration_s": (end - start) * dt, "end": cause, "censored": cause != "restored"})
        i = j + 1
    return out


def exposure_s(acquisition_frame: int, terminal_frame: int, dt: float) -> float:
    """Observable time after acquisition: consumed frames after the acquiring one up to termination."""
    return max(0, terminal_frame - acquisition_frame - 1) * dt if acquisition_frame >= 0 else 0.0


def reacquisition(fused, frame_index, interval: Dict, terminal_frame: int, dt: float) -> Dict:
    """Time from a restored interval's end to the next fused measurement (censored at episode end)."""
    if interval["end"] != "restored":
        return {"eligible": False}
    fused = np.asarray(fused, dtype=bool)
    frames = np.asarray(frame_index)
    later = np.flatnonzero((frames >= interval["end_frame"]) & fused)
    if len(later):
        return {"eligible": True, "time_s": (int(frames[later[0]]) - interval["end_frame"]) * dt, "censored": False}
    return {"eligible": True, "time_s": (terminal_frame - interval["end_frame"]) * dt, "censored": True}


def kaplan_meier(durations, events, at: Sequence[float]) -> Dict:
    """Survival S(t) of duration with ``events`` (True = restored); ties: events before censorings."""
    d = np.asarray(durations, dtype=np.float64)
    e = np.asarray(events, dtype=bool)
    out = {"n": int(len(d)), "events": int(e.sum())}
    if len(d) == 0:
        out.update({"S": {str(t): None for t in at}, "median_s": None})
        return out
    times = np.unique(d[e])
    s, curve = 1.0, []
    for t in times:
        n_risk = np.sum(d >= t - 1e-12)
        n_event = np.sum((np.abs(d - t) <= 1e-12) & e)
        s *= 1.0 - n_event / n_risk
        curve.append((t, s))

    def surv(t):
        value = 1.0
        for time, level in curve:
            if time <= t + 1e-12:
                value = level
        return value

    out["S"] = {str(t): surv(t) for t in at}
    median = next((time for time, level in curve if level <= 0.5), None)
    out["median_s"] = median
    out["min_risk_set_at"] = {str(t): int(np.sum(d >= t - 1e-12)) for t in at}
    return out


def aalen_johansen(durations, causes, at: Sequence[float], competing=CAUSES) -> Dict:
    """Cumulative incidence of each end cause; causes outside ``competing`` are censoring."""
    d = np.asarray(durations, dtype=np.float64)
    c = np.asarray(causes)
    result = {k: {str(t): 0.0 for t in at} for k in competing}
    if len(d) == 0:
        return result
    times = np.unique(d[np.isin(c, competing)])
    s, cif = 1.0, {k: 0.0 for k in competing}
    steps = []
    for t in times:
        n_risk = np.sum(d >= t - 1e-12)
        here = np.abs(d - t) <= 1e-12
        for k in competing:
            cif[k] += s * np.sum(here & (c == k)) / n_risk
        s *= 1.0 - np.sum(here & np.isin(c, competing)) / n_risk
        steps.append((t, dict(cif)))
    for k in competing:
        for t in at:
            value = 0.0
            for time, snapshot in steps:
                if time <= t + 1e-12:
                    value = snapshot[k]
            result[k][str(t)] = value
    return result


def summarise(intervals: List[Dict], exposures: Sequence[float], at: Sequence[float] = THRESHOLDS_S) -> Dict:
    """Group summary. ``intervals`` carry an ``episode`` key; ``exposures`` has one value per episode."""
    exposure = float(np.sum(exposures))
    durations = np.array([i["duration_s"] for i in intervals], dtype=np.float64)
    restored = np.array([i["end"] == "restored" for i in intervals], dtype=bool)
    causes = np.array([i["end"] for i in intervals], dtype=str)
    episodes = {i["episode"] for i in intervals}
    observed = {str(t): float(np.mean(durations >= t - 1e-12)) if len(durations) else None for t in at}
    return {
        "episodes": len(exposures), "exposure_s": exposure, "intervals": len(intervals),
        "episodes_with_interval": len(episodes),
        "share_episodes_with_interval": len(episodes) / len(exposures) if len(exposures) else None,
        "rate_per_exposure_s": len(intervals) / exposure if exposure > 0 else None,
        "censored_intervals": int((~restored).sum()),
        "end_causes": {k: int(np.sum(causes == k)) for k in CAUSES + ("unknown",)},
        "observed_share_at_least": observed,
        "restored_only_share_at_least": ({str(t): float(np.mean(durations[restored] >= t - 1e-12)) for t in at}
                                         if restored.any() else None),
        "kaplan_meier": kaplan_meier(durations, restored, at),
        "cumulative_incidence": aalen_johansen(durations, causes, at),
        "duration_quantiles_observed_s": ({q: float(np.quantile(durations, p)) for q, p in
                                           (("median", .5), ("p90", .9), ("p95", .95))} if len(durations) else None),
    }


def cluster_bootstrap(clusters: Dict, statistic, resamples: int = 1000, seed: int = 0, alpha: float = 0.05):
    """Percentile interval of ``statistic(list_of_cluster_values)`` resampling whole clusters."""
    keys = sorted(clusters)
    if len(keys) < 2:
        return {"clusters": len(keys), "interval": None, "note": "fewer than two clusters"}
    rng = np.random.default_rng(seed)
    values = []
    for _ in range(resamples):
        pick = rng.integers(0, len(keys), len(keys))
        value = statistic([clusters[keys[i]] for i in pick])
        if value is not None:
            values.append(value)
    if not values:
        return {"clusters": len(keys), "interval": None}
    lo, hi = np.quantile(values, [alpha / 2, 1 - alpha / 2])
    return {"clusters": len(keys), "resamples": resamples, "seed": seed, "interval": [float(lo), float(hi)]}


__all__ = ["CAUSES", "THRESHOLDS_S", "first_true", "episode_intervals", "exposure_s", "reacquisition",
           "kaplan_meier", "aalen_johansen", "summarise", "cluster_bootstrap"]
