"""Q02-B per-episode observation history, eligibility gate and track-loss events (CPU, pure).

History entries are built only from Q02-A evidence records and the copied tracker fields of the
same EvidenceInput. Nothing here reads simulator state, and nothing returns an action.

Q02A-v2.6 contract (Q02-B integration, 2026-10-22): a decision counts toward (re)acquisition only if
the camera flag is set, its footprint geometry is FOOTPRINT_CONSISTENT and the identity verdict is
CONSISTENT_NOT_CONFIRMED, and the tracker passes the continuity gate. Acquisition is therefore
PROVISIONAL: a footprint-consistent detection, never a same-target confirmation. An AMBIGUOUS or
CONTRADICTED identity marks the decision identity-suspect; a flagged detection whose identity is
NOT_TESTABLE is "unverified" and counts as neither hit nor confirmation.
"""
from dataclasses import dataclass, field
import math

import numpy as np

from .evidence import TARGET_VISIBLE   # noqa: F401  (kept for callers of the v1 history API)

# Detector lineages are reported, never merged (Q04 keeps them as separate conditions).
ORACLE_PROXY = "FROZEN_F_ORACLE_PROXY_RENDER"      # true-pose rasterized target, unique colour
ERROR_INJECTED = "FROZEN_F_PROXY_WITH_ERROR_INJECTION"
IMAGE_DETECTOR = "IMAGE_BASED_DETECTOR"
PROVENANCES = (ORACLE_PROXY, ERROR_INJECTED, IMAGE_DETECTOR)
# Caller lineage -> the Q02-A record's detector_provenance it must agree with (None: not yet defined).
RECORD_PROVENANCE = {ORACLE_PROXY: "ORACLE_TRUE_POSE_RASTER", ERROR_INJECTED: None, IMAGE_DETECTOR: None}
ACQUISITION_KIND = "PROVISIONAL_FOOTPRINT_CONSISTENT_NOT_IDENTITY_CONFIRMED"
TRACKER_DT_S = 0.1
TRACKER_MEMORY_S = 5.0
CHI2_3DOF_999 = 16.266   # consistency gate on consecutive track means; a declared prior


def cv_transition(dt):
    f = np.eye(6)
    f[0, 3] = f[1, 4] = f[2, 5] = dt
    return f


def cv_process_noise(dt):
    """Identical to navrl_perception.BatchedConstantVelocityTracker and boia.belief.tracker_Q."""
    return np.diag([0.025 * dt * dt] * 3 + [0.25 * dt] * 3)


@dataclass(frozen=True)
class HistoryEntry:
    episode: int
    decision: int
    decision_s: float
    detected: bool
    usable_detection: bool
    hypothesis: str
    provenance: str
    track_active: bool
    track_age_s: float
    track_mean: tuple
    track_cov: tuple             # full 6x6, row-major tuples
    sigma_pos_m: float
    identity_suspect: bool
    consistency_nis: object
    validation_problems: tuple
    informative: bool            # camera input usable: a miss can only be counted on such a decision
    geometry_support: str = "NOT_APPLICABLE"
    identity_verdict: str = "NOT_APPLICABLE"
    sphere_fit_status: str = "NOT_RUN"
    record_provenance: str = ""
    identity_unverified: bool = False   # flag set but identity NOT_TESTABLE: neither hit nor confirmation


@dataclass(frozen=True)
class HistoryParams:
    """Declared priors, not calibrated."""
    acquire_consecutive: int = 2       # detections needed before the target counts as acquired
    loss_consecutive: int = 2          # missed decisions before a loss event is declared
    reacquire_consecutive: int = 2     # detections needed to end a loss
    window_s: float = 10.0             # look-back for recent event counts
    dt_tol_s: float = 1e-6


class HistoryError(RuntimeError):
    pass


class ObservationHistory:
    """Append-only, one episode at a time, strictly one entry per 0.1 s decision."""

    def __init__(self, params=HistoryParams()):
        self.params, self.entries, self.episode = params, [], None

    def reset(self, episode):
        if self.episode is not None and episode <= self.episode:
            raise HistoryError("EPISODE_ID_NOT_INCREASING")
        self.episode, self.entries = int(episode), []

    def append(self, episode, decision, inp, record, provenance):
        if self.episode is None or episode != self.episode:
            raise HistoryError("APPEND_OUTSIDE_CURRENT_EPISODE")
        if provenance not in PROVENANCES:
            raise HistoryError("UNKNOWN_DETECTOR_PROVENANCE")
        expected = RECORD_PROVENANCE[provenance]
        if expected is None or getattr(record, "detector_provenance", None) != expected:
            raise HistoryError("DETECTOR_PROVENANCE_MISMATCH")   # lineage must agree with the evidence record
        if self.entries:
            last = self.entries[-1]
            if decision != last.decision + 1:
                raise HistoryError("DECISION_INDEX_NOT_CONSECUTIVE")
            if abs(inp.decision_s - last.decision_s - TRACKER_DT_S) > self.params.dt_tol_s:
                raise HistoryError("DECISION_CLOCK_INCONSISTENT")
        elif decision != 0:
            raise HistoryError("FIRST_DECISION_MUST_BE_ZERO")
        mean, cov = np.asarray(inp.track_mean, float), np.asarray(inp.track_cov, float)
        nis, suspect = None, False
        if self.entries and inp.track_active and self.entries[-1].track_active and np.isfinite(mean).all():
            prev = self.entries[-1]
            f, q = cv_transition(TRACKER_DT_S), cv_process_noise(TRACKER_DT_S)
            pred = f @ np.asarray(prev.track_mean)
            prev_cov = np.asarray(prev.track_cov)
            # A filter update moves the mean by K*nu, whose covariance P_pred - P_t is bounded by
            # P_pred; adding P_t keeps the gate conservative (fewer false identity flags).
            s = (f @ prev_cov @ f.T + q)[:3, :3] + cov[:3, :3]
            d = mean[:3] - pred[:3]
            try:
                nis = float(d @ np.linalg.solve(s, d))
                suspect = nis > CHI2_3DOF_999
            except np.linalg.LinAlgError:
                nis, suspect = None, True
        sigma = float(math.sqrt(max(np.trace(cov[:3, :3]), 0.0))) if np.isfinite(cov).all() else float("inf")
        camera_inputs = ("camera_clock", "rgb", "depth", "pose", "calibration")
        informative = not any(pr.split(":")[0] in camera_inputs or pr.startswith("track:INVALID")
                              for pr in record.validation_problems)
        flagged = record.detection_support == "CAMERA_FLAG"
        identity_bad = flagged and record.identity_verdict in ("AMBIGUOUS", "CONTRADICTED")
        unverified = flagged and record.identity_verdict == "NOT_TESTABLE"
        usable = bool(informative and flagged and record.geometry_support == "FOOTPRINT_CONSISTENT"
                      and record.identity_verdict == "CONSISTENT_NOT_CONFIRMED" and not suspect)
        entry = HistoryEntry(int(episode), int(decision), float(inp.decision_s), bool(inp.camera_detected), usable,
                             record.hypothesis, provenance, bool(inp.track_active), float(inp.track_age_s),
                             tuple(float(x) for x in mean), tuple(tuple(float(v) for v in row) for row in cov), sigma,
                             bool(suspect or identity_bad), nis, tuple(record.validation_problems), bool(informative),
                             record.geometry_support, record.identity_verdict, record.sphere_fit_status,
                             record.detector_provenance, bool(unverified))
        self.entries.append(entry)
        return entry


@dataclass(frozen=True)
class LossState:
    ever_acquired: bool
    acquisition_decision: object
    detection_provenance: object
    currently_lost: bool
    loss_onset_decision: object
    time_since_last_detection_s: object
    loss_duration_s: object
    recent_loss_events: int
    recent_reacquisitions: int
    identity_suspect_recent: bool
    raw_detection_now: bool
    acquisition_kind: object = None          # ACQUISITION_KIND once acquired; never an identity confirmation
    reacquisition_kind: object = None        # same, for the last reacquisition
    identity_unverified_now: bool = False


def _runs(flags):
    out, start = [], None
    for i, f in enumerate(flags):
        if f and start is None:
            start = i
        if not f and start is not None:
            out.append((start, i - 1)); start = None
    if start is not None:
        out.append((start, len(flags) - 1))
    return out


def loss_state(history):
    """Eligibility (previously acquired) and debounced loss/reacquisition events from the history."""
    p, e = history.params, history.entries
    if not e:
        return LossState(False, None, None, False, None, None, None, 0, 0, False, False)
    unverified_now = e[-1].identity_unverified
    ok = [x.usable_detection for x in e]
    runs = _runs(ok)
    acquired = [r for r in runs if r[1] - r[0] + 1 >= p.acquire_consecutive]
    if not acquired:
        return LossState(False, None, None, False, None, None, None, 0, 0,
                         any(x.identity_suspect for x in e), e[-1].detected, identity_unverified_now=unverified_now)
    acq = acquired[0][0] + p.acquire_consecutive - 1
    now = len(e) - 1
    last_det = max(i for i, f in enumerate(ok) if f)
    # Debounced segments after acquisition: a loss needs loss_consecutive misses, a reacquisition
    # needs reacquire_consecutive detections.
    lost, onset, events, reacq = False, None, [], []
    miss = hit = 0
    for i in range(acq + 1, len(e)):
        if not e[i].informative:
            continue  # no camera information: neither a hit nor a miss
        if ok[i]:
            hit, miss = hit + 1, 0
            if lost and hit >= p.reacquire_consecutive:
                lost = False; reacq.append(i)
        else:
            miss, hit = miss + 1, 0
            if not lost and miss >= p.loss_consecutive:
                lost, onset = True, i - miss + 1; events.append(onset)
    t = e[now].decision_s
    recent = lambda idx: sum(1 for i in idx if t - e[i].decision_s <= p.window_s)
    since = round(t - e[last_det].decision_s, 9)
    return LossState(True, acq, e[acq].provenance, lost, onset if lost else None, since,
                     round(t - e[onset].decision_s + TRACKER_DT_S, 9) if lost else None,
                     recent(events), recent(reacq),
                     any(x.identity_suspect for x in e if t - x.decision_s <= p.window_s), e[now].detected,
                     ACQUISITION_KIND, ACQUISITION_KIND if reacq else None, unverified_now)
