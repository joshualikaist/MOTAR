"""Module E inputs: the per-decision timing feature record (CPU, pure). No utility is estimated.

Every feature is AVAILABLE with a value and provenance, or UNAVAILABLE with a reason. Nothing is
filled from simulator ground truth. Delta-U = E[U|REACQUIRE] - E[U|CONTINUE] - C_switch is
NOT_ESTIMATED until matched counterfactual outcomes exist (BOIA_TIMING_EXPERIMENT_PLAN.md).
"""
from dataclasses import dataclass
import math

import numpy as np

from . import candidates as cd
from .geometry import quat_yaw_xyzw, world_to_vehicle, yaw_rotation
from .history import ORACLE_PROXY

AVAILABLE, UNAVAILABLE, NOT_ESTIMATED = "AVAILABLE", "UNAVAILABLE", "NOT_ESTIMATED"
FEATURES = ("ever_acquired", "time_since_last_detection", "tracker_active", "tracker_age", "position_uncertainty",
            "relative_target_position", "relative_target_velocity", "occlusion_evidence", "fov_status",
            "sensor_validity", "local_clearance", "candidate_count", "candidate_visibility_gain",
            "candidate_travel_time", "candidate_unknown_exposure", "recent_loss_events", "recent_switch_count",
            "detector_provenance", "identity_evidence")


@dataclass(frozen=True)
class Feature:
    name: str
    status: str
    value: object
    provenance: str


def _f(name, value, provenance, reason=None):
    if value is None:
        return Feature(name, UNAVAILABLE, None, reason or provenance)
    return Feature(name, AVAILABLE, value, provenance)


def extract(inp, record, loss, local_map, candidate_records, plan_status, recent_switches):
    """One immutable tuple of 18 Features, in FEATURES order, plus Delta-U NOT_ESTIMATED."""
    track_ok = "track:" not in " ".join(record.validation_problems) and inp.track_active
    yaw = quat_yaw_xyzw(inp.vehicle_quat) if "pose:" not in " ".join(record.validation_problems) else None
    rel_pos = rel_vel = None
    if track_ok and yaw is not None:
        rel_pos = tuple(float(x) for x in world_to_vehicle(np.asarray(inp.track_mean[:3]), inp.position, yaw))
        rel_vel = tuple(float(x) for x in (np.asarray(inp.track_mean[3:]) - np.asarray(inp.velocity)) @ yaw_rotation(yaw))
    sigma = float(math.sqrt(max(np.trace(np.asarray(inp.track_cov)[:3, :3]), 0.0))) if track_ok else None
    fov = None
    if record.fov_points_total:
        fov = ("INSIDE" if record.fov_points_inside == record.fov_points_total
               else "OUTSIDE" if record.fov_points_inside == 0 else "BOUNDARY")
    occ = dict(hypothesis=record.hypothesis, reasons=record.reasons,
               camera=dict(record.camera.__dict__), lidar=dict(record.lidar.__dict__),
               sensor_disagreement=any(r.startswith("SENSOR_DISAGREEMENT") for r in record.reasons),
               near_obstacle_warning=record.near_obstacle_warning)
    clearance = None
    if local_map is not None:
        d = (np.min(np.linalg.norm(local_map.hits_xy - np.asarray(local_map.origin_xy), axis=1))
             if local_map.hits_xy.size else None)
        clearance = dict(nearest_observed_hit_m=None if d is None else float(d),
                         forward_known_free_m=float(np.min(local_map.radii[np.abs(local_map.beam_az) <= math.radians(45)])))
    feasible = [c for c in candidate_records if c.status == cd.FEASIBLE]
    nonblocked = [c for c in candidate_records if c.status not in (cd.INFEASIBLE_BLOCKED, cd.INFEASIBLE_DYNAMICS)]
    gains = [c.visibility_gain for c in feasible]
    cand_reason = "NO_CANDIDATES_EVALUATED" if not candidate_records else plan_status or "NO_FEASIBLE_PLAN"
    feats = (
        _f("ever_acquired", bool(loss.ever_acquired), "DEBOUNCED_CAMERA_DETECTIONS_WITH_TRACK_CONSISTENCY"),
        _f("time_since_last_detection", loss.time_since_last_detection_s, "HISTORY_DECISION_CLOCK", "NEVER_ACQUIRED"),
        _f("tracker_active", bool(inp.track_active), "TRACKER_DELIVERED"),
        _f("tracker_age", float(inp.track_age_s) if track_ok else None, "TRACKER_AGE_SINCE_FUSED_CORRECTION", "TRACK_UNUSABLE"),
        _f("position_uncertainty", None if sigma is None else dict(sqrt_trace_m=sigma, sigma_range_m=record.sigma_range_m,
           sigma_lateral_m=record.sigma_lateral_m), "TRACKER_COVARIANCE", "TRACK_UNUSABLE"),
        _f("relative_target_position", rel_pos, "TRACKER_MEAN_IN_YAW_VEHICLE_FRAME", "TRACK_OR_POSE_UNUSABLE"),
        _f("relative_target_velocity", rel_vel, "TRACKER_MINUS_OWN_VELOCITY_YAW_VEHICLE", "TRACK_OR_POSE_UNUSABLE"),
        _f("occlusion_evidence", occ, "Q02A_EVIDENCE_RULE_UNCALIBRATED"),
        _f("fov_status", fov, "BELIEF_SIGMA_POINTS_VS_PINHOLE", "NO_BELIEF_GEOMETRY"),
        _f("sensor_validity", dict(problems=record.validation_problems), "Q02A_VALIDATION"),
        _f("local_clearance", clearance, "LIDAR_SINGLE_SCAN_KNOWN_FREE", "NO_LOCAL_MAP"),
        _f("candidate_count", len(feasible) if candidate_records else None, "Q02B_VIEWPOINT_LATTICE_METADATA", cand_reason),
        _f("candidate_visibility_gain", max(gains) if gains else None, "ORDINAL_VIEW_CLASS_CHANGE_BLOCKED_UNKNOWN_CLEAR", cand_reason),
        _f("candidate_travel_time", min(c.travel_time_s for c in feasible) if feasible else None,
           "DECLARED_SPEED_AND_YAW_CAPS", cand_reason),
        _f("candidate_unknown_exposure", min(c.path_unknown_fraction for c in nonblocked) if nonblocked else None,
           "PATH_SAMPLE_FRACTION_IN_UNKNOWN", cand_reason),
        _f("recent_loss_events", int(loss.recent_loss_events), "DEBOUNCED_HISTORY_WINDOW"),
        _f("recent_switch_count", int(recent_switches), "SHADOW_STATE_MACHINE_WINDOW"),
        _f("detector_provenance", dict(lineage=loss.detection_provenance, realism="NOT_REALISTIC_ORACLE_PROXY"
           if loss.detection_provenance == ORACLE_PROXY else "UNQUALIFIED"), "CALLER_DECLARED_LINEAGE",
           "NO_ACQUISITION_DETECTION"),
        _f("identity_evidence", dict(detection_support=record.detection_support, geometry_support=record.geometry_support,
           identity_verdict=record.identity_verdict, identity_confirmation=record.identity_confirmation,
           tracker_association=record.tracker_association, sphere_fit_status=record.sphere_fit_status,
           detector_provenance=record.detector_provenance, acquisition_kind=loss.acquisition_kind),
           "Q02A_V2_6_IDENTITY_CONTRACT"),
    )
    assert tuple(f.name for f in feats) == FEATURES
    return feats, Feature("delta_u", NOT_ESTIMATED, None, "NO_COUNTERFACTUAL_OUTCOME_DATA")
