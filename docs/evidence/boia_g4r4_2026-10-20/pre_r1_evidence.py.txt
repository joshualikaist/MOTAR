"""C1/C2 measured-support hypotheses and explicit interval provenance; no sensor handles."""
import math

from .types import EvidenceSet, UtilityValidity


def _finite(value):
    try:
        return (isinstance(value, (int, float)) and not isinstance(value, bool)
                and math.isfinite(value))
    except (OverflowError, TypeError):
        return False


def _interval(value):
    return (isinstance(value, tuple) and len(value) == 2 and all(_finite(x) for x in value)
            and 0 <= value[0] <= value[1])


def _proof(proof, domain):
    return (proof is not None and proof.kind in ("EXACT_SYNTHETIC", "HELD_OUT_DECLARATION")
            and proof.supported is True and isinstance(proof.reference, str) and bool(proof.reference.strip())
            and isinstance(domain, str) and bool(domain.strip()) and proof.domain == domain)


def validate_probability(interval, domain):
    """Validate a supplied declaration, NOT empirical calibration or conditional coverage."""
    if interval is None or not _interval((interval.lower, interval.upper)):
        return UtilityValidity(False, "MISSING_OR_INVALID_PROBABILITY_INTERVAL", domain)
    if interval.upper > 1:
        return UtilityValidity(False, "PROBABILITY_OUTSIDE_UNIT_INTERVAL", domain)
    if not _proof(interval.provenance, domain):
        return UtilityValidity(False, "UNSUPPORTED_CALIBRATION_DECLARATION", domain)
    return UtilityValidity(True, "SUPPORTED_DECLARATION_NOT_INDEPENDENTLY_VERIFIED", domain)


def build_hypotheses(measurement, domain):
    """Retain nonexclusive interpretations; first-hit evidence is never deleted.

    Aligned frames/error intervals and model exhaustiveness are caller declarations.
    Missing information yields UNKNOWN, not an invented world or confidence interval.
    """
    hit, surface = measurement.first_hit_m, measurement.camera_surface_m
    hits = (hit,) if _interval(hit) else ()
    labels, reasons = [], []
    if not _interval(hit):
        reasons.append("MISSING_OR_INVALID_FIRST_HIT")
    if not _proof(measurement.provenance, domain):
        reasons.append("UNSUPPORTED_MEASUREMENT_DECLARATION")
    times = (measurement.capture_s, measurement.arrival_s, measurement.now_s)
    if (not all(_finite(x) for x in times) or not _finite(measurement.max_age_s)
            or measurement.max_age_s < 0 or not times[0] <= times[1] <= times[2]
            or times[2] - times[0] > measurement.max_age_s):
        reasons.append("MISSING_STALE_OR_INVALID_TIMESTAMPS")
    if measurement.frame_aligned is not True:
        reasons.append("FRAME_ALIGNMENT_UNAVAILABLE")
    if _interval(hit) and _interval(surface) and measurement.same_ray_support is True:
        if hit[0] <= surface[1] and surface[0] <= hit[1]:
            labels.extend(("TARGET_SURFACE", "FOREGROUND_OCCLUDER"))
        elif hit[1] < surface[0]:
            labels.append("FOREGROUND_OCCLUDER")
        else:
            labels.append("FALSE_ASSOCIATION")
            reasons.append("RANGE_SUPPORT_DISAGREEMENT")
    else:
        labels.extend(("TARGET_SURFACE", "FOREGROUND_OCCLUDER"))
        reasons.append("RANGE_OR_COMMON_RAY_SUPPORT_UNAVAILABLE")
    association = validate_probability(measurement.association, domain)
    if not association.valid or measurement.association.lower < 1:
        labels.append("FALSE_ASSOCIATION")
    if not association.valid:
        reasons.append(association.reason)
    if measurement.model_exhaustive is not True:
        reasons.append("HYPOTHESIS_FAMILY_NOT_EXHAUSTIVE")
    if reasons:
        labels.append("UNKNOWN_OR_UNOBSERVED")
    return EvidenceSet(measurement.key, tuple(sorted(set(labels))), hits,
                       not reasons, tuple(reasons), domain)
