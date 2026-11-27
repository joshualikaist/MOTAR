"""Strict independent nominal CPU safety declarations, never physical certificates.

No utility/benefit record, task handle or model evaluation enters this validator.
The caller must supply prefix and complete-backup evidence. This module checks
bindings, scope and freshness, not whether a real sensor/plant supports the proof.
"""
from .evidence import _finite
from .types import SafetyContext, SafetyEvidence, SafetyProvenance, SafetyValidity


def _text(value):
    return isinstance(value, str) and bool(value.strip())


def validate_safety(context, key, candidate_id, wait_s, evidence):
    """Prefix horizon starts at issuance and covers F wait + next action interval.

    Expiry limits using this prediction NOW; a future option is advisory and must
    be re-evaluated at its execution time. Prefix/backup references and nominal
    qualification are declarations, not calibrated physical or continuous proofs.
    """
    def fail(reason):
        return SafetyValidity(False, reason)

    if not isinstance(context, SafetyContext):
        return fail("SAFETY_CONTEXT_MISSING")
    if (not isinstance(key, tuple) or len(key) != 3
            or not all(_text(x) for x in key[:2]) or not isinstance(key[2], int)
            or isinstance(key[2], bool) or key[2] < 0
            or context.key != key or not _text(candidate_id) or not _finite(wait_s) or wait_s < 0
            or not all(_text(x) for x in (context.sensor_version, context.boundary_version, context.model_id))
            or not all(_finite(x) and x >= 0 for x in (context.now_s, context.sensor_timestamp_s,
                       context.boundary_timestamp_s, context.max_age_s))
            or not _finite(context.decision_interval_s) or context.decision_interval_s <= 0):
        return fail("SAFETY_CONTEXT_INVALID_OR_UNBOUND")
    if not isinstance(evidence, SafetyEvidence):
        return fail("SAFETY_EVIDENCE_MISSING")
    if (evidence.key != key or evidence.candidate_id != candidate_id
            or not _finite(evidence.wait_s) or evidence.wait_s != wait_s):
        return fail("SAFETY_IDENTITY_MISMATCH")
    if (evidence.sensor_version != context.sensor_version
            or evidence.boundary_version != context.boundary_version
            or evidence.model_id != context.model_id
            or evidence.sensor_timestamp_s != context.sensor_timestamp_s
            or evidence.boundary_timestamp_s != context.boundary_timestamp_s):
        return fail("SAFETY_SENSOR_BOUNDARY_OR_MODEL_MISMATCH")
    stamps = (evidence.sensor_timestamp_s, evidence.boundary_timestamp_s,
              evidence.issued_at_s, evidence.expires_at_s)
    if (not all(_finite(x) and x >= 0 for x in stamps)
            or max(stamps[:2]) > evidence.issued_at_s
            or not evidence.issued_at_s <= context.now_s <= evidence.expires_at_s
            or any(context.now_s - stamp > context.max_age_s for stamp in stamps[:2])):
        return fail("SAFETY_STALE_OR_INVALID_TIMESTAMP")
    needed = context.now_s - evidence.issued_at_s + wait_s + context.decision_interval_s
    if (evidence.prefix_safe is not True or evidence.backup_feasible is not True
            or not _finite(evidence.prefix_horizon_s) or not _finite(needed)
            or evidence.prefix_horizon_s < needed
            or not _text(evidence.prefix_reference) or not _text(evidence.backup_reference)):
        return fail("SAFETY_PREFIX_OR_COMPLETE_BACKUP_UNSUPPORTED")
    proof = evidence.provenance
    if (evidence.model_qualification != "NOMINAL_CPU_MODEL_ONLY"
            or not isinstance(proof, SafetyProvenance) or proof.supported is not True
            or proof.kind not in ("EXACT_SYNTHETIC", "NOMINAL_CPU_ORACLE_DECLARATION")
            or not _text(proof.reference) or proof.model_id != context.model_id):
        return fail("SAFETY_PROVENANCE_OR_QUALIFICATION_UNSUPPORTED")
    return SafetyValidity(True, "CURRENT_NOMINAL_CPU_DECLARATION_ONLY")
