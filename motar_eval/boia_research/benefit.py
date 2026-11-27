"""C3 exact paired arithmetic and declaration-conditional error accounting, not mission value."""
import math

from .evidence import _finite, _proof
from .types import BenefitResult


def paired_bound(candidate_id, wait_s, baseline, candidate, contract, allowances):
    """Rows share env/episode/decision, scenario IDs and path IDs exactly.

    No loss is generated here. Caller-supplied losses are surrogates, not actual F
    continuation/capture estimates. All zero allowances require explicit provenance.
    Invalidity affects this candidate option only; callers may retain valid siblings.
    """
    def fail(reason):
        return BenefitResult(baseline.key, candidate_id, wait_s, False, reason,
                             domain=contract.domain)

    if not isinstance(candidate_id, str) or not candidate_id or not _finite(wait_s) or wait_s < 0:
        return fail("INVALID_OPTION_ID_OR_WAIT")
    if (baseline.key != candidate.key or baseline.scenario_ids != candidate.scenario_ids
            or baseline.path_ids != candidate.path_ids):
        return fail("UNPAIRED_ENV_EPISODE_DECISION_SCENARIO_OR_PATH")
    b, m = len(baseline.scenario_ids), len(baseline.path_ids)
    if (min(b, m) == 0 or len(set(baseline.scenario_ids)) != b
            or len(set(baseline.path_ids)) != m):
        return fail("EMPTY_OR_DUPLICATE_SCENARIO_OR_PATH_IDS")
    for table in (baseline, candidate):
        if (len(table.values) != b or any(len(row) != m for row in table.values)
                or not all(_finite(x) for row in table.values for x in row)):
            return fail("INVALID_OPTION_LOSS_TABLE")
    if not _proof(contract.provenance, contract.domain):
        return fail("CALIBRATION_UNAVAILABLE")
    parts = (allowances.mc, allowances.set_coverage, allowances.calibration,
             allowances.continuation, allowances.switching)
    if not all(_finite(x) and x >= 0 for x in parts):
        return fail("MISSING_OR_INVALID_ERROR_ALLOWANCE")
    if contract.provenance.continuation_covered is not True:
        return fail("F_CONTINUATION_MISMATCH_UNBOUNDED")
    if (not isinstance(contract.tested_options, int) or isinstance(contract.tested_options, bool)
            or not _finite(contract.tested_options) or contract.tested_options <= 0):
        return fail("INVALID_SELECTION_FAMILY")
    exact = contract.provenance.kind == "EXACT_SYNTHETIC"
    if exact:
        if (contract.sampling != "ENUMERATED" or contract.exhaustive is not True
                or any(x != 0 for x in parts[:4])):
            return fail("EXACT_SYNTHETIC_JUSTIFICATION_INCONSISTENT")
    else:
        if contract.exhaustive is not True and contract.set_coverage_proven is not True:
            return fail("FINITE_SCENARIOS_ONLY_NOT_A_COVERAGE_BOUND")
        if (contract.sampling != "IID_PATHS" or not _finite(contract.alpha)
                or not 0 < contract.alpha < 1 or not _finite(contract.difference_bound)
                or contract.difference_bound <= 0):
            return fail("MC_ASSUMPTIONS_UNAVAILABLE")
        log_family = math.log(2) + math.log(contract.tested_options) + math.log(b) - math.log(contract.alpha)
        required_mc = contract.difference_bound * math.sqrt(2 * log_family / m)
        if allowances.mc < required_mc:
            return fail("MC_ALLOWANCE_OMITS_SIMULTANEOUS_SELECTION")
    differences = tuple(tuple(f - g for f, g in zip(fr, gr))
                        for fr, gr in zip(baseline.values, candidate.values))
    if not exact and any(abs(d) > contract.difference_bound for row in differences for d in row):
        return fail("DECLARED_DIFFERENCE_BOUND_VIOLATED")
    try:
        means = tuple(math.fsum(row) / m for row in differences)
        raw = min(means)
        lower = raw - math.fsum(parts)
    except (OverflowError, ValueError):
        return fail("NONFINITE_PAIRED_ARITHMETIC")
    if not _finite(lower):
        return fail("NONFINITE_PAIRED_ARITHMETIC")
    return BenefitResult(baseline.key, candidate_id, wait_s, True,
                         "EXACT_SYNTHETIC_SURROGATE" if exact else "DECLARATION_CONDITIONAL_BOUND",
                         lower, raw, means, contract.covered_hypotheses,
                         contract.tested_options, not exact, allowances, contract.domain)
