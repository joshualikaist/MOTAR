"""Q02-B/Q03 CPU shadow benchmark metrics (pure). Inputs are shadow decision rows and DECLARED synthetic labels.

The labels are fixture declarations of when an intervention would be warranted in a scripted synthetic scene;
they are not simulator episode outcomes, and none of these metrics is a capture rate or a causal effect.
"""
from dataclasses import dataclass
import math

PROPOSE = "PROPOSE_REACQUIRE_SHADOW"
LABEL_SEMANTICS = "DECLARED_SYNTHETIC_FIXTURE_LABEL_NOT_A_SIMULATOR_OUTCOME"


@dataclass(frozen=True)
class Row:
    scenario: str
    policy: str
    decision_index: int
    time_s: float
    decision: str
    reasons: tuple
    hypothesis: str
    identity_verdict: str
    map_accepted: bool
    map_stale: bool
    plan_status: object
    l2_with_gain: int
    in_loss: bool              # shadow machine was in LOSS_SUSPECTED / REACQUIRE_PLANNED after the step
    warranted: bool            # declared label for this decision
    runtime_s: float


def _quantile(xs, q):
    if not xs:
        return None
    s = sorted(xs)
    k = (len(s) - 1) * q
    lo, hi = int(math.floor(k)), int(math.ceil(k))
    return s[lo] + (s[hi] - s[lo]) * (k - lo)


def summarize(rows):
    """Per policy metrics over all scenarios (never pooled across policies)."""
    out = {}
    for policy in sorted({r.policy for r in rows}):
        rs = [r for r in rows if r.policy == policy]
        scen = sorted({r.scenario for r in rs})
        trig = [r for r in rs if r.decision == PROPOSE]
        latency, missed, unnecessary_scen = [], [], []
        for s in scen:
            srs = sorted((r for r in rs if r.scenario == s), key=lambda r: r.decision_index)
            first_w = next((r for r in srs if r.warranted), None)
            first_t = next((r for r in srs if r.decision == PROPOSE and r.warranted), None)
            if first_w is not None and first_t is None:
                missed.append(s)
            if first_w is not None and first_t is not None:
                latency.append(round(first_t.time_s - first_w.time_s, 9))
            if any(r.decision == PROPOSE and not r.warranted for r in srs):
                unnecessary_scen.append(s)
        loss = [r for r in rs if r.in_loss]
        runtimes = [r.runtime_s for r in rs]
        out[policy] = dict(
            decisions=len(rs), scenarios=len(scen), trigger_count=len(trig),
            unnecessary_triggers=sum(1 for r in trig if not r.warranted), unnecessary_trigger_scenarios=unnecessary_scen,
            warranted_scenarios=sum(1 for s in scen if any(r.warranted for r in rs if r.scenario == s)),
            missed_trigger_scenarios=missed, trigger_latency_s=latency,
            abstain_count=sum(1 for r in rs if r.decision == "ABSTAIN"),
            invalid_input_rejections=sum(1 for r in rs if "INPUT_INVALID" in r.reasons),
            loss_decisions=len(loss), loss_decisions_with_candidate=sum(1 for r in loss if r.l2_with_gain > 0),
            runtime_s=dict(median=_quantile(runtimes, 0.5), p95=_quantile(runtimes, 0.95), max=max(runtimes) if runtimes else None),
            label_semantics=LABEL_SEMANTICS)
    return out
