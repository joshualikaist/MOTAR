"""Q02-B shadow state machine, ABSTAIN reason codes and the shadow decision record (CPU, pure).

Shadow only: Frozen F always owns the command, no action is produced, and the executing states
are unreachable because no executor exists. ABSTAIN expresses that the evidence cannot support a
decision; it is NOT a safe control command. Baseline triggers (F0, H1, H2, H3) are reference rules
for the Q04 comparison and may only PROPOSE in shadow; the BOIA rule needs Delta-U, which is
NOT_ESTIMATED, so it never proposes.
"""
from dataclasses import dataclass

from .evidence import IDENTITY_AMBIGUOUS, OCCLUDED, UNKNOWN as EVIDENCE_UNKNOWN

PRE_ACQUISITION, TRACK_F, LOSS_SUSPECTED = "PRE_ACQUISITION", "TRACK_F", "LOSS_SUSPECTED"
REACQUIRE_PLANNED, REACQUIRE_EXECUTING, HANDOVER_TO_F = "REACQUIRE_PLANNED", "REACQUIRE_EXECUTING", "HANDOVER_TO_F"
REACQUISITION_FAILED, TERMINAL_CAPTURE, TERMINAL_FAILURE = "REACQUISITION_FAILED", "TERMINAL_CAPTURE", "TERMINAL_FAILURE"
STATES = (PRE_ACQUISITION, TRACK_F, LOSS_SUSPECTED, REACQUIRE_PLANNED, REACQUIRE_EXECUTING, HANDOVER_TO_F,
          REACQUISITION_FAILED, TERMINAL_CAPTURE, TERMINAL_FAILURE)
TERMINAL = (TERMINAL_CAPTURE, TERMINAL_FAILURE)
ALLOWED = {
    PRE_ACQUISITION: {TRACK_F},
    TRACK_F: {LOSS_SUSPECTED},
    LOSS_SUSPECTED: {TRACK_F, REACQUIRE_PLANNED},
    REACQUIRE_PLANNED: {TRACK_F, LOSS_SUSPECTED, REACQUIRE_EXECUTING},
    REACQUIRE_EXECUTING: {HANDOVER_TO_F, REACQUISITION_FAILED},
    HANDOVER_TO_F: {TRACK_F},
    REACQUISITION_FAILED: {LOSS_SUSPECTED, TRACK_F},
    TERMINAL_CAPTURE: set(), TERMINAL_FAILURE: set()}

CONTINUE_F, ABSTAIN, PROPOSE_REACQUIRE_SHADOW = "CONTINUE_F", "ABSTAIN", "PROPOSE_REACQUIRE_SHADOW"
ABSTAIN_CODES = ("NOT_PREVIOUSLY_ACQUIRED", "INPUT_INVALID", "EVIDENCE_UNKNOWN", "IDENTITY_SUSPECT", "IDENTITY_UNVERIFIED",
                 "LOCAL_MAP_UNAVAILABLE", "BELIEF_INVALID",
                 "NO_FEASIBLE_PLAN", "UTILITY_NOT_ESTIMATED", "MIN_DWELL_NOT_MET", "COOLDOWN_ACTIVE",
                 "SWITCH_BUDGET_EXHAUSTED", "EXECUTOR_NOT_AVAILABLE", "TERMINAL")


class TransitionError(RuntimeError):
    pass


def validate_transition(before, after):
    if before not in STATES or after not in STATES:
        raise TransitionError("UNKNOWN_STATE")
    if before == after:
        if before in TERMINAL:
            return
        return
    if before in TERMINAL:
        raise TransitionError("TERMINAL_IS_ABSORBING")
    if after in TERMINAL:
        return
    if after not in ALLOWED[before]:
        raise TransitionError("ILLEGAL_TRANSITION_%s_TO_%s" % (before, after))


@dataclass(frozen=True)
class MachineParams:
    """Declared priors for hysteresis and anti-chattering; not tuned."""
    min_loss_dwell_s: float = 0.5
    cooldown_s: float = 3.0
    max_proposals_per_episode: int = 3
    window_s: float = 10.0
    executor_available: bool = False     # fixed in Q02-B: no trajectory follower exists


# --- trigger rules: (eligible context) -> (propose, reasons) -----------------------------------
def trigger_f0(ctx):
    return False, ("FROZEN_F_ONLY",)


def trigger_h1(ctx, timeout_s=1.0):
    return (ctx["loss_duration_s"] or 0.0) >= timeout_s, ("H1_FIXED_LOSS_TIMEOUT_%.1fs" % timeout_s,)


def trigger_h2(ctx, sigma_m=1.0):
    s = ctx["sigma_pos_m"]
    return s is not None and s >= sigma_m, ("H2_UNCERTAINTY_THRESHOLD_%.1fm" % sigma_m,)


def trigger_h3(ctx):
    return ctx["hypothesis"] == OCCLUDED, ("H3_OCCLUSION_TRIGGERED",)


def trigger_boia(ctx):
    return False, ("UTILITY_NOT_ESTIMATED",)


POLICIES = dict(F0=trigger_f0, H1=trigger_h1, H2=trigger_h2, H3=trigger_h3, BOIA=trigger_boia)


@dataclass(frozen=True)
class ShadowDecision:
    episode: int
    decision_index: int
    decision_s: float
    policy: str
    state_before: str
    state_after: str
    decision: str
    reasons: tuple
    features: tuple
    delta_u: object
    evidence_hypothesis: str
    plan_status: object
    owner: str = "F"
    action_applied: bool = False
    action: object = None
    fallback_if_no_plan: str = "UNDEFINED_PENDING_CONTROLLER_QUALIFICATION"
    ground_truth_used: bool = False
    terminal_source: object = None
    # v2.6 identity/provenance propagation (Q02-B integration): copied from the Q02-A record.
    identity_verdict: str = "NOT_APPLICABLE"
    geometry_support: str = "NOT_APPLICABLE"
    detector_provenance: str = ""
    local_map_status: object = None          # map validity summary from the pipeline (Q03)
    candidate_summary: object = None         # planner summary from the pipeline (Q03)
    runtime_s: object = None                 # CPU time of the whole decision (excluded from determinism checks)

    def __post_init__(self):
        if self.owner != "F" or self.action_applied or self.action is not None:
            raise TransitionError("SHADOW_RECORD_CANNOT_CARRY_AN_ACTION")
        if self.decision not in (CONTINUE_F, ABSTAIN, PROPOSE_REACQUIRE_SHADOW):
            raise TransitionError("UNKNOWN_DECISION")
        if self.decision == PROPOSE_REACQUIRE_SHADOW and self.policy == "BOIA":
            raise TransitionError("BOIA_CANNOT_PROPOSE_WITHOUT_DELTA_U")


class ShadowMachine:
    def __init__(self, policy="BOIA", params=MachineParams()):
        if policy not in POLICIES:
            raise TransitionError("UNKNOWN_POLICY")
        self.policy, self.trigger, self.params = policy, POLICIES[policy], params
        self.episode = None
        self._reset_state()

    def _reset_state(self):
        self.state, self.state_since_s, self.last_exit_plan_s, self.proposal_times = PRE_ACQUISITION, None, None, []

    def reset(self, episode):
        self.episode = episode
        self._reset_state()

    def _go(self, after, now_s):
        validate_transition(self.state, after)
        if after != self.state:
            if self.state == REACQUIRE_PLANNED:
                self.last_exit_plan_s = now_s
            if after == REACQUIRE_PLANNED:
                self.proposal_times.append(now_s)
            self.state, self.state_since_s = after, now_s

    def recent_switches(self, now_s):
        return sum(1 for t in self.proposal_times if now_s - t <= self.params.window_s)

    def step(self, *, now_s, decision_index, loss, record, belief, plan_status, features, delta_u,
             input_valid, terminal=None):
        before, p = self.state, self.params
        if self.state_since_s is None:
            self.state_since_s = now_s
        reasons = []
        if self.state in TERMINAL:
            raise TransitionError("STEP_AFTER_TERMINAL")
        if terminal is not None:
            if terminal not in ("capture", "failure"):
                raise TransitionError("TERMINAL_MUST_BE_capture_OR_failure")
            self._go(TERMINAL_CAPTURE if terminal == "capture" else TERMINAL_FAILURE, now_s)
            return self._record(before, ABSTAIN, ("TERMINAL",), now_s, decision_index, features, delta_u, record,
                                plan_status, "EVALUATOR_EPISODE_END")
        if not input_valid:
            return self._record(before, ABSTAIN, ("INPUT_INVALID",), now_s, decision_index, features, delta_u, record, plan_status)
        if not loss.ever_acquired:
            return self._record(before, ABSTAIN, ("NOT_PREVIOUSLY_ACQUIRED",), now_s, decision_index, features, delta_u,
                                record, plan_status)
        if self.state == PRE_ACQUISITION:
            self._go(TRACK_F, now_s)
        if not loss.currently_lost:
            if self.state in (LOSS_SUSPECTED, REACQUIRE_PLANNED):
                self._go(TRACK_F, now_s)
            why = ("TARGET_TRACKED",) if loss.raw_detection_now else ("LOSS_DEBOUNCE_PENDING",)
            return self._record(before, CONTINUE_F, why, now_s, decision_index, features, delta_u, record, plan_status)
        if self.state == TRACK_F:
            self._go(LOSS_SUSPECTED, now_s)
        # Lost: collect every reason the evidence cannot support a switch, then ask the trigger.
        if record.hypothesis == EVIDENCE_UNKNOWN:
            reasons.append("EVIDENCE_UNKNOWN")
        if (loss.identity_suspect_recent or record.hypothesis == IDENTITY_AMBIGUOUS
                or record.identity_verdict in ("AMBIGUOUS", "CONTRADICTED")):
            reasons.append("IDENTITY_SUSPECT")
        if record.detection_support == "CAMERA_FLAG" and record.identity_verdict == "NOT_TESTABLE":
            reasons.append("IDENTITY_UNVERIFIED")      # a flagged but unverifiable detection is no evidence either way
        if not belief.valid:
            reasons.append("BELIEF_INVALID")
        if plan_status is not None:
            reasons.append("NO_FEASIBLE_PLAN")
        if self.state == LOSS_SUSPECTED and now_s - self.state_since_s < p.min_loss_dwell_s - 1e-9:
            reasons.append("MIN_DWELL_NOT_MET")
        if self.last_exit_plan_s is not None and now_s - self.last_exit_plan_s < p.cooldown_s - 1e-9:
            reasons.append("COOLDOWN_ACTIVE")
        if len(self.proposal_times) >= p.max_proposals_per_episode and self.state != REACQUIRE_PLANNED:
            reasons.append("SWITCH_BUDGET_EXHAUSTED")
        ctx = dict(loss_duration_s=loss.loss_duration_s, hypothesis=record.hypothesis,
                   sigma_pos_m=(features[4].value or {}).get("sqrt_trace_m") if features[4].value else None)
        propose, why = self.trigger(ctx)
        if self.policy == "BOIA":
            reasons.append("UTILITY_NOT_ESTIMATED")
        gates_pass = not [r for r in reasons if r != "UTILITY_NOT_ESTIMATED"]
        if propose and gates_pass:
            self._go(REACQUIRE_PLANNED, now_s)
            return self._record(before, PROPOSE_REACQUIRE_SHADOW, tuple(why) + ("EXECUTOR_NOT_AVAILABLE",), now_s,
                                decision_index, features, delta_u, record, plan_status)
        if self.state == REACQUIRE_PLANNED:
            self._go(LOSS_SUSPECTED, now_s)  # plan withdrawn; cooldown starts
        if not reasons:
            reasons.append("TRIGGER_NOT_MET")
        return self._record(before, ABSTAIN, tuple(dict.fromkeys(tuple(reasons) + tuple(why))), now_s, decision_index,
                            features, delta_u, record, plan_status)

    def _record(self, before, decision, reasons, now_s, decision_index, features, delta_u, record, plan_status,
                terminal_source=None):
        return ShadowDecision(self.episode, decision_index, now_s, self.policy, before, self.state, decision,
                              tuple(reasons), features, delta_u, record.hypothesis, plan_status,
                              terminal_source=terminal_source, identity_verdict=record.identity_verdict,
                              geometry_support=record.geometry_support, detector_provenance=record.detector_provenance)
