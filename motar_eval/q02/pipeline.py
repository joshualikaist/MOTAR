"""Q02-B shadow recorder: one call per decision, A -> B -> C -> D -> timing features -> state machine.

Input is one Q02-A EvidenceInput plus the caller-declared detector lineage. Output is one immutable
ShadowDecision. No task, simulator, renderer, policy, RNG or ground-truth handle is held, and no
action is produced; terminal outcomes enter only as the evaluator's episode-end signal.
"""
import dataclasses
import time

from . import belief as bl, evidence as ev, obstacle_map as om, reacq_planner as rp, shadow as sh, timing as tm
from . import validation as va
from .contracts import FROZEN_F_CAMERA, FROZEN_F_LIDAR
from .geometry import quat_yaw_xyzw
from .history import HistoryError, ObservationHistory, loss_state


class ShadowRecorder:
    """Q02-B shadow recorder with the Q03 local map v0 and candidate planner v0 (Phase 3 integration).
    One call per decision; the map is per episode; nothing produces or applies an action."""

    def __init__(self, policy="BOIA", camera=FROZEN_F_CAMERA, lidar=FROZEN_F_LIDAR,
                 machine_params=sh.MachineParams(), limits=rp.MotionLimits(), map_params=om.MapParams()):
        self.camera, self.lidar, self.limits, self.map_params = camera, lidar, limits, map_params
        self.history, self.machine = ObservationHistory(), sh.ShadowMachine(policy, machine_params)
        self.map = om.LocalObstacleMap(map_params, lidar, camera)

    def reset(self, episode):
        self.history.reset(episode)
        self.machine.reset(episode)
        self.map = om.LocalObstacleMap(self.map_params, self.lidar, self.camera)

    def observe(self, episode, decision, inp, provenance, terminal=None):
        t0 = time.perf_counter()
        decision_record, cands, map_status, plan_why = self._observe(episode, decision, inp, provenance, terminal)
        useful = [c for c in cands if c.l2_kinematic_check == rp.PASS and "NO_VISIBILITY_GAIN" not in c.reasons]
        return dataclasses.replace(
            decision_record, runtime_s=time.perf_counter() - t0, local_map_status=map_status,
            candidate_summary=dict(evaluated=len(cands), l1_open=sum(c.l1_observed_direction_open == rp.PASS for c in cands),
                                   l2_pass=sum(c.l2_kinematic_check == rp.PASS for c in cands), l2_pass_with_view_gain=len(useful),
                                   l3_dynamic_feasibility=rp.NOT_VALIDATED, l4_closed_loop_safety=rp.NOT_VALIDATED,
                                   plan_status=decision_record.plan_status, reasons=plan_why)), cands

    def _observe(self, episode, decision, inp, provenance, terminal):
        record = ev.estimate(inp, self.camera, self.lidar)
        report = va.validate(inp, self.camera, self.lidar)          # same inputs; needed for the map's native depth
        try:
            self.history.append(episode, decision, inp, record, provenance)
            clock_ok = True
        except HistoryError as exc:
            if str(exc) in ("DECISION_CLOCK_INCONSISTENT", "DECISION_INDEX_NOT_CONSECUTIVE"):
                clock_ok = False   # the decision is refused; the history keeps its last valid entry
            else:
                raise
        receipt = self.map.integrate(inp, report) if clock_ok else om.FrameReceipt(False, ("DECISION_REFUSED",), float(inp.decision_s))
        map_status = dict(accepted=receipt.accepted, reasons=receipt.reasons, **self.map.validity(float(inp.decision_s)))
        loss = loss_state(self.history)
        belief = bl.from_tracker(inp)
        cands, plan_status, plan_why = (), None, ()
        if loss.ever_acquired and loss.currently_lost:
            yaw = quat_yaw_xyzw(inp.vehicle_quat) if report.status("pose") == "OK" else None
            if yaw is None:
                plan_status, plan_why = rp.NO_FEASIBLE_PLAN, ("POSE_INVALID",)
            else:
                cands, plan_status, plan_why = rp.plan(self.map, inp.position, yaw, inp.velocity, belief, float(inp.decision_s),
                                                       self.camera, self.limits)
        features, delta_u = tm.extract(inp, record, loss, self.map, cands, plan_status,
                                       self.machine.recent_switches(inp.decision_s))
        camera_inputs = ("camera_clock", "rgb", "depth", "pose", "calibration")
        # An inactive track is a legitimate loss state; a non-finite or non-PSD track is invalid input.
        valid = clock_ok and not any(p.split(":")[0] in camera_inputs or p.startswith("track:INVALID")
                                     for p in record.validation_problems)
        return self.machine.step(now_s=inp.decision_s, decision_index=decision, loss=loss, record=record, belief=belief,
                                 plan_status=plan_status, features=features, delta_u=delta_u, input_valid=valid,
                                 terminal=terminal), cands, map_status, plan_why
