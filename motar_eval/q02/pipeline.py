"""Q02-B shadow recorder: one call per decision, A -> B -> C -> D -> timing features -> state machine.

Input is one Q02-A EvidenceInput plus the caller-declared detector lineage. Output is one immutable
ShadowDecision. No task, simulator, renderer, policy, RNG or ground-truth handle is held, and no
action is produced; terminal outcomes enter only as the evaluator's episode-end signal.
"""
import dataclasses
import time

from . import belief as bl, candidates as cd, evidence as ev, localmap as lm, shadow as sh, timing as tm
from .contracts import FROZEN_F_CAMERA, FROZEN_F_LIDAR
from .history import HistoryError, ObservationHistory, loss_state


class ShadowRecorder:
    def __init__(self, policy="BOIA", camera=FROZEN_F_CAMERA, lidar=FROZEN_F_LIDAR,
                 machine_params=sh.MachineParams(), candidate_params=cd.CandidateParams()):
        self.camera, self.lidar, self.candidate_params = camera, lidar, candidate_params
        self.history, self.machine = ObservationHistory(), sh.ShadowMachine(policy, machine_params)

    def reset(self, episode):
        self.history.reset(episode)
        self.machine.reset(episode)

    def observe(self, episode, decision, inp, provenance, terminal=None):
        t0 = time.perf_counter()
        decision_record, records = self._observe(episode, decision, inp, provenance, terminal)
        feasible = sum(r.status == cd.FEASIBLE for r in records)
        return dataclasses.replace(decision_record, runtime_s=time.perf_counter() - t0,
                                   candidate_summary=dict(evaluated=len(records), feasible=feasible,
                                                          plan_status=decision_record.plan_status)), records

    def _observe(self, episode, decision, inp, provenance, terminal):
        record = ev.estimate(inp, self.camera, self.lidar)
        try:
            self.history.append(episode, decision, inp, record, provenance)
            clock_ok = True
        except HistoryError as exc:
            if str(exc) in ("DECISION_CLOCK_INCONSISTENT", "DECISION_INDEX_NOT_CONSECUTIVE"):
                clock_ok = False   # the decision is refused; the history keeps its last valid entry
            else:
                raise
        loss = loss_state(self.history)
        belief = bl.from_tracker(inp)
        local = lm.build(inp, self.lidar) if "lidar:" not in " ".join(record.validation_problems) else None
        records, plan_status = (), None
        if loss.ever_acquired and loss.currently_lost and local is not None:
            yaw = local.origin_yaw
            cands = cd.generate(local.origin_xy, yaw, self.candidate_params)
            records, plan_status, _ = cd.evaluate(local, local.origin_xy, yaw, belief, cands, inp.decision_s,
                                                   self.candidate_params)
        elif loss.ever_acquired and loss.currently_lost:
            plan_status = cd.NO_FEASIBLE_PLAN
        features, delta_u = tm.extract(inp, record, loss, local, records, plan_status,
                                       self.machine.recent_switches(inp.decision_s))
        camera_inputs = ("camera_clock", "rgb", "depth", "pose", "calibration")
        # An inactive track is a legitimate loss state; a non-finite or non-PSD track is invalid input.
        valid = clock_ok and not any(p.split(":")[0] in camera_inputs or p.startswith("track:INVALID")
                                     for p in record.validation_problems)
        return self.machine.step(now_s=inp.decision_s, decision_index=decision, loss=loss, record=record, belief=belief,
                                 plan_status=plan_status, features=features, delta_u=delta_u, input_valid=valid,
                                 terminal=terminal), records
