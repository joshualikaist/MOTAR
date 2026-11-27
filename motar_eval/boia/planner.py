"""07 §12 [ADAPTED/NEW]: opt-in CPU orchestration, not simulator integration.

State is explicit: step returns Decision.new_state; with_state returns a new frozen
planner. PRE classification follows its required F forecast, before cost/arbitration.
Only live rows are evaluated or logged. No actor/tracker/task state is written.
"""
import math
from dataclasses import dataclass, fields, replace
from typing import Optional

import torch

from . import access, arbitration, belief, candidates, cost, dynamics, fallback, geometry, safety, terminal, trace, visibility
from .types import BoiaParams, Decision, FreeContext, OnlineInputs


def _subset(inp, ids):
    return OnlineInputs(inp.pos[ids], inp.vel[ids], inp.vehicle_quat[ids], inp.yaw[ids], inp.body_angvel[ids],
                        inp.lidar_m[ids], inp.angles, inp.elevations, inp.mu[ids], inp.Sigma[ids],
                        inp.active[ids], inp.age[ids], inp.camera_visible[ids], inp.policy_action[ids], inp.cam, inp.lidar_far_m)


def _state_subset(state, ids):
    return arbitration.BoiaState(*(getattr(state, f.name)[ids].clone() for f in fields(state)))


def _merge_state(state, small, ids):
    values = []
    for field in fields(state):
        value = getattr(state, field.name).clone()
        value[ids] = getattr(small, field.name)
        values.append(value)
    return arbitration.BoiaState(*values)


def expected_horizon(Sigma, pdet_mean, Qb, R, dt):
    """Recursively propagate EACH expected posterior; never reset updates over H."""
    n, k, h = pdet_mean.shape
    assert Sigma.shape == (n, 6, 6) and Qb.shape == (6, 6) and R.shape == (n, 3, 3)
    transition = torch.eye(6, dtype=Sigma.dtype, device=Sigma.device)
    transition[:3, 3:] = torch.eye(3, dtype=Sigma.dtype, device=Sigma.device) * dt
    current = Sigma[:, None].expand(n, k, 6, 6).clone()
    future = []
    for j in range(h):
        predicted = transition @ current @ transition.t() + Qb
        current = belief.expected_cov(predicted[:, :, None], pdet_mean[:, :, j:j + 1], R)[:, :, 0]
        future.append(current)
    return torch.stack(future, 2)


@dataclass(frozen=True)
class BoiaPlanner:
    """Explicit CPU controller state; defaults remain uncalibrated design priors."""
    params: BoiaParams
    state: Optional[arbitration.BoiaState] = None

    def with_state(self, state):
        return replace(self, state=state)

    def step(self, task, perception, a_F, terminal_prev, generator):
        """Return action (N,4), Decision; the supplied local Generator advances.

        A mixed batch is copied once through the whitelist and only ~terminal rows
        enter evaluation/digests. An all-terminal call bypasses sensor access/RNG.
        Positive yaw lag has no permitted Euler-rate initialization in this interface
        and fails explicitly; use the declared None prior, not an invented body-z rate.
        """
        if not isinstance(generator, torch.Generator):
            raise TypeError("an explicit torch.Generator is required")
        assert a_F.ndim == 2 and a_F.shape[1] == 4 and a_F.shape[0] > 0
        assert bool(torch.isfinite(a_F).all()) and bool((a_F.abs() <= 1).all())
        n, k = a_F.shape[0], self.params.K
        assert k == 20 and terminal_prev.shape == (n,) and terminal_prev.dtype == torch.bool
        assert terminal_prev.device == a_F.device
        state = arbitration.initial_state(n, a_F) if self.state is None else self.state
        assert state.mode.shape == (n,) and state.a_prev.shape == (n, 4)
        cleared = terminal.apply(terminal_prev, state)
        ids = (~terminal_prev).nonzero(as_tuple=False).flatten()
        action = a_F.detach().clone()
        commands = action[:, None].expand(n, k, 4).clone()
        hard = torch.zeros((n, k), dtype=torch.bool, device=a_F.device)
        stop = torch.zeros_like(hard)
        scores = a_F.new_full((n, k), float("inf"))
        terms = {name: a_F.new_zeros(n, k) for name in ("Phi", "P_loss", "U", "S", "C")}
        delta = a_F.new_zeros(n)
        reason = torch.full((n,), 4, dtype=torch.int64, device=a_F.device)
        diagnostics, inp, updated = {}, None, cleared
        if ids.numel():
            params = self.params
            if params.R_camera_diag is None or len(params.R_camera_diag) != 3 or not all(math.isfinite(x) and x > 0 for x in params.R_camera_diag):
                raise ValueError("an explicit positive camera covariance prior is required; no calibration is inferred")
            if params.tau_psi is not None:
                raise ValueError("positive yaw lag requires a separately declared permitted initial yaw rate")
            inp = access.read_online(task, perception, a_F, params)
            if any(value.dtype != a_F.dtype for value in (inp.pos, inp.vel, inp.mu, inp.Sigma, inp.lidar_m, inp.yaw)):
                raise ValueError("floating online inputs and actions must share dtype")
            live = _subset(inp, ids)
            old = _state_subset(cleared, ids)
            points, valid = geometry.lidar_points_xy(live.lidar_m, live.angles, live.elevations, live.pos, live.yaw, live.lidar_far_m)
            excluded = geometry.exclude_target(points, valid, live.mu[:, :2], live.Sigma[:, :2, :2], params.r_T)
            obstacles = torch.where(live.active[:, None], excluded, valid)
            free = FreeContext(live.pos, live.yaw, geometry.known_free(live.lidar_m, live.angles, live.pos, live.yaw,
                                                                    live.lidar_far_m, params.m_free), live.angles, live.lidar_far_m)
            cmd, families, candidate_diagnostics = candidates.generate(live.policy_action, live, live.mu, points, obstacles, params)
            roll = dynamics.rollout(cmd, live.pos, live.vel, live.yaw, points, obstacles,
                                    dynamics.RiskcapConfig(max_range_m=live.lidar_far_m), params, params.H, params.S, params.dt)
            hard_live, stop_live, clearance, soft = safety.screen(roll, points, obstacles, free, params)
            process = belief.tracker_Q(params.dt, dtype=live.mu.dtype, device=live.mu.device) * params.s_Q
            paths, sample_diagnostics = belief.sample(live.mu, live.Sigma, process, params.M, params.H, generator,
                                                    dt=params.dt, return_diagnostics=True)
            pdet = visibility.forecast(roll.step_pos, roll.step_psi, paths, points, obstacles, free, live.cam, params)
            classified = arbitration.update_mode(old, live, pdet[:, 0], params)
            noise = torch.diag(live.pos.new_tensor(params.R_camera_diag))[None].expand(ids.numel(), 3, 3)
            covariances = expected_horizon(live.Sigma, pdet.mean(2), process, noise, params.dt)
            score, raw_terms = cost.evaluate(live, roll, paths, pdet, covariances, cmd, old.a_prev, classified.mode,
                                            hard_live, stop_live, clearance, params)
            chosen, engaged, why, new = arbitration.decide(score, hard_live, stop_live, cmd, live.policy_action,
                                                         classified.mode, classified, params)
            hold = new.mode == arbitration.Mode.SAFE_HOLD
            chosen = torch.where(hold[:, None], fallback.safe_hold(live), chosen)
            new = replace(new, a_prev=chosen.clone())
            action[ids], commands[ids], hard[ids], stop[ids], scores[ids] = chosen, cmd, hard_live, stop_live, score
            for name in terms:
                terms[name][ids] = raw_terms[name]
            delta[ids], reason[ids] = arbitration.advantage(score, hard_live & stop_live), why
            updated = _merge_state(cleared, new, ids)
            diagnostics = {"live_env_index": ids.clone(), "family_id": families,
                           "candidate": candidate_diagnostics, "samples": sample_diagnostics,
                           "pdet_mean": pdet.mean(2), "expected_covariance": covariances,
                           "body_soft_deficit_m": soft}
        decision = Decision(action, commands, hard, stop, scores, terms, delta, updated.mode,
                            updated.engaged, reason, updated, diagnostics=diagnostics)
        return action, replace(decision, trace=trace.record(decision, inp, terminal_prev))
