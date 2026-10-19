"""07 §§2,10 [NEW]: pure per-environment mode selection and bounded hysteresis."""
from dataclasses import dataclass

import torch


class Mode:
    """Integer labels only; no mutable controller state is stored at module scope."""
    TERMINAL = 0
    NEVER_ACQUIRED = 1
    TRACK_VISIBLE = 2
    PRE_OCCLUSION = 3
    LOST_PREDICTABLE = 4
    LOST_UNLOCALIZED = 5
    INTERCEPT = 6
    SAFE_HOLD = 7


@dataclass(frozen=True)
class BoiaState:
    """N-first mode/counter/bool tensors and a_prev (N,4); updates never mutate inputs.

    dwell counts completed ENGAGED decision steps, including a receding argmin that
    happens to equal F. It does not claim five distinct changed physical commands.
    """
    mode: torch.Tensor
    engaged: torch.Tensor
    dwell: torch.Tensor
    off_count: torch.Tensor
    miss_streak: torch.Tensor
    a_prev: torch.Tensor
    ever_detected: torch.Tensor


def initial_state(n, reference):
    """Create independent zero state using reference's floating dtype/device."""
    assert isinstance(n, int) and n > 0 and reference.is_floating_point()
    integer = torch.zeros(n, dtype=torch.int64, device=reference.device)
    boolean = torch.zeros(n, dtype=torch.bool, device=reference.device)
    return BoiaState(torch.full_like(integer, Mode.NEVER_ACQUIRED), boolean, integer,
                     integer.clone(), integer.clone(), reference.new_zeros((n, 4)), boolean.clone())


def update_mode(state, inp, forecast_F, params, terminal_prev=None):
    """Return NEW state; forecast_F (N,M,H) is the common F detection forecast.

    PRE uses the minimum, over the first min(H_pre,H) steps, of mean-sample P_det.
    INTERCEPT takes precedence. Only the explicit current terminal mask suppresses
    this observation; an old state.mode=TERMINAL cannot latch a new live episode.
    Camera misses and acquisition history never consult tracker recency/age.
    """
    n = inp.pos.shape[0]
    assert state.mode.shape == (n,) and inp.camera_visible.shape == inp.active.shape == (n,)
    assert forecast_F.ndim == 3 and forecast_F.shape[0] == n and min(forecast_F.shape[1:]) > 0
    assert bool(torch.isfinite(forecast_F).all()) and bool(((forecast_F >= 0) & (forecast_F <= 1)).all())
    terminal = torch.zeros(n, dtype=torch.bool, device=inp.pos.device) if terminal_prev is None else terminal_prev.bool()
    assert terminal.shape == (n,)
    camera = inp.camera_visible.bool() & ~terminal
    ever = (state.ever_detected | camera) & ~terminal
    misses = torch.where(camera, torch.zeros_like(state.miss_streak), state.miss_streak + 1)
    misses = torch.where(terminal, torch.zeros_like(misses), misses)
    distance = ((inp.mu[:, :3] - inp.pos) ** 2).sum(-1).sqrt()
    horizon = min(params.H_pre, forecast_F.shape[-1])
    low_forecast = forecast_F[..., :horizon].mean(1).amin(-1) < params.p_pre
    visible_mode = torch.where(low_forecast, torch.full_like(state.mode, Mode.PRE_OCCLUSION),
                               torch.full_like(state.mode, Mode.TRACK_VISIBLE))
    visible_mode = torch.where(distance < params.d_int, torch.full_like(state.mode, Mode.INTERCEPT), visible_mode)
    uncertainty = inp.Sigma[:, :3, :3].diagonal(dim1=-2, dim2=-1).sum(-1)
    predictable = inp.active.bool() & (uncertainty <= params.sigma2_lost) & (misses <= params.n_lost)
    lost_mode = torch.where(predictable, torch.full_like(state.mode, Mode.LOST_PREDICTABLE),
                           torch.full_like(state.mode, Mode.LOST_UNLOCALIZED))
    mode = torch.where(camera, visible_mode, lost_mode)
    mode = torch.where(~ever, torch.full_like(mode, Mode.NEVER_ACQUIRED), mode)
    mode = torch.where(terminal, torch.full_like(mode, Mode.TERMINAL), mode)
    return BoiaState(mode, state.engaged.clone() & ~terminal,
                     torch.where(terminal, torch.zeros_like(state.dwell), state.dwell),
                     torch.where(terminal, torch.zeros_like(state.off_count), state.off_count), misses,
                     torch.where(terminal[:, None], torch.zeros_like(state.a_prev), state.a_prev), ever)


def advantage(J, feasible):
    """J(F)-best feasible cost; +inf for infeasible F with an alternative, zero if none.

    NaN/nonfinite candidate costs are never feasible. The no-candidate zero is an
    explicit sentinel, not measured evidence of equal utility.
    """
    assert J.ndim == 2 and feasible.shape == J.shape
    valid = feasible.bool() & torch.isfinite(J)
    best = torch.where(valid, J, torch.full_like(J, float("inf"))).amin(-1)
    baseline = torch.where(valid[:, 0], J[:, 0], torch.full_like(best, float("inf")))
    any_valid = valid.any(-1)
    safe_best = torch.where(any_valid, best, torch.zeros_like(best))
    return torch.where(any_valid, baseline - safe_best, torch.zeros_like(best))


def decide(J, hard_ok, stop_ok, cmd, a_F, mode, state, params):
    """Return exactly action, engaged, reason, NEW state (no hidden mutable history).

    Reasons: 0 F, 1 safety, 2 gain-controller engaged, 3 hold, 4 terminal, 5 never seen.
    The fifth engaged decision remains engaged; gain-only release first becomes
    eligible on the next call from state.dwell>=n_min. Safe modes/terminal override
    that minimum. Receding selection never forces a worse candidate just to differ
    from F. If all candidates fail, slot1 is the explicit brake/fallback placeholder.
    """
    n, k = J.shape
    assert k >= 2 and cmd.shape == (n, k, 4) and a_F.shape == (n, 4)
    assert hard_ok.shape == stop_ok.shape == J.shape and mode.shape == (n,)
    assert bool(torch.isfinite(cmd).all()) and bool(torch.isfinite(a_F).all())
    if params.tau_on is None or not params.tau_on > 0:
        raise ValueError("tau_on requires an explicit positive declaration")
    assert params.n_min > 0 and params.n_off > 0 and 0 <= params.tau_off_ratio < 1
    feasible = hard_ok.bool() & stop_ok.bool() & torch.isfinite(J)
    any_feasible, f_ok = feasible.any(-1), feasible[:, 0]
    terminal = mode == Mode.TERMINAL
    eligible = (mode == Mode.PRE_OCCLUSION) | (mode == Mode.LOST_PREDICTABLE)
    delta = advantage(J, feasible)
    low = delta < params.tau_on * params.tau_off_ratio
    off = torch.where(state.engaged & low, state.off_count + 1, torch.zeros_like(state.off_count))
    release = (state.dwell >= params.n_min) & (off >= params.n_off)
    gain = eligible & torch.where(state.engaged, ~release, delta > params.tau_on)
    safety = ~f_ok & any_feasible & ~terminal
    hold = ~any_feasible & ~terminal
    engaged = (gain | safety | hold) & ~terminal
    safe_J = torch.where(feasible, J, torch.full_like(J, float("inf")))
    best = safe_J.argmin(-1)
    selected = cmd.gather(1, best[:, None, None].expand(n, 1, 4)).squeeze(1)
    action = torch.where(engaged[:, None], selected, a_F)
    action = torch.where(hold[:, None], cmd[:, 1], action)
    reason = torch.where(mode == Mode.NEVER_ACQUIRED, torch.full_like(mode, 5), torch.zeros_like(mode))
    reason = torch.where(gain & engaged, torch.full_like(reason, 2), reason)
    reason = torch.where(safety, torch.full_like(reason, 1), reason)
    reason = torch.where(hold, torch.full_like(reason, 3), reason)
    reason = torch.where(terminal, torch.full_like(reason, 4), reason)
    new_mode = torch.where(hold, torch.full_like(mode, Mode.SAFE_HOLD), mode)
    dwell = torch.where(engaged, torch.where(state.engaged, state.dwell + 1, torch.ones_like(state.dwell)),
                        torch.zeros_like(state.dwell))
    off = torch.where(engaged & eligible & ~safety & ~hold, off, torch.zeros_like(off))
    previous = torch.where(terminal[:, None], torch.zeros_like(action), action).clone()
    new_state = BoiaState(new_mode.clone(), engaged.clone(), dwell, off,
                         torch.where(terminal, torch.zeros_like(state.miss_streak), state.miss_streak),
                         previous, state.ever_detected.clone() & ~terminal)
    return action, engaged, reason, new_state
