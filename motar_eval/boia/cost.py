"""07 §9 [NEW]: analytic candidate costs; unit-tested utilities, not calibrated gains."""
import torch

from .arbitration import Mode
from .visibility import p_any


def _logdet_pd(cov):
    """Fail closed for non-PD position covariance; no hidden information/floor is added."""
    if not bool(torch.isfinite(cov).all()) or not torch.allclose(cov, cov.transpose(-1, -2), atol=1e-7, rtol=1e-6):
        raise ValueError("position covariance must be finite and symmetric")
    factor, info = torch.linalg.cholesky_ex(cov)
    if bool((info != 0).any()):
        raise ValueError("position covariance must be positive definite; no automatic numerical floor")
    return 2 * factor.diagonal(dim1=-2, dim2=-1).log().sum(-1)


def evaluate(inp, roll, samples, pdet, Sigma_exp, cmd, a_prev, mode, hard_ok, stop_ok, clr_min, params):
    """Return J (N,K) and raw terms Phi/P_loss/U/S/C (N,K), §9 exactly.

    samples (N,M,H,6), pdet (N,K,M,H), Sigma_exp (N,K,H,6,6), cmd (N,K,4).
    Initial inp.Sigma_pp and final expected Sigma_pp must be strictly positive definite;
    PSD-singular inputs explicitly fail rather than generating inf-inf or invented data.
    Infeasible J is +inf by design. C uses d_soft=1m, NOT U5's .398m deficit diagnostic.
    Visible weights also apply to safety-only modes; gain arbitration remains disabled
    there. PRE_OCCLUSION and LOST_PREDICTABLE use the declared lost-mode weights.
    """
    n, k, h, _ = roll.step_pos.shape
    assert h > 0 and roll.step_pos.shape == (n, k, h, 3)
    assert samples.ndim == 4 and samples.shape[0] == n and samples.shape[2:] == (h, 6)
    assert pdet.shape == (n, k, samples.shape[1], h) and Sigma_exp.shape == (n, k, h, 6, 6)
    assert cmd.shape == (n, k, 4) and a_prev.shape == (n, 4) and mode.shape == (n,)
    assert hard_ok.shape == stop_ok.shape == (n, k) and clr_min.shape == (n, k, h)
    assert inp.pos.shape == (n, 3) and inp.mu.shape == (n, 6) and inp.Sigma.shape == (n, 6, 6)
    assert bool(torch.isfinite(samples).all()) and bool(torch.isfinite(roll.step_pos).all())
    assert bool(torch.isfinite(inp.pos).all()) and bool(torch.isfinite(inp.mu).all())
    assert bool(torch.isfinite(cmd).all()) and bool(torch.isfinite(a_prev).all())
    assert not bool(torch.isnan(clr_min).any()) and params.dt > 0
    initial_logdet = _logdet_pd(inp.Sigma[:, :3, :3])
    final_logdet = _logdet_pd(Sigma_exp[:, :, -1, :3, :3])
    difference = samples[:, None, :, -1, :3] - roll.step_pos[:, :, None, -1]
    final_distance = (difference * difference).sum(-1).sqrt().mean(-1)
    initial_distance = ((inp.mu[:, :3] - inp.pos) ** 2).sum(-1).sqrt()
    phi = (final_distance - initial_distance[:, None]) / (2.0 * h * params.dt)
    uncertainty = .5 * (final_logdet - initial_logdet[:, None])
    smoothness = ((cmd - a_prev[:, None]) ** 2).sum(-1) / 4
    clearance = (1.0 - clr_min).clamp(min=0).mean(-1)
    terms = {"Phi": phi, "P_loss": 1 - p_any(pdet), "U": uncertainty,
             "S": smoothness, "C": clearance}
    lost = (mode == Mode.PRE_OCCLUSION) | (mode == Mode.LOST_PREDICTABLE)
    visible_weights, lost_weights = cmd.new_tensor(params.weights_visible), cmd.new_tensor(params.weights_lost)
    assert visible_weights.shape == lost_weights.shape == (5,)
    weights = torch.where(lost[:, None], lost_weights[None], visible_weights[None])
    stacked = torch.stack((phi, terms["P_loss"], uncertainty, smoothness, clearance), -1)
    score = (stacked * weights[:, None]).sum(-1)
    feasible = hard_ok.bool() & stop_ok.bool() & torch.isfinite(score)
    return torch.where(feasible, score, torch.full_like(score, float("inf"))), terms
