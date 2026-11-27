"""B3 / 07 §3: independent CV shadow prediction, sampling and expected updates.

The simulator package cannot be imported on the CPU path. The F and Q below
mirror navrl_perception.py:653-662 without importing or mutating that tracker.
Camera noise is an explicit world-frame covariance argument, never reconstructed
from unapproved sensor internals. These kernels do not calibrate that model.
"""
import math

import torch


def _positive_dt(dt):
    if isinstance(dt, bool) or not isinstance(dt, (int, float)):
        raise TypeError("dt must be a numeric scalar")
    dt = float(dt)
    if not math.isfinite(dt) or dt <= 0:
        raise ValueError("dt must be finite and positive")
    return dt


def _positive_count(value, label):
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ValueError(label + " must be a positive integer")
    return value


def _real(value, label):
    if not isinstance(value, torch.Tensor) or value.dtype not in (torch.float32, torch.float64):
        raise TypeError(label + " must be a float32/float64 tensor")
    if not bool(torch.isfinite(value).all()):
        raise ValueError(label + " must be finite")


def _covariance(value, dimension, label):
    _real(value, label)
    if value.shape[-2:] != (dimension, dimension):
        raise ValueError(label + " has incompatible matrix dimensions")
    transpose = value.transpose(-1, -2)
    scale = value.abs().amax(dim=(-1, -2)).clamp(min=1.)
    tolerance = 128 * torch.finfo(value.dtype).eps * scale
    if bool(((value - transpose).abs().amax(dim=(-1, -2)) > tolerance).any()):
        raise ValueError(label + " must be symmetric")
    symmetric = .5 * (value + transpose)
    if bool((torch.linalg.eigvalsh(symmetric).amin(-1) < -tolerance).any()):
        raise ValueError(label + " must be positive semidefinite")
    return symmetric


def _state(mu, covariance):
    _real(mu, "mean")
    if mu.ndim != 2 or mu.shape[1] != 6 or mu.shape[0] <= 0:
        raise ValueError("mean must have shape (N,6), N positive")
    if covariance.shape != (mu.shape[0], 6, 6):
        raise ValueError("covariance must have shape (N,6,6)")
    symmetric = _covariance(covariance, 6, "covariance")
    if covariance.dtype != mu.dtype or covariance.device != mu.device:
        raise ValueError("mean and covariance must share dtype and device")
    return symmetric


def _process(Qb, mu):
    if Qb.shape not in ((6, 6), (mu.shape[0], 6, 6)):
        raise ValueError("Qb must have shape (6,6) or (N,6,6)")
    symmetric = _covariance(Qb, 6, "Qb")
    if Qb.dtype != mu.dtype or Qb.device != mu.device:
        raise ValueError("Qb must share the mean dtype and device")
    return symmetric.expand(mu.shape[0], 6, 6)


def tracker_Q(dt, *, dtype=None, device=None):
    """MOTAR diagonal process noise: .025 dt² position, .25 dt velocity."""
    dt = _positive_dt(dt)
    if dtype is None:
        dtype = torch.get_default_dtype()
    if dtype not in (torch.float32, torch.float64):
        raise TypeError("tracker Q supports float32/float64")
    return torch.diag(torch.tensor([.025 * dt * dt] * 3 + [.25 * dt] * 3,
                                   dtype=dtype, device=device))


def predict(mu, Sigma, dt, Qb, H):
    """Return future means (N,H,6) and covariances (N,H,6,6), starting at t+dt.

    F is [[I,dt I],[0,I]]. A CV prediction need not increase each variance or
    form a PSD difference from its previous covariance: cross terms can be negative.
    No-detection preservation refers to ``expected_cov`` versus its given prior.
    """
    dt, H = _positive_dt(dt), _positive_count(H, "H")
    covariance = _state(mu, Sigma).clone()
    process = _process(Qb, mu)
    F = torch.eye(6, dtype=mu.dtype, device=mu.device)
    F[:3, 3:] = torch.eye(3, dtype=mu.dtype, device=mu.device) * dt
    mean = mu.clone()
    means, covariances = [], []
    for _ in range(H):
        mean = torch.cat((mean[:, :3] + dt * mean[:, 3:], mean[:, 3:]), -1)
        covariance = F @ covariance @ F.t() + process
        covariance = _covariance(covariance, 6, "predicted covariance")
        means.append(mean)
        covariances.append(covariance)
    return torch.stack(means, 1), torch.stack(covariances, 1)


def _factor(covariance, initial_jitter, max_retries):
    """Cholesky with only failed environments retried; all regularization is recorded."""
    factors, info = torch.linalg.cholesky_ex(covariance)
    retries = torch.zeros(covariance.shape[0], dtype=torch.long, device=covariance.device)
    used = torch.zeros(covariance.shape[0], dtype=covariance.dtype, device=covariance.device)
    scale = covariance.diagonal(dim1=-2, dim2=-1).abs().amax(-1).clamp(min=1.)
    identity = torch.eye(6, dtype=covariance.dtype, device=covariance.device)
    for attempt in range(max_retries):
        failed = info != 0
        if not bool(failed.any()):
            break
        amount = initial_jitter * (10. ** attempt) * scale[failed]
        trial = covariance[failed] + amount[:, None, None] * identity
        updated, updated_info = torch.linalg.cholesky_ex(trial)
        factors[failed], info[failed] = updated, updated_info
        retries[failed] += 1
        used[failed] = amount
    if bool((info != 0).any()):
        raise ValueError("Cholesky failed after the declared jitter retries")
    return factors, {"retries": retries, "jitter": used}


def sample(mu, Sigma, Qb, M, H, generator, *, dt=.1, return_diagnostics=False,
           initial_jitter=1e-12, max_retries=6):
    """Draw (N,M,H,6) future CV paths using ONLY the supplied Generator.

    The initial belief and every process increment are sampled independently.
    ``return_diagnostics=True`` returns (paths, metadata): per-environment initial
    and process Cholesky retries/jitter. Jitter is numerical regularization and
    changes the sampled covariance by the explicitly recorded diagonal amount.
    The supplied generator advances; the global generator and source tensors do not.
    """
    if not isinstance(generator, torch.Generator):
        raise TypeError("an explicit non-None torch.Generator is required")
    dt = _positive_dt(dt)
    M, H = _positive_count(M, "M"), _positive_count(H, "H")
    covariance = _state(mu, Sigma)
    process = _process(Qb, mu)
    if generator.device.type != mu.device.type or generator.device.index != mu.device.index:
        raise ValueError("generator and mean must use the same device")
    if not math.isfinite(initial_jitter) or initial_jitter <= 0:
        raise ValueError("initial_jitter must be finite and positive")
    if isinstance(max_retries, bool) or not isinstance(max_retries, int) or max_retries < 0:
        raise ValueError("max_retries must be a nonnegative integer")
    initial_factor, initial_info = _factor(covariance, initial_jitter, max_retries)
    process_factor, process_info = _factor(process, initial_jitter, max_retries)
    shape = (mu.shape[0], M, 6)
    initial_noise = torch.randn(shape, generator=generator, dtype=mu.dtype, device=mu.device)
    state = mu[:, None] + (initial_factor[:, None] * initial_noise[..., None, :]).sum(-1)
    paths = []
    for _ in range(H):
        noise = torch.randn(shape, generator=generator, dtype=mu.dtype, device=mu.device)
        increment = (process_factor[:, None] * noise[..., None, :]).sum(-1)
        state = torch.cat((state[..., :3] + dt * state[..., 3:], state[..., 3:]), -1) + increment
        paths.append(state)
    result = torch.stack(paths, 2)
    if return_diagnostics:
        return result, {"initial_cholesky": initial_info, "process_cholesky": process_info,
                        "dt": dt, "jitter_is_numerical_regularization": True}
    return result


def expected_cov(Sigma_pred, pdet_mean, R):
    """Expected camera update, shapes (N,K,H,6,6), (N,K,H), (N,3,3).

    Equivalent to P - p K H P. A convex mixture of prior and Joseph posterior
    improves numerical PSD behavior; p=0 returns an EXACT copy of the supplied P.
    No update is recursed across H here: the later orchestration owns that order.
    Singular innovation covariance fails closed rather than inventing information.
    """
    if Sigma_pred.ndim != 5 or Sigma_pred.shape[-2:] != (6, 6):
        raise ValueError("Sigma_pred must have shape (N,K,H,6,6)")
    covariance = _covariance(Sigma_pred, 6, "predicted covariance")
    _real(pdet_mean, "detection probabilities")
    if pdet_mean.shape != Sigma_pred.shape[:-2] or min(pdet_mean.shape) <= 0:
        raise ValueError("detection probabilities must have shape (N,K,H)")
    if R.shape != (Sigma_pred.shape[0], 3, 3):
        raise ValueError("R must have shape (N,3,3)")
    noise = _covariance(R, 3, "camera covariance")
    for value in (pdet_mean, R):
        if value.dtype != Sigma_pred.dtype or value.device != Sigma_pred.device:
            raise ValueError("update tensors must share dtype and device")
    if bool(((pdet_mean < 0) | (pdet_mean > 1)).any()):
        raise ValueError("detection probabilities must lie in [0,1]")
    output = Sigma_pred.clone()
    selected = pdet_mean > 0
    if not bool(selected.any()):
        return output
    prior = covariance[selected]
    measurement = noise[:, None, None].expand(*pdet_mean.shape, 3, 3)[selected]
    innovation = prior[:, :3, :3] + measurement
    factor, info = torch.linalg.cholesky_ex(innovation)
    if bool((info != 0).any()):
        raise ValueError("innovation covariance must be positive definite for a camera update")
    gain = torch.cholesky_solve(prior[:, :3, :], factor).transpose(-1, -2)
    ikh = torch.eye(6, dtype=prior.dtype, device=prior.device).expand_as(prior).clone()
    ikh[:, :, :3] -= gain
    posterior = ikh @ prior @ ikh.transpose(-1, -2) + gain @ measurement @ gain.transpose(-1, -2)
    probability = pdet_mean[selected].unsqueeze(-1).unsqueeze(-1)
    expected = (1 - probability) * prior + probability * posterior
    output[selected] = _covariance(expected, 6, "expected covariance")
    return output
