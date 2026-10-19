"""07 §5 [ADAPTED/NEW]: joint sampled visibility and BPOD's independence comparator."""
import math

import torch

from .geometry import seg_point_dist, segment_in_known_free


def fov_mask(rel_V, hfov, vfov, max_range):
    """Vehicle-camera relative vectors (...,3) -> bool; angles in radians, radial range."""
    assert rel_V.shape[-1] == 3 and 0 < hfov < math.pi and 0 < vfov < math.pi
    x, y, z = rel_V.unbind(-1)
    return (torch.isfinite(rel_V).all(-1) & (x > 0) & (y.abs() <= x * math.tan(hfov / 2))
            & (z.abs() <= x * math.tan(vfov / 2)) & ((rel_V * rel_V).sum(-1) <= max_range ** 2))


def los_clear(p_ego, x_tgt, pts_obs, valid_obs, r_occ, free_ctx, unknown_blocks=True):
    """Segments (N,*Q,2) -> (N,*Q), with a common measured point set per environment."""
    assert r_occ >= 0 and valid_obs.shape == pts_obs.shape[:2]
    d = seg_point_dist(p_ego, x_tgt, pts_obs)
    mask = valid_obs.reshape(valid_obs.shape[0], *([1] * (d.ndim - 2)), -1)
    clear = ((d >= r_occ) | ~mask).all(-1)
    if unknown_blocks:
        if free_ctx is None:
            raise ValueError("unknown-blocking LOS requires the original scan context")
        clear &= segment_in_known_free(p_ego, x_tgt, free_ctx)
    return clear


def _to_camera(relative, yaw, cam):
    yaw = yaw.reshape(yaw.shape + (1,) * (relative.ndim - yaw.ndim - 1))
    x = relative[..., 0] * yaw.cos() + relative[..., 1] * yaw.sin()
    y = -relative[..., 0] * yaw.sin() + relative[..., 1] * yaw.cos()
    offset = relative.new_tensor(cam.camera_offset)
    return torch.stack((x, y, relative[..., 2]), -1) - offset


def forecast(rollout_pos, rollout_yaw, samples, pts_obs, valid_obs, free_ctx, cam, params, mode="oracle"):
    """(N,K,H,3), (N,K,H), (N,M,H,6) -> P_det (N,K,M,H); candidate/time chunks."""
    n, k, h, xyz = rollout_pos.shape
    assert xyz == 3 and rollout_yaw.shape == (n, k, h)
    assert samples.ndim == 4 and samples.shape[0] == n and samples.shape[2:] == (h, 6)
    if params.r_bar_est is None:
        raise ValueError("r_bar_est requires an explicit declared prior")
    if mode not in ("oracle", "stress", "stressed"):
        raise ValueError("mode must be oracle, stress or stressed")
    if mode != "oracle" and params.p_miss is None:
        raise ValueError("stress mode requires explicit p_miss")
    out = rollout_pos.new_empty((n, k, samples.shape[1], h))
    for j in range(h):
        for c in range(k):
            rel = samples[:, :, j, :3] - rollout_pos[:, c, j, None]
            local = _to_camera(rel, rollout_yaw[:, c, j], cam)
            fov = fov_mask(local, cam.hfov, cam.vfov, cam.max_camera_range)
            d2 = (local * local).sum(-1).clamp(min=torch.finfo(local.dtype).tiny)
            pixels = math.pi * (cam.fx * params.r_T) ** 2 / d2
            detector = (pixels >= params.min_pixels).to(local.dtype)
            if mode != "oracle":
                detector = (1 - params.p_miss) * torch.sigmoid(params.beta * (pixels - params.n_0))
            ego = rollout_pos[:, c, j, None, :2].expand(n, samples.shape[1], 2)
            los = los_clear(ego, samples[:, :, j, :2], pts_obs, valid_obs,
                            params.r_bar_est + params.r_T, free_ctx, params.unknown_blocks)
            out[:, c, :, j] = torch.where(fov & los, detector, torch.zeros_like(detector))
    return out


def p_any(p):
    """Average per-path union probability: (N,K,M,H) -> (N,K)."""
    assert p.ndim == 4 and bool(((p >= 0) & (p <= 1)).all())
    return (1 - (1 - p).prod(-1)).mean(-1)


def _normal_clearance(g, grad, cov, strict=None):
    variance = (grad * (cov * grad.unsqueeze(-2)).sum(-1)).sum(-1).clamp(min=0)
    sd = torch.where(variance > 0, variance.sqrt(), torch.ones_like(variance))
    prob = (1 + torch.erf(g / (math.sqrt(2) * sd))) / 2
    deterministic = g >= 0
    if strict is not None:
        deterministic &= ~strict | (g > 0)
    return torch.where(variance > 0, prob, deterministic.to(g.dtype))


def bpod_analytic(p_ego, yaw, mu_pos, Sigma_pos, pts_obs, valid_obs, r_occ, cam):
    """BPOD pp.4–5 Eqs.15/19 [ADAPTED]: fixed ego, mean-linearized FOV/point-disc LOS.

    p_ego (N,*Q,3), yaw (N,*Q), mu_pos (N,3), Sigma_pos (N,3,3) -> (N,*Q).
    Uses one active FOV signed margin, then independent obstacle-clear probabilities.
    Size/range gates are linearized too; unknown-space probability is not modeled here.
    """
    assert p_ego.shape[:-1] == yaw.shape and p_ego.shape[-1] == 3
    n, qdim = p_ego.shape[0], p_ego.ndim - 2
    assert mu_pos.shape == (n, 3) and Sigma_pos.shape == (n, 3, 3)
    mu = mu_pos.reshape(n, *([1] * qdim), 3)
    cov = Sigma_pos.reshape(n, *([1] * qdim), 3, 3)
    local = _to_camera(mu - p_ego, yaw, cam)
    th, tv = math.tan(cam.hfov / 2), math.tan(cam.vfov / 2)
    normals = p_ego.new_tensor(((1, 0, 0), (th, 1, 0), (th, -1, 0), (tv, 0, 1), (tv, 0, -1)))
    normals = normals / (normals * normals).sum(-1, keepdim=True).sqrt()
    margins = (local.unsqueeze(-2) * normals).sum(-1)
    distance = (local * local).sum(-1).sqrt()
    range_limit = min(cam.max_camera_range, cam.fx * cam.target_radius * math.sqrt(math.pi / cam.min_pixels))
    margins = torch.cat((margins, (range_limit - distance).unsqueeze(-1)), -1)
    gradients = normals.expand(*local.shape[:-1], 5, 3)
    radial = -local / torch.where(distance > 0, distance, torch.ones_like(distance)).unsqueeze(-1)
    gradients = torch.cat((gradients, radial.unsqueeze(-2)), -2)
    active = margins.argmin(-1)
    gv = gradients.gather(-2, active[..., None, None].expand(*active.shape, 1, 3)).squeeze(-2)
    grad_world = torch.stack((gv[..., 0] * yaw.cos() - gv[..., 1] * yaw.sin(),
                              gv[..., 0] * yaw.sin() + gv[..., 1] * yaw.cos(), gv[..., 2]), -1)
    gamma_fov = _normal_clearance(margins.amin(-1), grad_world, cov, strict=active == 0)
    a, b = p_ego[..., :2], mu[..., :2].expand_as(p_ego[..., :2])
    points = pts_obs.reshape(n, *([1] * qdim), pts_obs.shape[1], 2)
    ab = (b - a).unsqueeze(-2)
    ap = points - a.unsqueeze(-2)
    den = (ab * ab).sum(-1)
    t = ((ap * ab).sum(-1) / torch.where(den > 0, den, torch.ones_like(den))).clamp(0, 1)
    delta = a.unsqueeze(-2) + t.unsqueeze(-1) * ab - points
    dist = (delta * delta).sum(-1).sqrt()
    grad = t.unsqueeze(-1) * delta / torch.where(dist > 0, dist, torch.ones_like(dist)).unsqueeze(-1)
    gamma_los = _normal_clearance(dist - r_occ, grad, cov[..., None, :2, :2])
    valid = valid_obs.reshape(n, *([1] * qdim), -1)
    return gamma_fov * torch.where(valid, gamma_los, torch.ones_like(gamma_los)).prod(-1)
