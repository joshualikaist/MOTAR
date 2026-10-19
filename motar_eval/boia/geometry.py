"""P0 correction to 07 §4: preserve measured points and project scan bounds to XY.

The scan-relative sectors remain a planar interpolation model, not a certificate
that sparse rays observed every intervening physical obstacle.
"""
import math

import torch

from .types import FreeContext


def lidar_points_xy(rho, angles, elev, pos, yaw, far_m):
    """Ranges (N,V*B) -> world XY points (N,V*B,2), validity (N,V*B)."""
    n, p = rho.shape
    assert p == angles.numel() * elev.numel() and pos.shape == (n, 3) and yaw.shape == (n,)
    r = rho.reshape(n, elev.numel(), angles.numel())
    valid = torch.isfinite(r) & (r > 0) & (r < 0.995 * far_m)
    h = torch.where(valid, r, torch.zeros_like(r)) * elev.cos()[None, :, None]
    a = angles[None, None, :] + yaw[:, None, None]
    pts = torch.stack((h * a.cos(), h * a.sin()), -1) + pos[:, None, None, :2]
    return pts.reshape(n, p, 2), valid.reshape(n, p)


def exclude_target(pts, valid, mu_xy, Sigma_xy, r_T, chi2=9.21):
    """ABSTAIN from identity classification; keep ALL already-valid first hits.

    Signature compatibility is retained, but a belief ellipse cannot distinguish
    a target return from a foreground obstacle or a false association. Even at
    zero covariance its former sqrt(chi2)*r_T gate erased uncertain obstacles.
    Means/covariances/radius no longer authorize removal. A true target return is
    conservatively retained too: self-blocking visibility/collision trade-offs
    remain explicit limitations, not evidence of successful target exclusion.
    """
    n, p, _ = pts.shape
    assert pts.shape == (n, p, 2) and valid.shape == (n, p) and valid.dtype == torch.bool
    assert mu_xy.shape == (n, 2) and Sigma_xy.shape == (n, 2, 2)
    return valid.clone()


def seg_point_dist(a, b, pts):
    """Segments (N,*Q,2), points (N,P,2) -> (N,*Q,P); zero-length segments allowed."""
    a, b = torch.broadcast_tensors(a, b)
    assert a.shape[0] == pts.shape[0] and a.shape[-1] == pts.shape[-1] == 2
    q = a.ndim - 2
    points = pts.reshape(pts.shape[0], *([1] * q), pts.shape[1], 2)
    ab = (b - a).unsqueeze(-2)
    ap = points - a.unsqueeze(-2)
    den = (ab * ab).sum(-1)
    den = torch.where(den > 0, den, torch.ones_like(den))
    t = ((ap * ab).sum(-1) / den).clamp(0, 1)
    delta = ap - t.unsqueeze(-1) * ab
    return (delta * delta).sum(-1).sqrt()


def known_free(rho, angles, pos, yaw, far_m, m_free, *, elevations=None):
    """Slant ranges (N,V*B) -> conservative horizontal sector radii (N,B), metres.

    Project EACH row by cos(elevation), then take the row minimum and subtract
    a horizontal margin. Legacy six-argument calls use the declared four-row
    RGB-D+LiDAR contract [20,10,0,-10] degrees. Other layouts must explicitly
    provide their V elevation angles in radians; a zero-angle row is not guessed.
    Invalid evidence blocks space. No first-hit ray is filled to the far plane.
    Inter-ray sectors still assume a planar obstacle model (not 3D free-space).
    """
    n = rho.shape[0]
    assert rho.ndim == 2 and rho.shape[1] % angles.numel() == 0
    assert pos.shape == (n, 3) and yaw.shape == (n,)
    if not rho.is_floating_point() or not math.isfinite(far_m) or far_m <= 0 or not math.isfinite(m_free) or m_free < 0:
        raise ValueError("finite positive far range and nonnegative margin are required")
    rows = rho.shape[1] // angles.numel()
    if elevations is None:
        if rows != 4:
            raise ValueError("nonstandard rows require explicitly declared elevations")
        elevations = rho.new_tensor((20., 10., 0., -10.)) * (math.pi / 180)
    else:
        elevations = torch.as_tensor(elevations, dtype=rho.dtype, device=rho.device)
    if elevations.shape != (rows,) or not bool(torch.isfinite(elevations).all()) or bool((elevations.abs() >= math.pi / 2).any()):
        raise ValueError("one finite forward elevation in radians is required per row")
    ranges = torch.where(torch.isfinite(rho), rho, torch.zeros_like(rho)).clamp(0, far_m)
    horizontal = ranges.reshape(n, rows, angles.numel()) * elevations.cos()[None, :, None]
    base = (horizontal.amin(1) - m_free).clamp(min=0)
    return torch.minimum(base, torch.minimum(base.roll(1, 1), base.roll(-1, 1)))


def _radii(relative, yaw, radius, angles):
    """Nearest-ray sectors for the full-circle, uniformly decreasing angle convention."""
    n = relative.shape[0]
    b = angles.numel()
    step = 2 * math.pi / b
    assert b >= 3 and angles.ndim == 1 and radius.shape == (n, b)
    gaps = torch.remainder(angles - angles.roll(-1), 2 * math.pi)
    assert bool(torch.allclose(gaps, torch.full_like(gaps, step), atol=1e-5, rtol=1e-5))
    phi = torch.atan2(relative[..., 1], relative[..., 0]) - yaw.reshape(n, *([1] * (relative.ndim - 2)))
    phase = torch.remainder((angles[0] - phi) / step, b)
    index = torch.floor(phase + 0.5).long() % b
    values = radius.gather(1, index.reshape(n, -1)).reshape(index.shape)
    boundary = (phase - phase.floor() - 0.5).abs() <= 8 * torch.finfo(relative.dtype).eps
    lower = phase.floor().long()
    left = radius.gather(1, (lower % b).reshape(n, -1)).reshape(index.shape)
    right = radius.gather(1, ((lower + 1) % b).reshape(n, -1)).reshape(index.shape)
    return torch.where(boundary, torch.minimum(left, right), values)


def _safe_context(pos, yaw, radius, far_m):
    """Reject a whole scan row on invalid pose/radius; never clip infinity to free.

    Sanitized local values keep trig/gather arithmetic defined. The validity mask
    is still required on every answer; these zeros are not replacement evidence.
    """
    n = pos.shape[0]
    assert pos.shape == (n, 3) and yaw.shape == (n,) and radius.ndim == 2 and radius.shape[0] == n
    valid = torch.isfinite(pos).all(-1) & torch.isfinite(yaw)
    valid &= torch.isfinite(radius).all(-1) & (radius >= 0).all(-1)
    if math.isfinite(far_m) and far_m > 0:
        valid &= (radius <= far_m).all(-1)
    else:
        valid &= False
    return (valid, torch.where(valid[:, None], pos, torch.zeros_like(pos)),
            torch.where(valid, yaw, torch.zeros_like(yaw)),
            torch.where(valid[:, None], radius, torch.zeros_like(radius)))


def point_in_known_free(q, pos, yaw, free_radius, angles, far_m=12.0):
    """Queries (N,*Q,2) -> (N,*Q); unseen space, invalid evidence and >far are blocked."""
    assert q.shape[0] == pos.shape[0] and q.shape[-1] == 2
    valid, pos, yaw, free_radius = _safe_context(pos, yaw, free_radius, far_m)
    origin = pos[:, :2].reshape(pos.shape[0], *([1] * (q.ndim - 2)), 2)
    rel = q - origin
    limit = _radii(rel, yaw, free_radius, angles)
    return valid.reshape(pos.shape[0], *([1] * (q.ndim - 2))) & torch.isfinite(rel).all(-1) & ((rel * rel).sum(-1) <= limit ** 2)


def segment_in_known_free(a, b, ctx):
    """Exact 2D sector-boundary partition, rather than endpoint/coarse sampled checks."""
    assert isinstance(ctx, FreeContext)
    a, b = torch.broadcast_tensors(a, b)
    n, shape = a.shape[0], a.shape[:-1]
    assert b.shape == a.shape and a.shape[-1] == 2 and n == ctx.pos.shape[0]
    valid, pos, yaw, radius = _safe_context(ctx.pos, ctx.yaw, ctx.free_radius, ctx.far_m)
    aa = (a - pos[:, :2].reshape(n, *([1] * (a.ndim - 2)), 2)).reshape(n, -1, 2)
    dd = (b - a).reshape(n, -1, 2)
    angle = ctx.angles[None] + yaw[:, None] - math.pi / ctx.angles.numel()
    ray = torch.stack((angle.cos(), angle.sin()), -1)
    answer = torch.empty(aa.shape[:2], dtype=torch.bool, device=a.device)
    for start in range(0, aa.shape[1], 1024):
        av, dv = aa[:, start:start + 1024], dd[:, start:start + 1024]
        den = dv[..., 0, None] * ray[:, None, :, 1] - dv[..., 1, None] * ray[:, None, :, 0]
        num = av[..., 0, None] * ray[:, None, :, 1] - av[..., 1, None] * ray[:, None, :, 0]
        t = -num / torch.where(den != 0, den, torch.ones_like(den))
        hit = av[:, :, None] + t[..., None] * dv[:, :, None]
        good = (den != 0) & (t > 0) & (t < 1) & ((hit * ray[:, None]).sum(-1) >= 0)
        cuts = torch.cat((torch.zeros_like(t[..., :1]), torch.where(good, t, torch.ones_like(t)), torch.ones_like(t[..., :1])), -1).sort(-1).values
        left = av[:, :, None] + cuts[..., :-1, None] * dv[:, :, None]
        right = av[:, :, None] + cuts[..., 1:, None] * dv[:, :, None]
        limit = _radii((left + right) / 2, yaw, radius, ctx.angles)
        max_r2 = torch.maximum((left * left).sum(-1), (right * right).sum(-1))
        answer[:, start:start + 1024] = ((cuts[..., 1:] == cuts[..., :-1]) | (max_r2 <= limit ** 2)).all(-1)
    endpoints = point_in_known_free(a, ctx.pos, ctx.yaw, ctx.free_radius, ctx.angles, ctx.far_m)
    endpoints &= point_in_known_free(b, ctx.pos, ctx.yaw, ctx.free_radius, ctx.angles, ctx.far_m)
    return valid.reshape(n, *([1] * (a.ndim - 2))) & answer.reshape(shape) & endpoints
