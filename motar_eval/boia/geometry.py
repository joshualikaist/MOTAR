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


# Additive C02 API: legacy functions above retain their existing behavior.
from .types import ContainmentReason, ContainmentResult, FreeBoundary2D, SafetyStatus


def _c02_point_segment(point, a, b):
    delta = b - a
    den = (delta * delta).sum(-1)
    fraction = ((point - a) * delta).sum(-1) / torch.where(den > 0, den, torch.ones_like(den))
    return ((point - a - fraction.clamp(0, 1)[..., None] * delta) ** 2).sum(-1).sqrt()


def _c02_segment_distance(a, b, c, d):
    """Analytic distance between broadcast closed planar segments."""
    best = torch.minimum(torch.minimum(_c02_point_segment(a, c, d), _c02_point_segment(b, c, d)),
                         torch.minimum(_c02_point_segment(c, a, b), _c02_point_segment(d, a, b)))
    u, v, w = b - a, d - c, c - a
    cross = u[..., 0] * v[..., 1] - u[..., 1] * v[..., 0]
    safe = torch.where(cross != 0, cross, torch.ones_like(cross))
    t = (w[..., 0] * v[..., 1] - w[..., 1] * v[..., 0]) / safe
    s = (w[..., 0] * u[..., 1] - w[..., 1] * u[..., 0]) / safe
    intersects = (cross != 0) & (t >= 0) & (t <= 1) & (s >= 0) & (s <= 1)
    return torch.where(intersects, torch.zeros_like(best), best)


def _c02_arc_distance(a, b, radii, centres, half_width):
    """Segment/circular-arc distance by all endpoint/interior stationary cases.

    Arc endpoints, segment endpoint radial projections, line-normal stationary
    circle points and actual line/circle intersections exhaust the minima.
    This is analytic finite enumeration, not dense boundary sampling.
    """
    def on_arc(v):
        phase = torch.remainder(torch.atan2(v[..., 1], v[..., 0]) - centres + math.pi,
                                2 * math.pi) - math.pi
        return phase.abs() <= half_width

    lo = torch.stack(((centres - half_width).cos(), (centres - half_width).sin()), -1)
    hi = torch.stack(((centres + half_width).cos(), (centres + half_width).sin()), -1)
    best = torch.minimum(_c02_point_segment(radii[..., None] * lo, a, b),
                         _c02_point_segment(radii[..., None] * hi, a, b))
    for endpoint in (a, b):
        norm = (endpoint * endpoint).sum(-1).sqrt()
        best = torch.minimum(best, torch.where(on_arc(endpoint), (norm - radii).abs(),
                                                torch.full_like(best, float("inf"))))
    delta = b - a
    den = (delta * delta).sum(-1)
    safe_den = torch.where(den > 0, den, torch.ones_like(den))
    normal = torch.stack((-delta[..., 1], delta[..., 0]), -1) / safe_den.sqrt()[..., None]
    for sign in (-1., 1.):
        q = sign * radii[..., None] * normal
        t = ((q - a) * delta).sum(-1) / safe_den
        mask = (den > 0) & (t >= 0) & (t <= 1) & on_arc(q)
        best = torch.minimum(best, torch.where(mask, _c02_point_segment(q, a, b),
                                                torch.full_like(best, float("inf"))))
    ad = (a * delta).sum(-1)
    discriminant = ad ** 2 - den * ((a * a).sum(-1) - radii ** 2)
    root = discriminant.clamp(min=0).sqrt()
    for sign in (-1., 1.):
        t = (-ad + sign * root) / safe_den
        q = a + t[..., None] * delta
        mask = (den > 0) & (discriminant >= 0) & (t >= 0) & (t <= 1) & on_arc(q)
        best = torch.where(mask, torch.zeros_like(best), best)
    return best


def capsule_in_free_model(a, b, boundary, radius):
    """Certify an XY capsule only in the declared radial-sector interpolation.

    a,b: broadcast (N,*Q,2) world metres; radius: scalar or broadcast (N,*Q)
    metres (an (N,) tensor also means per-env radius). CPU float32/64 inputs
    use local float64 geometry and return original-dtype clearance. No RNG,
    mutation, physical coverage certificate, sensor margin or control action.

    For an inside centre segment, clearance is the exact distance to outer
    arcs/radial steps minus radius. Outside segments have a nonpositive rejection
    witness, not a penetration-depth estimate. A radial step face belongs to the
    unknown side under the legacy adjacent-min boundary rule. Nonzero-radius
    tangency/roundoff-scale margin abstains even on an included outer arc.
    Uniform-radius internal rays are not boundaries and cause no false block.
    """
    if not isinstance(boundary, FreeBoundary2D):
        raise TypeError("boundary must explicitly declare FreeBoundary2D")
    if a.dtype != b.dtype or not b.is_floating_point():
        raise ValueError("segment endpoints must have the same floating dtype")
    a, b = torch.broadcast_tensors(a, b)
    if a.ndim < 2 or a.shape[0] == 0 or a.shape[-1] != 2 or not a.is_floating_point() or a.device.type != "cpu" or b.device.type != "cpu":
        raise ValueError("CPU floating (N,*Q,2) segments are required")
    ctx, shape = boundary.context, a.shape[:-1]
    if any(value.device.type != "cpu" for value in (ctx.pos, ctx.yaw, ctx.free_radius, ctx.angles)) or (isinstance(radius, torch.Tensor) and radius.device.type != "cpu"):
        raise ValueError("CPU boundary and radius inputs are required")
    n, count = a.shape[0], a.numel() // (2 * a.shape[0])
    if ctx.pos.shape != (n, 3) or ctx.yaw.shape != (n,) or ctx.free_radius.ndim != 2 or ctx.free_radius.shape[0] != n:
        raise ValueError("scan batch/pose shapes must match segments")
    status = torch.full(shape, int(SafetyStatus.INVALID), dtype=torch.int64)
    reason = torch.full(shape, int(ContainmentReason.INVALID_INPUT), dtype=torch.int64)
    clearance = torch.full(shape, float("nan"), dtype=a.dtype)
    raw_angles = ctx.angles.to(dtype=torch.float64)
    bins = raw_angles.numel()
    if raw_angles.ndim != 1 or bins < 3 or ctx.free_radius.shape[1] != bins or not bool(torch.isfinite(raw_angles).all()):
        return ContainmentResult(status, clearance, reason)
    step = 2 * math.pi / bins
    gaps = torch.remainder(raw_angles - raw_angles.roll(-1), 2 * math.pi)
    if not bool(torch.allclose(gaps, torch.full_like(gaps, step), atol=1e-5, rtol=1e-5)):
        return ContainmentResult(status, clearance, reason)
    valid_ctx, pos, yaw, radii = _safe_context(ctx.pos.to(torch.float64), ctx.yaw.to(torch.float64),
                                            ctx.free_radius.to(torch.float64), ctx.far_m)
    body_radius = torch.as_tensor(radius, dtype=torch.float64)
    if body_radius.ndim == 1 and body_radius.shape == (n,) and len(shape) > 1:
        body_radius = body_radius.reshape(n, *([1] * (len(shape) - 1)))
    body_radius = torch.broadcast_to(body_radius, shape).reshape(-1)
    aa, bb = a.to(torch.float64).reshape(-1, 2), b.to(torch.float64).reshape(-1, 2)
    ids = torch.arange(n).repeat_interleave(count)
    good = valid_ctx[ids] & torch.isfinite(aa).all(-1) & torch.isfinite(bb).all(-1)
    good &= torch.isfinite(body_radius) & (body_radius >= 0)
    # Canonical uniform angles follow _radii's phase convention exactly.
    angles = raw_angles[0] - torch.arange(bins, dtype=torch.float64) * step
    flat_status, flat_reason, flat_clearance = status.reshape(-1), reason.reshape(-1), clearance.reshape(-1)
    for start in range(0, aa.shape[0], 512):
        stop = min(start + 512, aa.shape[0])
        ok, env = good[start:stop], ids[start:stop]
        av = torch.where(ok[:, None], aa[start:stop], pos[env, :2])
        bv = torch.where(ok[:, None], bb[start:stop], pos[env, :2])
        local_a, local_b = (av - pos[env, :2])[:, None], (bv - pos[env, :2])[:, None]
        r = torch.where(ok, body_radius[start:stop], torch.zeros_like(body_radius[start:stop]))
        scan_radii = radii[env]
        centres = angles[None] + yaw[env, None]
        arc_distance = _c02_arc_distance(local_a, local_b, scan_radii, centres, step / 2).amin(-1)
        next_radius = scan_radii.roll(-1, -1)
        low, high = torch.minimum(scan_radii, next_radius), torch.maximum(scan_radii, next_radius)
        direction = torch.stack(((centres - step / 2).cos(), (centres - step / 2).sin()), -1)
        step_distance = _c02_segment_distance(local_a, local_b, low[..., None] * direction,
                                               high[..., None] * direction)
        step_distance = torch.where(high > low, step_distance, torch.full_like(step_distance, float("inf"))).amin(-1)
        distance = torch.minimum(arc_distance, step_distance)
        local_ctx = FreeContext(pos[env], yaw[env], scan_radii, angles, ctx.far_m)
        inside = segment_in_known_free(av, bv, local_ctx)
        margin = torch.where(inside, distance, -distance) - r
        # One-sided ambiguity band: it can only remove a certificate. It never
        # enlarges the model or changes a negative margin into a safe result.
        scale = 1 + local_a.norm(dim=-1).squeeze(-1) + local_b.norm(dim=-1).squeeze(-1)
        scale += scan_radii.amax(-1) * (1 + yaw[env].abs() + angles[0].abs()) + r
        numerical_band = 128 * torch.finfo(torch.float64).eps * scale
        ok &= torch.isfinite(margin) & torch.isfinite(distance)
        safe = inside & ((r == 0) | ((margin > numerical_band) & (step_distance - r > numerical_band)))
        flat_status[start:stop] = torch.where(ok, torch.where(safe, int(SafetyStatus.MODEL_SAFE),
                                            int(SafetyStatus.UNKNOWN)), int(SafetyStatus.INVALID))
        flat_reason[start:stop] = torch.where(ok, torch.where(safe, int(ContainmentReason.CLEAR),
                                            int(ContainmentReason.UNKNOWN_SPACE)), int(ContainmentReason.INVALID_INPUT))
        flat_clearance[start:stop] = torch.where(ok, margin, torch.full_like(margin, float("nan"))).to(a.dtype)
    return ContainmentResult(status, clearance, reason)
