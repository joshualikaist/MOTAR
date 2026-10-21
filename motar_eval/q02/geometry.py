"""Frame transforms and sensor geometry matching the audited Frozen F conventions (numpy, CPU)."""
import math

import numpy as np


def quat_yaw_xyzw(q):
    x, y, z, w = (float(c) for c in q)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def is_yaw_only(q, tol=1e-4):
    q = np.asarray(q, dtype=float)
    return abs(np.linalg.norm(q) - 1.0) < tol and abs(q[0]) < tol and abs(q[1]) < tol


def yaw_rotation(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def world_to_vehicle(p_world, position, yaw):
    """YAW_VEHICLE coordinates of world points: R(yaw)^T (p - position)."""
    return (np.asarray(p_world, dtype=float) - np.asarray(position, dtype=float)) @ yaw_rotation(yaw)


def vehicle_to_world(p_vehicle, position, yaw):
    return np.asarray(p_vehicle, dtype=float) @ yaw_rotation(yaw).T + np.asarray(position, dtype=float)


def cov_world_to_vehicle(cov3, yaw):
    r = yaw_rotation(yaw)
    return r.T @ np.asarray(cov3, dtype=float) @ r


def vehicle_to_camera(p_vehicle, camera):
    """Camera axes equal YAW_VEHICLE axes (nominal R=I); only the origin moves."""
    return np.asarray(p_vehicle, dtype=float) - np.asarray(camera.camera_offset, dtype=float)


def project(p_camera, camera):
    """Pinhole inverse of navrl_perception's ray = [1, -(u-cx)/fx, -(v-cy)/fy]. Returns
    (u, v, in_front, in_image); u/v are NaN behind the camera."""
    p = np.asarray(p_camera, dtype=float)
    x = p[..., 0]
    in_front = x > 1e-6
    with np.errstate(divide="ignore", invalid="ignore"):
        u = np.where(in_front, camera.cx - camera.fx * p[..., 1] / x, np.nan)
        v = np.where(in_front, camera.cy - camera.fy * p[..., 2] / x, np.nan)
    width, height = 2 * camera.cx + 1, 2 * camera.cy + 1
    in_image = in_front & (u >= -0.5) & (u <= width - 0.5) & (v >= -0.5) & (v <= height - 0.5)
    return u, v, in_front, in_image


def pixel_rays(u, v, camera):
    r = np.stack([np.ones_like(np.asarray(u, dtype=float)), -(np.asarray(u, dtype=float) - camera.cx) / camera.fx,
                  -(np.asarray(v, dtype=float) - camera.cy) / camera.fy], axis=-1)
    return r / np.linalg.norm(r, axis=-1, keepdims=True)


def lidar_directions(model):
    """(vbeams, hbeams, 3) unit rays: (cos az cos el, sin az cos el, sin el)."""
    az, el = model.azimuth_rad()[None, :], model.elevation_rad()[:, None]
    return np.stack([np.cos(az) * np.cos(el), np.sin(az) * np.cos(el), np.sin(el) * np.ones_like(az)], axis=-1)


def angle_diff(a, b):
    return np.arctan2(np.sin(np.asarray(a) - b), np.cos(np.asarray(a) - b))


def sphere_first_hit(origin, directions, centre, radius):
    """Distance along unit rays to a sphere surface (G4-R5 B04: t = u.c - sqrt(R^2 - |c_perp|^2)); inf if missed."""
    c = np.asarray(centre, dtype=float) - np.asarray(origin, dtype=float)
    d = np.asarray(directions, dtype=float)
    along = d @ c
    perp2 = float(c @ c) - along * along
    disc = radius * radius - perp2
    t = along - np.sqrt(np.clip(disc, 0.0, None))
    return np.where((disc >= 0.0) & (t > 0.0), t, np.inf)


# --- Q02A-v2: native obstacle-depth recovery and ray-envelope geometry --------------------------
def bilinear_matrix(n_out, n_in):
    """1-D operator of torch.nn.functional.interpolate(mode='bilinear', align_corners=False)."""
    a = np.zeros((n_out, n_in))
    scale = n_in / n_out
    for o in range(n_out):
        src = max((o + 0.5) * scale - 0.5, 0.0)
        x0 = min(int(math.floor(src)), n_in - 1)
        x1 = min(x0 + 1, n_in - 1)
        lam = src - x0
        a[o, x0] += 1.0 - lam
        a[o, x1] += lam
    return a


_RECOVERY_CACHE = {}


def _operators(out_hw, native_wh):
    key = (tuple(out_hw), tuple(native_wh))
    if key not in _RECOVERY_CACHE:
        av, ah = bilinear_matrix(out_hw[0], native_wh[1]), bilinear_matrix(out_hw[1], native_wh[0])
        ri = np.argsort(-av, axis=1)[:, :2]; rw = np.take_along_axis(av, ri, 1)
        ci = np.argsort(-ah, axis=1)[:, :2]; cw = np.take_along_axis(ah, ci, 1)
        n_w = native_wh[0]
        idx = (ri[:, None, :, None] * n_w + ci[None, :, None, :]).reshape(out_hw[0] * out_hw[1], 4)
        wts = (rw[:, None, :, None] * cw[None, :, None, :]).reshape(out_hw[0] * out_hw[1], 4)
        kvi, khi = np.linalg.inv(av.T @ av), np.linalg.inv(ah.T @ ah)
        _RECOVERY_CACHE[key] = (av, ah, idx, wts, kvi, khi)
    return _RECOVERY_CACHE[key]


def _solve_without(y_img, removed, ops, native_wh):
    """Exact least squares for the native grid using every pixel except `removed` (flat indices):
    Woodbury downdate of the separable normal matrix (Av^T Av) (x) (Ah^T Ah). Returns None if the
    remaining pixels do not determine the grid (cells hidden behind removed pixels)."""
    av, ah, idx, wts, kvi, khi = ops
    nh, nw = native_wh[1], native_wh[0]
    ninv = lambda v: np.einsum("ij,jk...,kl->il...", kvi, v.reshape(nh, nw, *v.shape[1:]), khi).reshape(v.shape)
    z0 = ninv((av.T @ y_img @ ah).reshape(-1))
    if removed.size == 0:
        return z0
    u = np.zeros((nh * nw, removed.size))
    for a in range(4):
        np.add.at(u, (idx[removed, a], np.arange(removed.size)), wts[removed, a])
    z = ninv(u)
    m = np.eye(removed.size) - u.T @ z
    if np.linalg.cond(m) > 1e10:
        return None
    yr = y_img.reshape(-1)[removed]
    # (N - U U^T) x = r - U y_r  with  N^-1 r = z0
    base = z0 - z @ yr
    return base + z @ np.linalg.solve(m, u.T @ base)


def _components(mask):
    """4-connected components of a boolean image (small, numpy-only BFS)."""
    labels, comps = -np.ones(mask.shape, dtype=int), []
    for start in zip(*np.nonzero(mask)):
        if labels[start] >= 0:
            continue
        stack, members = [start], []
        labels[start] = len(comps)
        while stack:
            r, c = stack.pop(); members.append((r, c))
            for rr, cc in ((r + 1, c), (r - 1, c), (r, c + 1), (r, c - 1)):
                if 0 <= rr < mask.shape[0] and 0 <= cc < mask.shape[1] and mask[rr, cc] and labels[rr, cc] < 0:
                    labels[rr, cc] = len(comps); stack.append((rr, cc))
        comps.append(np.array(members))
    return comps


def _segments(a):
    """Output indices grouped by their left native neighbour: within a group the 1-D bilinear output
    is exactly linear in the output index (constant in the clamped edge groups)."""
    x0 = np.argmax(a > 0, axis=1)
    pure = np.isclose(a.max(axis=1), 1.0)      # clamped edge outputs copy one native value
    key = x0 * 2 + pure
    return [np.flatnonzero(key == k) for k in np.unique(key)]


def _nonlinear_pixels(y, av, ah, tol):
    """Pixels where a pure bilinear upsample is locally violated (second difference inside a linear
    segment, along rows or columns). Local by construction: a pasted pixel cannot spread."""
    flag = np.zeros(y.shape, dtype=bool)
    for axis, a in ((1, ah), (0, av)):
        for seg in _segments(a):
            if seg.size < 2:
                continue
            block = np.take(y, seg, axis=axis)
            if seg.size == 2:
                bad = np.abs(np.diff(block, axis=axis)) > tol if (seg[0] == 0 or seg[-1] == y.shape[axis] - 1) else None
                if bad is not None:
                    hit = np.repeat(bad, 2, axis=axis)
                    idx = [slice(None)] * 2; idx[axis] = seg
                    flag[tuple(idx)] |= hit
                continue
            d2 = np.abs(np.diff(block, n=2, axis=axis)) > tol
            hit = np.zeros(block.shape, dtype=bool)
            sl = lambda k: tuple(slice(k, k + d2.shape[axis]) if ax == axis else slice(None) for ax in range(2))
            for k in range(3):
                hit[sl(k)] |= d2
            idx = [slice(None)] * 2; idx[axis] = seg
            flag[tuple(idx)] |= hit
    return flag


def recover_native_depth(depth, native_wh, tol=1e-3):
    """Invert the 40x24 -> 160x90 bilinear upsampling (both 1-D operators have full column rank).

    Pasted pixels (target/distractor, foreign content) are found by a local linearity test. A pasted
    blob's smooth interior can itself look like an upsample, so each flagged blob's bounding box,
    dilated by one native pixel, is treated as pasted. Native cells left without support are
    UNOBSERVED (NaN). Returns (native (H_n, W_n), pasted mask (H, W), max residual over used pixels).
    """
    y = np.asarray(depth, dtype=np.float64)
    ops = _operators(y.shape, native_wh)
    av, ah, idx, wts, kvi, khi = ops
    x = _solve_without(y, np.array([], dtype=int), ops, native_wh)
    fit = av @ x.reshape(native_wh[1], native_wh[0]) @ ah.T
    if np.abs(y - fit).max() <= tol:          # pure upsample: exact
        return x.reshape(native_wh[1], native_wh[0]), np.zeros(y.shape, dtype=bool), float(np.abs(y - fit).max())
    flat, n = y.reshape(-1), native_wh[0] * native_wh[1]
    pad_r = int(math.ceil(y.shape[0] / native_wh[1])); pad_c = int(math.ceil(y.shape[1] / native_wh[0]))
    pasted = _nonlinear_pixels(y, av, ah, tol)
    for _ in range(3):
        grown = pasted.copy()
        for comp in _components(pasted):
            r0, c0 = comp.min(axis=0); r1, c1 = comp.max(axis=0)
            grown[max(0, r0 - pad_r):r1 + pad_r + 1, max(0, c0 - pad_c):c1 + pad_c + 1] = True
        keep = ~grown.reshape(-1)
        support = np.bincount(idx[keep].ravel(), weights=wts[keep].ravel(), minlength=n)
        observed = support >= 1.0
        keep &= ~((~observed[idx]) & (wts > 1e-12)).any(axis=1)
        pos = np.full(n, -1); pos[observed] = np.arange(int(observed.sum())); m_obs = int(observed.sum())
        i, w, v = idx[keep], wts[keep], flat[keep]
        ii = np.where(w > 0, pos[i], 0)
        normal, rhs = np.zeros(m_obs * m_obs), np.zeros(m_obs)
        for a in range(4):
            rhs += np.bincount(ii[:, a], weights=w[:, a] * v, minlength=m_obs)
            for b in range(4):
                normal += np.bincount(ii[:, a] * m_obs + ii[:, b], weights=w[:, a] * w[:, b], minlength=m_obs * m_obs)
        x = np.full(n, np.nan)
        try:
            x[observed] = np.linalg.solve(normal.reshape(m_obs, m_obs), rhs)
        except np.linalg.LinAlgError:
            x[observed] = np.linalg.lstsq(normal.reshape(m_obs, m_obs), rhs, rcond=None)[0]
        fit = (wts * np.nan_to_num(x)[idx]).sum(axis=1)
        late = keep & (np.abs(flat - fit) > tol)       # pasted pixels the local test could not see
        if not late.any():
            break
        pasted |= late.reshape(y.shape)
    residual = float(np.abs(flat - fit)[keep].max()) if keep.any() else float("inf")
    return x.reshape(native_wh[1], native_wh[0]), ~keep.reshape(y.shape), residual


def native_camera_rays(camera, native_wh):
    """Unit rays (H_n, W_n, 3) of the native obstacle grid: same pinhole model at 40x24 (D:380-398)."""
    w, h = native_wh
    fx, fy = w / (2.0 * math.tan(camera.hfov / 2)), h / (2.0 * math.tan(camera.vfov / 2))
    vv, uu = np.mgrid[0:h, 0:w].astype(float)
    r = np.stack([np.ones_like(uu), -(uu - (w - 1) / 2.0) / fx, -(vv - (h - 1) / 2.0) / fy], axis=-1)
    return r / np.linalg.norm(r, axis=-1, keepdims=True)


def ray_envelope(origin, directions, centre, radius):
    """For unit rays from `origin`: does the ray line pass within `radius` of `centre` ahead of the
    origin? Returns (inside, entry, exit, perpendicular distance); entry/exit bound the chord."""
    c = np.asarray(centre, float) - np.asarray(origin, float)
    d = np.asarray(directions, float)
    s = d @ c
    perp = np.sqrt(np.clip(float(c @ c) - s * s, 0.0, None))
    inside = (s > 0.0) & (perp <= radius)
    half = np.sqrt(np.clip(radius * radius - perp * perp, 0.0, None))
    return inside, s - half, s + half, perp


def point_line_distance(points, origin, direction):
    """Distance of points (K,3) from the line origin + t*direction (unit)."""
    v = np.asarray(points, float) - np.asarray(origin, float)
    along = v @ np.asarray(direction, float)
    return np.sqrt(np.clip((v * v).sum(axis=-1) - along * along, 0.0, None))
