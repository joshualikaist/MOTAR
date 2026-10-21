"""Frame transforms and sensor geometry matching the audited Frozen F conventions (numpy, CPU)."""
from dataclasses import dataclass
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
RECOVERY_OK = "OK"
RECOVERY_NOT_CONVERGED = "NOT_CONVERGED"                      # new inconsistent pixels kept appearing
RECOVERY_ILL_CONDITIONED = "ILL_CONDITIONED"                  # remaining pixels do not determine the grid
RECOVERY_PASTE_SPANS_BORDERS = "PASTE_SPANS_OPPOSITE_BORDERS"  # a pasted blob may hide a self-consistent region


@dataclass(frozen=True)
class NativeDepth:
    """Result of recover_native_depth. `excluded` = pixels not used for the solve (pasted blobs grown
    to their bounding boxes); `foreign` = pixels flagged by the linearity or residual test that the
    recovered obstacle grid does not reproduce (|y - fit| > tol, or a cell unobserved): the pasted
    target/distractor pixels, possibly with up to two stencil neighbours at an edge. The smooth
    interior of a large constant paste is excluded but need not be foreign."""
    native: object
    excluded: object
    foreign: object
    residual: float
    status: str


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


def _dilate(mask):
    """3x3 dilation without wrap-around."""
    pad = np.pad(mask, 1)
    out = np.zeros_like(mask)
    for dr in range(3):
        for dc in range(3):
            out |= pad[dr:dr + mask.shape[0], dc:dc + mask.shape[1]]
    return out


def _components(mask):
    """8-connected components of a boolean image (small, numpy-only BFS)."""
    labels, comps = -np.ones(mask.shape, dtype=int), []
    for start in zip(*np.nonzero(mask)):
        if labels[start] >= 0:
            continue
        stack, members = [start], []
        labels[start] = len(comps)
        while stack:
            r, c = stack.pop(); members.append((r, c))
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    rr, cc = r + dr, c + dc
                    if 0 <= rr < mask.shape[0] and 0 <= cc < mask.shape[1] and mask[rr, cc] and labels[rr, cc] < 0:
                        labels[rr, cc] = len(comps); stack.append((rr, cc))
        comps.append(np.array(members))
    return comps


def _segments(a):
    """Output indices grouped by their left native neighbour, in output order: within a group the
    1-D bilinear output is exactly linear in the output index (constant in the clamped edge groups)."""
    x0 = np.argmax(a > 0, axis=1)
    pure = np.isclose(a.max(axis=1), 1.0)      # clamped edge outputs copy one native value
    key = x0 * 2 + pure
    return sorted((np.flatnonzero(key == k) for k in np.unique(key)), key=lambda seg: seg[0])


_WINDOW_CACHE = {}


def _pair_windows(a):
    """Windows of two adjacent segments with the projector onto the span of the (<= 3) native columns
    they depend on. A line is a 1-D upsample iff every such window lies in its span (consecutive
    windows share a native value, so the local fits chain into one native vector)."""
    key = (a.shape, a.tobytes())
    if key not in _WINDOW_CACHE:
        segs, out = _segments(a), []
        for s1, s2 in zip(segs[:-1], segs[1:]):
            w = np.concatenate([s1, s2])
            cols = np.flatnonzero(np.abs(a[w]).sum(axis=0) > 0)
            aw = a[np.ix_(w, cols)]
            out.append((w, np.eye(w.size) - aw @ np.linalg.pinv(aw)))
        _WINDOW_CACHE[key] = out
    return _WINDOW_CACHE[key]


def _nonlinear_pixels(y, av, ah, tol):
    """Pixels where a pure bilinear upsample is locally violated: along every column (rows operator)
    and every row (columns operator), each two-segment window must lie in the span of its native
    columns. This sees a jump inside a segment and a jump exactly on a segment boundary; the whole
    window of the offending line is flagged, so a paste's outline is closed. By the separable
    structure, an image whose rows and columns all pass is an upsample."""
    flag = np.zeros(y.shape, dtype=bool)
    for w, proj in _pair_windows(av):            # columns of the image, window over rows
        bad = (np.abs(proj @ y[w, :]) > tol).any(axis=0)
        flag[np.ix_(w, np.flatnonzero(bad))] = True
    for w, proj in _pair_windows(ah):            # rows of the image, window over columns
        bad = (np.abs(y[:, w] @ proj.T) > tol).any(axis=1)
        flag[np.ix_(np.flatnonzero(bad), w)] = True
    return flag


def recover_native_depth(depth, native_wh, tol=1e-3, max_passes=8):
    """Invert the 40x24 -> 160x90 bilinear upsampling (both 1-D operators have full column rank).

    Pasted pixels (target/distractor, foreign content) are found by a local two-segment window test
    (_nonlinear_pixels) that also sees jumps on segment boundaries, so a paste's outline is closed;
    flagged pixels are grouped on the 1-px-dilated mask (8-connected), and each group's bounding box,
    grown by one native pixel, is excluded. The smooth interior of a paste can itself look like an
    upsample: a blob whose box spans two opposite image borders may enclose such a region without a
    closed boundary, so it is reported (PASTE_SPANS_OPPOSITE_BORDERS) rather than trusted. Pixels
    inconsistent with the solve are excluded and the solve repeated until none remain (else
    NOT_CONVERGED). Native cells without support are UNOBSERVED (NaN); if the remaining normal matrix
    is ill-conditioned the grid is not determined (ILL_CONDITIONED, all NaN).
    """
    y = np.asarray(depth, dtype=np.float64)
    ops = _operators(y.shape, native_wh)
    av, ah, idx, wts, kvi, khi = ops
    shape_n = (native_wh[1], native_wh[0])
    x = kvi @ (av.T @ y @ ah) @ khi            # separable normal equations (Av^T Av) (x) (Ah^T Ah)
    fit = av @ x @ ah.T
    if np.abs(y - fit).max() <= tol:          # pure upsample: exact
        none = np.zeros(y.shape, dtype=bool)
        return NativeDepth(x, none, none.copy(), float(np.abs(y - fit).max()), RECOVERY_OK)
    flat, n = y.reshape(-1), native_wh[0] * native_wh[1]
    pad_r = int(math.ceil(y.shape[0] / native_wh[1])); pad_c = int(math.ceil(y.shape[1] / native_wh[0]))
    pasted = _nonlinear_pixels(y, av, ah, tol)
    status = RECOVERY_NOT_CONVERGED
    for _ in range(max_passes):
        grown, spans = pasted.copy(), False
        for comp in _components(_dilate(pasted)):
            r0, c0 = comp.min(axis=0); r1, c1 = comp.max(axis=0)
            grown[max(0, r0 - pad_r):r1 + pad_r + 1, max(0, c0 - pad_c):c1 + pad_c + 1] = True
            spans |= (r0 == 0 and r1 == y.shape[0] - 1) or (c0 == 0 and c1 == y.shape[1] - 1)
        keep = ~grown.reshape(-1)
        while True:                              # a cell is observed only by pixels that are still used
            support = np.bincount(idx[keep].ravel(), weights=wts[keep].ravel(), minlength=n)
            observed = support >= 1.0
            drop = keep & ((~observed[idx]) & (wts > 1e-12)).any(axis=1)
            if not drop.any():
                break
            keep &= ~drop
        pos = np.full(n, -1); pos[observed] = np.arange(int(observed.sum())); m_obs = int(observed.sum())
        i, w, v = idx[keep], wts[keep], flat[keep]
        ii = np.where(w > 0, pos[i], 0)
        normal, rhs = np.zeros(m_obs * m_obs), np.zeros(m_obs)
        for a in range(4):
            rhs += np.bincount(ii[:, a], weights=w[:, a] * v, minlength=m_obs)
            for b in range(4):
                normal += np.bincount(ii[:, a] * m_obs + ii[:, b], weights=w[:, a] * w[:, b], minlength=m_obs * m_obs)
        normal = normal.reshape(m_obs, m_obs)
        x = np.full(n, np.nan)
        try:
            chol = np.diag(np.linalg.cholesky(normal)) if m_obs else np.ones(1)   # no cell observed: nothing to solve
            conditioned = chol.min() >= 1e-5 * chol.max()
        except np.linalg.LinAlgError:
            conditioned = False
        if not conditioned:                      # residual from the minimum-norm fit; grid not reported
            fit_x = np.zeros(n); fit_x[observed] = np.linalg.lstsq(normal, rhs, rcond=None)[0]
            fit = (wts * fit_x[idx]).sum(axis=1)
            status = RECOVERY_ILL_CONDITIONED
            break
        if m_obs:
            x[observed] = np.linalg.solve(normal, rhs)
        fit = (wts * np.nan_to_num(x)[idx]).sum(axis=1)
        late = keep & (np.abs(flat - fit) > tol)       # pasted pixels the local test could not see
        if not late.any():
            status = RECOVERY_PASTE_SPANS_BORDERS if spans else RECOVERY_OK
            break
        pasted |= late.reshape(y.shape)
    residual = float(np.abs(flat - fit)[keep].max()) if keep.any() else float("inf")
    excluded = ~keep
    explained = (observed[idx] | (wts <= 1e-12)).all(axis=1) & (np.abs(flat - (wts * np.nan_to_num(x)[idx]).sum(axis=1)) <= tol)
    foreign = pasted.reshape(-1) & ~explained     # positively flagged, and not reproduced by the obstacle grid
    return NativeDepth(x.reshape(shape_n), excluded.reshape(y.shape), foreign.reshape(y.shape), residual, status)


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
