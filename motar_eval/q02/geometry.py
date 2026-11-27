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
    to their bounding boxes); `foreign` = pixels flagged by the window test along BOTH their row and
    their column (or by the residual test) that the recovered obstacle grid does not reproduce
    (|y - fit| > tol, or a cell unobserved), minus pixels at the no-return fill value. A blend pixel
    beside a paste is usually flagged along one direction only, because a window is flagged as a whole;
    obstacle pixels inside the paste's window-aligned box can still be foreign. The smooth interior of
    a large paste is excluded but need not be foreign. Not a segmentation of the paste."""
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


def _window_flags(y, av, ah, tol):
    """Pixels where a pure bilinear upsample is locally violated: along every column (rows operator)
    and every row (columns operator), each two-segment window must lie in the span of its native
    columns. This sees a jump inside a segment and a jump exactly on a segment boundary; the whole
    window of the offending line is flagged, so a paste's outline is closed. By the separable
    structure, an image whose rows and columns all pass is an upsample. Returns the flags of the
    column-direction and the row-direction test separately."""
    by_col, by_row = np.zeros(y.shape, dtype=bool), np.zeros(y.shape, dtype=bool)
    for w, proj in _pair_windows(av):            # columns of the image, window over rows
        bad = (np.abs(proj @ y[w, :]) > tol).any(axis=0)
        by_col[np.ix_(w, np.flatnonzero(bad))] = True
    for w, proj in _pair_windows(ah):            # rows of the image, window over columns
        bad = (np.abs(y[:, w] @ proj.T) > tol).any(axis=1)
        by_row[np.ix_(np.flatnonzero(bad), w)] = True
    return by_col, by_row


def _nonlinear_pixels(y, av, ah, tol):
    by_col, by_row = _window_flags(y, av, ah, tol)
    return by_col | by_row


def recover_native_depth(depth, native_wh, tol=1e-3, max_passes=8, fill_value=None):
    """Invert the 40x24 -> 160x90 bilinear upsampling (both 1-D operators have full column rank).

    Pasted pixels (target/distractor, foreign content) are found by a local two-segment window test
    (_nonlinear_pixels) that also sees jumps on segment boundaries, so a paste's outline is closed;
    flagged pixels are grouped on the 1-px-dilated mask (8-connected), and each group's bounding box,
    grown by one native pixel, is excluded. The smooth interior of a paste can itself look like an
    upsample: a blob whose box spans two opposite image borders may enclose such a region without a
    closed boundary, so it is reported (PASTE_SPANS_OPPOSITE_BORDERS) rather than trusted. Pixels
    inconsistent with the solve are excluded and the solve repeated until none remain (else
    NOT_CONVERGED). Native cells without support are UNOBSERVED (NaN); if the remaining normal matrix
    is ill-conditioned the grid is not determined (ILL_CONDITIONED, all NaN). A pixel at `fill_value`
    (the no-return fill) is never foreign: it is indistinguishable from "no hit".
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
    by_col, by_row = _window_flags(y, av, ah, tol)
    pasted = by_col | by_row                     # union: closes the outline for the exclusion
    core = by_col & by_row                       # both directions: the pixel itself deviates (paste candidates)
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
        core |= late.reshape(y.shape)            # inconsistent with fully observed cells: a pixel-level deviation
    residual = float(np.abs(flat - fit)[keep].max()) if keep.any() else float("inf")
    excluded = ~keep
    explained = (observed[idx] | (wts <= 1e-12)).all(axis=1) & (np.abs(flat - (wts * np.nan_to_num(x)[idx]).sum(axis=1)) <= tol)
    foreign = core.reshape(-1) & ~explained       # flagged in both directions, not reproduced by the obstacle grid
    if fill_value is not None:
        foreign &= np.abs(flat - fill_value) > tol
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


# --- Q02A-v2.5/v2.6: the declared target sphere as the paste model ---------------------------------
_FULL_RAYS = {}
SPHERE_OK = "OK"                                   # one consistent, uncontradicted sphere
SPHERE_NONE = "NO_CONSISTENT_SPHERE"              # no candidate lies on a sphere of the declared radius
SPHERE_VETOED = "MOST_CONSISTENT_SPHERE_CONTRADICTED"
SPHERE_TIE = "COMPETING_SPHERES"                  # distinct uncontradicted spheres explain equally many pixels
SPHERE_WEAK = "TOO_FEW_INLIERS"                   # < 3 inliers and other nearby candidates unexplained
SPHERE_SHAPE = "SHAPE_INCONSISTENT"               # same-object pixels near the sphere mostly off its surface
TARGET_SPHERE_OK, TARGET_SPHERE_NONE = SPHERE_OK, SPHERE_NONE   # v2.5 names


@dataclass(frozen=True)
class SphereFit:
    """Geometric consistency, contradiction and ranking kept apart (v2.6). `centre`/`mask` describe
    the most consistent sphere (max inliers) even when it is contradicted; only status OK may be
    used as footprint evidence. `inlier_mask` = candidate pixels on that sphere; `mask` adds
    non-candidate pixels whose depth equals its surface (the smooth interior of large pastes)."""
    status: str
    centre: object
    mask: object
    inlier_mask: object
    inliers: int
    contradictions: int        # farther pixels in its disc core that lie on a distinct target-like sphere (veto)
    farther_unexplained: int   # farther pixels in its disc core on no such sphere (obstacle blends; recorded, no veto)
    rival_inliers: int         # best distinct, uncontradicted competitor
    band_pixels: int           # candidates near it (lateral <= 2R, depth within its band)


def _full_rays(camera, shape):
    key = (camera.fx, camera.fy, camera.cx, camera.cy, tuple(shape))
    if key not in _FULL_RAYS:
        vv, uu = np.mgrid[0:shape[0], 0:shape[1]]
        _FULL_RAYS[key] = pixel_rays(uu, vv, camera)
    return _FULL_RAYS[key]


def fit_target_sphere(depth, candidates, camera, radius, eps=5e-3, inner=0.02, max_seeds=64, min_inliers=3,
                      min_band_fraction=0.5):
    """Sphere of the declared radius that explains the candidate (non-upsample) pixels, as evidence.

    The production target in the declared profile is an analytic sphere pasted with its exact ray
    depth wherever it is the nearest surface (navrl_detector.py:111-132), so its pixels lie on one
    sphere of radius `radius` (Q01 frames: residual <= 1e-4 m). Candidates seed sphere centres (pixel
    point + radius along its ray), refined by Gauss-Newton on |p - c| = radius; a sphere's inliers are
    the front-facing candidates within eps of its surface (geometric consistency).

    v2.6 (Opus review G1): the ranking is by consistency alone, and the most consistent sphere is
    judged with its OWN contradictions: candidate pixels inside its disc core that show a farther
    surface lying on a distinct target-like sphere (>= min_inliers inliers) -- a second target-like
    surface behind it, impossible for the single declared target. Farther pixels on no such sphere are
    obstacle-render blends beside the paste (bilinear mixes toward the 10 m fill); they are counted but
    cannot veto, and they never support anything. A contradicted best sphere is not replaced by a
    lower-ranked one (the v2.5 score let a 2-pixel blend sphere win): the paste is then ambiguous. Two distinct uncontradicted spheres
    with equal inliers are ambiguous. Fewer than `min_inliers` inliers is too weak unless the sphere
    explains every candidate near it (2 points always fit a 0.15 m sphere). Same-object candidates
    near it (lateral <= 2R, depth within [front - R, centre + R]) that are mostly off its surface mean
    the paste is not this sphere (G2: a box target). Thresholds are declared geometry checks, not
    probabilities, and passing them does not establish identity."""
    d = np.asarray(depth, dtype=np.float64)
    o = np.asarray(camera.camera_offset, dtype=np.float64)
    rays_all = _full_rays(camera, d.shape)
    empty = np.zeros(d.shape, dtype=bool)
    none = SphereFit(SPHERE_NONE, None, empty, empty, 0, 0, 0, 0, 0)
    if candidates is None:
        return none
    v, u = np.nonzero(candidates)
    if v.size == 0:
        return none
    rays = rays_all[v, u]; t = d[v, u]; pts = o + t[:, None] * rays
    order = np.argsort(t)
    seeds = order if order.size <= max_seeds else order[np.linspace(0, order.size - 1, max_seeds).astype(int)]
    spheres = []
    for i in seeds:
        c = pts[i] + radius * rays[i]
        for tol in (0.05, 0.02, eps):
            inl = np.abs(np.linalg.norm(pts - c, axis=1) - radius) < tol
            if not inl.any():
                break
            for _ in range(3):
                q = pts[inl] - c; dq = np.linalg.norm(q, axis=1)
                c = c + np.linalg.lstsq(-q / dq[:, None], -(dq - radius), rcond=None)[0]
        dist = np.linalg.norm(pts - c, axis=1)
        front = np.einsum("ij,ij->i", pts - c, rays) < 0
        inl = (np.abs(dist - radius) < eps) & front
        if not inl.any():
            continue
        oc = c - o; s = rays @ oc; perp2 = oc @ oc - s * s
        core = (s > 0) & (perp2 <= (radius - inner) ** 2)
        surface = s - np.sqrt(np.clip(radius * radius - perp2, 0.0, None))
        farther = core & (t > surface + eps)
        near = (s > 0) & (perp2 <= (2.0 * radius) ** 2)
        crange = float(np.linalg.norm(oc))
        band = near & (t >= crange - 2.0 * radius) & (t <= crange + radius)
        spheres.append((int(inl.sum()), farther, c, inl, int(near.sum()), int(band.sum())))
    if not spheres:
        return none
    spheres.sort(key=lambda x: (-x[0], int(x[1].sum())))           # ranking = geometric consistency only
    n_in, farther, c, inl, n_near, n_band = spheres[0]
    distinct = [x for x in spheres[1:] if np.linalg.norm(x[2] - c) > radius]
    # A farther pixel contradicts the best sphere only if it lies on a distinct target-like sphere (>= min_inliers):
    # then a second target-like surface is visible behind it, impossible for the single target. Farther pixels on no
    # such sphere are obstacle-render blends beside the paste; they are recorded but cannot veto.
    target_like = np.zeros(len(t), dtype=bool)
    for x in distinct:
        if x[0] >= min_inliers:
            target_like |= x[3]
    contra = int((farther & target_like).sum()); unexplained = int((farther & ~target_like).sum())
    rivals = [x[0] for x in distinct if not (x[1] & inl).any() and np.linalg.norm(x[2] - c) > 2.0 * radius]
    rival = max(rivals) if rivals else 0
    in_mask = empty.copy(); in_mask[v[inl], u[inl]] = True
    _, entry, _, perp = ray_envelope(o, rays_all.reshape(-1, 3), c, radius)
    surf = entry.reshape(d.shape)
    mask = (perp.reshape(d.shape) <= radius) & (surf > 0) & (np.abs(d - surf) <= eps)
    if contra > 0:
        status = SPHERE_VETOED
    elif rival >= n_in:
        status = SPHERE_TIE
    elif n_in < min_inliers and n_in < n_near:
        status = SPHERE_WEAK
    elif n_in < min_band_fraction * n_band:
        status = SPHERE_SHAPE
    else:
        status = SPHERE_OK
    return SphereFit(status, c, mask, in_mask, n_in, contra, unexplained, rival, n_band)
