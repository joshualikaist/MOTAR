"""Candidate minimal fix (scratch only, monkeypatched): abstain when the score winner is not also the sphere with the
most raw inliers, i.e. when the violation term decided the ranking (the veto was cast by box pixels)."""
import numpy as np
from motar_eval.q02 import geometry as geo, evidence as ev
from motar_eval.q02.geometry import _full_rays, ray_envelope, TARGET_SPHERE_NONE, TARGET_SPHERE_OK
def fit_target_sphere_A(depth, candidates, camera, radius, eps=5e-3, inner=0.02, max_seeds=64):
    d = np.asarray(depth, dtype=np.float64); o = np.asarray(camera.camera_offset, dtype=np.float64)
    rays_all = _full_rays(camera, d.shape); v, u = np.nonzero(candidates); empty = np.zeros(d.shape, dtype=bool)
    if v.size == 0: return None, empty, TARGET_SPHERE_NONE, 0
    rays = rays_all[v, u]; t = d[v, u]; pts = o + t[:, None] * rays
    order = np.argsort(t)
    seeds = order if order.size <= max_seeds else order[np.linspace(0, order.size - 1, max_seeds).astype(int)]
    best = None; most = 0
    for i in seeds:
        c = pts[i] + radius * rays[i]
        for tol in (0.05, 0.02, eps):
            inl = np.abs(np.linalg.norm(pts - c, axis=1) - radius) < tol
            if not inl.any(): break
            for _ in range(3):
                q = pts[inl] - c; dq = np.linalg.norm(q, axis=1)
                c = c + np.linalg.lstsq(-q / dq[:, None], -(dq - radius), rcond=None)[0]
        dist = np.linalg.norm(pts - c, axis=1); front = np.einsum("ij,ij->i", pts - c, rays) < 0
        n_in = int(((np.abs(dist - radius) < eps) & front).sum())
        if n_in == 0: continue
        most = max(most, n_in)
        oc = c - o; s = rays @ oc; perp2 = oc @ oc - s * s
        core = (s > 0) & (perp2 <= (radius - inner) ** 2); surface = s - np.sqrt(np.clip(radius * radius - perp2, 0.0, None))
        score = n_in - int((core & (t > surface + eps)).sum())
        if best is None or (score, n_in) > (best[0], best[1]): best = (score, n_in, c)
    if best is None or best[0] <= 0 or best[1] < most:      # FIX A: a vetoed sphere had more inliers -> ambiguous, abstain
        return None, empty, TARGET_SPHERE_NONE, 0
    c = best[2]
    _, entry, _, perp = ray_envelope(o, rays_all.reshape(-1, 3), c, radius)
    surface = entry.reshape(d.shape)
    mask = (perp.reshape(d.shape) <= radius) & (surface > 0) & (np.abs(d - surface) <= eps)
    return c, mask, TARGET_SPHERE_OK, best[1]
def install():
    geo.fit_target_sphere = fit_target_sphere_A; ev.fit_target_sphere = fit_target_sphere_A
