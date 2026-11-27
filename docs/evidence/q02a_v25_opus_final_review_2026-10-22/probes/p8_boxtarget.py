"""Claim check: 'A box-shaped target (physical dynamics, half extents 0.14/0.14/0.06) finds no consistent sphere,
so identity is never SUPPORTED there.' Box target pasted with exact ray depth where nearest (navrl_detector.py:81-108)."""
import sys; sys.path.insert(0, sys.argv[0].rsplit("/", 1)[0])
from fast import *
HE = np.array([0.14, 0.14, 0.06])
def obb_t(origin, dirs, centre, yaw, pitch=0.0):
    c, s = math.cos(yaw), math.sin(yaw)
    rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])
    cp, sp = math.cos(pitch), math.sin(pitch)
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rot = rz @ ry                       # box -> world
    ol = rot.T @ (np.asarray(origin, float) - centre); dl = dirs @ rot
    with np.errstate(divide="ignore", invalid="ignore"):
        t1 = (-HE - ol) / dl; t2 = (HE - ol) / dl
    tmin = np.max(np.minimum(t1, t2), axis=-1); tmax = np.min(np.maximum(t1, t2), axis=-1)
    ok = (tmax >= np.maximum(tmin, 0.0))
    t = np.where(tmin >= 0, tmin, tmax)
    return np.where(ok, t, np.inf)
cnt = collections.Counter(); rng = np.random.default_rng(5); ex = []
for R in (0.6, 1.0, 1.5, 2.0, 3.0, 5.0, 8.0):
    for k in range(12):
        yaw = float(rng.uniform(0, math.pi)); pitch = float(rng.uniform(-0.3, 0.3))
        az = math.radians(rng.uniform(-30, 30)); el = math.radians(rng.uniform(-15, 15))
        dr = np.array([math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el)]); tv = CAM_O + R * dr
        native = np.full((24, 40), 10.0); depth = np.clip(T.AV @ native @ T.AH.T, 0, 10)
        tt = obb_t(CAM_O, PR, tv, yaw, pitch); paste = np.isfinite(tt) & (tt <= 20)
        depth = np.where(paste, tt, depth).astype(np.float32)
        lidar = np.full(288, 12.0, np.float32)
        for name, trk in (("correct", tv), ("far+1.0", tv + 1.0 * dr), ("near-1.0", tv - 1.0 * dr)):
            inp = T.make(target_v=tuple(trk), detected=True, depth=depth, lidar=lidar)
            r = ev.estimate(inp)
            cnt[(name, r.identity_status, r.target_sphere_found)] += 1
            if name == "correct" and r.target_sphere_found and len(ex) < 4:
                ex.append((R, int(paste.sum()), r.target_pixels_in_footprint, r.target_pixels_at_track_range))
for k, v in sorted(cnt.items()): print(k, v)
print("examples (R, box pixels, sphere px in footprint, at range):", ex)
