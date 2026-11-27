"""Box target (physical dynamics profile) with bars: fit status, correct-track support, and false SUPPORTED over a track sweep."""
import sys; sys.path.insert(0, sys.argv[0].rsplit("/", 1)[0])
from fast import *
from p8_boxtarget import obb_t
def bscene(tv, yaw, pitch, boxes):
    native = np.minimum(obox_hits(CAM_O, NR, boxes), 10.0); depth = np.clip(T.AV @ native @ T.AH.T, 0, 10)
    tt = obb_t(CAM_O, PR, tv, yaw, pitch); paste = np.isfinite(tt) & (tt < obox_hits(CAM_O, PR, boxes)) & (tt <= 20)
    depth = np.where(paste, tt, depth)
    tl = np.minimum(obox_hits(np.zeros(3), LD, boxes), geo.sphere_first_hit(np.zeros(3), LD, tv, LID.target_proxy_radius_m))
    lidar = np.where(tl < 0.2, 0.0, np.where(tl >= 12, 12.0, tl))
    return depth.astype(np.float32), lidar.astype(np.float32), paste
rng = np.random.default_rng(int(sys.argv[1])); tot = collections.Counter(); ex = []
for i in range(int(sys.argv[2])):
    R = float(rng.uniform(0.6, 9.0)); az = math.radians(rng.uniform(-30, 30)); el = math.radians(rng.uniform(-15, 15))
    dr = np.array([math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el)]); tv = CAM_O + R * dr
    hor = np.array([dr[0], dr[1], 0]); hor /= np.linalg.norm(hor); side = np.array([-hor[1], hor[0], 0])
    boxes = []
    if rng.random() < 0.7:                      # occluder edge near the LOS in front
        s = float(rng.uniform(0.4, max(0.5, R - 0.3))); h = float(rng.uniform(0.2, 0.4))
        ctr = CAM_O + (s + h) * hor + float(rng.choice([-1, 1])) * (h + rng.normal(0, 0.06) * s / R) * side
        boxes.append((ctr[0], ctr[1], h, h, -1.0, float(rng.choice([1.0, 2.0])), 0.0))
    if rng.random() < 0.5:                      # bar behind
        ctr = CAM_O + (R + float(rng.uniform(0.4, 2.0))) * hor + float(rng.uniform(-0.3, 0.3)) * side
        boxes.append((ctr[0], ctr[1], 0.3, 0.3, -1.0, 2.0, 0.0))
    boxes = [b for b in boxes if not (abs(b[0] - tv[0]) < b[2] + 0.25 and abs(b[1] - tv[1]) < b[3] + 0.25)]
    depth, lidar, paste = bscene(tv, float(rng.uniform(0, math.pi)), float(rng.uniform(-0.3, 0.3)), boxes)
    if paste.sum() < 2: tot["noflag"] += 1; continue
    inp0 = T.make(target_v=tuple(tv), detected=True, depth=depth, lidar=lidar.reshape(-1))
    rep = va.validate(inp0, CAM, LID)
    c, mask, st, nin = geo.fit_target_sphere(inp0.depth, rep.depth_pasted, CAM, CAM.target_radius)
    sc = dict(target_v=tv, yaw=0.0, depth=depth, lidar=lidar, paste=paste, rep=rep, c=c, mask=mask, st=st, nin=nin, d=np.asarray(inp0.depth, float), inp0=inp0)
    tot["scenes"] += 1; tot["sphere_found"] += st == "OK"
    o, e = sweep(sc, lats=(0.0, 0.2), rng=rng)
    for k, v in o.items(): tot[k] += v
    if o["FALSE"]: tot["scenes_with_FALSE"] += 1; ex.append((round(R, 2), int(paste.sum()), e[:3]))
print(dict(tot)); print(ex[:8])
