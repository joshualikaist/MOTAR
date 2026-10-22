import sys; sys.path.insert(0, sys.argv[0].rsplit("/", 1)[0])
from tl import *
seed = int(sys.argv[2]); n_scenes = int(sys.argv[3]); out = sys.argv[1]
rng = np.random.default_rng(7000 + seed)
tot = collections.Counter(); scenes = []; t0 = time.time()
def ok_box(b, tv):
    cx, cy, hx, hy, z0, z1, yw = b
    rad = math.hypot(hx, hy)
    if math.hypot(tv[0] - cx, tv[1] - cy) < rad + 0.2 and z0 - 0.2 < tv[2] < z1 + 0.2: return False
    if math.hypot(cx, cy) < rad + 0.4: return False
    return True
i = 0
while tot["scenes"] < n_scenes and i < 20 * n_scenes:
    i += 1
    R = float(rng.uniform(0.6, 12.0)); az = math.radians(rng.uniform(-38, 38)); el = math.radians(rng.uniform(-20, 20))
    dr = np.array([math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el)])
    tv = CAM_O + R * dr
    if not (-0.8 <= tv[2] <= 2.0): continue
    boxes = []
    hor = np.array([dr[0], dr[1], 0.0]); hor /= np.linalg.norm(hor); side = np.array([-hor[1], hor[0], 0.0])
    # one occluder near the line of sight in front of the target, edge/corner close to the LOS
    n_occ = int(rng.choice([0, 1, 2], p=[0.2, 0.6, 0.2]))
    for _ in range(n_occ):
        s = float(rng.uniform(0.4, max(0.5, R - 0.3)))
        hx, hy = float(rng.uniform(0.2, 0.4)), float(rng.uniform(0.2, 0.4)); yw = float(rng.uniform(0, math.pi / 2))
        rad_eff = 0.5 * (hx + hy)
        lat = float(rng.choice([-1, 1])) * (rad_eff + float(rng.normal(0.0, 0.08)) * s / max(R, 0.5))
        pool = Z2 if rng.random() < 0.5 else Z3
        ctr = CAM_O + s * hor + lat * side + rad_eff * hor
        b = (float(ctr[0]), float(ctr[1]), hx, hy, pool[0], pool[1], yw)
        if ok_box(b, tv): boxes.append(b)
    # background / beside bars, sometimes touching an existing bar (compound wall)
    for _ in range(int(rng.choice([0, 1, 2, 3], p=[0.2, 0.4, 0.3, 0.1]))):
        if boxes and rng.random() < 0.3:
            cx, cy, hx, hy, z0, z1, yw = boxes[int(rng.integers(len(boxes)))]
            ang = rng.uniform(0, 2 * math.pi); dd = float(rng.uniform(0.2, 0.4))
            b = (cx + dd * math.cos(ang), cy + dd * math.sin(ang), float(rng.uniform(0.2, 0.4)), float(rng.uniform(0.2, 0.4)), z0, z1, float(rng.uniform(0, math.pi / 2)))
        else:
            s = float(rng.uniform(0.5, min(11.0, R + 3.0))); lat = float(rng.uniform(-1.2, 1.2))
            ctr = CAM_O + s * hor + lat * side
            pool = Z2 if rng.random() < 0.5 else Z3
            b = (float(ctr[0]), float(ctr[1]), float(rng.uniform(0.2, 0.4)), float(rng.uniform(0.2, 0.4)), pool[0], pool[1], float(rng.uniform(0, math.pi / 2)))
        if ok_box(b, tv): boxes.append(b)
    yaw = float(rng.uniform(-math.pi, math.pi))
    sc = cache(tv, boxes, yaw)
    if sc is None: tot["no_report"] += 1; continue
    if sc["paste"].sum() < 2: tot["noflag"] += 1; continue
    tot["scenes"] += 1; w = winner(sc); tot["win_" + w] += 1
    o, ex = sweep2(sc, rng=rng)
    for k, v in o.items(): tot[k] += v
    if o["FALSE"]: tot["scenes_with_FALSE"] += 1
    if o["FALSE_MATERIAL"]: tot["scenes_with_MATERIAL"] += 1
    rt = tracker_like(sc, rng)
    for k, v in rt.items(): tot["TL_" + k] += v
    if o["LOST_correct"]: tot["scenes_with_LOST"] += 1
    scenes.append(dict(R=R, az=math.degrees(az), el=math.degrees(el), boxes=boxes, yaw=yaw, paste=int(sc["paste"].sum()), cand=int(sc["rep"].depth_pasted.sum()),
                       winner=w, nin=sc["nin"], od=sc["rep"].status("obstacle_depth"), sweep=dict(o), false_ex=ex[:8]))
print(seed, dict(tot), round(time.time() - t0, 1))
for s in [s for s in scenes if s["sweep"].get("FALSE")][:12]: print(" F", {k: s[k] for k in ("R", "paste", "cand", "winner", "nin", "sweep", "false_ex")})
for s in [s for s in scenes if s["sweep"].get("LOST_correct")][:8]: print(" L", {k: s[k] for k in ("R", "paste", "cand", "winner", "nin", "sweep")})
json.dump(dict(total=dict(tot), scenes=scenes), open(out, "w"), default=float)
