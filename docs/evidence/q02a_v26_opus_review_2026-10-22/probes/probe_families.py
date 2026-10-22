import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from probe_common import *

def ok_box(b, tv):
    cx, cy, hx, hy, z0, z1, yw = b; rad = math.hypot(hx, hy)
    return not (math.hypot(tv[0] - cx, tv[1] - cy) < rad + 0.2 and z0 - 0.2 < tv[2] < z1 + 0.2) and math.hypot(cx, cy) >= rad + 0.4

def gen_random(rng, rmin=0.6, rmax=12.0):
    """The author's/previous reviewer's production-shaped generator, with a configurable range band."""
    while True:
        R = float(rng.uniform(rmin, rmax)); az = math.radians(rng.uniform(-38, 38)); el = math.radians(rng.uniform(-20, 20))
        dr = np.array([math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el)]); tv = CAM_O + R * dr
        if not (-0.8 <= tv[2] <= 2.0): continue
        boxes = []; hor = np.array([dr[0], dr[1], 0.0]); hor /= np.linalg.norm(hor); side = np.array([-hor[1], hor[0], 0.0])
        for _ in range(int(rng.choice([0, 1, 2], p=[0.2, 0.6, 0.2]))):
            s = float(rng.uniform(0.4, max(0.5, R - 0.3))); hx, hy = float(rng.uniform(0.2, 0.4)), float(rng.uniform(0.2, 0.4))
            yw = float(rng.uniform(0, math.pi / 2)); re = 0.5 * (hx + hy)
            lat = float(rng.choice([-1, 1])) * (re + float(rng.normal(0.0, 0.08)) * s / max(R, 0.5))
            pool = Z2 if rng.random() < 0.5 else Z3; ctr = CAM_O + s * hor + lat * side + re * hor
            b = (float(ctr[0]), float(ctr[1]), hx, hy, pool[0], pool[1], yw)
            if ok_box(b, tv): boxes.append(b)
        for _ in range(int(rng.choice([0, 1, 2, 3], p=[0.2, 0.4, 0.3, 0.1]))):
            if boxes and rng.random() < 0.3:
                cx, cy, hx, hy, z0, z1, yw = boxes[int(rng.integers(len(boxes)))]
                ang = rng.uniform(0, 2 * math.pi); dd = float(rng.uniform(0.2, 0.4))
                b = (cx + dd * math.cos(ang), cy + dd * math.sin(ang), float(rng.uniform(0.2, 0.4)), float(rng.uniform(0.2, 0.4)), z0, z1, float(rng.uniform(0, math.pi / 2)))
            else:
                s = float(rng.uniform(0.5, min(11.0, R + 3.0))); lat = float(rng.uniform(-1.2, 1.2)); ctr = CAM_O + s * hor + lat * side
                pool = Z2 if rng.random() < 0.5 else Z3
                b = (float(ctr[0]), float(ctr[1]), float(rng.uniform(0.2, 0.4)), float(rng.uniform(0.2, 0.4)), pool[0], pool[1], float(rng.uniform(0, math.pi / 2)))
            if ok_box(b, tv): boxes.append(b)
        return tv, boxes

def gen_gap(rng):
    """Target seen through a gap between two bars (slivers on both sides)."""
    while True:
        R = float(rng.uniform(2.0, 12.0)); az = math.radians(rng.uniform(-35, 35)); el = math.radians(rng.uniform(-15, 15))
        dr = np.array([math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el)]); tv = CAM_O + R * dr
        if not (-0.8 <= tv[2] <= 1.8): continue
        hor = np.array([dr[0], dr[1], 0.0]); hor /= np.linalg.norm(hor); side = np.array([-hor[1], hor[0], 0.0])
        s = float(rng.uniform(0.6, R - 0.5)); gap = float(rng.uniform(0.03, 0.45)) * s / R * 2.0
        off = float(rng.normal(0.0, 0.1)) * s / R
        pool = Z3 if rng.random() < 0.6 else Z2; boxes = []
        for sg in (-1, 1):
            h = float(rng.uniform(0.2, 0.4)); ctr = CAM_O + (s + h) * hor + (off + sg * (gap / 2 + h)) * side
            b = (float(ctr[0]), float(ctr[1]), h, h, pool[0], pool[1], 0.0)
            if ok_box(b, tv): boxes.append(b)
        return tv, boxes

def gen_clutter(rng):
    """Production-like clutter: 4-12 yawed bars uniform in front, target anywhere in the FOV band (no LOS bias)."""
    while True:
        R = float(rng.uniform(0.8, 14.0)); az = math.radians(rng.uniform(-40, 40)); el = math.radians(rng.uniform(-20, 20))
        dr = np.array([math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el)]); tv = CAM_O + R * dr
        if not (-0.8 <= tv[2] <= 1.8): continue
        boxes = []
        for _ in range(int(rng.integers(4, 13))):
            x = float(rng.uniform(0.5, 15.0)); y = float(rng.uniform(-9.0, 9.0)); h1, h2 = float(rng.uniform(0.2, 0.4)), float(rng.uniform(0.2, 0.4))
            pool = Z3 if rng.random() < 0.6 else Z2
            b = (x, y, h1, h2, pool[0], pool[1], float(rng.uniform(0, math.pi / 2)))
            if ok_box(b, tv): boxes.append(b)
        return tv, boxes

if __name__ == "__main__":
    fam, seed, n, out = sys.argv[1], int(sys.argv[2]), int(sys.argv[3]), sys.argv[4]
    rng = np.random.default_rng(91000 + seed); o = collections.Counter(); keep = []; t0 = time.time(); tries = 0
    gen = dict(random=lambda: gen_random(rng), far=lambda: gen_random(rng, 9.0, 19.0), gap=lambda: gen_gap(rng),
               clutter=lambda: gen_clutter(rng))[fam]
    if fam == "random": rng = np.random.default_rng(7000 + seed)   # identical to the author's generator seeding
    while o["scenes"] < n and tries < 40 * n:
        tries += 1
        tv, boxes = gen(); yaw = float(rng.uniform(-math.pi, math.pi))
        sc = J.cache(tv, boxes, yaw)
        if sc is None or sc["paste"].sum() < 2: continue
        analyse(sc, o, keep, dict(fam=fam, seed=seed, tv=[float(x) for x in tv], boxes=boxes, yaw=yaw))
    res = dict(family=fam, seed=seed, n=n, counts=dict(sorted(o.items())), worst=keep, seconds=round(time.time() - t0, 1))
    open(out, "w").write(json.dumps(res, default=float) + "\n"); print(json.dumps(dict(res, worst=keep[:5]), default=float))
