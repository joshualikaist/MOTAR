"""Randomized falsification of the prototype certificate (scratch only).

usage: python falsify.py VARIANT N SEED OUT
VARIANT: nominal | rotated | bias2cm | bias5cm | noise2cm | pose5cm | methodA
"""
import json
import math
import random
import sys
import time

sys.path.insert(0, 'orig')
sys.path.insert(0, '.')
import prototype as P  # noqa: E402
import indep as I  # noqa: E402

variant, N, seed, out = sys.argv[1], int(sys.argv[2]), int(sys.argv[3]), sys.argv[4]
rng = random.Random(seed)
YAW_MAX = {'yaw1deg': math.radians(1), 'yaw3deg': math.radians(3)}.get(variant, math.pi / 4)
shapes, ball = I.catalog()
PLAN = 0.45


def boundary_bar(path, rng):
    """One catalog bar whose plan clearance to the path is drawn in [-0.08, +0.08] m."""
    hx, hy = rng.choice(shapes)
    a, b = path
    t = rng.uniform(0.2, 1.0)
    p = (a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]))
    L = math.hypot(b[0] - a[0], b[1] - a[1])
    nx, ny = -(b[1] - a[1]) / L, (b[0] - a[0]) / L
    if rng.random() < 0.5:
        nx, ny = -nx, -ny
    want = PLAN + rng.uniform(-0.08, 0.08)
    yaw = 0.0 if variant not in ('rotated', 'yaw1deg', 'yaw3deg') else rng.uniform(-YAW_MAX, YAW_MAX)
    lo, hi = 0.0, 3.0               # bisection on centre offset along the normal
    for _ in range(60):
        mid = (lo + hi) / 2
        poly = I.rect_poly(p[0] + nx * mid, p[1] + ny * mid, hx, hy, yaw)
        if I.clearance(path, [poly], 0.0) < want:
            lo = mid
        else:
            hi = mid
    poly = I.rect_poly(p[0] + nx * hi, p[1] + ny * hi, hx, hy, yaw)
    return poly


rows = []
t_start = time.time()
for k in range(N):
    yaw_mode = 'rot' if variant in ('rotated', 'yaw1deg', 'yaw3deg') else 'axis'
    path = I.random_path(rng)
    polys, _ = I.random_scene(rng, shapes, n_bars=rng.randint(1, 8), yaw_mode=yaw_mode, yaw_max=YAW_MAX)
    if k % 2 == 1:
        bar = boundary_bar(path, rng)
        if I.clearance(((0.0, 0.0), (0.0, 0.0)), [bar], 0.0) > 0.3:
            polys = polys + [bar]
    kw = {}
    if variant == 'bias2cm':
        kw['range_bias'] = 0.02
    elif variant == 'bias5cm':
        kw['range_bias'] = 0.05
    elif variant == 'noise2cm':
        kw.update(range_sigma=0.02, rng=rng)
    elif variant == 'pose5cm':
        ang = rng.uniform(-math.pi, math.pi)
        kw['true_origin'] = (0.05 * math.cos(ang), 0.05 * math.sin(ang))
    rays = I.scan(polys, **kw)
    # Ground truth is always evaluated from the TRUE start (pose error moves the real path).
    true_path = path
    if variant == 'pose5cm':
        o = kw['true_origin']
        true_path = ((path[0][0] + o[0], path[0][1] + o[1]), (path[1][0] + o[0], path[1][1] + o[1]))
    plan_c = I.clearance(true_path, polys, PLAN)
    phys_c = I.clearance(true_path, polys, ball)
    t0 = time.perf_counter()
    r = P.certify([list(path[0]), list(path[1])], rays, shapes, 'A' if variant == 'methodA' else 'B',
                  axis_aligned_prior=True)
    dt = time.perf_counter() - t0
    rows.append({'certified': r['certified'], 'reason': r['reason'], 'nodes': r.get('nodes'),
                 'plan_clearance': plan_c, 'phys_clearance': phys_c, 'seconds': dt,
                 'length': math.hypot(path[1][0], path[1][1]), 'n_bars': len(polys)})

summary = {
    'variant': variant, 'N': N, 'seed': seed, 'wall_s': time.time() - t_start,
    'certified': sum(r['certified'] for r in rows),
    'plan_feasible': sum(r['plan_clearance'] > 1e-9 for r in rows),
    'FALSE_CERT_plan': sum(r['certified'] and r['plan_clearance'] <= 1e-9 for r in rows),
    'FALSE_CERT_physical': sum(r['certified'] and r['phys_clearance'] <= 1e-9 for r in rows),
    'worst_certified_plan_clearance': min([r['plan_clearance'] for r in rows if r['certified']], default=None),
    'worst_certified_phys_clearance': min([r['phys_clearance'] for r in rows if r['certified']], default=None),
    'feasible_but_not_certified': sum((not r['certified']) and r['plan_clearance'] > 1e-9 for r in rows),
    'reasons': {},
}
for r in rows:
    summary['reasons'][r['reason']] = summary['reasons'].get(r['reason'], 0) + 1
json.dump({'summary': summary, 'rows': rows}, open(out, 'w'))
print(json.dumps(summary))
