import sys, math
from pathlib import Path
import numpy as np
ROOT = Path("/home/fair/workspaces/aerial_gym_ws/src/MOTAR-mt-p1")
sys.path.insert(0, str(ROOT)); sys.path.insert(0, str(ROOT / "tests"))
import test_q03_local_map as t3
from motar_eval.q02 import obstacle_map as om, contracts as c
def extent(m, origin_xy, now):
    n, cm = m.p.size_cells, m.p.cell_m
    worst_near, worst_far = 0, 0
    for i in range(n):
        for j in range(n):
            if m.free_s[i, j] == -np.inf: continue
            if m.state_at(m.origin_xy + (np.array([i, j]) + 0.5) * cm, now) != om.FREE: continue
            lo = m.origin_xy + np.array([i, j]) * cm; hi = lo + cm
            near = np.linalg.norm(np.clip(origin_xy, lo, hi) - origin_xy)
            corners = [np.linalg.norm(np.array([x, y]) - origin_xy) for x in (lo[0], hi[0]) for y in (lo[1], hi[1])]
            worst_near, worst_far = max(worst_near, near), max(worst_far, max(corners))
    return worst_near, worst_far
for yaw in (0.0, 0.37):
    # camera only: LiDAR invalid (NaN samples) -> not integrated
    m = om.LocalObstacleMap()
    inp, rep = t3.frame_at((), yaw=yaw, lidar=np.full(288, np.nan))
    r = m.integrate(inp, rep); assert r.accepted and "LIDAR_NOT_INTEGRATED" in r.reasons, r
    cam = np.array(inp.position[:2]) + np.array([math.cos(yaw), math.sin(yaw)]) * 0.10
    print("yaw %.2f camera-only: max distance to nearest point of a FREE cell %.3f m, max far corner %.3f m (far plane 10 m)" % ((yaw,) + extent(m, cam, 0.0)))
    m = om.LocalObstacleMap()
    inp, rep = t3.frame_at((), yaw=yaw, depth=np.full((90, 160), np.nan, np.float32))
    r = m.integrate(inp, rep); print("  lidar-only accepted", r.accepted, r.reasons)
    print("  lidar-only: max nearest-point %.3f m, max far corner %.3f m (no-return 12 m)" % extent(m, np.array(inp.position[:2]), 0.0))
