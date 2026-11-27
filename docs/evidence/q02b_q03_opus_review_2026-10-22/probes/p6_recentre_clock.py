import sys, math
from pathlib import Path
import numpy as np
ROOT = Path("/home/fair/workspaces/aerial_gym_ws/src/MOTAR-mt-p1")
sys.path.insert(0, str(ROOT)); sys.path.insert(0, str(ROOT / "tests"))
import test_q03_local_map as t3, test_q02b_timing as qb
from motar_eval.q02 import obstacle_map as om, contracts as c
# (a) re-anchoring: bar at world (3, 2) seen at t=0 from (0,0); then frame at t=0.1 from (6.3, -1.7) facing away (yaw=-pi/2+...)
m = om.LocalObstacleMap()
inp, rep = t3.frame_at(((3.0, 2.0, 0.3),), stamp=0.0); m.integrate(inp, rep)
o0 = m.origin_xy.copy()
inp, rep = t3.frame_at(((3.0, 2.0, 0.3),), pos_xy=(6.3, -4.7), yaw=-2.0, stamp=0.1); r = m.integrate(inp, rep)
print("origin moved by", m.origin_xy - o0, r.accepted)
ii, jj = np.nonzero(m.occ_s == 0.0)
pts = m.origin_xy + (np.stack([ii, jj], 1) + 0.5) * m.p.cell_m
d = np.linalg.norm(pts - np.array([3.0, 2.0]), axis=1)
print("old-only occupied cells:", len(d), "distance to bar centre min/max %.3f/%.3f (radius 0.3)" % (d.min(), d.max()))
print("state at bar surface (2.69,2.0) @0.1:", m.state_at((2.69, 2.0), 0.1))
# (b) one packet with an inconsistent decision clock poisons the rest of the episode
rec = qb.ShadowRecorder("H1"); rec.reset(1)
out = []
for k in range(12):
    inp = qb.frame(k, **qb.SEEN)
    if k == 5:
        inp = qb.q2a.make(decision=0.1 * k + 0.05, detected=True, cam_clock=c.Clock(0.55, 0.55, "DECLARED_SIM_SCHEDULE"),
                          lidar_clock=c.Clock(0.55, 0.55, "DECLARED_SIM_SCHEDULE"))
    d, _ = rec.observe(1, k, inp, qb.ORACLE_PROXY)
    out.append((k, d.decision, d.reasons[0]))
print(out[4:])
