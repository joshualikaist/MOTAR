import sys
from pathlib import Path
import numpy as np
ROOT = Path("/home/fair/workspaces/aerial_gym_ws/src/MOTAR-mt-p1")
sys.path.insert(0, str(ROOT)); sys.path.insert(0, str(ROOT / "tests"))
import test_q03_planner as tp
from motar_eval.q02 import reacq_planner as rp, obstacle_map as om, validation as va, contracts as c
# (a) bar surface 0.15 m behind the vehicle (outside the camera FOV): LiDAR near-clipped beams
bars = ((-0.45, 0.0, 0.3),)
m = om.LocalObstacleMap(); inp, rep = tp.t3.frame_at(bars); r = m.integrate(inp, rep)
cat = va.classify_lidar(np.asarray(inp.lidar), c.FROZEN_F_LIDAR)
print("near-clipped beams", int((cat == c.NEAR_CLIPPED).sum()), "receipt", r.reasons, "near_warning", m.validity(0.0)["near_obstacle_warning"])
print("state of bar surface cell (-0.16,0):", m.state_at((-0.16, 0.0), 0.0), " own cell:", m.state_at((0.0, 0.0), 0.0))
cands, st, why = rp.plan(m, (0.0, 0.0, 1.5), 0.0, (0, 0, 0), tp.belief((0.0, 8.0)), 0.0)
print("plan status", st, why)
for cd in cands:
    print("  %-14s L1 %s L2 %s %s" % (cd.candidate_id, cd.l1_observed_direction_open, cd.l2_kinematic_check, cd.reasons))
# (b) a query before newer evidence: occupancy observed at t=0 and t=0.2, query at t=0.1
m = om.LocalObstacleMap()
for s in (0.0, 0.2):
    inp, rep = tp.t3.frame_at(((2.0, 0.0, 0.3),), stamp=s); m.integrate(inp, rep)
print("bar surface state @0.2:", m.state_at((1.71, 0.0), 0.2), " @0.1:", m.state_at((1.71, 0.0), 0.1))
print("segment through bar @0.2:", m.segment_status((0, 0), (3, 0), 0.0, 0.2)[0], " @0.1:", m.segment_status((0, 0), (3, 0), 0.0, 0.1)[0])
