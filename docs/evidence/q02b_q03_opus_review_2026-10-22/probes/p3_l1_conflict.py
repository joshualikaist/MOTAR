import sys
from pathlib import Path
import numpy as np
ROOT = Path("/home/fair/workspaces/aerial_gym_ws/src/MOTAR-mt-p1")
sys.path.insert(0, str(ROOT)); sys.path.insert(0, str(ROOT / "tests"))
import test_q03_planner as tp
from motar_eval.q02 import reacq_planner as rp, obstacle_map as om
m = om.LocalObstacleMap()
inp, rep = tp.t3.frame_at(((1.2, 0.0, 0.3),), stamp=0.0); print(m.integrate(inp, rep).accepted)
inp, rep = tp.t3.frame_at((), stamp=0.1); print(m.integrate(inp, rep).accepted)       # bar gone: free newer than occupancy
print("state at bar centre", m.state_at((1.2, 0.0), 0.1))
print("segment", m.segment_status((0, 0), (1.5, 0), 0.45, 0.1))
b = tp.belief((8.0, 0.0))
cands, st, why = rp.plan(m, (0.0, 0.0, 1.5), np.pi / 2, (0, 0, 0), b, 0.1)
for c in cands:
    if c.kind == "ADVANCE":
        print(c.candidate_id, "L1", c.l1_observed_direction_open, "L2", c.l2_kinematic_check, c.reasons)
