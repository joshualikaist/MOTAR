import sys, math
from pathlib import Path
import numpy as np
ROOT = Path("/home/fair/workspaces/aerial_gym_ws/src/MOTAR-mt-p1")
sys.path.insert(0, str(ROOT)); sys.path.insert(0, str(ROOT / "tests"))
import test_q03_planner as tp
from motar_eval.q02 import reacq_planner as rp, obstacle_map as om
bars = ((1.6, 0.0, 0.3),)          # bar surface 1.3 m straight ahead (+x)
b = tp.belief((0.0, 8.0))           # belief to the left, outside the 87 deg FOV
for vel in ((0.0, 0.0, 0.0), (2.0, 0.0, 0.0)):
    cands, status, why = tp.plan(bars, b=b, vel=vel)
    print("vel", vel, "plan_status", status, why)
    for c in cands:
        print("  %-14s L1 %-4s L2 %-4s vis %.2f cur %.2f travel %.2f traj_start %s traj_end %s %s" % (
            c.candidate_id, c.l1_observed_direction_open, c.l2_kinematic_check, c.visibility_score, c.current_visibility_score,
            c.travel_time_s, c.trajectory[0][1:3], tuple(round(x, 2) for x in c.trajectory[-1][1:3]), c.reasons))
lim = rp.MotionLimits()
stop = float(rp.braking_distance(np.array([2.0]), lim.braking_table, lim.t_react_s)[0][0])
m = om.LocalObstacleMap(); inp, rep = tp.t3.frame_at(bars); m.integrate(inp, rep)
print("stopping distance from current 2.0 m/s: %.2f m; segment (0,0)->(%.2f,0) with body radius: %s" % (
    stop, stop, m.segment_status((0, 0), (stop, 0), lim.body_radius_m, 0.0)))
