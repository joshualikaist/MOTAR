"""Isolated azimuth-wrap test for the extent neighbourhood: foreground on beam 0 (az 180), a same-range hit on beam 71 (az -175) only."""
import sys, math
import numpy as np
ROOT = "/home/fair/workspaces/aerial_gym_ws/src/MOTAR-mt-p1"
sys.path.insert(0, ROOT)
from motar_eval.q02 import geometry as geo, validation as va, evidence as ev, contracts as c
LID, PROF = c.FROZEN_F_LIDAR, c.FROZEN_F_PROFILE
P = ev.EvidenceParams()
dirs = geo.lidar_directions(LID).reshape(-1, 3)
centre = np.array([-9.0, 0.0, 0.0])      # belief behind the vehicle, on beam 0's azimuth, elevation row 2 (0 deg)
for other_col, label in ((71, "beam 71 (az -175, wraps to be adjacent)"), (1, "beam 1 (az 175, adjacent without wrap)"), (3, "beam 3 (az 165, 3 columns away)"), (69, "beam 69 (az -165, 3 columns away via wrap)")):
    scan = np.full((4, 72), 12.0, dtype=np.float32)
    scan[2, 0] = 7.0                      # foreground hit on the LOS beam, 1 m before the belief interval
    scan[2, other_col] = 7.0              # same range on the other beam, lateral offset 8*sin(5 deg)=0.70 m > 0.35 m
    cats = va.classify_lidar(scan.reshape(-1), LID, PROF)
    for wraps in (True, False):
        sr = ev._ray_counts(np.zeros(3), dirs, scan.reshape(-1), cats, 12.0, centre, 0.2 + 2 * 0.1, 0.1, P, 0.2, (4, 72, wraps))
        print("  %-48s wraps=%-5s rays=%d fg=%d extended=%s" % (label, wraps, sr.rays, sr.foreground, sr.foreground_extended))
# row adjacency: a hit one row up on the same column at same range -> lateral 8*sin(10deg)=1.39 m -> extended
scan = np.full((4, 72), 12.0, dtype=np.float32); scan[2, 0] = 7.0; scan[1, 0] = 7.0
cats = va.classify_lidar(scan.reshape(-1), LID, PROF)
sr = ev._ray_counts(np.zeros(3), dirs, scan.reshape(-1), cats, 12.0, centre, 0.4, 0.1, P, 0.2, (4, 72, True))
print("  row +1 same column: extended=%s" % sr.foreground_extended)
scan = np.full((4, 72), 12.0, dtype=np.float32); scan[2, 0] = 7.0; scan[0, 0] = 7.0
cats = va.classify_lidar(scan.reshape(-1), LID, PROF)
sr = ev._ray_counts(np.zeros(3), dirs, scan.reshape(-1), cats, 12.0, centre, 0.4, 0.1, P, 0.2, (4, 72, True))
print("  row +2 same column (outside +-1 rows): extended=%s" % sr.foreground_extended)
