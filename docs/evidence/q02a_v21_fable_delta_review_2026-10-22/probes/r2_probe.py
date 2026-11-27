"""R2: azimuth wrap in the extent test, finite-height bars, adjacent-object false OCCLUDED threshold."""
import sys, math, json
import numpy as np
ROOT = "/home/fair/workspaces/aerial_gym_ws/src/MOTAR-mt-p1"
sys.path.insert(0, ROOT); sys.path.insert(0, ROOT + "/tests")
import test_q02_evidence as t
from motar_eval.q02 import geometry as geo, validation as va, evidence as ev, contracts as c
LID, CAM, PROF = t.LID, t.CAM, t.PROF
P = ev.EvidenceParams()

# 1) wrap: direct _ray_counts call with the centre behind the vehicle (az ~ 180 deg -> beams 0 and 71 adjacent)
dirs = geo.lidar_directions(LID).reshape(-1, 3)
def lidar_scan(bars, target=None):
    d = geo.lidar_directions(LID)
    tt = t._cylinder_hits(np.zeros(3), d, bars)
    if target is not None:
        tt = np.minimum(tt, geo.sphere_first_hit(np.zeros(3), d, target, LID.target_proxy_radius_m))
    return np.where(tt < LID.min_range_m, 0.0, np.where(tt >= LID.max_range_m, LID.max_range_m, tt)).astype(np.float32)
print("azimuths beam0 %.1f beam71 %.1f beam36 %.1f" % tuple(np.degrees(LID.azimuth_rad())[[0, 71, 36]]))
for label, centre, bars in (("behind, bar centred at az 180 (beam 0)", (-9.0, 0.0, 0.0), ((-8.0, 0.0, 0.3),)),
                            ("behind, bar at az 177.5 between beams 0 and 71", (-9.0, 0.0, 0.0), ((-8.0 * math.cos(math.radians(2.5)), 8.0 * math.sin(math.radians(2.5)), 0.3),)),
                            ("ahead, bar on LOS (beam 36)", (9.0, 0.0, 0.0), ((8.0, 0.0, 0.3),))):
    scan = lidar_scan(bars).reshape(-1)
    cats = va.classify_lidar(scan, LID, PROF)
    sr = ev._ray_counts(np.zeros(3), dirs, scan, cats, 12.0, np.array(centre), 0.2 + 2 * 0.1, 0.1, P, 0.2, (4, 72, True))
    srnw = ev._ray_counts(np.zeros(3), dirs, scan, cats, 12.0, np.array(centre), 0.2 + 2 * 0.1, 0.1, P, 0.2, (4, 72, False))
    hits = np.flatnonzero((cats == "VALID_HIT"))
    print("  %-50s hit beams (row,col)=%s fg=%d extended(wrap)=%s extended(nowrap)=%s" % (label, [divmod(int(h), 72) for h in hits], sr.foreground, sr.foreground_extended, srnw.foreground_extended))

# 2) finite-height bar realism: production bars are 2 m (3 m bars_h3), vehicle at z=1.5. Infinite cylinder in test scene.
print("\n--- finite-height bar on the LOS (F5 far case), elevation rows +-10 deg masked to no-hit ---")
far = t.make(target_v=(10.5, 0., 0.), target_in_camera=False)
mean = np.array(far.track_mean); mean[0] += 1.0
for bar_x in (9.5, 6.0, 4.0, 3.0):
    for height in (2.0, 3.0, None):
        inp = t.make(target_v=(10.5, 0., 0.), render_target=False, bars=((bar_x, 0., .3),), track_mean=mean)
        lid = np.array(inp.lidar).reshape(4, 72)
        if height is not None:
            # beams: el 20,10,0,-10 from z=1.5; keep a hit only if its z at the hit range lies within [0, height]
            el = LID.elevation_rad()
            for i in range(4):
                for j in range(72):
                    rr = lid[i, j]
                    if 0.2 <= rr < 12.0:
                        z = 1.5 + rr * math.sin(el[i])
                        if not (0.0 <= z <= height):
                            lid[i, j] = 12.0
        r = ev.estimate(t.make(target_v=(10.5, 0., 0.), render_target=False, bars=((bar_x, 0., .3),), track_mean=mean, lidar=lid.reshape(-1)))
        print("  bar_x=%.1f height=%s -> %-10s ext=%s lid=%s cam=%s reasons=%s" % (bar_x, height, r.hypothesis, r.lidar.foreground_extended,
              (r.lidar.rays, r.lidar.foreground), (r.camera.rays, r.camera.foreground, r.camera.censored), ",".join(r.reasons)[:70]))

# 3) false OCCLUDED: target nearer than believed (F5) + an unrelated bar at the same range at angular offset
print("\n--- F5 far case (target 10.5 m, belief 11.5 m, LiDAR-only) + unrelated bar at 10.3 m at azimuth offset ---")
for off_deg in (2.5, 5, 7.5, 10, 12.5, 15, 20):
    bars = ((10.3 * math.cos(math.radians(off_deg)), 10.3 * math.sin(math.radians(off_deg)), .3),)
    r = ev.estimate(t.make(target_v=(10.5, 0., 0.), target_in_camera=False, track_mean=mean, bars=bars))
    print("  offset %5.1f deg (lateral %.2f m) -> %-10s ext=%s lid=(rays %d fg %d) reasons=%s" % (off_deg, 10.3 * math.sin(math.radians(off_deg)), r.hypothesis, r.lidar.foreground_extended, r.lidar.rays, r.lidar.foreground, ",".join(r.reasons)[:60]))
print("\n--- same at 7 m target / 8 m belief (camera sees nothing: target_in_camera=False) ---")
near7 = t.make(target_v=(7., 0., 0.), target_in_camera=False)
m7 = np.array(near7.track_mean); m7[0] += 1.0
for off_deg in (5, 10, 15):
    bars = ((6.8 * math.cos(math.radians(off_deg)), 6.8 * math.sin(math.radians(off_deg)), .3),)
    r = ev.estimate(t.make(target_v=(7., 0., 0.), target_in_camera=False, track_mean=m7, bars=bars))
    print("  offset %5.1f deg -> %-10s ext=%s reasons=%s" % (off_deg, r.hypothesis, r.lidar.foreground_extended, ",".join(r.reasons)[:80]))
