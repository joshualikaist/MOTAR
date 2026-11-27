"""D5: beam-adjacency extent test at range. Geometry of 5-deg spacing vs 0.8 m max bar width; F5 far case with a bar beside;
production-faithful variant (target pasted in the camera)."""
import sys, math
import numpy as np
ROOT = "/home/fair/workspaces/aerial_gym_ws/src/MOTAR-mt-p1"
sys.path.insert(0, ROOT); sys.path.insert(0, ROOT + "/tests")
import test_q02_evidence as t
from motar_eval.q02 import geometry as geo, validation as va, evidence as ev
print("adjacent-beam lateral separation d*tan(5deg):", [(R, round(R * math.tan(math.radians(5)), 3)) for R in (6, 8, 9, 9.2, 10, 10.3, 11.5)])
print("range at which 5 deg = 0.8 m: %.2f m; = 0.4 m (half max width): %.2f m" % (0.8 / math.tan(math.radians(5)), 0.4 / math.tan(math.radians(5))))
def far_case(bars, R=10.5, in_cam=False, extra=1.0):
    base = t.make(target_v=(R, 0., 0.), target_in_camera=in_cam); mean = np.array(base.track_mean); mean[0] += extra
    r = ev.estimate(t.make(target_v=(R, 0., 0.), target_in_camera=in_cam, track_mean=mean, bars=bars))
    return r.hypothesis, r.lidar.foreground_extended, ",".join(r.reasons)[:70]
print("\n--- F5 far case (target 10.5 m hit by LiDAR at ~10.3, NOT pasted in camera, belief 11.5 m) + bar beside at the same range ---")
for off in (0.5, 0.7, 0.9, 1.0, 1.3, 1.8, 2.0, 2.7):
    print("  bar 0.3 m radius centred %.1f m beside LOS at 10.3 m:" % off, far_case(((10.3, off, 0.3),)))
print("\n--- same, PRODUCTION-FAITHFUL (target pasted in the camera since 10.5 m < 20 m far plane) ---")
for off in (0.9, 1.8):
    print("  bar %.1f m beside:" % off, far_case(((10.3, off, 0.3),), in_cam=True))
print("\n--- can a SINGLE max-width bar (radius 0.4) trigger the extent test at >= 9 m? centred on LOS and off-centre ---")
for R in (6.0, 8.0, 9.0, 9.5, 10.3):
    for off in (0.0, 0.2, 0.4):
        base = t.make(target_v=(R + 1.0, 0., 0.), render_target=False); 
        r = ev.estimate(t.make(target_v=(R + 1.0, 0., 0.), render_target=False, bars=((R, off, 0.4),)))
        print("  bar R=%.1f off=%.1f radius 0.4 -> %s extended=%s cam_fg=%d lid_fg=%d" % (R, off, r.hypothesis, r.lidar.foreground_extended, r.camera.foreground, r.lidar.foreground))
print("\n--- nearer ranges: F5 analogue at 6 m / 8 m with a target-sized hit and a bar beside (camera sees the bar, so cam fg?) ---")
for R, off in ((6.0, 0.6), (6.0, 1.0), (8.0, 0.8), (8.0, 1.2)):
    print("  R=%.1f off=%.1f:" % (R, off), far_case(((R - 0.2, off, 0.3),), R=R))
