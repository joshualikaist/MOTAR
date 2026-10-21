"""Blend-neighbour false SUPPORTED: breakdown by gap, bar range, belief sigma; plus the identity outcome when the track is right."""
import sys, collections
import numpy as np
ROOT = "/home/fair/workspaces/aerial_gym_ws/src/MOTAR-mt-p1"
sys.path.insert(0, ROOT); sys.path.insert(0, ROOT + "/tests")
import test_q02_evidence as t
from motar_eval.q02 import geometry as geo, validation as va, evidence as ev
P = ev.EvidenceParams()
def run(tv, bars, shift, cov_pos):
    inp0 = t.make(target_v=tv, detected=True, bars=bars, cov_pos=cov_pos)
    mean = np.array(inp0.track_mean); mean[:3] += np.array(shift)
    inp = t.make(target_v=tv, detected=True, bars=bars, track_mean=mean, cov_pos=cov_pos)
    bg = np.asarray(t.make(target_v=tv, render_target=False, bars=bars).depth); d = np.asarray(inp.depth); paste = d != bg
    r = ev.estimate(inp); g = ev._belief_geometry(inp, t.CAM, t.LID, P)
    pv, pu = np.nonzero(paste); prays = geo.pixel_rays(pu, pv, t.CAM)
    pin, pen, pex, _ = geo.ray_envelope(np.asarray(t.CAM.camera_offset), prays, g["p_v"], g["env_cam"])
    pd = d[pv, pu]; pat = pin & (pd >= pen - P.sigma_k * g["sigma_r"] - P.range_tol_m) & (pd <= pex + P.sigma_k * g["sigma_r"] + P.range_tol_m)
    return r.identity_status, int(pat.sum()), r.pasted_pixels_at_track_range
tot = collections.Counter(); fs = collections.Counter(); keys = []
for R in (5.0, 8.0):
    for gap in (0.05, 0.1, 0.2, 0.3, 0.5, 0.8):
        for Rb_off in (0.0, -2.0, -4.0):
            Rb = R + Rb_off
            if Rb < 2.0: continue
            ybar = (0.15 + gap + 0.3) * Rb / R        # same image-plane gap
            for e in (0.5, 0.7):
                for cov_pos in (0.01, 0.04):
                    ident, true_at, at = run((R, 0., 0.), ((Rb, ybar, 0.3),), (0., e, 0.), cov_pos)
                    k = (gap, Rb_off, cov_pos); tot[k] += 1
                    if ident == "SUPPORTED_BY_FOOTPRINT" and true_at == 0: fs[k] += 1
print("false SUPPORTED / cases by (image gap m at target range, bar range offset, cov_pos):")
for k in sorted(tot): print("  gap=%.2f bar_dR=%+.0f cov=%.2f : %d/%d" % (k[0], k[1], k[2], fs[k], tot[k]))
print("TOTAL false SUPPORTED:", sum(fs.values()), "/", sum(tot.values()))
print("by gap:", {g: (sum(v for k, v in fs.items() if k[0] == g), sum(v for k, v in tot.items() if k[0] == g)) for g in (0.05, 0.1, 0.2, 0.3, 0.5, 0.8)})
print("by cov:", {c: (sum(v for k, v in fs.items() if k[2] == c), sum(v for k, v in tot.items() if k[2] == c)) for c in (0.01, 0.04)})
print("by bar dR:", {o: (sum(v for k, v in fs.items() if k[1] == o), sum(v for k, v in tot.items() if k[1] == o)) for o in (0.0, -2.0, -4.0)})
# the minimum track error that yields a false SUPPORTED at R=8, gap 0.1, bar at 8
print("\nR=8 gap=0.1 bar at 8 m, cov 0.01: track error sweep (identity, true paste at range, at):")
for e in (0.3, 0.35, 0.4, 0.45, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.2):
    print("  e=%.2f ->" % e, run((8., 0., 0.), ((8., 0.55, 0.3),), (0., e, 0.), 0.01))
print("\nalso radial error (track beyond/before the target on the LOS), bar beside gap 0.1:")
for e in (-0.6, -0.4, 0.4, 0.6, 0.9):
    print("  radial e=%.1f ->" % e, run((8., 0., 0.), ((8., 0.55, 0.3),), (e, 0., 0.), 0.01))
