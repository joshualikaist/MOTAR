"""Obstacle-blend neighbours in `foreign`: false SUPPORTED / false RANGE_CONTRADICTED with production-shaped scenes
(sphere target beside a bar, wrong tracks). Composition of the `at` pixels: true paste vs blend."""
import sys, math, collections
import numpy as np
ROOT = "/home/fair/workspaces/aerial_gym_ws/src/MOTAR-mt-p1"
sys.path.insert(0, ROOT); sys.path.insert(0, ROOT + "/tests")
import test_q02_evidence as t
from motar_eval.q02 import geometry as geo, validation as va, evidence as ev
P = ev.EvidenceParams()
def analyse(tv, bars, track_shift, flag=True):
    inp0 = t.make(target_v=tv, detected=flag, bars=bars)
    mean = np.array(inp0.track_mean); mean[:3] += np.array(track_shift)
    inp = t.make(target_v=tv, detected=flag, bars=bars, track_mean=mean)
    bg = np.asarray(t.make(target_v=tv, render_target=False, bars=bars).depth); d = np.asarray(inp.depth)
    paste = d != bg
    rep = va.validate(inp, t.CAM, t.LID); r = ev.estimate(inp)
    f = rep.depth_pasted
    v, u = np.nonzero(f); rays = geo.pixel_rays(u, v, t.CAM)
    g = ev._belief_geometry(inp, t.CAM, t.LID, P)
    inside, entry, exit_, _ = geo.ray_envelope(np.asarray(t.CAM.camera_offset), rays, g["p_v"], g["env_cam"])
    dd = d[v, u]; at = inside & (dd >= entry - P.sigma_k * g["sigma_r"] - P.range_tol_m) & (dd <= exit_ + P.sigma_k * g["sigma_r"] + P.range_tol_m)
    isp = paste[v, u]
    # true paste pixels inside the envelope / at range (regardless of foreign)
    pv, pu = np.nonzero(paste); prays = geo.pixel_rays(pu, pv, t.CAM)
    pin, pen, pex, _ = geo.ray_envelope(np.asarray(t.CAM.camera_offset), prays, g["p_v"], g["env_cam"])
    pd = d[pv, pu]; pat = pin & (pd >= pen - P.sigma_k * g["sigma_r"] - P.range_tol_m) & (pd <= pex + P.sigma_k * g["sigma_r"] + P.range_tol_m)
    return dict(hyp=r.hypothesis, ident=r.identity_status, in_fp=int(inside.sum()), at=int(at.sum()),
                at_paste=int((at & isp).sum()), at_blend=int((at & ~isp).sum()), in_blend=int((inside & ~isp).sum()),
                foreign=int(f.sum()), foreign_blend=int((f & ~paste).sum()), true_paste_in=int(pin.sum()), true_paste_at=int(pat.sum()),
                od=rep.status("obstacle_depth"), reasons=",".join(r.reasons)[:80])
print("--- A. bar at the target range beside the target; track shifted toward the bar (false SUPPORTED?) ---")
false_sup = []; cases = 0
for R in (3.0, 5.0, 8.0, 9.5):
    for gap in (0.05, 0.1, 0.2, 0.3, 0.5, 0.8):
        for dR in (0.0, 0.4, -0.4):
            bars = ((R + dR, 0.15 + gap + 0.3, 0.3),)
            for e in (0.4, 0.5, 0.6, 0.7, 0.8, 1.0, 1.2):
                a = analyse((R, 0.0, 0.0), bars, (0.0, e, 0.0)); cases += 1
                if a["ident"] == "SUPPORTED_BY_FOOTPRINT" and a["true_paste_at"] == 0:
                    false_sup.append((R, gap, dR, e, a))
print("  cases", cases, "FALSE SUPPORTED (no true paste pixel at range):", len(false_sup))
for R, gap, dR, e, a in false_sup[:40]:
    print("   R=%.1f gap=%.2f dR=%.1f track_err=%.1f -> at=%d (paste %d, blend %d) in_fp=%d foreign=%d(blend %d) true_paste_in=%d od=%s" % (R, gap, dR, e, a["at"], a["at_paste"], a["at_blend"], a["in_fp"], a["foreign"], a["foreign_blend"], a["true_paste_in"], a["od"]))
print("  by range:", collections.Counter(R for R, *_ in false_sup), " by track_err:", collections.Counter(e for *_, e, a in false_sup))
print("\n--- B. same geometry, track CORRECT: regression check (should be SUPPORTED) ---")
bad = 0; n = 0
for R in (3.0, 5.0, 8.0, 9.5):
    for gap in (0.05, 0.1, 0.2, 0.5):
        for dR in (0.0, 0.4, -0.4, -3.0):
            a = analyse((R, 0.0, 0.0), ((R + dR, 0.15 + gap + 0.3, 0.3),), (0.0, 0.0, 0.0)); n += 1
            if a["ident"] != "SUPPORTED_BY_FOOTPRINT": bad += 1; print("   NOT SUPPORTED:", R, gap, dR, a)
print("  correct-track cases", n, "not SUPPORTED:", bad)
print("\n--- C. bar at a DIFFERENT range beside the target; track shifted onto the bar (RANGE_CONTRADICTED vs CONTRADICTED) ---")
cnt = collections.Counter()
for R in (5.0, 8.0):
    for Rb in (3.0, 4.0, 6.0):
        for gap in (0.05, 0.2, 0.5):
            for e in (0.5, 0.7, 1.0):
                a = analyse((R, 0.0, 0.0), ((Rb, (0.15 + gap + 0.3) * Rb / R, 0.3),), (0.0, e * 1.0, 0.0))
                cnt[(a["ident"], a["true_paste_in"] > 0, a["at_blend"] > 0 or a["in_blend"] > 0)] += 1
print("  (identity, true paste inside, blend inside):", dict(cnt))
print("\n--- D. no-flag path: blend neighbours inside a wrong footprint -> IDENTITY_AMBIGUOUS? (oracle flag would be set, so informational) ---")
for R, gap, e in ((8.0, 0.1, 0.7), (5.0, 0.2, 0.8)):
    a = analyse((R, 0.0, 0.0), ((R, 0.15 + gap + 0.3, 0.3),), (0.0, e, 0.0), flag=False)
    print("  R=%.1f gap=%.1f err=%.1f ->" % (R, gap, e), a["hyp"], a["reasons"], "in_fp", a["in_fp"], "blend", a["in_blend"])
print("\n--- E. how far from the paste edge do foreign blend pixels extend? (target 8 m, bar 8 m beside, gap 0.5) ---")
tv = (8.0, 0.0, 0.0); bars = ((8.0, 0.95, 0.3),)
inp = t.make(target_v=tv, detected=True, bars=bars); bg = np.asarray(t.make(target_v=tv, render_target=False, bars=bars).depth); d = np.asarray(inp.depth)
paste = d != bg; rep = va.validate(inp, t.CAM, t.LID); f = rep.depth_pasted
pv, pu = np.nonzero(paste); fv, fu = np.nonzero(f & ~paste)
print("  paste cols %d..%d rows %d..%d ; foreign-blend cols %s rows %s ; depths of foreign-blend: %s" % (pu.min(), pu.max(), pv.min(), pv.max(), sorted(set(fu.tolist())), sorted(set(fv.tolist())), np.unique(np.round(d[fv, fu], 2))[:12]))
