"""Smooth-bump ceiling through validate()/estimate(): can an unflagged smooth paste create a phantom obstacle that
turns a clear LOS into OCCLUDED or hides a bar?"""
import sys, math
import numpy as np
ROOT = "/home/fair/workspaces/aerial_gym_ws/src/MOTAR-mt-p1"
sys.path.insert(0, ROOT); sys.path.insert(0, ROOT + "/tests")
import test_q02_evidence as t
from motar_eval.q02 import geometry as geo, validation as va, evidence as ev
VV, UU = np.mgrid[0:90, 0:160]
LIMIT = va.ValidationLimits().max_pasted_fraction
native = np.full((24, 40), 10.0); base = np.clip(t.AV @ native @ t.AH.T, 0, 10)
# a track at 8 m straight ahead projects to the image centre (v~44.5,u~79.5); bump centred there
print("A(m)  L(px) ay   status  excl  foreign  err_max   ds   od   hyp(no flag, track 8 m)   cam(rays,fg,clear)")
for L, ay in ((60, 0.6), (80, 0.6), (80, 1.0), (100, 0.6), (120, 0.5)):
    for A in (0.2, 0.5, 1.0, 1.5, 2.0, 3.0, 5.0):
        g = np.clip(1 - ((UU - 79.5) / L) ** 2 - ((VV - 44.5) / (L * ay)) ** 2, 0, None) ** 2
        depth = np.clip(base - A * g, 0, 10).astype(np.float32)
        rec = geo.recover_native_depth(depth, (40, 24)); seen = np.isfinite(rec.native)
        err = float(np.abs(rec.native[seen] - native[seen]).max()) if seen.any() else 0.0
        inp = t.make(render_target=False, depth=depth)
        rep = va.validate(inp, t.CAM, t.LID); r = ev.estimate(inp)
        print("%4.1f  %3d  %.1f  %-28s %.3f %5d  %.3e  %-14s %-11s %-18s %s %s" % (A, L, ay, rec.status, rec.excluded.mean(), rec.foreign.sum(), err,
              rep.status("depth_structure"), rep.status("obstacle_depth"), r.hypothesis, (r.camera.rays, r.camera.foreground, r.camera.clear), ",".join(r.reasons)[:50]))
# the hole case through estimate(): constant 2 m paste everywhere except a small hole; track at 8 m
print("\nhole case: constant 3 m paste except a 2 px hole at the centre, no flag, track 8 m:")
m = ~(((VV - 44.5) ** 2 + (UU - 79.5) ** 2) < 4)
depth = base.copy(); depth[m] = 3.0
inp = t.make(render_target=False, depth=depth.astype(np.float32)); rep = va.validate(inp, t.CAM, t.LID); r = ev.estimate(inp)
print("  ds=%s od=%s native_min=%.2f hyp=%s reasons=%s" % (rep.status("depth_structure"), rep.status("obstacle_depth"), np.nanmin(rep.depth_native) if rep.depth_native is not None else float('nan'), r.hypothesis, r.reasons))
# phantom from the constant-offset ceiling: delta 2e-3 on a disc covering the LOS (harmless, confirm hypothesis unchanged)
depth = base.copy(); disc = ((VV - 44.5) ** 2 + (UU - 79.5) ** 2) < 400; depth[disc] -= 2e-3
r = ev.estimate(t.make(render_target=False, depth=depth.astype(np.float32)))
print("\ndelta=2e-3 disc on LOS: hyp=%s reasons=%s" % (r.hypothesis, r.reasons))
