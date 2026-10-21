"""D4: would `residual = 0 when nothing is kept` accept a pure-noise frame? Emulate via in-process mock (no repo change)."""
import sys
from unittest import mock
import numpy as np
ROOT = "/home/fair/workspaces/aerial_gym_ws/src/MOTAR-mt-p1"
sys.path.insert(0, ROOT); sys.path.insert(0, ROOT + "/tests")
import test_q02_evidence as t
from motar_eval.q02 import geometry as geo, validation as va, evidence as ev
noise = np.random.default_rng(5).uniform(1, 9, (90, 160))
rec = geo.recover_native_depth(noise, (40, 24), fill_value=10.0)
print("pure noise: status=%s excluded=%.3f residual=%s foreign=%d observed=%d" % (rec.status, rec.excluded.mean(), rec.residual, rec.foreign.sum(), np.isfinite(rec.native).sum()))
orig = geo.recover_native_depth
def patched(depth, wh, **kw):
    r = orig(depth, wh, **kw)
    if r.excluded.all():
        return geo.NativeDepth(r.native, r.excluded, r.foreign, 0.0, r.status)
    return r
with mock.patch.object(va, "recover_native_depth", patched):
    for name, kw in (("pure noise, no flag", dict(render_target=False, depth=noise)), ("pure noise, flag", dict(render_target=False, depth=noise, detected=True)),
                     ("target 0.32 m, flag", dict(target_v=(0.32, 0., 0.), detected=True)), ("target 0.32 m, no flag", dict(target_v=(0.32, 0., 0.)))):
        inp = t.make(**kw); rep = va.validate(inp, t.CAM, t.LID); r = ev.estimate(inp)
        print("  [residual:=0 when nothing kept] %-24s ds=%s od=%s camera_usable=%s -> %s %s reasons=%s" % (name, rep.status("depth_structure"), rep.status("obstacle_depth"), rep.camera_usable, r.hypothesis, r.identity_status, ",".join(r.reasons)[:70]))
print("current v2.2 behaviour:")
for name, kw in (("pure noise, no flag", dict(render_target=False, depth=noise)), ("pure noise, flag", dict(render_target=False, depth=noise, detected=True)),
                 ("target 0.32 m, flag", dict(target_v=(0.32, 0., 0.), detected=True))):
    inp = t.make(**kw); rep = va.validate(inp, t.CAM, t.LID); r = ev.estimate(inp)
    print("  %-24s ds=%s od=%s -> %s %s reasons=%s" % (name, rep.status("depth_structure"), rep.status("obstacle_depth"), r.hypothesis, r.identity_status, ",".join(r.reasons)[:70]))
