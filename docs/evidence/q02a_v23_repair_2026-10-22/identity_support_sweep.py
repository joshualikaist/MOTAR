"""v2.2 (94acd1a) vs v2.3 identity support on correct tracks: targets partly hidden behind a bar in front (slivers),
targets beside or behind bars, elevation and yaw offsets. Truth = true paste pixels at track range inside the envelope
(from a render without the target). Usage: python -B <this file> <scratch dir> <out.json>. CPU only; writes the
extracted v2.2 package under <scratch dir> and the counts to <out.json>."""
import sys, collections, subprocess, importlib, json
from pathlib import Path
import numpy as np
ROOT = Path("/home/fair/workspaces/aerial_gym_ws/src/MOTAR-mt-p1"); sys.path.insert(0, str(ROOT)); sys.path.insert(0, str(ROOT / "tests"))
scratch = Path(sys.argv[1]); pkg = scratch / "q02v22"; pkg.mkdir(parents=True, exist_ok=True)
for n in ("__init__", "contracts", "geometry", "validation", "evidence"):
    (pkg / (n + ".py")).write_text(subprocess.check_output(["git", "-C", str(ROOT), "show", "94acd1a:motar_eval/q02/%s.py" % n], text=True))
sys.path.insert(0, str(scratch)); old = importlib.import_module("q02v22.evidence")
import test_q02_evidence as t
from motar_eval.q02 import evidence as new, geometry as geo
P = new.EvidenceParams()
def truth(inp, tv, bars):
    bg = np.asarray(t.make(target_v=tv, render_target=False, bars=bars).depth); d = np.asarray(inp.depth); paste = d != bg
    g = new._belief_geometry(inp, t.CAM, t.LID, P); pv, pu = np.nonzero(paste)
    if pv.size == 0: return 0, 0
    rays = geo.pixel_rays(pu, pv, t.CAM); pin, pen, pex, _ = geo.ray_envelope(np.asarray(t.CAM.camera_offset), rays, g["p_v"], g["env_cam"])
    pd = d[pv, pu]; at = pin & (pd >= pen - P.sigma_k * g["sigma_r"] - P.range_tol_m) & (pd <= pex + P.sigma_k * g["sigma_r"] + P.range_tol_m)
    return int(paste.sum()), int(at.sum())
cnt = collections.Counter(); lost = []
for R in (2.0, 4.0, 6.0, 8.0, 9.5, 12.0):
    for kind, bars in (("none", ()),
                       ("front thin r0.05 centred", ((R * 0.6, 0.0, 0.05),)), ("front r0.1 off 0.1", ((R * 0.6, 0.1 * 0.6, 0.1),)),
                       ("front r0.2 half", ((R * 0.6, 0.2 * 0.6 + 0.0, 0.2),)), ("front r0.3 edge", ((R * 0.6, 0.45 * 0.6, 0.3),)),
                       ("beside same range gap0.1", ((R, 0.15 + 0.1 + 0.3, 0.3),)), ("behind", ((R + 1.0, 0.0, 0.3),))):
        for z in (0.0, 0.3, -0.3):
            for yaw in (0.0, 0.4):
                tv = (R, 0.05, z); inp = t.make(target_v=tv, detected=True, bars=bars, yaw=yaw)
                n_paste, n_at = truth(inp, tv, bars)
                a, b = old.estimate(inp), new.estimate(inp)
                cnt["cases"] += 1
                cnt["v22 SUPPORTED"] += a.identity_status == "SUPPORTED_BY_FOOTPRINT"
                cnt["v23 SUPPORTED"] += b.identity_status == "SUPPORTED_BY_FOOTPRINT"
                cnt["truth paste at range"] += n_at > 0
                if a.identity_status == "SUPPORTED_BY_FOOTPRINT" and b.identity_status != "SUPPORTED_BY_FOOTPRINT":
                    lost.append((R, kind, z, yaw, n_paste, n_at, b.hypothesis, b.identity_status, b.pasted_pixels_in_footprint))
                if b.identity_status == "SUPPORTED_BY_FOOTPRINT" and n_at == 0: cnt["v23 false SUPPORTED"] += 1
                if a.identity_status == "SUPPORTED_BY_FOOTPRINT" and n_at == 0: cnt["v22 false SUPPORTED"] += 1
print(dict(cnt)); print("lost SUPPORTED v2.2 -> v2.3:", len(lost))
Path(sys.argv[2]).write_text(json.dumps(dict(counts=dict(cnt), lost_supported=len(lost), lost=lost), indent=1, default=str) + "\n")
for l in lost[:30]: print("  ", l)
