"""Counterfactual (scratch only): rivals = any distinct sphere (> R) with an inlier set disjoint from the best's.
Runs the author's corner sweeps with the patched fit, to size the minimal tie fix. Usage: fam out.json"""
import sys, os, json, types, inspect
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from probe_common import *
src = inspect.getsource(geo.fit_target_sphere)
old = "rivals = [x[0] for x in distinct if not (x[1] & inl).any() and np.linalg.norm(x[2] - c) > 2.0 * radius]"
assert old in src
new = "rivals = [x[0] for x in distinct if not (x[3] & inl).any() and not (x[1] & inl & target_like_best).any()]"
src = src.replace(old, "target_like_best = np.ones(len(t), dtype=bool) if n_in >= min_inliers else np.zeros(len(t), dtype=bool)\n    " + new)
ns = dict(geo.__dict__); exec(src, ns); patched = ns["fit_target_sphere"]
geo.fit_target_sphere = patched; ev.fit_target_sphere = patched
fam, out = sys.argv[1], sys.argv[2]
o = IS.corner(J, Z2 if fam == "corner_z2" else Z3, out) if fam.startswith("corner") else IS.random_scenes(J, int(sys.argv[3]), int(sys.argv[4]))
open(out, "w").write(json.dumps(dict(family=fam, counts=dict(sorted(o.items())))) + "\n"); print(fam, {k: o[k] for k in ("scenes", "scenes_with_material_false", "FALSE_SUPPORT", "FALSE_MATERIAL", "correct_tracks_with_truth", "correct_supported", "INCORRECT_REJECTION", "TL_FALSE_MATERIAL", "TL_INCORRECT_REJECTION")})
