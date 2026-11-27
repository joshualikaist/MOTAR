import sys, numpy as np
ROOT="/home/fair/workspaces/aerial_gym_ws/src/MOTAR-mt-p1"; sys.path.insert(0, ROOT); sys.path.insert(0, ROOT+"/tests")
import test_q02_evidence as t
from motar_eval.q02 import geometry as geo
import torch
rng = np.random.default_rng(11)
x = rng.uniform(0.5, 10.0, (24, 40)); x[rng.uniform(size=x.shape) < 0.7] = 10.0
y = torch.nn.functional.interpolate(torch.tensor(x)[None, None], size=(90, 160), mode="bilinear", align_corners=False)[0, 0].numpy()
vv, uu = np.mgrid[0:90, 0:160]
y2 = y.copy(); blob = (vv - 45) ** 2 + (uu - 90) ** 2 < 49; y2[blob] = 3.0 + 0.001 * (uu[blob] - 90) ** 2
rec2 = geo.recover_native_depth(y2.astype(np.float32), (40, 24), fill_value=10.0)
rec2b = geo.recover_native_depth(y2.astype(np.float32), (40, 24))
rows, cols = np.nonzero(blob.any(axis=1))[0], np.nonzero(blob.any(axis=0))[0]
box = np.zeros_like(blob); box[rows.min():rows.max()+1, cols.min():cols.max()+1] = True
for name, r in (("fill=10 (production path)", rec2), ("fill=None (test path)", rec2b)):
    print(name, "blob", int(blob.sum()), "foreign&blob", int((r.foreign & blob).sum()), "foreign&~blob", int((r.foreign & ~blob).sum()),
          "foreign outside box", int((r.foreign & ~box).sum()), "box", int(box.sum()))
