"""Read-only diagnosis of the single M2 V1 violation (not evidence; outside the run directory)."""
import json, math, sys, pathlib
import numpy as np
FZ = pathlib.Path("/home/fair/motar_tmp/cplanner_m2_20261104/frozen")
for p in (FZ / "cplanner/tests", FZ / "cplanner/motar", FZ / "cplanner/src", FZ):
    sys.path.insert(0, str(p))
import v1_hitbit as V, replay as RP, registry as REG, adapter as AD
from motar_eval import occlusion_trace as OT
cell = sys.argv[1]; K = int(sys.argv[2]); B = int(sys.argv[3])
tr = OT.read_trace(cell); fr = tr["frames"]; x = fr[K]
look = OT.reset_lookup(tr["layouts"]); key = (int(x["env"]), int(x["episode"]))
c, h, q = (np.array(a, float) for a in OT.layout_arrays(tr["layouts"], look[key]))
p = np.array(x["p_pos"], float); tq = np.array(x["p_quat"], float); tp = np.array(x["t_pos"], float)
yaw, sq = RP.sensor_yaw(tq)
sc = RP.Scene(c, h, q)
raw, seg = sc.raw(p.astype(np.float32), sq, tp.astype(np.float32))
i, j = divmod(B, 72)
man = REG.directions("MOTAR-72x4").reshape(-1, 3)
cy, sy = math.cos(yaw), math.sin(yaw)
rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1.0]])
d_cert = (man @ rz.T)[B]
rv = RP.ray_vectors().reshape(-1, 3).astype(np.float64)[B]
d_kern = V.quat_rotate_xyzw(sq.astype(np.float64), rv); d_kern /= np.linalg.norm(d_kern)
t_c, w_c = V.oracle(p, d_cert[None], c, h, q, tp, RP.TARGET_RADIUS)
t_k, w_k = V.oracle(p, d_kern[None], c, h, q, tp, RP.TARGET_RADIUS)
b = int(w_c[0])
res = {"frame": int(x["frame"]), "env": key[0], "episode": key[1], "row": i, "col": j, "p_pos": p.tolist(),
       "yaw": yaw, "kernel_raw_m": float(raw[i, j]), "seg": int(seg[i, j]),
       "oracle_cert_dir": [float(t_c[0]), int(w_c[0])], "oracle_kernel_dir": [float(t_k[0]), int(w_k[0])]}
if b >= 0:
    cb, hb = c[b], h[b]
    res["bar"] = {"center": cb.tolist(), "half": hb.tolist(), "quat": q[b].tolist()}
    for name, d in (("cert", d_cert), ("kern", d_kern)):
        o = p - cb
        t1 = (-hb - o) / d; t2 = (hb - o) / d
        lo, hi = np.minimum(t1, t2), np.maximum(t1, t2)
        tmin, tmax = lo.max(), hi.min()
        entry = o + tmin * d; exit_ = o + tmax * d
        res["slab_" + name] = {"tmin": tmin, "tmax": tmax, "chord_m": tmax - tmin, "entry_axis": int(lo.argmax()),
                               "exit_axis": int(hi.argmin()), "entry_local": entry.tolist(), "exit_local": exit_.tolist(),
                               "entry_margin_to_faces": (hb - np.abs(entry)).tolist(),
                               "exit_margin_to_faces": (hb - np.abs(exit_)).tolist()}
# z of the ray at the oracle distance and the ground crossing
res["ray_z_at_oracle"] = float(p[2] + t_c[0] * d_cert[2]) if np.isfinite(t_c[0]) else None
res["t_ground_z0"] = float(-p[2] / d_cert[2]) if d_cert[2] < 0 else None
# perturb the kernel ray: does the Warp mesh hit nearby directions?
print(json.dumps(res, indent=1, default=float))
