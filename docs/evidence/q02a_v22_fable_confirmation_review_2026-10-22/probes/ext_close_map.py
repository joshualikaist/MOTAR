"""Extended contact-range map: finer steps, larger lateral/vertical offsets, yaw, bars nearby.
Leak = obstacle_depth OK while an observed native cell deviates from the TRUE native render by > 1e-3."""
import sys, json, math, collections, time
import numpy as np
ROOT = "/home/fair/workspaces/aerial_gym_ws/src/MOTAR-mt-p1"
sys.path.insert(0, ROOT); sys.path.insert(0, ROOT + "/tests")
import test_q02_evidence as t
from motar_eval.q02 import geometry as geo, validation as va, evidence as ev
cam_o = np.array(t.CAM.camera_offset)
NR = geo.native_camera_rays(t.CAM, t.PROF.obstacle_native_wh)
def true_native(bars):
    return np.minimum(t._cylinder_hits(cam_o, NR, bars), t.PROF.obstacle_far_m)
def run(tv, bars=(), yaw=0.0):
    inp_f = t.make(target_v=tv, detected=True, bars=bars, yaw=yaw)
    rep = va.validate(inp_f, t.CAM, t.LID)
    od = rep.status("obstacle_depth"); ds = rep.status("depth_structure")
    leak = False; dev = float("nan"); nmin = float("nan")
    if rep.obstacle_depth_usable:
        tn = true_native(bars); seen = np.isfinite(rep.depth_native)
        dev = float(np.abs(rep.depth_native[seen] - tn[seen]).max()) if seen.any() else 0.0
        nmin = float(rep.depth_native[seen].min()) if seen.any() else float("nan")
        leak = dev > 1e-3
    rf = ev.estimate(inp_f); rn = ev.estimate(t.make(target_v=tv, bars=bars, yaw=yaw))
    detail = dict((c.name, c.detail) for c in rep.checks)["obstacle_depth"]
    return dict(r=round(tv[0], 4), lat=tv[1], z=tv[2], bars=str(bars), yaw=yaw, ds=ds, od=od, detail=detail[:28],
                contact=bool(rep.camera_contact_range), leak=leak, dev=dev, nmin=nmin,
                hyp_f=rf.hypothesis, id_f=rf.identity_status, warn_f=rf.near_obstacle_warning,
                hyp_n=rn.hypothesis, reasons_n=",".join(rn.reasons)[:60])
out = []; t0 = time.time()
# A. fine grid, no bars
for r_ in np.arange(0.25, 0.6001, 0.005):
    for lat in (0.0, 0.05, 0.1, 0.15, 0.2, 0.3):
        for z in (-0.1, -0.05, 0.0, 0.05, 0.1, 0.15):
            out.append(run((float(r_), lat, z)))
print("A grid done", len(out), "%.0fs" % (time.time() - t0)); sys.stdout.flush()
# B. bars nearby at contact-range targets
for r_ in (0.265, 0.28, 0.29, 0.30, 0.31, 0.32, 0.33, 0.35, 0.40, 0.50):
    for lat in (0.0, 0.1):
        for z in (0.0, 0.05):
            for bars in (((1.0, 0.0, 0.3),), ((0.6, 0.5, 0.3),), ((0.5, -0.4, 0.2),), ((2.0, 0.0, 0.4),), ((0.45, 0.0, 0.2),)):
                out.append(run((r_, lat, z), bars))
print("B bars done", len(out), "%.0fs" % (time.time() - t0)); sys.stdout.flush()
# C. yaw spot checks
for yaw in (math.pi / 2, -2.0, 0.7):
    for r_ in (0.28, 0.30, 0.32, 0.40):
        out.append(run((r_, 0.0, 0.0), (), yaw))
json.dump(out, open(sys.argv[1], "w"), indent=0, default=float)
leaks = [o for o in out if o["leak"]]
print("TOTAL", len(out), "LEAKS", len(leaks))
for o in leaks[:20]: print("  LEAK", o)
c = collections.Counter((o["ds"], o["od"], o["detail"].split()[0] if o["detail"] else "", o["contact"], o["hyp_f"], o["id_f"]) for o in out)
for k, v in sorted(c.items(), key=lambda kv: -kv[1]): print("%5d" % v, k)
print("contact frames:", sum(o["contact"] for o in out), " contact & warn_f:", sum(o["contact"] and o["warn_f"] for o in out))
print("flag lost (hyp_f != TARGET_VISIBLE):", collections.Counter((o["ds"], o["detail"].split()[0] if o["detail"] else "") for o in out if o["hyp_f"] != "TARGET_VISIBLE"))
print("od OK frames (grid A, no bars):", sum(1 for o in out if o["od"] == "OK" and o["bars"] == "()"), " min r with od OK:", min([o["r"] for o in out if o["od"] == "OK" and o["bars"] == "()"] or [None]))
print("od OK & nmin<10-1e-3 (phantom) :", sum(1 for o in out if o["od"] == "OK" and o["bars"] == "()" and o["nmin"] < 10 - 1e-3))
print("B: bars rows od statuses:", collections.Counter((o["od"], o["detail"].split()[0] if o["detail"] else "") for o in out if o["bars"] != "()"))
print("C: yaw rows:", [(o["yaw"], o["r"], o["od"], o["detail"][:14], o["hyp_f"]) for o in out if o["yaw"] != 0.0])
