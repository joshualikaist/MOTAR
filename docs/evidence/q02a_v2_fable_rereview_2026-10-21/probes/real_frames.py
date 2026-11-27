import sys, json, time
import numpy as np
ROOT="/home/fair/workspaces/aerial_gym_ws/src/MOTAR-mt-p1"
sys.path.insert(0, ROOT)
from motar_eval.q02 import contracts as c, evidence as ev, geometry as geo, validation as va
print("module files:", ev.__file__, geo.__file__)
root='/home/fair/workspaces/aerial_gym_ws/boia_shadow_runs/BOIA-SHADOW-Q01-V2/S1_ON/'
rows=[json.loads(l) for l in open(root+'sensors.jsonl')]
ck=lambda t: c.Clock(t["capture_sim_s"], t["delivery_sim_s"], t["provenance"], t["frame"], t["depth_convention"])
AV, AH = geo.bilinear_matrix(90,24), geo.bilinear_matrix(160,40)
for i in range(3):
    z=dict(np.load(root+'sensor_%03d.npz'%i, allow_pickle=False))
    d=z['depth'].reshape(90,160).astype(np.float64); rgb=z['rgb'].reshape(3,90,160)
    red = (rgb[0] > 0.5) & (rgb[1] < 0.3)   # target paint (pure red) -- renderer-side proxy, review only
    t0=time.perf_counter(); native, pasted, res = geo.recover_native_depth(d, (40,24)); dt=time.perf_counter()-t0
    fit = AV @ np.nan_to_num(native, nan=10.0) @ AH.T
    nonp = ~pasted
    print("frame",i,"recovery %.1f ms"%(dt*1e3),"residual %.2e"%res,"pasted px",pasted.sum(),"red px",red.sum(),
          "red&~pasted",(red&~pasted).sum(),"unobserved cells",np.isnan(native).sum(),
          "native==10",(native>=10-1e-3).sum(),"native<10 finite",(np.isfinite(native)&(native<10-1e-3)).sum(),
          "min native %.3f"%np.nanmin(native), "max |d-fit| non-pasted %.2e"%np.abs(d-fit)[nonp].max())
    # where are pasted pixels vs red pixels (bbox)
    vv,uu=np.nonzero(pasted); rv,ru=np.nonzero(red)
    print("   pasted bbox rows %d-%d cols %d-%d ; red bbox rows %d-%d cols %d-%d ; red depth range %.3f-%.3f"%(
        vv.min(),vv.max(),uu.min(),uu.max(), rv.min(),rv.max(),ru.min(),ru.max(), d[red].min(), d[red].max()))
    for label, flag in (("recorded", None), ("cleared", False)):
        v=dict(z)
        if flag is not None: v["camera"]=np.zeros_like(v["camera"])
        inp=c.EvidenceInput.from_arrays(v, ck(rows[i]["camera_time"]), ck(rows[i]["lidar_time"]), rows[i]["camera_time"]["decision_sim_s"], tuple(rows[i]["calibration"]))
        t0=time.perf_counter(); r=ev.estimate(inp); dt=time.perf_counter()-t0; r2=ev.estimate(inp)
        print("   %-9s %-19s %s id=%s det=%s range=%.2f sig_r=%.3f sig_lat=%.3f cam=%s lid=%s pasted_fp=%d near=%s problems=%s %.0fms same=%s"%(
            label, r.hypothesis, r.reasons, r.identity_status, r.camera_detected, r.predicted_range_m, r.sigma_range_m, r.sigma_lateral_m,
            (r.camera.rays,r.camera.foreground,r.camera.consistent,r.camera.clear,r.camera.censored,r.camera.near_clipped,r.camera.unobserved),
            (r.lidar.rays,r.lidar.foreground,r.lidar.consistent,r.lidar.clear,r.lidar.censored), r.pasted_pixels_in_footprint, r.near_obstacle_warning, r.validation_problems, dt*1e3, r==r2))
