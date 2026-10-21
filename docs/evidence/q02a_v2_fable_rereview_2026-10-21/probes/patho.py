import sys, math, traceback, time
import numpy as np
ROOT="/home/fair/workspaces/aerial_gym_ws/src/MOTAR-mt-p1"
sys.path.insert(0, ROOT); sys.path.insert(0, ROOT+"/tests")
import test_q02_evidence as t
from motar_eval.q02 import contracts as c, evidence as ev, geometry as geo, validation as va
AV,AH=t.AV,t.AH
vv,uu=np.mgrid[0:90,0:160]
cases={}
cases["checkerboard 1px"]=np.where((vv+uu)%2==0,10.0,9.0)
cases["checkerboard 4px blocks"]=np.where(((vv//4)+(uu//4))%2==0,10.0,9.0)
cases["border ring of 0"]=np.pad(np.full((88,158),10.0),1,constant_values=0.0)
cases["sine pattern"]=5+4*np.sin(uu/7.0)*np.cos(vv/5.0)
cases["half image 3.0 (left)"]=np.where(uu<80,3.0,10.0)
cases["half image 3.0 (top)"]=np.where(vv<45,3.0,10.0)
cases["quarter image 3.0"]=np.where((uu<80)&(vv<45),3.0,10.0)
cases["all 5.0 constant"]=np.full((90,160),5.0)
cases["ramp 0..10 cols"]=uu/159*10
cases["staircase 4px"]=(uu//4).astype(float)/39*10
cases["two values random"]=np.where(np.random.default_rng(4).uniform(size=(90,160))<0.5,10.0,3.0)
y0=np.clip(AV@np.full((24,40),10.0)@AH.T,0,10)
y=y0.copy(); y[::8,:]=3.0; cases["every 8th row 3.0"]=y
y=y0.copy(); y[:, ::4]=3.0; cases["every 4th col 3.0"]=y
y=y0.copy(); y[45,80]=np.float32(9.9999); cases["one px 9.9999"]=y
y=y0.copy(); y[40:50,70:90]=10.0-1e-4; cases["rect 10-1e-4 (sub-tol)"]=y
y=y0.copy(); y[40:50,70:90]=10.0-2e-3; cases["rect 10-2e-3 (2x tol)"]=y
y=y0.copy(); y[40:50,70:90]=10.0-5e-3; cases["rect 10-5e-3"]=y
for name,d in cases.items():
    t0=time.perf_counter()
    try:
        inp=t.make(render_target=False, depth=d)
        rep=va.validate(inp,t.CAM,t.LID)
        r=ev.estimate(inp)
        nat=rep.depth_native
        print("%-28s %-14s res=%s pasted=%s unobs=%s nat[min,max]=%s -> %s %s  %.0fms"%(name,rep.status("depth_structure"),
              None if rep.depth_residual is None else "%.1e"%rep.depth_residual, None if rep.depth_pasted is None else int(rep.depth_pasted.sum()),
              None if nat is None else int(np.isnan(nat).sum()), None if nat is None else (round(float(np.nanmin(nat)),4),round(float(np.nanmax(nat)),4)),
              r.hypothesis, r.reasons, (time.perf_counter()-t0)*1e3))
    except Exception as e:
        print("%-28s EXCEPTION %r"%(name,e)); traceback.print_exc(limit=3)
