import sys, math
import numpy as np
ROOT="/home/fair/workspaces/aerial_gym_ws/src/MOTAR-mt-p1"
sys.path.insert(0, ROOT)
from motar_eval.q02 import geometry as geo
rng=np.random.default_rng(7)
# brute force: min distance of ray points (t>=0) to centre, and chord bounds by root-finding
bad=0
for k in range(20000):
    o=rng.normal(size=3)*0.5; cen=rng.normal(size=3)*6; R=rng.uniform(0.05,1.5)
    d=rng.normal(size=3); d/=np.linalg.norm(d)
    inside,entry,exit_,perp=geo.ray_envelope(o,d[None],cen,R)
    ts=np.linspace(0,40,400001)
    pts=o+ts[:,None]*d
    dist=np.linalg.norm(pts-cen,axis=1)
    within=dist<=R
    bf_inside=within.any() and (ts[within].max()>0)
    # exclude cases where the only intersection is behind the origin (s<=0) -- implementation requires s>0
    s=d@(cen-o)
    if abs(perp[0]-R)<2e-3 or abs(s)<2e-3: continue
    if bool(inside[0])!=bool(bf_inside and s>0):
        bad+=1; print("mismatch inside",o,cen,R,d,inside,bf_inside,s,perp)
        continue
    if inside[0]:
        t_in,t_out=ts[within].min(),ts[within].max()
        if abs(t_in-max(entry[0],0))>2e-4 or abs(t_out-exit_[0])>2e-4:
            bad+=1; print("chord mismatch",entry,exit_,t_in,t_out)
print("ray_envelope brute force mismatches:",bad)
# origins: camera vs lidar on an identical target
from motar_eval.q02 import contracts as c
CAM=c.FROZEN_F_CAMERA
d=np.array([[1.,0,0]])
for cen in ([8,0,0.03],[8,0.0,0.18],[8,0,0.0],[0.05,0,0.03],[-5,0,0]):
    print(cen,"cam",geo.ray_envelope(np.array(CAM.camera_offset),d,cen,0.15),"lid",geo.ray_envelope(np.zeros(3),d,cen,0.2))
