import sys, numpy as np
ROOT="/home/fair/workspaces/aerial_gym_ws/src/MOTAR-mt-p1"; sys.path.insert(0, ROOT)
from motar_eval.q02 import geometry as geo
AV,AH=geo.bilinear_matrix(90,24),geo.bilinear_matrix(160,40)
vv,uu=np.mgrid[0:90,0:160]; d2=(vv-45)**2+(uu-80)**2
def dil(m):
    out=m.copy()
    for dr in (-1,0,1):
        for dc in (-1,0,1):
            out|=np.roll(np.roll(m,dr,0),dc,1)
    return out
orig=geo._components
def comps8(mask):   # scratch-only monkeypatch: label the 1-px-dilated mask (8-connected closure), return original members
    labs=orig(dil(mask)); return labs
for name,m,val in (("r40 disc",d2<1600,2.0),("r60 disc (crosses top/bottom)",d2<3600,2.0),("r25 disc",d2<625,2.0)):
    x=np.full((24,40),10.0); y=np.clip(AV@x@AH.T,0,10); y[m]=val
    geo._components=orig; nat,p,res=geo.recover_native_depth(y,(40,24)); a=(m&~p).sum()
    geo._components=comps8; nat2,p2,res2=geo.recover_native_depth(y,(40,24)); b=(m&~p2).sum()
    print("%-32s kept pasted: 4-conn=%5d  dilated/8-conn=%5d   pasted frac %.2f -> %.2f"%(name,a,b,p.mean(),p2.mean()))
geo._components=orig
