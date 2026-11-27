import sys; sys.path.insert(0, sys.argv[0].rsplit("/", 1)[0])
from common import *
cnt = collections.Counter(); ex = []
for R in (10.0, 11.0, 12.0, 14.0, 15.0, 17.0, 19.0):
    for x_f in (4.0, 6.0, 8.0, 9.5):
        for a in (-0.9, -0.5, 0.0, 0.5):
            for b in (-0.9, -0.5, 0.0, 0.5):
                gm = M.front_bar_corner(R, R - x_f, a, b)
                if gm is None: continue
                tv, boxes, dr = gm
                depth, lidar, paste, native = A.scene(tv, boxes)
                if paste.sum() < 2: cnt["noflag"] += 1; continue
                inp = A.t.make(target_v=tuple(tv), detected=True, depth=depth, lidar=lidar.reshape(-1))
                rep = va.validate(inp, CAM, LID)
                c, mask, st, nin = geo.fit_target_sphere(inp.depth, rep.depth_pasted, CAM, CAM.target_radius)
                cand = rep.depth_pasted
                cnt["flagged"] += 1
                cnt["cand_nontarget>0"] += int((cand & ~paste).sum() > 0)
                cnt["cand_true==0"] += int((cand & paste).sum() == 0)
                w = "none" if st != "OK" else ("true" if (mask & ~paste).sum() == 0 else ("mixed" if (mask & paste).sum() else "spurious"))
                cnt["win_" + w] += 1
                if w in ("spurious", "mixed") and len(ex) < 12:
                    dd = np.asarray(inp.depth, float)
                    ex.append(dict(R=R, x_f=x_f, a=a, b=b, paste=int(paste.sum()), cand=int(cand.sum()), cand_true=int((cand & paste).sum()), nin=nin,
                                   mask_depths=[round(float(x), 2) for x in dd[mask][:5]], true_depths=[round(float(x), 2) for x in dd[paste][:3]]))
print(dict(cnt)); [print(e) for e in ex]
