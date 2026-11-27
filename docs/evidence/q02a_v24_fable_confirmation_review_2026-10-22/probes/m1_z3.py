"""F1 with the pinned 3 m pool (bar top +2 m above a 1 m-altitude camera) and with a 2-bar L (vertical edge of bar A in front,
top edge of bar B in front at another range)."""
import sys, json, collections, numpy as np, math
sys.path.insert(0, "/tmp/claude-1000/-home-fair-workspaces-aerial-gym-ws-src/35a63340-4061-4cc2-958f-43729b9e3417/scratchpad/fable_confirm_v24")
import attack_v24 as M
A, CAM_O, Z3 = M.A, M.CAM_O, M.Z3
cnt = collections.Counter(); fs = []
for R in (5.0, 6.0, 8.0, 10.0):
    for d_f in (1.0, 1.5, 2.5, 3.5):
        for a in (-0.5, -0.2, 0.0, 0.3):
            for b in (-0.5, -0.2, 0.0, 0.3):
                geom = M.front_bar_corner(R, d_f, a, b, pool=Z3)
                if geom is None: cnt["infeasible"] += 1; continue
                tv, boxes, dr = geom
                for delta in (-0.2, 0.0, 0.2):
                    res = M.run(target_v=tv, boxes=boxes, track_v=M.track_at_face(tv, dr, R - d_f, delta), cov_pos=0.01)
                    if res["paste"] < 2: cnt["no_flag"] += 1; continue
                    cnt["flagged"] += 1
                    if res["ident"] == "SUPPORTED_BY_FOOTPRINT" and res["true_at"] == 0:
                        cnt["FALSE"] += 1; fs.append((R, d_f, a, b, delta, res["pasted_at"], res["true_far"], round(res["err_range"], 2)))
print("Z3 pool (top +2 m):", dict(cnt)); print("  false examples:", fs[:10])
# two-bar L: bar A (3 m) vertical edge at y=0 covers image-right; bar B (2 m) top edge covers below z=+1; both in front
cnt2 = collections.Counter(); fs2 = []
for R in (3.0, 4.0, 6.0, 8.0):
    for dA, dB in ((1.0, 1.5), (1.5, 1.0), (1.5, 2.5), (2.5, 1.5)):
        xA, xB = R - dA, R - dB
        edge = math.atan2(1.0 - CAM_O[2], xB - CAM_O[0])
        if edge > math.radians(26): continue
        th = 0.15 / R
        for a in (-0.4, 0.0, 0.4):
            for b in (-0.4, 0.0, 0.4):
                ang_z = edge + b * th; ang_y = a * th
                dr = M.unit([1.0, math.tan(ang_y), math.tan(ang_z)]); tv = CAM_O + R * dr
                boxes = [(xA + 0.3, -0.3, 0.3, 0.3, Z3[0], Z3[1]), (xB + 0.4, 0.0, 0.4, 0.4, -1.0, 1.0)]
                # bar B must not hide the whole target: it spans y in [-0.4,0.4] at xB; fine
                for face in (xA, xB):
                    for delta in (-0.2, 0.0, 0.2):
                        res = M.run(target_v=tv, boxes=boxes, track_v=M.track_at_face(tv, dr, face, delta), cov_pos=0.01)
                        if res["paste"] < 2: cnt2["no_flag"] += 1; continue
                        cnt2["flagged"] += 1
                        if res["ident"] == "SUPPORTED_BY_FOOTPRINT" and res["true_at"] == 0:
                            cnt2["FALSE"] += 1; fs2.append((R, dA, dB, a, b, "A" if face == xA else "B", delta, res["pasted_at"], res["true_far"], round(res["err_range"], 2)))
print("two-bar L:", dict(cnt2)); print("  false examples:", fs2[:10])
json.dump(dict(z3=dict(cnt), z3_false=fs, L=dict(cnt2), L_false=fs2), open(sys.argv[1], "w"), default=float)
