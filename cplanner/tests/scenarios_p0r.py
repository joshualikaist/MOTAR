"""R2 rules 6 and 8: fixed research scenes with fixed expected classes per segment, on 72x1 and 72x4, each run with no
camera and with three camera variants (outputs must be identical). Plus the 72x4 open-scene normal-function family.
usage: scenarios_p0r.py OUT_JSON"""
import json
import math
import random
import sys

import numpy as np

from harness import bar, camera, classify, core, dump, header, noncat_box, pole, scan, target

O = (0.0, 0.0)
V, J, T, D, X = "VALID", "JUSTIFIED_REFUSAL", "TERMINAL_UNCERTIFIED", "DEADLOCK_OR_EVIDENCE_LIMIT", "OUT_OF_SCOPE"


def P(r, deg, o=O):
    return (o[0] + r * math.cos(math.radians(deg)), o[1] + r * math.sin(math.radians(deg)))


# (id, description, objects, [(path, {acceptable classes})], camera point or None, extra)
SCENES = [
    ("S01", "target only, open arena", [target(4.0, 0.0)],
     [((O, (0.0, 2.0)), {V}), ((O, (3.0, 0.0)), {V}), ((O, (3.7, 0.0)), {T}), ((O, (2.5, -1.5)), {V})],
     (4.0, 0.0, 1.0), {}),
    ("S02", "one catalog bar", [bar(2.0, 0.0, 0.3, 0.3)],
     [((O, (3.0, 0.0)), {J}), ((O, (0.0, 2.5)), {V}), ((O, (-2.0, -1.0)), {V}), ((O, (1.2, 0.0)), {V}),
      ((O, (1.6, 0.0)), {J})], None, {}),
    ("S03", "narrow 1.0 m corridor between two bars", [bar(2.0, 0.8, 0.3, 0.3), bar(2.0, -0.8, 0.3, 0.3)],
     [((O, (4.0, 0.0)), {V, D}), ((O, (4.0, 0.3)), {J}), ((O, (1.0, 0.0)), {V})], None, {}),
    ("S04", "bar BEHIND the target on the same bearing", [target(3.0, 0.0), bar(4.0, 0.0, 0.3, 0.3)],
     [((O, (2.0, 0.0)), {V}), ((O, (3.5, 0.0)), {J}), ((O, (2.6, 0.0)), {T})], (3.0, 0.0, 1.0), {}),
    ("S05", "bar IN FRONT of the target (camera sees the target behind it)",
     [bar(2.5, 0.0, 0.25, 0.25), target(3.4, 0.0)],
     [((O, (3.0, 0.0)), {J}), ((O, (1.6, 0.0)), {V}), ((O, (2.0, 0.0)), {J})], (3.4, 0.0, 1.0), {}),
    ("S06", "v7 F1 geometry: thin pole 0.555 m before the target on its bearing",
     [target(*P(2.40, 30)), pole(*P(1.845, 30), 0.015)],
     [((O, P(2.0, 30)), {J}), ((O, P(1.2, 30)), {V})], (*P(2.40 - 0.09, 30), 1.0), {}),
    ("S07", "non-target pole adjacent to the target", [target(3.0, 0.0), pole(3.0, 0.45, 0.03)],
     [((O, (2.2, 0.0)), {V}), ((O, (2.7, 0.0)), {T}), ((O, (3.0, 1.0)), {V}), ((O, (3.0, 0.6)), {J, X})],
     (3.0, 0.0, 1.0), {}),
    ("S08", "camera false positive on a bar", [bar(2.0, 1.0, 0.3, 0.3)],
     [((O, (2.0, 0.0)), {V, D}), ((O, (2.0, 0.5)), {J}), ((O, (0.0, -2.0)), {V})], (2.0, 1.0, 1.0), {}),
    ("S09", "target range error +-3 sigma (sigma 0.7 m)", [target(3.0, 0.0), bar(4.0, 0.0, 0.3, 0.3)],
     [((O, (2.0, 0.0)), {V}), ((O, (3.5, 0.0)), {J}), ((O, (2.5, 0.0)), {T})], (3.0 - 2.1, 0.0, 1.0), {}),
    ("S10", "camera/LiDAR frame mismatch (wrong extrinsic)", [target(3.0, 0.0), pole(1.9, 0.0, 0.03)],
     [((O, (2.5, 0.0)), {J}), ((O, (0.0, 2.0)), {V})], (3.0, 0.0, 1.0), {"bad_extrinsic": True}),
    ("S11", "partial occlusion: bar hidden behind a bar (UNKNOWN region)",
     [bar(2.0, 0.0, 0.3, 0.3), bar(4.0, 0.0, 0.3, 0.3)],
     [((((3.5, 0.6)), (4.5, 0.6)), {J}), (((3.0, 0.0), (3.2, 0.0)), {D, J}), ((O, (0.0, 2.0)), {V})], None, {}),
    ("S12", "thin non-catalog poles between beams (unobserved; declared limitation)",
     [pole(5.0, 0.21, 0.01), pole(2.5, -0.11, 0.01)],
     [((O, (5.5, 0.21)), {X, J}), ((O, (3.0, -0.11)), {X, J}), ((O, (0.0, 2.0)), {V})], None,
     {"declared_limitation": "non_catalog"}),
    ("S13", "observed non-catalog obstacle", [noncat_box(2.0, 0.0, 0.1, 0.1)],
     [((O, (3.0, 0.0)), {J}), ((O, (0.0, 2.0)), {V})], None, {}),
    ("S15", "target close (0.8 m)", [target(0.8, 0.0)],
     [((O, (0.3, 0.0)), {T}), ((O, (-1.5, 0.0)), {V}), ((O, (0.0, 1.5)), {V, T})], (0.8, 0.0, 1.0), {}),
    ("S16", "final 0.5 m approach (capture)", [target(2.0, 0.0)],
     [((O, (1.55, 0.0)), {T}), ((O, (1.4, 0.0)), {T}), ((O, (1.3, 0.0)), {V}), ((O, (1.0, 0.0)), {V})],
     (2.0, 0.0, 1.0), {}),
    ("S17", "30 % of beams not delivered", [bar(3.0, 2.0, 0.3, 0.3)],
     [((O, (0.0, 2.0)), {D}), ((O, (-2.0, 0.0)), {D})], None, {"drop": 0.3}),
    ("S18", "every candidate refused (boxed in)",
     [bar(0.75, 0.0, 0.3, 0.3), bar(-0.75, 0.0, 0.3, 0.3), bar(0.0, 0.75, 0.3, 0.3), bar(0.0, -0.75, 0.3, 0.3)],
     [((O, P(1.5, a)), {J}) for a in range(0, 360, 45)], None, {"expect_all_refused": True}),
]


def run_scene(objs, paths, layout, cam_point, extra, cam_variant):
    pose = (0.0, 0.0, 1.0, 0.0)
    cfg = "SYNTH-72x1" if layout == "P0R-72x1" else "SYNTH-72x4"
    r, s, who = scan(objs, pose, layout)
    if extra.get("drop"):
        rng = random.Random(7)
        for i in range(s.shape[0]):
            for j in range(s.shape[1]):
                if rng.random() < extra["drop"]:
                    s[i, j] = core.NOT_DELIVERED
    cam = None
    if cam_point is not None and cam_variant != "none":
        ext = (0.10, 0.0, 0.03, 0.0, 0.0, 0.0)
        if cam_variant == "truthful":
            cam = camera(0.1, pose, cam_point, ext=ext, sigma_r=0.05)
        elif cam_variant == "noisy_minus1sigma":
            cam = camera(0.1, pose, cam_point, ext=ext, sigma_r=0.7, sigma_b=0.01, range_err=-0.7)
        elif cam_variant == "wrong_extrinsic":
            cam = camera(0.1, pose, cam_point, ext=ext)
            cam[2:8] += np.array([0.3, 0.2, 0.0, 0.0, 0.0, 0.25])
        if extra.get("bad_extrinsic") and cam is not None:
            cam[7] += 0.4
    c = core.CertifierCore(cfg, 0, 0)
    rep = c.step(r, s, *header(1, pose), camera=cam)
    res = c.certify(np.array([p for p, _ in paths], float), 1)
    return rep, res, c


rows, fails = [], 0
for layout in ("P0R-72x1", "MOTAR-72x4"):
    for sid, what, objs, paths, cam_point, extra in SCENES:
        outs = {}
        for cv in ("none", "truthful", "noisy_minus1sigma", "wrong_extrinsic"):
            rep, res, c = run_scene(objs, paths, layout, cam_point, extra, cv)
            outs[cv] = (rep, res, list(c.exceptions))
        base_rep, base, exc = outs["none"]
        identical = all(json.dumps(o[1], sort_keys=True) == json.dumps(base, sort_keys=True) for o in outs.values())
        for (p, accept), r in zip(paths, base):
            cl = classify(r, p, objs)
            ok = cl in accept and identical and not exc
            fails += not ok
            rows.append({"layout": layout, "scene": sid, "what": what, "path": p, "accept": sorted(accept), "class": cl,
                         "reason": r["reason"], "certified": r["static_bar_certified"], "camera_identical": identical,
                         "labelled": outs["truthful"][0].get("labelled_returns"), "pass": ok})
            print("%-10s %-4s %-5s %-26s %-36s %s" % (layout, sid, "PASS" if ok else "FAIL", cl, r["reason"],
                                                       what[:60]), flush=True)
        if extra.get("expect_all_refused"):
            allref = all(not r["static_bar_certified"] for r in base)
            fails += not allref
            rows.append({"layout": layout, "scene": sid, "what": "all candidates refused -> fallback needed",
                         "pass": allref, "class": "ALL_REFUSED" if allref else "SOME_CERTIFIED"})

# moving target without obstacles: 12 steps, map trail and per-step approach classes
seq = []
for layout in ("P0R-72x1", "MOTAR-72x4"):
    cfg = "SYNTH-72x1" if layout == "P0R-72x1" else "SYNTH-72x4"
    c = core.CertifierCore(cfg, 0, 0)
    pose = (0.0, 0.0, 1.0, 0.0)
    for k in range(1, 13):
        tc = (3.0, -1.5 + 0.15 * k, 1.0)
        objs = [target(*tc)]
        r, s, _ = scan(objs, pose, layout)
        rep = c.step(r, s, *header(k, pose), camera=camera(k * 0.1, pose, tc))
        d = math.hypot(*tc[:2])
        u = (tc[0] / d, tc[1] / d)
        paths = [(O, (u[0] * (d - 1.0), u[1] * (d - 1.0))), (O, (u[0] * (d - 0.3), u[1] * (d - 0.3)))]
        res = c.certify(np.array(paths, float), k)
        cls = [classify(x, p, objs) for x, p in zip(res, paths)]
        trail = c.map_states(np.array([[3.0 - 0.2, -1.5 + 0.15]]), k * 0.1)[0]
        ok = cls == [V, T]
        fails += not ok
        seq.append({"layout": layout, "step": k, "classes": cls, "reasons": [x["reason"] for x in res],
                    "trail_cell_of_step1": trail, "labelled": rep["labelled_returns"], "pass": ok})
    print(layout, "moving target", [x["classes"] for x in seq[-12:]], [x["trail_cell_of_step1"] for x in seq[-12:]])

# normal function: 72x4 open scenes, random pose and yaw, short segments
rng = random.Random(1029)
open_rows = {"P0R-72x1": [0, 0], "MOTAR-72x4": [0, 0]}
for layout in open_rows:
    cfg = "SYNTH-72x1" if layout == "P0R-72x1" else "SYNTH-72x4"
    for i in range(200):
        pose = (rng.uniform(-3, 3), rng.uniform(-3, 3), 1.0, rng.uniform(-math.pi, math.pi))
        r, s, _ = scan([], pose, layout)
        c = core.CertifierCore(cfg, 0, 0)
        c.step(r, s, *header(1, pose))
        a, L = rng.uniform(-math.pi, math.pi), rng.uniform(0.3, 2.5)
        res = c.certify(np.array([((pose[0], pose[1]), (pose[0] + L * math.cos(a), pose[1] + L * math.sin(a)))]), 1)[0]
        open_rows[layout][0] += res["static_bar_certified"]
        open_rows[layout][1] += 1
open_ok = all(v[0] >= 0.9 * v[1] for v in open_rows.values())
fails += not open_ok
print("open-scene certification", open_rows)
summary = {"segments": len([r for r in rows if "path" in r]), "failures": fails,
           "classes": {k: sum(1 for r in rows if r.get("class") == k) for k in (V, J, T, D, X, "FALSE_CERT")},
           "open_scene_certified": open_rows}
print(json.dumps(summary))
dump(sys.argv[1], {"summary": summary, "pass": fails == 0, "rows": rows, "moving_target": seq})
