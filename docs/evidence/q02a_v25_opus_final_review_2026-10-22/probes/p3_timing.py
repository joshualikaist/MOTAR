import sys; sys.path.insert(0, sys.argv[0].rsplit("/", 1)[0])
from common import *
for R, boxes in ((0.30, []), (0.35, []), (0.45, []), (0.6, []), (1.0, []), (0.45, [(0.9, 0.3, 0.3, 0.3, -1, 2)]), (0.6, [(0.45, -0.35, 0.3, 0.12, -1, 2)]), (3.0, []), (15.0, []), (19.0, [])):
    tv = np.array([R, 0.0, 0.0]) if R < 10 else np.array([R, 0.4, 0.2])
    depth, lidar, paste, native = A.scene(tv, boxes)
    inp = A.t.make(target_v=tuple(tv), detected=True, depth=depth, lidar=lidar.reshape(-1))
    rep = va.validate(inp, CAM, LID)
    ts = []
    for _ in range(3):
        t0 = time.perf_counter(); out = geo.fit_target_sphere(inp.depth, rep.depth_pasted, CAM, CAM.target_radius); ts.append(time.perf_counter() - t0)
    te = []
    for _ in range(3):
        t0 = time.perf_counter(); r = ev.estimate(inp); te.append(time.perf_counter() - t0)
    c, mask, st, nin = out
    print("R %.2f bars %d paste %d cand %d status %s inl %d mask %d mask_true %d centre_err %s fit_ms %.0f est_ms %.0f ident %s od %s" % (
        R, len(boxes), paste.sum(), rep.depth_pasted.sum(), st, nin, mask.sum(), (mask & paste).sum(),
        None if c is None else "%.2e" % np.linalg.norm(c - tv), 1e3 * min(ts), 1e3 * min(te), r.identity_status, rep.status("obstacle_depth")))
