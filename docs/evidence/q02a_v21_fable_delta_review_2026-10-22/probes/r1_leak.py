"""R1 adversarial leak hunt against v2.1 recover_native_depth / validate (read-only)."""
import sys, math, json, itertools
import numpy as np
ROOT = "/home/fair/workspaces/aerial_gym_ws/src/MOTAR-mt-p1"
sys.path.insert(0, ROOT); sys.path.insert(0, ROOT + "/tests")
import test_q02_evidence as t
from motar_eval.q02 import geometry as geo, validation as va, evidence as ev
AV, AH = t.AV, t.AH
VV, UU = np.mgrid[0:90, 0:160]
LIMIT = va.ValidationLimits().max_pasted_fraction

def up(native):
    return np.clip(AV @ native @ AH.T, 0.0, 10.0)

def natives(rng):
    bars = np.full((24, 40), 10.0); bars[:, 5:7] = 4.0; bars[:, 18:21] = 7.5; bars[8:14, 30:33] = 2.2
    rnd = rng.uniform(0.5, 10.0, (24, 40)); rnd[rng.uniform(size=rnd.shape) < 0.6] = 10.0
    smooth = 3 + 0.1 * np.arange(40)[None, :] + 0.05 * np.arange(24)[:, None]
    return dict(empty=np.full((24, 40), 10.0), bars=bars, random=rnd, smooth=smooth)

def shapes(rng):
    out = {}
    out["disc_r40"] = (VV - 45) ** 2 + (UU - 80) ** 2 < 1600
    out["disc_r25_off"] = (VV - 30) ** 2 + (UU - 50) ** 2 < 625
    out["ring_r40_w3"] = ((VV - 45) ** 2 + (UU - 80) ** 2 < 1600) & ((VV - 45) ** 2 + (UU - 80) ** 2 > 37 ** 2)
    out["ring_r40_w1"] = ((VV - 45) ** 2 + (UU - 80) ** 2 < 1600) & ((VV - 45) ** 2 + (UU - 80) ** 2 > 39 ** 2)
    out["thin_hline_1px"] = (VV == 45) & (UU > 30) & (UU < 130)
    out["thin_vline_1px"] = (UU == 80) & (VV > 10) & (VV < 80)
    out["single_px"] = (VV == 45) & (UU == 80)
    out["single_px_corner"] = (VV == 0) & (UU == 0)
    out["single_px_edge_pure_row"] = (VV == 1) & (UU == 80)
    out["2x2_px"] = (VV >= 44) & (VV <= 45) & (UU >= 80) & (UU <= 81)
    out["rect_seg_aligned"] = (VV >= 32) & (VV < 47) & (UU >= 61) & (UU < 85)   # from the repo test
    out["rect_seg_aligned2"] = (VV >= 2) & (VV < 6) & (UU >= 2) & (UU < 6)         # first non-pure segments
    out["rect_pure_rows"] = (VV <= 1) & (UU >= 40) & (UU < 120)                      # the two pure top rows only
    out["rect_pure_cols"] = (UU <= 1) & (VV >= 20) & (VV < 70)
    out["rect_pure_rows_bottom"] = (VV >= 88) & (UU >= 40) & (UU < 120)
    out["corner_tl"] = (VV < 30) & (UU < 50)
    out["corner_br"] = (VV >= 60) & (UU >= 110)
    out["top_border_blob"] = (VV - 0) ** 2 / 4 + (UU - 80) ** 2 / 9 < 100
    out["left_border_blob"] = (VV - 45) ** 2 / 9 + (UU) ** 2 / 4 < 100
    out["three_borders"] = (UU < 100)                     # touches top, bottom, left
    out["band_rows"] = (VV >= 30) & (VV < 60)              # spans left/right
    out["diag_line"] = np.abs(VV - UU * 90 / 160) < 1.0    # thin diagonal across the image
    out["multi_blob"] = ((VV - 20) ** 2 + (UU - 30) ** 2 < 100) | ((VV - 70) ** 2 + (UU - 130) ** 2 < 150) | ((VV - 45) ** 2 + (UU - 80) ** 2 < 50)
    out["two_blobs_close"] = ((VV - 45) ** 2 + (UU - 70) ** 2 < 36) | ((VV - 45) ** 2 + (UU - 90) ** 2 < 36)
    out["hole_in_full_paste"] = ~((VV - 45) ** 2 + (UU - 80) ** 2 < 100)   # paste everywhere except a hole
    out["hole_small"] = ~((VV - 45) ** 2 + (UU - 80) ** 2 < 4)
    out["checker_4px"] = ((VV // 4 + UU // 4) % 2) == 0
    out["ellipse_wide"] = (VV - 45) ** 2 / 100 + (UU - 80) ** 2 / 2500 < 1
    out["half_disc_top"] = ((VV - 0) ** 2 + (UU - 80) ** 2 < 2500)
    out["L_shape"] = ((VV >= 20) & (VV < 70) & (UU >= 40) & (UU < 50)) | ((VV >= 60) & (VV < 70) & (UU >= 40) & (UU < 120))
    out["U_touch_top"] = ((VV < 60) & (UU >= 40) & (UU < 50)) | ((VV < 60) & (UU >= 110) & (UU < 120)) | ((VV >= 50) & (VV < 60) & (UU >= 40) & (UU < 120))
    return out

def value_fields(rng, m, base):
    yield "const2", np.full(m.shape, 2.0)
    yield "const9.5", np.full(m.shape, 9.5)
    yield "ramp_u", 1.0 + 0.03 * UU
    yield "ramp_v", 1.0 + 0.05 * VV
    yield "curved", 3.0 + 0.001 * (UU - 80) ** 2 + 0.002 * (VV - 45) ** 2
    yield "sphere_like", 3.0 - 0.15 * np.sqrt(np.clip(1 - ((UU - 80) / 60) ** 2 - ((VV - 45) / 40) ** 2, 0, None))
    yield "base_plus_1e-3", base + 1e-3
    yield "base_plus_3e-3", base + 3e-3
    yield "base_plus_1e-2", base + 1e-2
    yield "base_minus_5e-3", base - 5e-3
    yield "over10_12", np.full(m.shape, 12.0)
    yield "over10_10.5", np.full(m.shape, 10.5)
    yield "other_upsample", up(rng.uniform(0.5, 10.0, (24, 40)))     # a different legitimate render
    yield "base_shift_cells", up(np.roll(base_native_holder[0], 1, axis=1))

base_native_holder = [None]

def check(native, depth, tag, results, f32=True):
    d = depth.astype(np.float32) if f32 else depth
    rec = geo.recover_native_depth(d, (40, 24))
    seen = np.isfinite(rec.native)
    err = float(np.abs(rec.native[seen] - native[seen]).max()) if seen.any() else 0.0
    usable = rec.status == geo.RECOVERY_OK and rec.excluded.mean() <= LIMIT
    leak = usable and err > 1e-3
    # validate() invariant: depth_native None unless obstacle_depth OK
    inp = t.make(render_target=False, depth=np.clip(d, 0, None))
    rep = va.validate(inp, t.CAM, t.LID)
    inv_ok = (rep.depth_native is None) == (rep.status("obstacle_depth") != va.OK)
    results.append(dict(tag=tag, status=rec.status, excluded=round(float(rec.excluded.mean()), 3),
                        foreign=int(rec.foreign.sum()), residual=rec.residual, err=err, usable=bool(usable), leak=bool(leak),
                        ds=rep.status("depth_structure"), od=rep.status("obstacle_depth"), invariant=bool(inv_ok)))
    return leak

def main():
    rng = np.random.default_rng(99)
    results, leaks = [], []
    nat = natives(rng)
    for nname, native in nat.items():
        base_native_holder[0] = native
        base = up(native)
        for sname, m in shapes(rng).items():
            for vname, field in value_fields(rng, m, base):
                depth = base.copy(); depth[m] = field[m]
                tag = "%s|%s|%s" % (nname, sname, vname)
                if check(native, depth, tag, results):
                    leaks.append(tag)
    # delta sweep: constant offset paste on a disc, find max native error with usable status
    sweep = []
    for nname in ("empty", "bars"):
        native = nat[nname]; base = up(native)
        for delta in (5e-4, 1e-3, 1.2e-3, 1.5e-3, 1.8e-3, 2e-3, 2.5e-3, 3e-3, 4e-3, 6e-3, 1e-2):
            for sname in ("disc_r40", "rect_seg_aligned", "corner_tl", "disc_r25_off"):
                m = shapes(rng)[sname]
                depth = base.copy(); depth[m] = base[m] + delta
                rec = geo.recover_native_depth(depth.astype(np.float32), (40, 24))
                seen = np.isfinite(rec.native)
                err = float(np.abs(rec.native[seen] - native[seen]).max()) if seen.any() else 0.0
                usable = rec.status == geo.RECOVERY_OK and rec.excluded.mean() <= LIMIT
                sweep.append(dict(native=nname, shape=sname, delta=delta, status=rec.status, excluded=round(float(rec.excluded.mean()), 3),
                                  err=err, usable=bool(usable), leak=bool(usable and err > 1e-3)))
    # smooth low-curvature bump: amplitude A over half-width L px
    bumps = []
    for nname in ("empty", "bars"):
        native = nat[nname]; base = up(native)
        for A in (0.01, 0.05, 0.1, 0.2, 0.5, 1.0):
            for L in (20, 40, 60):
                g = np.clip(1 - ((UU - 80) / L) ** 2 - ((VV - 45) / (L * 0.6)) ** 2, 0, None) ** 2
                depth = base - A * g
                rec = geo.recover_native_depth(depth.astype(np.float32), (40, 24))
                seen = np.isfinite(rec.native)
                err = float(np.abs(rec.native[seen] - native[seen]).max()) if seen.any() else 0.0
                usable = rec.status == geo.RECOVERY_OK and rec.excluded.mean() <= LIMIT
                bumps.append(dict(native=nname, A=A, L=L, status=rec.status, excluded=round(float(rec.excluded.mean()), 3),
                                  foreign=int(rec.foreign.sum()), err=err, usable=bool(usable), leak=bool(usable and err > 1e-3)))
    n_inv = sum(1 for r in results if not r["invariant"])
    print("shape-family cases:", len(results), "leaks:", len(leaks), "invariant violations:", n_inv)
    for r in results:
        if r["leak"] or not r["invariant"]:
            print("  LEAK/INV", r)
    from collections import Counter
    print("status counts:", Counter(r["status"] for r in results))
    print("usable with err>1e-4 (near-miss):", [(r["tag"], r["err"]) for r in results if r["usable"] and r["err"] > 1e-4][:30])
    print("\n--- delta sweep (constant offset paste) ---")
    for r in sweep:
        print("  %-6s %-18s delta=%.1e %-28s excl=%.3f err=%.2e usable=%s leak=%s" % (r["native"], r["shape"], r["delta"], r["status"], r["excluded"], r["err"], r["usable"], r["leak"]))
    print("\n--- smooth bumps ---")
    for r in bumps:
        print("  %-6s A=%.2f L=%2d %-28s excl=%.3f foreign=%5d err=%.3e usable=%s leak=%s" % (r["native"], r["A"], r["L"], r["status"], r["excluded"], r["foreign"], r["err"], r["usable"], r["leak"]))
    json.dump(dict(results=results, sweep=sweep, bumps=bumps), open(sys.argv[1], "w"), indent=1, default=float)

main()
