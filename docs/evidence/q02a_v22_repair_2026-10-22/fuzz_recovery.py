"""Property fuzz of Q02-A native depth recovery (v2.2 copy of ../q02a_v21_repair_2026-10-22/fuzz_recovery.py,
which labelled every run Q02A-v2.1). A random paste over a random obstacle upsample is
either fully excluded (every observed native cell equals the true obstacle depth within 1e-3 m) or
reported (status != OK, or pasted fraction above the obstacle_depth limit). Also the production-faithful
sphere sweep of the re-review (spheres crossing the top/bottom border at 0.5-2.0 m).

Usage: python -B docs/evidence/q02a_v22_repair_2026-10-22/fuzz_recovery.py <out.json> [seed:count ...]
CPU only; writes only <out.json>.
"""
import json, math, sys, time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT)); sys.path.insert(0, str(ROOT / "tests"))
import test_q02_evidence as t
from motar_eval.q02 import contracts as c, evidence as ev, geometry as geo, validation as va

VV, UU = np.mgrid[0:90, 0:160]
LIMIT = va.ValidationLimits().max_pasted_fraction


def random_case(rng):
    kind = rng.choice(["random", "empty", "smooth"])
    if kind == "random":
        native = rng.uniform(0.5, 10.0, (24, 40)); native[rng.uniform(size=native.shape) < 0.6] = 10.0
    elif kind == "empty":
        native = np.full((24, 40), 10.0)
    else:
        native = 3 + 0.1 * np.arange(40)[None, :] + 0.05 * np.arange(24)[:, None]
    shape = rng.choice(["disc", "rect", "ring", "two"])
    if shape == "disc":
        m = (VV - rng.uniform(-20, 110)) ** 2 + (UU - rng.uniform(-20, 180)) ** 2 < rng.uniform(2, 70) ** 2
    elif shape == "rect":
        r0, c0 = rng.integers(0, 90), rng.integers(0, 160)
        m = np.zeros((90, 160), bool); m[r0:r0 + rng.integers(1, 60), c0:c0 + rng.integers(1, 100)] = True
    elif shape == "ring":
        d2 = (VV - rng.uniform(0, 90)) ** 2 + (UU - rng.uniform(0, 160)) ** 2; r = rng.uniform(5, 50)
        m = (d2 < r * r) & (d2 > (r - rng.uniform(1, 6)) ** 2)
    else:
        m = np.zeros((90, 160), bool)
        for _ in range(2):
            m |= (VV - rng.uniform(0, 90)) ** 2 + (UU - rng.uniform(0, 160)) ** 2 < rng.uniform(2, 40) ** 2
    vals = rng.uniform(0.5, 9.5) if rng.uniform() < 0.7 else rng.uniform(0.5, 9.5) + 0.02 * UU
    depth = np.clip(t.AV @ native @ t.AH.T, 0.0, 10.0)
    depth[m] = vals[m] if np.ndim(vals) else vals
    return native, depth.astype(np.float32)


def fuzz(seed, count):
    rng = np.random.default_rng(seed); outcome, leaks, times = {}, 0, []
    for _ in range(count):
        native, depth = random_case(rng)
        t0 = time.perf_counter(); rec = geo.recover_native_depth(depth, (40, 24)); times.append(time.perf_counter() - t0)
        label = rec.status if rec.excluded.mean() <= LIMIT else "PASTED_DOMINATES"
        outcome[label] = outcome.get(label, 0) + 1
        seen = np.isfinite(rec.native)
        if label == geo.RECOVERY_OK and seen.any() and np.abs(rec.native[seen] - native[seen]).max() > 1e-3:
            leaks += 1
    return dict(seed=seed, cases=count, silent_leaks=leaks, outcomes=outcome,
                recovery_ms_median=round(1e3 * float(np.median(times)), 1), recovery_ms_max=round(1e3 * max(times), 1))


def sphere_sweep(seed=9, frames=400):
    rs = np.random.default_rng(seed); leaks = occluded = 0
    for _ in range(frames):
        rng_ = rs.uniform(0.5, 2.0); x_cam = rng_ - 0.1
        ang_r = math.degrees(math.asin(min(0.15 / max(x_cam, 0.16), 1.0)))
        edge = rs.choice([-1, 1]) * math.radians(rs.uniform(29 - ang_r - 2, 29 + ang_r - 1))
        tv = (rng_, rs.uniform(-0.3, 0.3) * x_cam, x_cam * math.tan(edge) + 0.03)
        mean = np.array(t.make(target_v=tv).track_mean); mean[0] += 1.0
        inp = t.make(target_v=tv, track_mean=mean)
        report = va.validate(inp, t.CAM, t.LID)
        paste = np.asarray(inp.depth) < 10.0 - 1e-6
        if report.obstacle_depth_usable and (paste & ~report.depth_excluded).any():
            leaks += 1
        occluded += ev.estimate(inp).hypothesis == ev.OCCLUDED
    return dict(seed=seed, frames=frames, usable_frames_with_paste_kept=leaks, occluded=int(occluded))


if __name__ == "__main__":
    runs = [tuple(int(v) for v in a.split(":")) for a in sys.argv[2:]] or [(7, 1000), (101, 3000)]
    out = dict(contract=c.EVIDENCE_CONTRACT_VERSION, blas_threads="set OMP/OPENBLAS/MKL_NUM_THREADS=1 for timing",
               fuzz=[fuzz(s, n) for s, n in runs], sphere_sweep=sphere_sweep())
    Path(sys.argv[1]).write_text(json.dumps(out, indent=1) + "\n")
    print(json.dumps(out, indent=1))
