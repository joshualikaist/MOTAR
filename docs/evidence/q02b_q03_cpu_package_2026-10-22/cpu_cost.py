"""Usage: python -B docs/evidence/q02b_q03_cpu_package_2026-10-22/cpu_cost.py <repo_root>
CPU latency and peak memory of the Q02-B/Q03 shadow pipeline (host-measured, 1 BLAS thread)."""
import json, resource, sys, time, tracemalloc
from pathlib import Path
ROOT = Path(sys.argv[1]); sys.path[:0] = [str(ROOT), str(ROOT / "tests")]
import numpy as np
import test_q02b_timing as qb, test_q03_local_map as t3
from motar_eval.q02 import obstacle_map as om, pipeline as pl, reacq_planner as rp, evidence as ev, validation as va
from motar_eval.q02.benchmark_metrics import _quantile as q

def stats(xs): return dict(n=len(xs), median=q(xs, .5), p95=q(xs, .95), max=max(xs))
out = {}
# 1. full decision (pipeline.observe) on a loss script: 4 seen + 20 occluded + 4 seen, H3
script = [qb.SEEN] * 4 + [qb.OCC] * 20 + [qb.SEEN] * 4
frames = [qb.frame(k, **kw) for k, kw in enumerate(script)]
lat = []
for rep in range(5):
    rec = pl.ShadowRecorder(policy="H3") if "policy" in pl.ShadowRecorder.__init__.__code__.co_varnames else pl.ShadowRecorder()
    rec.reset(rep + 1)
    for k, f in enumerate(frames):
        t0 = time.perf_counter(); rec.observe(rep + 1, k, f, qb.ORACLE_PROXY); lat.append(time.perf_counter() - t0)
out["observe_s"] = stats(lat)
# 2. components on a single-bar frame
inp, rep_ = t3.frame_at(((4.0, 0.0, 0.3),))
def tm(fn, n=30):
    xs = []
    for _ in range(n):
        t0 = time.perf_counter(); fn(); xs.append(time.perf_counter() - t0)
    return stats(xs)
out["validate_s"] = tm(lambda: va.validate(inp, t3.q2a.CAM, t3.q2a.LID))
out["map_integrate_s"] = tm(lambda: om.LocalObstacleMap().integrate(inp, rep_))
m = om.LocalObstacleMap(); m.integrate(inp, rep_)
import test_q03_planner as tp
out["plan_s"] = tm(lambda: rp.plan(m, (0.0, 0.0, 1.5), 0.0, (0.0, 0.0, 0.0), tp.belief(), 0.0))
# 3. peak Python heap (tracemalloc) over one full loss script
tracemalloc.start()
rec = pl.ShadowRecorder(); rec.reset(99)
for k, f in enumerate(frames):
    rec.observe(99, k, f, qb.ORACLE_PROXY)
cur, peak = tracemalloc.get_traced_memory(); tracemalloc.stop()
out["tracemalloc_peak_MB"] = round(peak / 2**20, 3)
out["map_arrays_MB"] = round(sum(a.nbytes for a in (m.free_s, m.occ_s, m.conf_s) if hasattr(m, "free_s")) / 2**20, 3) if hasattr(m, "free_s") else "NOT_RECORDED"
out["process_maxrss_MB"] = round(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024, 1)
out["host"] = dict(omp=__import__("os").environ.get("OMP_NUM_THREADS"), numpy=np.__version__, python=sys.version.split()[0])
print(json.dumps(out, indent=1))
