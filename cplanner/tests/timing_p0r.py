"""R2 rule 9 (reported only, no real-time claim): wall time of CertifierCore.step and certify for K = 1, 8, 32 on
72x1 and 72x4 random bar scenes, single process; and the deadline behaviour. usage: timing_p0r.py OUT_JSON"""
import json
import math
import random
import sys
import time

import numpy as np

from harness import CATALOG, bar, core, dump, header, scan, target


def q(xs):
    xs = sorted(xs)
    pick = lambda p: xs[min(len(xs) - 1, int(p * len(xs)))]  # noqa: E731
    return {"n": len(xs), "p50": pick(.5), "p95": pick(.95), "p99": pick(.99), "max": xs[-1],
            "over_100ms": sum(x > 100 for x in xs), "over_100ms_frac": sum(x > 100 for x in xs) / len(xs)}


out = {}
for layout, cfg in (("P0R-72x1", "SYNTH-72x1"), ("MOTAR-72x4", "SYNTH-72x4")):
    rng = random.Random(77)
    t_step, t_cert = [], {1: [], 8: [], 32: []}
    t_total8, deadline = [], {"requests": 0, "deadline_unknown": 0, "certified": 0, "ms": []}
    for i in range(150):
        pose = (0.0, 0.0, 1.0, rng.uniform(-math.pi, math.pi))
        objs = [bar(rng.uniform(-5, 5), rng.uniform(-5, 5), *rng.choice(CATALOG)) for _ in range(rng.randint(0, 8))]
        objs = [o for o in objs if max(abs(o["c"][0]) - o["h"][0], abs(o["c"][1]) - o["h"][1]) > 0.4]
        objs.append(target(rng.uniform(-4, 4), rng.uniform(-4, 4)))
        r, s, _ = scan(objs, pose, layout)
        c = core.CertifierCore(cfg, 0, 0)
        t0 = time.perf_counter()
        c.step(r, s, *header(1, pose))
        t_step.append(1e3 * (time.perf_counter() - t0))
        for K in (1, 8, 32):
            paths = []
            for _ in range(K):
                a, L = rng.uniform(-math.pi, math.pi), rng.uniform(0.3, 3.0)
                paths.append(((0.0, 0.0), (L * math.cos(a), L * math.sin(a))))
            t0 = time.perf_counter()
            res = c.certify(np.array(paths), 1)
            t_cert[K].append(1e3 * (time.perf_counter() - t0))
            if K == 8:
                t_total8.append(t_step[-1] + t_cert[8][-1])
            if K == 32:
                t0 = time.perf_counter()
                res = c.certify(np.array(paths), 1, deadline_s=0.05)
                deadline["ms"].append(1e3 * (time.perf_counter() - t0))
                deadline["requests"] += K
                deadline["deadline_unknown"] += sum(x["reason"] == "DEADLINE_UNKNOWN" for x in res)
                deadline["certified"] += sum(x["static_bar_certified"] for x in res)
    out[layout] = {"step_ms": q(t_step), "certify_ms": {str(k): q(v) for k, v in t_cert.items()},
                   "step_plus_certify8_ms": q(t_total8),
                   "certify32_with_50ms_deadline": dict(deadline, ms=q(deadline["ms"]))}
    print(layout, json.dumps(out[layout], indent=1), flush=True)
dump(sys.argv[1], {"timing": out, "note": "single process, CPU, python/numpy; reported only, no real-time claim"})
