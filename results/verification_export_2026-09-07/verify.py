#!/usr/bin/env python3
"""Recompute the headline R-B numbers from governor_cells.csv alone -- counts in, intervals out.

Standard library only, and it imports nothing from tools/. If this script and the paper disagree,
the paper is wrong. Run: python verify.py
"""
import csv
import math
from collections import defaultdict
from pathlib import Path

HERE = Path(__file__).resolve().parent
Z = 1.959963984540054
rows = list(csv.DictReader(open(HERE / "governor_cells.csv", encoding="utf-8")))
print(f"rows: {len(rows)}")

# 0. every rate must be reproducible from its counts
bad = 0
for r in rows:
    n = int(r["actual_episodes"])
    for cnt in ("n_crash", "n_captured", "n_timeout"):
        if cnt in r and r[cnt] != "":
            pass
    total = sum(int(r[k]) for k in ("n_crash", "n_captured", "n_timeout"))
    if total != n:
        bad += 1
print(f"0. crash+captured+timeout == actual_episodes on every row: {'yes' if bad == 0 else f'NO ({bad} rows)'}")

# 1. the R-B replication: ep25000 lineage, three evaluation seeds, five densities, three narrow laws
EP25000 = "f70221393660"
seeds = (523, 527, 531)
dens = (70, 100, 130, 160, 205)
cell = {}
for r in rows:
    if not r["checkpoint_sha256"].startswith(EP25000):
        continue
    key = (int(r["cond_seed"]), int(r["cond_bars"]), r["cond_speed_governor_mode"],
           round(float(r["cond_speed_governor_half_width_m"]), 2))
    cell.setdefault(key, (int(r["n_crash"]), int(r["actual_episodes"])))   # first root wins; dupes are bit-identical


def wald(a, b):
    (ca, na), (cb, nb) = a, b
    pa, pb = ca / na, cb / nb
    d = 100 * (pa - pb)
    se = 100 * math.sqrt(pa * (1 - pa) / na + pb * (1 - pb) / nb)
    return d, se


def pool(est):
    w = [1 / se ** 2 for _, se in est]
    m = sum(wi * d for wi, (d, _) in zip(w, est)) / sum(w)
    se = math.sqrt(1 / sum(w))
    q = sum(wi * (d - m) ** 2 for wi, (d, _) in zip(w, est))
    df = len(est) - 1
    i2 = max(0.0, (q - df) / q) if q > 0 else 0.0
    return m, se, q, df, i2


print("\n1. three-seed pooling (fixed effect, inverse variance), crash pp")
lowest = 0
for name, arm_a, arm_b in (("arc - riskcap", "dwa_arc", "riskcap"),
                           ("arc - stopcap", "dwa_arc", "stopcap"),
                           ("stopcap - riskcap", "stopcap", "riskcap")):
    est = []
    for s in seeds:
        for b in dens:
            a, c = cell.get((s, b, arm_a, 0.45)), cell.get((s, b, arm_b, 0.45))
            if a and c:
                est.append(wald(a, c))
    m, se, q, df, i2 = pool(est)
    print(f"   {name:<18} n={len(est):>2}  {m:+.2f} [{m - Z * se:+.2f}, {m + Z * se:+.2f}]"
          f"   Q={q:.1f} (df {df})  I2={100 * i2:.0f}%")
for s in seeds:
    for b in dens:
        rates = {m: cell[(s, b, m, 0.45)][0] / cell[(s, b, m, 0.45)][1] for m in ("dwa_arc", "riskcap", "stopcap")
                 if (s, b, m, 0.45) in cell}
        if len(rates) == 3 and min(rates, key=rates.get) == "dwa_arc":
            lowest += 1
print(f"   arc has the lowest crash rate in {lowest} of 15 seed x density cells")

print("\n2. stopcap - riskcap per density (the P5 rejection)")
for b in dens:
    est = [wald(cell[(s, b, "stopcap", 0.45)], cell[(s, b, "riskcap", 0.45)]) for s in seeds]
    m, se, *_ = pool(est)
    signs = "".join("-" if d < 0 else "+" for d, _ in est)
    print(f"   {b:>3} bars  {m:+.2f} [{m - Z * se:+.2f}, {m + Z * se:+.2f}]   per-seed signs {signs}")

print("\n3. arc tube 0.45 -> 1.2 m, pooled over seeds")
for b in (70, 205):
    est = [wald(cell[(s, b, "dwa_arc", 1.2)], cell[(s, b, "dwa_arc", 0.45)]) for s in seeds
           if (s, b, "dwa_arc", 1.2) in cell]
    m, se, *_ = pool(est)
    print(f"   {b:>3} bars  crash {m:+.2f} [{m - Z * se:+.2f}, {m + Z * se:+.2f}]  (n={len(est)} seeds)")

print("\n4. cross-tree check: the same seed-523 cell from two source trees")
pair = [(r["root"], r["n_crash"], r["n_captured"], r["n_timeout"], r["runtime_git_commit"][:8])
        for r in rows if r["cell"] == "ep25000_d205_dwa_arc" and int(r["cond_seed"]) == 523
        and r["root"] in ("D1p_ep25000_s523", "RB_crosstree_s523")]
for p in pair:
    print("   ", p)
print("   bit-identical:", len(pair) == 2 and pair[0][1:4] == pair[1][1:4])
