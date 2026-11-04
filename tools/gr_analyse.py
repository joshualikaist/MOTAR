"""Analysis of GR-0 (docs/plans/prereg_gain_randomization_gr0_v1.md): Policy F under controller-gain randomization.

Reads gr0_run_report.json (verdict COMPLETE required) and each cell's quota ledger (episodes.jsonl, in_quota rows).
Blocks are the 12 (density, seed) pairs; levels S20 and S40 are compared with S00 (OFF) inside each block.
  delta_capture_pp, delta_crash_pp  mean over blocks of (level - S00) quota rates, in percentage points
  95 % CI                            percentile bootstrap over blocks (10,000 resamples, seed 20,261,110)
Reading per level (fixed in the preregistration):
  GAIN_SENSITIVE  (delta_capture <= -3 pp and CI upper < 0) or (delta_crash >= +2 pp and CI lower > 0)
  GAIN_ROBUST     CI(delta_capture) inside [-3, +3] pp and CI(delta_crash) inside [-2, +2] pp
  INCONCLUSIVE    otherwise
usage: gr_analyse.py RUN_DIR OUT_JSON
"""
import json
import pathlib
import random
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import kroc2027_eval as K                                  # noqa: E402

BOOT, BOOT_SEED = 10000, 20261110
CAP_PP, CRASH_PP = 3.0, 2.0


def rates(cell_dir):
    rows = [r for r in K.read_jsonl(cell_dir / "episodes.jsonl") if r.get("in_quota")]
    n = len(rows)
    return {"n": n, **{o: sum(1 for r in rows if r.get("outcome") == o) / n for o in ("capture", "crash", "timeout")}}


def ci(values, rng):
    means = sorted(sum(rng.choice(values) for _ in values) / len(values) for _ in range(BOOT))
    return means[int(0.025 * BOOT)], means[int(0.975 * BOOT) - 1]


def reading(dc, dc_ci, dx, dx_ci):
    if (dc <= -CAP_PP and dc_ci[1] < 0) or (dx >= CRASH_PP and dx_ci[0] > 0):
        return "GAIN_SENSITIVE"
    if -CAP_PP <= dc_ci[0] and dc_ci[1] <= CAP_PP and -CRASH_PP <= dx_ci[0] and dx_ci[1] <= CRASH_PP:
        return "GAIN_ROBUST"
    return "INCONCLUSIVE"


def main():
    run, out = pathlib.Path(sys.argv[1]), pathlib.Path(sys.argv[2])
    if out.exists():
        raise SystemExit("refusing to overwrite " + str(out))
    report = json.loads((run / "gr0_run_report.json").read_text())
    if report.get("verdict") != "COMPLETE":
        raise SystemExit("run is not COMPLETE: %s" % report.get("verdict"))
    cells = {}
    for rec in report["records"]:
        cells[(rec["level"], rec["density"], rec["seed"])] = rates(run / rec["level"] / rec["cell_id"])
    blocks = sorted({(d, s) for _, d, s in cells})
    levels = sorted({lv for lv, _, _ in cells})
    pooled = {lv: {o: 100.0 * sum(cells[(lv, d, s)][o] for d, s in blocks) / len(blocks)
                   for o in ("capture", "crash", "timeout")} for lv in levels}
    by_density = {lv: {str(d): {o: 100.0 * sum(cells[(lv, d, s)][o] for s in sorted({s for dd, s in blocks if dd == d}))
                                / len([1 for dd, _ in blocks if dd == d]) for o in ("capture", "crash", "timeout")}
                       for d in sorted({d for d, _ in blocks})} for lv in levels}
    rng = random.Random(BOOT_SEED)
    comparisons = {}
    for lv in levels:
        if lv == "S00":
            continue
        dcs = [100.0 * (cells[(lv, d, s)]["capture"] - cells[("S00", d, s)]["capture"]) for d, s in blocks]
        dxs = [100.0 * (cells[(lv, d, s)]["crash"] - cells[("S00", d, s)]["crash"]) for d, s in blocks]
        dc, dx = sum(dcs) / len(dcs), sum(dxs) / len(dxs)
        dc_ci, dx_ci = ci(dcs, rng), ci(dxs, rng)
        comparisons[lv] = {"delta_capture_pp": dc, "delta_capture_ci95_pp": dc_ci, "delta_crash_pp": dx,
                           "delta_crash_ci95_pp": dx_ci, "reading": reading(dc, dc_ci, dx, dx_ci),
                           "block_delta_capture_pp": dcs, "block_delta_crash_pp": dxs}
    res = {"analysis": "GR-0", "run_dir": str(run), "commit": report["commit"], "blocks": [list(b) for b in blocks],
           "episodes_per_cell": sorted({c["n"] for c in cells.values()}), "pooled_rate_pct": pooled,
           "by_density_pct": by_density, "comparisons": comparisons,
           "thresholds_pp": {"capture": CAP_PP, "crash": CRASH_PP}, "bootstrap": {"resamples": BOOT, "seed": BOOT_SEED}}
    with out.open("x") as fh:
        json.dump(res, fh, indent=1)
        fh.write("\n")
    print(json.dumps({lv: {k: c[k] for k in ("delta_capture_pp", "delta_capture_ci95_pp", "delta_crash_pp",
                                             "delta_crash_ci95_pp", "reading")} for lv, c in comparisons.items()},
                     indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
