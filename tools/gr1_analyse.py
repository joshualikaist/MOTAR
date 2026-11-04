"""Analysis of GR-1 (docs/plans/prereg_gain_randomization_gr1_v1.md): GR1-rand40 vs GR1-ctrl (and Policy F from GR-0).

Inputs: the GR-0 run folder (Policy F) and the two gr1eval run folders (verdict COMPLETE each). Quota-ledger rates per
cell; blocks are the 12 (density, seed) pairs; 95 % CIs are percentile bootstraps over blocks (10,000 resamples,
seed 20,261,111).
  P1 (robustness)    mean over blocks of crash(rand40, S40) - crash(ctrl, S40), in pp
  P2 (nominal cost)  mean over blocks of capture(rand40, S00) - capture(ctrl, S00), in pp
Reading (fixed in the preregistration, first match wins):
  GR1_MATERIAL_LOSS   P2 <= -2 pp and CI(P2) upper < 0
  GR1_SUPPORTED       P1 <= -1.5 pp and CI(P1) upper < 0 and CI(P2) lower > -2 pp
  GR1_NOT_SUPPORTED   CI(P1) lower > -1.5 pp
  INCONCLUSIVE        otherwise
Reported: pooled and per-density rates for every policy and level; rand40 - F and ctrl - F at every level; the S40 -
S00 crash gap per policy.
usage: gr1_analyse.py GR0_DIR RAND40_DIR CTRL_DIR OUT_JSON
"""
import json
import pathlib
import random
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import gr_analyse as A                                     # noqa: E402  (rates, ci)

BOOT_SEED = 20261111


def load(run_dir, report_name):
    rep = json.loads((run_dir / report_name).read_text())
    if rep.get("verdict") != "COMPLETE":
        raise SystemExit("%s is not COMPLETE: %s" % (run_dir, rep.get("verdict")))
    return {(r["level"], r["density"], r["seed"]): A.rates(run_dir / r["level"] / r["cell_id"]) for r in rep["records"]}


def main():
    gr0, rnd, ctl, out = (pathlib.Path(a) for a in sys.argv[1:5])
    if out.exists():
        raise SystemExit("refusing to overwrite " + str(out))
    pol = {"F": load(gr0, "gr0_run_report.json"), "rand40": load(rnd, "gr1eval_run_report.json"),
           "ctrl": load(ctl, "gr1eval_run_report.json")}
    blocks = sorted({(d, s) for _, d, s in pol["F"]})
    levels = ("S00", "S20", "S40")
    for name, cells in pol.items():
        missing = [(lv, d, s) for lv in levels for d, s in blocks if (lv, d, s) not in cells]
        if missing:
            raise SystemExit("%s lacks cells %s" % (name, missing[:3]))
    rng = random.Random(BOOT_SEED)

    def delta(a, la, b, lb, metric):
        xs = [100.0 * (pol[a][(la, d, s)][metric] - pol[b][(lb, d, s)][metric]) for d, s in blocks]
        return {"mean_pp": sum(xs) / len(xs), "ci95_pp": A.ci(xs, rng), "blocks_pp": xs}

    p1 = delta("rand40", "S40", "ctrl", "S40", "crash")
    p2 = delta("rand40", "S00", "ctrl", "S00", "capture")
    if p2["mean_pp"] <= -2.0 and p2["ci95_pp"][1] < 0:
        reading = "GR1_MATERIAL_LOSS"
    elif p1["mean_pp"] <= -1.5 and p1["ci95_pp"][1] < 0 and p2["ci95_pp"][0] > -2.0:
        reading = "GR1_SUPPORTED"
    elif p1["ci95_pp"][0] > -1.5:
        reading = "GR1_NOT_SUPPORTED"
    else:
        reading = "INCONCLUSIVE"
    pooled = {n: {lv: {m: 100.0 * sum(c[(lv, d, s)][m] for d, s in blocks) / len(blocks)
                        for m in ("capture", "crash", "timeout")} for lv in levels} for n, c in pol.items()}
    by_density = {n: {lv: {str(d): {m: 100.0 * sum(c[(lv, d, s)][m] for s in sorted({s for dd, s in blocks if dd == d}))
                                    / len([1 for dd, _ in blocks if dd == d]) for m in ("capture", "crash")}
                           for d in sorted({d for d, _ in blocks})} for lv in levels} for n, c in pol.items()}
    secondary = {}
    for lv in levels:
        for n in ("rand40", "ctrl"):
            secondary["%s_minus_F_%s" % (n, lv)] = {"capture": delta(n, lv, "F", lv, "capture"),
                                                    "crash": delta(n, lv, "F", lv, "crash")}
    gaps = {n: delta(n, "S40", n, "S00", "crash") for n in pol}
    res = {"analysis": "GR-1", "reading": reading, "P1_robustness_crash_S40_rand40_minus_ctrl": p1,
           "P2_nominal_capture_S00_rand40_minus_ctrl": p2, "pooled_rate_pct": pooled, "by_density_pct": by_density,
           "secondary": secondary, "crash_gap_S40_minus_S00": gaps, "blocks": [list(b) for b in blocks],
           "bootstrap": {"resamples": A.BOOT, "seed": BOOT_SEED}, "inputs": [str(gr0), str(rnd), str(ctl)]}
    with out.open("x") as fh:
        json.dump(res, fh, indent=1)
        fh.write("\n")
    print(json.dumps({"reading": reading, "P1": {k: p1[k] for k in ("mean_pp", "ci95_pp")},
                      "P2": {k: p2[k] for k in ("mean_pp", "ci95_pp")},
                      "pooled": {n: {lv: round(v["crash"], 2) for lv, v in d.items()} for n, d in pooled.items()}},
                     indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
