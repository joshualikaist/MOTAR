"""Analysis of GR-1 (docs/plans/prereg_gain_randomization_gr1_v1.md as amended by
docs/plans/prereg_gain_randomization_gr1_v1_amendment1_2026-11-04.md): GR1-rand40 vs GR1-ctrl, Policy F from GR-0.

Claim tested (amendment 1, M1): on the GR-0 grid (E2, A3, 70-205 bars x seeds 4501-4503, quota 16), this rand40
continuation of Policy F crashes at least 1.5 pp less than this ctrl continuation under +-40 % gains (S40). It does
so without losing capture under +-40 % gains, and loses at most 2 pp nominal capture (S00). The unit of inference is
this pair of training runs (one seed each).

Bindings, checked before any number (amendment 1, H1-H3):
  - Each eval folder's gr1eval_run_report.json names its train_report.json. The arm and checkpoint SHA must match
    that report, every cell receipt must carry that SHA, and the two arms must be rand40 and ctrl.
  - The two train reports are TRAINED on the same commit.
  - No GR-1 cell's episodes.jsonl may equal the GR-0 Policy F cell with the same (level, density, seed), and no
    rand40 cell may equal the matching ctrl cell. Any identical pair gives EVAL_BINDING_FAIL.
Quota rates; blocks are the 12 (density, seed) pairs. Percentile bootstrap over blocks (10,000 resamples, seed
20,261,111); t-intervals are reported as a sensitivity check (review L2).
  P1    mean over blocks of crash(rand40, S40) - crash(ctrl, S40), pp
  P2    mean over blocks of capture(rand40, S00) - capture(ctrl, S00), pp
  P3    mean over blocks of capture(rand40, S40) - capture(ctrl, S40), pp (amendment 1, M1b)
  DiD   (crash gap S40 - S00 of rand40) - (gap of ctrl), pp (amendment 1, M1a)
Reading, first match wins:
  EVAL_BINDING_FAIL   a binding check fails
  GR1_MATERIAL_LOSS   P2 <= -2 pp and CI(P2) upper < 0
  GR1_SUPPORTED       P1 <= -1.5 pp and CI(P1) upper < 0, CI(P2) lower > -2 pp, CI(P3) lower > 0
  GR1_NOT_SUPPORTED   CI(P1) lower > -1.5 pp
  INCONCLUSIVE        otherwise
Qualifiers, reported with the reading:
  - wording: "reduced gain sensitivity" only if CI(DiD) upper < 0; otherwise "lower crash under +-40 % gains".
  - drift (M3): if |ctrl - F crash at S40| >= |P1|, SUPPORTED is "not separable from continuation drift".
Reported, not gated: pooled and per-density rates; rand40 - F and ctrl - F per level; crash gaps; the crash-cause
table (bar contact and below per 100 episodes, summed over cells, result.json aggregate window; review M2).
usage: gr1_analyse.py GR0_DIR RAND40_EVAL_DIR CTRL_EVAL_DIR OUT_JSON
"""
import json
import math
import pathlib
import random
import statistics
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import gr_analyse as A                                     # noqa: E402  (rates, ci, bootstrap constants)
import kroc2027_eval as K                                  # noqa: E402

BOOT_SEED = 20261111
T_CRIT_11 = 2.200985                                       # two-sided 95 % Student t, 11 degrees of freedom


def load_eval(run_dir, report_name):
    rep = json.loads((run_dir / report_name).read_text())
    if rep.get("verdict") != "COMPLETE":
        raise SystemExit("%s is not COMPLETE: %s" % (run_dir, rep.get("verdict")))
    cells = {}
    for r in rep["records"]:
        d = run_dir / r["level"] / r["cell_id"]
        cells[(r["level"], r["density"], r["seed"])] = {"dir": d, **A.rates(d)}
    return rep, cells


def t_interval(xs):
    m = sum(xs) / len(xs)
    se = statistics.stdev(xs) / math.sqrt(len(xs))
    return [m - T_CRIT_11 * se, m + T_CRIT_11 * se]


def crash_causes(cells):
    out = {}
    for (lv, _d, _s), c in cells.items():
        res = json.loads((c["dir"] / "result.json").read_text())
        n = res.get("actual_episodes") or 0
        cc = res.get("crash_causes", {})
        o = out.setdefault(lv, {"episodes": 0, "bar_contact": 0, "below": 0, "above": 0, "out_of_bounds": 0})
        o["episodes"] += n
        for k in ("bar_contact", "below", "above", "out_of_bounds"):
            v = cc.get(k, 0)
            o[k] += int(v) if isinstance(v, (int, float)) else 0
    for o in out.values():
        o.update({k + "_per_100": 100.0 * o[k] / max(1, o["episodes"]) for k in ("bar_contact", "below")})
    return out


def main():
    gr0, rnd, ctl, out = (pathlib.Path(a) for a in sys.argv[1:5])
    if out.exists():
        raise SystemExit("refusing to overwrite " + str(out))
    _, F = load_eval(gr0, "gr0_run_report.json")
    rep_r, RND = load_eval(rnd, "gr1eval_run_report.json")
    rep_c, CTL = load_eval(ctl, "gr1eval_run_report.json")
    pol = {"F": F, "rand40": RND, "ctrl": CTL}
    blocks = sorted({(d, s) for _, d, s in F})
    levels = ("S00", "S20", "S40")
    binding, trains = {}, {}
    for name, rep in (("rand40", rep_r), ("ctrl", rep_c)):
        tr = json.loads(pathlib.Path(rep["train_report"]).read_text())
        trains[name] = tr
        binding[name + "_label_matches_train_report"] = rep.get("policy") == name == tr.get("arm")
        binding[name + "_sha_matches_train_report"] = (rep.get("checkpoint_sha256")
                                                       == tr["checkpoint"]["copy_sha256"] == tr["checkpoint"]["sha256"])
        binding[name + "_receipts_carry_sha"] = all(
            json.loads((c["dir"] / "receipt.json").read_text()).get("checkpoint_sha256") == rep.get("checkpoint_sha256")
            for c in pol[name].values())
        binding[name + "_trained"] = tr.get("verdict") == "TRAINED"
        missing = [(lv, d, s) for lv in levels for d, s in blocks if (lv, d, s) not in pol[name]]
        binding[name + "_complete_grid"] = not missing
    binding["same_training_commit"] = trains["rand40"].get("commit") == trains["ctrl"].get("commit")
    differ_f, differ_arms = True, True
    for lv in levels:
        for d, s in blocks:
            ef = K.read_jsonl(F[(lv, d, s)]["dir"] / "episodes.jsonl")
            er = K.read_jsonl(RND[(lv, d, s)]["dir"] / "episodes.jsonl")
            ec = K.read_jsonl(CTL[(lv, d, s)]["dir"] / "episodes.jsonl")
            differ_f &= (er != ef) and (ec != ef)
            differ_arms &= er != ec
    binding["gr1_cells_differ_from_F"] = differ_f
    binding["rand40_cells_differ_from_ctrl"] = differ_arms
    rng = random.Random(BOOT_SEED)

    def delta(a, la, b, lb, metric):
        xs = [100.0 * (pol[a][(la, d, s)][metric] - pol[b][(lb, d, s)][metric]) for d, s in blocks]
        return {"mean_pp": sum(xs) / len(xs), "ci95_pp": A.ci(xs, rng), "t95_pp": t_interval(xs), "blocks_pp": xs}

    p1 = delta("rand40", "S40", "ctrl", "S40", "crash")
    p2 = delta("rand40", "S00", "ctrl", "S00", "capture")
    p3 = delta("rand40", "S40", "ctrl", "S40", "capture")
    did_x = [100.0 * ((RND[("S40", d, s)]["crash"] - RND[("S00", d, s)]["crash"])
                      - (CTL[("S40", d, s)]["crash"] - CTL[("S00", d, s)]["crash"])) for d, s in blocks]
    did = {"mean_pp": sum(did_x) / len(did_x), "ci95_pp": A.ci(did_x, rng), "t95_pp": t_interval(did_x),
           "blocks_pp": did_x}
    if not all(binding.values()):
        reading = "EVAL_BINDING_FAIL"
    elif p2["mean_pp"] <= -2.0 and p2["ci95_pp"][1] < 0:
        reading = "GR1_MATERIAL_LOSS"
    elif p1["mean_pp"] <= -1.5 and p1["ci95_pp"][1] < 0 and p2["ci95_pp"][0] > -2.0 and p3["ci95_pp"][0] > 0:
        reading = "GR1_SUPPORTED"
    elif p1["ci95_pp"][0] > -1.5:
        reading = "GR1_NOT_SUPPORTED"
    else:
        reading = "INCONCLUSIVE"
    drift = delta("ctrl", "S40", "F", "S40", "crash")
    qualifiers = {
        "wording": ("reduced gain sensitivity" if did["ci95_pp"][1] < 0 else "lower crash under +-40 % gains")
        if reading == "GR1_SUPPORTED" else None,
        "drift_not_separable": bool(reading == "GR1_SUPPORTED" and abs(drift["mean_pp"]) >= abs(p1["mean_pp"])),
        "t_interval_agrees": {"P1": p1["t95_pp"][1] < 0, "P2": p2["t95_pp"][0] > -2.0, "P3": p3["t95_pp"][0] > 0},
    }
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
    res = {"analysis": "GR-1 (amendment 1)", "reading": reading, "qualifiers": qualifiers, "binding": binding,
           "P1_crash_S40_rand40_minus_ctrl": p1, "P2_capture_S00_rand40_minus_ctrl": p2,
           "P3_capture_S40_rand40_minus_ctrl": p3, "DiD_crash_gap_rand40_minus_ctrl": did,
           "drift_crash_S40_ctrl_minus_F": drift, "pooled_rate_pct": pooled, "by_density_pct": by_density,
           "secondary": secondary, "crash_gap_S40_minus_S00": gaps,
           "crash_causes_exploratory": {n: crash_causes(c) for n, c in pol.items()},
           "train_reports": {n: rep.get("train_report") for n, rep in (("rand40", rep_r), ("ctrl", rep_c))},
           "blocks": [list(b) for b in blocks], "bootstrap": {"resamples": A.BOOT, "seed": BOOT_SEED},
           "inputs": [str(gr0), str(rnd), str(ctl)]}
    with out.open("x") as fh:
        json.dump(res, fh, indent=1, default=str)
        fh.write("\n")
    print(json.dumps({"reading": reading, "qualifiers": qualifiers, "binding_ok": all(binding.values()),
                      "P1": {k: p1[k] for k in ("mean_pp", "ci95_pp")}, "P2": {k: p2[k] for k in ("mean_pp", "ci95_pp")},
                      "P3": {k: p3[k] for k in ("mean_pp", "ci95_pp")}, "DiD": {k: did[k] for k in ("mean_pp", "ci95_pp")}},
                     indent=1, default=str))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
