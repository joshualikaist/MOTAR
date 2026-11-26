"""Analysis of CAM-REF stage 2 (docs/plans/prereg_camonly_ref_stage2_2026-11-26.md): Policy F at 205 bars, F4 (F at P_T)
plus one P_T -> P_S factor each (F4D delay, F4R vehicle + gain randomisation, F4Y yaw clip, F4V FREE 2.5, F4A altitude /
mount), against stage 1's F4 and F8 cells (SHA-256 pinned; F4ID / F8ID reproduced their seed-4501 cells on this commit).

Counts, rates, the episode bootstrap (10,000 resamples, numpy default_rng(4611), cells in the fixed order CELL_ORDER x
seeds), the paired seed-block differences and their intervals are stage 1's (tools/camref_eval_analyse.py).
Readings:
  Q1  each factor: X - F4 for X in F4D, F4R, F4Y, F4V, F4A (pp, paired over the complete seed blocks). Labels on
      bar-contact crash, crash (all) and capture: MATERIAL (|difference| >= 5 pp and the bootstrap interval excludes 0),
      SMALL (interval excludes 0, |difference| < 5 pp), NO_DETECTED_CHANGE (otherwise). Target-contact and
      out-of-bounds crash: reported.
  Q2  the whole step F8 - F4 (same labels) and the interaction share: (F8 - F4) - sum of the five Q1 differences, per
      metric, with its bootstrap interval. Reported, not labelled.
  Use of the result (fixed): the factors whose Q1 bar-contact label is MATERIAL with a positive sign are named the
  carriers of F's bar-contact loss from P_T to P_S (largest first). None MATERIAL while Q2 bar-contact F8 - F4 is
  MATERIAL -> no single factor carries it (interaction or several small ones). This is about F (360 deg ranging,
  zero-shot); it is not a measurement of a camera-only policy and not a flight claim.
Analysis controls (run before the eval is read; a miss reads CONTROL_FAIL): synthetic cells with every factor equal to F4
-> all NO_DETECTED_CHANGE; F4Y bar crash +8 pp in every block -> F4Y MATERIAL, the other four NO_DETECTED_CHANGE.
usage: tools/camref_stage2_analyse.py controls OUT_JSON
       tools/camref_stage2_analyse.py eval IDENTITY_DIR EVAL_DIR OUT_JSON
"""
import json
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import camref_eval_analyse as A                              # noqa: E402  (counts, bootstrap, effect, load_cell)
import camref_stage2_run as S                                # noqa: E402  (arms, references, pins)

R = S.R
FACTOR_ARMS = S.NEW_ARMS                                     # F4D, F4R, F4Y, F4V, F4A
CELL_ORDER = ("F4",) + FACTOR_ARMS + ("F8",)
LABELLED = ("crash_bar", "crash", "capture")
SEEDS = R.SEEDS


def readings(cells, blocks=SEEDS):
    boot = A.bootstrap(cells, order=CELL_ORDER)
    q1 = {x: A.effect(cells, {x: 1.0, "F4": -1.0}, blocks, boot, labelled=LABELLED) for x in FACTOR_ARMS}
    q2 = {"F8-F4": A.effect(cells, {"F8": 1.0, "F4": -1.0}, blocks, boot, labelled=LABELLED)}
    w = {"F8": 1.0, "F4": -1.0 + len(FACTOR_ARMS)}               # (F8 - F4) - sum_x (x - F4)
    for x in FACTOR_ARMS:
        w[x] = -1.0
    q2["interaction"] = A.effect(cells, w, blocks, boot, labelled=())
    per_cell = {name: A.cell_summary(by, boot, name) for name, by in cells.items() if by}
    return {"Q1": q1, "Q2": q2, "cells": per_cell}


def use_of_result(r):
    carriers = []
    for x, e in r["Q1"].items():
        bar = e.get("crash_bar") if isinstance(e, dict) else None
        if isinstance(bar, dict) and bar["label"] == "MATERIAL" and bar["mean_pp"] > 0:
            carriers.append((bar["mean_pp"], x))
    carriers = [x for _, x in sorted(carriers, reverse=True)]
    step = r["Q2"]["F8-F4"]
    step_bar = step.get("crash_bar", {}).get("label") if isinstance(step, dict) else None
    if carriers:
        text = ["bar-contact loss of F from P_T to P_S carried by %s (largest first)" % ", ".join(carriers)]
        if "F4Y" in carriers:
            text.append("F4Y: the 60 deg/s cap is a measured contributor for a policy with 360 deg ranging; supports "
                        "testing a higher cap in the next training, not a camera-only gain")
        if "F4D" in carriers:
            text.append("F4D: latency reduction (sparse reprojection, 1-thread pinning) prioritised before training")
        return {"carriers": carriers, "text": "; ".join(text)}
    if step_bar == "MATERIAL":
        return {"carriers": [], "text": "no single factor carries F's bar-contact loss (interaction or several small ones)"}
    return {"carriers": [], "text": "no preregistered pointer applies (F8 - F4 bar contact %s)" % step_bar}


def synthetic(n=1024, f4=(0.24, 0.75, 0.09), bump=None):
    """{arm: {seed: counts}} with every arm equal to F4's (capture, crash, bar crash) and `bump` = (arm, +bar pp)."""
    cells = {}
    for a in CELL_ORDER:
        cap, crash, bar = f4
        if bump and a == bump[0]:
            crash, bar, cap = crash + bump[1], bar + bump[1], cap - bump[1]
        cells[a] = {s: A.synthetic_counts(n, cap, crash, bar_share=bar / crash) for s in SEEDS}
    return cells


def controls():
    res, ok = {}, True
    r = readings(synthetic())
    labs = {x: r["Q1"][x]["crash_bar"]["label"] for x in FACTOR_ARMS}
    res["identical"] = {"labels": labs}
    ok &= all(v == "NO_DETECTED_CHANGE" for v in labs.values())
    r = readings(synthetic(bump=("F4Y", 0.08)))
    labs = {x: r["Q1"][x]["crash_bar"]["label"] for x in FACTOR_ARMS}
    res["F4Y_plus_8pp"] = {"labels": labs, "use_of_result": use_of_result(r)}
    ok &= labs["F4Y"] == "MATERIAL" and all(v == "NO_DETECTED_CHANGE" for x, v in labs.items() if x != "F4Y")
    ok &= use_of_result(r)["carriers"] == ["F4Y"]
    return bool(ok), res


def load_eval(ident_dir, eval_dir):
    with S.installed():                                     # review r1 P3-1: stage 1's constants restored afterwards
        return _load_eval(ident_dir, eval_dir)


def _load_eval(ident_dir, eval_dir):
    ident = json.loads((pathlib.Path(ident_dir) / "identity_report.json").read_text())
    ev = json.loads((pathlib.Path(eval_dir) / "eval_run_report.json").read_text())
    b = {"identity_verdict": ident.get("verdict"), "eval_verdict": ev.get("verdict"),
         "identity_ok": ident.get("verdict") == "IDENTITY_PASS",
         "same_commit": ident.get("commit") == ev.get("commit"),
         "eval_names_identity": str(pathlib.Path(ident_dir).resolve() / "identity_report.json")
         in (ev.get("linked_reports") or {}).values(),
         "unit": ev.get("unit") == S.UNIT and ev.get("scope") == S.SCOPE,
         "references_at_pins": not R.reference_problems(),
         "script_matches_eval_commit": A.file_matches_commit(pathlib.Path(__file__).resolve(),
                                                             "tools/camref_stage2_analyse.py", ev.get("commit") or "")}
    cells = {}
    for r in ev.get("records") or []:
        if r.get("verdict") == "OK" and r.get("arm") in FACTOR_ARMS:
            cells.setdefault(r["arm"], {})[r["seed"]] = A.load_cell(r["cell_dir"])
    cells["F4"] = {s: A.load_cell(R.reference_dir("CAMREF1_F4", s)) for s in SEEDS}
    cells["F8"] = {s: A.load_cell(R.reference_dir("CAMREF1_F8", s)) for s in SEEDS}
    b["complete_blocks"] = sorted(s for s in SEEDS if all(s in (cells.get(a) or {}) for a in FACTOR_ARMS))
    b["ok"] = all(b[k] for k in ("identity_ok", "same_commit", "eval_names_identity", "unit", "references_at_pins",
                                 "script_matches_eval_commit"))
    return b, cells


def main(argv=None):
    argv = list(argv if argv is not None else sys.argv[1:])
    if len(argv) == 2 and argv[0] == "controls":
        ok, res = controls()
        pathlib.Path(argv[1]).write_text(json.dumps({"controls_ok": ok, "controls": res}, indent=1, default=float) + "\n")
        print(json.dumps({"controls_ok": ok}, indent=1))
        return 0 if ok else 1
    if len(argv) != 4 or argv[0] != "eval":
        raise SystemExit(__doc__.split("usage:")[1])
    ident_dir, eval_dir, out = argv[1], argv[2], pathlib.Path(argv[3])
    ok, res = controls()
    binding, cells = load_eval(ident_dir, eval_dir)
    report = {"unit": S.UNIT, "prereg": S.PREREG, "controls_ok": ok, "controls": res, "binding": binding}
    if not ok:
        report["verdict"] = "CONTROL_FAIL"
    elif not binding["ok"]:
        report["verdict"] = "EVAL_BINDING_FAIL"
    elif not binding["complete_blocks"]:
        report["verdict"] = "NOT_RUN"                       # complete seed blocks only; never fall back
    else:
        r = readings(cells, blocks=binding["complete_blocks"])
        report.update(r, use_of_result=use_of_result(r), verdict="READ")
    out.write_text(json.dumps(report, indent=1, default=float) + "\n")
    print(json.dumps({"verdict": report["verdict"], "use_of_result": report.get("use_of_result")}, indent=1))
    return 0 if report["verdict"] == "READ" else 1


if __name__ == "__main__":
    raise SystemExit(main())
