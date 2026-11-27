"""Analysis of CAM-S3-E (docs/plans/prereg_camonly_s3w_eval_2026-11-27.md): CAM-S3 S3-W ep18000 against CAM-S2b-FT ep15000
at P_S, 205 bars, seeds 4501-4503 (one block per seed), the first 8 episodes per env.

Cells: S0 (CAM-DX1 V312, SHA-256 pinned; S0ID reproduced its seed-4501 cell on this commit), S0T (S2b-FT on bars_h3_thin),
S0M (S2b-FT with the measured masks), W (S3-W in its trained runtime on bars_h3), WT (W on bars_h3_thin).
Counts, crash classes (navrl_cams2diag), rates, the paired seed-block differences and their intervals are stage 1's
(tools/camref_eval_analyse.py); the episode bootstrap draws 10,000 resamples from numpy default_rng(4711), cells in the
fixed order CELL_ORDER x seeds.
Readings:
  Q1  (primary, labelled) W against the user's bar, on the episode-weighted pooled rates of the complete seed blocks:
      BAR_MET iff crash <= 20 %, capture >= 70 % and timeout <= 10 % (point estimates); else BAR_NOT_MET. Reported with
      it: bootstrap intervals, one-sided 95 % Wilson bounds of crash and bar-contact crash, bar / target-contact / OOB.
  Q2  (labelled) W - S0 (bars_h3) and WT - S0T (bars_h3_thin): bar-contact crash, crash (all), capture; MATERIAL
      (|difference| >= 5 pp and the bootstrap interval excludes 0), SMALL (interval excludes 0, |difference| < 5 pp),
      NO_DETECTED_CHANGE (otherwise); positive = the first cell higher.
  Q3  (reported, no label) S0M - S0 (the mask model on S2b-FT), W - S0M (policy + sector rule + 90 deg/s under the same
      sensor model); WT, S0, S0T, S0M against the bar (as Q1).
  Use of the result (fixed): Q1 BAR_MET -> the runtime unit (rule 2 + 90 deg/s in the flight runtime) and the Orin timing
  are next; BAR_NOT_MET with Q2 W - S0 crash MATERIAL and negative -> the CAM-S3 changes lower the crash but not to the
  bar; otherwise -> fine-tuning this lineage stops and the next design is structural (memory S3-M, or the learning
  rate), for the user. No flight claim.
Analysis controls (run before the eval is read; a miss reads CONTROL_FAIL): synthetic cells all equal -> Q2 all
NO_DETECTED_CHANGE; W crash -8 pp (capture +8 pp) in every block -> W - S0 crash MATERIAL negative, WT - S0T
NO_DETECTED_CHANGE; W at 18 / 73 / 9 % (crash / capture / timeout) -> BAR_MET; W at 22 / 70 / 8 % and at 17 / 71 / 12 %
-> BAR_NOT_MET; the three use-of-result pointers (BAR_MET -> RUNTIME_NEXT, W at 21 % crash and -8 pp -> HELPS_NOT_ENOUGH,
all equal at 29 % crash -> STRUCTURAL_NEXT).
usage: tools/camref_stage3_analyse.py controls OUT_JSON
       tools/camref_stage3_analyse.py eval IDENTITY_DIR EVAL_DIR OUT_JSON
"""
import json
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import camref_eval_analyse as A                              # noqa: E402  (counts, bootstrap, effect, load_cell, wilson)
import camref_stage3_run as S                                # noqa: E402  (arms, pins)

R = S.R
NEW_ARMS = S.NEW_ARMS                                        # S0T, S0M, W, WT
CELL_ORDER = ("S0",) + NEW_ARMS
BOOT_SEED = 4711
LABELLED = ("crash_bar", "crash", "capture")
SEEDS = R.SEEDS
BAR_CRASH_MAX, BAR_CAPTURE_MIN, BAR_TIMEOUT_MAX = 0.20, 0.70, 0.10
Q2 = {"W-S0": {"W": 1.0, "S0": -1.0}, "WT-S0T": {"WT": 1.0, "S0T": -1.0}}
Q3 = {"S0M-S0": {"S0M": 1.0, "S0": -1.0}, "W-S0M": {"W": 1.0, "S0M": -1.0}}
BAR_REPORTED = ("WT", "S0", "S0T", "S0M")


def bar_reading(cells, name, blocks, boot):
    """Q1 for one cell: pooled rates of the complete blocks against the bar, with intervals and Wilson bounds."""
    by = cells.get(name) or {}
    present = [b for b in blocks if (by.get(b) or {}).get("n")]
    if not present:
        return "NOT_RUN"
    n = sum(by[b]["n"] for b in present)
    pooled = {m: sum(by[b][m] for b in present) / n for m in ("capture", "crash", "timeout")}
    out = {"blocks": present, "n": n, "pooled": pooled,
           "summary": A.cell_summary({b: by[b] for b in present}, boot, name)}
    for metric, key in (("all_crashes", "crash"), ("bar_contact_crashes", "crash_bar")):
        if any(A.rate(by[b], key) is None for b in present):
            out[metric] = "NOT_RECORDED"
            continue
        k = sum(by[b][key] for b in present)
        lo, hi = A.wilson(k, n)
        out[metric] = {"k": k, "rate": k / n, "wilson_lower_one_sided_95": lo, "wilson_upper_one_sided_95": hi}
    met = (pooled["crash"] <= BAR_CRASH_MAX and pooled["capture"] >= BAR_CAPTURE_MIN
           and pooled["timeout"] <= BAR_TIMEOUT_MAX)
    out["label"] = "BAR_MET" if met else "BAR_NOT_MET"
    return out


def readings(cells, blocks=SEEDS):
    boot = A.bootstrap(cells, seed=BOOT_SEED, order=CELL_ORDER)
    q1 = {"W": bar_reading(cells, "W", blocks, boot)}
    q2 = {k: A.effect(cells, w, blocks, boot, labelled=LABELLED) for k, w in Q2.items()}
    q3 = {k: A.effect(cells, w, blocks, boot, labelled=()) for k, w in Q3.items()}
    q3["bar"] = {name: bar_reading(cells, name, blocks, boot) for name in BAR_REPORTED}
    per_cell = {name: A.cell_summary(by, boot, name) for name, by in cells.items() if by}
    return {"Q1": q1, "Q2": q2, "Q3": q3, "cells": per_cell}


def use_of_result(r):
    q1 = r["Q1"]["W"]
    lab = q1.get("label") if isinstance(q1, dict) else q1
    e = r["Q2"].get("W-S0")
    crash = e.get("crash") if isinstance(e, dict) else None
    if lab == "BAR_MET":
        return {"pointer": "RUNTIME_NEXT", "text": "W meets the bar (crash <= 20 %, capture >= 70 %, timeout <= 10 %): "
                "the runtime unit (rule 2 + 90 deg/s in the flight runtime) and the Orin timing are next; no flight claim"}
    if isinstance(crash, dict) and crash["label"] == "MATERIAL" and crash["mean_pp"] < 0:
        return {"pointer": "HELPS_NOT_ENOUGH", "text": "the CAM-S3 changes lower the crash materially (W - S0 %.1f pp) "
                "but W misses the bar" % crash["mean_pp"]}
    return {"pointer": "STRUCTURAL_NEXT", "text": "W misses the bar without a material crash reduction against S0: "
            "fine-tuning this lineage stops; the next design is structural (memory S3-M, or the learning rate), for the "
            "user (Q1 %s, W - S0 crash %s)" % (lab, crash.get("label") if isinstance(crash, dict) else crash)}


def synthetic(n=1024, base=(0.65, 0.29, 0.06), w=None):
    """{cell: {seed: counts}} with every cell at `base` = (capture, crash, bar crash) and W at `w` if given."""
    cells = {}
    for a in CELL_ORDER:
        cap, crash, bar = w if (w and a == "W") else base
        cells[a] = {s: A.synthetic_counts(n, cap, crash, bar_share=bar / crash) for s in SEEDS}
    return cells


def controls():
    res, ok = {}, True
    r = readings(synthetic())
    labs = {k: {m: r["Q2"][k][m]["label"] for m in LABELLED} for k in Q2}
    good = all(v == "NO_DETECTED_CHANGE" for d in labs.values() for v in d.values())
    res["identical"] = {"labels": labs, "ok": good}
    ok &= good
    r = readings(synthetic(w=(0.73, 0.21, 0.03)))
    labs = {k: {m: r["Q2"][k][m]["label"] for m in LABELLED} for k in Q2}
    good = (labs["W-S0"]["crash"] == "MATERIAL" and r["Q2"]["W-S0"]["crash"]["mean_pp"] < 0
            and all(v == "NO_DETECTED_CHANGE" for v in labs["WT-S0T"].values()))
    res["W_crash_minus_8pp"] = {"labels": labs, "use_of_result": use_of_result(r), "ok": good}
    ok &= good
    for name, w, want in (("bar_met_18_73_9", (0.73, 0.18, 0.05), "BAR_MET"),
                          ("bar_not_met_22_70_8", (0.70, 0.22, 0.05), "BAR_NOT_MET"),
                          ("bar_not_met_timeout_17_71_12", (0.71, 0.17, 0.05), "BAR_NOT_MET")):
        r = readings(synthetic(w=w))
        got = r["Q1"]["W"]["label"]
        res[name] = {"W": w, "pooled": r["Q1"]["W"]["pooled"], "label": got, "required": want,
                     "use_of_result": use_of_result(r), "ok": got == want}
        ok &= got == want
    pointers = {"bar_met": res["bar_met_18_73_9"]["use_of_result"]["pointer"],
                "minus_8pp_at_21": res["W_crash_minus_8pp"]["use_of_result"]["pointer"],
                "identical": use_of_result(readings(synthetic()))["pointer"]}
    res["pointers"] = {"got": pointers, "ok": pointers == {"bar_met": "RUNTIME_NEXT", "minus_8pp_at_21": "HELPS_NOT_ENOUGH",
                                                            "identical": "STRUCTURAL_NEXT"}}
    ok &= res["pointers"]["ok"]
    return bool(ok), res


def load_eval(ident_dir, eval_dir):
    with S.installed():
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
                                                             "tools/camref_stage3_analyse.py", ev.get("commit") or "")}
    cells = {}
    for r in ev.get("records") or []:
        if r.get("verdict") == "OK" and r.get("arm") in NEW_ARMS:
            cells.setdefault(r["arm"], {})[r["seed"]] = A.load_cell(r["cell_dir"])
    cells["S0"] = {s: A.load_cell(R.reference_dir("DX1_V312", s)) for s in SEEDS}
    rec = next((r for r in ident.get("records") or [] if r.get("arm") == "S0ID" and r.get("verdict") == "OK"), None)
    b["s0id_cell_reported"] = A.load_cell(rec["cell_dir"]) if rec else None
    b["complete_blocks"] = sorted(s for s in SEEDS if all(s in (cells.get(a) or {}) for a in NEW_ARMS))
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
