"""Analysis of CAM-S3-E (docs/plans/prereg_camonly_s3w_eval_2026-11-27.md as amended by
docs/plans/prereg_camonly_s3w_eval_2026-11-27_amendment1.md): CAM-S3 S3-W ep18000 against CAM-S2b-FT ep15000 at P_S,
205 bars, seeds 4501-4503 (one block per seed), the first 8 episodes per env.

Cells: S0 (CAM-DX1 V312, SHA-256 pinned; S0ID reproduced its seed-4501 cell on this commit), S0T (S2b-FT on bars_h3_thin),
S0M (S2b-FT with the measured masks), W (S3-W in its trained runtime on bars_h3), WT (W on bars_h3_thin).
Counts, crash classes (navrl_cams2diag), rates, the paired seed-block differences and their intervals are stage 1's
(tools/camref_eval_analyse.py); the episode bootstrap draws 10,000 resamples from numpy default_rng(4711), cells in the
fixed order CELL_ORDER x seeds. Only the complete seed blocks are read, everywhere (per-cell summaries included).
Readings (amendment 1):
  Q1  (primary) W against the user's bar on the episode-weighted pooled rates of the complete blocks:
      BAR_MET_POINT iff crash <= 20 %, capture >= 70 % and timeout <= 10 % (point estimates), else BAR_NOT_MET_POINT;
      with it stage 1's one-sided 95 % Wilson label (A.bar_label: BELOW_20 / UNRESOLVED / ABOVE_20) for all crashes and
      for bar-contact crashes, the bootstrap intervals and the crash classes.
  Q2  (labelled, co-primary) W - S0 (each policy in its own runtime and sensor model), W - S0M (the same measured sensor
      model; S2b-FT keeps its 1.0472 rad/s action scale, S3-W its 1.5708) and WT - S0T (bars_h3_thin): bar-contact
      crash, crash (all), capture; MATERIAL (|difference| >= 5 pp and the bootstrap interval excludes 0), SMALL
      (interval excludes 0, |difference| < 5 pp), NO_DETECTED_CHANGE; positive = the first cell higher.
      A comparison reads IMPROVES when its crash is MATERIAL and negative and its capture is not MATERIAL and negative;
      CRASH_DOWN_CAPTURE_DOWN when both are MATERIAL and negative; NO_IMPROVEMENT otherwise.
  Q3  (reported, no label) S0M - S0 (the mask model on S2b-FT); WT, S0, S0T, S0M against the bar (as Q1); camsafe and
      speed-governor intervention shares per cell as their modules log them (last [camsafe] and NavRL speedgov lines;
      NOT_RECORDED when a log lacks them).
  Use of the result (fixed; NOT_RUN when Q1 or Q2 W - S0 / W - S0M is NOT_RUN):
    BAR_MET_POINT with the all-crash Wilson label BELOW_20             -> RUNTIME_NEXT (runtime unit, Orin timing);
    BAR_MET_POINT otherwise                                             -> RUNTIME_NEXT_QUALIFIED (the bar is met on the
        point estimates only; the user decides between more seeds and the runtime unit);
    BAR_NOT_MET_POINT, W - S0 and W - S0M both IMPROVES                 -> HELPS_NOT_ENOUGH;
    BAR_NOT_MET_POINT, exactly one of them IMPROVES                     -> SENSOR_MODEL_DEPENDENT (reported with both
        intervals; the user decides);
    BAR_NOT_MET_POINT, none IMPROVES, one CRASH_DOWN_CAPTURE_DOWN      -> CRASH_DOWN_CAPTURE_DOWN (structural next);
    otherwise                                                          -> STRUCTURAL_NEXT (memory S3-M, or the learning
        rate, for the user). No flight claim.
Analysis controls (run before the eval is read; a miss reads CONTROL_FAIL): all cells equal -> Q2 all
NO_DETECTED_CHANGE, STRUCTURAL_NEXT; W crash -8 pp at 21 % -> W - S0 and W - S0M MATERIAL negative, WT - S0T
NO_DETECTED_CHANGE, HELPS_NOT_ENOUGH; W 18 / 73 / 9 % (crash / capture / timeout) -> BAR_MET_POINT, BELOW_20,
RUNTIME_NEXT; 22 / 70 / 8 % and 17 / 71 / 12 % -> BAR_NOT_MET_POINT; review r1 probe cases: (1) 19.6 / 70.5 / 9.9 % ->
BAR_MET_POINT, UNRESOLVED, RUNTIME_NEXT_QUALIFIED; (2) W 53 % capture / 21 % crash against 65 / 29 ->
CRASH_DOWN_CAPTURE_DOWN; (3) S0M at 40 % crash, W at 27 % -> W - S0 NO_DETECTED_CHANGE, W - S0M MATERIAL,
SENSOR_MODEL_DEPENDENT; NOT_RUN cells -> NOT_RUN.
usage: tools/camref_stage3_analyse.py controls OUT_JSON
       tools/camref_stage3_analyse.py eval IDENTITY_DIR EVAL_DIR OUT_JSON
"""
import hashlib
import json
import pathlib
import re
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
Q2 = {"W-S0": {"W": 1.0, "S0": -1.0}, "W-S0M": {"W": 1.0, "S0M": -1.0}, "WT-S0T": {"WT": 1.0, "S0T": -1.0}}
POINTER_PAIR = ("W-S0", "W-S0M")
Q3 = {"S0M-S0": {"S0M": 1.0, "S0": -1.0}}
BAR_REPORTED = ("WT", "S0", "S0T", "S0M")
BOUND_FILES = ("tools/camref_stage3_analyse.py", "tools/camref_eval_analyse.py", "tools/camref_stage3_run.py",
               "tools/camref_eval_run.py")


def bar_reading(cells, name, blocks, boot):
    """Q1 for one cell: pooled rates of the complete blocks against the bar, the point label, the Wilson labels."""
    by = cells.get(name) or {}
    present = [b for b in blocks if (by.get(b) or {}).get("n")]
    if not present or len(present) != len(blocks):
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
        out[metric] = {"k": k, "rate": k / n, "wilson_lower_one_sided_95": lo, "wilson_upper_one_sided_95": hi,
                       "wilson_label": A.bar_label(k, n)}
    met = (pooled["crash"] <= BAR_CRASH_MAX and pooled["capture"] >= BAR_CAPTURE_MIN
           and pooled["timeout"] <= BAR_TIMEOUT_MAX)
    out["label"] = "BAR_MET_POINT" if met else "BAR_NOT_MET_POINT"
    return out


def comparison_status(e):
    """IMPROVES / CRASH_DOWN_CAPTURE_DOWN / NO_IMPROVEMENT of one labelled Q2 effect (NOT_RUN passes through)."""
    if not isinstance(e, dict):
        return "NOT_RUN"
    crash, cap = e.get("crash"), e.get("capture")
    if not isinstance(crash, dict) or not isinstance(cap, dict):
        return "NOT_RUN"
    down = crash["label"] == "MATERIAL" and crash["mean_pp"] < 0
    cap_down = cap["label"] == "MATERIAL" and cap["mean_pp"] < 0
    if down and not cap_down:
        return "IMPROVES"
    return "CRASH_DOWN_CAPTURE_DOWN" if down else "NO_IMPROVEMENT"


def readings(cells, blocks=SEEDS):
    blocks = list(blocks)
    boot = A.bootstrap(cells, seed=BOOT_SEED, order=CELL_ORDER)
    q1 = {"W": bar_reading(cells, "W", blocks, boot)}
    q2 = {k: A.effect(cells, w, blocks, boot, labelled=LABELLED) for k, w in Q2.items()}
    for k in q2:
        if isinstance(q2[k], dict):
            q2[k]["status"] = comparison_status(q2[k])
    q3 = {k: A.effect(cells, w, blocks, boot, labelled=()) for k, w in Q3.items()}
    q3["bar"] = {name: bar_reading(cells, name, blocks, boot) for name in BAR_REPORTED}
    per_cell = {}
    for name, by in cells.items():                         # review r1 P3-2: complete blocks only
        sel = {b: by[b] for b in blocks if b in (by or {})}
        if sel and len(sel) == len(blocks):
            per_cell[name] = A.cell_summary(sel, boot, name)
    return {"Q1": q1, "Q2": q2, "Q3": q3, "cells": per_cell}


def use_of_result(r):
    q1 = r["Q1"]["W"]
    status = {k: comparison_status(r["Q2"].get(k)) for k in POINTER_PAIR}
    if not isinstance(q1, dict) or "NOT_RUN" in status.values():
        return {"pointer": "NOT_RUN", "text": "Q1 or a co-primary Q2 comparison is NOT_RUN (no complete seed block)"}
    wil = q1["all_crashes"].get("wilson_label") if isinstance(q1.get("all_crashes"), dict) else "NOT_RECORDED"
    both = "W - S0 %s, W - S0M %s" % (status["W-S0"], status["W-S0M"])
    if q1["label"] == "BAR_MET_POINT":
        if wil == "BELOW_20":
            return {"pointer": "RUNTIME_NEXT", "text": "W meets the bar on the point estimates and its one-sided 95 %% "
                    "Wilson crash bound is below 20 %%: the runtime unit (rule 2 + 90 deg/s) and the Orin timing are "
                    "next; no flight claim (%s)" % both}
        return {"pointer": "RUNTIME_NEXT_QUALIFIED", "text": "W meets the bar on the point estimates only (all-crash "
                "Wilson label %s): more seeds or the runtime unit, for the user (%s)" % (wil, both)}
    imp = [k for k, v in status.items() if v == "IMPROVES"]
    if len(imp) == 2:
        return {"pointer": "HELPS_NOT_ENOUGH", "text": "W crashes less than S0 and than S0M without a material capture "
                "loss (yaw, arena, masks, sector rule and 3,000 epochs confounded) but misses the bar (%s)" % both}
    if len(imp) == 1:
        return {"pointer": "SENSOR_MODEL_DEPENDENT", "text": "the crash reduction of W depends on the sensor model of "
                "the reference (%s); both intervals reported, the user decides" % both}
    if "CRASH_DOWN_CAPTURE_DOWN" in status.values():
        return {"pointer": "CRASH_DOWN_CAPTURE_DOWN", "text": "W crashes less only together with a material capture loss "
                "(a more passive policy, not an improvement): fine-tuning this lineage stops; structural next (%s)"
                % both}
    return {"pointer": "STRUCTURAL_NEXT", "text": "W misses the bar without a material crash reduction: fine-tuning this "
            "lineage stops; the next design is structural (memory S3-M, or the learning rate), for the user (%s)" % both}


# ---------------------------------------------------------------- intervention shares (reported, review r1 P3-8)
CAMSAFE_RE = re.compile(r"\[camsafe\] calls=(\d+) capped_frac=([0-9.]+) penalised_frac=([0-9.]+)")
GOVERNOR_RE = re.compile(r"NavRL speedgov \| mode=(\w+) intervene=([0-9.]+) stop=([0-9.]+)")


def interventions(log_text):
    """The last camsafe and speed-governor lines of a cell log, as those modules log them (running shares)."""
    cs = CAMSAFE_RE.findall(log_text)
    gv = GOVERNOR_RE.findall(log_text)
    return {"camsafe": ({"calls": int(cs[-1][0]), "capped_frac": float(cs[-1][1]), "penalised_frac": float(cs[-1][2])}
                        if cs else "NOT_RECORDED"),
            "speed_governor": ({"mode": gv[-1][0], "intervene": float(gv[-1][1]), "stop": float(gv[-1][2])}
                               if gv else "NOT_RECORDED")}


def intervention_table(cell_dirs):
    """{cell: {seed: interventions + the log's SHA-256}}; a missing log reads NOT_RECORDED."""
    out = {}
    for name, by in cell_dirs.items():
        for seed, d in by.items():
            log = pathlib.Path(d) / "cell.log"
            if not log.is_file():
                out.setdefault(name, {})[str(seed)] = {"camsafe": "NOT_RECORDED", "speed_governor": "NOT_RECORDED"}
                continue
            data = log.read_bytes()
            out.setdefault(name, {})[str(seed)] = dict(interventions(data.decode(errors="replace")),
                                                       log_sha256=hashlib.sha256(data).hexdigest())
    return out


# ---------------------------------------------------------------- controls
def synthetic(n=1024, base=(0.65, 0.29, 0.06), w=None):
    """{cell: {seed: counts}} with every cell at `base` = (capture, crash, bar crash) and W at `w` if given."""
    cells = {}
    for a in CELL_ORDER:
        cap, crash, bar = w if (w and a == "W") else base
        cells[a] = {s: A.synthetic_counts(n, cap, crash, bar_share=bar / crash) for s in SEEDS}
    return cells


def controls():
    res, ok = {}, True

    def case(name, cells, want):
        r = readings(cells)
        q1 = r["Q1"]["W"]
        got = {"pointer": use_of_result(r)["pointer"]}
        if isinstance(q1, dict):
            got.update(q1=q1["label"], wilson=q1["all_crashes"]["wilson_label"])
        for k in Q2:
            e = r["Q2"][k]
            if isinstance(e, dict):
                got[k] = {m: e[m]["label"] for m in LABELLED}
                got[k + "_status"] = e["status"]
        good = all(got.get(k) == v for k, v in want.items())
        res[name] = {"got": got, "required": want, "ok": good}
        return good

    nd = {m: "NO_DETECTED_CHANGE" for m in LABELLED}
    ok &= case("identical", synthetic(), {"W-S0": nd, "W-S0M": nd, "WT-S0T": nd, "q1": "BAR_NOT_MET_POINT",
                                          "pointer": "STRUCTURAL_NEXT"})
    ok &= case("W_crash_minus_8pp", synthetic(w=(0.73, 0.21, 0.03)),
               {"W-S0_status": "IMPROVES", "W-S0M_status": "IMPROVES", "WT-S0T": nd, "pointer": "HELPS_NOT_ENOUGH"})
    ok &= case("bar_met_18_73_9", synthetic(w=(0.73, 0.18, 0.05)),
               {"q1": "BAR_MET_POINT", "wilson": "BELOW_20", "pointer": "RUNTIME_NEXT"})
    ok &= case("bar_not_met_22_70_8", synthetic(w=(0.70, 0.22, 0.05)), {"q1": "BAR_NOT_MET_POINT"})
    ok &= case("bar_not_met_timeout_17_71_12", synthetic(w=(0.71, 0.17, 0.05)), {"q1": "BAR_NOT_MET_POINT"})
    # review r1 probe cases (~/motar_tmp/cams3e_20261127/review_r1/probe_analysis.py)
    ok &= case("probe1_point_met_wilson_unresolved", synthetic(w=(0.705, 0.196, 0.05)),
               {"q1": "BAR_MET_POINT", "wilson": "UNRESOLVED", "pointer": "RUNTIME_NEXT_QUALIFIED"})
    ok &= case("probe2_crash_down_capture_down", synthetic(w=(0.53, 0.21, 0.05)),
               {"W-S0_status": "CRASH_DOWN_CAPTURE_DOWN", "pointer": "CRASH_DOWN_CAPTURE_DOWN"})
    cells = synthetic()
    cells["S0M"] = {s: A.synthetic_counts(1024, 0.54, 0.40, bar_share=0.5) for s in SEEDS}
    cells["W"] = {s: A.synthetic_counts(1024, 0.67, 0.27, bar_share=0.2) for s in SEEDS}
    ok &= case("probe3_sensor_model_dependent", cells,
               {"W-S0_status": "NO_IMPROVEMENT", "W-S0M_status": "IMPROVES", "pointer": "SENSOR_MODEL_DEPENDENT"})
    cells = synthetic()
    cells["W"] = {4501: cells["W"][4501]}
    r = readings(cells, blocks=SEEDS)
    good = use_of_result(r)["pointer"] == "NOT_RUN" and r["Q1"]["W"] == "NOT_RUN" and "W" not in r["cells"]
    res["incomplete_W_not_run"] = {"pointer": use_of_result(r)["pointer"], "ok": good}
    ok &= good
    return bool(ok), res


# ---------------------------------------------------------------- loading
def load_eval(ident_dir, eval_dir):
    with S.installed():
        return _load_eval(ident_dir, eval_dir)


def _load_eval(ident_dir, eval_dir):
    ident = json.loads((pathlib.Path(ident_dir) / "identity_report.json").read_text())
    ev = json.loads((pathlib.Path(eval_dir) / "eval_run_report.json").read_text())
    commit = ev.get("commit") or ""
    b = {"identity_verdict": ident.get("verdict"), "eval_verdict": ev.get("verdict"),
         "identity_ok": ident.get("verdict") == "IDENTITY_PASS",
         "eval_not_void": ev.get("verdict") in ("COMPLETE", "INCOMPLETE"),       # review r1 P3-1
         "same_commit": ident.get("commit") == commit,
         "eval_names_identity": str(pathlib.Path(ident_dir).resolve() / "identity_report.json")
         in (ev.get("linked_reports") or {}).values(),
         "unit": ev.get("unit") == S.UNIT and ev.get("scope") == S.SCOPE,
         "references_at_pins": not R.reference_problems(),
         "scripts_match_eval_commit": {rel: A.file_matches_commit(A.ROOT / rel, rel, commit)       # review r1 P3-9
                                       for rel in BOUND_FILES}}
    cells, dirs = {}, {}
    for r in ev.get("records") or []:
        if r.get("verdict") == "OK" and r.get("arm") in NEW_ARMS:
            cells.setdefault(r["arm"], {})[r["seed"]] = A.load_cell(r["cell_dir"])
            dirs.setdefault(r["arm"], {})[r["seed"]] = r["cell_dir"]
    cells["S0"] = {s: A.load_cell(R.reference_dir("DX1_V312", s)) for s in SEEDS}
    dirs["S0"] = {s: str(R.reference_dir("DX1_V312", s)) for s in SEEDS}
    rec = next((r for r in ident.get("records") or [] if r.get("arm") == "S0ID" and r.get("verdict") == "OK"), None)
    b["s0id_cell_reported"] = A.load_cell(rec["cell_dir"]) if rec else None
    b["complete_blocks"] = sorted(s for s in SEEDS if all(s in (cells.get(a) or {}) for a in NEW_ARMS))
    b["ok"] = (all(b[k] for k in ("identity_ok", "eval_not_void", "same_commit", "eval_names_identity", "unit",
                                  "references_at_pins")) and all(b["scripts_match_eval_commit"].values()))
    return b, cells, dirs


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
    binding, cells, dirs = load_eval(ident_dir, eval_dir)
    report = {"unit": S.UNIT, "prereg": S.PREREG, "amendment": S.AMENDMENT, "controls_ok": ok, "controls": res,
              "binding": binding}
    if not ok:
        report["verdict"] = "CONTROL_FAIL"
    elif not binding["ok"]:
        report["verdict"] = "EVAL_BINDING_FAIL"
    elif not binding["complete_blocks"]:
        report["verdict"] = "NOT_RUN"                       # complete seed blocks only; never fall back
    else:
        r = readings(cells, blocks=binding["complete_blocks"])
        r["Q3"]["interventions"] = intervention_table(dirs)
        report.update(r, use_of_result=use_of_result(r), verdict="READ")
    out.write_text(json.dumps(report, indent=1, default=float) + "\n")
    print(json.dumps({"verdict": report["verdict"], "use_of_result": report.get("use_of_result")}, indent=1))
    return 0 if report["verdict"] == "READ" else 1


if __name__ == "__main__":
    raise SystemExit(main())
