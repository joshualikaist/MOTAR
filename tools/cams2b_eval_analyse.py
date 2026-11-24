"""Analysis of CAM-S2b-FT-E (docs/plans/prereg_camonly_s2b_ft_eval_2026-11-24.md).

Blocks are (density, seed) pairs: densities 175 and 205 x seeds 4501-4503 = 6 blocks per arm; rates are per-block quota
rates (gr_analyse.rates). Intervals as CAM-S2a-E (tools/cams2a_eval_analyse.py, imported unchanged): a stratified
percentile bootstrap (10,000 resamples, seed 20,261,124, seeds resampled within density) and a stratified t-interval with
df = sum_d (n_d - 1) = 4 (t 0.975 = 2.7764). A label needs both intervals to agree. Crash contrasts use CAM-K2-E's gate-A
rule with CAM-K2-E3's labels (cams2a_eval_analyse.crash_label); capture and timeout contrasts C2's scheme (bands 3 pp and
2 pp).

Gated:
  G1 (flight candidate)  FT - A60 on the 6 blocks:
        better = crash in {SMALL_REDUCTION, MATERIAL_REDUCTION, CRASH_TO_TIMEOUT} or capture HIGHER;
        worse  = crash WORSE or capture LOWER.
        better and worse -> FT_TRADEOFF; worse -> FT_WORSE;
        better, crash in {NEGLIGIBLE, SMALL_REDUCTION, MATERIAL_REDUCTION, CRASH_TO_TIMEOUT} and capture in
        {NEGLIGIBLE, SMALL_HIGHER, HIGHER} -> FT_BETTER (review r1 P2-3: a capture gain never hides a crash rise, not even
        one under the 2 pp floor that NO_DETECTED_CHANGE would carry);
        crash NEGLIGIBLE and capture in {NEGLIGIBLE, SMALL_HIGHER} -> FT_NO_MATERIAL_DIFFERENCE;
        otherwise FT_UNRESOLVED.
        Without bound A60 cells: G1_NOT_ANSWERED (a60_source A60_PATH_FAIL / A60_NOT_REPRODUCED, or RERUN_REQUIRED
        without stage a60) or A60_BINDING_FAIL.
  G2 (45 deg/s)          FT45 - FT: CAM-S2a-E's gate-Y rule with the prefix YAWCLIP45_: [yawclip] telemetry missing in
        any FT45 cell -> YAWCLIP45_INACTIVE; pooled binding share (FT45 recorder rows, requested |yaw| = |action| x 1.0472
        over 0.7854 rad/s) < 1 % -> YAWCLIP45_WEAKLY_BINDING; crash WORSE or capture LOWER -> YAWCLIP45_SENSITIVE; crash in
        {NEGLIGIBLE, SMALL_REDUCTION, MATERIAL_REDUCTION} and capture in {NEGLIGIBLE, SMALL_HIGHER, HIGHER, SMALL_LOWER} ->
        YAWCLIP45_TOLERATED (admits a capture loss under 3 pp); otherwise YAWCLIP45_UNRESOLVED.
Reported, not gated: pooled and per-density rates per arm; crash per 1,000 steps; crash split (recorder classes
target_contact_swept / contact / below / above / out_of_bounds); capture kinematics (relative speed, closing, lateral:
mean, p50, p90, share above 1.0 m/s); blind behaviour; yaw binding shares of every arm at its recorder reference;
[yawclip] / [camsafe] last lines (running totals); every MISMATCH line.
Controls (computed before the run on CAM-K1-E's cells at 160/205 bars x 4501-4503, the two CAM-K1-E densities nearest to
175/205, i.e. THIS unit's 6-block, df-4 geometry; any required miss reads CONTROL_FAIL and no gate label is given): at
2,048 episodes per cell the crash rule must read WORSE on F66 - K1s1, not MATERIAL_REDUCTION or WORSE on K1s1 - K1s2,
MATERIAL_REDUCTION on K1s1 - F66 and the capture scheme HIGHER on K1s1 - F66; at this unit's 1,024 episodes per cell the
K1s1 - K1s2 null must not read MATERIAL_REDUCTION or WORSE, the null with +5 pp crash (-5 pp capture) added to K1s1 must
read WORSE (LOWER); reported, not required: the +3 pp and +2 pp crash contrasts; the cells must match CAM-K1-E's manifest.
Bindings (EVAL_BINDING_FAIL otherwise; the A60 group alone gives A60_BINDING_FAIL for G1 and leaves G2 labelled): see
report_bindings, a60_bindings and load_eval.
usage: tools/cams2b_eval_analyse.py controls CAMK1E_RAW OUT_JSON
       tools/cams2b_eval_analyse.py eval CAMK1E_RAW IDENTITY_DIR EVAL_DIR A60_DIR|- OUT_JSON
"""
import json
import pathlib
import random
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import camk1e_analyse as E1                                  # noqa: E402  (strat_ci, file_matches_commit, sha)
import camonly_c2_analyse as C2A                             # noqa: E402  (governor)
import gr_analyse as A                                       # noqa: E402  (rates, BOOT)
import kroc2027_eval as K                                    # noqa: E402  (read_jsonl)
import cams2a_eval_analyse as SA                             # noqa: E402  (CAM-S2a-E's rules and diagnostics, unchanged)
import cams2b_eval_run as W                                  # noqa: E402  (arms, plans, cell checks, reuse)

ROOT = pathlib.Path(__file__).resolve().parents[1]
BOOT_SEED = SA.BOOT_SEED
METRICS = SA.METRICS
K1E_DENSITIES = (160, 205)
G1 = "FT_minus_A60"
G2 = "FT45_minus_FT"
CONTRASTS = {G1: {"FT": 1, "A60": -1}, G2: {"FT45": 1, "FT": -1}}
ARMS = ("FT", "FT45", "A60")
MIN_BINDING_SHARE = SA.MIN_BINDING_SHARE                     # 1 %
CRASH_BETTER = ("SMALL_REDUCTION", "MATERIAL_REDUCTION", "CRASH_TO_TIMEOUT")


def blocks():
    return [(d, s) for d in W.DENSITIES for s in W.SEEDS]


def gate_g1(crash, capture):
    better = crash in CRASH_BETTER or capture == "HIGHER"
    worse = crash == "WORSE" or capture == "LOWER"
    if better and worse:
        return "FT_TRADEOFF"
    if worse:
        return "FT_WORSE"
    if better and crash in ("NEGLIGIBLE",) + CRASH_BETTER and capture in ("NEGLIGIBLE", "SMALL_HIGHER", "HIGHER"):
        return "FT_BETTER"
    if crash == "NEGLIGIBLE" and capture in ("NEGLIGIBLE", "SMALL_HIGHER"):
        return "FT_NO_MATERIAL_DIFFERENCE"
    return "FT_UNRESOLVED"


def gate_g2(crash, capture, telemetry_present, binding_share):
    return SA.gate_y(crash, capture, telemetry_present, binding_share).replace("YAWCLIP_", "YAWCLIP45_", 1)


# ---------------------------------------------------------------- controls (CAM-K1-E, 6 blocks, df 4)
def controls(k1e):
    """SA.controls' plan on this unit's geometry: 2 densities x 3 seeds, df 4. Fixed order, one generator."""
    blks = [(d, s) for d in K1E_DENSITIES for s in W.SEEDS]
    full = SA.load_camk1e(k1e, blks)
    q8 = SA.load_camk1e(k1e, blks, quota=W.QUOTA)
    q8_shift = SA.shifted(q8, "K1s1", SA.SHIFT_PP)
    null_mean = 100.0 * sum(q8["K1s1"][b]["crash"] - q8["K1s2"][b]["crash"] for b in blks) / len(blks)
    q8_contrast = {pp: SA.shifted(q8, "K1s1", pp - null_mean) for pp in SA.CONTRAST_REPORTED_PP}
    rng = random.Random(BOOT_SEED)
    out = {}
    plan = (("crash_on_K1E_F66_minus_K1s1", full, {"F66": 1, "K1s1": -1}, "crash", "WORSE"),
            ("crash_on_K1E_K1s1_minus_K1s2", full, {"K1s1": 1, "K1s2": -1}, "crash", "not MATERIAL_REDUCTION or WORSE"),
            ("crash_on_K1E_K1s1_minus_F66", full, {"K1s1": 1, "F66": -1}, "crash", "MATERIAL_REDUCTION"),
            ("capture_on_K1E_K1s1_minus_F66", full, {"K1s1": 1, "F66": -1}, "capture", "HIGHER"),
            ("crash_on_K1E_K1s1_minus_K1s2_quota8", q8, {"K1s1": 1, "K1s2": -1}, "crash", "not MATERIAL_REDUCTION or WORSE"),
            ("crash_on_K1E_K1s1_plus5pp_minus_K1s2_quota8", q8_shift, {"K1s1": 1, "K1s2": -1}, "crash", "WORSE"),
            ("capture_on_K1E_K1s1_minus5pp_minus_K1s2_quota8", q8_shift, {"K1s1": 1, "K1s2": -1}, "capture", "LOWER"),
            ("crash_contrast_plus3pp_K1s1_minus_K1s2_quota8", q8_contrast[3.0], {"K1s1": 1, "K1s2": -1}, "crash", "REPORTED"),
            ("crash_contrast_plus2pp_K1s1_minus_K1s2_quota8", q8_contrast[2.0], {"K1s1": 1, "K1s2": -1}, "crash", "REPORTED"))
    for name, cells, w, metric, required in plan:
        c = SA.labelled(cells, w, blks, rng)
        label = c[metric]["label"]
        ok = (True if required == "REPORTED" else
              label not in ("MATERIAL_REDUCTION", "WORSE") if required.startswith("not ") else label == required)
        out[name] = {"metric": metric, "label": label, "required": required, "ok": ok, "mean_pp": c[metric]["mean_pp"],
                     "ci95_pp": c[metric]["ci95_pp"], "t95_pp": c[metric]["t95_pp"], "t_df": c[metric]["t_df"],
                     "episodes_per_cell": cells["K1s1"][blks[0]]["n"]}
    for name in ("crash_on_K1E_K1s1_minus_K1s2", "crash_on_K1E_K1s1_minus_K1s2_quota8"):
        cr = out[name]
        cr["half_width_pp"] = {"bootstrap": (cr["ci95_pp"][1] - cr["ci95_pp"][0]) / 2, "t": (cr["t95_pp"][1] - cr["t95_pp"][0]) / 2}
    out["blocks"] = [list(b) for b in blks]
    out["quota8_null_mean_pp"] = null_mean
    out["contrast_shift_pp"] = {str(pp): pp - null_mean for pp in SA.CONTRAST_REPORTED_PP}
    out["camk1e_cells_match_manifest"] = SA.camk1e_manifest_ok(k1e, blks)
    out["ok"] = all(v["ok"] for v in out.values() if isinstance(v, dict) and "ok" in v) and out["camk1e_cells_match_manifest"]
    return out


# ---------------------------------------------------------------- bindings
def report_bindings(ident, ev, ident_path, blocks_present):
    """The bindings common to both gates (pure). `blocks_present` maps arm -> sorted blocks with an OK, re-checked cell."""
    plan = W.eval_plan()
    recs = ev.get("records") or []
    order = [(r.get("arm"), r.get("density"), r.get("seed")) for r in recs]
    linked = (ev.get("linked_reports") or {}).get("identity_report.json")
    b = {"identity_pass": ident.get("verdict") == "IDENTITY_PASS",
         "identity_same_commit": bool(ident.get("commit")) and ident.get("commit") == ev.get("commit"),
         "identity_is_the_one_eval_named": bool(linked) and pathlib.Path(linked).resolve() == pathlib.Path(ident_path).resolve(),
         "eval_complete": ev.get("verdict") == "COMPLETE" and ev.get("cells_planned") == len(plan) == 12,
         "order_as_planned": order == plan,
         "all_records_ok": len(recs) == len(plan) and all(r.get("verdict") == "OK" for r in recs),
         "env_allowlisted_same": bool(ident.get("env_allowlisted")) and ident.get("env_allowlisted") == ev.get("env_allowlisted"),
         "same_training_inputs_and_ft_checkpoint": all(
             bool(ident.get(k)) and ident.get(k) == ev.get(k) for k in
             ("s2a_train_receipt_sha256", "ft_train_receipt_sha256", "ft_train_report_sha256", "ft_checkpoint_sha256")),
         "changed_paths_allowlisted": ev.get("changed_since_s2a_e") is not None and ev.get("changed_since_ft_training") is not None
         and not W.allowlist_problems(ev.get("changed_since_s2a_e"), W.S2A_CODE_ALLOWLIST, W.S2A_E_COMMIT)
         and not W.allowlist_problems(ev.get("changed_since_ft_training"), W.FT_CODE_ALLOWLIST, ev.get("ft_train_head") or "")}
    for arm in W.EVAL_ARMS:
        b["complete_grid_" + arm] = blocks_present.get(arm, []) == blocks()
    return b


def a60_bindings(ident, a60_report, ev, eval_report_path, blocks_present, recheck_ok):
    """The A60 group (pure on reports). Returns (source, bindings); source REUSED, RERUN or None (not answerable)."""
    src = ident.get("a60_source")
    b = {"a60_complete_grid": blocks_present.get("A60", []) == blocks()}
    if src == "REUSE_OK":
        repro = ident.get("a60_reproduction") or {}
        b.update(identity_reuse_decision_ok=not ident.get("reuse_problems"),
                 reproduction_equal=all(repro.get(k) is True for k in ("result_equal", "episodes_equal", "diag_equal")),
                 reuse_manifest_unchanged=not W.manifest_problems(ident.get("reuse_manifest")),
                 reused_cells_pass_checks_again=recheck_ok, no_a60_rerun_report=a60_report is None)
        return "REUSED", b
    if src == "RERUN_REQUIRED" and a60_report is not None:
        recs = a60_report.get("records") or []
        linked = (a60_report.get("linked_reports") or {}).get("eval_run_report.json")
        b.update(a60_complete=a60_report.get("verdict") == "COMPLETE"
                 and [(r.get("arm"), r.get("density"), r.get("seed")) for r in recs] == W.a60_plan()
                 and all(r.get("verdict") == "OK" for r in recs),
                 a60_same_commit=a60_report.get("commit") == ev.get("commit"),
                 a60_after_this_eval=bool(linked) and pathlib.Path(linked).resolve() == pathlib.Path(eval_report_path).resolve(),
                 a60_same_training_inputs=all(a60_report.get(k) == ev.get(k) for k in
                                              ("s2a_train_receipt_sha256", "ft_checkpoint_sha256", "env_allowlisted")),
                 rerun_cells_pass_checks_again=recheck_ok)
        return "RERUN", b
    return None, b


def recheck_cell(cell_dir, arm, d, s, commit, committed, train, ckpt_sha, root):
    receipt, receipt_sha = train
    p, _ = W.output_problems(cell_dir, commit, ckpt_sha, receipt_sha, receipt.get("env_navrl") or {})
    if not p:
        p += W.receipt_problems(cell_dir, arm, d, s, receipt, receipt_sha, committed, ckpt_sha, root=root)
    return p


def load_eval(ident_dir, eval_dir, a60_dir=None):
    ident = json.loads((ident_dir / "identity_report.json").read_text())
    ev = json.loads((eval_dir / "eval_run_report.json").read_text())
    a60_report = json.loads((a60_dir / "a60_run_report.json").read_text()) if a60_dir else None
    s2a_receipt = json.loads(W.S2A_TRAIN_RECEIPT.read_text()) if W.S2A_TRAIN_RECEIPT.is_file() else {}
    ft_receipt = json.loads(W.FT_TRAIN_RECEIPT.read_text()) if W.FT_TRAIN_RECEIPT.is_file() else {}
    train = {"FT": (ft_receipt, ev.get("ft_train_receipt_sha256")), "S2A": (s2a_receipt, ev.get("s2a_train_receipt_sha256"))}
    committed = ev.get("committed_sha256") or {}
    ft_pin = ev.get("ft_checkpoint_sha256")
    cells = {a: {} for a in ARMS}
    diag = {}
    problems = {}
    ok_common, ok_a60 = True, True
    for r in ev.get("records") or []:
        arm, blk = r.get("arm"), (r.get("density"), r.get("seed"))
        if arm not in W.EVAL_ARMS or r.get("verdict") != "OK":
            ok_common = False
            continue
        d = eval_dir / arm / r["cell_id"]
        try:
            p = recheck_cell(d, arm, blk[0], blk[1], ev.get("commit"), committed, train["FT"], ft_pin,
                             pathlib.Path(ev.get("tree") or "/nonexistent"))      # the eval's tree (review r1 P2-5)
            p += [] if r.get("checkpoint_sha256") == ft_pin else ["record checkpoint is not the FT pin"]
            if p:
                problems["%s/%d/%d" % (arm, blk[0], blk[1])] = p
                ok_common = False
                continue
            cells[arm][blk] = {"dir": d, **A.rates(d)}
            diag["%s/%d/%d" % (arm, blk[0], blk[1])] = SA.cell_diagnostics(d)
        except (OSError, ValueError, KeyError, TypeError) as e:
            problems["%s/%d/%d" % (arm, blk[0], blk[1])] = [str(e)]
            ok_common = False
    # The A60 cells: reused (CAM-S2a-E's tree and commit) or rerun here (stage a60).
    src = ident.get("a60_source")
    if src == "REUSE_OK":
        at = W.committed_sha256_at(W.S2A_E_COMMIT)
        a60_dirs = {b: (W.reused_cell_dir(*b), W.S2A_E_COMMIT, at, W.S2A_E_TREE) for b in blocks()}
    elif src == "RERUN_REQUIRED" and a60_report is not None:
        a60_dirs = {(r.get("density"), r.get("seed")): (a60_dir / "A60" / r["cell_id"], a60_report.get("commit"),
                                                        a60_report.get("committed_sha256") or {},
                                                        pathlib.Path(a60_report.get("tree") or "/nonexistent"))
                    for r in a60_report.get("records") or [] if r.get("arm") == "A60" and r.get("verdict") == "OK"}
    else:
        a60_dirs = {}
    for blk, (d, commit, comm, root) in sorted(a60_dirs.items()):
        try:
            p = recheck_cell(d, "A60", blk[0], blk[1], commit, comm, (s2a_receipt, W.sha(W.S2A_TRAIN_RECEIPT)), W.S2A_CKPT_SHA, root)
            if p:
                problems["A60/%d/%d" % blk] = p
                ok_a60 = False
                continue
            cells["A60"][blk] = {"dir": d, **A.rates(d)}
            diag["A60/%d/%d" % blk] = SA.cell_diagnostics(d)
        except (OSError, ValueError, KeyError, TypeError) as e:
            problems["A60/%d/%d" % blk] = [str(e)]
            ok_a60 = False
    present = {a: sorted(c) for a, c in cells.items()}
    binding = report_bindings(ident, ev, ident_dir / "identity_report.json", present)
    binding["cells_pass_the_runner_checks_again"] = ok_common
    binding["recorder_lengths_equal_ledger"] = all(v["recorder_lengths_equal_ledger"] for k, v in diag.items()
                                                   if not k.startswith("A60/"))
    binding["ft45_executed_yaw_never_over_clip"] = all(v["yaw"]["exec_over_ref_steps"] == 0 for k, v in diag.items()
                                                       if k.startswith("FT45/"))
    binding["training_receipts_at_recorded_sha"] = (W.FT_TRAIN_RECEIPT.is_file() and W.S2A_TRAIN_RECEIPT.is_file()
                                                    and W.sha(W.FT_TRAIN_RECEIPT) == ev.get("ft_train_receipt_sha256")
                                                    and W.sha(W.S2A_TRAIN_RECEIPT) == ev.get("s2a_train_receipt_sha256"))
    for ident_arm, eval_arm in (("FT_identity", "FT"),):
        a = ident_dir / ident_arm / W.cell_id(ident_arm, 205, 4501)
        b = eval_dir / eval_arm / W.cell_id(eval_arm, 205, 4501)
        try:
            c = W.G.identity_compare(a, b)
            binding["identity_%s_equals_eval_%s" % (ident_arm, eval_arm)] = c["result_equal"] and c["episodes_equal"]
        except (OSError, ValueError, KeyError):
            binding["identity_%s_equals_eval_%s" % (ident_arm, eval_arm)] = False
    source, ab = a60_bindings(ident, a60_report, ev, eval_dir / "eval_run_report.json", present, ok_a60)
    ab["a60_recorder_lengths_equal_ledger"] = all(v["recorder_lengths_equal_ledger"] for k, v in diag.items() if k.startswith("A60/"))
    ab["a60_executed_yaw_never_over_clip"] = all(v["yaw"]["exec_over_ref_steps"] == 0 for k, v in diag.items() if k.startswith("A60/"))
    if source == "RERUN":
        a = ident_dir / "A60_repro" / W.cell_id("A60_repro", *W.REPRO_BLOCK)
        b = a60_dir / "A60" / W.cell_id("A60", *W.REPRO_BLOCK)
        try:
            c = W.G.identity_compare(a, b)
            ab["identity_A60_repro_equals_a60_cell"] = c["result_equal"] and c["episodes_equal"]
        except (OSError, ValueError, KeyError):
            ab["identity_A60_repro_equals_a60_cell"] = False
    return binding, ab, source, cells, ev, diag, problems


def differ(cells, a, b, blks):
    return all(blk in cells[a] and blk in cells[b] and K.read_jsonl(cells[a][blk]["dir"] / "episodes.jsonl")
               != K.read_jsonl(cells[b][blk]["dir"] / "episodes.jsonl") for blk in blks)


def ft_training_summary(recorded_sha=None):
    """S2b-FT's training run as its report records it (reported, not gated; review r1 P3-7): did it reach 205 bars?
    `report_sha256_equals_eval_record` says whether the report read now is the one the eval stage bound (review r2 P3-4)."""
    try:
        r = json.loads(W.FT_TRAIN_REPORT.read_text())
    except (OSError, ValueError):
        return "NOT_RECORDED"
    stages = r.get("stages")
    return {"report_sha256_equals_eval_record": W.sha(W.FT_TRAIN_REPORT) == recorded_sha, "verdict": r.get("verdict"), "last_epoch": r.get("last_epoch"), "final_checkpoint": r.get("final_checkpoint"),
            "promotions": r.get("promotions"), "curriculum_check": r.get("curriculum_check"),
            "last_stage": stages[-1] if isinstance(stages, list) and stages else stages}


def write(out, res):
    with out.open("x") as fh:
        json.dump(res, fh, indent=1, default=str)
        fh.write("\n")


def main_controls(k1e, out):
    res = {"analysis": "CAM-S2b-FT-E controls", "controls": controls(k1e),
           "bootstrap": {"resamples": A.BOOT, "seed": BOOT_SEED, "scheme": "seeds within density", "t_df": 4},
           "script_sha256": E1.sha(__file__), "cams2a_analysis_sha256": E1.sha(SA.__file__)}
    write(out, res)
    print(json.dumps({k: (v["label"] if isinstance(v, dict) and "label" in v else v) for k, v in res["controls"].items()}, indent=1))
    return 0 if res["controls"]["ok"] else 1


def main_eval(k1e, ident_dir, eval_dir, a60_dir, out):
    blks = blocks()
    binding, ab, source, cells, ev, diag, problems = load_eval(ident_dir, eval_dir, a60_dir)
    _ck, s2a_ck_problems = W.s2a_checkpoint()
    ftp = pathlib.Path(ev.get("ft_checkpoint") or "/nonexistent")
    binding["checkpoints_at_pins"] = not s2a_ck_problems and ftp.is_file() and W.sha(ftp) == ev.get("ft_checkpoint_sha256")
    binding["script_equals_eval_commit"] = E1.file_matches_commit(__file__, "tools/cams2b_eval_analyse.py", ev.get("commit"))
    binding["cells_differ_FT45_vs_FT"] = differ(cells, "FT45", "FT", blks)
    if not all(binding.values()):
        res = {"analysis": "CAM-S2b-FT-E", "reading": "EVAL_BINDING_FAIL", "binding": binding, "a60_binding": ab,
               "a60_source": source, "cell_problems": problems}
        write(out, res)
        print(json.dumps(res, indent=1, default=str))
        return 1
    ab["cells_differ_FT_vs_A60"] = differ(cells, "FT", "A60", blks) if cells["A60"] else False
    a60_ok = source is not None and all(ab.values())
    ctrl = controls(k1e)
    rng = random.Random(BOOT_SEED)
    contrasts = {G2: SA.labelled(cells, CONTRASTS[G2], blks, rng)}
    if a60_ok:
        contrasts[G1] = SA.labelled(cells, CONTRASTS[G1], blks, rng)
    labels = {n: {m: c[m]["label"] for m in METRICS} for n, c in contrasts.items()}
    ft45_records = [r for r in ev.get("records") or [] if r.get("arm") == "FT45"]
    telemetry = bool(ft45_records) and all(isinstance(r.get("yawclip_telemetry_last_line"), dict) for r in ft45_records)
    shares = {a: SA.binding_share(diag, a) for a in ARMS}
    if not ctrl["ok"]:
        reading = {"reading": "CONTROL_FAIL", "gate_G1": None, "gate_G2": None}
    else:
        g1 = (gate_g1(labels[G1]["crash"], labels[G1]["capture"]) if a60_ok else
              ("A60_BINDING_FAIL" if source is not None else "G1_NOT_ANSWERED"))
        reading = {"reading": "REPORTED", "gate_G1": g1,
                   "gate_G2": gate_g2(labels[G2]["crash"], labels[G2]["capture"], telemetry, shares["FT45"])}
    pooled = {k: {m: 100.0 * sum(v[m] for v in c.values()) / len(c) for m in METRICS} for k, c in cells.items() if c}
    per_density = {k: {str(d): {m: 100.0 * sum(c[b][m] for b in c if b[0] == d) / sum(1 for b in c if b[0] == d)
                                for m in METRICS} for d in W.DENSITIES if any(b[0] == d for b in c)} for k, c in cells.items() if c}
    pooled_diag = {arm: SA.pooled_diagnostics([v for k, v in diag.items() if k.startswith(arm + "/")])
                   for arm in ARMS if any(k.startswith(arm + "/") for k in diag)}
    res = {"analysis": "CAM-S2b-FT-E", **reading, "labels": labels, "a60_source": source,
           "yaw": {"telemetry_present_in_every_FT45_cell": telemetry, "min_binding_share": MIN_BINDING_SHARE,
                   "binding_share_requested_over_reference": shares,
                   "reference_radps": {a: W.ARMS[a]["diag_ref"] for a in ARMS},
                   "note": "FT and FT45 rows count |action| x 1.0472 over 0.7854 rad/s; A60 rows |action| x 3.0 over 1.047 rad/s. "
                           "YAWCLIP45_TOLERATED admits a capture loss with intervals excluding 0 but a mean under 3 pp."},
           "controls": ctrl, "binding": binding, "a60_binding": ab, "contrasts": contrasts,
           "pooled_rate_pct": pooled, "per_density_rate_pct": per_density, "diagnostics_per_cell": diag,
           "diagnostics_pooled_per_arm": pooled_diag, "speed_governor": {k: C2A.governor(c) for k, c in cells.items() if c},
           "yawclip_telemetry_last_line": {"%s/%d/%d" % (r["arm"], r["density"], r["seed"]): r.get("yawclip_telemetry_last_line")
                                           for r in ev.get("records") or []},
           "camsafe_telemetry_last_line": {"%s/%d/%d" % (r["arm"], r["density"], r["seed"]): r.get("camsafe_telemetry_last_line")
                                           for r in ev.get("records") or []},
           "mismatch_lines": {"%s/%d/%d" % (r["arm"], r["density"], r["seed"]): r.get("mismatch_lines") for r in ev.get("records") or []},
           "ft_training": ft_training_summary(ev.get("ft_train_report_sha256")),
           "blocks": [list(b) for b in blks],
           "bootstrap": {"resamples": A.BOOT, "seed": BOOT_SEED, "scheme": "seeds within density", "t_df": 4,
                         "note": "3 seeds per stratum: at most 10 distinct bootstrap means per stratum; the t-interval is the operative guard"},
           "bands_pp": {"material": SA.A_BAND, "floor": SA.FLOOR, "capture": SA.CAP_BAND, "timeout": SA.TO_BAND},
           "inputs": [str(k1e), str(ident_dir), str(eval_dir), str(a60_dir) if a60_dir else None], "commit": ev.get("commit"),
           "script_sha256": E1.sha(__file__)}
    write(out, res)
    print(json.dumps({"reading": res["reading"], "gate_G1": res["gate_G1"], "gate_G2": res["gate_G2"], "labels": labels,
                      "a60_source": source, "binding_share": shares}, indent=1))
    return 0


def main(argv=None):
    argv = list(argv if argv is not None else sys.argv[1:])
    if argv[:1] == ["controls"] and len(argv) == 3:
        k1e, out = pathlib.Path(argv[1]), pathlib.Path(argv[2])
        if out.exists():
            raise SystemExit("refusing to overwrite " + str(out))
        return main_controls(k1e, out)
    if argv[:1] == ["eval"] and len(argv) == 6:
        k1e, ident_dir, eval_dir = (pathlib.Path(a) for a in argv[1:4])
        a60_dir = None if argv[4] == "-" else pathlib.Path(argv[4])
        out = pathlib.Path(argv[5])
        if out.exists():
            raise SystemExit("refusing to overwrite " + str(out))
        return main_eval(k1e, ident_dir, eval_dir, a60_dir, out)
    raise SystemExit("\n".join(__doc__.strip().splitlines()[-2:]).strip())


if __name__ == "__main__":
    raise SystemExit(main())
