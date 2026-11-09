"""Analysis of CAM-K2-E3 (docs/plans/prereg_camonly_k2_eval_v3_2026-11-10.md): CAM-K2 v3 (trained with the
depth-validity rule, evaluated with it) and CAM-K2 v2 with the rule added at evaluation, against CAM-K2-E's CAM-K2 v2
cells (no rule), all under CAM-K2 v2's treatments on GR-0's S00 grid.

Intervals as CAM-K2-E (tools/camk2e_analyse.py): blocks are (density, seed) pairs, per-block quota rates; a stratified
percentile bootstrap (10,000 resamples, seed 20,261,108, seeds resampled within density) and a stratified t-interval
(df 8 over the 12 blocks). Contrasts in pp; a label needs both intervals to agree.

Gated, crash rate, per seed pair s:
  D_s = V3s - V2s (CAM-K2-E's K2s), gate D (the v3 package against v2):
        UNRESOLVED            the two intervals disagree on whether 0 is excluded, or on the side
        MATERIAL_REDUCTION    both upper ends < -5 pp
        SMALL_REDUCTION       both upper ends < 0 and the mean <= -2 pp
        WORSE                 both lower ends > 0 and the mean >= +2 pp
        NEGLIGIBLE            both intervals inside (-2, +2) pp
        NO_DETECTED_CHANGE    otherwise
        A crash reduction with timeouts up by 2 pp or more reads CRASH_TO_TIMEOUT instead of a REDUCTION.
        (CAM-K2-E's gate A rule, tools/camk2e_analyse.gate_a, with its labels renamed.)
Reported with the same labels, per seed pair s (not gated):
  R_s = V2Rs - V2s  the rule added at evaluation to v2's policy (no retraining; includes v2's train/eval input
                    mismatch);
  T_s = V3s - V2Rs  v3 against v2, both evaluated with the rule: training with the rule, minus that mismatch, plus the
                    variance between two separate training runs (review r1 m-1).
  D_s = R_s + T_s exactly (the same blocks). No causal split beyond this arithmetic is claimed.
Lineage reading per gate: the label both seeds share, else SEEDS_DISAGREE.
Controls (each must read as required, else CONTROL_FAIL and no gate label): CAM-K2-E's five controls on CAM-K1-E's
cells (tools/camk2e_analyse.controls, unchanged: the gate-A rule must read WORSE on F66 - K1s1, not MATERIAL on
K1s1 - K1s2, MATERIAL on K1s1 - F66, and gate N's two), plus a null on CAM-K2-E's own cells: K2s1 - K2s2 under the
treatments must not read MATERIAL_REDUCTION or WORSE; and at the claim's scale (review r1 M-1), on CAM-K2-E's treated
cells: K1s - K2s must read WORSE and K2s - K1s SMALL_ or MATERIAL_REDUCTION, for both seeds.
Reported, not gated: seed spreads (V3s1 - V3s2, V2Rs1 - V2Rs2), capture and timeout contrasts, pooled and per-density
rates, crash causes, speed-governor telemetry, and per cell the rule's summary counts (near / invalid / pixels).
Bindings, checked before any number (EVAL_BINDING_FAIL otherwise): identity IDENTITY_PASS on the eval commit and the
report the eval named; eval COMPLETE with 48 cells in the planned order; the same runner environment and CAM-K2-E
folder; every eval cell passes the runner's checks again (camk2e_run.receipt_problems + camk2v3e_run.validity_problems)
with its checkpoint SHA; every eval cell's result condition equals CAM-K2-E's K2s cell of the same block, and its NAVRL
snapshot differs from it only in SNAPSHOT_EXPECTED (run ids, checkpoint, receipts, the rule's keys); complete grids;
in every block V3s differs from V2s and from V2Rs, V2Rs from V2s, and V3s1 from V3s2; the identity cells
V2Rs1_negative and V3s1_smoke equal the eval's V2Rs1 and V3s1 cells (205 bars, seed 4501); CAM-K2-E's reference cells at
its committed manifest's SHA-256; the checkpoints at their pins (camk2v3e_run.checkpoints); this script's bytes equal
to the eval commit's.
usage: tools/camk2v3e_analyse.py eval CAMK1E_RAW CAMK2E_RAW IDENTITY_DIR EVAL_DIR OUT_JSON
"""
import json
import pathlib
import random
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import camk2e_analyse as A2                                  # noqa: E402  (intervals, gate rule, controls)
import camk2v3e_run as V                                     # noqa: E402  (arms, plan, cell checks, checkpoints)

E1, C2A, A, K, R2 = A2.E1, A2.C2A, A2.A, A2.K, A2.R2
ROOT = pathlib.Path(__file__).resolve().parents[1]
BOOT_SEED = A2.BOOT_SEED
CAMK2E_MANIFEST = ROOT / "docs/evidence/camonly_2026-11-04/camk2e/SHA256_MANIFEST.json"
METRICS = A2.METRICS
CELL = V.CELL
RENAME = {"MATERIAL_ADAPTATION": "MATERIAL_REDUCTION", "SMALL_ADAPTATION": "SMALL_REDUCTION",
          "NO_DETECTED_ADAPTATION": "NO_DETECTED_CHANGE"}
SNAPSHOT_EXPECTED = {"NAVRL_BULK_EVAL_JSON", "NAVRL_EVAL_RUN_NONCE", "NAVRL_KROC2027_RUN_ID", "NAVRL_EVAL_CHECKPOINT",
                     "NAVRL_REALVEH_RECEIPT"} | set(V.VALIDITY_KEYS)


def gate_d(boot, t, mean, timeout_mean=0.0):
    label = A2.gate_a(boot, t, mean, timeout_mean)
    return RENAME.get(label, label)


def load_reference(k2e, blocks):
    """CAM-K2-E's eval K2s1 / K2s2 cells as V2s1 / V2s2."""
    cells = {}
    for key, arm in (("V2s1", "K2s1"), ("V2s2", "K2s2")):
        cells[key] = {b: {"dir": k2e / "eval_01" / arm / (CELL % b), **A.rates(k2e / "eval_01" / arm / (CELL % b))}
                      for b in blocks}
    return cells


def camk2e_manifest_ok(k2e, blocks, arms=("K2s1", "K2s2")):
    """CAM-K2-E's cells of `arms` at its committed manifest (review r2 m2-1: the controls' K1s1 / K1s2 cells too)."""
    rels = ["raw/eval_01/%s/%s/%s" % (arm, CELL % b, f) for arm in arms for b in blocks
            for f in ("episodes.jsonl", "result.json", "receipt.json")]
    return E1.manifest_ok(CAMK2E_MANIFEST, k2e.parent, rels, "files")


def load_eval(ident_dir, eval_dir, k2e, blocks):
    ident = json.loads((ident_dir / "identity_report.json").read_text())
    ev = json.loads((eval_dir / "eval_run_report.json").read_text())
    plan = V.eval_plan()
    binding = {"identity_pass": ident.get("verdict") == "IDENTITY_PASS",
               "identity_same_commit": ident.get("commit") == ev.get("commit"),
               "identity_is_the_one_eval_named": str((ident_dir / "identity_report.json").resolve())
               == str(pathlib.Path(ev.get("identity_report") or "/nonexistent").resolve()),
               "eval_complete": ev.get("verdict") == "COMPLETE" and ev.get("cells_expected") == len(plan) == 48,
               "env_allowlisted_same": bool(ident.get("env_allowlisted"))
               and ident.get("env_allowlisted") == ev.get("env_allowlisted"),
               "camk2e_raw_same": ident.get("camk2e_raw") == ev.get("camk2e_raw") == str(k2e.resolve()),
               "order_as_planned": [(r.get("arm"), r.get("density"), r.get("seed")) for r in ev.get("records", [])]
               == [tuple(x) for x in plan]}
    committed = ev.get("committed_sha256") or {}
    vsha = ev.get("validity_sha256")
    ck = ev.get("checkpoints") or {}
    cells = {a: {} for a in V.EVAL_ARMS}
    receipts_ok, cond_ok, snap_ok = True, True, True
    for r in ev.get("records", []):
        arm = r.get("arm")
        if arm not in cells:
            receipts_ok = False
            continue
        d = eval_dir / arm / r["cell_id"]
        blk = (r["density"], r["seed"])
        cells[arm][blk] = {"dir": d, **A.rates(d)}
        try:
            want_ck = V.arm_ckpt(arm, {k: tuple(v) for k, v in ck.items()})
            receipt = json.loads((d / "receipt.json").read_text())
            receipts_ok &= (not R2.receipt_problems(d, "K2s1", r["seed"], committed)
                            and not V.validity_problems(d, True, vsha)
                            and receipt.get("checkpoint_sha256") == want_ck[1] == r.get("checkpoint_sha256"))
            ref = k2e / "eval_01" / ("K2s1" if arm.endswith("s1") else "K2s2") / r["cell_id"]
            cond_ok &= K.comparable(json.loads((d / "result.json").read_text()))["condition"] == \
                K.comparable(json.loads((ref / "result.json").read_text()))["condition"]
            snap_ok &= snapshot_ok(d, ref)
        except (OSError, ValueError, KeyError, TypeError):
            receipts_ok = False
    binding["cells_pass_the_runner_checks_again"] = receipts_ok
    binding["conditions_equal_camk2e"] = cond_ok
    binding["snapshots_differ_from_camk2e_only_in_expected_keys"] = snap_ok
    for arm in V.EVAL_ARMS:
        binding["complete_grid_" + arm] = sorted(cells[arm]) == blocks
    for ident_arm, eval_arm in (("V2Rs1_negative", "V2Rs1"), ("V3s1_smoke", "V3s1")):
        a = ident_dir / ident_arm / (CELL % (205, 4501)) / "episodes.jsonl"
        b = eval_dir / eval_arm / (CELL % (205, 4501)) / "episodes.jsonl"
        binding["identity_%s_equals_eval_%s" % (ident_arm, eval_arm)] = a.is_file() and b.is_file() and \
            K.read_jsonl(a) == K.read_jsonl(b)
    return binding, cells, ev


def snapshot_ok(cell_dir, ref_dir):
    """The cell's NAVRL snapshot differs from the reference cell's only in SNAPSHOT_EXPECTED (review r1 m-2)."""
    a, b = E1.snapshot(cell_dir), E1.snapshot(ref_dir)
    return {k for k in set(a) | set(b) if a.get(k) != b.get(k)} <= SNAPSHOT_EXPECTED


def load_camk2e_k1(k2e, blocks):
    """CAM-K2-E's treated CAM-K1 cells (for the claim-scale controls)."""
    return {"K2E_K1s%d" % s: {b: {"dir": k2e / "eval_01" / ("K1s%d" % s) / (CELL % b),
                                  **A.rates(k2e / "eval_01" / ("K1s%d" % s) / (CELL % b))} for b in blocks}
            for s in (1, 2)}


def claim_scale_controls(cells, blocks):
    """Review r1 M-1: controls at the claim's scale on CAM-K2-E's treated cells (2-5 pp differences): K1s - K2s must read
    WORSE and K2s - K1s must read SMALL_REDUCTION or MATERIAL_REDUCTION, for both seeds. Fixed order, one generator."""
    rng = random.Random(BOOT_SEED)
    out = {}
    for s in (1, 2):
        for name, w, ok in (("D_on_CAMK2E_K1s%d_minus_K2s%d" % (s, s), {"K2E_K1s%d" % s: 1, "V2s%d" % s: -1},
                             lambda lab: lab == "WORSE"),
                            ("D_on_CAMK2E_K2s%d_minus_K1s%d" % (s, s), {"V2s%d" % s: 1, "K2E_K1s%d" % s: -1},
                             lambda lab: lab in ("SMALL_REDUCTION", "MATERIAL_REDUCTION"))):
            c = {m: A2.contrast(cells, w, blocks, m, rng) for m in METRICS}
            label = gate_d(c["crash"]["ci95_pp"], c["crash"]["t95_pp"], c["crash"]["mean_pp"],
                           c["timeout"]["mean_pp"])
            out[name] = {"gate": "D", "label": label, "ok": ok(label), "crash": c["crash"],
                         "required": "WORSE" if "K1s%d_minus" % s in name else "SMALL_REDUCTION or MATERIAL_REDUCTION"}
    return out


def null_control(cells, blocks):
    rng = random.Random(BOOT_SEED)
    c = {m: A2.contrast(cells, {"V2s1": 1, "V2s2": -1}, blocks, m, rng) for m in METRICS}
    label = gate_d(c["crash"]["ci95_pp"], c["crash"]["t95_pp"], c["crash"]["mean_pp"], c["timeout"]["mean_pp"])
    cr = c["crash"]
    return {"gate": "D", "label": label, "required": "not MATERIAL_REDUCTION and not WORSE",
            "ok": label not in ("MATERIAL_REDUCTION", "WORSE"), "crash": cr,
            "half_width_pp": {"bootstrap": (cr["ci95_pp"][1] - cr["ci95_pp"][0]) / 2,
                              "t": (cr["t95_pp"][1] - cr["t95_pp"][0]) / 2}}


def write(out, res):
    with out.open("x") as fh:
        json.dump(res, fh, indent=1, default=str)
        fh.write("\n")


def main_eval(k1e, k2e, ident_dir, eval_dir, out):
    blocks = [(d, s) for d in R2.G.DENSITIES for s in R2.G.SEEDS]
    nominal_blocks = [(d, s) for d in R2.NOMINAL_DENSITIES for s in R2.G.SEEDS]
    cells = load_reference(k2e, blocks)
    binding, ev_cells, ev = load_eval(ident_dir, eval_dir, k2e, blocks)
    cells.update(ev_cells)
    binding["camk2e_cells_match_manifest"] = camk2e_manifest_ok(k2e, blocks)
    _ck, ck_problems = V.checkpoints()
    binding["checkpoints_trained_with_manifests"] = not ck_problems
    binding["script_equals_eval_commit"] = E1.file_matches_commit(__file__, "tools/camk2v3e_analyse.py",
                                                                  ev.get("commit"))
    diffs = {}
    for s in (1, 2):
        diffs["V3s%d_vs_V2s%d" % (s, s)] = A2.differ(cells, "V3s%d" % s, "V2s%d" % s, blocks)
        diffs["V2Rs%d_vs_V2s%d" % (s, s)] = A2.differ(cells, "V2Rs%d" % s, "V2s%d" % s, blocks)
        diffs["V3s%d_vs_V2Rs%d" % (s, s)] = A2.differ(cells, "V3s%d" % s, "V2Rs%d" % s, blocks)
    diffs["V3s1_vs_V3s2"] = A2.differ(cells, "V3s1", "V3s2", blocks)
    binding["cells_differ"] = all(diffs.values())
    binding["cells_differ_detail"] = diffs
    if not all(v for k, v in binding.items() if k != "cells_differ_detail"):
        res = {"analysis": "CAM-K2-E3", "reading": "EVAL_BINDING_FAIL", "binding": binding}
        write(out, res)
        print(json.dumps(res, indent=1, default=str))
        return 1
    k1cells = A2.load_camk1e(k1e, blocks)
    ctrl = A2.controls(k1cells, blocks, nominal_blocks)
    ctrl["D_on_CAMK2E_K2s1_minus_K2s2"] = null_control(cells, blocks)
    ctrl.update(claim_scale_controls({**cells, **load_camk2e_k1(k2e, blocks)}, blocks))
    ctrl_ok = (all(c["ok"] for c in ctrl.values()) and A2.camk1e_manifest_ok(k1e, blocks)
               and camk2e_manifest_ok(k2e, blocks, arms=("K1s1", "K1s2")))
    rng = random.Random(BOOT_SEED)
    defs = {}
    for s in (1, 2):
        defs["D_s%d" % s] = {"V3s%d" % s: 1, "V2s%d" % s: -1}
        defs["R_s%d" % s] = {"V2Rs%d" % s: 1, "V2s%d" % s: -1}
        defs["T_s%d" % s] = {"V3s%d" % s: 1, "V2Rs%d" % s: -1}
    defs["seed_V3s1_minus_V3s2"] = {"V3s1": 1, "V3s2": -1}
    defs["seed_V2Rs1_minus_V2Rs2"] = {"V2Rs1": 1, "V2Rs2": -1}
    contrasts = {}
    for name, w in defs.items():
        contrasts[name] = {}
        for metric in METRICS:
            c = A2.contrast(cells, w, blocks, metric, rng)
            c["label_c2_scheme"] = C2A.label(c["ci95_pp"], c["t95_pp"], c["mean_pp"],
                                             A2.CAP_BAND if metric == "capture" else 2.0)
            contrasts[name][metric] = c
    labels = {}
    for name in [k for k in defs if k[0] in "DRT"]:
        cr, to = contrasts[name]["crash"], contrasts[name]["timeout"]["mean_pp"]
        labels[name] = gate_d(cr["ci95_pp"], cr["t95_pp"], cr["mean_pp"], to)
    if not ctrl_ok:
        reading = {"reading": "CONTROL_FAIL", "gate_D": None, "R": None, "T": None}
    else:
        reading = {"reading": "REPORTED", "gate_D": E1.lineage(labels["D_s1"], labels["D_s2"]),
                   "R": E1.lineage(labels["R_s1"], labels["R_s2"]), "T": E1.lineage(labels["T_s1"], labels["T_s2"])}
    pooled = {k: {m: 100.0 * sum(v[m] for v in c.values()) / len(c) for m in METRICS} for k, c in cells.items()}
    per_density = {k: {str(d): {m: 100.0 * sum(c[b][m] for b in c if b[0] == d) / sum(1 for b in c if b[0] == d)
                                for m in METRICS} for d in sorted({b[0] for b in c})} for k, c in cells.items()}
    validity = {"%s/%d/%d" % (r["arm"], r["density"], r["seed"]): r.get("validity_summary")
                for r in ev.get("records", [])}
    res = {"analysis": "CAM-K2-E3", **reading, "labels_per_seed": labels, "controls": ctrl,
           "controls_ok": ctrl_ok, "binding": binding, "contrasts": contrasts, "pooled_rate_pct": pooled,
           "per_density_rate_pct": per_density,
           "crash_causes_legacy_window": {k: C2A.causes(c) for k, c in cells.items()},
           "speed_governor": {k: C2A.governor(c) for k, c in cells.items()},
           "validity_summary_per_cell": validity, "blocks": [list(b) for b in blocks],
           "bootstrap": {"resamples": A.BOOT, "seed": BOOT_SEED, "scheme": "seeds within density", "t_df": 8},
           "bands_pp": {"D": A2.A_BAND, "floor": A2.FLOOR, "capture_c2_scheme": A2.CAP_BAND},
           "inputs": [str(k1e), str(k2e), str(ident_dir), str(eval_dir)], "commit": ev.get("commit"),
           "script_sha256": E1.sha(__file__)}
    write(out, res)
    print(json.dumps({"reading": res["reading"], "gate_D": res["gate_D"], "R": res["R"], "T": res["T"],
                      "labels_per_seed": labels, "controls": {k: v["label"] for k, v in ctrl.items()}}, indent=1))
    return 0


def main(argv=None):
    argv = list(argv if argv is not None else sys.argv[1:])
    if argv[:1] == ["eval"] and len(argv) == 6:
        k1e, k2e, ident_dir, eval_dir, out = (pathlib.Path(a) for a in argv[1:6])
        if out.exists():
            raise SystemExit("refusing to overwrite " + str(out))
        return main_eval(k1e, k2e, ident_dir, eval_dir, out)
    raise SystemExit(__doc__.strip().splitlines()[-1].strip())


if __name__ == "__main__":
    raise SystemExit(main())
