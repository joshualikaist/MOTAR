"""Analysis of CAM-K2-E (docs/plans/prereg_camonly_k2_eval_v1_2026-11-08.md): CAM-K2 seeds 1 and 2 against CAM-K1
seeds 1 and 2 under CAM-K2's treatments, and CAM-K2 with the treatments off against CAM-K1-E's CAM-K1 cells.

GR-0's S00 grid (E2, A3, 70/115/160/205 bars x seeds 4501-4503, quota 16). Blocks are (density, seed) pairs; per-block
quota rates. The densities are fixed, so intervals resample seeds within each density (as CAM-K1-E,
tools/camk1e_analyse.py): a stratified percentile bootstrap (10,000 resamples, seed 20,261,108) and a stratified
t-interval with df = sum over densities of (seeds - 1): 8 df over the 12 main blocks, 4 df over the 6 nominal blocks.
Contrasts are in pp. Labels need both intervals to agree.

Gated, crash rate, per seed pair s:
  A_s = K2s - K1s, both under the treatments (12 blocks), gate A (adaptation):
        UNRESOLVED              the two intervals disagree on whether 0 is excluded, or on the side
        MATERIAL_ADAPTATION     both upper ends < -5 pp
        SMALL_ADAPTATION        both upper ends < 0 and the mean <= -2 pp
        WORSE                   both lower ends > 0 and the mean >= +2 pp
        NEGLIGIBLE              both intervals inside (-2, +2) pp
        NO_DETECTED_ADAPTATION  otherwise
        A crash reduction with timeouts up by 2 pp or more (the timeout contrast's mean) reads CRASH_TO_TIMEOUT
        instead of MATERIAL_ADAPTATION or SMALL_ADAPTATION.
Reported with its label, crash rate, per seed pair s:
  N_s = K2s_nom - K1s(CAM-K1-E) on the 6 nominal blocks (70 and 205 bars x 3 seeds; df 4), gate N (no nominal loss):
        NO_MATERIAL_LOSS   both upper ends < +3 pp
        MATERIAL_LOSS      both lower ends > +3 pp
        LOSS_INCONCLUSIVE  otherwise
  "Nominal" (treatments OFF) is F's vehicle: navrl_quad's own T/W 3.26, tau 0.04 s and exact depth, a weaker vehicle
  than the trained one (review r1 E-m1).
  I_s = (K2s - K1s)_ON - (K2s_nom - K1s(CAM-K1-E))_OFF on the 6 nominal blocks (df 4), gate I (interaction; review r1
  E-M1), weights {K2s: +1, K1s: -1, K2s_nom: -1, K1E_K1s: +1}:
        INTERACTION_UNRESOLVED   the two intervals disagree on whether 0 is excluded, or on the side
        INTERACTION_NEGATIVE     both upper ends < 0 (K2s gains more under the treatments than nominally)
        INTERACTION_POSITIVE     both lower ends > 0
        INTERACTION_NOT_DETECTED otherwise
  The word "adaptation" is allowed for seed s only when gate A reads MATERIAL_ADAPTATION or SMALL_ADAPTATION and gate
  I reads INTERACTION_NEGATIVE (adaptation_supported); otherwise gate A reads "lower crash under the treatments",
  which may be continued training alone.
  Lineage reading per gate: the label both seeds share, else SEEDS_DISAGREE.
Controls on existing data (CAM-K1-E's cells), each with its own generator seeded 20,261,108 so that the `controls`
mode and the full analysis give the same numbers; any miss reads CONTROL_FAIL and no gate label is reported:
  gate A on CAM-K1-E's F66 - K1s1 must read WORSE (a known-bad "adaptation");
  gate A on CAM-K1-E's K1s1 - K1s2 must not read MATERIAL_ADAPTATION;
  review r1 E-M2, the passing labels are reachable on these block counts:
  gate A on CAM-K1-E's K1s1 - F66 (12 blocks) must read MATERIAL_ADAPTATION;
  gate N on CAM-K1-E's K1s2 - K1s1 (the 6 nominal blocks, df 4) must read NO_MATERIAL_LOSS;
  gate N on CAM-K1-E's F66 - K1s1 (the 6 nominal blocks) must read MATERIAL_LOSS.
  The null controls' interval half-widths are reported as the resolution.
Reported, not gated: the treatments' cost to CAM-K1 (K1s under the treatments - CAM-K1-E's K1s), the seed spreads
(K2s1 - K2s2, K1s1 - K1s2 under the treatments, K2s1_nom - K2s2_nom), the same contrasts for capture and timeout,
pooled and per-density rates, crash causes (result.json's legacy window: all actual episodes, not the quota rows) and
the speed governor's telemetry.
Bindings, checked before any number (any failure reads EVAL_BINDING_FAIL): identity IDENTITY_PASS on the eval commit and
the identity report the eval named; eval COMPLETE with 60 cells; the same runner environment and CAM-K1-E folder; every
eval cell passes tools/camk2e_run.receipt_problems again (realveh receipt, snapshot treatment keys, camera, telemetry,
record) with the run report's committed SHA-256, and its checkpoint SHA is the run report's; every eval cell's result
condition equals CAM-K1-E's K1s1 cell of the same block, and its NAVRL snapshot differs from that cell's only in
camk2e_run.SNAPSHOT_EXPECTED; complete grids; in every block K2s differs from K1s, K1s under the treatments from
CAM-K1-E's K1s, K2s_nom from CAM-K1-E's K1s and K2s1 from K2s2; the identity cells K1s1_on_negative and K2s1_smoke equal
the eval's K1s1 and K2s1 cells (205 bars, seed 4501); CAM-K1-E's cells at its committed manifest's SHA-256; the CAM-K2
checkpoints at TRAINED train reports and sidecar manifests (camk2e_run.checkpoints); this script's bytes equal to the
eval commit's.
usage: tools/camk2e_analyse.py eval CAMK1E_RAW IDENTITY_DIR EVAL_DIR OUT_JSON
       tools/camk2e_analyse.py controls CAMK1E_RAW OUT_JSON
"""
import json
import math
import pathlib
import random
import statistics
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import camk1e_analyse as E1                                  # noqa: E402  (intervals, gate P, bindings helpers)
import camk2e_run as R2                                      # noqa: E402  (arms, plan, receipt checks, checkpoints)
import camonly_c2_analyse as C2A                             # noqa: E402
import gr_analyse as A                                       # noqa: E402
import kroc2027_eval as K                                    # noqa: E402

ROOT = pathlib.Path(__file__).resolve().parents[1]
BOOT_SEED = E1.BOOT_SEED
CAMK1E_MANIFEST = ROOT / "docs/evidence/camonly_2026-11-04/camk1e/SHA256_MANIFEST.json"
A_BAND, N_BAND, FLOOR = 5.0, 3.0, 2.0
CAP_BAND = 3.0
T_CRIT = {4: 2.776445105, 8: E1.T_CRIT_8}                     # t 0.975 quantiles; df 8 is CAM-K1-E's constant
CELL = R2.CELL
METRICS = ("crash", "capture", "timeout")


def gate_a(boot, t, mean, timeout_mean=0.0):
    if C2A.side(boot) != C2A.side(t):
        return "UNRESOLVED"
    if E1.both(boot, t, lambda ci: ci[1] < -A_BAND):
        return "CRASH_TO_TIMEOUT" if timeout_mean >= FLOOR else "MATERIAL_ADAPTATION"
    if E1.both(boot, t, lambda ci: ci[1] < 0) and mean <= -FLOOR:
        return "CRASH_TO_TIMEOUT" if timeout_mean >= FLOOR else "SMALL_ADAPTATION"
    if E1.both(boot, t, lambda ci: ci[0] > 0) and mean >= FLOOR:
        return "WORSE"
    if E1.both(boot, t, lambda ci: -FLOOR < ci[0] and ci[1] < FLOOR):
        return "NEGLIGIBLE"
    return "NO_DETECTED_ADAPTATION"


def gate_i(boot, t):
    """The interaction label (review r1 E-M1)."""
    if C2A.side(boot) != C2A.side(t):
        return "INTERACTION_UNRESOLVED"
    if E1.both(boot, t, lambda ci: ci[1] < 0):
        return "INTERACTION_NEGATIVE"
    if E1.both(boot, t, lambda ci: ci[0] > 0):
        return "INTERACTION_POSITIVE"
    return "INTERACTION_NOT_DETECTED"


def adaptation_supported(gate_a_label, gate_i_label):
    return gate_a_label in ("MATERIAL_ADAPTATION", "SMALL_ADAPTATION") and gate_i_label == "INTERACTION_NEGATIVE"


def gate_n(boot, t):
    """CAM-K1-E's gate P rule with its 3 pp band (tools/camk1e_analyse.gate_p)."""
    assert E1.P_BAND == N_BAND
    return E1.gate_p(boot, t)


def strat_t(xs, blocks):
    """Stratified t-interval over equal-size strata, df = sum_d (n_d - 1) (8 for 12 blocks, 4 for 6)."""
    by = {}
    for x, (d, _s) in zip(xs, blocks):
        by.setdefault(d, []).append(x)
    groups = [by[d] for d in sorted(by)]
    df = sum(len(g) - 1 for g in groups)
    m = sum(xs) / len(xs)
    se = math.sqrt(sum(statistics.variance(g) / len(g) for g in groups) / len(groups) ** 2)
    return [m - T_CRIT[df] * se, m + T_CRIT[df] * se], df


def contrast(cells, weights, blocks, metric, rng):
    xs = [100.0 * sum(w * cells[k][b][metric] for k, w in weights.items()) for b in blocks]
    t, df = strat_t(xs, blocks)
    return {"mean_pp": sum(xs) / len(xs), "ci95_pp": E1.strat_ci(xs, blocks, rng), "t95_pp": t, "t_df": df,
            "blocks_pp": xs, "blocks": [list(b) for b in blocks]}


def load_camk1e(k1e, blocks):
    cells = {}
    for key, arm in (("K1E_F66", "F66"), ("K1E_K1s1", "K1s1"), ("K1E_K1s2", "K1s2")):
        cells[key] = {b: {"dir": k1e / "eval_01" / arm / (CELL % b), **A.rates(k1e / "eval_01" / arm / (CELL % b))}
                      for b in blocks}
    return cells


def camk1e_manifest_ok(k1e, blocks):
    rels = ["raw/eval_01/%s/%s/%s" % (arm, CELL % b, f) for arm in ("F66", "K1s1", "K1s2") for b in blocks
            for f in ("episodes.jsonl", "result.json", "receipt.json")]
    return E1.manifest_ok(CAMK1E_MANIFEST, k1e.parent, rels, "files")


def controls(cells, blocks, nominal_blocks):
    """Gate A on CAM-K1-E's F66 - K1s1 (must be WORSE) and K1s1 - K1s2 (must not be MATERIAL_ADAPTATION); review r1
    E-M2: gate A on K1s1 - F66 (must be MATERIAL_ADAPTATION), gate N on K1s2 - K1s1 (must be NO_MATERIAL_LOSS) and on
    F66 - K1s1 (must be MATERIAL_LOSS), both on the 6 nominal blocks. The order is fixed (one generator)."""
    rng = random.Random(BOOT_SEED)
    out = {}
    for name, w, blks, gate, required in (
            ("A_on_K1E_F66_minus_K1s1", {"K1E_F66": 1, "K1E_K1s1": -1}, blocks, "A", "WORSE"),
            ("A_on_K1E_K1s1_minus_K1s2", {"K1E_K1s1": 1, "K1E_K1s2": -1}, blocks, "A", "not MATERIAL_ADAPTATION"),
            ("A_on_K1E_K1s1_minus_F66", {"K1E_K1s1": 1, "K1E_F66": -1}, blocks, "A", "MATERIAL_ADAPTATION"),
            ("N_on_K1E_K1s2_minus_K1s1", {"K1E_K1s2": 1, "K1E_K1s1": -1}, nominal_blocks, "N", "NO_MATERIAL_LOSS"),
            ("N_on_K1E_F66_minus_K1s1", {"K1E_F66": 1, "K1E_K1s1": -1}, nominal_blocks, "N", "MATERIAL_LOSS")):
        c = {m: contrast(cells, w, blks, m, rng) for m in METRICS}
        if gate == "A":
            label = gate_a(c["crash"]["ci95_pp"], c["crash"]["t95_pp"], c["crash"]["mean_pp"],
                           c["timeout"]["mean_pp"])
        else:
            label = gate_n(c["crash"]["ci95_pp"], c["crash"]["t95_pp"])
        ok = label == required if not required.startswith("not ") else label != required[4:]
        out[name] = {"gate": gate, "label": label, "required": required, "ok": ok, "crash": c["crash"],
                     "timeout_mean_pp": c["timeout"]["mean_pp"], "blocks": len(blks)}
    for name in ("A_on_K1E_K1s1_minus_K1s2", "N_on_K1E_K1s2_minus_K1s1"):      # the nulls' resolution
        cr = out[name]["crash"]
        out[name]["half_width_pp"] = {"bootstrap": (cr["ci95_pp"][1] - cr["ci95_pp"][0]) / 2,
                                      "t": (cr["t95_pp"][1] - cr["t95_pp"][0]) / 2}
    return out


def load_eval(ident_dir, eval_dir, k1e, main_blocks, nominal_blocks):
    ident = json.loads((ident_dir / "identity_report.json").read_text())
    ev = json.loads((eval_dir / "eval_run_report.json").read_text())
    plan = R2.eval_plan()
    binding = {"identity_pass": ident.get("verdict") == "IDENTITY_PASS",
               "identity_same_commit": ident.get("commit") == ev.get("commit"),
               "identity_is_the_one_eval_named": str((ident_dir / "identity_report.json").resolve())
               == str(pathlib.Path(ev.get("identity_report") or "/nonexistent").resolve()),
               "eval_complete": ev.get("verdict") == "COMPLETE" and ev.get("cells_expected") == len(plan) == 60,
               "env_allowlisted_same": bool(ident.get("env_allowlisted"))
               and ident.get("env_allowlisted") == ev.get("env_allowlisted"),
               "camk1e_raw_same": ident.get("camk1e_raw") == ev.get("camk1e_raw") == str(k1e.resolve()),
               "order_as_planned": [(r.get("arm"), r.get("density"), r.get("seed")) for r in ev.get("records", [])]
               == [tuple(x) for x in plan]}
    committed = ev.get("committed_sha256") or {}
    ck = ev.get("checkpoints") or {}
    cells = {a: {} for a in R2.MAIN_ARMS + R2.NOMINAL_ARMS}
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
            want_ck = R2.arm_ckpt(arm, {k: tuple(v) for k, v in ck.items()})
            receipt = json.loads((d / "receipt.json").read_text())
            receipts_ok &= (not R2.receipt_problems(d, arm, r["seed"], committed)
                            and receipt.get("checkpoint_sha256") == want_ck[1] == r.get("checkpoint_sha256"))
            ref = k1e / "eval_01" / ("K1s1" if arm.endswith("s1") or arm.endswith("s1_nom") else "K1s2") / r["cell_id"]
            cond_ok &= K.comparable(json.loads((d / "result.json").read_text()))["condition"] == \
                K.comparable(json.loads((ref / "result.json").read_text()))["condition"]
            a_snap, b_snap = E1.snapshot(d), E1.snapshot(ref)
            snap_ok &= {k for k in set(a_snap) | set(b_snap) if a_snap.get(k) != b_snap.get(k)} <= R2.SNAPSHOT_EXPECTED
        except (OSError, ValueError, KeyError, TypeError):
            receipts_ok = False
    binding["cells_pass_the_runner_checks_again"] = receipts_ok
    binding["conditions_equal_camk1e"] = cond_ok
    binding["snapshots_differ_from_camk1e_only_in_expected_keys"] = snap_ok
    for arm in R2.MAIN_ARMS:
        binding["complete_grid_" + arm] = sorted(cells[arm]) == main_blocks
    for arm in R2.NOMINAL_ARMS:
        binding["complete_grid_" + arm] = sorted(cells[arm]) == nominal_blocks
    for ident_arm, eval_arm in (("K1s1_on_negative", "K1s1"), ("K2s1_smoke", "K2s1")):
        a = ident_dir / ident_arm / (CELL % (205, 4501)) / "episodes.jsonl"
        b = eval_dir / eval_arm / (CELL % (205, 4501)) / "episodes.jsonl"
        binding["identity_%s_equals_eval_%s" % (ident_arm, eval_arm)] = a.is_file() and b.is_file() and \
            K.read_jsonl(a) == K.read_jsonl(b)
    return binding, cells, ev


def raise_commands(cells):
    """Review r1 M-1, descriptive: per treated cell, the share of motor commands above navrl_quad's own 2 N and at the
    raised limit, from the last raise-counter line of the cell log (NOT_RECORDED where absent)."""
    out = {}
    for arm in R2.MAIN_ARMS:
        for blk, c in sorted((cells.get(arm) or {}).items()):
            try:
                text = (c["dir"] / "cell.log").read_text(errors="replace")
            except OSError:
                text = ""
            veh = R2.T2.telemetry_lines([text], R2.T2.VR_MOD.LOG_PREFIX)
            lines = [x for x in veh if x.get("event") in ("reset", "update")]
            d = {}
            probs = R2.T2.raise_telemetry_problems([x for x in veh if x.get("event") == "init"], lines, d)
            out["%s/%d/%d" % (arm, blk[0], blk[1])] = d.get("raise") if not probs and d.get("raise") else "NOT_RECORDED"
    return out


def differ(cells, a, b, blocks):
    return all(blk in cells[a] and blk in cells[b] and K.read_jsonl(cells[a][blk]["dir"] / "episodes.jsonl")
               != K.read_jsonl(cells[b][blk]["dir"] / "episodes.jsonl") for blk in blocks)


def write(out, res):
    with out.open("x") as fh:
        json.dump(res, fh, indent=1, default=str)
        fh.write("\n")


def main_controls(k1e, out):
    blocks = [(d, s) for d in R2.G.DENSITIES for s in R2.G.SEEDS]
    nominal_blocks = [(d, s) for d in R2.NOMINAL_DENSITIES for s in R2.G.SEEDS]
    cells = load_camk1e(k1e, blocks)
    res = {"analysis": "CAM-K2-E controls", "camk1e_manifest_ok": camk1e_manifest_ok(k1e, blocks),
           "controls": controls(cells, blocks, nominal_blocks), "inputs": [str(k1e)],
           "script_sha256": E1.sha(__file__)}
    res["reading"] = ("CONTROLS_OK" if res["camk1e_manifest_ok"] and all(c["ok"] for c in res["controls"].values())
                      else "CONTROL_FAIL")
    write(out, res)
    print(json.dumps({"reading": res["reading"], **{k: v["label"] for k, v in res["controls"].items()}}, indent=1))
    return 0 if res["reading"] == "CONTROLS_OK" else 1


def main_eval(k1e, ident_dir, eval_dir, out):
    main_blocks = [(d, s) for d in R2.G.DENSITIES for s in R2.G.SEEDS]
    nominal_blocks = [(d, s) for d in R2.NOMINAL_DENSITIES for s in R2.G.SEEDS]
    cells = load_camk1e(k1e, main_blocks)
    binding, ev_cells, ev = load_eval(ident_dir, eval_dir, k1e, main_blocks, nominal_blocks)
    cells.update(ev_cells)
    binding["camk1e_cells_match_manifest"] = camk1e_manifest_ok(k1e, main_blocks)
    _ck, ck_problems = R2.checkpoints()
    binding["camk2_checkpoints_trained_with_manifests"] = not ck_problems
    binding["script_equals_eval_commit"] = E1.file_matches_commit(__file__, "tools/camk2e_analyse.py", ev.get("commit"))
    diffs = {}
    for s in (1, 2):
        diffs["K2s%d_vs_K1s%d" % (s, s)] = differ(cells, "K2s%d" % s, "K1s%d" % s, main_blocks)
        diffs["K1s%d_vs_K1E_K1s%d" % (s, s)] = differ(cells, "K1s%d" % s, "K1E_K1s%d" % s, main_blocks)
        diffs["K2s%d_nom_vs_K1E_K1s%d" % (s, s)] = differ(cells, "K2s%d_nom" % s, "K1E_K1s%d" % s, nominal_blocks)
    diffs["K2s1_vs_K2s2"] = differ(cells, "K2s1", "K2s2", main_blocks)
    binding["cells_differ"] = all(diffs.values())
    binding["cells_differ_detail"] = diffs
    if not all(v for k, v in binding.items() if k != "cells_differ_detail"):
        res = {"analysis": "CAM-K2-E", "reading": "EVAL_BINDING_FAIL", "binding": binding}
        write(out, res)
        print(json.dumps(res, indent=1, default=str))
        return 1
    ctrl = controls(cells, main_blocks, nominal_blocks)
    rng = random.Random(BOOT_SEED)
    defs = {"A_K2s1": ({"K2s1": 1, "K1s1": -1}, main_blocks), "A_K2s2": ({"K2s2": 1, "K1s2": -1}, main_blocks),
            "N_K2s1": ({"K2s1_nom": 1, "K1E_K1s1": -1}, nominal_blocks),
            "N_K2s2": ({"K2s2_nom": 1, "K1E_K1s2": -1}, nominal_blocks),
            "cost_K1s1": ({"K1s1": 1, "K1E_K1s1": -1}, main_blocks),
            "cost_K1s2": ({"K1s2": 1, "K1E_K1s2": -1}, main_blocks),
            "seed_K2s1_minus_K2s2": ({"K2s1": 1, "K2s2": -1}, main_blocks),
            "seed_K1s1_minus_K1s2_treated": ({"K1s1": 1, "K1s2": -1}, main_blocks),
            "seed_K2s1_nom_minus_K2s2_nom": ({"K2s1_nom": 1, "K2s2_nom": -1}, nominal_blocks),
            "I_K2s1": ({"K2s1": 1, "K1s1": -1, "K2s1_nom": -1, "K1E_K1s1": 1}, nominal_blocks),   # review r1 E-M1
            "I_K2s2": ({"K2s2": 1, "K1s2": -1, "K2s2_nom": -1, "K1E_K1s2": 1}, nominal_blocks)}
    contrasts = {}
    for name, (w, blocks) in defs.items():
        contrasts[name] = {}
        for metric in METRICS:
            c = contrast(cells, w, blocks, metric, rng)
            c["label_c2_scheme"] = C2A.label(c["ci95_pp"], c["t95_pp"], c["mean_pp"],
                                             CAP_BAND if metric == "capture" else 2.0)
            contrasts[name][metric] = c
    cr = {k: contrasts[k]["crash"] for k in defs}
    to = {k: contrasts[k]["timeout"]["mean_pp"] for k in defs}
    gates = {"A_K2s1": gate_a(cr["A_K2s1"]["ci95_pp"], cr["A_K2s1"]["t95_pp"], cr["A_K2s1"]["mean_pp"], to["A_K2s1"]),
             "A_K2s2": gate_a(cr["A_K2s2"]["ci95_pp"], cr["A_K2s2"]["t95_pp"], cr["A_K2s2"]["mean_pp"], to["A_K2s2"]),
             "N_K2s1": gate_n(cr["N_K2s1"]["ci95_pp"], cr["N_K2s1"]["t95_pp"]),
             "N_K2s2": gate_n(cr["N_K2s2"]["ci95_pp"], cr["N_K2s2"]["t95_pp"]),
             "I_K2s1": gate_i(cr["I_K2s1"]["ci95_pp"], cr["I_K2s1"]["t95_pp"]),
             "I_K2s2": gate_i(cr["I_K2s2"]["ci95_pp"], cr["I_K2s2"]["t95_pp"])}
    supported = {"K2s%d" % s: adaptation_supported(gates["A_K2s%d" % s], gates["I_K2s%d" % s]) for s in (1, 2)}
    if not all(c["ok"] for c in ctrl.values()):
        reading = {"reading": "CONTROL_FAIL", "gate_A": None, "gate_N": None, "gate_I": None,
                   "adaptation_supported": None}
    else:
        reading = {"reading": "REPORTED", "gate_A": E1.lineage(gates["A_K2s1"], gates["A_K2s2"]),
                   "gate_N": E1.lineage(gates["N_K2s1"], gates["N_K2s2"]),
                   "gate_I": E1.lineage(gates["I_K2s1"], gates["I_K2s2"]),
                   "adaptation_supported": all(supported.values())}
    pooled = {k: {m: 100.0 * sum(v[m] for v in c.values()) / len(c) for m in METRICS} for k, c in cells.items()}
    per_density = {k: {str(d): {m: 100.0 * sum(c[b][m] for b in c if b[0] == d) / sum(1 for b in c if b[0] == d)
                                for m in METRICS} for d in sorted({b[0] for b in c})} for k, c in cells.items()}
    res = {"analysis": "CAM-K2-E", **reading, "gates_per_seed": gates, "adaptation_supported_per_seed": supported,
           "controls": ctrl, "binding": binding,
           "raise_commands_per_cell": raise_commands(cells),
           "contrasts": contrasts, "pooled_rate_pct": pooled, "per_density_rate_pct": per_density,
           "crash_causes_legacy_window": {k: C2A.causes(c) for k, c in cells.items()},
           "speed_governor": {k: C2A.governor(c) for k, c in cells.items()},
           "blocks": {"main": [list(b) for b in main_blocks], "nominal": [list(b) for b in nominal_blocks]},
           "bootstrap": {"resamples": A.BOOT, "seed": BOOT_SEED, "scheme": "seeds within density",
                         "t_df": {"main": 8, "nominal": 4}},
           "bands_pp": {"A": A_BAND, "N": N_BAND, "floor": FLOOR, "capture_c2_scheme": CAP_BAND},
           "inputs": [str(k1e), str(ident_dir), str(eval_dir)], "commit": ev.get("commit"),
           "script_sha256": E1.sha(__file__)}
    write(out, res)
    print(json.dumps({"reading": res["reading"], "gate_A": res["gate_A"], "gate_N": res["gate_N"],
                      "gate_I": res["gate_I"], "adaptation_supported": res["adaptation_supported"],
                      "gates_per_seed": gates, "controls": {k: v["label"] for k, v in ctrl.items()}}, indent=1))
    return 0


def main(argv=None):
    argv = list(argv if argv is not None else sys.argv[1:])
    if argv[:1] == ["controls"] and len(argv) == 3:
        k1e, out = pathlib.Path(argv[1]), pathlib.Path(argv[2])
        if out.exists():
            raise SystemExit("refusing to overwrite " + str(out))
        return main_controls(k1e, out)
    if argv[:1] == ["eval"] and len(argv) == 5:
        k1e, ident_dir, eval_dir, out = (pathlib.Path(a) for a in argv[1:5])
        if out.exists():
            raise SystemExit("refusing to overwrite " + str(out))
        return main_eval(k1e, ident_dir, eval_dir, out)
    raise SystemExit(__doc__.strip().splitlines()[-2].strip() + "\n" + __doc__.strip().splitlines()[-1].strip())


if __name__ == "__main__":
    raise SystemExit(main())
