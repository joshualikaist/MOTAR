"""Analysis of CAM-K1-E (docs/plans/prereg_camonly_k1_eval_v1_2026-11-08.md, as repaired after design review r1):
CAM-K1 seeds 1 and 2 against camera-only Policy F at the same 66 x 54 deg camera (F66) and against Policy F with
camera + LiDAR at 87 deg (A, GR-0's S00 cells).

GR-0's S00 grid (E2, A3, 70/115/160/205 bars x seeds 4501-4503, quota 16). Blocks are the 12 (density, seed) pairs;
per-block quota rates. The densities are fixed, so intervals resample seeds within each density (review r1 M2): a
stratified percentile bootstrap (10,000 resamples, seed 20,261,108) and a stratified t-interval (t 0.975, 8 df) as a
sensitivity check. Contrasts are in pp.

Gated, crash rate, per CAM-K1 seed s (labels need both intervals to agree):
  R_s = K1s - F66   gate R (recovery at the same camera):
        UNRESOLVED            the two intervals disagree on whether 0 is excluded, or on the side
        MATERIAL_RECOVERY     both upper ends < -10 pp
        SMALL_RECOVERY        both upper ends < 0 and the mean <= -2 pp
        WORSE                 both lower ends > 0 and the mean >= +2 pp
        NEGLIGIBLE            both intervals inside (-2, +2) pp
        NO_DETECTED_RECOVERY  otherwise
        A crash reduction with timeouts up by 2 pp or more (mean of the timeout contrast) reads CRASH_TO_TIMEOUT
        instead of MATERIAL_RECOVERY or SMALL_RECOVERY (review r1 m9).
  P_s = K1s - A     gate P (closeness to camera + LiDAR; review r1 M3: band 3 pp):
        NO_MATERIAL_LOSS      both upper ends < +3 pp
        MATERIAL_LOSS         both lower ends > +3 pp
        LOSS_INCONCLUSIVE     otherwise
  Lineage reading per gate: the label both seeds share, else SEEDS_DISAGREE.
Controls at claim scale on existing data (review r1 m8); any miss reads CONTROL_FAIL and no gate label is reported:
  gate P on C87 - A must read MATERIAL_LOSS; gate R on A - C87 must read MATERIAL_RECOVERY;
  gate P on S20 - A (GR-0's +-20 % gains) must read NO_MATERIAL_LOSS; gate R on S20 - A must not read a recovery.
Reported, not gated: the same contrasts for capture and timeout, F66 - C87, F66 - A, K1s1 - K1s2, C87 - A, pooled and
per-density rates, crash causes and the speed governor's telemetry.
Bindings, checked before any number (any failure reads EVAL_BINDING_FAIL): identity IDENTITY_PASS on the eval commit and
the identity report the eval named; eval COMPLETE with 36 cells; the same runner environment and C2 folder; every eval
cell's camtrain receipt, recorder camera FOV and checkpoint SHA as the run report names; every eval cell's result
condition equal to C87's and its NAVRL snapshot differing from C87's only in the expected keys (review r1 M4); complete
grids; F66 differs from C87, each seed from F66 and the seeds from each other in every block; the identity cells F66 and
K1s1 equal the eval's; GR-0's S00/S20 and C2's no_lidar cells at their committed manifests' SHA-256; this script's bytes
equal to the eval commit's (review r1 m6).
usage: tools/camk1e_analyse.py GR0_MAIN C2_DIR IDENTITY_DIR EVAL_DIR OUT_JSON
"""
import hashlib
import json
import pathlib
import math
import random
import subprocess
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import statistics                                            # noqa: E402
import camonly_c2_analyse as C2A                             # noqa: E402
import gr_analyse as A                                       # noqa: E402
import kroc2027_eval as K                                    # noqa: E402

ROOT = pathlib.Path(__file__).resolve().parents[1]
BOOT_SEED = 20261108
GR0_MANIFEST = ROOT / "docs/evidence/gain_randomization_2026-11-04/SHA256_MANIFEST.json"
C2_MANIFEST = ROOT / "docs/evidence/camonly_2026-11-04/c1c2/SHA256_MANIFEST_c1r2_c2.json"
R_BAND, P_BAND, FLOOR = 10.0, 3.0, 2.0
CAP_BAND = 3.0
T_CRIT_8 = 2.306004
SNAPSHOT_EXPECTED = {"NAVRL_BULK_EVAL_JSON", "NAVRL_EVAL_RUN_NONCE", "NAVRL_KROC2027_RUN_ID", "NAVRL_EVAL_CHECKPOINT",
                     "NAVRL_SENSOR_ABLATION_RECEIPT", "NAVRL_CAMTRAIN_RECEIPT", "NAVRL_CAMTRAIN_HFOV_DEG",
                     "NAVRL_CAMTRAIN_VFOV_DEG", "NAVRL_CAMTRAIN_YAW_ALIGN_WEIGHT", "NAVRL_CAMTRAIN_VISIBILITY_BONUS"}
CAM_AFTER = {"detector_hfov_deg": 66.0, "detector_vfov_deg": 54.0, "yaw_align_weight": 0.6, "visibility_bonus": 0.03}
EVAL_ARMS = ("F66", "K1s1", "K1s2")
CELL = "E2_obstacle_aware__%dbars__seed%d"


def sha(path):
    return hashlib.sha256(pathlib.Path(path).read_bytes()).hexdigest()


def both(boot, t, test):
    return test(boot) and test(t)


def gate_r(boot, t, mean, timeout_mean=0.0):
    if C2A.side(boot) != C2A.side(t):
        return "UNRESOLVED"
    if both(boot, t, lambda ci: ci[1] < -R_BAND):
        return "CRASH_TO_TIMEOUT" if timeout_mean >= FLOOR else "MATERIAL_RECOVERY"
    if both(boot, t, lambda ci: ci[1] < 0) and mean <= -FLOOR:
        return "CRASH_TO_TIMEOUT" if timeout_mean >= FLOOR else "SMALL_RECOVERY"
    if both(boot, t, lambda ci: ci[0] > 0) and mean >= FLOOR:
        return "WORSE"
    if both(boot, t, lambda ci: -FLOOR < ci[0] and ci[1] < FLOOR):
        return "NEGLIGIBLE"
    return "NO_DETECTED_RECOVERY"


def strat_ci(xs, blocks, rng):
    """Stratified percentile bootstrap: resample the 3 seeds within each density (review r1 M2)."""
    by = {}
    for x, (d, _s) in zip(xs, blocks):
        by.setdefault(d, []).append(x)
    groups = [by[d] for d in sorted(by)]
    n = len(xs)
    means = sorted(sum(sum(rng.choice(g) for _ in g) for g in groups) / n for _ in range(A.BOOT))
    return [means[int(0.025 * A.BOOT)], means[int(0.975 * A.BOOT) - 1]]


def strat_t(xs, blocks):
    """Stratified t-interval: equal-size strata, var(mean) = sum_d s_d^2 / n_d / D^2, df = sum(n_d - 1) = 8."""
    by = {}
    for x, (d, _s) in zip(xs, blocks):
        by.setdefault(d, []).append(x)
    groups = [by[d] for d in sorted(by)]
    m = sum(xs) / len(xs)
    var = sum(statistics.variance(g) / len(g) for g in groups) / len(groups) ** 2
    se = math.sqrt(var)
    return [m - T_CRIT_8 * se, m + T_CRIT_8 * se]


def gate_p(boot, t):
    if both(boot, t, lambda ci: ci[1] < P_BAND):
        return "NO_MATERIAL_LOSS"
    if both(boot, t, lambda ci: ci[0] > P_BAND):
        return "MATERIAL_LOSS"
    return "LOSS_INCONCLUSIVE"


def lineage(a, b):
    return a if a == b else "SEEDS_DISAGREE"


def contrast(cells, weights, blocks, metric, rng):
    xs = [100.0 * sum(w * cells[k][b][metric] for k, w in weights.items()) for b in blocks]
    boot, t, mean = strat_ci(xs, blocks, rng), strat_t(xs, blocks), sum(xs) / len(xs)
    return {"mean_pp": mean, "ci95_pp": boot, "t95_pp": t, "blocks_pp": xs}


def manifest_ok(man_path, raw_root, rels, key):
    man = json.loads(man_path.read_text())
    files = man[key]
    root = pathlib.Path(man.get("raw_dir") or man.get("root"))
    ok = root.resolve() == pathlib.Path(raw_root).resolve()
    for rel in rels:
        entry = files.get(rel)
        digest = entry.get("sha256") if isinstance(entry, dict) else entry
        ok &= digest is not None and digest == sha(root / rel)
    return ok


def file_matches_commit(path, rel, commit, root=ROOT):
    """The bytes of ``path`` equal ``rel`` at ``commit`` (review r1 m6)."""
    r = subprocess.run(["git", "-C", str(root), "show", "%s:%s" % (commit, rel)], capture_output=True)
    return r.returncode == 0 and r.stdout == pathlib.Path(path).read_bytes()


def snapshot(d):
    return json.loads((d / "receipt.json").read_text()).get("env_snapshot_navrl", {})


def load_eval(ident_dir, eval_dir, c2_dir, blocks):
    ident = json.loads((ident_dir / "identity_report.json").read_text())
    ev = json.loads((eval_dir / "eval_run_report.json").read_text())
    binding = {"identity_pass": ident.get("verdict") == "IDENTITY_PASS",
               "identity_same_commit": ident.get("commit") == ev.get("commit"),
               "identity_is_the_one_eval_named": str((ident_dir / "identity_report.json").resolve())
               == str(pathlib.Path(ev.get("identity_report") or "/nonexistent").resolve()),
               "eval_complete": ev.get("verdict") == "COMPLETE" and ev.get("cells_expected") == 36,
               "env_allowlisted_same": bool(ident.get("env_allowlisted"))
               and ident.get("env_allowlisted") == ev.get("env_allowlisted"),
               "c2_dir_same": ident.get("c2_dir") == ev.get("c2_dir") == str(c2_dir.resolve())}
    cells = {k: {} for k in EVAL_ARMS}
    receipts_ok, fov_ok, cond_ok, snap_ok = True, True, True, True
    ck = ev.get("checkpoints") or {}
    for r in ev.get("records", []):
        arm = r.get("arm")
        if arm not in cells:
            receipts_ok = False
            continue
        d = eval_dir / arm / r["cell_id"]
        blk = (r["density"], r["seed"])
        cells[arm][blk] = {"dir": d, **A.rates(d)}
        want_ck = ck.get("F" if arm == "F66" else arm) or [None, None]
        try:
            rec = json.loads((d / "camtrain_receipt.json").read_text())
            receipt = json.loads((d / "receipt.json").read_text())
            receipts_ok &= (rec["ablation"]["condition"] == "no_lidar"
                            and rec.get("ablation_module_sha256") == ev.get("module_sha256")
                            and rec.get("entry_sha256") == ev.get("entry_sha256")
                            and rec.get("overrides_after") == CAM_AFTER
                            and receipt.get("env_snapshot_navrl", {}).get("NAVRL_SENSOR_ABLATION") == "no_lidar"
                            and receipt.get("checkpoint_sha256") == want_ck[1] == r.get("checkpoint_sha256"))
            cam = json.loads((d / K.RECORDER_SUMMARY).read_text()).get("camera", {})
            fov_ok &= abs(cam["hfov_deg"] - 66.0) < 1e-6 and abs(cam["vfov_deg"] - 54.0) < 1e-6
            c87 = c2_dir / "no_lidar" / r["cell_id"]
            cond_ok &= K.comparable(json.loads((d / "result.json").read_text()))["condition"] == \
                K.comparable(json.loads((c87 / "result.json").read_text()))["condition"]
            a_snap, b_snap = snapshot(d), snapshot(c87)
            snap_ok &= {k for k in set(a_snap) | set(b_snap) if a_snap.get(k) != b_snap.get(k)} <= SNAPSHOT_EXPECTED
        except (OSError, ValueError, KeyError, TypeError):
            receipts_ok = False
    binding["camtrain_receipts_and_checkpoints_match"] = receipts_ok
    binding["recorder_camera_fov_66x54"] = fov_ok
    binding["conditions_equal_c87"] = cond_ok
    binding["snapshots_differ_from_c87_only_in_expected_keys"] = snap_ok
    for arm in EVAL_ARMS:
        binding["complete_grid_" + arm] = sorted(cells[arm]) == blocks
    for ident_arm, eval_arm in (("F66_negative", "F66"), ("K1s1_smoke", "K1s1")):
        a = ident_dir / ident_arm / (CELL % (205, 4501)) / "episodes.jsonl"
        b = eval_dir / eval_arm / (CELL % (205, 4501)) / "episodes.jsonl"
        binding["identity_%s_equals_eval_%s" % (ident_arm, eval_arm)] = a.is_file() and b.is_file() and \
            K.read_jsonl(a) == K.read_jsonl(b)
    return binding, cells, ev


def gate_of(c, kind, timeout_mean=0.0):
    return gate_r(c["ci95_pp"], c["t95_pp"], c["mean_pp"], timeout_mean) if kind == "R" else \
        gate_p(c["ci95_pp"], c["t95_pp"])


def main(argv=None):
    gr0, c2_dir, ident_dir, eval_dir, out = (pathlib.Path(a) for a in (argv or sys.argv[1:6]))
    if out.exists():
        raise SystemExit("refusing to overwrite " + str(out))
    rep0 = json.loads((gr0 / "gr0_run_report.json").read_text())
    blocks = sorted({(r["density"], r["seed"]) for r in rep0["records"] if r["level"] == "S00"})
    cells = {"A": {}, "C87": {}, "S20": {}}
    for d, s in blocks:
        for key, base in (("A", gr0 / "S00"), ("S20", gr0 / "S20"), ("C87", c2_dir / "no_lidar")):
            cells[key][(d, s)] = {"dir": base / (CELL % (d, s)), **A.rates(base / (CELL % (d, s)))}
    binding, ev_cells, ev = load_eval(ident_dir, eval_dir, c2_dir, blocks)
    cells.update(ev_cells)
    binding["gr0_complete"] = rep0.get("verdict") == "COMPLETE"
    binding["blocks_are_12"] = len(blocks) == 12
    rels0 = ["%s/%s/%s/%s" % (gr0.name, lv, CELL % b, f) for lv in ("S00", "S20") for b in blocks
             for f in ("episodes.jsonl", "result.json")]
    rels2 = ["%s/no_lidar/%s/%s" % (c2_dir.name, CELL % b, f) for b in blocks for f in ("episodes.jsonl", "result.json")]
    binding["gr0_s00_s20_cells_match_manifest"] = manifest_ok(GR0_MANIFEST, gr0.parent, rels0, "raw_files")
    binding["c2_no_lidar_cells_match_manifest"] = manifest_ok(C2_MANIFEST, c2_dir.parent, rels2, "files")
    binding["script_equals_eval_commit"] = file_matches_commit(__file__, "tools/camk1e_analyse.py", ev.get("commit"))
    differ = {}
    for a, b in (("F66", "C87"), ("K1s1", "F66"), ("K1s2", "F66"), ("K1s1", "K1s2")):
        differ[a + "_vs_" + b] = all(blk in cells[a] and blk in cells[b] and
                                     K.read_jsonl(cells[a][blk]["dir"] / "episodes.jsonl")
                                     != K.read_jsonl(cells[b][blk]["dir"] / "episodes.jsonl") for blk in blocks)
    binding["cells_differ"] = all(differ.values())
    binding["cells_differ_detail"] = differ
    if not all(v for k, v in binding.items() if k != "cells_differ_detail"):
        res = {"analysis": "CAM-K1-E", "reading": "EVAL_BINDING_FAIL", "binding": binding}
        out.write_text(json.dumps(res, indent=1) + "\n")
        print(json.dumps(res, indent=1))
        return 1
    rng = random.Random(BOOT_SEED)
    defs = {"R_K1s1": {"K1s1": 1, "F66": -1}, "R_K1s2": {"K1s2": 1, "F66": -1},
            "P_K1s1": {"K1s1": 1, "A": -1}, "P_K1s2": {"K1s2": 1, "A": -1},
            "control_C87_minus_A": {"C87": 1, "A": -1}, "control_A_minus_C87": {"A": 1, "C87": -1},
            "control_S20_minus_A": {"S20": 1, "A": -1},
            "fov_F66_minus_C87": {"F66": 1, "C87": -1}, "loss_F66_minus_A": {"F66": 1, "A": -1},
            "seed_K1s1_minus_K1s2": {"K1s1": 1, "K1s2": -1}}
    contrasts = {}
    for name, w in defs.items():
        contrasts[name] = {}
        for metric in ("crash", "capture", "timeout"):
            c = contrast(cells, w, blocks, metric, rng)
            c["label_c2_scheme"] = C2A.label(c["ci95_pp"], c["t95_pp"], c["mean_pp"],
                                             CAP_BAND if metric == "capture" else 2.0)
            contrasts[name][metric] = c
    cr = {k: contrasts[k]["crash"] for k in defs}
    to = {k: contrasts[k]["timeout"]["mean_pp"] for k in defs}
    gates = {"R_K1s1": gate_of(cr["R_K1s1"], "R", to["R_K1s1"]), "R_K1s2": gate_of(cr["R_K1s2"], "R", to["R_K1s2"]),
             "P_K1s1": gate_of(cr["P_K1s1"], "P"), "P_K1s2": gate_of(cr["P_K1s2"], "P")}
    controls = {"P_on_C87_minus_A": (gate_of(cr["control_C87_minus_A"], "P"), "MATERIAL_LOSS"),
                "R_on_A_minus_C87": (gate_of(cr["control_A_minus_C87"], "R", to["control_A_minus_C87"]),
                                     "MATERIAL_RECOVERY"),
                "P_on_S20_minus_A": (gate_of(cr["control_S20_minus_A"], "P"), "NO_MATERIAL_LOSS"),
                "R_on_S20_minus_A": (gate_of(cr["control_S20_minus_A"], "R", to["control_S20_minus_A"]),
                                     "not MATERIAL_RECOVERY or SMALL_RECOVERY")}
    ctrl_ok = {k: (v[0] == v[1]) if not v[1].startswith("not ") else v[0] not in ("MATERIAL_RECOVERY",
                                                                                  "SMALL_RECOVERY")
               for k, v in controls.items()}
    if not all(ctrl_ok.values()):
        reading = {"reading": "CONTROL_FAIL", "gate_R": None, "gate_P": None}
    else:
        reading = {"reading": "REPORTED", "gate_R": lineage(gates["R_K1s1"], gates["R_K1s2"]),
                   "gate_P": lineage(gates["P_K1s1"], gates["P_K1s2"])}
    pooled = {k: {m: 100.0 * sum(c[b][m] for b in blocks) / len(blocks) for m in ("capture", "crash", "timeout")}
              for k, c in cells.items()}
    per_density = {k: {str(d): {m: 100.0 * sum(c[(d, s)][m] for (dd, s) in blocks if dd == d) / 3.0
                                for m in ("capture", "crash", "timeout")} for d in sorted({b[0] for b in blocks})}
                   for k, c in cells.items()}
    res = {"analysis": "CAM-K1-E", **reading, "gates_per_seed": gates,
           "controls": {k: {"label": v[0], "required": v[1], "ok": ctrl_ok[k]} for k, v in controls.items()},
           "binding": binding, "contrasts": contrasts, "pooled_rate_pct": pooled, "per_density_rate_pct": per_density,
           "crash_causes_exploratory": {k: C2A.causes(c) for k, c in cells.items()},
           "speed_governor": {k: C2A.governor(c) for k, c in cells.items()},
           "blocks": [list(b) for b in blocks],
           "bootstrap": {"resamples": A.BOOT, "seed": BOOT_SEED, "scheme": "seeds within density", "t_df": 8},
           "bands_pp": {"R": R_BAND, "P": P_BAND, "floor": FLOOR, "capture_c2_scheme": CAP_BAND},
           "inputs": [str(gr0), str(c2_dir), str(ident_dir), str(eval_dir)], "commit": ev.get("commit"),
           "script_sha256": sha(__file__)}
    with out.open("x") as fh:
        json.dump(res, fh, indent=1, default=str)
        fh.write("\n")
    print(json.dumps({"reading": res["reading"], "gate_R": res["gate_R"], "gate_P": res["gate_P"],
                      "gates_per_seed": gates, "controls": res["controls"], "pooled": pooled}, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
