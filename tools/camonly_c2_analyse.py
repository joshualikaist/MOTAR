"""Analysis of C2, the camera-only study's zero-shot conditions (docs/plans/prereg_camonly_c1c2_v1.md, as amended by
docs/plans/prereg_camonly_c1c2_v1_amendment1_2026-11-04.md after the design review AMEND_FIRST).

Policy F, GR-0's S00 grid (E2, A3, 70/115/160/205 bars x seeds 4501-4503, quota 16). Condition A (`full`) is GR-0's
S00 cells, admitted by C1's identity. Blocks are the 12 (density, seed) pairs; per-block quota rates; percentile
bootstrap over blocks (10,000 resamples, seed 20,261,112) and a t-interval as a sensitivity check.

Contrasts, in pp, each for capture and for crash (X - A means "condition X minus full"):
  total        C - A            removing all LiDAR
  in_fov       B_in - A         removing the in-FOV LiDAR (observation, association and governor)
  out_fov      B_out - A        removing the out-of-FOV LiDAR
  interaction  C - B_in - B_out + A
  obs_in       B_in_gL - A      removing the in-FOV LiDAR from perception only (governor keeps the raw LiDAR)
  gov_in       B_in - B_in_gL   additionally removing it from the governor
  gov_all      C - C_gL         the governor's share once perception is camera-only
  obs_all      C_gL - A         removing all LiDAR from perception only (amendment 1, M1; total = obs_all + gov_all)
Labels per contrast and metric (amendment 1, M4; band 3 pp capture, 2 pp crash, as GR-0):
  UNRESOLVED          the bootstrap CI and the t-interval disagree on whether 0 is excluded, or on the side
  LOWER / HIGHER      both exclude 0 on that side and |mean| >= band
  SMALL_LOWER / SMALL_HIGHER  both exclude 0 on that side and |mean| < band
  NEGLIGIBLE          neither excludes 0 and both lie inside (-band, +band)
  NO_DETECTED_CHANGE  otherwise (0 inside, but the interval reaches beyond the band)
No adoption decision is taken. Conditions without out-of-FOV LiDAR (B_out, C, C_gL) are zero-shot out of
distribution for F (the K1 fill reads as free space) and are labelled OOD.
Bindings, checked before any number: C1_IDENTITY_PASS on the C2 commit; C2 COMPLETE; every C2 cell's ablation
receipt and snapshot name its condition; complete grid; each C2 cell differs from the matching full cell.
Amendment 1 adds: GR-0 COMPLETE and its S00 cells at the committed manifest's SHA-256 (L1), the same --gr0-dir in
C1 and C2; each C2 cell's recorded condition equals its GR-0 cell's and its NAVRL snapshot differs only in the
expected keys (M3); C1 and C2 recorded the same allowlisted runner environment (M3); B_in != B_in_gL and
C != C_gL in every block, so the governor ablation engaged (M2); C1's no_lidar cell equals C2's (L9).
Reported, not gated: pooled rates, the crash-cause table (bar contact, below, out of bounds per 100 episodes, over
the aggregate window of result.json, not the quota; L6) and the speed-governor telemetry per condition (M2).
usage: tools/camonly_c2_analyse.py GR0_MAIN C1_DIR C2_DIR OUT_JSON
"""
import hashlib
import json
import math
import pathlib
import random
import statistics
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import gr_analyse as A                                       # noqa: E402
import kroc2027_eval as K                                    # noqa: E402

BOOT_SEED = 20261112
BAND_PP = {"capture": 3.0, "crash": 2.0}                       # amendment 1, M4 (GR-0's CAP_PP / CRASH_PP)
ROOT = pathlib.Path(__file__).resolve().parents[1]
GR0_MANIFEST = ROOT / "docs/evidence/gain_randomization_2026-11-04/SHA256_MANIFEST.json"
SNAPSHOT_EXPECTED = {"NAVRL_BULK_EVAL_JSON", "NAVRL_EVAL_RUN_NONCE", "NAVRL_KROC2027_RUN_ID", "NAVRL_SENSOR_ABLATION",
                     "NAVRL_SENSOR_ABLATION_RECEIPT"}                 # amendment 1, M3 (measured on C1 vs GR-0)
GOV_KEYS = ("intervention_rate", "mean_executed_speed_mps", "mean_clearance_m", "mean_scale")
T_CRIT_11 = 2.200985
NAMES = {"A": "full", "B_in": "no_lidar_in_fov", "B_out": "no_lidar_out_fov", "C": "no_lidar",
         "C_gL": "no_lidar_obs_only", "B_in_gL": "no_lidar_in_fov_obs_only"}
OOD = ("B_out", "C", "C_gL")
CONTRASTS = {"total": {"C": 1, "A": -1}, "in_fov": {"B_in": 1, "A": -1}, "out_fov": {"B_out": 1, "A": -1},
             "interaction": {"C": 1, "B_in": -1, "B_out": -1, "A": 1}, "obs_in": {"B_in_gL": 1, "A": -1},
             "gov_in": {"B_in": 1, "B_in_gL": -1}, "gov_all": {"C": 1, "C_gL": -1},
             "obs_all": {"C_gL": 1, "A": -1}}


def t_interval(xs):
    m = sum(xs) / len(xs)
    se = statistics.stdev(xs) / math.sqrt(len(xs))
    return [m - T_CRIT_11 * se, m + T_CRIT_11 * se]


def side(ci):
    return -1 if ci[1] < 0 else (1 if ci[0] > 0 else 0)


def label(boot, t, mean, band):
    """Amendment 1, M4."""
    if side(boot) != side(t):
        return "UNRESOLVED"
    s = side(boot)
    if s:
        return ("LOWER" if s < 0 else "HIGHER") if abs(mean) >= band else ("SMALL_LOWER" if s < 0 else "SMALL_HIGHER")
    inside = all(-band < x < band for x in list(boot) + list(t))
    return "NEGLIGIBLE" if inside else "NO_DETECTED_CHANGE"


def sha(path):
    return hashlib.sha256(pathlib.Path(path).read_bytes()).hexdigest()


def governor(cells):
    """Sample-weighted speed-governor telemetry over cells (result.json `speed_governor`)."""
    tot, acc = 0, {k: 0.0 for k in GOV_KEYS}
    for c in cells.values():
        g = json.loads((c["dir"] / "result.json").read_text()).get("speed_governor", {})
        n = g.get("samples") or 0
        tot += n
        for k in GOV_KEYS:
            acc[k] += n * float(g.get(k) or 0.0)
    return {"samples": tot, **{k: (acc[k] / tot if tot else None) for k in GOV_KEYS}}


def causes(cells):
    out = {"episodes": 0, "bar_contact": 0, "below": 0, "out_of_bounds": 0}
    for c in cells.values():
        res = json.loads((c["dir"] / "result.json").read_text())
        out["episodes"] += res.get("actual_episodes") or 0
        for k in ("bar_contact", "below", "out_of_bounds"):
            v = res.get("crash_causes", {}).get(k, 0)
            out[k] += int(v) if isinstance(v, (int, float)) else 0
    out.update({k + "_per_100": 100.0 * out[k] / max(1, out["episodes"]) for k in ("bar_contact", "below", "out_of_bounds")})
    return out


def main():
    gr0, c1_dir, c2_dir, out = (pathlib.Path(a) for a in sys.argv[1:5])
    if out.exists():
        raise SystemExit("refusing to overwrite " + str(out))
    c1 = json.loads((c1_dir / "c1_report.json").read_text())
    c2 = json.loads((c2_dir / "c2_run_report.json").read_text())
    rep0 = json.loads((gr0 / "gr0_run_report.json").read_text())
    blocks = sorted({(r["density"], r["seed"]) for r in rep0["records"] if r["level"] == "S00"})
    cells = {"A": {}}
    for d, s in blocks:
        cid = "E2_obstacle_aware__%dbars__seed%d" % (d, s)
        cells["A"][(d, s)] = {"dir": gr0 / "S00" / cid, **A.rates(gr0 / "S00" / cid)}
    binding = {"c1_identity_pass": c1.get("verdict") == "C1_IDENTITY_PASS",
               "c1_same_commit": c1.get("commit") == c2.get("commit"),
               "c2_complete": c2.get("verdict") == "COMPLETE",
               # amendment 1, L1: the GR-0 inputs
               "gr0_complete": rep0.get("verdict") == "COMPLETE",
               "gr0_dir_same_in_c1_c2": str(gr0.resolve()) == c1.get("gr0_dir") == c2.get("gr0_dir"),
               # amendment 1, M3: one runner environment for C1 and C2
               "env_allowlisted_same_c1_c2": bool(c1.get("env_allowlisted"))
               and c1.get("env_allowlisted") == c2.get("env_allowlisted")}
    man = json.loads(GR0_MANIFEST.read_text())
    man_ok = pathlib.Path(man["raw_dir"]).resolve() == gr0.resolve().parent
    for d, s in blocks:
        for f in ("episodes.jsonl", "result.json", "receipt.json"):
            rel = "%s/S00/E2_obstacle_aware__%dbars__seed%d/%s" % (gr0.name, d, s, f)
            man_ok &= rel in man["raw_files"] and man["raw_files"][rel]["sha256"] == sha(gr0 / rel.split("/", 1)[1])
    binding["gr0_s00_cells_match_manifest"] = man_ok
    receipts_ok, differ_ok, cond_ok, snap_ok = True, True, True, True
    for key, name in NAMES.items():
        if key == "A":
            continue
        cells[key] = {}
        for r in c2.get("records", []):
            if r.get("condition") != name:
                continue
            d = c2_dir / name / r["cell_id"]
            cells[key][(r["density"], r["seed"])] = {"dir": d, **A.rates(d)}
            try:
                rec = json.loads((d / "sensor_ablation_receipt.json").read_text())
                snap = json.loads((d / "receipt.json").read_text()).get("env_snapshot_navrl", {})
                receipts_ok &= rec.get("condition") == name == snap.get("NAVRL_SENSOR_ABLATION") \
                    and rec.get("module_sha256") == c2.get("module_sha256")
                if (r["density"], r["seed"]) in cells["A"]:
                    a_dir = cells["A"][(r["density"], r["seed"])]["dir"]
                    a_snap = json.loads((a_dir / "receipt.json").read_text()).get("env_snapshot_navrl", {})
                    diff = {k for k in set(snap) | set(a_snap) if snap.get(k) != a_snap.get(k)}
                    snap_ok &= diff <= SNAPSHOT_EXPECTED
                    cond_ok &= K.comparable(json.loads((d / "result.json").read_text()))["condition"] == \
                        K.comparable(json.loads((a_dir / "result.json").read_text()))["condition"]
            except (OSError, ValueError, KeyError):
                receipts_ok = False
            if (r["density"], r["seed"]) in cells["A"]:
                differ_ok &= K.read_jsonl(d / "episodes.jsonl") != \
                    K.read_jsonl(cells["A"][(r["density"], r["seed"])]["dir"] / "episodes.jsonl")
        binding["complete_grid_" + key] = sorted(cells[key]) == blocks
    binding["ablation_receipts_match"] = receipts_ok
    binding["cells_differ_from_full"] = differ_ok
    binding["conditions_equal_gr0"] = cond_ok
    binding["snapshots_differ_only_in_expected_keys"] = snap_ok
    # amendment 1, M2: the governor ablation engaged in the simulator (raw-LiDAR governor vs ablated governor)
    gov_engaged = True
    for a, b in (("B_in", "B_in_gL"), ("C", "C_gL")):
        for blk in blocks:
            if blk in cells.get(a, {}) and blk in cells.get(b, {}):
                gov_engaged &= K.read_jsonl(cells[a][blk]["dir"] / "episodes.jsonl") != \
                    K.read_jsonl(cells[b][blk]["dir"] / "episodes.jsonl")
    binding["governor_ablation_engaged"] = gov_engaged
    # amendment 1, L9: C1's no_lidar cell reproduces in C2
    c1_nl = c1_dir / "no_lidar" / "E2_obstacle_aware__205bars__seed4501" / "episodes.jsonl"
    c2_nl = c2_dir / "no_lidar" / "E2_obstacle_aware__205bars__seed4501" / "episodes.jsonl"
    binding["c1_no_lidar_equals_c2"] = c1_nl.is_file() and c2_nl.is_file() and \
        K.read_jsonl(c1_nl) == K.read_jsonl(c2_nl)
    if not all(binding.values()):
        res = {"analysis": "camonly C2", "reading": "EVAL_BINDING_FAIL", "binding": binding}
        out.write_text(json.dumps(res, indent=1) + "\n")
        print(json.dumps(res, indent=1))
        return 1
    rng = random.Random(BOOT_SEED)
    contrasts = {}
    for cname, weights in CONTRASTS.items():
        contrasts[cname] = {}
        for metric in ("capture", "crash"):
            xs = [100.0 * sum(w * cells[k][b][metric] for k, w in weights.items()) for b in blocks]
            ci, tci, mean = A.ci(xs, rng), t_interval(xs), sum(xs) / len(xs)
            contrasts[cname][metric] = {"mean_pp": mean, "ci95_pp": list(ci), "t95_pp": tci,
                                        "label": label(ci, tci, mean, BAND_PP[metric]), "band_pp": BAND_PP[metric],
                                        "blocks_pp": xs}
        contrasts[cname]["ood_terms"] = sorted(k for k in weights if k in OOD)
    pooled = {k: {m: 100.0 * sum(c[b][m] for b in blocks) / len(blocks) for m in ("capture", "crash", "timeout")}
              for k, c in cells.items()}
    res = {"analysis": "camonly C2", "reading": "REPORTED", "binding": binding, "conditions": NAMES, "ood": list(OOD),
           "contrasts": contrasts, "pooled_rate_pct": pooled,
           "crash_causes_exploratory": {k: causes(c) for k, c in cells.items()},
           "speed_governor": {k: governor(c) for k, c in cells.items()},
           "blocks": [list(b) for b in blocks], "bootstrap": {"resamples": A.BOOT, "seed": BOOT_SEED},
           "inputs": [str(gr0), str(c1_dir), str(c2_dir)], "commit": c2.get("commit")}
    with out.open("x") as fh:
        json.dump(res, fh, indent=1, default=str)
        fh.write("\n")
    print(json.dumps({"reading": res["reading"], "pooled": pooled,
                      "contrasts": {c: {m: (round(v[m]["mean_pp"], 2), [round(x, 2) for x in v[m]["ci95_pp"]],
                                            v[m]["label"]) for m in ("capture", "crash")}
                                    for c, v in contrasts.items()}}, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
