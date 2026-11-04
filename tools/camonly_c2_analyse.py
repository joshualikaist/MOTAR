"""Analysis of C2, the camera-only study's zero-shot conditions (docs/plans/prereg_camonly_c1c2_v1.md).

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
Labels per contrast and metric: LOWER (CI upper < 0), HIGHER (CI lower > 0), NO_DETECTED_CHANGE (CI contains 0).
No adoption decision is taken. Conditions without out-of-FOV LiDAR (B_out, C, C_gL) are zero-shot out of
distribution for F (the K1 fill reads as free space) and are labelled OOD.
Bindings, checked before any number: C1_IDENTITY_PASS on the C2 commit; C2 COMPLETE; every C2 cell's ablation
receipt and snapshot name its condition; complete grid; each C2 cell differs from the matching full cell.
Reported, not gated: pooled rates and the crash-cause table (bar contact and below per 100 episodes).
usage: tools/camonly_c2_analyse.py GR0_MAIN C1_DIR C2_DIR OUT_JSON
"""
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
T_CRIT_11 = 2.200985
NAMES = {"A": "full", "B_in": "no_lidar_in_fov", "B_out": "no_lidar_out_fov", "C": "no_lidar",
         "C_gL": "no_lidar_obs_only", "B_in_gL": "no_lidar_in_fov_obs_only"}
OOD = ("B_out", "C", "C_gL")
CONTRASTS = {"total": {"C": 1, "A": -1}, "in_fov": {"B_in": 1, "A": -1}, "out_fov": {"B_out": 1, "A": -1},
             "interaction": {"C": 1, "B_in": -1, "B_out": -1, "A": 1}, "obs_in": {"B_in_gL": 1, "A": -1},
             "gov_in": {"B_in": 1, "B_in_gL": -1}, "gov_all": {"C": 1, "C_gL": -1}}


def t_interval(xs):
    m = sum(xs) / len(xs)
    se = statistics.stdev(xs) / math.sqrt(len(xs))
    return [m - T_CRIT_11 * se, m + T_CRIT_11 * se]


def label(ci):
    return "LOWER" if ci[1] < 0 else ("HIGHER" if ci[0] > 0 else "NO_DETECTED_CHANGE")


def causes(cells):
    out = {"episodes": 0, "bar_contact": 0, "below": 0}
    for c in cells.values():
        res = json.loads((c["dir"] / "result.json").read_text())
        out["episodes"] += res.get("actual_episodes") or 0
        for k in ("bar_contact", "below"):
            v = res.get("crash_causes", {}).get(k, 0)
            out[k] += int(v) if isinstance(v, (int, float)) else 0
    out.update({k + "_per_100": 100.0 * out[k] / max(1, out["episodes"]) for k in ("bar_contact", "below")})
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
               "c2_complete": c2.get("verdict") == "COMPLETE"}
    receipts_ok, differ_ok = True, True
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
            except (OSError, ValueError):
                receipts_ok = False
            if (r["density"], r["seed"]) in cells["A"]:
                differ_ok &= K.read_jsonl(d / "episodes.jsonl") != \
                    K.read_jsonl(cells["A"][(r["density"], r["seed"])]["dir"] / "episodes.jsonl")
        binding["complete_grid_" + key] = sorted(cells[key]) == blocks
    binding["ablation_receipts_match"] = receipts_ok
    binding["cells_differ_from_full"] = differ_ok
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
            ci = A.ci(xs, rng)
            contrasts[cname][metric] = {"mean_pp": sum(xs) / len(xs), "ci95_pp": list(ci), "t95_pp": t_interval(xs),
                                        "label": label(ci), "blocks_pp": xs}
        contrasts[cname]["ood_terms"] = sorted(k for k in weights if k in OOD)
    pooled = {k: {m: 100.0 * sum(c[b][m] for b in blocks) / len(blocks) for m in ("capture", "crash", "timeout")}
              for k, c in cells.items()}
    res = {"analysis": "camonly C2", "reading": "REPORTED", "binding": binding, "conditions": NAMES, "ood": list(OOD),
           "contrasts": contrasts, "pooled_rate_pct": pooled,
           "crash_causes_exploratory": {k: causes(c) for k, c in cells.items()},
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
