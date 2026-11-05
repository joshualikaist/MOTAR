"""Independent GR-1b audit (Opus 5.5 reviewer). Read-only on inputs; writes only to its own directory."""
import hashlib, json, math, os, random, statistics, difflib, pathlib, sys
assert os.environ.get("CUDA_VISIBLE_DEVICES") == "", "CUDA must be hidden"
P = pathlib.Path
RES = P("/home/fair/workspaces/aerial_gym_ws/src/aerial_gym_simulator/results")
TREE = P("/home/fair/workspaces/aerial_gym_ws/src/MOTAR-gainrand")
OUT = P("/home/fair/motar_tmp/gr1b_resume_20261105/opus_review")
GR0 = RES / "gain_randomization_gr0_v1/gr0_main"
B1, B = RES / "gain_randomization_gr1_v1", RES / "gain_randomization_gr1b_v1"
PAIRS = {"s1": {"rand40": B1 / "eval_rand40", "ctrl": B1 / "eval_ctrl",
                "tr_rand40": B1 / "train_rand40/train_report.json", "tr_ctrl": B1 / "train_ctrl/train_report.json"},
         "s2": {"rand40": B / "eval_rand40_s2", "ctrl": B / "eval_ctrl_s2",
                "tr_rand40": B / "train_rand40_s2/train_report.json", "tr_ctrl": B / "train_ctrl_s2/train_report.json"}}
LEVELS = ("S00", "S20", "S40")
DENS, SEEDS = (70, 115, 160, 205), (4501, 4502, 4503)
GRID = {(l, d, s) for l in LEVELS for d in DENS for s in SEEDS}
BLOCKS = sorted({(d, s) for d in DENS for s in SEEDS})
SCALE = {"S00": None, "S20": 0.2, "S40": 0.4}
PER_CELL_KEYS = {"NAVRL_BULK_EVAL_JSON", "NAVRL_EVAL_CHECKPOINT", "NAVRL_EVAL_RUN_NONCE", "NAVRL_KROC2027_RUN_ID"}


def sha(p):
    h = hashlib.sha256()
    with open(p, "rb") as f:
        for blk in iter(lambda: f.read(1048576), b""):
            h.update(blk)
    return h.hexdigest()


def jl(p):
    return [json.loads(x) for x in p.read_text().splitlines() if x.strip()]


def inside(path, root):
    try:
        P(path).resolve().relative_to(root.resolve()); return True
    except Exception:
        return False


def neg(x):
    return x != 0 and math.copysign(1, x) == -1


def boot_ci(xs, rng, B=10000):
    means = sorted(sum(rng.choice(xs) for _ in xs) / len(xs) for _ in range(B))
    return [means[int(0.025 * B)], means[int(0.975 * B) - 1]]


def tci(xs):
    m = sum(xs) / len(xs); se = statistics.stdev(xs) / math.sqrt(len(xs)); return [m - 2.200985 * se, m + 2.200985 * se]


def audit_eval(d, report_name, ckpt_sha):
    rep = json.loads((d / report_name).read_text())
    prob = []
    hs, he = rep["host_start"], rep["host_end"]
    if rep.get("verdict") != "COMPLETE": prob.append("verdict")
    if rep.get("preflight_problems"): prob.append("preflight_problems")
    for k in ("fault_count", "boot_id", "online"):
        if hs[k] != he[k]: prob.append("host %s changed start/end" % k)
    for m, o in (rep.get("import_origins") or {}).items():
        if not inside(o, TREE): prob.append("report import origin %s outside tree: %s" % (m, o))
    if set((rep.get("import_origins") or {})) != {"aerial_gym", "motar_eval"}:
        prob.append("import_origins keys %s" % list(rep.get("import_origins") or {}))
    keys = [(r["level"], r["density"], r["seed"]) for r in rep["records"]]
    if len(keys) != 36 or len(set(keys)) != 36 or set(keys) != GRID: prob.append("grid")
    cells, env_ref = {}, None
    for r in rep["records"]:
        key = (r["level"], r["density"], r["seed"])
        cd = d / r["level"] / r["cell_id"]
        if r["cell_id"] != "E2_obstacle_aware__%dbars__seed%d" % (r["density"], r["seed"]): prob.append("cell_id %s" % (key,))
        if r.get("verdict") != "OK" or r.get("exit") != 0 or r.get("problems"): prob.append("record not OK %s" % (key,))
        if r.get("quota") != 16 or r["stats"]["quota_episodes"] != 2048: prob.append("quota %s" % (key,))
        if r.get("commit") != rep["commit"]: prob.append("record commit %s" % (key,))
        if r.get("scale") != SCALE[r["level"]]: prob.append("scale %s" % (key,))
        ha = r["host_after"]
        for k in ("fault_count", "boot_id", "online"):
            if ha[k] != hs[k]: prob.append("cell host %s %s" % (k, key))
        rc = json.loads((cd / "receipt.json").read_text())
        rs_actual = sha(cd / "result.json")
        if rc.get("status") != "OK" or rc.get("git_commit") != rep["commit"] or rc.get("git_dirty_files") != 0:
            prob.append("receipt status/commit/dirty %s" % (key,))
        env = rc["env_snapshot_navrl"]
        if ckpt_sha is not None and (rc.get("checkpoint_sha256") != ckpt_sha or env.get("NAVRL_EVAL_CHECKPOINT") != ckpt_sha):
            prob.append("receipt ckpt sha %s" % (key,))
        if rc.get("result_sha256") != rs_actual or r["stats"].get("result_sha256") != rs_actual:
            prob.append("result sha %s" % (key,))
        g = {k: v for k, v in env.items() if k.startswith("NAVRL_GAIN_RAND")}
        want = {} if SCALE[r["level"]] is None else {"NAVRL_GAIN_RAND": "1", "NAVRL_GAIN_RAND_SCALE": repr(SCALE[r["level"]]),
                                                      "NAVRL_GAIN_RAND_SEED": str(r["seed"])}
        if g != want: prob.append("gain env %s %s" % (key, g))
        if (env.get("NAVRL_SEED") != str(r["seed"]) or env.get("NAVRL_NUM_BARS") != str(r["density"]) or
                env.get("NAVRL_EPISODE_QUOTA_PER_ENV") != "16" or env.get("NAVRL_KROC2027_ARM") != "A3" or
                env.get("NAVRL_TARGET_DISTANCE_BACKEND") != "direct_l2_v1"):
            prob.append("env cond %s" % (key,))
        base = {k: v for k, v in env.items() if k not in PER_CELL_KEYS and not k.startswith("NAVRL_GAIN_RAND")
                and k not in ("NAVRL_SEED", "NAVRL_NUM_BARS", "NAVRL_DENSITY_START", "NAVRL_DENSITY_FINAL")}
        if env_ref is None: env_ref = base
        elif base != env_ref: prob.append("env base differs within folder %s" % (key,))
        ks = json.loads((cd / "kroc2027_summary.json").read_text())
        for m in ("aerial_gym", "motar_eval"):
            o = (ks.get("import_origins") or {}).get(m)
            if not o or not inside(o, TREE): prob.append("cell import origin %s %s %s" % (m, key, o))
        v = cd / "kroc2027_violations.jsonl"
        if v.exists() and v.read_text().strip(): prob.append("violations %s" % (key,))
        rows = jl(cd / "episodes.jsonl")
        q = [x for x in rows if x.get("in_quota")]
        ids = [x["episode_id"] for x in q]
        cnt = {o: sum(1 for x in q if x.get("outcome") == o) for o in ("capture", "crash", "timeout")}
        if len(q) != 2048 or len(set(ids)) != 2048 or sum(cnt.values()) != 2048: prob.append("ledger quota %s" % (key,))
        per_env = {}
        for x in q: per_env[x["env_index"]] = per_env.get(x["env_index"], 0) + 1
        if len(per_env) != 128 or set(per_env.values()) != {16}: prob.append("per-env quota %s" % (key,))
        if any(x.get("cell_id") != r["cell_id"] or x.get("seed") != r["seed"] or x.get("bars") != r["density"] for x in q):
            prob.append("row fields %s" % (key,))
        if any(bool(x.get("capture")) + bool(x.get("crash")) + bool(x.get("timeout")) != 1 or not x.get(x["outcome"]) for x in q):
            prob.append("outcome flags %s" % (key,))
        if cnt != r["stats"]["counts"]: prob.append("stats counts %s" % (key,))
        res = json.loads((cd / "result.json").read_text())
        cells[key] = {"n": len(q), "counts": cnt, **{o: cnt[o] / len(q) for o in cnt}, "rows_hash": sha(cd / "episodes.jsonl"),
                      "cc": res.get("crash_causes", {}), "act": res.get("actual_episodes") or 0, "env": base}
    return rep, cells, prob


def pair_stats(F, ev, trs, pc):
    pol = {"F": F, **ev}
    rng = random.Random(20261111)

    def delta(a, la, b, lb, m, r=rng):
        xs = [100.0 * (pol[a][(la, d, s)][m] - pol[b][(lb, d, s)][m]) for d, s in BLOCKS]
        return {"mean_pp": sum(xs) / 12, "ci95_pp": boot_ci(xs, r), "t95_pp": tci(xs), "blocks_pp": xs,
                "n_neg_blocks": sum(1 for x in xs if neg(x))}
    p1 = delta("rand40", "S40", "ctrl", "S40", "crash")
    p2 = delta("rand40", "S00", "ctrl", "S00", "capture")
    p3 = delta("rand40", "S40", "ctrl", "S40", "capture")
    cr = lambda n, lv, d, s: pol[n][(lv, d, s)]["crash"]
    dx = [100.0 * ((cr("rand40", "S40", d, s) - cr("rand40", "S00", d, s)) - (cr("ctrl", "S40", d, s) - cr("ctrl", "S00", d, s)))
          for d, s in BLOCKS]
    did = {"mean_pp": sum(dx) / 12, "ci95_pp": boot_ci(dx, rng), "t95_pp": tci(dx), "blocks_pp": dx,
           "n_neg_blocks": sum(1 for x in dx if neg(x))}
    drift = delta("ctrl", "S40", "F", "S40", "crash")
    binding_ok = (all(not pc["eval"][a]["problems"] for a in ev) and not any(pc["identical_ledgers"].values())
                  and trs["rand40"]["commit"] == trs["ctrl"]["commit"] and all(trs[a]["verdict"] == "TRAINED" for a in trs))
    if not binding_ok: reading = "EVAL_BINDING_FAIL"
    elif p2["mean_pp"] <= -2 and p2["ci95_pp"][1] < 0: reading = "GR1_MATERIAL_LOSS"
    elif p1["mean_pp"] <= -1.5 and p1["ci95_pp"][1] < 0 and p2["ci95_pp"][0] > -2 and p3["ci95_pp"][0] > 0: reading = "GR1_SUPPORTED"
    elif p1["ci95_pp"][0] > -1.5: reading = "GR1_NOT_SUPPORTED"
    else: reading = "INCONCLUSIVE"
    alt = {sd: boot_ci(p1["blocks_pp"], random.Random(sd)) for sd in (1, 2, 3, 12345, 999)}
    pooled = {n: {lv: {m: 100.0 * sum(pol[n][(lv, d, s)][m] for d, s in BLOCKS) / 12 for m in ("capture", "crash", "timeout")}
                  for lv in LEVELS} for n in pol}
    zero = [(n, lv, d, s) for n in ("rand40", "ctrl") for lv in ("S00", "S40") for d, s in BLOCKS if cr(n, lv, d, s) == 0]
    blk_lr = None if zero else [math.log(cr("rand40", "S40", d, s) / cr("rand40", "S00", d, s)) -
                                math.log(cr("ctrl", "S40", d, s) / cr("ctrl", "S00", d, s)) for d, s in BLOCKS]
    pr = lambda n, lv: pooled[n][lv]["crash"]
    pooled_lr = math.log(pr("rand40", "S40") / pr("rand40", "S00")) - math.log(pr("ctrl", "S40") / pr("ctrl", "S00"))
    sec = {}
    for lv in LEVELS:
        for n in ("rand40", "ctrl"):
            r3 = random.Random(7)
            sec["%s_minus_F_%s" % (n, lv)] = {m: {k: v for k, v in delta(n, lv, "F", lv, m, r3).items() if k != "blocks_pp"}
                                              for m in ("capture", "crash")}
    causes = {}
    for n in pol:
        for lv in LEVELS:
            o = {"episodes": 0, "bar_contact": 0, "below": 0, "above": 0, "out_of_bounds": 0}
            for d, s in BLOCKS:
                c = pol[n][(lv, d, s)]; o["episodes"] += c["act"]
                for k in ("bar_contact", "below", "above", "out_of_bounds"):
                    v = c["cc"].get(k, 0); o[k] += int(v) if isinstance(v, (int, float)) else 0
            o["quota_crashes"] = sum(pol[n][(lv, d, s)]["counts"]["crash"] for d, s in BLOCKS)
            o["bar_per_100"] = 100.0 * o["bar_contact"] / o["episodes"]; o["below_per_100"] = 100.0 * o["below"] / o["episodes"]
            causes.setdefault(n, {})[lv] = o
    gaps = {n: sum(100.0 * (cr(n, "S40", d, s) - cr(n, "S00", d, s)) for d, s in BLOCKS) / 12 for n in pol}
    s00diff = sum(100.0 * (cr("rand40", "S00", d, s) - cr("ctrl", "S00", d, s)) for d, s in BLOCKS) / 12
    pc.update({"P1": p1, "P2": p2, "P3": p3, "DiD": did, "drift_ctrl_minus_F_S40": drift, "reading_independent": reading,
               "binding_ok": binding_ok, "P1_ci_other_boot_seeds": alt, "pooled_pct": pooled,
               "crash_ratio_S40_over_S00": {n: pr(n, "S40") / pr(n, "S00") for n in pol},
               "log_ratio_DiD_pooled": pooled_lr, "log_ratio_DiD_block_mean": None if blk_lr is None else sum(blk_lr) / 12,
               "log_ratio_block_neg": None if blk_lr is None else sum(1 for x in blk_lr if neg(x)), "zero_rate_cells": zero,
               "crash_gap_S40_minus_S00_pp": gaps, "F_contrasts": sec, "crash_causes": causes,
               "S00_crash_diff_pp": s00diff, "decomp_check": [p1["mean_pp"], s00diff + did["mean_pp"]]})


def main():
    out = {"model": "claude-opus-5-5 (requested; self-reported)", "checks": {}, "pairs": {}}
    F_rep, F, fprob = audit_eval(GR0, "gr0_run_report.json", None)
    out["checks"]["F_gr0_main"] = {"commit": F_rep["commit"], "problems": fprob, "host_start": F_rep["host_start"],
                                   "host_end": F_rep["host_end"], "import_origins": F_rep.get("import_origins")}
    for pn, pp in PAIRS.items():
        trs = {a: json.loads(pp["tr_" + a].read_text()) for a in ("rand40", "ctrl")}
        pc = {"train": {}, "eval": {}}
        ev = {}
        for a in ("rand40", "ctrl"):
            tr = trs[a]; ck = tr["checkpoint"]
            orig = P(ck["run_dir"]) / ck["file"]
            slog = pp["tr_" + a].parent / tr["session_log"]["file"]
            pc["train"][a] = {"verdict": tr["verdict"], "commit": tr["commit"], "prereg": tr.get("prereg"), "exit": tr["exit"],
                              "copy_sha_actual": sha(ck["copy"]), "copy_sha_reported": ck["copy_sha256"],
                              "orig_sha_actual": sha(orig) if orig.exists() else "MISSING", "orig_sha_reported": ck["sha256"],
                              "session_log_sha_ok": sha(slog) == tr["session_log"]["sha256"],
                              "host_faults": [tr["host_start"]["fault_count"], tr["host_end"]["fault_count"]],
                              "boot_same": tr["host_start"]["boot_id"] == tr["host_end"]["boot_id"],
                              "problems": tr.get("problems"), "preflight": tr.get("preflight_problems"),
                              "new_run_dirs": tr.get("new_run_dirs"), "wall_min": tr["wall_s"] / 60}
            rep, cells, prob = audit_eval(pp[a], "gr1eval_run_report.json", ck["copy_sha256"])
            if (rep.get("policy") != a or rep.get("checkpoint_sha256") != ck["copy_sha256"] or
                    P(rep["train_report"]).resolve() != pp["tr_" + a].resolve() or P(rep["checkpoint"]).resolve() != P(ck["copy"]).resolve()):
                prob.append("eval/train binding")
            ev[a] = cells
            pc["eval"][a] = {"commit": rep["commit"], "report_sha": sha(pp[a] / "gr1eval_run_report.json"),
                             "repro_report": rep.get("repro_report"), "import_origins": rep.get("import_origins"),
                             "host_start": rep["host_start"], "host_end": rep["host_end"], "problems": prob,
                             "prereg_field": rep.get("prereg"), "n_cells": len(cells),
                             "in_quota_per_cell": sorted({c["n"] for c in cells.values()}),
                             "in_quota_total": sum(c["n"] for c in cells.values())}
        ident = {"rand40_eq_F": 0, "ctrl_eq_F": 0, "rand40_eq_ctrl": 0}
        envdiff = {"rand40_vs_ctrl": set(), "rand40_vs_F": set(), "ctrl_vs_F": set()}
        for k in sorted(GRID):
            ident["rand40_eq_F"] += ev["rand40"][k]["rows_hash"] == F[k]["rows_hash"]
            ident["ctrl_eq_F"] += ev["ctrl"][k]["rows_hash"] == F[k]["rows_hash"]
            ident["rand40_eq_ctrl"] += ev["rand40"][k]["rows_hash"] == ev["ctrl"][k]["rows_hash"]
            for nm, e, o in (("rand40_vs_ctrl", ev["rand40"][k]["env"], ev["ctrl"][k]["env"]),
                             ("rand40_vs_F", ev["rand40"][k]["env"], F[k]["env"]), ("ctrl_vs_F", ev["ctrl"][k]["env"], F[k]["env"])):
                envdiff[nm].update(x for x in set(e).union(o) if e.get(x) != o.get(x))
        pc["identical_ledgers"] = ident
        pc["env_base_keys_differing"] = {k: sorted(v) for k, v in envdiff.items()}
        pair_stats(F, ev, trs, pc)
        out["pairs"][pn] = pc
    s1, s2 = out["pairs"]["s1"], out["pairs"]["s2"]
    out["across_pairs"] = {q: {"s1": s1[q]["mean_pp"], "s2": s2[q]["mean_pp"], "mean": (s1[q]["mean_pp"] + s2[q]["mean_pp"]) / 2,
                               "s2_minus_s1": s2[q]["mean_pp"] - s1[q]["mean_pp"]} for q in ("P1", "P2", "P3", "DiD")}
    for q in ("log_ratio_DiD_block_mean", "log_ratio_DiD_pooled"):
        a, b = s1[q], s2[q]
        out["across_pairs"][q] = {"s1": a, "s2": b, "mean": None if None in (a, b) else (a + b) / 2,
                                  "s2_minus_s1": None if None in (a, b) else b - a}
    src = {}
    for rel in ("tools/gr1_analyse.py", "tools/gr_analyse.py", "tools/gr_run.py", "tools/gr1b_train.py", "tools/gr1_train.py",
                "tools/kroc2027_eval.py", "aerial_gym/rl_training/rl_games/train_navrl_gr1.sh",
                "aerial_gym/rl_training/rl_games/train_navrl_gr1b.sh"):
        src[rel] = sha(TREE / rel) if (TREE / rel).exists() else "MISSING"
    out["source_sha"] = src
    for a, b, nm in (("aerial_gym/rl_training/rl_games/train_navrl_gr1.sh", "aerial_gym/rl_training/rl_games/train_navrl_gr1b.sh", "launcher"),
                     ("tools/gr1_train.py", "tools/gr1b_train.py", "trainer")):
        out["diff_" + nm] = list(difflib.unified_diff((TREE / a).read_text().splitlines(), (TREE / b).read_text().splitlines(),
                                                      lineterm="", n=0))
    rp = json.loads((B / "repro/gr1repro_report.json").read_text())
    out["repro"] = {"verdict": rp["verdict"], "commit": rp["commit"], "faults": [rp["host_start"]["fault_count"], rp["host_end"]["fault_count"]],
                    "S40_eps_sha": sha(B / "repro/S40/E2_obstacle_aware__205bars__seed4501/episodes.jsonl"),
                    "S40_gr1l_eps_sha": sha(B / "repro/S40_gr1launcher/E2_obstacle_aware__205bars__seed4501/episodes.jsonl"),
                    "gr0_eps_sha": sha(GR0 / "S40/E2_obstacle_aware__205bars__seed4501/episodes.jsonl")}
    with (OUT / "audit_results.json").open("x") as fh:
        fh.write(json.dumps(out, indent=1, default=str) + "\n")
    print("done")


if __name__ == "__main__":
    main()
