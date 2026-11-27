"""Independent re-derivation of the T-LOAD reading from raw files (reviewer r1; read-only, stdlib only)."""
import hashlib
import json
import os
import pathlib
import sys

RES = pathlib.Path("/home/fair/workspaces/aerial_gym_ws/src/aerial_gym_simulator/results")
OUT = RES / "tload_v1"
EVID = pathlib.Path("/home/fair/workspaces/aerial_gym_ws/src/MOTAR-camonly/docs/evidence/tload_2026-11-05")
COMMIT = "812f6b1bb9404600a6cae96269851174f1319511"
F_SHA = "f702213936601860995cf61dcc570247e72543b1976e3716055cd8ec5593ad40"
MODULE_SHA = "ea9ccacbc79918e3080486d47d63fed21569628657a20b240d198521036ff894"
CELLS = (("full", 205, 4501), ("no_lidar", 205, 4501), ("full", 70, 4501))
ARMS = ("L0a", "L1", "L2", "L3", "L0b")
CID = "E2_obstacle_aware__%dbars__seed%d"
PREREG_SHA = {  # prereg section 4
    ("full", 205): ("8899856375b19c6d1865e0f807827e479077325045bf82f1c44ab3113f77e6ad",
                    "3eb439456a0e7055994705db994523bdc0ead956effe8c2242ad35cfb99c0777"),
    ("no_lidar", 205): ("4a161f9eb0f0792f14dc3de6e1432a5786a099c661fc461bf69d8f4fd0119a3b",
                        "adc872d7015cded00f7017836ca31942c556c0138f981d6716b8f40b07841dd0"),
    ("full", 70): ("11e7ec818586df7b6f6859c2dacdc0a167882650e3e8e5a4e22f5dc316389543",
                   "9f96ecf1edbbc6bfdf03264a587189c6331834429797b3d03fe7ab0fabb37d73"),
}
REFS = {
    ("full", 205): {"gate": ["camonly_c1c2_v1/c1/full", "camonly_c1c2_v1/c1_r2/full"],
                    "report": ["gain_randomization_gr0_v1/gr0_main/S00"]},
    ("no_lidar", 205): {"gate": ["camonly_c1c2_v1/c1/no_lidar", "camonly_c1c2_v1/c1_r2/no_lidar",
                                 "camonly_c1c2_v1/c2/no_lidar"], "report": []},
    ("full", 70): {"gate": [], "report": ["gain_randomization_gr0_v1/gr0_main/S00"]},
}
FILES = ("episodes.jsonl", "kroc2027_episodes.jsonl")


def sha(p):
    h = hashlib.sha256()
    with open(p, "rb") as f:
        for b in iter(lambda: f.read(1 << 20), b""):
            h.update(b)
    return h.hexdigest()


def comparable_result(d):
    r = json.loads((d / "result.json").read_text())
    r = dict(r)
    cond = dict(r.pop("condition"))
    cond.pop("evaluation_nonce", None)
    r["condition"] = cond
    r.pop("checkpoint", None)
    return r


def equal(a, b):
    ha = {f: sha(a / f) if (a / f).is_file() else None for f in FILES}
    hb = {f: sha(b / f) if (b / f).is_file() else None for f in FILES}
    files = None not in ha.values() and ha == hb
    res = comparable_result(a) == comparable_result(b)
    return {"files_equal": files, "result_equal": res, "equal": files and res, "a": ha, "b": hb}


def main():
    out = {"runs": [], "problems": []}
    P = out["problems"]
    rep = json.loads((EVID / "tload_report.json").read_text())
    raw_rep = OUT / "tload_report.json"
    out["report_committed_equals_raw"] = sha(raw_rep) == sha(EVID / "tload_report.json")
    rep_runs = {r["idx"]: r for r in rep["runs"]}
    idx = 0
    dirs = {}
    inodes = {}
    nonces = {}
    for cond, d, s in CELLS:
        for arm in ARMS:
            idx += 1
            cdir = OUT / ("%02d_%s" % (idx, arm)) / cond / (CID % (d, s))
            dirs[(cond, d, arm)] = cdir
            row = {"idx": idx, "arm": arm, "cell": [cond, d, s], "dir": str(cdir)}
            for f in FILES + ("result.json",):
                p = cdir / f
                if p.is_symlink():
                    P.append("symlink %s" % p)
                st = os.stat(p)
                key = (st.st_dev, st.st_ino)
                if key in inodes:
                    P.append("shared inode %s %s" % (p, inodes[key]))
                inodes[key] = str(p)
            row["sha"] = {f: sha(cdir / f) for f in FILES}
            rc = json.loads((cdir / "receipt.json").read_text())
            ab = json.loads((cdir / "sensor_ablation_receipt.json").read_text())
            res = json.loads((cdir / "result.json").read_text())
            row["receipt"] = {"status": rc.get("status"), "git_commit": rc.get("git_commit"),
                              "dirty": rc.get("git_dirty_files"), "ckpt": rc.get("checkpoint_sha256"),
                              "snap_ablation": rc.get("env_snapshot_navrl", {}).get("NAVRL_SENSOR_ABLATION"),
                              "snap_bars": rc.get("env_snapshot_navrl", {}).get("NAVRL_NUM_BARS"),
                              "snap_seed": rc.get("env_snapshot_navrl", {}).get("NAVRL_SEED"),
                              "snap_quota": rc.get("env_snapshot_navrl", {}).get("NAVRL_EPISODE_QUOTA_PER_ENV"),
                              "snap_backend": rc.get("env_snapshot_navrl", {}).get("NAVRL_TARGET_DISTANCE_BACKEND"),
                              "snap_gain": {k: v for k, v in rc.get("env_snapshot_navrl", {}).items()
                                            if k.startswith("NAVRL_GAIN")},
                              "started_at": rc.get("started_at"), "finished_at": rc.get("finished_at"),
                              "duration_s": rc.get("duration_s"),
                              "result_sha_ok": rc.get("result_sha256") == sha(cdir / "result.json"),
                              "ablation_condition": ab.get("condition"), "ablation_module_sha": ab.get("module_sha256"),
                              "beams": (ab.get("hbeams"), ab.get("vbeams"))}
            r = row["receipt"]
            if not (r["status"] == "OK" and r["git_commit"] == COMMIT and r["dirty"] == 0 and r["ckpt"] == F_SHA
                    and r["snap_ablation"] == cond and r["ablation_condition"] == cond
                    and r["ablation_module_sha"] == MODULE_SHA and r["beams"] == (72, 4)
                    and r["snap_bars"] == str(d) and r["snap_seed"] == str(s) and r["snap_quota"] == "16"
                    and r["snap_backend"] == "direct_l2_v1" and not r["snap_gain"] and r["result_sha_ok"]):
                P.append("receipt problem run %d: %s" % (idx, r))
            nonce = res.get("condition", {}).get("evaluation_nonce")
            if nonce in nonces:
                P.append("nonce reused run %d and %s" % (idx, nonces[nonce]))
            nonces[nonce] = idx
            row["nonce"] = nonce
            rows = [json.loads(l) for l in (cdir / "episodes.jsonl").read_text().splitlines() if l.strip()]
            q = [x for x in rows if x.get("in_quota")]
            row["episodes_rows"] = len(rows)
            row["quota_rows"] = len(q)
            row["outcomes"] = {o: sum(1 for x in q if x.get("outcome") == o) for o in ("capture", "crash", "timeout")}
            row["result_outcome"] = res.get("outcome")
            psha = PREREG_SHA[(cond, d)]
            row["matches_prereg_sha"] = (row["sha"]["episodes.jsonl"], row["sha"]["kroc2027_episodes.jsonl"]) == psha
            rr = rep_runs[idx]
            row["report_digest_matches"] = rr["digest"] == row["sha"]
            row["report_dir_matches"] = rr["dir"] == str(cdir)
            if not row["report_digest_matches"] or not row["report_dir_matches"]:
                P.append("report digest/dir mismatch run %d" % idx)
            out["runs"].append(row)
    # equality vs L0a and references
    for cond, d, s in CELLS:
        l0a = dirs[(cond, d, "L0a")]
        for arm in ARMS[1:]:
            e = equal(l0a, dirs[(cond, d, arm)])
            out.setdefault("eq_L0a", {})["%s/%d/%s" % (cond, d, arm)] = {k: e[k] for k in ("files_equal", "result_equal", "equal")}
        for role in ("gate", "report"):
            for rel in REFS[(cond, d)][role]:
                ref = RES / rel / (CID % (d, s))
                e = equal(ref, l0a)
                out.setdefault("refs", {})["%s/%d %s %s" % (cond, d, role, rel)] = {
                    **{k: e[k] for k in ("files_equal", "result_equal", "equal")},
                    "ref_matches_prereg_sha": (e["a"]["episodes.jsonl"], e["a"]["kroc2027_episodes.jsonl"]) == PREREG_SHA[(cond, d)]}
    # positive controls: the comparator detects real differences
    pc = {}
    pc["full205_vs_nolidar205"] = {k: v for k, v in equal(dirs[("full", 205, "L0a")], dirs[("no_lidar", 205, "L0a")]).items() if k in ("files_equal", "result_equal", "equal")}
    pc["full205_vs_full70"] = {k: v for k, v in equal(dirs[("full", 205, "L0a")], dirs[("full", 70, "L0a")]).items() if k in ("files_equal", "result_equal", "equal")}
    # result.json raw bytes differ between twins (nonce) -> comparator strips only the nonce/checkpoint
    a = json.loads((dirs[("full", 205, "L0a")] / "result.json").read_text())
    b = json.loads((dirs[("full", 205, "L2")] / "result.json").read_text())
    diff_keys = sorted(k for k in set(a) | set(b) if a.get(k) != b.get(k))
    diff_cond = sorted(k for k in set(a["condition"]) | set(b["condition"]) if a["condition"].get(k) != b["condition"].get(k))
    pc["raw_result_diff_L0a_vs_L2_full205"] = {"top": diff_keys, "condition": diff_cond}
    pc["checkpoint_field"] = (a.get("checkpoint"), b.get("checkpoint"))
    out["positive_controls"] = pc
    # reading recomputed
    eq = out["eq_L0a"]
    refs = out["refs"]
    gate_ok = all(v["equal"] for k, v in refs.items() if " gate " in k)
    idle = all(eq["%s/%d/L0b" % (c, d)]["equal"] for c, d, _ in CELLS)
    arms = {a: all(eq["%s/%d/%s" % (c, d, a)]["equal"] for c, d, _ in CELLS) for a in ("L1", "L2", "L3")}
    out["recomputed"] = {"gate_ok": gate_ok, "idle_ok": idle, "arms_equal": arms}
    # manifest
    man = json.loads((EVID / "SHA256_MANIFEST_tload_v1.json").read_text())
    out["manifest_keys"] = sorted(k for k in man if not isinstance(man[k], (dict, list)))[:20] if isinstance(man, dict) else None
    print(json.dumps(out, indent=1, default=str))


if __name__ == "__main__":
    main()
