"""CPU smoke test of tools/camonly_run.py (c1, c2) and tools/camonly_c2_analyse.py, end to end (no simulator, no GPU).

Stubs only the preflight (tree, GPU) and gr_run.run_cell (the simulator); run_cell copies the real GR-0 S00 cell,
writes the ablation receipt the entry point would write and the condition into the receipt's NAVRL snapshot.
Each condition turns a different share of the quota captures into crashes (amendment 1, L8), so a swapped
condition mapping changes the contrasts; the analysis's contrast means are compared with means computed here from
the cells themselves. Known-bad cases: a dead governor, an environment mismatch, a GR-0 copy, a C1 / C2 no_lidar
mismatch and an unexpected snapshot key.
usage: smoke_c1c2.py TREE WORK_DIR OUT_JSON
"""
import hashlib
import json
import pathlib
import random
import shutil
import subprocess
import sys

TREE, WORK, OUT = (pathlib.Path(a).resolve() for a in sys.argv[1:4])
GR0 = pathlib.Path("/home/fair/workspaces/aerial_gym_ws/src/aerial_gym_simulator/results/gain_randomization_gr0_v1/gr0_main")
sys.path.insert(0, str(TREE / "tools"))
import camonly_run as R                                       # noqa: E402

if WORK.exists():
    raise SystemExit("work dir exists")
WORK.mkdir(parents=True)
CALLS = []
MODE = {"missing_receipt": False, "wrong_condition_at": None, "no_effect": False}
MODULE_SHA = hashlib.sha256((TREE / R.MODULE).read_bytes()).hexdigest()
EFFECT = {"full": 0.0, "no_lidar_in_fov": 0.05, "no_lidar_out_fov": 0.10, "no_lidar": 0.30, "no_lidar_obs_only": 0.20,
          "no_lidar_in_fov_obs_only": 0.02}


def fake_preflight(protocol, approved, out_root):
    return "HEADSMOKE", (["output already exists"] if out_root.exists() else []), {}


def rows_of(p):
    return [json.loads(l) for l in p.read_text().splitlines() if l.strip()]


def fake_run_cell(root, protocol, out_root, level, density, seed, quota, run_id, scale, head, start, records,
                  timeout_s, extra=None, launcher=None, ckpt_sha=None):
    cid = "E2_obstacle_aware__%dbars__seed%d" % (density, seed)
    dst = out_root / level / cid
    dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.copytree(GR0 / "S00" / cid, dst)
    cond = extra["NAVRL_SENSOR_ABLATION"]
    CALLS.append({"level": level, "density": density, "seed": seed, "scale": scale, "quota": quota, "run_id": run_id,
                  "launcher": launcher, "ckpt_sha": ckpt_sha, "extra": dict(extra)})
    if cond != "full" and not (cond == "no_lidar" and MODE["no_effect"]):
        rows = rows_of(dst / "episodes.jsonl")
        rows[0]["min_relative_distance_m"] = rows[0].get("min_relative_distance_m", 0.0) + 1e-6 * (1 + len(cond))
        rng = random.Random(density * 10 + seed)
        for x in rows:
            if x.get("in_quota") and x.get("outcome") == "capture" and rng.random() < EFFECT[cond]:
                x.update(outcome="crash", capture=False, crash=True)
        (dst / "episodes.jsonl").write_text("".join(json.dumps(x, sort_keys=True) + "\n" for x in rows))
    if not MODE["missing_receipt"]:
        written = cond
        if MODE["wrong_condition_at"] is not None and len(CALLS) == MODE["wrong_condition_at"]:
            written = "full"
        (dst / "sensor_ablation_receipt.json").write_text(json.dumps({
            "condition": written, "module_sha256": MODULE_SHA, "module": str(TREE / R.MODULE),
            "perception": str(TREE / "aerial_gym/task/navrl_task/navrl_perception.py"),
            "task": str(TREE / "aerial_gym/task/navrl_task/navrl_task.py"), "hbeams": 72, "vbeams": 4}))
    receipt = json.loads((dst / "receipt.json").read_text())
    receipt.setdefault("env_snapshot_navrl", {})["NAVRL_SENSOR_ABLATION"] = cond
    (dst / "receipt.json").write_text(json.dumps(receipt))
    rec = {"level": level, "cell_id": cid, "density": density, "seed": seed, "verdict": "OK", "problems": []}
    records.append(rec)
    return rec


R.preflight = fake_preflight
R.G.run_cell = fake_run_cell
R.G.host = lambda: {"fault_count": 0}


def main_with(argv):
    CALLS.clear()
    sys.argv = ["camonly_run.py"] + argv
    try:
        rc = R.main()
    except Exception as e:
        return {"rc": "EXCEPTION", "error": "%s: %s" % (type(e).__name__, e)}
    return {"rc": rc, "calls": list(CALLS)}


def rep(path):
    return json.loads(path.read_text()) if path.is_file() else {}


def analyse(c1, c2, out):
    r = subprocess.run([sys.executable, "-B", str(TREE / "tools/camonly_c2_analyse.py"), str(GR0), str(c1), str(c2),
                        str(out)], capture_output=True, text=True,
                       env={"PATH": "/usr/bin:/bin", "PYTHONNOUSERSITE": "1", "PYTHONDONTWRITEBYTECODE": "1"})
    return {"rc": r.returncode, "out": rep(out), "stderr": r.stderr[-800:]}


ok, res = {}, {}
base = ["--approved-commit", "x", "--gr0-dir", str(GR0)]
P = WORK / "runs"
P.mkdir()
# runner: c1 normal
r = main_with(["c1"] + base + ["--out", str(P / "c1")])
c1rep = rep(P / "c1" / "c1_report.json")
calls = r.get("calls") or []
res["c1_pass"] = {"rc": r["rc"], "verdict": c1rep.get("verdict"), "error": r.get("error"), "calls": calls}
ok["c1_pass"] = (r["rc"] == 0 and c1rep.get("verdict") == "C1_IDENTITY_PASS" and len(calls) == 2
                 and [c["extra"]["NAVRL_SENSOR_ABLATION"] for c in calls] == ["full", "no_lidar"]
                 and all(c["launcher"] == R.LAUNCHER and c["ckpt_sha"] == R.G.F_SHA and c["scale"] is None
                         and c["quota"] == 16 and c["density"] == 205 and c["seed"] == 4501
                         and c["extra"]["CAMONLY_PY"] == str(TREE / "tools/camonly_python") for c in calls))
# runner: switch without effect -> C1 fail
MODE["no_effect"] = True
r = main_with(["c1"] + base + ["--out", str(WORK / "neg" / "c1_noeffect")])
MODE["no_effect"] = False
ok["c1_no_effect_fails"] = r["rc"] == 1 and rep(WORK / "neg" / "c1_noeffect" / "c1_report.json").get("verdict") == "C1_IDENTITY_FAIL"
# runner: missing ablation receipt -> cell FAIL -> C1 fail
MODE["missing_receipt"] = True
r = main_with(["c1"] + base + ["--out", str(WORK / "neg2" / "c1_noreceipt")])
MODE["missing_receipt"] = False
c = rep(WORK / "neg2" / "c1_noreceipt" / "c1_report.json")
ok["c1_missing_receipt_fails"] = (r["rc"] == 1 and c.get("verdict") == "C1_IDENTITY_FAIL"
                                  and all(x["verdict"] == "FAIL" for x in c.get("records", [])))
# runner: c2 without a c1 pass in the parent
r = main_with(["c2"] + base + ["--out", str(WORK / "neg3" / "c2")])
ok["c2_needs_c1"] = r["rc"] == 2 and not (WORK / "neg3" / "c2").exists()
# runner: wrong condition in a receipt stops c2 at that cell
(WORK / "neg4").mkdir()
shutil.copytree(P / "c1", WORK / "neg4" / "c1")
MODE["wrong_condition_at"] = 7
r = main_with(["c2"] + base + ["--out", str(WORK / "neg4" / "c2")])
MODE["wrong_condition_at"] = None
c = rep(WORK / "neg4" / "c2" / "c2_run_report.json")
ok["c2_wrong_condition_stops"] = r["rc"] == 1 and c.get("verdict") == "INCOMPLETE" and len(c.get("records", [])) == 7
shutil.rmtree(WORK / "neg4")
# runner: c2 normal
r = main_with(["c2"] + base + ["--out", str(P / "c2")])
c2rep = rep(P / "c2" / "c2_run_report.json")
calls = r.get("calls") or []
order = [(c["density"], c["seed"], c["extra"]["NAVRL_SENSOR_ABLATION"]) for c in calls]
want = [(d, s, cond) for d in R.G.DENSITIES for s in R.G.SEEDS for cond in R.C2_CONDITIONS]
res["c2"] = {"rc": r["rc"], "verdict": c2rep.get("verdict"), "n_calls": len(calls), "error": r.get("error")}
ok["c2_complete_and_ordered"] = r["rc"] == 0 and c2rep.get("verdict") == "COMPLETE" and order == want
# analysis: known-good, with contrast means checked against an independent computation from the cells
a = analyse(P / "c1", P / "c2", WORK / "analysis_ok.json")
con = a["out"].get("contrasts", {})
res["analysis_ok"] = {"rc": a["rc"], "reading": a["out"].get("reading"), "stderr": a["stderr"],
                      "binding": a["out"].get("binding"),
                      "labels": {k: (v["capture"]["label"], v["crash"]["label"], round(v["capture"]["mean_pp"], 3))
                                 for k, v in con.items()}}
KEYS = {"A": "full", "B_in": "no_lidar_in_fov", "B_out": "no_lidar_out_fov", "C": "no_lidar", "C_gL": "no_lidar_obs_only",
        "B_in_gL": "no_lidar_in_fov_obs_only"}
WEIGHTS = {"total": {"C": 1, "A": -1}, "in_fov": {"B_in": 1, "A": -1}, "out_fov": {"B_out": 1, "A": -1},
           "interaction": {"C": 1, "B_in": -1, "B_out": -1, "A": 1}, "obs_in": {"B_in_gL": 1, "A": -1},
           "gov_in": {"B_in": 1, "B_in_gL": -1}, "gov_all": {"C": 1, "C_gL": -1}, "obs_all": {"C_gL": 1, "A": -1}}


def cap_rate(key, d, s):
    base = (GR0 / "S00") if key == "A" else (P / "c2" / KEYS[key])
    q = [x for x in rows_of(base / ("E2_obstacle_aware__%dbars__seed%d" % (d, s)) / "episodes.jsonl") if x.get("in_quota")]
    return sum(1 for x in q if x.get("outcome") == "capture") / len(q)


blocks = [(d, s) for d in R.G.DENSITIES for s in R.G.SEEDS]
indep = {c: 100.0 * sum(sum(w * cap_rate(k, d, s) for k, w in ws.items()) for d, s in blocks) / len(blocks)
         for c, ws in WEIGHTS.items()}
res["independent_capture_means"] = indep
ok["analysis_known_good"] = (a["rc"] == 0 and a["out"].get("reading") == "REPORTED" and set(con) == set(WEIGHTS)
                             and all(abs(con[c]["capture"]["mean_pp"] - indep[c]) < 1e-9 for c in WEIGHTS)
                             and con["total"]["capture"]["label"] == "LOWER" and con["total"]["crash"]["label"] == "HIGHER"
                             and "speed_governor" in a["out"])


def bad(name, edit, restore):
    edit()
    try:
        r = analyse(P / "c1", P / "c2", WORK / ("analysis_%s.json" % name))
    finally:
        restore()
    return r["out"].get("reading") == "EVAL_BINDING_FAIL", r["out"].get("binding")


def swap_file(path, new_bytes):
    old = path.read_bytes()
    return (lambda: path.write_bytes(new_bytes)), (lambda: path.write_bytes(old))


cell = P / "c2" / "no_lidar_out_fov" / "E2_obstacle_aware__115bars__seed4502" / "sensor_ablation_receipt.json"
e, r_ = swap_file(cell, cell.read_bytes().replace(b'"no_lidar_out_fov"', b'"no_lidar"'))
ok["analysis_bad_receipt"], _ = bad("bad_receipt", e, r_)
ep = P / "c2" / "no_lidar_in_fov" / "E2_obstacle_aware__70bars__seed4501" / "episodes.jsonl"
e, r_ = swap_file(ep, (GR0 / "S00" / "E2_obstacle_aware__70bars__seed4501" / "episodes.jsonl").read_bytes())
ok["analysis_identical_cell"], _ = bad("identical", e, r_)
# dead governor: B_in_gL equal to B_in in one block (amendment 1, M2)
ep2 = P / "c2" / "no_lidar_in_fov_obs_only" / "E2_obstacle_aware__160bars__seed4503" / "episodes.jsonl"
e, r_ = swap_file(ep2, (P / "c2" / "no_lidar_in_fov" / "E2_obstacle_aware__160bars__seed4503" / "episodes.jsonl").read_bytes())
ok["analysis_dead_governor"], b_ = bad("dead_governor", e, r_)
ok["analysis_dead_governor"] &= b_ is not None and not b_["governor_ablation_engaged"]
# unexpected snapshot key (amendment 1, M3)
rc = P / "c2" / "no_lidar" / "E2_obstacle_aware__205bars__seed4503" / "receipt.json"
j = json.loads(rc.read_text()); j["env_snapshot_navrl"]["NAVRL_SOMETHING_ELSE"] = "1"
e, r_ = swap_file(rc, json.dumps(j).encode())
ok["analysis_unexpected_snapshot_key"], b_ = bad("snapshot", e, r_)
ok["analysis_unexpected_snapshot_key"] &= b_ is not None and not b_["snapshots_differ_only_in_expected_keys"]
# C1 / C2 no_lidar mismatch (amendment 1, L9)
c1nl = P / "c1" / "no_lidar" / "E2_obstacle_aware__205bars__seed4501" / "episodes.jsonl"
rows = rows_of(c1nl); rows[1]["min_relative_distance_m"] = rows[1].get("min_relative_distance_m", 0.0) + 1e-3
e, r_ = swap_file(c1nl, "".join(json.dumps(x, sort_keys=True) + "\n" for x in rows).encode())
ok["analysis_c1_c2_mismatch"], b_ = bad("c1c2", e, r_)
ok["analysis_c1_c2_mismatch"] &= b_ is not None and not b_["c1_no_lidar_equals_c2"]
# a GR-0 copy elsewhere is not the manifest's (amendment 1, L1)
fake = WORK / "gr0_fake"
(fake / "S00").mkdir(parents=True)
shutil.copy(GR0 / "gr0_run_report.json", fake / "gr0_run_report.json")
for d, s in blocks:
    shutil.copytree(GR0 / "S00" / ("E2_obstacle_aware__%dbars__seed%d" % (d, s)), fake / "S00" / ("E2_obstacle_aware__%dbars__seed%d" % (d, s)))
rr = subprocess.run([sys.executable, "-B", str(TREE / "tools/camonly_c2_analyse.py"), str(fake), str(P / "c1"), str(P / "c2"),
                     str(WORK / "analysis_fake_gr0.json")], capture_output=True, text=True,
                    env={"PATH": "/usr/bin:/bin", "PYTHONNOUSERSITE": "1", "PYTHONDONTWRITEBYTECODE": "1"})
fo = rep(WORK / "analysis_fake_gr0.json")
ok["analysis_fake_gr0"] = fo.get("reading") == "EVAL_BINDING_FAIL" and not fo["binding"]["gr0_s00_cells_match_manifest"]
# runner: c2 refuses a runner environment that differs from C1's (amendment 1, M3)
(WORK / "neg5").mkdir()
shutil.copytree(P / "c1", WORK / "neg5" / "c1")
c1j = json.loads((WORK / "neg5" / "c1" / "c1_report.json").read_text()); c1j["env_allowlisted"]["PATH"] = "/other"
(WORK / "neg5" / "c1" / "c1_report.json").write_text(json.dumps(c1j))
r = main_with(["c2"] + base + ["--out", str(WORK / "neg5" / "c2")])
ok["c2_env_mismatch_refused"] = r["rc"] == 2 and not (WORK / "neg5" / "c2").exists()
out = {"tree": str(TREE), "runner_sha256": hashlib.sha256((TREE / "tools/camonly_run.py").read_bytes()).hexdigest(),
       "analysis_sha256": hashlib.sha256((TREE / "tools/camonly_c2_analyse.py").read_bytes()).hexdigest(),
       "results": res, "checks": ok, "pass": all(ok.values())}
OUT.write_text(json.dumps(out, indent=1, default=str) + "\n")
print(json.dumps({"checks": ok, "pass": out["pass"], "labels": res.get("analysis_ok", {}).get("labels"),
                  "errors": {k: v.get("error") for k, v in res.items() if isinstance(v, dict) and v.get("error")}},
                 indent=1))
raise SystemExit(0 if out["pass"] else 1)
