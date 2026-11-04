"""CPU smoke test of tools/camonly_run.py (c1, c2) and tools/camonly_c2_analyse.py, end to end (no simulator, no GPU).

Stubs only the preflight (tree, GPU) and gr_run.run_cell (the simulator); run_cell copies the real GR-0 S00 cell,
writes the ablation receipt the entry point would write and the condition into the receipt's NAVRL snapshot.
`no_lidar` turns 30 % of the quota captures into crashes (a known effect); the other conditions only nudge one
ledger field (identical rates, different bytes).
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
        rows[0]["min_relative_distance_m"] = rows[0].get("min_relative_distance_m", 0.0) + 1e-6
        if cond == "no_lidar":
            rng = random.Random(density * 10 + seed)
            for x in rows:
                if x.get("in_quota") and x.get("outcome") == "capture" and rng.random() < 0.3:
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
# analysis: known-good
a = analyse(P / "c1", P / "c2", WORK / "analysis_ok.json")
con = a["out"].get("contrasts", {})
res["analysis_ok"] = {"rc": a["rc"], "reading": a["out"].get("reading"), "stderr": a["stderr"],
                      "labels": {k: (v["capture"]["label"], v["crash"]["label"], round(v["capture"]["mean_pp"], 2))
                                 for k, v in con.items()}}
ok["analysis_known_good"] = (a["rc"] == 0 and a["out"].get("reading") == "REPORTED"
                             and con["total"]["capture"]["label"] == "LOWER" and con["total"]["crash"]["label"] == "HIGHER"
                             and con["gov_all"]["capture"]["label"] == "LOWER"
                             and con["in_fov"]["capture"]["label"] == "NO_DETECTED_CHANGE"
                             and con["obs_in"]["capture"]["label"] == "NO_DETECTED_CHANGE")
# analysis: a receipt naming another condition
cell = P / "c2" / "no_lidar_out_fov" / "E2_obstacle_aware__115bars__seed4502" / "sensor_ablation_receipt.json"
saved = cell.read_text()
cell.write_text(saved.replace('"no_lidar_out_fov"', '"no_lidar"'))
a = analyse(P / "c1", P / "c2", WORK / "analysis_bad_receipt.json")
cell.write_text(saved)
ok["analysis_bad_receipt"] = a["out"].get("reading") == "EVAL_BINDING_FAIL" and not a["out"]["binding"]["ablation_receipts_match"]
# analysis: a C2 cell identical to full
ep = P / "c2" / "no_lidar_in_fov" / "E2_obstacle_aware__70bars__seed4501" / "episodes.jsonl"
saved = ep.read_bytes()
shutil.copy(GR0 / "S00" / "E2_obstacle_aware__70bars__seed4501" / "episodes.jsonl", ep)
a = analyse(P / "c1", P / "c2", WORK / "analysis_identical.json")
ep.write_bytes(saved)
ok["analysis_identical_cell"] = a["out"].get("reading") == "EVAL_BINDING_FAIL" and not a["out"]["binding"]["cells_differ_from_full"]
out = {"tree": str(TREE), "runner_sha256": hashlib.sha256((TREE / "tools/camonly_run.py").read_bytes()).hexdigest(),
       "analysis_sha256": hashlib.sha256((TREE / "tools/camonly_c2_analyse.py").read_bytes()).hexdigest(),
       "results": res, "checks": ok, "pass": all(ok.values())}
OUT.write_text(json.dumps(out, indent=1, default=str) + "\n")
print(json.dumps({"checks": ok, "pass": out["pass"], "labels": res.get("analysis_ok", {}).get("labels"),
                  "errors": {k: v.get("error") for k, v in res.items() if isinstance(v, dict) and v.get("error")}},
                 indent=1))
raise SystemExit(0 if out["pass"] else 1)
