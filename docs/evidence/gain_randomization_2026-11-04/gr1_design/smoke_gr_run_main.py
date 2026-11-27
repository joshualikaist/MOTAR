"""CPU smoke test of tools/gr_run.py main() for the gr1repro and gr1eval stages (no simulator, no GPU).

Stubs only preflight() (clean-tree / GPU checks) and run_cell() (the simulator). run_cell copies the matching real
GR-0 cell into the expected output folder, or a perturbed copy, and records the arguments it was called with.
Everything else in main() runs as committed, including identity_compare and the training-report binding.
usage: smoke_gr_run_main.py TREE WORK_DIR [GR_RUN_SOURCE]
GR_RUN_SOURCE (optional) runs another version of gr_run.py as if it were TREE/tools/gr_run.py (known-bad control).
"""
import json
import pathlib
import shutil
import sys

TREE, WORK = pathlib.Path(sys.argv[1]).resolve(), pathlib.Path(sys.argv[2]).resolve()
GR0 = pathlib.Path("/home/fair/workspaces/aerial_gym_ws/src/aerial_gym_simulator/results/gain_randomization_gr0_v1/gr0_main")
TRAIN = pathlib.Path("/home/fair/workspaces/aerial_gym_ws/src/aerial_gym_simulator/results/gain_randomization_gr1_v1")
sys.path.insert(0, str(TREE / "tools"))
if len(sys.argv) > 3:
    import types
    G = types.ModuleType("gr_run")
    G.__file__ = str(TREE / "tools" / "gr_run.py")
    SRC = pathlib.Path(sys.argv[3]).read_text()
    exec(compile(SRC, G.__file__, "exec"), G.__dict__)
else:
    import gr_run as G                                      # noqa: E402

if WORK.exists():
    raise SystemExit("work dir exists")
WORK.mkdir(parents=True)
CALLS = []
MODE = {"perturb": False, "fail_at": None}


def fake_preflight(root, protocol, approved, out_root, gr1=False):
    problems = ["output already exists: %s" % out_root] if out_root.exists() else []
    return "HEADSMOKE", problems, {}


def fake_run_cell(root, protocol, out_root, level, density, seed, quota, run_id, scale, head, start_host, records,
                  timeout_s, extra=None, launcher=None, ckpt_sha=None):
    cid = "%s__%dbars__seed%d" % (G.ARM, density, seed)
    gr0_level = level.split("_")[0]
    dst = out_root / level / cid
    dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.copytree(GR0 / gr0_level / cid, dst)
    if MODE["perturb"]:
        rows = [json.loads(l) for l in (dst / "episodes.jsonl").read_text().splitlines() if l.strip()]
        rows[0]["min_relative_distance_m"] = rows[0].get("min_relative_distance_m", 0.0) + 1e-6
        (dst / "episodes.jsonl").write_text("".join(json.dumps(r, sort_keys=True) + "\n" for r in rows))
    CALLS.append({"level": level, "density": density, "seed": seed, "run_id": run_id, "scale": scale,
                  "extra": extra, "launcher": launcher, "ckpt_sha": ckpt_sha, "quota": quota})
    verdict = "FAIL" if MODE["fail_at"] is not None and len(CALLS) == MODE["fail_at"] else "OK"
    rec = {"level": level, "cell_id": cid, "density": density, "seed": seed, "scale": scale, "verdict": verdict,
           "problems": [] if verdict == "OK" else ["smoke failure"]}
    records.append(rec)
    return rec


G.preflight = fake_preflight
G.run_cell = fake_run_cell
G.host = lambda: {"fault_count": 0}


def main_with(argv):
    CALLS.clear()
    sys.argv = ["gr_run.py"] + argv
    try:
        rc = G.main()
    except Exception as e:                                  # an exception here is the bug class this test exists for
        return {"rc": "EXCEPTION", "error": "%s: %s" % (type(e).__name__, e)}
    return {"rc": rc, "calls": list(CALLS)}


def report(path):
    return json.loads(path.read_text()) if path.is_file() else {}


res, ok = {}, {}
root = WORK / "gr1"
root.mkdir()
# 1. gr1repro, exact copies -> GR1_REPRO_PASS with two cells, run id GR1-F, second cell through the GR-1 launcher
r = main_with(["gr1repro", "--approved-commit", "x", "--gr0-dir", str(GR0), "--out", str(root / "repro_ok")])
rep = report(root / "repro_ok" / "gr1repro_report.json")
res["repro_ok"] = {"rc": r["rc"], "verdict": rep.get("verdict"), "calls": r.get("calls"), "error": r.get("error")}
c = r.get("calls") or []
ok["repro_ok"] = (r["rc"] == 0 and rep.get("verdict") == "GR1_REPRO_PASS" and len(c) == 2
                  and all(x["run_id"] == "GR1-F" and x["scale"] == 0.4 and x["density"] == 205 and x["seed"] == 4501
                          and x["quota"] == 16 for x in c)
                  and c[0]["launcher"] is None and c[0]["extra"] is None and c[0]["ckpt_sha"] is None
                  and c[1]["launcher"] == G.GR1_LAUNCHER and c[1]["ckpt_sha"] == G.F_SHA
                  and c[1]["extra"]["GR1_CKPT_SHA"] == G.F_SHA
                  and pathlib.Path(c[1]["extra"]["GR1_CKPT"]).is_file()
                  and G.sha(c[1]["extra"]["GR1_CKPT"]) == G.F_SHA)
# 2. gr1repro with perturbed episodes -> GR1_REPRO_FAIL (known-bad control of the comparison)
MODE["perturb"] = True
r = main_with(["gr1repro", "--approved-commit", "x", "--gr0-dir", str(GR0), "--out", str(WORK / "neg" / "repro_bad")])
MODE["perturb"] = False
rep = report(WORK / "neg" / "repro_bad" / "gr1repro_report.json")
res["repro_perturbed"] = {"rc": r["rc"], "verdict": rep.get("verdict"), "error": r.get("error")}
ok["repro_perturbed"] = r["rc"] == 1 and rep.get("verdict") == "GR1_REPRO_FAIL"
# 3. gr1eval without --train-report -> PREFLIGHT_FAIL, no exception
r = main_with(["gr1eval", "--approved-commit", "x", "--out", str(root / "eval_noreport")])
res["eval_no_train_report"] = {"rc": r["rc"], "error": r.get("error")}
ok["eval_no_train_report"] = r["rc"] == 2 and not (root / "eval_noreport").exists()
# 4. gr1eval with no repro report on this commit -> PREFLIGHT_FAIL
lone = WORK / "lone"
lone.mkdir()
r = main_with(["gr1eval", "--approved-commit", "x", "--train-report", str(TRAIN / "train_ctrl" / "train_report.json"),
               "--out", str(lone / "eval_ctrl")])
res["eval_no_repro"] = {"rc": r["rc"], "error": r.get("error")}
ok["eval_no_repro"] = r["rc"] == 2 and not (lone / "eval_ctrl").exists()
# 5. gr1eval for each arm, after the repro PASS above (same fake HEAD) -> COMPLETE, 36 cells, bound to the report
for arm in ("rand40", "ctrl"):
    tr_path = TRAIN / ("train_" + arm) / "train_report.json"
    tr = json.loads(tr_path.read_text())
    r = main_with(["gr1eval", "--approved-commit", "x", "--train-report", str(tr_path), "--out", str(root / ("eval_" + arm))])
    rep = report(root / ("eval_" + arm) / "gr1eval_run_report.json")
    c = r.get("calls") or []
    res["eval_" + arm] = {"rc": r["rc"], "verdict": rep.get("verdict"), "n_calls": len(c), "policy": rep.get("policy"),
                          "sha": rep.get("checkpoint_sha256"), "error": r.get("error"),
                          "first_call": c[0] if c else None}
    ok["eval_" + arm] = (r["rc"] == 0 and rep.get("verdict") == "COMPLETE" and len(c) == 36
                         and rep.get("policy") == arm == tr["arm"]
                         and rep.get("checkpoint_sha256") == tr["checkpoint"]["sha256"]
                         and rep.get("train_report") == str(tr_path.resolve())
                         and all(x["run_id"] == "GR1-" + arm.upper() and x["launcher"] == G.GR1_LAUNCHER
                                 and x["ckpt_sha"] == tr["checkpoint"]["sha256"]
                                 and x["extra"]["GR1_CKPT"] == str(pathlib.Path(tr["checkpoint"]["copy"]).resolve())
                                 for x in c)
                         and sorted({(x["level"], x["scale"]) for x in c}) == sorted(G.LEVELS.items())
                         and len({(x["density"], x["seed"]) for x in c}) == 12)
# 6. gr1eval with a failing cell -> INCOMPLETE, rc 1, stops at that cell
MODE["fail_at"] = 5
r = main_with(["gr1eval", "--approved-commit", "x", "--train-report", str(TRAIN / "train_ctrl" / "train_report.json"),
               "--out", str(root / "eval_fail")])
MODE["fail_at"] = None
rep = report(root / "eval_fail" / "gr1eval_run_report.json")
res["eval_cell_fail"] = {"rc": r["rc"], "verdict": rep.get("verdict"), "n_calls": len(r.get("calls") or []),
                         "error": r.get("error")}
ok["eval_cell_fail"] = r["rc"] == 1 and rep.get("verdict") == "INCOMPLETE" and len(r.get("calls") or []) == 5

out = {"tree": str(TREE), "gr_run_sha256": G.sha(TREE / "tools" / "gr_run.py"), "results": res, "checks": ok,
       "pass": all(ok.values())}
(WORK / "smoke_result.json").write_text(json.dumps(out, indent=1, default=str) + "\n")
print(json.dumps({"checks": ok, "pass": out["pass"],
                  "errors": {k: v.get("error") for k, v in res.items() if v.get("error")}}, indent=1))
raise SystemExit(0 if out["pass"] else 1)
