"""Known-bad / known-good controls for the amended tools/gr1_analyse.py, on synthetic GR-1 folders built from the real
GR-0 Policy F cells (CPU only; written outside the repository; not evidence of any GR-1 result).
usage: test_gr1_analyse_synthetic.py STAGED_TOOLS_DIR WORK_DIR
"""
import json
import pathlib
import random
import shutil
import subprocess
import sys

TOOLS, WORK = pathlib.Path(sys.argv[1]), pathlib.Path(sys.argv[2])
GR0 = pathlib.Path("/home/fair/workspaces/aerial_gym_ws/src/aerial_gym_simulator/results/gain_randomization_gr0_v1/gr0_main")
PY = "/home/fair/miniconda3/envs/aerialgym/bin/python"
REAL_TOOLS = pathlib.Path("/home/fair/workspaces/aerial_gym_ws/src/MOTAR-gainrand/tools")
rep0 = json.loads((GR0 / "gr0_run_report.json").read_text())


def read_rows(p):
    return [json.loads(l) for l in p.read_text().splitlines() if l.strip()]


def make_arm(root, name, sha, commit, transform):
    """transform(level, rows) -> rows; rows are F's quota ledger rows of the cell."""
    tr_dir = root / ("train_" + name)
    tr_dir.mkdir(parents=True)
    train_report = tr_dir / "train_report.json"
    train_report.write_text(json.dumps({"arm": name.split("_")[0], "verdict": "TRAINED", "commit": commit,
                                        "checkpoint": {"sha256": sha, "copy_sha256": sha, "copy": "x"}}))
    ev = root / ("eval_" + name)
    recs = []
    for r in rep0["records"]:
        src = GR0 / r["level"] / r["cell_id"]
        dst = ev / r["level"] / r["cell_id"]
        dst.mkdir(parents=True)
        rows = transform(r["level"], read_rows(src / "episodes.jsonl"))
        (dst / "episodes.jsonl").write_text("".join(json.dumps(x, sort_keys=True) + "\n" for x in rows))
        shutil.copy(src / "result.json", dst / "result.json")
        (dst / "receipt.json").write_text(json.dumps({"checkpoint_sha256": sha}))
        recs.append(dict(r))
    (ev / "gr1eval_run_report.json").write_text(json.dumps({
        "verdict": "COMPLETE", "policy": name.split("_")[0], "checkpoint_sha256": sha,
        "train_report": str(train_report), "records": recs}))
    return ev


def nudge(rows, eps=1e-6):
    rows = [dict(x) for x in rows]
    rows[0]["min_relative_distance_m"] = rows[0].get("min_relative_distance_m", 0.0) + eps    # differ, same rates
    return rows


def crash_to(outcome, frac, level_only="S40", seed=0, eps=3e-6):
    def tf(level, rows):
        rows = nudge(rows, eps)
        if level != level_only:
            return rows
        rng = random.Random(seed)
        for x in rows:
            if x.get("in_quota") and x.get("outcome") == "crash" and rng.random() < frac:
                x["outcome"] = outcome
                x["crash"] = False
                x["capture" if outcome == "capture" else "timeout"] = True
        return rows
    return tf


def run(case, rnd_dir, ctl_dir):
    out = WORK / ("analysis_" + case + ".json")
    env = {"PYTHONNOUSERSITE": "1", "CUDA_VISIBLE_DEVICES": "", "PATH": "/usr/bin:/bin",
           "PYTHONPATH": str(REAL_TOOLS), "PYTHONDONTWRITEBYTECODE": "1"}
    sys_path_tools = WORK / "tools"
    done = subprocess.run([PY, "-B", str(sys_path_tools / "gr1_analyse.py"), str(GR0), str(rnd_dir), str(ctl_dir),
                           str(out)], capture_output=True, text=True, env=env)
    if done.returncode != 0:
        return {"error": done.stderr[-600:]}
    return json.loads(out.read_text())


def main():
    if WORK.exists():
        raise SystemExit("work dir exists")
    (WORK / "tools").mkdir(parents=True)
    for f in ("gr1_analyse.py",):
        shutil.copy(TOOLS / f, WORK / "tools" / f)
    S = WORK / "synth"
    ctl = make_arm(S, "ctrl", "c" * 64, "COMMIT", lambda lv, rows: nudge(rows))
    eff = make_arm(S, "rand40_effect", "a" * 64, "COMMIT", crash_to("capture", 0.45))      # ~-3.7 pp crash at S40
    null = make_arm(S, "rand40_null", "b" * 64, "COMMIT", lambda lv, rows: [dict(x, **({"min_relative_distance_m": x.get("min_relative_distance_m", 0) + 2e-6} if i == 0 else {})) for i, x in enumerate(rows)])
    tmo = make_arm(S, "rand40_timeout", "d" * 64, "COMMIT", crash_to("timeout", 0.45, eps=4e-6))     # crashes become timeouts
    cases = {
        "effect": run("effect", eff, ctl),
        "null": run("null", null, ctl),
        "swapped": run("swapped", ctl, eff),
        "same_folder_twice": run("same", ctl, ctl),
        "crash_to_timeout": run("timeout", tmo, ctl),
    }
    expect = {"effect": "GR1_SUPPORTED", "null": "GR1_NOT_SUPPORTED", "swapped": "EVAL_BINDING_FAIL",
              "same_folder_twice": "EVAL_BINDING_FAIL"}
    summary = {}
    for k, v in cases.items():
        summary[k] = {"reading": v.get("reading"), "error": v.get("error"),
                      "P1": (v.get("P1_crash_S40_rand40_minus_ctrl") or {}).get("mean_pp"),
                      "P3": (v.get("P3_capture_S40_rand40_minus_ctrl") or {}).get("mean_pp")}
    ok = all(summary[k]["reading"] == e for k, e in expect.items()) and summary["crash_to_timeout"]["reading"] != "GR1_SUPPORTED"
    summary["pass"] = ok
    (WORK / "synthetic_controls.json").write_text(json.dumps(summary, indent=1) + "\n")
    print(json.dumps(summary, indent=1))
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
