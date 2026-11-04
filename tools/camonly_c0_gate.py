"""C0 gate of the camera-only study (docs/plans/camonly_study_plan_v1.md, "C0 gate"): run once on a clean commit.

Items 1-4 of the plan, recorded in one JSON (item 5, the independent review, follows):
  1 tests/test_navrl_sensor_ablation.py passes
  2 tests/camonly_c0_mutants.py: the unmodified module passes, every mutant fails at least one test
  3 tools/run_tests.py (PUBLIC_CPU, one process) adds no failing test to the base set BASE_LOG, and the new tests
    ran inside it
  4 git diff BASE -- navrl_task.py navrl_perception.py is empty
Host fault count before and after; the tree must be clean and HEAD must equal --approved-commit.
usage: tools/camonly_c0_gate.py --approved-commit SHA --base-log PATH --out DIR
"""
import argparse
import json
import pathlib
import re
import subprocess
import sys
import time

ROOT = pathlib.Path(__file__).resolve().parents[1]
BASE = "ab6531b"
PINNED = ["aerial_gym/task/navrl_task/navrl_task.py", "aerial_gym/task/navrl_task/navrl_perception.py"]
PY = sys.executable
CORES = "6,7,12,13"
ENV = {"PATH": "/usr/bin:/bin", "PYTHONNOUSERSITE": "1", "CUDA_VISIBLE_DEVICES": "", "PYTHONDONTWRITEBYTECODE": "1",
       "HOME": str(pathlib.Path.home()), "LC_ALL": "C"}


def git(*a):
    return subprocess.run(["git", "-C", str(ROOT)] + list(a), capture_output=True, text=True).stdout


def faults():
    out = subprocess.run(["journalctl", "-k", "-b", "0", "--no-pager"], capture_output=True, text=True).stdout
    return len(re.findall(r"Machine check|segfault|general protection", out))


def run(cmd, log, timeout):
    t0 = time.time()
    with open(log, "x") as fh:
        r = subprocess.run(["timeout", "-k", "60", str(timeout), "nice", "-n", "10", "taskset", "-c", CORES] + cmd,
                           cwd=str(ROOT), env=ENV, stdout=fh, stderr=subprocess.STDOUT)
    return r.returncode, time.time() - t0


def failing(log_text):
    return sorted({re.sub(r" \(.*=.*\)$", "", m) for m in re.findall(r"^(?:FAIL|ERROR): (.+)$", log_text, re.M)})


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--approved-commit", required=True)
    ap.add_argument("--base-log", required=True)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    out = pathlib.Path(a.out).resolve()
    out.mkdir(parents=True)                      # refuses an existing directory
    head = git("rev-parse", "HEAD").strip()
    status = git("status", "--porcelain")
    pre = []
    if not head.startswith(a.approved_commit):
        pre.append("HEAD %s is not the approved commit %s" % (head, a.approved_commit))
    if status.strip():
        pre.append("tree not clean: %s" % status[:500])
    rec = {"gate": "camonly C0", "commit": head, "base": BASE, "faults_start": faults(), "preflight_problems": pre,
           "started": time.strftime("%Y-%m-%dT%H:%M:%S%z")}
    if pre:
        rec["verdict"] = "PREFLIGHT_FAIL"
        (out / "c0_gate.json").write_text(json.dumps(rec, indent=1) + "\n")
        print(json.dumps(rec, indent=1))
        return 2
    rc1, w1 = run([PY, "-B", "tests/test_navrl_sensor_ablation.py", "-v"], out / "c0_tests.log", 600)
    rc2, w2 = run([PY, "-B", "tests/camonly_c0_mutants.py", str(ROOT), str(out / "mutants.json")],
                  out / "mutants.log", 900)
    rc3, w3 = run([PY, "-B", "tools/run_tests.py"], out / "public_cpu.log", 3000)
    base_fail = failing(pathlib.Path(a.base_log).read_text())
    suite_text = (out / "public_cpu.log").read_text()
    cand_fail = failing(suite_text)
    def last_ran(text):                          # the final unittest summary line ("ran" also appears in sub-reports)
        found = re.findall(r"^Ran (\d+) tests? in", text, re.M)
        return int(found[-1]) if found else None
    ran, base_ran = last_ran(suite_text), last_ran(pathlib.Path(a.base_log).read_text())
    n_new = last_ran((out / "c0_tests.log").read_text())
    mut = json.loads((out / "mutants.json").read_text()) if (out / "mutants.json").is_file() else {}
    pinned_diff = git("diff", BASE, "--", *PINNED)
    items = {
        "1_c0_tests": rc1 == 0,
        "2_mutants": rc2 == 0 and bool(mut.get("pass")),
        "3_suite_no_new_failures": sorted(set(cand_fail) - set(base_fail)) == [],
        "3_new_tests_ran_in_suite": None not in (ran, base_ran, n_new) and ran - base_ran == n_new and n_new > 0,
        "3_new_tests_not_failing_in_suite": not any("test_navrl_sensor_ablation" in t for t in cand_fail),
        "4_pinned_files_unchanged": pinned_diff == "",
    }
    rec.update({
        "items": items,
        "c0_tests": {"exit": rc1, "wall_s": w1},
        "mutants": {"exit": rc2, "wall_s": w2, "summary": {k: v.get("failures", 0) + v.get("errors", 0)
                                                           for k, v in mut.get("mutants", {}).items()}},
        "suite": {"exit": rc3, "wall_s": w3, "ran": ran, "base_ran": base_ran, "c0_tests": n_new,
                  "failing": cand_fail, "base_failing": base_fail, "base_log": str(pathlib.Path(a.base_log).resolve()),
                  "new_failures": sorted(set(cand_fail) - set(base_fail)),
                  "fixed_vs_base": sorted(set(base_fail) - set(cand_fail))},
        "faults_end": faults(), "finished": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
    })
    host_ok = rec["faults_end"] == rec["faults_start"]
    rec["verdict"] = ("VOID_HOST_FAULT" if not host_ok else
                      "C0_CONDITIONAL_PASS" if all(items.values()) else "C0_FAIL")
    rec["note"] = "item 5 (independent review) pending; CONDITIONAL until it passes"
    (out / "c0_gate.json").write_text(json.dumps(rec, indent=1) + "\n")
    print(json.dumps({"verdict": rec["verdict"], "items": items, "new_failures": rec["suite"]["new_failures"]},
                     indent=1))
    return 0 if rec["verdict"] == "C0_CONDITIONAL_PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
