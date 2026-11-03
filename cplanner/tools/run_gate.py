"""R2' gate driver (rules: docs/plans/cplanner_r2prime_rules_v1.md, fixed 2026-11-04 before measurement).

Runs ONE fixed plan of bounded CPU commands from a frozen `git archive` copy of `cplanner/` and writes only to a new
output directory. Fail-closed:
- every gated command must write its JSON with an explicit boolean verdict; a missing file, missing key, non-bool
  value or non-zero exit is a FAIL of that rule (a vacuous empty check set can never pass);
- the host kernel fault record (machine check, hardware error, segfault, general protection) and the boot id are read
  before and after every command; any change voids the whole run (AGENTS.md: host faults void a run) and stops it;
- every bound input (frozen tree, external references, interpreter) is hashed before and after; any change is a FAIL;
- deterministic suites run twice and must agree after removing timing keys (`wall_s`, `ms_*`); the randomized
  suites rerun a seed prefix and every per-seed digest must match the first run.
The overall label is at most CONDITIONAL: PASS requires the independent review (R3') under the charter's scope.

usage: run_gate.py OUT_DIR            (run from anywhere; the frozen tree is this file's parent's parent)
"""
import datetime
import hashlib
import json
import os
import pathlib
import platform
import re
import subprocess
import sys
import time

TREE = pathlib.Path(__file__).resolve().parents[1]          # frozen copy of cplanner/
TESTS = TREE / "tests"
PY = "/home/fair/miniconda3/envs/aerialgym/bin/python"
ALLOWED_CPUS = "0-3,8-11,14-23"     # excludes cores 2, 3, 6 (cpu4-7, cpu12-13): every logged MCE since 2026-10-28
WORKERS = "12"
KST = datetime.timezone(datetime.timedelta(hours=9))

EXTERNAL = [
    "/home/fair/motar_tmp/p0_v7_20261029/baseline/v3_run4/certify_b.py",
    "/home/fair/motar_tmp/p0_v7_20261029/baseline/v4_run5/certify_b.py",
    "/home/fair/motar_tmp/p0_v7_20261029/baseline/v5_run6/certify_b.py",
    "/home/fair/motar_tmp/p0_v7_20261029/baseline/v6_run7/certify_b.py",
    "/home/fair/motar_tmp/p0_v7_20261029/src/certify_b.py",
    "/home/fair/motar_tmp/p0_v7_20261029/results/run8/regress_boundary.json",
    "/home/fair/workspaces/aerial_gym_ws/src/MOTAR-mt-p1/motar_eval/occlusion_trace.py",
]
EXTERNAL_GLOBS = [
    ("/home/fair/workspaces/aerial_gym_ws/src/MOTAR-mt-p1/resources/models/environment_assets/bars_h3", "*.urdf"),
]
B1R_TRACES = "/home/fair/workspaces/aerial_gym_ws/src/aerial_gym_simulator/results/b1r_classical_baseline_v1/A/F_SHADOW"

# (name, argv after the interpreter, timeout s, verdict key or None for REPORTED, rule)
DETERMINISTIC = [
    ("check_static", ["check_static.py", "{o}/check_static.json"], 300, "pass", "1-2"),
    ("check_r1_core", ["check_r1_core.py", "{o}/check_r1_core.json"], 300, "pass", "4"),
    ("attacks_p0r", ["attacks_p0r.py", "{o}/attacks_p0r.json"], 900, "pass", "4"),
    ("scenarios_p0r", ["scenarios_p0r.py", "{o}/scenarios_p0r.json"], 1200, "pass", "6/8"),
    ("map_invariants", ["map_invariants.py", "{o}/map_invariants.json"], 2400, "pass", "7"),
    ("check_r1_temporal", ["check_r1_temporal.py", "{o}/check_r1_temporal.json"], 900, "focused_checks_pass", "5C"),
    ("regress_v7", ["regress_v7.py", "{o}/regress_v7.json"], 2400, "pass", "3/8"),
]
RANDOMIZED = [
    ("ni_A", ["noninterference_v2.py", "A", "{o}/ni_A.json", "20000", WORKERS], 6 * 3600, "pass", "5A"),
    ("ni_B", ["noninterference_v2.py", "B", "{o}/ni_B.json", "2000", WORKERS], 3 * 3600, "pass", "5B"),
]
DUPLICATES = [(n, [a.replace("{o}/", "{o}/dup/") for a in argv], t, k, r) for n, argv, t, k, r in DETERMINISTIC]
DUPLICATES += [
    ("ni_A", ["noninterference_v2.py", "A", "{o}/dup/ni_A.json", "2000", WORKERS], 3600, "pass", "5A"),
    ("ni_B", ["noninterference_v2.py", "B", "{o}/dup/ni_B.json", "200", WORKERS], 3600, "pass", "5B"),
]
TIMING = ("timing_p0r", ["timing_p0r.py", "{o}/timing_p0r.json"], 3600, None, "9")
FAULT = re.compile(r"Machine check|segfault|general protection|mce:.*Hardware Error|BERT:.*Hardware Error|"
                   r"Hardware Error", re.I)


def sha(p):
    return hashlib.sha256(pathlib.Path(p).read_bytes()).hexdigest()


def inputs():
    files = [p for p in TREE.rglob("*") if p.is_file() and "__pycache__" not in p.parts and p.suffix != ".pyc"]
    out = {str(p): sha(p) for p in sorted(files)}
    for p in EXTERNAL:
        out[p] = sha(p)
    for d, g in EXTERNAL_GLOBS:
        for p in sorted(pathlib.Path(d).glob(g)):
            out[str(p)] = sha(p)
    for bars in (70, 205):
        d = pathlib.Path(B1R_TRACES) / ("E2_obstacle_aware__%dbars__seed4301" % bars)
        for p in sorted(d.glob("occlusion_*")):
            out[str(p)] = sha(p)
    out[PY] = sha(os.path.realpath(PY))
    return out


def host():
    r = subprocess.run(["journalctl", "-k", "-b", "0", "--no-pager", "-o", "short-iso"], stdout=subprocess.PIPE,
                       stderr=subprocess.PIPE, universal_newlines=True, timeout=60)
    if r.returncode:
        raise RuntimeError("cannot read kernel journal: " + r.stderr[:300])
    ev = [s for s in r.stdout.splitlines() if FAULT.search(s)]
    rd = lambda p: pathlib.Path(p).read_text().strip() if pathlib.Path(p).exists() else "NOT_AVAILABLE"  # noqa: E731
    return {"boot_id": rd("/proc/sys/kernel/random/boot_id"), "fault_events": ev, "fault_count": len(ev),
            "online": rd("/sys/devices/system/cpu/online"), "offline": rd("/sys/devices/system/cpu/offline"),
            "bios": rd("/sys/class/dmi/id/bios_version"), "loadavg": rd("/proc/loadavg"),
            "time": datetime.datetime.now(KST).isoformat()}


def env():
    e = dict(os.environ)
    for k in ("PYTHONPATH", "LD_PRELOAD"):
        e.pop(k, None)
    e.update(CUDA_VISIBLE_DEVICES="", PYTHONNOUSERSITE="1", PYTHONDONTWRITEBYTECODE="1", PYTHONHASHSEED="0",
             OMP_NUM_THREADS="1", OPENBLAS_NUM_THREADS="1", MKL_NUM_THREADS="1", NUMEXPR_NUM_THREADS="1",
             P0R_SRC=str(TREE / "src"))
    return e


def strip_timing(o):
    if isinstance(o, dict):
        return {k: strip_timing(v) for k, v in o.items() if k != "wall_s" and not str(k).startswith("ms_")}
    if isinstance(o, list):
        return [strip_timing(v) for v in o]
    return o


def run_one(out, logs, name, argv, timeout_s, key, rule, label):
    args = [a.format(o=str(out)) for a in argv]
    before = host()
    t0 = time.monotonic()
    with (logs / (label + ".stdout.log")).open("w") as so, (logs / (label + ".stderr.log")).open("w") as se:
        r = subprocess.run(["timeout", "-k", "30s", "%ds" % timeout_s, "taskset", "-c", ALLOWED_CPUS, PY, "-B"] + args,
                           cwd=str(TESTS), env=env(), stdout=so, stderr=se)
    after = host()
    rec = {"label": label, "name": name, "rule": rule, "argv": args, "timeout_s": timeout_s, "exit": r.returncode,
           "elapsed_s": time.monotonic() - t0, "host_before_count": before["fault_count"],
           "host_after_count": after["fault_count"], "loadavg_after": after["loadavg"],
           "host_valid": before["boot_id"] == after["boot_id"] and before["fault_events"] == after["fault_events"]}
    target = pathlib.Path(args[[i for i, a in enumerate(args) if a.endswith(".json")][0]])
    rec["output"] = str(target)
    if target.exists():
        rec["output_sha256"] = sha(target)
        try:
            data = json.loads(target.read_text())
        except Exception as e:                      # unreadable output is a FAIL
            data, rec["output_error"] = None, type(e).__name__
    else:
        data = None
    if key is None:
        rec["verdict"] = "REPORTED" if (data is not None and r.returncode == 0) else "MISSING"
    else:
        v = data.get(key) if isinstance(data, dict) else None
        rec["verdict_value"] = v
        rec["verdict"] = "PASS" if (v is True and r.returncode == 0) else "FAIL"
    if not rec["host_valid"]:
        rec["verdict"] = "VOID_HOST_FAULT"
        rec["new_fault_events"] = [e for e in after["fault_events"] if e not in before["fault_events"]]
    return rec, data


def main():
    out = pathlib.Path(sys.argv[1]).resolve()
    if out.exists():
        raise SystemExit("refusing to overwrite " + str(out))
    out.mkdir(parents=True)
    (out / "dup").mkdir()
    logs = out / "logs"
    logs.mkdir()
    start = host()
    (out / "host_start.json").write_text(json.dumps(start, indent=1) + "\n")
    bound = inputs()
    (out / "inputs_before.json").write_text(json.dumps(bound, indent=1) + "\n")
    meta = {"tree": str(TREE), "python": PY, "python_version": platform.python_version(), "allowed_cpus": ALLOWED_CPUS,
            "workers": WORKERS, "started": start["time"], "argv": sys.argv,
            "frozen_commit": (TREE / "FROZEN_COMMIT").read_text().strip() if (TREE / "FROZEN_COMMIT").exists()
            else "NOT_RECORDED"}
    (out / "meta.json").write_text(json.dumps(meta, indent=1) + "\n")
    records, data = [], {}
    void = False
    plan = [(c, "main") for c in DETERMINISTIC + RANDOMIZED] + [(c, "dup") for c in DUPLICATES] + [(TIMING, "main")]
    for (name, argv, tmo, key, rule), phase in plan:
        label = name if phase == "main" else "dup_" + name
        print("==", label, datetime.datetime.now(KST).isoformat(), flush=True)
        rec, d = run_one(out, logs, name, argv, tmo, key, rule, label)
        records.append(rec)
        data[label] = d
        print("  ", rec["verdict"], "exit", rec["exit"], "%.0fs" % rec["elapsed_s"], flush=True)
        (out / "records.json").write_text(json.dumps(records, indent=1) + "\n")
        if rec["verdict"] == "VOID_HOST_FAULT":
            void = True
            break
    bound_after = inputs()
    (out / "inputs_after.json").write_text(json.dumps(bound_after, indent=1) + "\n")
    changed = sorted(p for p in bound.keys() | bound_after.keys() if bound.get(p) != bound_after.get(p))
    # determinism
    det = {}
    if not void:
        for name, *_ in DETERMINISTIC:
            a, b = data.get(name), data.get("dup_" + name)
            det[name] = a is not None and b is not None and strip_timing(a) == strip_timing(b)
        for name, *_ in RANDOMIZED:
            a, b = data.get(name), data.get("dup_" + name)
            ok = isinstance(a, dict) and isinstance(b, dict) and bool(b.get("digests"))
            if ok:
                ok = all(a["digests"].get(s) == dg for s, dg in b["digests"].items())
            det[name] = ok
    rules = {}
    for rec in records:
        if rec["label"].startswith("dup_"):
            continue
        rules.setdefault(rec["rule"], []).append(rec["verdict"])
    gated = {r: all(v == "PASS" for v in vs) for r, vs in rules.items() if r != "9"}
    dup_runs_pass = all(r["verdict"] == "PASS" for r in records if r["label"].startswith("dup_"))
    expected = len(DETERMINISTIC) + len(RANDOMIZED) + len(DUPLICATES) + 1
    summary = {
        "rules": gated, "timing_rule_9": rules.get("9"),
        "determinism": det, "duplicate_runs_pass": dup_runs_pass,
        "inputs_unchanged": not changed, "changed_inputs": changed[:50],
        "host_void": void, "commands_run": len(records), "commands_expected": expected,
    }
    complete = (not void and len(records) == expected and bool(gated) and all(gated.values()) and bool(det)
                and all(det.values()) and dup_runs_pass and not changed and rules.get("9") == ["REPORTED"])
    summary["verdict"] = "VOID_HOST_FAULT" if void else ("CONDITIONAL" if complete else "FAIL")
    summary["verdict_scope"] = ("rules 1-8 + 5A/5B/5C + determinism + host + inputs; PASS only after the "
                                "independent R3' review under docs/plans/cplanner_charter_v1.md")
    summary["host_end"] = host()
    summary["finished"] = datetime.datetime.now(KST).isoformat()
    (out / "gate_summary.json").write_text(json.dumps(summary, indent=1) + "\n")
    print(json.dumps({k: summary[k] for k in ("verdict", "rules", "determinism", "inputs_unchanged",
                                                "commands_run")}, indent=1))
    return 0 if summary["verdict"] == "CONDITIONAL" else 1


if __name__ == "__main__":
    raise SystemExit(main())
