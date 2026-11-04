"""Direction-bound repair gate driver (docs/plans/prereg_cplanner_dirbound_v1.md; decision D-2026-11-04-2).

Runs from a frozen `git archive` copy (cplanner/, motar_eval/, the Warp LiDAR kernel) and writes only to a new output
directory. Step 0 builds the native coverage library inside the output directory (cplanner/native/build.py); every
command gets CPLANNER_COVERAGE_SO pointing at it and CPLANNER_GALPHA_CONFIG=MOTAR-72x4-RUNTIME-DIR-NATIVE. Reuses
the R2' driver's helpers (pinning off the faulty cores, fault record and boot id before and after each command,
timeouts) and M2's bar-pool preflight and input binding. Plan, in order:
  D0  the seven deterministic R2' suites on this tree (configurations with eps = 0 unchanged)
  D1  tests/check_direction_bound.py   2,000 scenes, twice
  D2  tests/v1v2_kernelray.py          1,000 frames per V1 cell, twice
  D3  tests/galpha_replay.py           6 episodes per G-alpha cell on MOTAR-72x4-RUNTIME-DIR-NATIVE, twice
Fail-closed: D0-D2 must write their verdict key true and exit 0; D3 must write a reading other than FAIL_INCOMPLETE
with stats.false_cert == 0 (otherwise STOP_FALSE_CERT); duplicates must agree after removing timing keys; inputs
hashed before and after; a host fault voids and stops the run. The online CPU list is recorded before and after every
command and any change is reported (R3' minor M8). Label at most CONDITIONAL until the independent review.
usage: run_dirbound.py OUT_DIR
"""
import datetime
import json
import pathlib
import subprocess
import sys

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import run_gate as G                                    # noqa: E402
import run_m2 as M2                                     # noqa: E402

ROOT = G.TREE.parent
V1_CELLS = [str(c) for c in M2.V1_CELLS]
GA_CELLS = [str(c) for c in M2.GA_CELLS]
GA_CONFIG = "MOTAR-72x4-RUNTIME-DIR-NATIVE"
D0 = [(n, argv, t, k) for n, argv, t, k, _ in G.DETERMINISTIC]
D1 = ("dirbound", ["check_direction_bound.py", "{o}/dirbound.json", "2000", G.WORKERS], 2 * 3600, "pass")
D2 = ("v1v2", ["v1v2_kernelray.py", "{o}/v1v2.json", "1000", G.WORKERS] + V1_CELLS, 3600, "pass")
D3 = ("galpha_dir", ["galpha_replay.py", "{o}/galpha_dir.json", "6", G.WORKERS] + GA_CELLS, 3 * 3600, None)
TIMING_KEYS = ("timing_ms", "mean_ms_per_segment")

_base_env = G.env


def _env_with(lib):
    def env():
        e = _base_env()
        e["CPLANNER_COVERAGE_SO"] = str(lib)
        e["CPLANNER_GALPHA_CONFIG"] = GA_CONFIG
        return e
    return env


def strip(o):
    o = G.strip_timing(o)
    if isinstance(o, dict):
        return {k: strip(v) for k, v in o.items() if k not in TIMING_KEYS}
    if isinstance(o, list):
        return [strip(v) for v in o]
    return o


def judge(name, key, rec, data):
    if rec["verdict"] == "VOID_HOST_FAULT":
        return rec
    if name == "galpha_dir":
        return M2.judge("galpha", rec, data)
    v = data.get(key) if isinstance(data, dict) else None
    rec["verdict_value"] = v
    rec["verdict"] = "PASS" if (v is True and rec["exit"] == 0) else "FAIL"
    return rec


def main():
    out = pathlib.Path(sys.argv[1]).resolve()
    if out.exists():
        raise SystemExit("refusing to overwrite " + str(out))
    out.mkdir(parents=True)
    (out / "dup").mkdir()
    logs = out / "logs"
    logs.mkdir()
    start = G.host()
    (out / "host_start.json").write_text(json.dumps(start, indent=1) + "\n")
    meta = {"tree": str(G.TREE), "root": str(ROOT), "python": G.PY, "allowed_cpus": G.ALLOWED_CPUS,
            "workers": G.WORKERS, "started": start["time"], "argv": sys.argv, "galpha_config": GA_CONFIG,
            "frozen_commit": (G.TREE / "FROZEN_COMMIT").read_text().strip() if (G.TREE / "FROZEN_COMMIT").exists()
            else "NOT_RECORDED", "prereg": "docs/plans/prereg_cplanner_dirbound_v1.md"}
    (out / "meta.json").write_text(json.dumps(meta, indent=1) + "\n")
    lib = out / "native" / "libcoverage_b.so"
    lib.parent.mkdir()
    b = subprocess.run([G.PY, "-B", str(G.TREE / "native" / "build.py"), str(lib)], stdout=subprocess.PIPE,
                       stderr=subprocess.PIPE, universal_newlines=True, env=_base_env())
    (out / "native" / "build.json").write_text(b.stdout)
    build_ok = b.returncode == 0 and lib.exists()
    G.env = _env_with(lib)
    pre = M2.preflight()
    (out / "preflight.json").write_text(json.dumps(pre, indent=1) + "\n")
    bound = M2.inputs()
    bound[str(lib)] = G.sha(lib) if lib.exists() else None
    (out / "inputs_before.json").write_text(json.dumps(bound, indent=1) + "\n")
    plan = ([((n, a, t, k), "main") for n, a, t, k in D0] + [(D1, "main"), (D1, "dup"), (D2, "main"), (D2, "dup"),
                                                              (D3, "main"), (D3, "dup")])
    records, data, void, online_changes = [], {}, False, []
    if build_ok and pre["pass"]:
        for (name, argv, tmo, key), phase in plan:
            argv = argv if phase == "main" else [a.replace("{o}/", "{o}/dup/") for a in argv]
            label = name if phase == "main" else "dup_" + name
            print("==", label, datetime.datetime.now(G.KST).isoformat(), flush=True)
            online_before = G.host()["online"]
            rec, d = G.run_one(out, logs, name, argv, tmo, None, "DIRBOUND", label)
            rec["online_before"], rec["online_after"] = online_before, G.host()["online"]
            if rec["online_before"] != rec["online_after"]:
                online_changes.append(label)
            rec = judge(name, key, rec, d)
            records.append(rec)
            data[label] = d
            print("  ", rec["verdict"], rec.get("verdict_value"), "exit", rec["exit"], "%.0fs" % rec["elapsed_s"],
                  flush=True)
            (out / "records.json").write_text(json.dumps(records, indent=1) + "\n")
            if rec["verdict"] == "VOID_HOST_FAULT":
                void = True
                break
    bound_after = M2.inputs()
    bound_after[str(lib)] = G.sha(lib) if lib.exists() else None
    (out / "inputs_after.json").write_text(json.dumps(bound_after, indent=1) + "\n")
    changed = sorted(p for p in bound.keys() | bound_after.keys() if bound.get(p) != bound_after.get(p))
    det = {}
    if not void:
        for name in ("dirbound", "v1v2", "galpha_dir"):
            a, b2 = data.get(name), data.get("dup_" + name)
            det[name] = a is not None and b2 is not None and strip(a) == strip(b2)
    by = {r["label"]: r for r in records}
    ga = by.get("galpha_dir", {})
    gd = data.get("galpha_dir") or {}
    summary = {
        "build_ok": build_ok, "preflight_pass": pre["pass"],
        "d0": {n: by.get(n, {}).get("verdict", "NOT_RUN") for n, *_ in D0},
        "d1_dirbound": by.get("dirbound", {}).get("verdict", "NOT_RUN"),
        "d2_v1v2": by.get("v1v2", {}).get("verdict", "NOT_RUN"),
        "d3_galpha_reading": ga.get("verdict_value", "NOT_RUN"), "d3_galpha_verdict": ga.get("verdict", "NOT_RUN"),
        "d3_galpha_false_cert": ga.get("false_cert"), "d3_certifiable_rate": gd.get("certifiable_rate"),
        "d3_certifiable_rate_by_group": gd.get("certifiable_rate_by_group"),
        "duplicates": {r["label"]: r["verdict"] for r in records if r["label"].startswith("dup_")},
        "determinism": det, "inputs_unchanged": not changed, "changed_inputs": changed[:50], "host_void": void,
        "online_cpu_changes": online_changes, "commands_run": len(records), "commands_expected": len(plan),
    }
    complete = (not void and build_ok and pre["pass"] and len(records) == len(plan) and bool(det)
                and all(det.values()) and not changed
                and all(r["verdict"] == "PASS" for r in records if r["name"] != "galpha_dir")
                and all(by.get(l, {}).get("verdict") == "REPORTED" for l in ("galpha_dir", "dup_galpha_dir")))
    if void:
        summary["verdict"] = "VOID_HOST_FAULT"
    elif any(r.get("verdict") == "STOP_FALSE_CERT" for r in records):
        summary["verdict"] = "STOP_FALSE_CERT"
    else:
        summary["verdict"] = "CONDITIONAL" if complete else "FAIL"
    summary["verdict_scope"] = ("D0 eps = 0 suites + D1 direction-bound fuzz + D2 V1-v2 + D3 G-alpha reading on the "
                                "repaired configuration + build + determinism + host + inputs; decided only after an "
                                "independent review")
    summary["host_end"] = G.host()
    summary["finished"] = datetime.datetime.now(G.KST).isoformat()
    (out / "dirbound_summary.json").write_text(json.dumps(summary, indent=1) + "\n")
    print(json.dumps({k: summary[k] for k in ("verdict", "d0", "d1_dirbound", "d2_v1v2", "d3_galpha_reading",
                                                "d3_certifiable_rate", "determinism", "online_cpu_changes")},
                     indent=1))
    return 0 if summary["verdict"] == "CONDITIONAL" else 1


if __name__ == "__main__":
    raise SystemExit(main())
