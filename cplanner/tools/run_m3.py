"""M3 (part 1) gate driver: native coverage equivalence (E1), R2' suites on the changed tree (E2) and G-beta compute
(docs/plans/prereg_cplanner_m3_v1.md; charter docs/plans/cplanner_charter_v1.md §3, §7).

Runs from a frozen `git archive` copy (cplanner/, motar_eval/, the Warp LiDAR kernel) and writes only to a new output
directory. Step 0 builds the native library inside the output directory with cplanner/native/build.py (fixed flags)
and records its build JSON and SHA-256; every later command gets CPLANNER_COVERAGE_SO pointing at it. Reuses the R2'
driver's helpers (pinning off the faulty cores, fault record and boot id before and after each command, timeouts).
Fail-closed:
- E1 `m3_native_equivalence.py` must write `pass: true` and exit 0, twice; the two runs must agree after removing
  timing keys;
- E2: the seven deterministic R2' suites must each write their verdict key true and exit 0 on this tree;
- G-beta `gbeta_timing.py` runs last, alone (one process pinned to one P-core thread); its reading is reported
  (PASS_K16 / PASS_K8_ONLY / FAIL); FAIL_INCOMPLETE or a non-zero exit fails the run; it runs twice, the non-timing
  fields must agree, and if the two readings differ the G-beta reading is UNSTABLE (not a PASS);
- inputs hashed before and after; a host fault voids and stops the run. Label at most CONDITIONAL until review.
usage: run_m3.py OUT_DIR
"""
import datetime
import json
import pathlib
import subprocess
import sys

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import run_gate as G                                    # noqa: E402
import run_m2 as M2                                     # noqa: E402  (cells, input binding helpers)

ROOT = G.TREE.parent
E1_CELLS = [str(c) for c in M2.V1_CELLS]
GB_CELLS = [str(c) for c in M2.GA_CELLS]
E1 = ("e1", ["m3_native_equivalence.py", "{o}/e1.json", "2000", G.WORKERS, "3"] + E1_CELLS, 3 * 3600, "pass")
E2 = [(n, argv, t, k) for n, argv, t, k, _ in G.DETERMINISTIC]
GB = ("gbeta", ["gbeta_timing.py", "{o}/gbeta.json", "3"] + GB_CELLS, 2 * 3600, None)
GB_READINGS = ("PASS_K16", "PASS_K8_ONLY", "FAIL")
TIMING_KEYS = ("timing_ms", "mean_ms_per_segment", "reading")   # G-beta reading stability is judged separately

_base_env = G.env


def _env_with_lib(lib):
    def env():
        e = _base_env()
        e["CPLANNER_COVERAGE_SO"] = str(lib)
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
    ok_exit = rec["exit"] == 0
    if name == "gbeta":
        reading = data.get("reading") if isinstance(data, dict) else None
        rec["verdict_value"] = reading
        rec["verdict"] = "REPORTED" if (ok_exit and reading in GB_READINGS) else "FAIL"
    else:
        v = data.get(key) if isinstance(data, dict) else None
        rec["verdict_value"] = v
        rec["verdict"] = "PASS" if (v is True and ok_exit) else "FAIL"
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
            "workers": G.WORKERS, "started": start["time"], "argv": sys.argv,
            "frozen_commit": (G.TREE / "FROZEN_COMMIT").read_text().strip() if (G.TREE / "FROZEN_COMMIT").exists()
            else "NOT_RECORDED", "prereg": "docs/plans/prereg_cplanner_m3_v1.md"}
    (out / "meta.json").write_text(json.dumps(meta, indent=1) + "\n")
    lib = out / "native" / "libcoverage_b.so"
    lib.parent.mkdir()
    b = subprocess.run([G.PY, "-B", str(G.TREE / "native" / "build.py"), str(lib)], stdout=subprocess.PIPE,
                       stderr=subprocess.PIPE, universal_newlines=True, env=_base_env())
    (out / "native" / "build.json").write_text(b.stdout)
    build_ok = b.returncode == 0 and lib.exists()
    G.env = _env_with_lib(lib)
    pre = M2.preflight()
    (out / "preflight.json").write_text(json.dumps(pre, indent=1) + "\n")
    bound = M2.inputs()
    bound[str(lib)] = G.sha(lib) if lib.exists() else None
    (out / "inputs_before.json").write_text(json.dumps(bound, indent=1) + "\n")
    plan = [(E1, "main"), (E1, "dup")] + [((n, a, t, k), "main") for n, a, t, k in E2] + [(GB, "main"), (GB, "dup")]
    records, data, void = [], {}, False
    if build_ok and pre["pass"]:
        for (name, argv, tmo, key), phase in plan:
            argv = argv if phase == "main" else [a.replace("{o}/", "{o}/dup/") for a in argv]
            label = name if phase == "main" else "dup_" + name
            print("==", label, datetime.datetime.now(G.KST).isoformat(), flush=True)
            rec, d = G.run_one(out, logs, name, argv, tmo, None, "M3", label)
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
        for name in ("e1", "gbeta"):
            a, b2 = data.get(name), data.get("dup_" + name)
            det[name] = a is not None and b2 is not None and strip(a) == strip(b2)
    by = {r["label"]: r for r in records}
    gb = data.get("gbeta") or {}
    summary = {
        "build_ok": build_ok, "preflight_pass": pre["pass"],
        "e1": by.get("e1", {}).get("verdict", "NOT_RUN"), "dup_e1": by.get("dup_e1", {}).get("verdict", "NOT_RUN"),
        "e2": {n: by.get(n, {}).get("verdict", "NOT_RUN") for n, *_ in E2},
        "gbeta_reading_first": by.get("gbeta", {}).get("verdict_value", "NOT_RUN"),
        "dup_gbeta_reading": by.get("dup_gbeta", {}).get("verdict_value", "NOT_RUN"),
        "gbeta_total16": (gb.get("timing_ms") or {}).get("total16"),
        "dup_gbeta_total16": ((data.get("dup_gbeta") or {}).get("timing_ms") or {}).get("total16"),
        "gbeta_total8": (gb.get("timing_ms") or {}).get("total8"),
        "determinism": det, "inputs_unchanged": not changed, "changed_inputs": changed[:50], "host_void": void,
        "commands_run": len(records), "commands_expected": len(plan),
    }
    complete = (not void and build_ok and pre["pass"] and len(records) == len(plan) and bool(det)
                and all(det.values()) and not changed
                and all(r["verdict"] == "PASS" for r in records if r["name"] != "gbeta")
                and all(by.get(l, {}).get("verdict") == "REPORTED" for l in ("gbeta", "dup_gbeta")))
    r1, r2 = summary["gbeta_reading_first"], summary["dup_gbeta_reading"]
    summary["gbeta_reading"] = r1 if r1 == r2 else "UNSTABLE"          # differing timing readings: not a PASS
    summary["verdict"] = "VOID_HOST_FAULT" if void else ("CONDITIONAL" if complete else "FAIL")
    summary["verdict_scope"] = ("E1 native equivalence + E2 R2' suites on the changed tree + G-beta reading + build + "
                                "determinism + host + inputs; decided only after an independent review")
    summary["host_end"] = G.host()
    summary["finished"] = datetime.datetime.now(G.KST).isoformat()
    (out / "m3_summary.json").write_text(json.dumps(summary, indent=1) + "\n")
    print(json.dumps({k: summary[k] for k in ("verdict", "e1", "e2", "gbeta_reading", "gbeta_total16",
                                                "determinism", "inputs_unchanged", "commands_run")}, indent=1))
    return 0 if summary["verdict"] == "CONDITIONAL" else 1


if __name__ == "__main__":
    raise SystemExit(main())
