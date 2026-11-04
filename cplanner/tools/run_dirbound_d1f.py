"""Driver for gate D1-F, the full-bound direction adversary (docs/plans/prereg_cplanner_dirbound_d1f_v1.md; review I1).

Runs from a frozen `git archive` copy and writes only to a new output directory. The certifier, the replay and the
adapter must be byte-identical to the first gate run's frozen copy (commit 8e2b77d): D1-F tests that certifier. The
native library is built inside the output directory (cplanner/native/build.py) and passed as CPLANNER_COVERAGE_SO.
Plan:
  d1f       tests/check_direction_bound_full.py, 2,000 scenes, seed 20,261,112, WORKERS workers
  dup_d1f   the same, again (determinism)
Six workers keep the CPU load moderate beside a GPU training run. Host checks, input binding and online CPU list as
in run_dirbound.py. Label at most CONDITIONAL until the independent review.
usage: run_dirbound_d1f.py OUT_DIR
"""
import datetime
import hashlib
import json
import pathlib
import subprocess
import sys

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import run_gate as G                                    # noqa: E402
import run_m2 as M2                                     # noqa: E402

FIRST_FROZEN = pathlib.Path("/home/fair/motar_tmp/cplanner_dirbound_20261104/frozen/cplanner")
CARRY = ("src", "native", "motar")                      # certifier, native port, replay and adapter
WORKERS = "6"
D1F = ("d1f", ["check_direction_bound_full.py", "{o}/d1f.json", "2000", WORKERS, "20261112"], 3 * 3600, "pass")
_base_env = G.env


def tree_digest(root):
    out = {}
    for sub in CARRY:
        for p in sorted((root / sub).rglob("*")):
            if p.is_file() and "__pycache__" not in p.parts:
                out[str(p.relative_to(root))] = hashlib.sha256(p.read_bytes()).hexdigest()
    return out


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
    here, first = tree_digest(G.TREE), tree_digest(FIRST_FROZEN)
    carry_ok = bool(here) and here == first
    meta = {"tree": str(G.TREE), "python": G.PY, "allowed_cpus": G.ALLOWED_CPUS, "workers": WORKERS,
            "started": start["time"], "argv": sys.argv, "first_frozen": str(FIRST_FROZEN),
            "carry_over_identical": carry_ok, "carry_files": len(here),
            "carry_differences": sorted(k for k in here.keys() | first.keys() if here.get(k) != first.get(k))[:50],
            "frozen_commit": (G.TREE / "FROZEN_COMMIT").read_text().strip() if (G.TREE / "FROZEN_COMMIT").exists()
            else "NOT_RECORDED", "prereg": "docs/plans/prereg_cplanner_dirbound_d1f_v1.md"}
    (out / "meta.json").write_text(json.dumps(meta, indent=1) + "\n")
    lib = out / "native" / "libcoverage_b.so"
    lib.parent.mkdir()
    b = subprocess.run([G.PY, "-B", str(G.TREE / "native" / "build.py"), str(lib)], stdout=subprocess.PIPE,
                       stderr=subprocess.PIPE, universal_newlines=True, env=_base_env())
    (out / "native" / "build.json").write_text(b.stdout)
    build_ok = b.returncode == 0 and lib.exists()

    def env_with_lib():
        e = _base_env()
        e["CPLANNER_COVERAGE_SO"] = str(lib)
        return e
    G.env = env_with_lib
    pre = M2.preflight()
    (out / "preflight.json").write_text(json.dumps(pre, indent=1) + "\n")
    bound = M2.inputs()
    bound[str(lib)] = G.sha(lib) if lib.exists() else None
    (out / "inputs_before.json").write_text(json.dumps(bound, indent=1) + "\n")
    plan = [(D1F, "main"), (D1F, "dup")]
    records, data, void, online_changes = [], {}, False, []
    if carry_ok and build_ok and pre["pass"]:
        for (name, argv, tmo, key), phase in plan:
            argv = argv if phase == "main" else [a.replace("{o}/", "{o}/dup/") for a in argv]
            label = name if phase == "main" else "dup_" + name
            print("==", label, datetime.datetime.now(G.KST).isoformat(), flush=True)
            online_before = G.host()["online"]
            rec, d = G.run_one(out, logs, name, argv, tmo, None, "DIRBOUND-D1F", label)
            rec["online_before"], rec["online_after"] = online_before, G.host()["online"]
            if rec["online_before"] != rec["online_after"]:
                online_changes.append(label)
            if rec["verdict"] != "VOID_HOST_FAULT":
                v = d.get("pass") if isinstance(d, dict) else None
                rec["verdict_value"] = v
                if key is None:
                    rec["verdict"] = "REPORTED" if (isinstance(d, dict) and d.get("checks", {}).get("completed")) \
                        else "FAIL"
                else:
                    rec["verdict"] = "PASS" if (v is True and rec["exit"] == 0) else "FAIL"
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
        a, b2 = data.get("d1f"), data.get("dup_d1f")
        det["d1f"] = a is not None and b2 is not None and G.strip_timing(a) == G.strip_timing(b2)
    by = {r["label"]: r for r in records}
    summary = {"carry_over_identical": carry_ok, "build_ok": build_ok, "preflight_pass": pre["pass"],
               "d1f": by.get("d1f", {}).get("verdict", "NOT_RUN"),
               "dup_d1f": by.get("dup_d1f", {}).get("verdict", "NOT_RUN"),
               "determinism": det, "inputs_unchanged": not changed, "changed_inputs": changed[:50],
               "host_void": void, "online_cpu_changes": online_changes, "commands_run": len(records),
               "commands_expected": len(plan)}
    complete = (not void and carry_ok and build_ok and pre["pass"] and len(records) == len(plan)
                and det.get("d1f") is True and not changed and by.get("d1f", {}).get("verdict") == "PASS"
                and by.get("dup_d1f", {}).get("verdict") == "PASS")
    summary["verdict"] = "VOID_HOST_FAULT" if void else ("CONDITIONAL" if complete else "FAIL")
    summary["verdict_scope"] = ("D1-F full-bound adversary twice on the certifier of 8e2b77d (byte-identical) + build + "
                                "host + inputs; decided only after an independent review")
    summary["host_end"] = G.host()
    summary["finished"] = datetime.datetime.now(G.KST).isoformat()
    (out / "d1f_summary.json").write_text(json.dumps(summary, indent=1) + "\n")
    print(json.dumps({k: summary[k] for k in ("verdict", "carry_over_identical", "d1f", "dup_d1f", "determinism")},
                     indent=1))
    return 0 if summary["verdict"] == "CONDITIONAL" else 1


if __name__ == "__main__":
    raise SystemExit(main())
