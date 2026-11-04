"""Driver for amendment 1 of the direction-bound repair: gate D2-A1 only
(docs/plans/prereg_cplanner_dirbound_v1_amendment1_2026-11-04.md).

Runs from a frozen `git archive` copy and writes only to a new output directory. The certifier, the replay and the
adapter must be byte-identical to the first gate run's frozen copy (commit 8e2b77d), so D0, D1 and D3 of that run
carry over; any difference fails the run. Plan:
  a1        tests/v1v2a_kernelray.py, 1,000 frames per V1 cell, sampling seed 20,261,110 (fresh), WORKERS workers
  dup_a1    the same, again (determinism)
  a1_d2     the same rule on the first run's D2 sample (seed 20,261,109); reported, not gated
Two workers keep the CPU load low beside a GPU evaluation. Host checks, input binding and online CPU list as in
run_dirbound.py. Label at most CONDITIONAL until the independent review.
usage: run_dirbound_a1.py OUT_DIR
"""
import datetime
import hashlib
import json
import pathlib
import sys

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import run_gate as G                                    # noqa: E402
import run_m2 as M2                                     # noqa: E402

FIRST_FROZEN = pathlib.Path("/home/fair/motar_tmp/cplanner_dirbound_20261104/frozen/cplanner")
CARRY = ("src", "native", "motar")                      # certifier, native port, replay and adapter
WORKERS = "2"
V1_CELLS = [str(c) for c in M2.V1_CELLS]
A1 = ("a1", ["v1v2a_kernelray.py", "{o}/a1.json", "1000", WORKERS, "20261110"] + V1_CELLS, 3 * 3600, "pass")
A1_D2 = ("a1_d2", ["v1v2a_kernelray.py", "{o}/a1_d2.json", "1000", WORKERS, "20261109"] + V1_CELLS, 3 * 3600, None)


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
            else "NOT_RECORDED", "prereg": "docs/plans/prereg_cplanner_dirbound_v1_amendment1_2026-11-04.md"}
    (out / "meta.json").write_text(json.dumps(meta, indent=1) + "\n")
    pre = M2.preflight()
    (out / "preflight.json").write_text(json.dumps(pre, indent=1) + "\n")
    bound = M2.inputs()
    (out / "inputs_before.json").write_text(json.dumps(bound, indent=1) + "\n")
    plan = [(A1, "main"), (A1, "dup"), (A1_D2, "main")]
    records, data, void, online_changes = [], {}, False, []
    if carry_ok and pre["pass"]:
        for (name, argv, tmo, key), phase in plan:
            argv = argv if phase == "main" else [a.replace("{o}/", "{o}/dup/") for a in argv]
            label = name if phase == "main" else "dup_" + name
            print("==", label, datetime.datetime.now(G.KST).isoformat(), flush=True)
            online_before = G.host()["online"]
            rec, d = G.run_one(out, logs, name, argv, tmo, None, "DIRBOUND-A1", label)
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
    (out / "inputs_after.json").write_text(json.dumps(bound_after, indent=1) + "\n")
    changed = sorted(p for p in bound.keys() | bound_after.keys() if bound.get(p) != bound_after.get(p))
    det = {}
    if not void:
        a, b = data.get("a1"), data.get("dup_a1")
        det["a1"] = a is not None and b is not None and G.strip_timing(a) == G.strip_timing(b)
    by = {r["label"]: r for r in records}
    summary = {"carry_over_identical": carry_ok, "preflight_pass": pre["pass"],
               "a1": by.get("a1", {}).get("verdict", "NOT_RUN"), "dup_a1": by.get("dup_a1", {}).get("verdict", "NOT_RUN"),
               "a1_d2_reported": by.get("a1_d2", {}).get("verdict", "NOT_RUN"),
               "a1_d2_pass_value": by.get("a1_d2", {}).get("verdict_value"),
               "determinism": det, "inputs_unchanged": not changed, "changed_inputs": changed[:50],
               "host_void": void, "online_cpu_changes": online_changes, "commands_run": len(records),
               "commands_expected": len(plan)}
    complete = (not void and carry_ok and pre["pass"] and len(records) == len(plan) and det.get("a1") is True
                and not changed and by.get("a1", {}).get("verdict") == "PASS"
                and by.get("dup_a1", {}).get("verdict") == "PASS" and by.get("a1_d2", {}).get("verdict") == "REPORTED")
    summary["verdict"] = "VOID_HOST_FAULT" if void else ("CONDITIONAL" if complete else "FAIL")
    summary["verdict_scope"] = ("D2-A1 on a fresh sample twice + carry-over of D0/D1/D3 by byte-identical certifier, "
                                "replay and adapter + host + inputs; decided only after an independent review")
    summary["host_end"] = G.host()
    summary["finished"] = datetime.datetime.now(G.KST).isoformat()
    (out / "a1_summary.json").write_text(json.dumps(summary, indent=1) + "\n")
    print(json.dumps({k: summary[k] for k in ("verdict", "carry_over_identical", "a1", "dup_a1", "a1_d2_reported",
                                                "a1_d2_pass_value", "determinism")}, indent=1))
    return 0 if summary["verdict"] == "CONDITIONAL" else 1


if __name__ == "__main__":
    raise SystemExit(main())
