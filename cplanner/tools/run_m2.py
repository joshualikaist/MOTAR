"""M2 gate driver: V1 (status mapping on replayed scans) and G-alpha (certifiability on replay), as fixed in
docs/plans/prereg_cplanner_m2_v1.md and the charter (docs/plans/cplanner_charter_v1.md §3, §4, §7).

Runs from a frozen `git archive` copy that holds `cplanner/`, `motar_eval/` and the MOTAR Warp LiDAR kernel file, and
writes only to a new output directory. It reuses the R2' driver's host, environment and command helpers
(`run_gate.py`: pinning `taskset -c 0-3,8-11,14-23`, fault record and boot id before and after every command,
`timeout -k`). Fail-closed:
- preflight: every cell's `receipt.json` must record `env_snapshot_navrl.NAVRL_BAR_POOL == "bars_h3"` (charter §4)
  and the frame and layout files must exist; otherwise nothing is measured;
- V1 must write `pass: true` and exit 0; G-alpha must write a `reading` label and exit 0, and its `stats.false_cert`
  must be 0 (a non-zero count is the preregistered STOP_FALSE_CERT);
- both commands run twice (deterministic: fixed sampling seeds) and must agree after removing timing keys;
- every bound input is hashed before and after; any change is a FAIL; a host fault voids the run and stops it.
The overall label is at most CONDITIONAL: the gate is decided only after an independent review.

usage: run_m2.py OUT_DIR
"""
import datetime
import json
import pathlib
import sys

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import run_gate as G                                    # noqa: E402  (same frozen tree)

ROOT = G.TREE.parent                                    # frozen archive root: cplanner/, motar_eval/, aerial_gym/...
KERNEL = ROOT / "aerial_gym" / "sensors" / "warp" / "warp_kernels" / "warp_lidar_kernels.py"
TRACE_READER = ROOT / "motar_eval" / "occlusion_trace.py"
B1R = pathlib.Path("/home/fair/workspaces/aerial_gym_ws/src/aerial_gym_simulator/results/b1r_classical_baseline_v1")
V1_CELLS = [B1R / "A" / "F_SHADOW" / ("E2_obstacle_aware__%dbars__seed4301" % b) for b in (70, 115, 160, 205)]
V1_CELLS += [B1R / "C-PP" / "B-PP" / ("E2_obstacle_aware__%dbars__seed4301" % b) for b in (160, 205)]
GA_CELLS = [B1R / arm / ("E2_obstacle_aware__%dbars__seed%d" % (b, s))
            for arm in ("A/F_SHADOW", "C-PP/B-PP") for b in (160, 205) for s in range(4301, 4308)]
V1_FRAMES, GA_EPISODES = "400", "6"
TRACE_FILES = ("receipt.json", "occlusion_frames.bin", "occlusion_layouts.npz", "occlusion_trace_summary.json")
READINGS = ("STOP_FALSE_CERT", "FILTER_ARM_RETAINED", "MONITOR_ONLY_PILOT", "FILTER_PAUSED", "FAIL_INCOMPLETE")
TIMING_KEYS = ("timing_ms",)

PLAN = [
    ("v1", ["v1_hitbit.py", "{o}/v1.json", V1_FRAMES, G.WORKERS] + [str(c) for c in V1_CELLS], 2 * 3600),
    ("galpha", ["galpha_replay.py", "{o}/galpha.json", GA_EPISODES, G.WORKERS] + [str(c) for c in GA_CELLS],
     4 * 3600),
]


def inputs():
    out = {}
    for p in sorted(G.TREE.rglob("*")):
        if p.is_file() and "__pycache__" not in p.parts and p.suffix != ".pyc":
            out[str(p)] = G.sha(p)
    for p in (KERNEL, TRACE_READER, ROOT / "motar_eval" / "__init__.py"):
        out[str(p)] = G.sha(p)
    for c in sorted(set(V1_CELLS + GA_CELLS)):
        for f in TRACE_FILES:
            if (c / f).exists():
                out[str(c / f)] = G.sha(c / f)
    out[G.PY] = G.sha(G.os.path.realpath(G.PY))
    return out


def preflight():
    cells = {}
    for c in sorted(set(V1_CELLS + GA_CELLS)):
        rec = {"exists": c.is_dir(), "missing_files": [f for f in TRACE_FILES if not (c / f).exists()]}
        try:
            snap = json.loads((c / "receipt.json").read_text()).get("env_snapshot_navrl", {})
            rec["bar_pool"] = snap.get("NAVRL_BAR_POOL", "NOT_RECORDED")
        except Exception as e:
            rec["bar_pool"] = "UNREADABLE_" + type(e).__name__
        rec["ok"] = rec["exists"] and not rec["missing_files"] and rec["bar_pool"] == "bars_h3"
        cells[str(c)] = rec
    return {"cells": cells, "n_cells": len(cells), "pass": bool(cells) and all(r["ok"] for r in cells.values()),
            "required_pool": "bars_h3", "kernel_file": str(KERNEL), "kernel_exists": KERNEL.exists(),
            "trace_reader_exists": TRACE_READER.exists()}


def strip(o):
    o = G.strip_timing(o)
    if isinstance(o, dict):
        return {k: strip(v) for k, v in o.items() if k not in TIMING_KEYS}
    if isinstance(o, list):
        return [strip(v) for v in o]
    return o


def judge(name, rec, data):
    """Replace run_one's REPORTED/FAIL with this gate's fail-closed rule."""
    ok_exit = rec["exit"] == 0
    if rec["verdict"] == "VOID_HOST_FAULT":
        return rec
    if name == "v1":
        v = data.get("pass") if isinstance(data, dict) else None
        rec["verdict_value"] = v
        rec["verdict"] = "PASS" if (v is True and ok_exit) else "FAIL"
    else:
        reading = data.get("reading") if isinstance(data, dict) else None
        fc = data.get("stats", {}).get("false_cert") if isinstance(data, dict) else None
        rec["verdict_value"] = reading
        rec["false_cert"] = fc
        rec["verdict"] = ("REPORTED" if (ok_exit and reading in READINGS and reading != "FAIL_INCOMPLETE"
                                         and reading != "STOP_FALSE_CERT" and fc == 0)
                          else ("STOP_FALSE_CERT" if reading == "STOP_FALSE_CERT" or (fc or 0) > 0 else "FAIL"))
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
            else "NOT_RECORDED", "prereg": "docs/plans/prereg_cplanner_m2_v1.md"}
    (out / "meta.json").write_text(json.dumps(meta, indent=1) + "\n")
    pre = preflight()
    (out / "preflight.json").write_text(json.dumps(pre, indent=1) + "\n")
    bound = inputs()
    (out / "inputs_before.json").write_text(json.dumps(bound, indent=1) + "\n")
    records, data, void = [], {}, False
    if pre["pass"] and pre["kernel_exists"] and pre["trace_reader_exists"]:
        for phase in ("main", "dup"):
            for name, argv, tmo in PLAN:
                argv = argv if phase == "main" else [a.replace("{o}/", "{o}/dup/") for a in argv]
                label = name if phase == "main" else "dup_" + name
                print("==", label, datetime.datetime.now(G.KST).isoformat(), flush=True)
                rec, d = G.run_one(out, logs, name, argv, tmo, None, "M2", label)
                rec = judge(name, rec, d)
                records.append(rec)
                data[label] = d
                print("  ", rec["verdict"], rec.get("verdict_value"), "exit", rec["exit"], "%.0fs" % rec["elapsed_s"],
                      flush=True)
                (out / "records.json").write_text(json.dumps(records, indent=1) + "\n")
                if rec["verdict"] == "VOID_HOST_FAULT":
                    void = True
                    break
            if void:
                break
    bound_after = inputs()
    (out / "inputs_after.json").write_text(json.dumps(bound_after, indent=1) + "\n")
    changed = sorted(p for p in bound.keys() | bound_after.keys() if bound.get(p) != bound_after.get(p))
    det = {}
    if not void:
        for name, *_ in PLAN:
            a, b = data.get(name), data.get("dup_" + name)
            det[name] = a is not None and b is not None and strip(a) == strip(b)
    by = {r["label"]: r for r in records}
    ga = by.get("galpha", {})
    summary = {
        "preflight_pass": pre["pass"], "v1": by.get("v1", {}).get("verdict", "NOT_RUN"),
        "galpha_reading": ga.get("verdict_value", "NOT_RUN"), "galpha_verdict": ga.get("verdict", "NOT_RUN"),
        "galpha_false_cert": ga.get("false_cert"),
        "certifiable_rate": (data.get("galpha") or {}).get("certifiable_rate"),
        "certifiable_rate_by_group": (data.get("galpha") or {}).get("certifiable_rate_by_group"),
        "duplicates": {r["label"]: r["verdict"] for r in records if r["label"].startswith("dup_")},
        "determinism": det, "inputs_unchanged": not changed, "changed_inputs": changed[:50], "host_void": void,
        "commands_run": len(records), "commands_expected": 2 * len(PLAN),
    }
    complete = (not void and pre["pass"] and len(records) == 2 * len(PLAN) and bool(det) and all(det.values())
                and not changed and all(by.get(l, {}).get("verdict") == "PASS" for l in ("v1", "dup_v1"))
                and all(by.get(l, {}).get("verdict") == "REPORTED" for l in ("galpha", "dup_galpha")))
    if void:
        summary["verdict"] = "VOID_HOST_FAULT"
    elif ga.get("verdict") == "STOP_FALSE_CERT":
        summary["verdict"] = "STOP_FALSE_CERT"
    else:
        summary["verdict"] = "CONDITIONAL" if complete else "FAIL"
    summary["verdict_scope"] = ("V1 rules 1-4 + G-alpha reading (charter §3) + preflight bar pool + determinism + "
                                "host + inputs; decided only after an independent review")
    summary["host_end"] = G.host()
    summary["finished"] = datetime.datetime.now(G.KST).isoformat()
    (out / "m2_summary.json").write_text(json.dumps(summary, indent=1) + "\n")
    print(json.dumps({k: summary[k] for k in ("verdict", "v1", "galpha_reading", "certifiable_rate", "determinism",
                                                "inputs_unchanged", "commands_run")}, indent=1))
    return 0 if summary["verdict"] == "CONDITIONAL" else 1


if __name__ == "__main__":
    raise SystemExit(main())
