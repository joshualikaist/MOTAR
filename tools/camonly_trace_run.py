"""Camera-only G3: per-step traces of C2 cells (docs/plans/prereg_camonly_c2trace_v1.md).

Reruns 12 C2 cells (conditions full, no_lidar_out_fov and no_lidar; 70/115/160/205 bars; seed 4501; quota 16)
through the unchanged camonly cell path (gr_run.run_cell with the camera-only launcher copy, the ablation entry point
and its receipt check), with one addition: NAVRL_KROC2027_OCCLUSION_TRACE=1, the KROC2027 recorder's compact binary
telemetry (both UAVs' states per consumed frame and every reset's obstacle layout; detached tensors only).

Noninterference gate: each traced cell's episodes.jsonl must be byte-identical to its untraced reference
(full: GR-0 S00; the others: C2), and its result.json outcome counts equal. Reading NONINTERFERENCE_PASS (all 12) or
TRACE_INTERFERES (the traces then describe these runs only, not C2's). Each cell's trace must also read back with
motar_eval/occlusion_trace.read_trace (both SHA-256 values checked).
Fail-closed preflight as tools/camonly_run.py, plus this preregistration committed.
usage: tools/camonly_trace_run.py --approved-commit SHA --out DIR [--max-hours H]
"""
import argparse
import hashlib
import importlib.util
import json
import pathlib
import subprocess
import sys

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(HERE))
import camonly_run as C                                     # noqa: E402

G, K = C.G, C.K
PREREG = "docs/plans/prereg_camonly_c2trace_v1.md"
CONDITIONS = ("full", "no_lidar_out_fov", "no_lidar")
DENSITIES = (70, 115, 160, 205)
SEED = 4501
RES = pathlib.Path("/home/fair/workspaces/aerial_gym_ws/src/aerial_gym_simulator/results")
REF = {"full": RES / "gain_randomization_gr0_v1/gr0_main/S00",
       "no_lidar_out_fov": RES / "camonly_c1c2_v1/c2/no_lidar_out_fov",
       "no_lidar": RES / "camonly_c1c2_v1/c2/no_lidar"}
TRACE = {"NAVRL_KROC2027_OCCLUSION_TRACE": "1"}


def load_ot():
    spec = importlib.util.spec_from_file_location("occlusion_trace_c2t", str(ROOT / "motar_eval/occlusion_trace.py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def sha(p):
    return hashlib.sha256(pathlib.Path(p).read_bytes()).hexdigest()


def noninterference(cell_dir, ref_dir):
    out = {"ref": str(ref_dir)}
    try:
        out["episodes_equal"] = sha(cell_dir / "episodes.jsonl") == sha(ref_dir / "episodes.jsonl")
        out["kroc_equal"] = sha(cell_dir / "kroc2027_episodes.jsonl") == sha(ref_dir / "kroc2027_episodes.jsonl")
        a = json.loads((cell_dir / "result.json").read_text())["outcome"]
        b = json.loads((ref_dir / "result.json").read_text())["outcome"]
        out["outcome_equal"] = all(a[k] == b[k] for k in ("captured", "crash", "timeout"))
    except (OSError, ValueError, KeyError) as e:
        out["error"] = str(e)
        out["episodes_equal"] = out["outcome_equal"] = False
    out["pass"] = bool(out["episodes_equal"] and out["outcome_equal"])
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--approved-commit", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--max-hours", type=float, default=1.5)
    args = ap.parse_args()
    out_root = pathlib.Path(args.out).resolve()
    protocol = json.loads(K.PROTOCOL.read_text())
    head, problems, origins = C.preflight(protocol, args.approved_commit, out_root)
    if subprocess.run(["git", "-C", str(ROOT), "ls-files", "--error-unmatch", PREREG], capture_output=True).returncode:
        problems.append("%s is not committed" % PREREG)
    for cond, ref in REF.items():
        for d in DENSITIES:
            if not (ref / ("E2_obstacle_aware__%dbars__seed%d" % (d, SEED)) / "episodes.jsonl").is_file():
                problems.append("reference missing: %s %d" % (cond, d))
    module_sha = G.sha(ROOT / C.MODULE)
    start = G.host()
    report = {"unit": "camonly G3 trace", "prereg": PREREG, "tree": str(ROOT), "commit": head, "started": G.now(),
              "host_start": start, "import_origins": origins, "module_sha256": module_sha,
              "launcher_sha256": C.LAUNCHER_SHA, "trace_env": TRACE, "preflight_problems": problems}
    if problems:
        report["reading"] = "PREFLIGHT_FAIL"
        print(json.dumps(report, indent=1, default=str))
        return 2
    out_root.mkdir(parents=True)
    OT = load_ot()
    budget = K.GpuBudget(args.max_hours)
    records, cells, void, stop = [], [], False, False
    for d in DENSITIES:
        for cond in CONDITIONS:
            if budget.exhausted():
                report["budget_exhausted"] = True
                stop = True
                break
            extra = {"NAVRL_SENSOR_ABLATION": cond, "CAMONLY_PY": str(C.WRAPPER)}
            extra.update(TRACE)
            rec = G.run_cell(ROOT, protocol, out_root, cond, d, SEED, C.QUOTA, "CAMONLY-C2T", None, head, start,
                             records, G.CELL_TIMEOUT_S, extra, C.LAUNCHER, G.F_SHA)
            cell_dir = out_root / cond / rec["cell_id"]
            if rec["verdict"] == "OK":
                extra_problems = C.ablation_problems(cell_dir, cond, module_sha)
                try:
                    tr = OT.read_trace(cell_dir)
                    rec["trace_frames"], rec["trace_layouts"] = int(len(tr["frames"])), int(tr["summary"]["layouts"])
                except (OSError, ValueError, KeyError) as e:
                    extra_problems.append("trace unreadable: %s" % e)
                if extra_problems:
                    rec["problems"].extend(extra_problems)
                    rec["verdict"] = "FAIL"
            ni = noninterference(cell_dir, REF[cond] / rec["cell_id"]) if rec["verdict"] == "OK" else None
            cells.append({"condition": cond, "density": d, "seed": SEED, "dir": str(cell_dir), "verdict": rec["verdict"],
                          "noninterference": ni, "trace_frames": rec.get("trace_frames")})
            print("   trace", cond, d, rec["verdict"], ni and ni["pass"], rec.get("trace_frames"), flush=True)
            if rec["verdict"] != "OK":
                void = rec["verdict"] == "VOID_HOST_FAULT"
                stop = True
                break
        if stop:
            break
    complete = len(cells) == len(CONDITIONS) * len(DENSITIES) and all(c["verdict"] == "OK" for c in cells)
    if void:
        reading = "VOID_HOST_FAULT"
    elif not complete:
        reading = "INCOMPLETE"
    else:
        reading = "NONINTERFERENCE_PASS" if all(c["noninterference"]["pass"] for c in cells) else "TRACE_INTERFERES"
    report.update(reading=reading, cells=cells, records=records, host_end=G.host(), finished=G.now())
    (out_root / "c2trace_run_report.json").write_text(json.dumps(report, indent=1, default=str) + "\n")
    print(json.dumps({k: report[k] for k in ("unit", "reading", "commit")}, indent=1))
    return 0 if reading in ("NONINTERFERENCE_PASS", "TRACE_INTERFERES") else 1


if __name__ == "__main__":
    raise SystemExit(main())
