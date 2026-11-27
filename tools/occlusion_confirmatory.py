#!/usr/bin/env python3
"""OCC-P2: committed, frozen-source, fresh-output confirmatory execution and preservation.

`preflight` never launches a simulation. `run --approved-commit SHA` launches only
the preregistered OCC-P2 matrix after every condition passes. No resume/overwrite.
"""
from __future__ import annotations

import argparse
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
PLAN = ROOT / "docs/plans/prereg_occlusion_confirmatory_v1.json"
PREREG = ROOT / "docs/plans/prereg_occlusion_confirmatory_v1.md"


def load_tool(name):
    spec = importlib.util.spec_from_file_location("_occ_p2_" + name, ROOT / "tools" / (name + ".py"))
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


legacy = load_tool("kroc2027_eval")


def emit_new(path, value):
    with Path(path).open("x", encoding="utf-8") as stream:
        json.dump(value, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")


def atomic_progress(path, value):
    temp = Path(str(path) + ".tmp")
    temp.write_text(json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n")
    os.replace(temp, path)


def matrix(plan):
    d = plan["design"]
    return [dict(group=g, seed=s, density=b, search=None,
                 cell_dir="%s/%s/%s" % (g, d["groups"][g]["arm"],
                                        legacy.kp.cell_label(d["groups"][g]["target_model"], b, s)),
                 **{k: d["groups"][g][k] for k in ("arm", "lineage", "target_model", "view")})
            for s in d["seeds"] for b in d["densities"] for g in ("static", "moving")]


def verify_preserved(plan):
    """Full read-only raw verification, including stopped pilot attempts."""
    import numpy as np
    problems, checked, manifests = [], 0, {}
    for binding in plan["frozen"]["prior_manifests"]:
        path = Path(binding["path"])
        if legacy.kp.sha256_file(path) != binding["sha256"]:
            problems.append("manifest bytes changed: " + str(path))
        data = json.loads(path.read_text())
        for relative, digest in data["files"].items():
            file = path.parent / relative
            if not file.is_file() or legacy.kp.sha256_file(file) != digest:
                problems.append("raw bytes changed: " + str(file))
            checked += 1
        manifests[str(path)] = {"sha256": binding["sha256"], "files_verified": len(data["files"])}
    scenes = set()
    pilot = Path(plan["frozen"]["pilot_root"])
    pilot_seeds = set()
    for path in sorted(pilot.rglob("occlusion_layouts.npz")):
        with np.load(path, allow_pickle=False) as layouts:
            scenes.update(str(v) for v in layouts["hashes"])
        metadata = json.loads((path.parent / "occlusion_trace_summary.json").read_text())["metadata"]
        pilot_seeds.add(int(metadata["context"]["seed"]))
    if pilot_seeds != set(plan["design"]["pilot_seeds"]):
        problems.append("recorded pilot seeds differ from declared pilot seed set")
    if set(plan["design"]["seeds"]) & pilot_seeds:
        problems.append("confirmatory and exploratory seed sets overlap")
    import hashlib
    digest = hashlib.sha256(("\n".join(sorted(scenes)) + "\n").encode()).hexdigest()
    return {"problems": problems, "files_verified": checked, "manifests": manifests,
            "pilot_seeds": sorted(pilot_seeds), "pilot_unique_scene_hashes": len(scenes),
            "pilot_scene_set_sha256": digest}, scenes


def preflight(plan, approved_commit):
    head = legacy.git("rev-parse", "HEAD").strip()
    protocol = legacy.kp.load_protocol(legacy.PROTOCOL)
    # No inherited experiment knobs; launcher defaults are frozen and explicitly recorded.
    env = legacy.base_env(inherited={k: v for k, v in os.environ.items() if not k.startswith("NAVRL_")})
    problems = []
    if approved_commit != head:
        problems.append("approved source HEAD must match the full current commit")
    if legacy.git("status", "--porcelain", "--untracked-files=no").strip():
        problems.append("tracked tree is dirty")
    for path in (PLAN, PREREG, Path(__file__).resolve(), ROOT / "tools/occlusion_confirmatory_analysis.py"):
        relative = str(path.relative_to(ROOT))
        if subprocess.run(["git", "ls-files", "--error-unmatch", relative], cwd=ROOT,
                          capture_output=True).returncode:
            problems.append("uncommitted preregistration/code: " + relative)
    frozen = plan["frozen"]
    problems += legacy.simulation_source_problems(frozen["g0_commit"], head, frozen["runtime_paths"])
    g0_path = Path(frozen["g0_report"])
    if legacy.kp.sha256_file(g0_path) != frozen["g0_report_sha256"]:
        problems.append("G0 report SHA mismatch")
    g0 = json.loads(g0_path.read_text())
    if g0.get("verdict") != "G0_PASS":
        problems.append("G0 was not PASS")
    pilot = Path(frozen["pilot_root"])
    if legacy.kp.sha256_file(pilot / "analysis.json") != frozen["pilot_analysis_sha256"]:
        problems.append("exploratory analysis SHA changed")
    problems += legacy.static_problems(protocol)
    import_issues, origins = legacy.import_problems(env, legacy.launcher_path(protocol).parent)
    problems += import_issues
    gpu = legacy.gpu_names()
    if gpu != [protocol["hardware"]["gpu"]]:
        problems.append("GPU differs from preregistered frozen platform")
    output = Path(plan["output_root"])
    if output.exists() or output.is_symlink():
        problems.append("output already exists; never resume or overwrite")
    ancestor = output
    while not ancestor.exists():
        ancestor = ancestor.parent
    free = legacy.free_disk_gb(ancestor)
    if free < plan["safety"]["min_free_disk_gb"]:
        problems.append("disk below 8 GB")
    preserved, scenes = verify_preserved(plan)
    problems += preserved["problems"]
    cells = matrix(plan)
    if len(cells) != 112 or len({c["cell_dir"] for c in cells}) != 112:
        problems.append("matrix is not 112 distinct cells")
    report = {"status": "PASS" if not problems else "REFUSED", "problems": problems,
              "source_commit": head, "source_tree": legacy.git("rev-parse", "HEAD^{tree}").strip(),
              "checkpoint_sha256": legacy.policy_f_sha(protocol), "gpu": gpu,
              "import_origins": origins, "output_root": str(output.resolve()),
              "free_disk_gb": free, "plan_sha256": legacy.kp.sha256_file(PLAN),
              "prereg_sha256": legacy.kp.sha256_file(PREREG), "preservation": preserved,
              "cells": len(cells), "primary_episodes": len(cells) * 128 * 4,
              "g0_report_sha256": frozen["g0_report_sha256"],
              "source_g0_runtime_diff": "NONE" if not legacy.simulation_source_problems(
                  frozen["g0_commit"], head, frozen["runtime_paths"]) else "DIFFERENT"}
    return report, protocol, env, scenes


def terminate_group(proc):
    try:
        os.killpg(proc.pid, signal.SIGTERM)
    except ProcessLookupError:
        pass
    try:
        proc.wait(timeout=10)
    except subprocess.TimeoutExpired:
        pass
    # The Bash leader can exit while a TERM-ignoring GPU descendant remains alive.
    # Always kill the original owned group, regardless of the leader's wait result.
    try:
        os.killpg(proc.pid, signal.SIGKILL)
    except ProcessLookupError:
        pass
    proc.wait(timeout=10)


def supervised_launch(command, env, log_path, timeout_s, stop_check, poll_s=2.0):
    """Continuously guard an entire process group; keep all partial outputs."""
    start = time.monotonic()
    with Path(log_path).open("a", encoding="utf-8") as log:
        proc = subprocess.Popen(command, env=env, stdout=log, stderr=subprocess.STDOUT,
                                start_new_session=True)
        reason = None
        try:
            while proc.poll() is None:
                reason = stop_check()
                if reason is None and time.monotonic() - start >= timeout_s:
                    reason = "CELL_OR_WALL_CLOCK_CAP"
                if reason:
                    terminate_group(proc)
                    break
                time.sleep(poll_s)
        except BaseException:
            terminate_group(proc)
            raise
        finally:
            # Also clean descendants when the launcher leader exits unexpectedly on its own.
            try:
                os.killpg(proc.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
    return proc.returncode, time.monotonic() - start, reason


def scene_overlap(directory, pilot_scenes):
    import numpy as np
    with np.load(Path(directory) / "occlusion_layouts.npz", allow_pickle=False) as data:
        scenes = set(str(v) for v in data["hashes"])
    return sorted(scenes & pilot_scenes), len(scenes)


def write_manifest(root):
    files = {str(p.relative_to(root)): legacy.kp.sha256_file(p)
             for p in sorted(root.rglob("*")) if p.is_file() and p.name != "RAW_MANIFEST.json"}
    emit_new(root / "RAW_MANIFEST.json", {"schema": "motar.raw-manifest.v1", "root": root.name,
                                          "files": files, "file_count": len(files)})


def run(plan, pre, protocol, env, scenes):
    output = Path(plan["output_root"])
    output.mkdir(parents=True, exist_ok=False)
    emit_new(output / "launch_receipt.json", dict(pre, started_at=legacy.datetime.now(legacy.timezone.utc).isoformat(),
                                                 authorization=plan["authority"]))
    emit_new(output / "pilot_scene_hashes.json", sorted(scenes))
    report = {"run_id": plan["run_id"], "source_commit": pre["source_commit"], "cells": {},
              "stop_reason": None, "verdict": "RUNNING", "code_and_worklog_frozen": True}
    start = time.monotonic()
    deadline = start + plan["safety"]["wall_clock_cap_hours"] * 3600
    head = pre["source_commit"]

    def guard():
        if time.monotonic() >= deadline:
            return "WALL_CLOCK_CAP"
        if legacy.free_disk_gb(output) < plan["safety"]["min_free_disk_gb"]:
            return "FREE_DISK_BELOW_8_GB"
        if legacy.git("rev-parse", "HEAD").strip() != head or legacy.git(
                "status", "--porcelain", "--untracked-files=no").strip():
            return "TRACKED_SOURCE_OR_WORKLOG_CHANGED"
        return None

    mt_plan = json.loads((ROOT / legacy.MT_PLAN).read_text())
    try:
        for index, cell in enumerate(matrix(plan), 1):
            reason = guard()
            if reason:
                report["stop_reason"] = reason
                break
            directory = output / cell["cell_dir"]
            if directory.exists():
                raise FileExistsError("cell already exists: " + str(directory))
            variables = legacy.cell_env(env, protocol, target_model=cell["target_model"],
                        density=cell["density"], seed=cell["seed"], out_dir=directory.parent,
                        quota=4, view="visible", recorder=True, run_id=plan["run_id"], arm=cell["arm"])
            variables.update(plan["design"]["groups"][cell["group"]]["extra_env"])
            variables.update(plan["telemetry_env"])
            rc, seconds, stopped = supervised_launch(["bash", str(legacy.launcher_path(protocol))],
                variables, output / "runner.log", min(plan["safety"]["cell_cap_seconds"], deadline-time.monotonic()),
                guard, plan["safety"]["poll_seconds"])
            issues = [] if rc == 0 else ["launcher exit %s" % rc]
            if stopped:
                issues.append(stopped)
            issues += legacy.check_cell(directory, protocol, 4, require_motion=True,
                                         expect=legacy.cell_expect(cell, plan["run_id"], head, 4))
            issues += legacy.occlusion_trace_problems(directory)
            if cell["group"] == "moving":
                issues += legacy.mt_identity_problems(directory, mt_plan)
            overlap, n_scenes = ([], 0)
            if (directory / "occlusion_layouts.npz").is_file():
                overlap, n_scenes = scene_overlap(directory, scenes)
                if overlap:
                    issues.append("OCC-P1 scene reuse: %d hashes" % len(overlap))
            report["cells"][cell["cell_dir"]] = {"issues": issues, "seconds": seconds,
                "free_disk_gb_after": legacy.free_disk_gb(output), "output_bytes": legacy.dir_bytes(directory),
                "pilot_scene_overlap": overlap, "scene_hashes": n_scenes}
            atomic_progress(output / "run_report.json", report)
            print(json.dumps({"completed": index, "total": 112, "cell": cell["cell_dir"],
                              "seconds": round(seconds, 1), "issues": issues,
                              "free_disk_gb": round(legacy.free_disk_gb(output), 2)}), flush=True)
            if issues:
                report["stop_reason"] = stopped or "FIRST_FAILED_CELL: " + cell["cell_dir"]
                break
    except BaseException as exc:
        report["stop_reason"] = "%s: %s" % (type(exc).__name__, exc)
    report.update(verdict="COMPLETE" if len(report["cells"]) == 112 and not report["stop_reason"] else "STOPPED",
                  wall_hours=(time.monotonic()-start)/3600, free_disk_gb_after=legacy.free_disk_gb(output),
                  checkpoint_sha256_after=legacy.policy_f_sha(protocol), finished_at=legacy.datetime.now(legacy.timezone.utc).isoformat())
    if report["checkpoint_sha256_after"] != pre["checkpoint_sha256"]:
        report["stop_reason"], report["verdict"] = "CHECKPOINT_CHANGED", "STOPPED"
    # GPU child is gone before these read-only CPU checks. No tracked writes during execution.
    preservation, _ = verify_preserved(plan)
    emit_new(output / "preservation_after.json", preservation)
    if preservation["problems"]:
        report["verdict"], report["stop_reason"] = "STOPPED", "PRIOR_RAW_CHANGED"
    atomic_progress(output / "run_report.json", report)
    if report["verdict"] == "COMPLETE":
        try:
            analysis = load_tool("occlusion_confirmatory_analysis").analyse(output, plan)
            emit_new(output / "analysis.json", analysis)
        except Exception as exc:
            report["analysis_error"] = "%s: %s" % (type(exc).__name__, exc)
            atomic_progress(output / "run_report.json", report)
    write_manifest(output)
    print(json.dumps({k: v for k, v in report.items() if k != "cells"}, indent=2), flush=True)
    return 0 if report["verdict"] == "COMPLETE" and not report.get("analysis_error") else 1


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("preflight", "run", "analyse"))
    parser.add_argument("--approved-commit", default="")
    parser.add_argument("--out", type=Path)
    args = parser.parse_args(argv)
    plan = json.loads(PLAN.read_text())
    if args.command == "analyse":
        result = load_tool("occlusion_confirmatory_analysis").analyse(Path(plan["output_root"]), plan)
        if args.out:
            emit_new(args.out, result)
        else:
            print(json.dumps(result, indent=2, allow_nan=False))
        return 0
    pre, protocol, env, scenes = preflight(plan, args.approved_commit)
    if args.command == "preflight":
        if args.out:
            emit_new(args.out, pre)
        print(json.dumps(pre, indent=2, allow_nan=False))
        return 0 if pre["status"] == "PASS" else 2
    if pre["problems"]:
        print(json.dumps(pre, indent=2))
        return 2
    return run(plan, pre, protocol, env, scenes)


if __name__ == "__main__":
    sys.exit(main())
