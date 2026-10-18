#!/usr/bin/env python3
"""H-R3 P02 exploratory orchestrator (docs/plans/h_r3_p02.json): frozen preflight, watchdog, no retry.

Reuses the qualified S-L1 supervision read-only. Arms B0-B5 run as separate processes with the same
seed and configuration (deterministic replay). Importing launches nothing; smoke runs are
mechanics/safety only, never evidence.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT / "tools") not in sys.path:
    sys.path.insert(0, str(ROOT / "tools"))
import s_l1b_run as b  # noqa: E402

a, base, el0 = b.a, b.base, b.el0
PLAN_PATH = ROOT / "docs/plans/h_r3_p02.json"
LAUNCHER = ROOT / "tools/h_r3_p02_launcher.py"
ARMS = ("B0", "B1", "B2", "B3", "B4", "B5")


def load_plan():
    p = a.read_json(PLAN_PATH)
    p["run"]["closed_loop_density"] = p["run"]["density"]
    return p


def process_specs(p, seeds, num_envs, steps):
    return [{"id": "seed%d_%s" % (seed, arm), "batch": "HR3", "hr3_arm": arm, "config": p["run"]["config"], "arm": "P0",
             "seed": seed, "num_envs": num_envs, "steps": steps, "cap_s": p["run"]["process_wall_cap_s"],
             "lineage": p["switch"]["value_used"]} for seed in seeds for arm in ARMS]


def preflight(p, approved, run_dir, smoke=False):
    old = a.read_json(base.resolve(p["references"]["eq1d_plan"]))
    # P02 mechanics output is preregistered under results/ and requires the same
    # exact-HEAD/clean-tree guards as main. EQ-1D's temporary-smoke relaxation is inapplicable.
    problems, checks = base.preflight(old, approved, run_dir, False)
    paths = (PLAN_PATH, PLAN_PATH.with_suffix(".md"), ROOT / "docs/plans/h_r3_p02_amendment1.md", Path(__file__).resolve(), LAUNCHER,
             ROOT / "motar_eval/h_r3_p02_arms.py", ROOT / "motar_eval/h_r3_p02_identity.py", ROOT / "tools/h_r3_p02_rng.py",
             ROOT / "tools/h_r3_p02_analysis.py", ROOT / "tests/test_h_r3_p02_arms.py", ROOT / "tests/test_h_r3_p02_identity.py",
             ROOT / "tests/test_h_r3_p02_rng.py", ROOT / "tests/test_h_r3_p02_analysis.py", ROOT / "tests/test_h_r3_p02_runner.py", ROOT / "tools/h_r3_p02_planning.py", ROOT / "tests/test_h_r3_p02_planning.py")
    checks["hr3_committed_inputs"] = {str(x.relative_to(ROOT)): a.committed(x) for x in paths}
    checks["hr3_input_sha256"] = {str(x.relative_to(ROOT)): base.sha256_file(x) for x in paths if x.is_file()}
    if not all(checks["hr3_committed_inputs"].values()):
        problems.append("H-R3 plan/code not committed unchanged")
    if checks.get("policy_f_sha256") != p["policy_f_sha256"]:
        problems.append("Policy F binding differs")
    checks["untracked_runtime_files"] = base.git("ls-files", "--others", "--exclude-standard", "--",
                                                "aerial_gym", "motar_eval", "tools").stdout.splitlines()
    if checks["untracked_runtime_files"]:
        problems.append("untracked runtime source would be outside the approved commit")
    if not smoke:
        evidence=ROOT / "docs/evidence/h_r3_p02_2026-10-18"
        try:
            gates=a.read_json(evidence/"smoke_analysis.json")
            if gates["smoke_status"]!="PASS":problems.append("P02 smoke not PASS")
            prior=a.read_json(base.resolve(p["run"]["output_root"])/"HR3-P02-SMOKE01/run_report.json")
            if prior["preflight"]["hr3_input_sha256"]!=checks["hr3_input_sha256"]:problems.append("P02 source differs from smoke")
            if not a.committed(evidence/"smoke_analysis.json"):problems.append("P02 smoke result not committed")
            checks["smoke_analysis_sha256"]=base.sha256_file(evidence/"smoke_analysis.json")
        except Exception as error:problems.append("Missing/invalid P02 smoke: "+str(error))
    return problems, checks


def validate(out, spec, checks):
    done = a.validate_done(out, spec, checks)
    meta = a.read_json(out / "h_r3_p02_meta.json")
    if done.get("actor_obs_dim")!=898:raise ValueError("actor898 missing")
    if any(meta.get(k)!=spec[k] for k in ("seed","num_envs","steps")):raise ValueError("completed process config mismatch")
    if meta.get("sensor_lineage")!="off" or meta.get("step_dt")!=0.1:raise ValueError("sensor lineage/timestep changed")
    if meta["arm"] != spec["hr3_arm"] or meta["rms_digest_start"] != checks["rms_digest"] or meta["rms_digest_end"] != checks["rms_digest"]:
        raise ValueError("arm identity or RMS changed")
    if meta["model_training_flag"] is not False or not meta["vision_mode"]:
        raise ValueError("actor not in evaluation/vision mode")
    if any(ROOT.resolve() not in Path(meta["origins"][key]).resolve().parents for key in ("aerial_gym", "motar_eval")):
        raise ValueError("import origin outside the worktree")
    return done


def diagnostic_match(run_dir,spec):
    """Stop a mechanics run immediately if an unrelated reset or evaluator changes another row."""
    import numpy as np
    normal=run_dir/("seed%d_B0"%spec["seed"])/"h_r3_p02_trace.npz"
    variant=run_dir/spec["id"]/"h_r3_p02_trace.npz"
    with np.load(normal,allow_pickle=False) as a0,np.load(variant,allow_pickle=False) as b0:
        mask=np.arange(spec["num_envs"])!=15 if spec.get("force_reset") else np.ones(spec["num_envs"],bool)
        for key in ("digest","sensor_digest","action","rng_probe"):
            if not np.array_equal(a0[key][:,mask],b0[key][:,mask]):
                raise ValueError("P02 diagnostic crosstalk: %s %s"%(spec["id"],key))
    return {"PASS":True,"keys":["digest","sensor_digest","action","rng_probe"],"excluded_envs":[15] if spec.get("force_reset") else []}


def run(p, run_dir, specs, smoke, checks):
    run_dir.mkdir(parents=True, exist_ok=False)
    started = time.monotonic()
    deadline = started + p["run"]["total_wall_cap_s"]
    source_head = base.git("rev-parse", "HEAD").stdout.strip()
    baseline = base.git("status", "--porcelain", "--untracked-files=no").stdout
    p["run"]["disk_min_free_gb"] = p["run"]["disk_min_free_gb"]
    report = {"schema": "motar.h-r3-p02-run-report.v1", "run_dir": str(run_dir), "smoke": smoke, "preflight": checks,
              "effective_processes": specs, "processes": [], "source_commit": source_head, "status": "RUNNING",
              "stop_reason": None, "plan_sha256": base.sha256_file(PLAN_PATH)}
    try:
        for spec in specs:
            reason, _ = a.runtime_guard(p, run_dir, source_head, baseline, deadline)
            if reason:
                report["stop_reason"] = reason
                break
            gpu = base.gpu_state()
            cap = gpu["total_mib"] - gpu["used_mib"] - p["run"]["oom_reserve_mib"]
            if cap <= 0 or "3070" not in gpu["name"]:
                raise ValueError("invalid launch-time GPU platform or memory headroom")
            entry = {"id": spec["id"], "gpu_at_launch": gpu, "gpu_memory_cap_mib": cap, "free_disk_gb_before": base.free_gb(run_dir)}
            report["processes"].append(entry)
            out = run_dir / spec["id"]
            command = [base.PYTHON, "-B", str(LAUNCHER), "--arm", spec["hr3_arm"], "--num-envs", str(spec["num_envs"]),
                       "--steps", str(spec["steps"]), "--out", str(out), "--process-id", spec["id"], "--seed", str(spec["seed"])]
            if spec.get("force_reset"):command.append("--force-reset")
            if spec.get("no_identity"):command.append("--no-identity")
            entry["command"] = command
            process_deadline = time.monotonic() + spec["cap_s"]
            guard = (lambda proc, process_deadline=process_deadline, cap=cap: a.runtime_guard(
                p, run_dir, source_head, baseline, deadline, process_deadline=process_deadline, proc=proc, cap_mib=cap))
            rc, wall, peak, reason = a.supervised_launch(command, b.process_env(p, spec, out), run_dir / (spec["id"] + ".log"),
                                                         guard, p["run"]["poll_s"])
            entry.update(returncode=rc, wall_s=wall, peak_gpu_mib=peak)
            if reason or rc != 0:
                report["stop_reason"] = reason or "NONZERO_EXIT: %s: %s" % (spec["id"], rc)
                break
            reason,_=a.runtime_guard(p,run_dir,source_head,baseline,deadline)
            if reason:
                report["stop_reason"]=reason
                break
            entry["process_done"] = validate(out, spec, checks)
            if spec.get("force_reset") or spec.get("no_identity"):entry["diagnostic_match"]=diagnostic_match(run_dir,spec)
            print("[h-r3-p02] %s rc=%d wall=%.1fs peak=%dMiB" % (spec["id"], rc, wall, peak), flush=True)
    except Exception as error:
        report["stop_reason"] = "INTEGRITY_EXCEPTION: %s: %s" % (type(error).__name__, error)
    report["status"] = "STOPPED_INTEGRITY" if report["stop_reason"] else "COMPLETED"
    report["wall_s_total"] = time.monotonic() - started
    report["gpu_seconds_total"] = sum(item.get("wall_s", 0) for item in report["processes"])
    report["output_bytes_total"] = base.dir_bytes(run_dir)
    report["free_disk_gb_after"] = base.free_gb(run_dir)
    old = a.read_json(base.resolve(p["references"]["eq1d_plan"]))
    report["policy_f_sha256_after"] = base.sha256_file(base.RL_DIR / old["frozen"]["policy_f_rel_to_rl_games"])
    if report["policy_f_sha256_after"] != p["policy_f_sha256"]:
        report["status"], report["stop_reason"] = "STOPPED_INTEGRITY", "POLICY_F_CHANGED"
    with (run_dir / "run_report.json").open("x") as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
        stream.write("\n")
    report["manifest_members"] = base.write_manifest(run_dir)
    return report


def main(argv=None):
    ap=argparse.ArgumentParser(description=__doc__.split("\n\n")[0]);ap.add_argument("stage",choices=("preflight","smoke","run"))
    ap.add_argument("--approved-commit",required=True);ap.add_argument("--smoke",action="store_true")
    args=ap.parse_args(argv);p=load_plan();smoke=args.stage=="smoke" or args.smoke
    if smoke:
        seeds=[p["run"]["smoke"]["seed"]];n=p["run"]["smoke"]["num_envs"];steps=p["run"]["smoke"]["steps"]
        run_dir=base.resolve(p["run"]["output_root"])/"HR3-P02-SMOKE01"
    else:seeds=p["run"]["seeds"];n=p["run"]["num_envs"];steps=p["run"]["steps"];run_dir=base.resolve(p["run"]["output_root"])/p["run"]["run_id"]
    specs=process_specs(p,seeds,n,steps)
    if smoke:
        for suffix,key in (("forced","force_reset"),("noeval","no_identity")):
            spec=dict(specs[0]);spec.update(id="seed%d_B0_"%seeds[0]+suffix,**{key:True});specs.append(spec)
    if smoke:specs=[specs[0],specs[-2],specs[-1]]+specs[1:-2]
    problems,checks=preflight(p,args.approved_commit,run_dir,smoke)
    if args.stage=="preflight" or problems:
        print(json.dumps({"problems":problems,"checks":checks},indent=2,default=str));return 2 if problems else 0
    report=run(p,run_dir,specs,smoke,checks)
    print(json.dumps({k:report[k] for k in ("status","stop_reason","run_dir","gpu_seconds_total")},indent=2))
    return 0 if report["status"]=="COMPLETED" else 1

if __name__=="__main__":sys.exit(main())
