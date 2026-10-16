#!/usr/bin/env python3
"""KROC2027 evaluation: pilot, plan, validate, run (GPU, approval-gated) and analyse.

Protocol: docs/plans/kroc2027_protocol_v1.json; preregistration: docs/plans/prereg_kroc2027_A_v1.md.

    python -B tools/kroc2027_eval.py pilot [--raw-dir DIR]     # CPU; exploratory table from seen data
    python -B tools/kroc2027_eval.py plan --stage canary|main  # CPU; cells and GPU-hour estimate
    python -B tools/kroc2027_eval.py validate                   # CPU; hashes, gates, import origins
    python -B tools/kroc2027_eval.py run --stage canary --run-id KROC27-A-C01 --approved-commit SHA
    python -B tools/kroc2027_eval.py run --stage main --run-id KROC27-A-M01 --approved-commit SHA
    python -B tools/kroc2027_eval.py analyse                    # CPU; after every main cell

`run` never starts by itself. It refuses unless the tracked tree is clean, the preregistration is
committed, the pinned launcher and player match, the GPU is the RTX 3070, the run id belongs to the
stage, the approved commit is HEAD, and both ``aerial_gym`` and ``motar_eval`` resolve inside this
tree from the launcher's own directory (an editable install otherwise resolves to another checkout).
``--dry-run`` prints the plan and every problem and never launches anything. Each cell runs the
2026-09-18 launcher byte-for-byte; KROC2027 settings reach the task only through NAVRL_KROC2027_*.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from motar_eval import protocol as kp  # noqa: E402

PROTOCOL = ROOT / "docs/plans/kroc2027_protocol_v1.json"
PREREG = "docs/plans/prereg_kroc2027_A_v1.md"
PILOT_OUT = ROOT / "docs/plans/kroc2027_pilot_v1.json"
REPLICATION_DIR = ROOT / "results/target_motion_replication_7seed"
E2_AUDIT = ROOT / "tools/audits/audit_e2_target_validity.py"
RECORDER_EPISODES = "kroc2027_episodes.jsonl"
RECORDER_SUMMARY = "kroc2027_summary.json"
# Inherited variables that would silently change a cell; every run starts without them.
SCRUBBED_PREFIXES = ("NAVRL_KROC2027_",)
SCRUBBED_KEYS = (
    "NAVRL_TRAJECTORY_DIGEST", "NAVRL_TRAJECTORY_DIGEST_JSON", "NAVRL_TRAJECTORY_DIGEST_LABEL",
    "NAVRL_OBS_DUMP", "NAVRL_OBS_DUMP_STRIDE", "NAVRL_OBS_DUMP_MAX",
    "NAVRL_EPISODE_LEDGER", "NAVRL_EPISODE_QUOTA_PER_ENV", "NAVRL_PLAYER_GAMES_CAP",
    "ARMS", "DENSITIES", "SEEDS", "GAMES", "NUM_ENVS", "OUT_ROOT", "PYTHONPATH",
    "NAVRL_TARGET_DISTANCE_BACKEND",
)
OBS_DUMP_STRIDE, OBS_DUMP_MAX = 20, 65536
OFF_PATH_CELL = ("E0_static", 70, 4104)


# ---------------------------------------------------------------- tree, imports, environment
def launcher_path(protocol, root=ROOT) -> Path:
    return root / protocol["policy"]["launcher"]


def is_inside(path, root) -> bool:
    path, root = Path(path).resolve(), Path(root).resolve()
    return path == root or root in path.parents


def import_origin(module: str, cwd, env) -> str:
    """Where ``module`` resolves for a child started in ``cwd`` with ``env`` ('' if nowhere).

    ``find_spec`` locates the package without executing its ``__init__``, so no Isaac Gym import.
    The probe runs from the launcher's directory because the player does: ``runner.py`` puts that
    directory, not the repository root, first on ``sys.path``.
    """
    code = ("import importlib.util, sys\n"
            "spec = importlib.util.find_spec(sys.argv[1])\n"
            "print(spec.origin if spec is not None and spec.origin else '')\n")
    python = env.get("PYTHON", sys.executable)
    done = subprocess.run([python, "-B", "-c", code, module], cwd=str(cwd), env=env,
                          capture_output=True, text=True, timeout=120)
    lines = [line for line in done.stdout.strip().splitlines() if line.strip()]
    return lines[-1].strip() if done.returncode == 0 and lines else ""


def import_problems(env, cwd, root=ROOT):
    problems, origins = [], {}
    for module in ("aerial_gym", "motar_eval"):
        origin = import_origin(module, cwd, env)
        origins[module] = origin
        if not origin:
            problems.append("%s does not import from %s" % (module, cwd))
        elif not is_inside(origin, root):
            problems.append("%s resolves outside this tree: %s" % (module, origin))
    return problems, origins


def base_env(root=ROOT, inherited=None):
    """The inherited environment without anything that could change a cell, plus PYTHONPATH."""
    env = dict(os.environ if inherited is None else inherited)
    for key in list(env):
        if key in SCRUBBED_KEYS or key.startswith(SCRUBBED_PREFIXES):
            env.pop(key)
    env["PYTHONPATH"] = str(root)
    env["PYTHONNOUSERSITE"] = "1"
    return env


def cell_env(env, protocol, *, target_model, density, seed, out_dir, quota, view=None,
             recorder=False, run_id="", arm="", ledger=True, digest_path=None, obs_dump_path=None,
             search=None, on_violation=None, packets=False):
    """Variables for one launcher invocation (one cell). ``view`` None leaves the task untouched.

    ``on_violation`` "record" keeps a cell running after a motion-contract violation (canary only,
    so the moving-model rule can reject a model); unset, the recorder's default "stop" applies.
    """
    ev = protocol["evaluation"]
    out = dict(env)
    out.update(ARMS=target_model, DENSITIES=str(int(density)), SEEDS=str(int(seed)),
               GAMES=str(int(quota) * int(ev["num_envs"])), NUM_ENVS=str(ev["num_envs"]),
               OUT_ROOT=str(out_dir))
    if ledger:
        out.update(NAVRL_EPISODE_LEDGER="1", NAVRL_EPISODE_QUOTA_PER_ENV=str(int(quota)),
                   NAVRL_PLAYER_GAMES_CAP=str(ev["player_games_cap"]))
    if view is not None:
        out.update(NAVRL_KROC2027_INITIAL_VIEW=view,
                   NAVRL_KROC2027_INITIAL_DISTANCE_M=str(protocol["initial_view"]["distance_m"]))
    if recorder:
        out.update(NAVRL_KROC2027_RECORDER="1", NAVRL_KROC2027_RUN_ID=run_id,
                   NAVRL_KROC2027_ARM=arm,
                   NAVRL_KROC2027_PROTOCOL_SHA256=kp.sha256_file(PROTOCOL))
    if search is not None:
        out["NAVRL_KROC2027_SEARCH"] = search
    if on_violation is not None:
        out["NAVRL_KROC2027_ON_VIOLATION"] = on_violation
    if packets:
        out["NAVRL_KROC2027_PACKETS"] = "1"
    if digest_path is not None:
        out.update(NAVRL_TRAJECTORY_DIGEST="1", NAVRL_TRAJECTORY_DIGEST_LABEL="kroc2027-canary",
                   NAVRL_TRAJECTORY_DIGEST_JSON=str(digest_path))
    if obs_dump_path is not None:
        out.update(NAVRL_OBS_DUMP=str(obs_dump_path), NAVRL_OBS_DUMP_STRIDE=str(OBS_DUMP_STRIDE),
                   NAVRL_OBS_DUMP_MAX=str(OBS_DUMP_MAX))
    return out


BUDGET_EXIT = 124


def run_launcher(protocol, env, timeout_s=None) -> int:
    """One launcher invocation; killed with its whole process group if it outlives ``timeout_s``."""
    import signal

    proc = subprocess.Popen(["bash", str(launcher_path(protocol))], env=env, start_new_session=True)
    try:
        return proc.wait(timeout=timeout_s)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGTERM)
        try:
            proc.wait(timeout=60)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
            proc.wait()
        return BUDGET_EXIT


class GpuBudget:
    """Wall-clock cap for one invocation: refuse new cells and kill a running one at the cap."""

    def __init__(self, max_hours):
        self.max_s = None if max_hours is None else float(max_hours) * 3600.0
        self.started = time.time()

    def remaining_s(self):
        return None if self.max_s is None else self.max_s - (time.time() - self.started)

    def exhausted(self):
        left = self.remaining_s()
        return left is not None and left <= 0


# ---------------------------------------------------------------- preconditions
def git(*args, root=ROOT) -> str:
    return subprocess.run(["git", "-C", str(root)] + list(args), capture_output=True, text=True,
                          check=True).stdout


def gpu_names():
    try:
        done = subprocess.run(["nvidia-smi", "--query-gpu=name", "--format=csv,noheader"],
                              capture_output=True, text=True, timeout=30)
        return [line.strip() for line in done.stdout.strip().splitlines() if line.strip()]
    except (OSError, subprocess.TimeoutExpired):
        return []


def static_problems(protocol, root=ROOT):
    """Problems that do not depend on the stage: pinned files and the checkpoint."""
    problems = []
    pol = protocol["policy"]
    for key, rel in (("launcher_sha256", pol["launcher"]), ("player_sha256", pol["player"])):
        path = root / rel
        if not path.is_file():
            problems.append("missing %s" % rel)
        elif kp.sha256_file(path) != pol[key]:
            problems.append("%s changed (pinned %s)" % (rel, pol[key][:12]))
    checkpoint = launcher_path(protocol, root).parent / pol["checkpoint_rel_to_launcher"]
    if not checkpoint.is_file():
        problems.append("checkpoint missing at %s" % checkpoint)
    elif kp.sha256_file(checkpoint) != pol["checkpoint_sha256"]:
        problems.append("checkpoint SHA-256 mismatch at %s" % checkpoint)
    return problems


def canary_root(protocol, override=None, root=ROOT) -> Path:
    """The canary output folder: the protocol's, or a fresh one for a re-run (results/** is never
    overwritten, so a re-run needs its own folder; the main run reads the same one)."""
    return Path(override).resolve() if override else root / protocol["experiment_A"]["out_root"]["canary"]


def preconditions(stage, protocol, run_id, approved_commit, out_root, root=ROOT, gpus=None,
                  canary_dir=None):
    head = git("rev-parse", "HEAD", root=root).strip()
    problems = []
    expected_id = protocol["experiment_A"]["run_ids"][stage]
    if run_id != expected_id:
        problems.append("run id %r is not the %s run id %r" % (run_id, stage, expected_id))
    if not approved_commit or len(approved_commit) < 12 or not head.startswith(approved_commit):
        problems.append("--approved-commit must name HEAD (%s)" % head)
    if git("status", "--porcelain", "--untracked-files=no", root=root).strip():
        problems.append("tracked files are modified; commit first")
    if subprocess.run(["git", "-C", str(root), "ls-files", "--error-unmatch", PREREG],
                      capture_output=True).returncode:
        problems.append("the preregistration %s is not committed" % PREREG)
    problems.extend(static_problems(protocol, root))
    gpus = gpu_names() if gpus is None else gpus
    if gpus != [protocol["hardware"]["gpu"]]:
        problems.append("expected exactly one %s, found %s" % (protocol["hardware"]["gpu"], gpus))
    if stage == "main":
        report_path = (canary_dir or canary_root(protocol, root=root)) / "canary_report.json"
        report = json.loads(report_path.read_text()) if report_path.is_file() else {}
        if report.get("verdict") != "CANARY_PASS" or report.get("git_commit") != head:
            problems.append("main needs a CANARY_PASS on this commit (%s)" % report_path)
    if out_root.exists() and stage == "canary":
        problems.append("canary output already exists: %s" % out_root)
    return head, problems


# ---------------------------------------------------------------- per-cell checks (pure)
def comparable(result):
    """A cell result without its per-run nonce (as in tools/target_motion_replication.py)."""
    out = {k: v for k, v in result.items() if k != "condition"}
    out["condition"] = {k: v for k, v in result["condition"].items() if k != "evaluation_nonce"}
    return out


def read_jsonl(path):
    return [json.loads(line) for line in Path(path).read_text().splitlines() if line.strip()]


def ledger_problems(summary, rows, num_envs, quota):
    problems = []
    quota_rows = [r for r in rows if r.get("in_quota")]
    if summary.get("status") != "COMPLETE":
        problems.append("ledger status %s" % summary.get("status"))
    if summary.get("partition_violations", 0):
        problems.append("ledger partition violations")
    per_env = {}
    for r in quota_rows:
        per_env[r["env_index"]] = per_env.get(r["env_index"], 0) + 1
    if len(quota_rows) != num_envs * quota or len(per_env) != num_envs:
        problems.append("quota rows %d over %d envs, expected %d over %d"
                        % (len(quota_rows), len(per_env), num_envs * quota, num_envs))
    elif any(n != quota for n in per_env.values()):
        problems.append("an environment does not hold exactly %d quota episodes" % quota)
    ids = [r["episode_id"] for r in rows]
    if len(set(ids)) != len(ids):
        problems.append("duplicate episode ids")
    return problems


def recorder_problems(summary, records, ledger_rows, root=ROOT, require_motion=True):
    """The recorder covered every ledger episode, from this tree, and every gate passed."""
    problems = []
    origins = summary.get("import_origins", {})
    for module in ("aerial_gym", "motar_eval"):
        origin = origins.get(module, "")
        if not origin or not is_inside(origin, root):
            problems.append("recorder %s origin outside this tree: %r" % (module, origin))
    ledger_keys = {(r["env_index"], r["env_episode_index"]) for r in ledger_rows}
    record_keys = {(r["env_index"], r["env_episode_index"]) for r in records}
    missing = ledger_keys - record_keys
    if missing:
        problems.append("%d ledger episodes have no recorder record" % len(missing))
    outcome = {(r["env_index"], r["env_episode_index"]): r["outcome"] for r in ledger_rows}
    disagree = sum(1 for r in records
                   if (r["env_index"], r["env_episode_index"]) in outcome
                   and r.get("outcome") != outcome[(r["env_index"], r["env_episode_index"])])
    if disagree:
        problems.append("%d recorder outcomes disagree with the ledger" % disagree)
    # The recorder's acquisition chronology must agree, episode by episode, with the task's own
    # first-acquisition telemetry wherever the task recorded it.
    for ours, theirs in (("first_obs_fused_valid", "task_first_fused_obs"),
                         ("first_obs_camera_valid", "task_first_camera_obs")):
        mismatched = sum(1 for r in records if theirs in r and r.get(ours) != r[theirs])
        if mismatched:
            problems.append("%d episodes: %s differs from %s" % (mismatched, ours, theirs))
    gates = summary.get("motion_gate", {})
    if require_motion and gates.get("verdict") != "PASS":
        problems.append("motion gate %s" % gates.get("verdict"))
    # Swept clearance and the arena wall belong to the motion contract (preregistration section 5),
    # judged with the motion gate; in the canary they choose the moving model, not instrumentation.
    clearance = summary.get("swept_clearance", {})
    if require_motion and (clearance.get("violations", 1) or clearance.get("wall_violations", 1)):
        problems.append("swept clearance or wall violations: %s" % clearance)
    initial = summary.get("initial_view", {})
    if initial.get("failed_closed", 0):
        problems.append("initial-view sampler failed closed %d times" % initial["failed_closed"])
    return problems


def project_a_hours(canary_seconds, densities, quota_canary, quota_main, seeds):
    """Sum over main cells of canary seconds per quota episode scaled to the main quota.

    ``canary_seconds[(arm, density)]`` is one canary cell's duration; startup cost is scaled too,
    which over-estimates, the safe direction.
    """
    total = 0.0
    for (arm, density), seconds in canary_seconds.items():
        if density in densities:
            total += seconds * (quota_main / quota_canary) * len(seeds)
    return total / 3600.0


def scale_decision(projected_a_h, canary_used_h, budget):
    available = (budget["total"] - canary_used_h - budget["B_main_static_two_densities_stressed"]
                 - budget["B_smoke"] - 2.0)
    return {"projected_A_hours": projected_a_h, "available_for_A_hours": available,
            "densities": [70, 115, 160, 205] if projected_a_h <= available else [115, 205]}


# ---------------------------------------------------------------- commands
def cmd_pilot(args):
    raw = Path(args.raw_dir) if args.raw_dir else REPLICATION_DIR
    record = kp.pilot_from_replication(REPLICATION_DIR, raw)
    text = json.dumps(record, indent=1, sort_keys=True) + "\n"
    if args.check:
        current = PILOT_OUT.read_text() if PILOT_OUT.is_file() else ""
        print(json.dumps({"pilot_matches": current == text}, indent=1))
        return 0 if current == text else 1
    PILOT_OUT.write_text(text)
    print(text)
    return 0


def cmd_plan(args):
    protocol = kp.load_protocol(PROTOCOL)
    moving = args.moving if args.moving != "none" else None
    cells = kp.a_cells(protocol, args.stage, moving)
    print(json.dumps({"stage": args.stage, "moving_model": moving, "cells": len(cells),
                      "episodes": len(cells) * protocol["evaluation"]["episodes_per_cell"],
                      "gpu_budget_hours": protocol["gpu_budget_hours"],
                      "order": [c["cell_dir"] for c in cells]}, indent=1))
    return 0


def cmd_validate(args):
    protocol = kp.load_protocol(PROTOCOL)
    from motar_eval.contracts import GateTolerances, MotionLimits

    report = {"protocol_sha256": kp.sha256_file(PROTOCOL), "checks": {}, "problems": []}
    tol = GateTolerances(**protocol["motion_gate"]["tolerances"])
    limits = MotionLimits(**protocol["motion_gate"]["limits"])
    report["checks"]["motion_gate_defaults_match_protocol"] = (
        tol == GateTolerances() and limits == MotionLimits())
    derived = protocol["motion_gate"]["derived_g2"]
    report["checks"]["derived_g2_match"] = (
        math.isclose(derived["ulp_m"], tol.ulp_m(), rel_tol=0, abs_tol=0)
        and math.isclose(derived["accel_mps2"], tol.g2_accel_mps2(limits.dt_s), rel_tol=1e-4)
        and math.isclose(derived["speed_mps"], tol.g2_speed_mps(limits.dt_s), rel_tol=1e-3)
        and math.isclose(derived["position_m"], tol.g2_position_m(), rel_tol=1e-3))
    pinned = static_problems(protocol)
    report["checks"]["pinned_launcher_player_checkpoint"] = not pinned
    helpers = protocol["analysis_A"]
    report["checks"]["analysis_helpers_unchanged"] = (
        kp.sha256_file(ROOT / helpers["helpers"]) == helpers["helpers_sha256"])
    report["problems"].extend(pinned)
    env = base_env()
    problems, origins = import_problems(env, launcher_path(protocol).parent)
    report["import_origins"] = origins
    report["checks"]["import_origins_inside_tree"] = not problems
    report["problems"].extend(problems)
    audit = subprocess.run([env.get("PYTHON", sys.executable), "-B", str(E2_AUDIT)], cwd=str(ROOT),
                           env=dict(env, CUDA_VISIBLE_DEVICES=""), capture_output=True, text=True)
    report["checks"]["e2_executor_cpu_audit"] = audit.returncode == 0
    if args.raw_dir or (REPLICATION_DIR / "E0_static__70bars__seed4104").is_dir():
        raw = Path(args.raw_dir) if args.raw_dir else REPLICATION_DIR
        record = kp.pilot_from_replication(REPLICATION_DIR, raw)
        text = json.dumps(record, indent=1, sort_keys=True) + "\n"
        report["checks"]["pilot_reproduces"] = PILOT_OUT.is_file() and PILOT_OUT.read_text() == text
    prereg_tracked = subprocess.run(["git", "-C", str(ROOT), "ls-files", "--error-unmatch", PREREG],
                                    capture_output=True).returncode == 0
    report["preregistration_committed"] = prereg_tracked
    report["verdict"] = "PASS" if all(report["checks"].values()) else "FAIL"
    print(json.dumps(report, indent=1, sort_keys=True))
    return 0 if report["verdict"] == "PASS" else 1


def _timed_launch(protocol, env, dry_run, log, budget=None):
    if dry_run:
        log.append({k: env[k] for k in sorted(env) if k.startswith(("NAVRL_KROC2027_", "NAVRL_EPISODE",
                                                                    "ARMS", "DENSITIES", "SEEDS",
                                                                    "GAMES", "OUT_ROOT", "PYTHONPATH",
                                                                    "NAVRL_TRAJECTORY", "NAVRL_OBS_DUMP"))})
        return 0, 0.0
    started = time.time()
    rc = run_launcher(protocol, env, None if budget is None else budget.remaining_s())
    return rc, time.time() - started


def canary_subruns(protocol, out_root, run_id):
    """Every canary invocation: (name, keyword arguments for cell_env)."""
    exp = protocol["experiment_A"]
    seed = exp["canary"]["seed"]
    q = exp["canary"]["quota_per_env"]
    static = protocol["target_models"]["static"]
    runs = []
    model, density, rep_seed = OFF_PATH_CELL
    runs.append(("off_path", dict(target_model=model, density=density, seed=rep_seed,
                                  out_dir=out_root / "off_path",
                                  quota=protocol["evaluation"]["quota_per_env"])))
    for label, recorder in (("recorder_off", False), ("recorder_on", True)):
        directory = out_root / label
        runs.append((label, dict(target_model=static, density=70, seed=seed, out_dir=directory,
                                 quota=q, view="visible", recorder=recorder, run_id=run_id, arm="A1",
                                 digest_path=directory / ("%s__70bars__seed%d" % (static, seed))
                                 / "trajectory_digest.json",
                                 obs_dump_path=directory / "obs_dump.npz",
                                 on_violation="record" if recorder else None)))
    # Preregistration section 5: in the canary a violation is recorded and the cell continues.
    for arm, spec in sorted(exp["arms"].items()):
        models = [static] if spec["target"] == "static" else protocol["target_models"]["moving_candidates"]
        for model in models:
            for density in exp["densities"]:
                runs.append(("%s_%s_%d" % (arm, model, density),
                             dict(target_model=model, density=density, seed=seed,
                                  out_dir=out_root / "cells" / arm, quota=q, view=spec["view"],
                                  recorder=True, run_id=run_id, arm=arm, on_violation="record")))
    # B1 integrity: supervisor on, hidden static arm, two densities. Invariants only; outcomes unused.
    for density in (115, 205):
        runs.append(("B1_%s_%d" % (static, density),
                     dict(target_model=static, density=density, seed=seed, out_dir=out_root / "b1",
                          quota=q, view="hidden", recorder=True, run_id=run_id, arm="B1",
                          search="b1", on_violation="record")))
    return runs


def b1_problems(summary):
    """B1 integrity from a recorder summary: enabled, searched, never moved or over-turned."""
    search = summary.get("search", {})
    problems = []
    if not search.get("enabled"):
        problems.append("supervisor not enabled")
    if search.get("episodes_with_search", 0) == 0:
        problems.append("no episode ever searched")
    if search.get("invariant_violations", 1):
        problems.append("%s episodes moved horizontally or turned faster than the scan rate while searching"
                        % search.get("invariant_violations"))
    return problems


def cmd_run(args):
    protocol = kp.load_protocol(PROTOCOL)
    if args.stage == "b_main":
        return cmd_run_b(args, protocol, base_env())
    if args.stage == "b_smoke":
        return cmd_run_b_smoke(args, protocol, base_env())
    if args.stage == "packets":
        return cmd_run_packets(args, protocol, base_env())
    if args.stage == "mt_canary":
        return cmd_run_mt_canary(args, protocol, base_env())
    if args.stage == "mt_main":
        return cmd_run_mt_main(args, protocol, base_env())
    if args.stage == "occlusion_pilot":
        return cmd_run_occlusion_pilot(args, protocol, base_env())
    exp = protocol["experiment_A"]
    canary_dir = canary_root(protocol, args.canary_root)
    out_root = canary_dir if args.stage == "canary" else ROOT / exp["out_root"][args.stage]
    head, problems = preconditions(args.stage, protocol, args.run_id, args.approved_commit, out_root,
                                   canary_dir=canary_dir)
    env = base_env()
    import_issues, origins = import_problems(env, launcher_path(protocol).parent)
    problems.extend(import_issues)
    if problems and not args.dry_run:
        print(json.dumps({"stage": args.stage, "status": "REFUSED", "problems": problems,
                          "import_origins": origins}, indent=1))
        return 2
    launched = []
    if args.stage == "canary":
        runs = canary_subruns(protocol, out_root, args.run_id)
        if args.dry_run:
            for name, kw in runs:
                _timed_launch(protocol, cell_env(env, protocol, **kw), True, launched)
            print(json.dumps({"stage": "canary", "status": "DRY_RUN", "problems": problems,
                              "import_origins": origins, "subruns": [n for n, _ in runs],
                              "env": launched}, indent=1))
            return 0
        out_root.mkdir(parents=True, exist_ok=False)
        (out_root / "preflight.json").write_text(json.dumps({
            "run_id": args.run_id, "git_commit": head, "authorised_by_user": args.approved_commit,
            "protocol_sha256": kp.sha256_file(PROTOCOL), "import_origins": origins,
            "started_at": datetime.now(timezone.utc).isoformat()}, indent=1, sort_keys=True) + "\n")
        timings, codes = {}, {}
        for name, kw in runs:
            rc, seconds = _timed_launch(protocol, cell_env(env, protocol, **kw), False, launched)
            codes[name], timings[name] = rc, round(seconds, 1)
        report = canary_report(protocol, out_root, runs, codes, timings, head)
        (out_root / "canary_report.json").write_text(json.dumps(report, indent=1, sort_keys=True) + "\n")
        print(json.dumps({k: report[k] for k in ("verdict", "moving_model", "scale")}, indent=1))
        return 0 if report["verdict"] == "CANARY_PASS" else 1
    # main
    canary = json.loads((canary_dir / "canary_report.json").read_text()) \
        if (canary_dir / "canary_report.json").is_file() else {}
    moving = canary.get("moving_model")
    densities = canary.get("scale", {}).get("densities", exp["densities"])
    cells = [c for c in kp.a_cells(protocol, "main", moving) if c["density"] in densities]
    if args.dry_run:
        print(json.dumps({"stage": "main", "status": "DRY_RUN", "problems": problems,
                          "moving_model": moving, "densities": densities, "cells": len(cells),
                          "import_origins": origins}, indent=1))
        return 0
    quota = protocol["evaluation"]["quota_per_env"]
    budget = GpuBudget(args.max_gpu_hours)
    for cell in cells:
        head_now, again = preconditions("main", protocol, args.run_id, args.approved_commit, out_root,
                                        canary_dir=canary_dir)
        if again or head_now != head:
            print(json.dumps({"status": "STOPPED", "before": cell["cell_dir"], "problems": again}))
            return 2
        directory = out_root / cell["cell_dir"]
        expect = cell_expect(cell, args.run_id, head, quota)
        if (directory / RECORDER_SUMMARY).is_file() and (directory / "episode_ledger_summary.json").is_file():
            # A completed cell is skipped only if its raw files and run contract still verify.
            skip_issues = check_cell(directory, protocol, quota, require_motion=True, expect=expect)
            if skip_issues:
                print(json.dumps({"status": "STOPPED", "completed_cell_fails": cell["cell_dir"],
                                  "problems": skip_issues}))
                return 1
            continue
        if (directory / "result.json").exists() or directory.exists():
            print(json.dumps({"status": "STOPPED", "partial_cell": cell["cell_dir"]}))
            return 1
        if budget.exhausted():
            print(json.dumps({"status": "STOPPED_BUDGET", "before": cell["cell_dir"]}))
            return 3
        kw = dict(target_model=cell["target_model"], density=cell["density"], seed=cell["seed"],
                  out_dir=out_root / cell["arm"], quota=quota, view=cell["view"], recorder=True,
                  run_id=args.run_id, arm=cell["arm"])
        rc, _ = _timed_launch(protocol, cell_env(env, protocol, **kw), False, launched, budget)
        if rc == BUDGET_EXIT:
            print(json.dumps({"status": "STOPPED_BUDGET", "killed": cell["cell_dir"]}))
            return 3
        cell_issues = [] if rc == 0 else ["launcher exit %d" % rc]
        cell_issues += check_cell(directory, protocol, quota, require_motion=True, expect=expect)
        if cell_issues:
            print(json.dumps({"status": "STOPPED", "failed": cell["cell_dir"], "problems": cell_issues}))
            return 1
    return 0


def raw_hash_problems(directory, receipt, summary, rec_summary):
    """Recompute every raw file's SHA-256 and compare it with the record that binds it."""
    problems = []
    for label, path, recorded in (
            ("result.json", directory / "result.json", receipt.get("result_sha256")),
            ("ledger rows", directory / summary.get("rows_file", "episodes.jsonl"), summary.get("rows_sha256")),
            ("recorder episodes", directory / RECORDER_EPISODES, rec_summary.get("episodes_sha256"))):
        if not recorded:
            problems.append("%s has no recorded SHA-256" % label)
        elif not path.is_file() or kp.sha256_file(path) != recorded:
            problems.append("%s SHA-256 does not match its record" % label)
    return problems


def cell_expect(cell, run_id, commit, quota):
    """What a cell's receipt must say it ran: the contract the raw files are checked against."""
    return {"commit": commit, "target_model": cell["target_model"], "density": cell["density"],
            "seed": cell["seed"], "view": cell["view"], "search": cell.get("search"),
            "arm": cell["arm"], "run_id": run_id, "quota": quota,
            "protocol_sha256": kp.sha256_file(PROTOCOL)}


def contract_problems(receipt, rec_summary, expect):
    problems = []
    env = receipt.get("env_snapshot_navrl", {})
    checks = (
        ("git_commit", receipt.get("git_commit"), expect["commit"]),
        ("launcher arm", receipt.get("arm"), expect["target_model"]),
        ("density", receipt.get("density_bars"), expect["density"]),
        ("seed", receipt.get("seed"), expect["seed"]),
        ("initial view", env.get("NAVRL_KROC2027_INITIAL_VIEW"), expect["view"]),
        ("search", env.get("NAVRL_KROC2027_SEARCH"), expect["search"]),
        ("arm label", env.get("NAVRL_KROC2027_ARM"), expect["arm"]),
        ("run id", env.get("NAVRL_KROC2027_RUN_ID"), expect["run_id"]),
        ("quota", env.get("NAVRL_EPISODE_QUOTA_PER_ENV"), str(expect["quota"])),
        ("protocol", env.get("NAVRL_KROC2027_PROTOCOL_SHA256"), expect["protocol_sha256"]),
        ("recorder run id", rec_summary.get("context", {}).get("run_id"), expect["run_id"]),
    )
    for label, got, want in checks:
        if got != want:
            problems.append("%s is %r, expected %r" % (label, got, want))
    return problems


def check_cell(directory, protocol, quota, require_motion=True, expect=None, root=ROOT):
    """Ledger, recorder, receipt, raw SHA-256 and (with ``expect``) the cell's run contract.

    ``root`` is the tree the cell ran from (its recorder import origins must lie inside it)."""
    issues = []
    try:
        summary = json.loads((directory / "episode_ledger_summary.json").read_text())
        rows = read_jsonl(directory / summary.get("rows_file", "episodes.jsonl"))
        issues += ledger_problems(summary, rows, protocol["evaluation"]["num_envs"], quota)
        rec_summary = json.loads((directory / RECORDER_SUMMARY).read_text())
        records = read_jsonl(directory / RECORDER_EPISODES)
        issues += recorder_problems(rec_summary, records, rows, root=root, require_motion=require_motion)
        receipt = json.loads((directory / "receipt.json").read_text())
        if receipt.get("status") != "OK":
            issues.append("receipt status %s" % receipt.get("status"))
        if receipt.get("checkpoint_sha256") != protocol["policy"]["checkpoint_sha256"]:
            issues.append("receipt checkpoint mismatch")
        issues += raw_hash_problems(directory, receipt, summary, rec_summary)
        if expect is not None:
            issues += contract_problems(receipt, rec_summary, expect)
    except (OSError, ValueError, KeyError) as exc:
        issues.append("%s: %s" % (type(exc).__name__, exc))
    return issues


def canary_report(protocol, out_root, runs, codes, timings, head):
    """Integrity verdict, moving-model choice and the timing-only scale decision."""
    exp = protocol["experiment_A"]
    q = exp["canary"]["quota_per_env"]
    checks, detail = {}, {}
    try:
        checks["return_codes_zero"] = all(rc == 0 for rc in codes.values())
        model, density, seed = OFF_PATH_CELL
        cid = kp.cell_label(model, density, seed)
        off = json.loads((out_root / "off_path" / cid / "episode_ledger_summary.json").read_text())
        import csv
        with open(REPLICATION_DIR / "cells.csv", newline="") as stream:
            reference = {r["cell_id"]: r for r in csv.DictReader(stream)}[cid]
        checks["off_path_reproduces_replication_cell"] = off.get("rows_sha256") == reference["rows_sha256"]
        static = protocol["target_models"]["static"]
        cid_c = kp.cell_label(static, 70, exp["canary"]["seed"])
        r_off = json.loads((out_root / "recorder_off" / cid_c / "result.json").read_text())
        r_on = json.loads((out_root / "recorder_on" / cid_c / "result.json").read_text())
        d_off = json.loads((out_root / "recorder_off" / cid_c / "trajectory_digest.json").read_text())
        d_on = json.loads((out_root / "recorder_on" / cid_c / "trajectory_digest.json").read_text())
        s_off = json.loads((out_root / "recorder_off" / cid_c / "episode_ledger_summary.json").read_text())
        s_on = json.loads((out_root / "recorder_on" / cid_c / "episode_ledger_summary.json").read_text())
        checks["recorder_noninterference_result"] = comparable(r_off) == comparable(r_on)
        checks["recorder_noninterference_digest"] = (
            (d_off["sha256"], d_off["steps"]) == (d_on["sha256"], d_on["steps"]) and d_off["steps"] > 0)
        checks["recorder_noninterference_ledger"] = s_off.get("rows_sha256") == s_on.get("rows_sha256")
        try:
            import numpy as np
            sys.path.insert(0, str(ROOT / "tools"))
            import target_motion_replication as tmr  # noqa: E402  (unmodified helper)
            obs_ok, n_common = tmr.observation_gate(np.load(out_root / "recorder_off" / "obs_dump.npz"),
                                                    np.load(out_root / "recorder_on" / "obs_dump.npz"))
        except (OSError, ValueError, KeyError) as exc:
            obs_ok, n_common = False, 0
            detail["obs_dump_error"] = str(exc)
        checks["recorder_noninterference_actor_observation"] = obs_ok
        detail["observation_rows_compared"] = n_common
        b1 = {}
        for name, kw in runs:
            if not name.startswith("B1_"):
                continue
            directory = kw["out_dir"] / kp.cell_label(kw["target_model"], kw["density"], kw["seed"])
            issues = check_cell(directory, protocol, q, require_motion=False)
            summary = json.loads((directory / RECORDER_SUMMARY).read_text()) \
                if (directory / RECORDER_SUMMARY).is_file() else {}
            issues += b1_problems(summary)
            b1[name] = issues
        detail["b1_integrity"] = b1
        detail["b1_ready"] = bool(b1) and not any(b1.values())
        gate_results, cell_issues = {}, {}
        for name, kw in runs:
            if not name.startswith("A"):
                continue
            directory = kw["out_dir"] / kp.cell_label(kw["target_model"], kw["density"], kw["seed"])
            issues = check_cell(directory, protocol, q, require_motion=False)
            summary = json.loads((directory / RECORDER_SUMMARY).read_text()) \
                if (directory / RECORDER_SUMMARY).is_file() else {}
            gate_results[name] = {
                "motion_gate": summary.get("motion_gate", {}).get("verdict"),
                "swept_clearance": summary.get("swept_clearance", {}),
                "initial_view": summary.get("initial_view", {}),
            }
            if issues:
                cell_issues[name] = issues
        checks["instrumentation_all_cells"] = not cell_issues
        detail["cell_issues"] = cell_issues
        static_ok = all(v["motion_gate"] == "PASS" for k, v in gate_results.items()
                        if "_%s_" % static in k)
        checks["static_target_gates_pass"] = static_ok
        moving = None
        model_verdicts = {}
        for model in protocol["target_models"]["moving_candidates"]:
            mine = [v for k, v in gate_results.items() if "_%s_" % model in k]
            ok = bool(mine) and all(v["motion_gate"] == "PASS"
                                    and not v["swept_clearance"].get("violations", 1)
                                    and not v["swept_clearance"].get("wall_violations", 1)
                                    for v in mine)
            model_verdicts[model] = "PASS" if ok else "FAIL"
            if ok and moving is None:
                moving = model
        detail["gates"] = gate_results
        detail["moving_model_verdicts"] = model_verdicts
        seconds = {}
        for name, kw in runs:
            if name.startswith("A") and kw["target_model"] in (static, moving):
                seconds[(name.split("_")[0], kw["density"])] = timings.get(name, 0.0)
        used = sum(timings.values()) / 3600.0
        projected = project_a_hours(seconds, exp["densities"], q,
                                    protocol["evaluation"]["quota_per_env"], exp["seeds"])
        scale = scale_decision(projected, used, protocol["gpu_budget_hours"])
    except Exception as exc:  # an unreadable artifact is a failure, preserved as found
        detail["error"] = "%s: %s" % (type(exc).__name__, exc)
        moving, scale = None, {}
    verdict = "CANARY_PASS" if checks and all(checks.values()) else "CANARY_FAIL"
    return {"purpose": "integrity, motion-model choice and timing only; outcomes are not used",
            "git_commit": head, "verdict": verdict, "checks": checks, "detail": detail,
            "moving_model": moving, "scale": scale, "timings_s": timings, "return_codes": codes,
            "finished_at": datetime.now(timezone.utc).isoformat()}


def cmd_analyse(args):
    protocol = kp.load_protocol(PROTOCOL)
    helpers = protocol["analysis_A"]
    if kp.sha256_file(ROOT / helpers["helpers"]) != helpers["helpers_sha256"]:
        print(json.dumps({"status": "REFUSED", "problems": ["%s changed" % helpers["helpers"]]}))
        return 2
    exp = protocol["experiment_A"]
    out_root = ROOT / exp["out_root"]["main"]
    canary = json.loads((canary_root(protocol, args.canary_root) / "canary_report.json").read_text())
    moving = canary.get("moving_model")
    densities = canary.get("scale", {}).get("densities", exp["densities"])
    quota = protocol["evaluation"]["quota_per_env"]
    per_cell, problems = {}, []
    for cell in kp.a_cells(protocol, "main", moving):
        if cell["density"] not in densities:
            continue
        directory = out_root / cell["cell_dir"]
        problems += ["%s: %s" % (cell["cell_dir"], p) for p in check_cell(directory, protocol, quota)]
        summary = json.loads((directory / "episode_ledger_summary.json").read_text())
        rows = read_jsonl(directory / summary.get("rows_file", "episodes.jsonl"))
        quota_rows = kp.quota_rows(rows, protocol["evaluation"]["num_envs"], quota)
        joined = kp.join_episode_records(quota_rows, read_jsonl(directory / RECORDER_EPISODES))
        per_cell[(cell["arm"], cell["density"], cell["seed"])] = kp.cell_metrics(joined)
    arms = sorted({k[0] for k in per_cell})
    seeds = exp["seeds"]
    capture = kp.seed_level(per_cell, arms, densities, seeds, "capture")
    report = {"protocol_sha256": kp.sha256_file(PROTOCOL), "moving_model": moving,
              "densities": densities, "integrity_problems": problems}
    if moving is not None:
        report["confirmatory"] = kp.analyse_a(capture, seeds, protocol["analysis_A"], not problems)
    else:
        # Amendment 2026-10-15: no moving model passed, so H-A is tested on the static pair alone.
        report["confirmatory"] = kp.analyse_a_static(capture, seeds, protocol["analysis_A"], not problems)
    report["secondary_seed_level"] = {
        metric: {"%s/%d" % k: v for k, v in kp.seed_level(per_cell, arms, densities, seeds, metric).items()}
        for metric in ("crash", "timeout", "ever_acquired", "never_acquired",
                       "never_acquired_timeout", "never_acquired_crash", "never_acquired_failure")}
    naf = kp.seed_level(per_cell, arms, densities, seeds, "never_acquired_failure")
    selected = kp.select_b_arm(naf, seeds)
    report["experiment_B_arm"] = selected
    visible_arm = {"A2": "A1", "A4": "A3"}[selected["arm"]]
    ever = kp.seed_level(per_cell, arms, densities, seeds, "ever_acquired")
    crash = kp.seed_level(per_cell, arms, densities, seeds, "crash")
    timeout = kp.seed_level(per_cell, arms, densities, seeds, "timeout")
    failures = sum(crash[(selected["arm"], s)] + timeout[(selected["arm"], s)] for s in seeds)
    report["experiment_B_inputs"] = {
        "selected_arm": selected,
        "target_model": protocol["target_models"]["static"] if selected["target"] == "static" else moving,
        "capture_visible_minus_hidden": [capture[(visible_arm, s)] - capture[(selected["arm"], s)]
                                         for s in seeds],
        "ever_acquired_visible_minus_hidden": [ever[(visible_arm, s)] - ever[(selected["arm"], s)]
                                               for s in seeds],
        "never_acquired_failure_share": (sum(naf[(selected["arm"], s)] for s in seeds) / failures
                                         if failures else 0.0),
        "integrity_ok": not problems,
    }
    (out_root / "analysis.json").write_text(json.dumps(report, indent=1, sort_keys=True) + "\n")
    print(json.dumps({"integrity_problems": len(problems), "B_arm": report["experiment_B_arm"]}, indent=1))
    return 0 if not problems else 1


B_PREREG = "docs/plans/prereg_kroc2027_B_v1.md"
B_SMOKE_SEED, B_SMOKE_QUOTA = 4300, 4


def cmd_b_gate(args):
    """Evaluate the Experiment B gate once, from the A analysis and the operator's inputs."""
    protocol = kp.load_protocol(PROTOCOL)
    a_out = ROOT / protocol["experiment_A"]["out_root"]["main"]
    analysis = json.loads((a_out / "analysis.json").read_text())
    inputs = analysis["experiment_B_inputs"]
    budget = protocol["gpu_budget_hours"]
    stressed_4 = budget["B_main_static_four_densities_stressed"]
    densities = ([70, 115, 160, 205]
                 if kp.b_density_count(args.remaining_gpu_hours, stressed_4, budget["B_smoke"], 2.0) == 4
                 else [115, 205])
    planned = stressed_4 if len(densities) == 4 else budget["B_main_static_two_densities_stressed"]
    prereg = subprocess.run(["git", "-C", str(ROOT), "ls-files", "--error-unmatch", B_PREREG],
                            capture_output=True).returncode == 0
    try:
        head = git("rev-parse", "HEAD").strip()
    except subprocess.CalledProcessError:
        head = ""
    approved = bool(args.b_run_approved and args.approved_commit and len(args.approved_commit) >= 12
                    and head.startswith(args.approved_commit))
    gate = kp.b_gate(inputs["integrity_ok"], inputs["selected_arm"],
                     inputs["capture_visible_minus_hidden"], inputs["ever_acquired_visible_minus_hidden"],
                     inputs["never_acquired_failure_share"], args.remaining_gpu_hours,
                     planned + budget["B_smoke"], 1.0, prereg, approved)
    gate.update(target_model=inputs["target_model"], densities=densities,
                remaining_gpu_hours=args.remaining_gpu_hours,
                a_analysis_sha256=kp.sha256_file(a_out / "analysis.json"),
                source_commit=head,
                b_preregistration=B_PREREG,
                b_preregistration_sha256=kp.sha256_file(ROOT / B_PREREG) if (ROOT / B_PREREG).is_file() else None,
                approval={"run_id": protocol["experiment_B"]["run_ids"]["main"],
                          "approved_commit": args.approved_commit or None,
                          "approved_by_user_flag": bool(args.b_run_approved)},
                evaluated_at=datetime.now(timezone.utc).isoformat())
    out = ROOT / protocol["experiment_B"]["out_root"]["main"]
    out.mkdir(parents=True, exist_ok=True)
    (out / "b_gate.json").write_text(json.dumps(gate, indent=1, sort_keys=True) + "\n")
    print(json.dumps({k: gate[k] for k in ("verdict", "criteria", "target_model", "densities")}, indent=1))
    return 0 if gate["verdict"] == "GO" else 1


def b_link_problems(protocol, gate, head, approved_commit, a_out):
    """The B gate decision must still describe this source, this A analysis and this B prereg."""
    problems = []
    if gate.get("verdict") != "GO":
        problems.append("the B gate is not GO")
    if gate.get("source_commit") != head:
        problems.append("the B gate was evaluated on %s, not HEAD %s" % (gate.get("source_commit"), head))
    approval = gate.get("approval") or {}
    if not approved_commit or not head.startswith(approved_commit) or \
            not (approval.get("approved_commit") and head.startswith(approval["approved_commit"])):
        problems.append("the run and the gate must both carry the user-approved commit (HEAD %s)" % head)
    analysis = a_out / "analysis.json"
    if not analysis.is_file() or kp.sha256_file(analysis) != gate.get("a_analysis_sha256"):
        problems.append("the A analysis changed since the gate was evaluated")
    prereg = ROOT / B_PREREG
    tracked = subprocess.run(["git", "-C", str(ROOT), "ls-files", "--error-unmatch", B_PREREG],
                             capture_output=True).returncode == 0
    if not tracked or not prereg.is_file() or kp.sha256_file(prereg) != gate.get("b_preregistration_sha256"):
        problems.append("the B preregistration is not the committed file the gate recorded")
    return problems


def cmd_run_b_smoke(args, protocol, env):
    """KROC27-B-S01: B1 only, seed 4300, 115 and 205 bars, 4 per env; task-path invariants only."""
    head = git("rev-parse", "HEAD").strip()
    out_root = ROOT / protocol["experiment_B"]["out_root"]["main"] / "smoke"
    problems = []
    if args.run_id != protocol["experiment_B"]["run_ids"]["smoke"]:
        problems.append("run id %r is not the B smoke run id" % args.run_id)
    if not args.approved_commit or len(args.approved_commit) < 12 or not head.startswith(args.approved_commit):
        problems.append("--approved-commit must name HEAD (%s)" % head)
    if git("status", "--porcelain", "--untracked-files=no").strip():
        problems.append("tracked files are modified; commit first")
    if subprocess.run(["git", "-C", str(ROOT), "ls-files", "--error-unmatch", B_PREREG],
                      capture_output=True).returncode:
        problems.append("the B preregistration is not committed")
    problems.extend(static_problems(protocol))
    if gpu_names() != [protocol["hardware"]["gpu"]]:
        problems.append("expected exactly one %s" % protocol["hardware"]["gpu"])
    if out_root.exists():
        problems.append("smoke output already exists: %s" % out_root)
    import_issues, origins = import_problems(env, launcher_path(protocol).parent)
    problems.extend(import_issues)
    cells = [{"arm": "B1", "search": "b1", "target_model": protocol["target_models"]["static"],
              "view": "hidden", "density": d, "seed": B_SMOKE_SEED,
              "cell_dir": "B1/" + kp.cell_label(protocol["target_models"]["static"], d, B_SMOKE_SEED)}
             for d in (115, 205)]
    if args.dry_run or problems:
        print(json.dumps({"stage": "b_smoke", "status": "DRY_RUN" if args.dry_run else "REFUSED",
                          "problems": problems, "cells": len(cells), "import_origins": origins}, indent=1))
        return 0 if args.dry_run else 2
    budget = GpuBudget(args.max_gpu_hours)
    report = {"run_id": args.run_id, "git_commit": head, "cells": {}}
    for cell in cells:
        kw = dict(target_model=cell["target_model"], density=cell["density"], seed=cell["seed"],
                  out_dir=out_root / "B1", quota=B_SMOKE_QUOTA, view="hidden", recorder=True,
                  run_id=args.run_id, arm="B1", search="b1", on_violation="record")
        rc, seconds = _timed_launch(protocol, cell_env(env, protocol, **kw), False, [], budget)
        directory = out_root / cell["cell_dir"]
        issues = [] if rc == 0 else ["launcher exit %d" % rc]
        issues += check_cell(directory, protocol, B_SMOKE_QUOTA, require_motion=True,
                             expect=cell_expect(cell, args.run_id, head, B_SMOKE_QUOTA))
        summary = json.loads((directory / RECORDER_SUMMARY).read_text()) \
            if (directory / RECORDER_SUMMARY).is_file() else {}
        issues += b1_problems(summary)
        report["cells"][cell["cell_dir"]] = {"issues": issues, "seconds": round(seconds, 1),
                                             "search": summary.get("search", {})}
    report["verdict"] = "SMOKE_PASS" if not any(c["issues"] for c in report["cells"].values()) else "SMOKE_FAIL"
    (out_root / "smoke_report.json").write_text(json.dumps(report, indent=1, sort_keys=True) + "\n")
    print(json.dumps({"verdict": report["verdict"]}, indent=1))
    return 0 if report["verdict"] == "SMOKE_PASS" else 1


PACKETS_RUN_ID = "KROC27-MT-P01"
PACKETS_OUT = "results/kroc2027_moving_target_packets_v1"


def packet_cells(protocol):
    """The four reproducible counterexample cells of the canary (160 bars, seed 4190, 4 per env)."""
    seed = protocol["experiment_A"]["canary"]["seed"]
    return [{"arm": arm, "search": None, "target_model": model, "view": view, "density": 160, "seed": seed,
             "cell_dir": "%s/%s" % (arm, kp.cell_label(model, 160, seed))}
            for arm, view in (("A3", "visible"), ("A4", "hidden"))
            for model in protocol["target_models"]["moving_candidates"]]


def packets_summary(directory):
    packets = read_jsonl(directory / "kroc2027_violations.jsonl") \
        if (directory / "kroc2027_violations.jsonl").is_file() else []
    n = len(packets)
    share = lambda pred: (sum(1 for p in packets if pred(p)) / n) if n else None  # noqa: E731
    return {"packets": n,
            "replay_matches_executed": share(lambda p: p["replay"]["matches_executed"]),
            "task_flag_false": share(lambda p: p["task_step_feasible_flag"] is False),
            "no_safe_first_step": share(lambda p: p["candidates_first_step_safe"] == 0),
            "footprint_reached": share(lambda p: p["swept_clearance_m"] <= 0.0),
            "waypoint_pattern": share(lambda p: p["pattern"] == 1)}


def cmd_run_packets(args, protocol, env):
    head = git("rev-parse", "HEAD").strip()
    out_root = ROOT / PACKETS_OUT
    problems = []
    if args.run_id != PACKETS_RUN_ID:
        problems.append("run id %r is not %s" % (args.run_id, PACKETS_RUN_ID))
    if not args.approved_commit or len(args.approved_commit) < 12 or not head.startswith(args.approved_commit):
        problems.append("--approved-commit must name HEAD (%s)" % head)
    if git("status", "--porcelain", "--untracked-files=no").strip():
        problems.append("tracked files are modified; commit first")
    problems.extend(static_problems(protocol))
    if gpu_names() != [protocol["hardware"]["gpu"]]:
        problems.append("expected exactly one %s" % protocol["hardware"]["gpu"])
    if out_root.exists() and any(out_root.iterdir()):
        problems.append("packet output already exists: %s" % out_root)
    import_issues, origins = import_problems(env, launcher_path(protocol).parent)
    problems.extend(import_issues)
    cells = packet_cells(protocol)
    q = protocol["experiment_A"]["canary"]["quota_per_env"]
    if args.dry_run or problems:
        print(json.dumps({"stage": "packets", "status": "DRY_RUN" if args.dry_run else "REFUSED",
                          "problems": problems, "cells": len(cells), "import_origins": origins}, indent=1))
        return 0 if args.dry_run else 2
    budget = GpuBudget(args.max_gpu_hours)
    report = {"run_id": args.run_id, "git_commit": head, "purpose": "diagnose E1/E2 collision counterexamples",
              "cells": {}}
    for cell in cells:
        kw = dict(target_model=cell["target_model"], density=160, seed=cell["seed"], out_dir=out_root / cell["arm"],
                  quota=q, view=cell["view"], recorder=True, run_id=args.run_id, arm=cell["arm"],
                  on_violation="record", packets=True)
        rc, seconds = _timed_launch(protocol, cell_env(env, protocol, **kw), False, [], budget)
        directory = out_root / cell["cell_dir"]
        issues = [] if rc == 0 else ["launcher exit %d" % rc]
        issues += check_cell(directory, protocol, q, require_motion=False,
                             expect=cell_expect(cell, args.run_id, head, q))
        report["cells"][cell["cell_dir"]] = {"issues": issues, "seconds": round(seconds, 1),
                                             "summary": packets_summary(directory)}
        if rc == BUDGET_EXIT:
            break
    (out_root / "packets_report.json").write_text(json.dumps(report, indent=1, sort_keys=True) + "\n")
    print(json.dumps({k: v["summary"] for k, v in report["cells"].items()}, indent=1))
    return 0


MT_PLAN = "docs/plans/mt_p1_precision_v1.json"
MT_PLAN_MD = "docs/plans/mt_p1_precision_v1.md"
MT_MODEL = "bounded_planar_drone_v1_rollout_direct_l2_v1"
MT_RUNNER_PRECISION = {"matmul_allow_tf32": True, "float32_matmul_precision": "high"}


def mt_canary_cells(plan):
    """MT-P2-C01: new lineage x arm x density, seed 4190, in the plan's declared order."""
    run = plan["future_run"]
    views = {"A3_visible": ("A3", "visible"), "A4_hidden": ("A4", "hidden")}
    cells = []
    for lineage in run["lineages"]:
        alias = plan["new_lineages"][lineage]["launcher_arm_alias"]
        for arm_key in run["arms"]:
            arm, view = views[arm_key]
            for density in run["densities"]:
                cells.append({"lineage": lineage, "target_model": alias, "arm": arm, "view": view,
                              "search": None, "density": int(density), "seed": int(run["seed"]),
                              "cell_dir": "%s/%s/%s" % (lineage, arm, kp.cell_label(alias, density, run["seed"]))})
    return cells


def mt_identity_problems(directory, plan):
    """The cell ran the new lineage: backend, model, full packets, stop mode and the runner's TF32."""
    problems = []
    try:
        receipt = json.loads((directory / "receipt.json").read_text())
        env = receipt.get("env_snapshot_navrl", {})
        for key, want in (("NAVRL_TARGET_DISTANCE_BACKEND", "direct_l2_v1"),
                          ("NAVRL_KROC2027_PACKET_SCHEMA", plan["future_run"]["packet_schema"]),
                          ("NAVRL_KROC2027_PACKETS", "1"),
                          ("NAVRL_KROC2027_ON_VIOLATION", plan["future_run"]["on_violation"])):
            if env.get(key) != want:
                problems.append("receipt %s is %r, expected %r" % (key, env.get(key), want))
        model = json.loads((directory / "result.json").read_text()).get("condition", {}).get("target_motion_model")
        if model != MT_MODEL:
            problems.append("result target_motion_model is %r, expected %r" % (model, MT_MODEL))
        summary = json.loads((directory / RECORDER_SUMMARY).read_text())
        context = summary.get("context", {})
        if context.get("cfg_target_distance_backend") != "direct_l2_v1":
            problems.append("recorder backend is %r" % context.get("cfg_target_distance_backend"))
        if context.get("target_motion_model_id") != MT_MODEL:
            problems.append("recorder model is %r" % context.get("target_motion_model_id"))
        precision = summary.get("matmul_precision", {})
        for key, want in MT_RUNNER_PRECISION.items():
            if precision.get(key) != want:
                problems.append("runner %s is %r, expected the unchanged %r" % (key, precision.get(key), want))
        if summary.get("violation_packets", {}).get("schema") != "full_v2":
            problems.append("recorder packets are not full_v2")
    except (OSError, ValueError, KeyError) as exc:
        problems.append("%s: %s" % (type(exc).__name__, exc))
    return problems


def mt_cell_evidence(directory):
    """What a failed cell retained: the first violation, gate verdicts and packet files."""
    out = {}
    try:
        summary = json.loads((directory / RECORDER_SUMMARY).read_text())
        out = {"reason": summary.get("reason"),
               "motion_gate": summary.get("motion_gate", {}).get("verdict"),
               "swept_clearance": {k: summary.get("swept_clearance", {}).get(k)
                                   for k in ("violations", "wall_violations", "min_clearance_m", "first_violation")},
               "violation_packets": summary.get("violation_packets")}
    except (OSError, ValueError) as exc:
        out["recorder_summary"] = "NOT_RECORDED (%s)" % type(exc).__name__
    bundles = directory / "kroc2027_motion_inputs"
    out["bundle_files"] = sorted(p.name for p in bundles.glob("*.json")) if bundles.is_dir() else []
    return out


def cmd_run_mt_canary(args, protocol, env):
    """MT-P2-C01 (docs/plans/mt_p1_precision_v1.*): 16 cells, stop at the first failure or the cap."""
    plan = json.loads((ROOT / MT_PLAN).read_text())
    run = plan["future_run"]
    head = git("rev-parse", "HEAD").strip()
    out_root = ROOT / run["output_root"]
    cap = run["hard_budget_gpu_hours"] if args.max_gpu_hours is None else args.max_gpu_hours
    problems = []
    if args.run_id != run["run_id"]:
        problems.append("run id %r is not %s" % (args.run_id, run["run_id"]))
    if not args.approved_commit or len(args.approved_commit) < 12 or not head.startswith(args.approved_commit):
        problems.append("--approved-commit must name HEAD (%s)" % head)
    if git("status", "--porcelain", "--untracked-files=no").strip():
        problems.append("tracked files are modified; commit first")
    for rel in (MT_PLAN, MT_PLAN_MD):
        if subprocess.run(["git", "-C", str(ROOT), "ls-files", "--error-unmatch", rel], capture_output=True).returncode:
            problems.append("the plan %s is not committed" % rel)
    if not 0 < cap <= run["hard_budget_gpu_hours"]:
        problems.append("--max-gpu-hours %s outside (0, %s]" % (cap, run["hard_budget_gpu_hours"]))
    problems.extend(static_problems(protocol))
    if run["checkpoint_sha256"] != protocol["policy"]["checkpoint_sha256"]:
        problems.append("plan and protocol name different checkpoints")
    if kp.sha256_file(launcher_path(protocol)) != run["launcher_sha256"]:
        problems.append("launcher differs from the plan's pinned SHA-256")
    gpus = gpu_names()
    if gpus != [protocol["hardware"]["gpu"]]:
        problems.append("expected exactly one %s, found %s" % (protocol["hardware"]["gpu"], gpus))
    if out_root.exists():
        problems.append("canary output already exists: %s" % out_root)
    import_issues, origins = import_problems(env, launcher_path(protocol).parent)
    problems.extend(import_issues)
    cells = mt_canary_cells(plan)
    q = int(run["quota_per_env"])
    if args.dry_run or problems:
        print(json.dumps({"stage": "mt_canary", "status": "DRY_RUN" if args.dry_run else "REFUSED",
                          "problems": problems, "cells": [c["cell_dir"] for c in cells],
                          "max_gpu_hours": cap, "output_root": str(out_root), "import_origins": origins}, indent=1))
        return 0 if args.dry_run else 2
    out_root.mkdir(parents=True)
    launch = {"run_id": args.run_id, "source_commit": head, "source_tree": git("rev-parse", "HEAD^{tree}").strip(),
              "tracked_tree_clean": True, "plan_sha256": kp.sha256_file(ROOT / MT_PLAN),
              "plan_md_sha256": kp.sha256_file(ROOT / MT_PLAN_MD),
              "checkpoint_sha256": run["checkpoint_sha256"], "launcher_sha256": run["launcher_sha256"],
              "gpu": gpus, "import_origins": origins, "python": sys.version, "output_root": str(out_root),
              "max_gpu_hours": cap, "cells": [c["cell_dir"] for c in cells],
              "started_at": datetime.now(timezone.utc).isoformat()}
    (out_root / "launch_receipt.json").write_text(json.dumps(launch, indent=1, sort_keys=True) + "\n")
    budget = GpuBudget(cap)
    report = {"run_id": args.run_id, "git_commit": head, "plan_sha256": launch["plan_sha256"],
              "max_gpu_hours": cap, "cells": {}, "stop_reason": None}
    report_path = out_root / "mt_canary_report.json"
    for cell in cells:
        if budget.exhausted():
            report["stop_reason"] = "budget exhausted before %s" % cell["cell_dir"]
            break
        if git("rev-parse", "HEAD").strip() != head or git("status", "--porcelain", "--untracked-files=no").strip():
            report["stop_reason"] = "source changed before %s" % cell["cell_dir"]
            break
        kw = dict(target_model=cell["target_model"], density=cell["density"], seed=cell["seed"],
                  out_dir=out_root / cell["lineage"] / cell["arm"], quota=q, view=cell["view"], recorder=True,
                  run_id=args.run_id, arm=cell["arm"], on_violation=run["on_violation"], packets=True)
        cell_vars = cell_env(env, protocol, **kw)
        cell_vars.update(NAVRL_TARGET_DISTANCE_BACKEND="direct_l2_v1",
                         NAVRL_KROC2027_PACKET_SCHEMA=run["packet_schema"])
        rc, seconds = _timed_launch(protocol, cell_vars, False, [], budget)
        directory = out_root / cell["cell_dir"]
        issues = [] if rc == 0 else ["launcher exit %d" % rc]
        issues += check_cell(directory, protocol, q, require_motion=True,
                             expect=cell_expect(cell, args.run_id, head, q))
        issues += mt_identity_problems(directory, plan)
        entry = {"lineage": cell["lineage"], "issues": issues, "seconds": round(seconds, 1),
                 "cumulative_hours": round((time.time() - budget.started) / 3600.0, 4)}
        if issues:
            entry["evidence"] = mt_cell_evidence(directory)
        report["cells"][cell["cell_dir"]] = entry
        report_path.write_text(json.dumps(report, indent=1, sort_keys=True) + "\n")
        if rc == BUDGET_EXIT:
            report["stop_reason"] = "budget cap reached during %s (cell killed, kept as partial)" % cell["cell_dir"]
            break
        if issues:
            report["stop_reason"] = "first failed cell %s" % cell["cell_dir"]
            break
    passed = len(report["cells"]) == len(cells) and all(not c["issues"] for c in report["cells"].values())
    report["verdict"] = "CANARY_PASS" if passed else (
        "BUDGET_STOP" if report["stop_reason"] and "budget" in report["stop_reason"] else "CANARY_FAIL")
    report["finished_at"] = datetime.now(timezone.utc).isoformat()
    report_path.write_text(json.dumps(report, indent=1, sort_keys=True) + "\n")
    print(json.dumps({"verdict": report["verdict"], "stop_reason": report["stop_reason"],
                      "cells_run": len(report["cells"])}, indent=1))
    return 0 if passed else 1


A_AMENDMENT2 = "docs/plans/kroc2027_A_amendment2_v1.json"
A_AMENDMENT2_MD = "docs/plans/prereg_kroc2027_A_v1_amendment2_2026-10-16.md"


def select_mt_lineage(report, plan, candidates):
    """Preregistration section 5 rule on the new lineage: the first candidate whose canary cells all pass."""
    per_lineage = len(plan["future_run"]["arms"]) * len(plan["future_run"]["densities"])
    for lineage in candidates:
        cells = {k: v for k, v in report.get("cells", {}).items() if k.startswith(lineage + "/")}
        if len(cells) == per_lineage and all(not v["issues"] for v in cells.values()):
            return lineage
    return None


def mt_main_cells(amendment, plan, lineage):
    """KROC27-A-M02: A3/A4 with the selected lineage, seed-major (an interrupted run holds whole seeds)."""
    alias = plan["new_lineages"][lineage]["launcher_arm_alias"]
    cells = []
    for seed in amendment["seeds"]:
        for density in amendment["densities"]:
            for arm, view in sorted(amendment["arms"].items()):
                cells.append({"lineage": lineage, "target_model": alias, "arm": arm, "view": view, "search": None,
                              "density": int(density), "seed": int(seed),
                              "cell_dir": "%s/%s/%s" % (lineage, arm, kp.cell_label(alias, density, seed))})
    return cells


def simulation_source_problems(canary_commit, head, paths):
    """The main run simulates with exactly the canary's source: the trees under ``paths`` are identical."""
    done = subprocess.run(["git", "-C", str(ROOT), "diff", "--quiet", canary_commit, head, "--"] + list(paths),
                          capture_output=True)
    return [] if done.returncode == 0 else ["simulation source differs from the canary commit %s in %s"
                                            % (canary_commit[:12], " ".join(paths))]


def cmd_run_mt_main(args, protocol, env):
    """KROC27-A-M02 (amendment 2): A3/A4 main cells with the canary-selected direct_l2_v1 lineage."""
    amendment = json.loads((ROOT / A_AMENDMENT2).read_text())
    plan = json.loads((ROOT / MT_PLAN).read_text())
    canary = amendment["canary"]
    head = git("rev-parse", "HEAD").strip()
    out_root = ROOT / amendment["output_root"]
    cap = amendment["max_gpu_hours"] if args.max_gpu_hours is None else args.max_gpu_hours
    problems = []
    if args.run_id != amendment["run_id"]:
        problems.append("run id %r is not %s" % (args.run_id, amendment["run_id"]))
    if not args.approved_commit or len(args.approved_commit) < 12 or not head.startswith(args.approved_commit):
        problems.append("--approved-commit must name HEAD (%s)" % head)
    if git("status", "--porcelain", "--untracked-files=no").strip():
        problems.append("tracked files are modified; commit first")
    for rel in (A_AMENDMENT2, A_AMENDMENT2_MD, MT_PLAN):
        if subprocess.run(["git", "-C", str(ROOT), "ls-files", "--error-unmatch", rel], capture_output=True).returncode:
            problems.append("%s is not committed" % rel)
    if not 0 < cap <= amendment["max_gpu_hours"]:
        problems.append("--max-gpu-hours %s outside (0, %s]" % (cap, amendment["max_gpu_hours"]))
    report_path = ROOT / canary["report"]
    report = json.loads(report_path.read_text()) if report_path.is_file() else {}
    if report.get("verdict") != canary["required_verdict"] or report.get("git_commit") != canary["source_commit"]:
        problems.append("needs %s of %s at %s" % (canary["required_verdict"], canary["source_commit"][:12], report_path))
    if report_path.is_file() and kp.sha256_file(report_path) != canary["report_sha256"]:
        problems.append("canary report differs from the SHA-256 pinned in the amendment")
    lineage = select_mt_lineage(report, plan, amendment["moving_candidates"])
    if lineage is None:
        problems.append("no direct_l2_v1 lineage passed every canary cell")
    problems.extend(simulation_source_problems(canary["source_commit"], head, canary["simulation_paths"]))
    problems.extend(static_problems(protocol))
    if kp.sha256_file(launcher_path(protocol)) != plan["future_run"]["launcher_sha256"]:
        problems.append("launcher differs from the plan's pinned SHA-256")
    gpus = gpu_names()
    if gpus != [protocol["hardware"]["gpu"]]:
        problems.append("expected exactly one %s, found %s" % (protocol["hardware"]["gpu"], gpus))
    if out_root.exists():
        problems.append("main output already exists: %s" % out_root)
    import_issues, origins = import_problems(env, launcher_path(protocol).parent)
    problems.extend(import_issues)
    cells = mt_main_cells(amendment, plan, lineage) if lineage else []
    q = int(amendment["quota_per_env"])
    if args.dry_run or problems:
        print(json.dumps({"stage": "mt_main", "status": "DRY_RUN" if args.dry_run else "REFUSED",
                          "problems": problems, "lineage": lineage, "cells": len(cells), "max_gpu_hours": cap,
                          "output_root": str(out_root), "import_origins": origins}, indent=1))
        return 0 if args.dry_run else 2
    out_root.mkdir(parents=True)
    launch = {"run_id": args.run_id, "source_commit": head, "source_tree": git("rev-parse", "HEAD^{tree}").strip(),
              "tracked_tree_clean": True, "amendment_sha256": kp.sha256_file(ROOT / A_AMENDMENT2),
              "amendment_md_sha256": kp.sha256_file(ROOT / A_AMENDMENT2_MD), "lineage": lineage,
              "canary_report_sha256": kp.sha256_file(report_path), "checkpoint_sha256": protocol["policy"]["checkpoint_sha256"],
              "launcher_sha256": plan["future_run"]["launcher_sha256"], "gpu": gpus, "import_origins": origins,
              "python": sys.version, "output_root": str(out_root), "max_gpu_hours": cap,
              "cells": [c["cell_dir"] for c in cells], "started_at": datetime.now(timezone.utc).isoformat()}
    (out_root / "launch_receipt.json").write_text(json.dumps(launch, indent=1, sort_keys=True) + "\n")
    budget = GpuBudget(cap)
    run = {"run_id": args.run_id, "git_commit": head, "lineage": lineage, "max_gpu_hours": cap, "cells": {},
           "stop_reason": None}
    run_path = out_root / "run_report.json"
    for cell in cells:
        if budget.exhausted():
            run["stop_reason"] = "budget exhausted before %s" % cell["cell_dir"]
            break
        if git("rev-parse", "HEAD").strip() != head or git("status", "--porcelain", "--untracked-files=no").strip():
            run["stop_reason"] = "source changed before %s" % cell["cell_dir"]
            break
        kw = dict(target_model=cell["target_model"], density=cell["density"], seed=cell["seed"],
                  out_dir=out_root / cell["lineage"] / cell["arm"], quota=q, view=cell["view"], recorder=True,
                  run_id=args.run_id, arm=cell["arm"], on_violation=amendment["on_violation"], packets=True)
        cell_vars = cell_env(env, protocol, **kw)
        cell_vars.update(NAVRL_TARGET_DISTANCE_BACKEND="direct_l2_v1",
                         NAVRL_KROC2027_PACKET_SCHEMA=amendment["packet_schema"])
        rc, seconds = _timed_launch(protocol, cell_vars, False, [], budget)
        directory = out_root / cell["cell_dir"]
        issues = [] if rc == 0 else ["launcher exit %d" % rc]
        issues += check_cell(directory, protocol, q, require_motion=True,
                             expect=cell_expect(cell, args.run_id, head, q))
        issues += mt_identity_problems(directory, plan)
        entry = {"issues": issues, "seconds": round(seconds, 1),
                 "cumulative_hours": round((time.time() - budget.started) / 3600.0, 4)}
        if issues:
            entry["evidence"] = mt_cell_evidence(directory)
        run["cells"][cell["cell_dir"]] = entry
        run_path.write_text(json.dumps(run, indent=1, sort_keys=True) + "\n")
        if rc == BUDGET_EXIT:
            run["stop_reason"] = "budget cap reached during %s (cell killed, kept as partial)" % cell["cell_dir"]
            break
        if issues:
            run["stop_reason"] = "first failed cell %s" % cell["cell_dir"]
            break
    passed = len(run["cells"]) == len(cells) and all(not c["issues"] for c in run["cells"].values())
    run["verdict"] = "COMPLETE" if passed else (
        "BUDGET_STOP" if run["stop_reason"] and "budget" in run["stop_reason"] else "STOPPED")
    run["finished_at"] = datetime.now(timezone.utc).isoformat()
    run_path.write_text(json.dumps(run, indent=1, sort_keys=True) + "\n")
    print(json.dumps({"verdict": run["verdict"], "stop_reason": run["stop_reason"], "cells_run": len(run["cells"])},
                     indent=1))
    return 0 if passed else 1


def manifest_problems(directory, root_dir, manifest):
    """Every file of a raw cell matches the run's RAW_MANIFEST.json (the cells are git-ignored)."""
    files = manifest["files"]
    rel = str(Path(directory).resolve().relative_to(Path(root_dir).resolve()))
    names = [k for k in files if k.startswith(rel + "/")]
    if not names:
        return ["%s is not in the manifest" % rel]
    return ["%s SHA-256 differs from the manifest" % k for k in names
            if not (Path(root_dir) / k).is_file() or kp.sha256_file(Path(root_dir) / k) != files[k]]


def cmd_analyse_a_moving(args):
    """Amendment 2: section 7 restored (Delta_A, I_A, Holm) with A1/A2 from KROC27-A-M01 and A3/A4 from M02."""
    protocol = kp.load_protocol(PROTOCOL)
    spec = protocol["analysis_A"]
    if kp.sha256_file(ROOT / spec["helpers"]) != spec["helpers_sha256"]:
        print(json.dumps({"status": "REFUSED", "problems": ["%s changed" % spec["helpers"]]}))
        return 2
    amendment = json.loads((ROOT / A_AMENDMENT2).read_text())
    plan = json.loads((ROOT / MT_PLAN).read_text())
    out_root = ROOT / amendment["output_root"]
    run = json.loads((out_root / "run_report.json").read_text())
    launch = json.loads((out_root / "launch_receipt.json").read_text())
    lineage = run["lineage"]
    static = amendment["static_arms"]
    static_root = Path(args.static_root or (ROOT / static["folder"])).resolve()
    manifest = json.loads((ROOT / static["raw_manifest"]).read_text())
    quota = int(amendment["quota_per_env"])
    seeds, densities = amendment["seeds"], amendment["densities"]
    per_cell, problems = {}, []
    if run.get("verdict") != "COMPLETE":
        problems.append("main run verdict %s" % run.get("verdict"))
    if kp.sha256_file(ROOT / static["analysis"]) != static["analysis_sha256"]:
        problems.append("%s changed" % static["analysis"])
    for seed in seeds:
        for density in densities:
            for arm in ("A1", "A2"):
                directory = static_root / arm / kp.cell_label(protocol["target_models"]["static"], density, seed)
                cell = {"arm": arm, "target_model": protocol["target_models"]["static"], "density": density,
                        "seed": seed, "view": protocol["experiment_A"]["arms"][arm]["view"], "search": None}
                receipt = json.loads((directory / "receipt.json").read_text())
                problems += ["%s/%s: %s" % (arm, directory.name, p) for p in
                             manifest_problems(directory, static_root, manifest)
                             + check_cell(directory, protocol, quota, root=static_root.parents[1],
                                          expect=cell_expect(cell, static["run_id"], receipt.get("git_commit"), quota))]
                if receipt.get("git_commit") != static["source_commit"]:
                    problems.append("%s/%s ran on %s" % (arm, directory.name, receipt.get("git_commit")))
                per_cell[(arm, density, seed)] = _quota_metrics(directory, protocol, quota)
    for cell in mt_main_cells(amendment, plan, lineage):
        directory = out_root / cell["cell_dir"]
        problems += ["%s: %s" % (cell["cell_dir"], p) for p in
                     check_cell(directory, protocol, quota, expect=cell_expect(cell, amendment["run_id"],
                                                                               launch["source_commit"], quota))
                     + mt_identity_problems(directory, plan)]
        per_cell[(cell["arm"], cell["density"], cell["seed"])] = _quota_metrics(directory, protocol, quota)
    arms = ["A1", "A2", "A3", "A4"]
    capture = kp.seed_level(per_cell, arms, densities, seeds, "capture")
    report = {"protocol_sha256": kp.sha256_file(PROTOCOL), "amendment_sha256": kp.sha256_file(ROOT / A_AMENDMENT2),
              "moving_lineage": lineage, "static_run": static["run_id"], "moving_run": amendment["run_id"],
              "moving_source_commit": launch["source_commit"], "densities": densities, "integrity_problems": problems,
              "confirmatory": kp.analyse_a(capture, seeds, spec, not problems),
              "amendment_1_static_result": "kept as recorded in %s (not re-labelled)" % static["analysis"],
              "secondary_seed_level": {
                  metric: {"%s/%d" % k: v for k, v in kp.seed_level(per_cell, arms, densities, seeds, metric).items()}
                  for metric in ("capture", "crash", "timeout", "ever_acquired", "never_acquired",
                                 "never_acquired_timeout", "never_acquired_crash", "never_acquired_failure")}}
    (out_root / "analysis.json").write_text(json.dumps(report, indent=1, sort_keys=True) + "\n")
    print(json.dumps({"integrity_problems": len(problems),
                      "decisions": {k: v.get("decision") for k, v in report["confirmatory"]["contrasts"].items()}},
                     indent=1))
    return 0 if not problems else 1


def _quota_metrics(directory, protocol, quota):
    summary = json.loads((directory / "episode_ledger_summary.json").read_text())
    rows = read_jsonl(directory / summary.get("rows_file", "episodes.jsonl"))
    joined = kp.join_episode_records(kp.quota_rows(rows, protocol["evaluation"]["num_envs"], quota),
                                     read_jsonl(directory / RECORDER_EPISODES))
    return kp.cell_metrics(joined)


OCC_PLAN = "docs/plans/occlusion_pilot_v1.json"
OCC_PLAN_MD = "docs/plans/occlusion_pilot_v1.md"
OCC_REPORT = "pilot_report.json"


def occlusion_pilot_cells(plan):
    """OCC-P1: static and moving visible-start cells, seed-major, with their historical counterparts."""
    cells = []
    for seed in plan["seeds"]:
        for density in plan["densities"]:
            for group in ("static", "moving"):
                spec = plan["groups"][group]
                label = kp.cell_label(spec["target_model"], density, seed)
                cells.append({"group": group, "lineage": spec["lineage"], "arm": spec["arm"],
                              "target_model": spec["target_model"], "view": spec["view"], "search": None,
                              "density": int(density), "seed": int(seed),
                              "cell_dir": "%s/%s/%s" % (group, spec["arm"], label),
                              "historical": str(Path(spec["historical"]["cells"]) / label)})
    return cells


def historical_match(new_dir, old_dir, quota, fields):
    """First ``quota`` episodes per env of a rerun against the historical cell; differences are listed."""
    out = {"ledger_rows_compared": 0, "ledger_mismatches": [], "recorder_records_compared": 0,
           "recorder_mismatches": [], "keys_only_new": [], "keys_only_old": []}
    try:
        def ledger(d):
            return {(r["env_index"], r["env_episode_index"]): r for r in read_jsonl(Path(d) / "episodes.jsonl")
                    if r["env_episode_index"] < quota}
        def records(d):
            return {(r["env_index"], r["env_episode_index"]): r for r in read_jsonl(Path(d) / RECORDER_EPISODES)
                    if r["env_episode_index"] < quota}
        new_l, old_l = ledger(new_dir), ledger(old_dir)
        if set(new_l) != set(old_l):
            out["ledger_mismatches"].append("episode keys differ: %d new vs %d old" % (len(new_l), len(old_l)))
        for key in sorted(set(new_l) & set(old_l)):
            out["ledger_rows_compared"] += 1
            diff = [f for f in fields if new_l[key].get(f) != old_l[key].get(f)]
            if diff and len(out["ledger_mismatches"]) < 10:
                out["ledger_mismatches"].append({"episode": list(key), "fields": diff})
        new_r, old_r = records(new_dir), records(old_dir)
        if set(new_r) != set(old_r):
            out["recorder_mismatches"].append("record keys differ: %d new vs %d old" % (len(new_r), len(old_r)))
        keys_new, keys_old = set(), set()
        for key in sorted(set(new_r) & set(old_r)):
            out["recorder_records_compared"] += 1
            a, b = new_r[key], old_r[key]
            keys_new |= set(a) - set(b)
            keys_old |= set(b) - set(a)
            diff = sorted(f for f in set(a) & set(b) if a[f] != b[f])
            if diff and len(out["recorder_mismatches"]) < 10:
                out["recorder_mismatches"].append({"episode": list(key), "fields": diff})
        out["keys_only_new"], out["keys_only_old"] = sorted(keys_new), sorted(keys_old)
        matched = not out["ledger_mismatches"] and not out["recorder_mismatches"] and out["ledger_rows_compared"] > 0
        out["verdict"] = "MATCH" if matched else "MISMATCH"
    except (OSError, ValueError, KeyError) as exc:
        out["verdict"] = "NOT_COMPARABLE"
        out["error"] = "%s: %s" % (type(exc).__name__, exc)
    return out


def occlusion_trace_problems(directory):
    from motar_eval import occlusion_trace as ot

    try:
        trace = ot.read_trace(directory)
    except (OSError, ValueError, KeyError) as exc:
        return ["occlusion trace: %s: %s" % (type(exc).__name__, exc)]
    problems = []
    if trace["summary"]["frames"] == 0:
        problems.append("occlusion trace has no frames")
    if trace["summary"]["reason"] not in ("complete", "ledger complete", "quota complete"):
        problems.append("occlusion trace closed for %r" % trace["summary"]["reason"])
    return problems


def free_disk_gb(path):
    import shutil

    return shutil.disk_usage(path).free / 1e9


def cmd_run_occlusion_pilot(args, protocol, env):
    """OCC-P1 (docs/plans/occlusion_pilot_v1.*): 16 cells, G0 comparison per cell, stop at the first failure."""
    plan = json.loads((ROOT / OCC_PLAN).read_text())
    mt_plan = json.loads((ROOT / MT_PLAN).read_text())
    head = git("rev-parse", "HEAD").strip()
    out_root = ROOT / plan["output_root"]
    cap = plan["max_gpu_hours"] if args.max_gpu_hours is None else args.max_gpu_hours
    problems = []
    if args.run_id != plan["run_id"]:
        problems.append("run id %r is not %s" % (args.run_id, plan["run_id"]))
    if not args.approved_commit or len(args.approved_commit) < 12 or not head.startswith(args.approved_commit):
        problems.append("--approved-commit must name HEAD (%s)" % head)
    if git("status", "--porcelain", "--untracked-files=no").strip():
        problems.append("tracked files are modified; commit first")
    for rel in (OCC_PLAN, OCC_PLAN_MD):
        if subprocess.run(["git", "-C", str(ROOT), "ls-files", "--error-unmatch", rel], capture_output=True).returncode:
            problems.append("the plan %s is not committed" % rel)
    if plan.get("status", "").startswith("DRAFT"):
        problems.append("the plan is still a draft")
    if plan["requires_external_backup_confirmation"] and not args.external_backup_confirmed:
        problems.append("--external-backup-confirmed is required")
    if not 0 < cap <= plan["max_gpu_hours"]:
        problems.append("--max-gpu-hours %s outside (0, %s]" % (cap, plan["max_gpu_hours"]))
    if free_disk_gb(ROOT) < plan["min_free_disk_gb"]:
        problems.append("free disk %.1f GB below %.1f GB" % (free_disk_gb(ROOT), plan["min_free_disk_gb"]))
    problems.extend(static_problems(protocol))
    gpus = gpu_names()
    if gpus != [protocol["hardware"]["gpu"]]:
        problems.append("expected exactly one %s, found %s" % (protocol["hardware"]["gpu"], gpus))
    if out_root.exists():
        problems.append("pilot output already exists: %s" % out_root)
    cells = occlusion_pilot_cells(plan)
    for group, spec in plan["groups"].items():
        hist = spec["historical"]
        manifest_path = Path(hist["raw_manifest"])
        if not manifest_path.is_file():
            problems.append("historical manifest missing: %s" % manifest_path)
            continue
        manifest = json.loads(manifest_path.read_text())
        for cell in (c for c in cells if c["group"] == group):
            problems += ["historical %s" % p for p in manifest_problems(cell["historical"], manifest_path.parent, manifest)]
    import_issues, origins = import_problems(env, launcher_path(protocol).parent)
    problems.extend(import_issues)
    q = int(plan["quota_per_env"])
    if args.dry_run or problems:
        print(json.dumps({"stage": "occlusion_pilot", "status": "DRY_RUN" if args.dry_run else "REFUSED",
                          "problems": problems, "cells": [c["cell_dir"] for c in cells], "max_gpu_hours": cap,
                          "output_root": str(out_root), "import_origins": origins}, indent=1))
        return 0 if args.dry_run else 2
    out_root.mkdir(parents=True)
    launch = {"run_id": args.run_id, "source_commit": head, "source_tree": git("rev-parse", "HEAD^{tree}").strip(),
              "plan_sha256": kp.sha256_file(ROOT / OCC_PLAN), "plan_md_sha256": kp.sha256_file(ROOT / OCC_PLAN_MD),
              "checkpoint_sha256": protocol["policy"]["checkpoint_sha256"], "gpu": gpus, "import_origins": origins,
              "free_disk_gb_at_start": round(free_disk_gb(ROOT), 2), "max_gpu_hours": cap,
              "external_backup_confirmed": bool(args.external_backup_confirmed),
              "cells": [c["cell_dir"] for c in cells], "started_at": datetime.now(timezone.utc).isoformat()}
    (out_root / "launch_receipt.json").write_text(json.dumps(launch, indent=1, sort_keys=True) + "\n")
    budget = GpuBudget(cap)
    report = {"run_id": args.run_id, "git_commit": head, "max_gpu_hours": cap, "cells": {}, "stop_reason": None}
    report_path = out_root / OCC_REPORT
    for cell in cells:
        if budget.exhausted():
            report["stop_reason"] = "budget exhausted before %s" % cell["cell_dir"]
            break
        if git("rev-parse", "HEAD").strip() != head or git("status", "--porcelain", "--untracked-files=no").strip():
            report["stop_reason"] = "source changed before %s" % cell["cell_dir"]
            break
        spec = plan["groups"][cell["group"]]
        kw = dict(target_model=cell["target_model"], density=cell["density"], seed=cell["seed"],
                  out_dir=out_root / cell["group"] / cell["arm"], quota=q, view=cell["view"], recorder=True,
                  run_id=args.run_id, arm=cell["arm"])
        cell_vars = cell_env(env, protocol, **kw)
        cell_vars.update(plan["telemetry_env"])
        cell_vars.update(spec["extra_env"])
        rc, seconds = _timed_launch(protocol, cell_vars, False, [], budget)
        directory = out_root / cell["cell_dir"]
        issues = [] if rc == 0 else ["launcher exit %d" % rc]
        issues += check_cell(directory, protocol, q, require_motion=True,
                             expect=cell_expect(cell, args.run_id, head, q))
        if cell["group"] == "moving":
            issues += mt_identity_problems(directory, mt_plan)
        issues += occlusion_trace_problems(directory)
        g0 = historical_match(directory, cell["historical"], q, plan["g0_noninterference"]["ledger_fields"])
        entry = {"issues": issues, "g0": g0, "seconds": round(seconds, 1),
                 "cumulative_hours": round((time.time() - budget.started) / 3600.0, 4)}
        report["cells"][cell["cell_dir"]] = entry
        report_path.write_text(json.dumps(report, indent=1, sort_keys=True) + "\n")
        if rc == BUDGET_EXIT:
            report["stop_reason"] = "budget cap reached during %s (cell killed, kept as partial)" % cell["cell_dir"]
            break
        if issues:
            report["stop_reason"] = "first failed cell %s" % cell["cell_dir"]
            break
        if g0["verdict"] != "MATCH":
            report["stop_reason"] = "G0 %s at %s" % (g0["verdict"], cell["cell_dir"])
            break
    done = len(report["cells"]) == len(cells) and not report["stop_reason"]
    report["verdict"] = "PILOT_COMPLETE" if done else (
        "BUDGET_STOP" if report["stop_reason"] and "budget" in report["stop_reason"] else "STOPPED")
    report["g0_noninterference"] = ("PASS" if done and all(c["g0"]["verdict"] == "MATCH"
                                                          for c in report["cells"].values()) else "NOT_PASSED")
    report["renderer_parity"] = "PENDING_ANALYSIS (tools/occlusion_analysis.py pilot; separate PASS condition)"
    report["finished_at"] = datetime.now(timezone.utc).isoformat()
    report_path.write_text(json.dumps(report, indent=1, sort_keys=True) + "\n")
    print(json.dumps({"verdict": report["verdict"], "g0": report["g0_noninterference"],
                      "stop_reason": report["stop_reason"], "cells_run": len(report["cells"])}, indent=1))
    return 0 if done else 1


def cmd_run_b(args, protocol, env):
    out_root = ROOT / protocol["experiment_B"]["out_root"]["main"]
    a_out = ROOT / protocol["experiment_A"]["out_root"]["main"]
    head = git("rev-parse", "HEAD").strip()
    gate_path = out_root / "b_gate.json"
    gate = json.loads(gate_path.read_text()) if gate_path.is_file() else {}
    quota = protocol["evaluation"]["quota_per_env"]

    def problems_now():
        found = []
        if args.run_id != protocol["experiment_B"]["run_ids"]["main"]:
            found.append("run id %r is not the B main run id" % args.run_id)
        if git("rev-parse", "HEAD").strip() != head:
            found.append("HEAD moved during the run")
        if git("status", "--porcelain", "--untracked-files=no").strip():
            found.append("tracked files are modified; commit first")
        found.extend(b_link_problems(protocol, gate, head, args.approved_commit, a_out))
        found.extend(static_problems(protocol))
        if gpu_names() != [protocol["hardware"]["gpu"]]:
            found.append("expected exactly one %s" % protocol["hardware"]["gpu"])
        return found

    problems = problems_now()
    import_issues, origins = import_problems(env, launcher_path(protocol).parent)
    problems.extend(import_issues)
    cells = kp.b_cells(protocol, gate.get("target_model") or "", gate.get("densities", []))
    if args.dry_run or problems:
        print(json.dumps({"stage": "b_main", "status": "DRY_RUN" if args.dry_run else "REFUSED",
                          "problems": problems, "cells": len(cells), "import_origins": origins}, indent=1))
        return 0 if args.dry_run else 2
    budget = GpuBudget(args.max_gpu_hours)
    launched = []
    for cell in cells:
        again = problems_now()
        if again:
            print(json.dumps({"status": "STOPPED", "before": cell["cell_dir"], "problems": again}))
            return 2
        directory = out_root / cell["cell_dir"]
        expect = cell_expect(cell, args.run_id, head, quota)
        if (directory / RECORDER_SUMMARY).is_file() and (directory / "episode_ledger_summary.json").is_file():
            skip_issues = check_cell(directory, protocol, quota, require_motion=True, expect=expect)
            if cell["search"]:
                skip_issues += b1_problems(json.loads((directory / RECORDER_SUMMARY).read_text()))
            if skip_issues:
                print(json.dumps({"status": "STOPPED", "completed_cell_fails": cell["cell_dir"],
                                  "problems": skip_issues}))
                return 1
            continue
        if directory.exists():
            print(json.dumps({"status": "STOPPED", "partial_cell": cell["cell_dir"]}))
            return 1
        if budget.exhausted():
            print(json.dumps({"status": "STOPPED_BUDGET", "before": cell["cell_dir"]}))
            return 3
        kw = dict(target_model=cell["target_model"], density=cell["density"], seed=cell["seed"],
                  out_dir=out_root / cell["arm"], quota=quota, view="hidden", recorder=True,
                  run_id=args.run_id, arm=cell["arm"], search=cell["search"])
        rc, _ = _timed_launch(protocol, cell_env(env, protocol, **kw), False, launched, budget)
        if rc == BUDGET_EXIT:
            print(json.dumps({"status": "STOPPED_BUDGET", "killed": cell["cell_dir"]}))
            return 3
        issues = [] if rc == 0 else ["launcher exit %d" % rc]
        issues += check_cell(directory, protocol, quota, require_motion=True, expect=expect)
        if cell["search"]:
            issues += b1_problems(json.loads((directory / RECORDER_SUMMARY).read_text())) \
                if (directory / RECORDER_SUMMARY).is_file() else ["no recorder summary"]
        if issues:
            print(json.dumps({"status": "STOPPED", "failed": cell["cell_dir"], "problems": issues}))
            return 1
    return 0


def cmd_manifest(args):
    """A tracked manifest for git-ignored raw results: every file's SHA-256, plus provenance."""
    root = Path(args.root).resolve()
    files = {}
    for path in sorted(p for p in root.rglob("*") if p.is_file()):
        rel = str(path.relative_to(root))
        if rel == "RAW_MANIFEST.json":
            continue
        files[rel] = kp.sha256_file(path)
    commits = sorted({json.loads(p.read_text()).get("git_commit") for p in root.rglob("receipt.json")} - {None})
    manifest = {"schema": "motar.raw-manifest.v1", "root": root.name, "files": files,
                "receipt_git_commits": commits, "file_count": len(files),
                "protocol_sha256": kp.sha256_file(PROTOCOL)}
    text = json.dumps(manifest, indent=1, sort_keys=True) + "\n"
    if args.out:
        Path(args.out).write_text(text)
    print(json.dumps({"files": len(files), "receipt_git_commits": commits}, indent=1))
    return 0


def cmd_analyse_b(args):
    """Experiment B analysis, fixed in prereg_kroc2027_B_v1.md before any B data."""
    protocol = kp.load_protocol(PROTOCOL)
    out_root = ROOT / protocol["experiment_B"]["out_root"]["main"]
    gate = json.loads((out_root / "b_gate.json").read_text())
    quota = protocol["evaluation"]["quota_per_env"]
    seeds = protocol["experiment_B"]["seeds"]
    head_receipts = set()
    per_cell, problems = {}, []
    for cell in kp.b_cells(protocol, gate["target_model"], gate["densities"]):
        directory = out_root / cell["cell_dir"]
        receipt = json.loads((directory / "receipt.json").read_text())
        head_receipts.add(receipt.get("git_commit"))
        expect = cell_expect(cell, protocol["experiment_B"]["run_ids"]["main"], receipt.get("git_commit"), quota)
        problems += ["%s: %s" % (cell["cell_dir"], p) for p in check_cell(directory, protocol, quota, expect=expect)]
        summary = json.loads((directory / "episode_ledger_summary.json").read_text())
        rows = read_jsonl(directory / summary.get("rows_file", "episodes.jsonl"))
        joined = kp.join_episode_records(kp.quota_rows(rows, protocol["evaluation"]["num_envs"], quota),
                                         read_jsonl(directory / RECORDER_EPISODES))
        per_cell[(cell["arm"], cell["density"], cell["seed"])] = kp.cell_metrics(joined)
    if len(head_receipts) != 1:
        problems.append("cells ran on several commits: %s" % sorted(head_receipts))
    level = {m: kp.seed_level(per_cell, ["B0", "B1"], gate["densities"], seeds, m)
             for m in ("capture", "crash", "timeout", "never_acquired", "never_acquired_failure")}
    report = {"preregistration_sha256": kp.sha256_file(ROOT / B_PREREG), "gate": gate,
              "integrity_problems": problems,
              "confirmatory": kp.analyse_b(level["capture"], level["crash"], level["timeout"], seeds, not problems),
              "secondary_seed_level": {m: {"%s/%d" % k: v for k, v in vals.items()} for m, vals in level.items()}}
    (out_root / "analysis.json").write_text(json.dumps(report, indent=1, sort_keys=True) + "\n")
    print(json.dumps({"integrity_problems": len(problems),
                      "decision": report["confirmatory"]["primary"]["decision"],
                      "adopt": report["confirmatory"]["adopt"]}, indent=1))
    return 0 if not problems else 1


def cmd_analyse_b_secondary(args):
    """Experiment B secondary outcomes (prereg section 4) into analysis_secondary.json; analysis.json stays."""
    protocol = kp.load_protocol(PROTOCOL)
    out_root = ROOT / protocol["experiment_B"]["out_root"]["main"]
    gate = json.loads((out_root / "b_gate.json").read_text())
    confirmatory = out_root / "analysis.json"
    quota = protocol["evaluation"]["quota_per_env"]
    seeds = protocol["experiment_B"]["seeds"]
    per_cell, metrics, problems = {}, {}, []
    for cell in kp.b_cells(protocol, gate["target_model"], gate["densities"]):
        directory = out_root / cell["cell_dir"]
        receipt = json.loads((directory / "receipt.json").read_text())
        expect = cell_expect(cell, protocol["experiment_B"]["run_ids"]["main"], receipt.get("git_commit"), quota)
        problems += ["%s: %s" % (cell["cell_dir"], p) for p in check_cell(directory, protocol, quota, expect=expect)]
        summary = json.loads((directory / "episode_ledger_summary.json").read_text())
        rows = read_jsonl(directory / summary.get("rows_file", "episodes.jsonl"))
        joined = kp.join_episode_records(kp.quota_rows(rows, protocol["evaluation"]["num_envs"], quota),
                                         read_jsonl(directory / RECORDER_EPISODES))
        key = (cell["arm"], cell["density"], cell["seed"])
        per_cell[key] = kp.b_secondary_cell(joined)
        metrics[key] = kp.cell_metrics(joined)
    capture = kp.seed_level(metrics, ["B0", "B1"], gate["densities"], seeds, "capture")
    report = {"preregistration_sha256": kp.sha256_file(ROOT / B_PREREG),
              "confirmatory_analysis_sha256": kp.sha256_file(confirmatory),
              "source_commit": git("rev-parse", "HEAD").strip(),
              "tracked_tree_dirty": bool(git("status", "--porcelain", "--untracked-files=no").strip()),
              "integrity_problems": problems,
              "secondary": kp.analyse_b_secondary(per_cell, gate["densities"], seeds, capture,
                                                  gate["capture_diff_test"]["mean"]),
              "per_cell": {"%s/%dbars/seed%d" % k: v for k, v in sorted(per_cell.items())}}
    (out_root / "analysis_secondary.json").write_text(json.dumps(report, indent=1, sort_keys=True) + "\n")
    print(json.dumps({"integrity_problems": len(problems),
                      "recovery_fraction": report["secondary"]["recovery_fraction"]["fraction"]}, indent=1))
    return 0 if not problems else 1


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    sub = parser.add_subparsers(dest="command", required=True)
    p = sub.add_parser("pilot")
    p.add_argument("--raw-dir", help="folder holding the git-ignored replication cells")
    p.add_argument("--check", action="store_true", help="compare with the committed pilot JSON")
    pl = sub.add_parser("plan")
    pl.add_argument("--stage", choices=("canary", "main"), required=True)
    pl.add_argument("--moving", default="E2_obstacle_aware",
                    choices=("E2_obstacle_aware", "E1_cv", "none"))
    v = sub.add_parser("validate")
    v.add_argument("--raw-dir", help="folder holding the git-ignored replication cells")
    r = sub.add_parser("run")
    r.add_argument("--stage", choices=("canary", "main", "b_smoke", "b_main", "packets", "mt_canary", "mt_main",
                            "occlusion_pilot"),
                   required=True)
    r.add_argument("--run-id", required=True)
    r.add_argument("--approved-commit", default="")
    r.add_argument("--dry-run", action="store_true")
    r.add_argument("--canary-root", help="canary folder (default: the protocol's); for a re-run")
    r.add_argument("--external-backup-confirmed", action="store_true",
                   help="occlusion_pilot only: the operator confirms an external backup of existing raw results")
    r.add_argument("--max-gpu-hours", type=float, default=None,
                   help="stop before a new cell, and kill a running one, once this wall time is used")
    an = sub.add_parser("analyse")
    an.add_argument("--canary-root", help="canary folder whose report chose the moving model")
    sub.add_parser("analyse-b")
    sub.add_parser("analyse-b-secondary")
    am = sub.add_parser("analyse-a-moving")
    am.add_argument("--static-root", help="folder holding the KROC27-A-M01 A1/A2 raw cells "
                                          "(default: results/kroc2027_A_v1 in this tree)")
    g = sub.add_parser("b-gate")
    g.add_argument("--remaining-gpu-hours", type=float, required=True)
    g.add_argument("--b-run-approved", action="store_true",
                   help="set only after the user has approved the B run")
    g.add_argument("--approved-commit", default="", help="the commit the user approved for B")
    m = sub.add_parser("manifest")
    m.add_argument("--root", required=True, help="raw results folder")
    m.add_argument("--out", help="where to write RAW_MANIFEST.json (default: print a summary only)")
    args = parser.parse_args(argv)
    return {"pilot": cmd_pilot, "plan": cmd_plan, "validate": cmd_validate, "run": cmd_run,
            "analyse": cmd_analyse, "analyse-b": cmd_analyse_b,
            "analyse-b-secondary": cmd_analyse_b_secondary, "b-gate": cmd_b_gate,
            "analyse-a-moving": cmd_analyse_a_moving,
            "manifest": cmd_manifest}[args.command](args)


if __name__ == "__main__":
    sys.exit(main())
