#!/usr/bin/env python3
"""Run the preregistered held-out decision cell for the ref5in D1 adaptation probe.

The contract is fixed before D1 training: seed 331, at least 8,193 requested episodes, 70 bars,
the full 6..28 m goal distribution, deterministic/original actions and governor off.  D1 cannot
rewrite the failed P2 decision and this evaluator cannot unlock P3 automatically.
"""

from __future__ import annotations

from datetime import datetime, timezone
import importlib.util
import json
import math
from pathlib import Path
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[1]
RL_ROOT = ROOT / "aerial_gym/rl_training/rl_games"
RUN_GLOB = "*_navrl_v2-ref5in-d1-q3-adapt-s197"
P1C_RUN = "ppo_260813_0540_navrl_v2-ref5in-smoke-c-s197"
P1C_CHECKPOINT = RL_ROOT / "runs" / P1C_RUN / "nn/last_gen_ppo_ep_900_rew_137.08087.pth"
P1C_SHA = "f1670a1d74dd92cb00d6a58898e9cc1b96eb9cbe155d1e85812a345e7aaae6bf"
OUTPUT = ROOT / "results/navrl_ref5in_d1_eval_seed331"
CELL = OUTPUT / "ref5in"
SOURCE_BUNDLE = OUTPUT / "source_bundle"
SEED = 331
EPISODES = 8193
SOURCE_EPOCH = 900
TERMINAL_EPOCH = 1900
BRANCH_EPOCHS = TERMINAL_EPOCH - SOURCE_EPOCH


def load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


BASE = load_module("ref5in_d1_base", ROOT / "tools/run_navrl_ref5in_outcome_diagnostic.py")
V2 = load_module("ref5in_d1_v2", ROOT / "tools/run_navrl_ref5in_outcome_diagnostic_v2.py")
SMOKE = load_module("ref5in_d1_smoke", ROOT / "tools/analyze_navrl_ref5in_smoke.py")
P2 = load_module("ref5in_d1_p2", ROOT / "tools/attest_navrl_ref5in_p2.py")


def require(condition: bool, message: str) -> None:
    if not condition:
        raise BASE.ContractError(message)


def configure_base(checkpoint: Path, checkpoint_sha: str) -> None:
    BASE.CHECKPOINT = checkpoint
    BASE.CHECKPOINT_SHA = checkpoint_sha
    BASE.OUTPUT = OUTPUT
    BASE.CELL = CELL
    BASE.SOURCE_BUNDLE = SOURCE_BUNDLE
    BASE.SEED = SEED
    BASE.EPISODES = EPISODES


def unique_d1_run() -> Path:
    runs = sorted(path for path in (RL_ROOT / "runs").glob(RUN_GLOB) if path.is_dir())
    require(len(runs) == 1, f"expected exactly one D1 run, found {[p.name for p in runs]}")
    return runs[0]


def verify_training() -> dict:
    import torch

    run = unique_d1_run()
    marker = run / ".aerial_training_finished"
    run_summary = BASE.load_json(run / "aerial_run/run_summary.json")
    lineage = (run / "aerial_run/resumed_from.txt").read_text(encoding="utf-8").strip()
    checkpoint = SMOKE.latest_checkpoint(run, TERMINAL_EPOCH)
    checkpoint_sha = BASE.sha256_file(checkpoint)
    payload = torch.load(str(checkpoint), map_location="cpu", weights_only=False)
    state = payload.get("env_state") or {}
    require(marker.is_file(), "D1 completion marker missing")
    require(run_summary.get("exit_reason") == "max_epochs", "D1 did not end at max_epochs")
    require(int(run_summary.get("first_epoch", -1)) == SOURCE_EPOCH + 1, "D1 branch first epoch mismatch")
    require(int(run_summary.get("last_epoch", -1)) == TERMINAL_EPOCH, "D1 terminal epoch mismatch")
    require(int(run_summary.get("epochs_logged", -1)) == BRANCH_EPOCHS, "D1 branch length mismatch")
    require(lineage == P1C_RUN, "D1 resumed_from lineage mismatch")
    require(int(payload.get("epoch", -1)) == TERMINAL_EPOCH, "D1 checkpoint epoch mismatch")
    require(int(payload.get("frame", -1)) == TERMINAL_EPOCH * 32 * 128, "D1 frame mismatch")
    expected_state = {
        "cfg_training_seed": 197,
        "cfg_training_num_envs": 128,
        "cfg_robot_name": "navrl_ref5in_quad",
        "cfg_obstacle_selector": "cluster_sector",
        "cfg_action_policy": "squashed_gaussian",
        "cfg_speed_governor_mode": "off",
        "cfg_perception_perturb": False,
        "cfg_general_goal_dist_min": 22.5,
        "cfg_general_goal_dist_max": 28.0,
        "n_bars_active": 70,
        "num_task_steps": TERMINAL_EPOCH * 32,
    }
    mismatches = {key: (state.get(key), value) for key, value in expected_state.items()
                  if state.get(key) != value}
    require(not mismatches, f"D1 checkpoint contract mismatch: {mismatches}")
    require(math.isclose(float(state.get("cfg_action_learning_rate", 0)), 1.5e-5, abs_tol=1e-12),
            "D1 initial learning rate mismatch")
    require(math.isclose(float(state.get("current_action_learning_rate", 0)), 1.5e-5, abs_tol=1e-12),
            "D1 current learning rate mismatch")
    nonfinite = SMOKE.all_tensors_finite(payload)
    require(not nonfinite, f"non-finite D1 checkpoint tensors: {nonfinite[:8]}")

    log = SMOKE.matching_log(run.name)
    outcome = SMOKE.outcome_window(log, expected_epochs=BRANCH_EPOCHS)
    tb = SMOKE.tensorboard_summary(run)
    require(not tb["all_scalars"]["empty_tags"] and not tb["all_scalars"]["nonfinite_tags"],
            "D1 TensorBoard contains empty/non-finite scalar tags")
    for tag in ("ppo/kl", "ppo/behavior_kl_audit_max"):
        require(tb[tag]["count"] == BRANCH_EPOCHS and tb[tag]["max"] < 0.04,
                f"D1 {tag} safety gate failed: {tb[tag]}")
    for tag in ("ppo/epoch_rollback_total", "ppo/kl_skipped_minibatches"):
        require(tb[tag]["count"] == BRANCH_EPOCHS and tb[tag]["max"] == 0.0,
                f"D1 {tag} is nonzero: {tb[tag]}")
    for axis in ("x", "y", "z", "yaw"):
        item = tb[f"policy_action/raw_oob_{axis}"]
        require(item["count"] == BRANCH_EPOCHS and item["max"] == 0.0,
                f"D1 raw OOB {axis} is nonzero: {item}")

    manifest = Path(str(state.get("cfg_training_source_manifest", ""))).resolve()
    manifest_sha = str(state.get("cfg_training_source_manifest_sha256", ""))
    source = SMOKE.verify_source_manifest(manifest, manifest_sha)
    require(source["git_dirty"] is False, "D1 runtime receipt was dirty")
    training_map, training_manifest = P2.manifest_map(manifest, 1)
    require(P2.current_runtime_map() == training_map, "current runtime differs from D1 training")
    require(training_manifest.get("python_environment_sha256"), "D1 Python receipt missing")
    return {
        "run": run,
        "checkpoint": checkpoint,
        "checkpoint_sha256": checkpoint_sha,
        "manifest": manifest,
        "manifest_sha256": manifest_sha,
        "runtime_map": training_map,
        "python_environment_sha256": training_manifest["python_environment_sha256"],
        "tensorboard": tb,
        "last100_training_outcomes": outcome,
        "session_log": log,
    }


def verify_eval_source(training: dict) -> None:
    receipt = BASE.load_json(CELL / "70bars.receipt.json")
    manifest = Path(str(receipt.get("runtime_source_manifest", ""))).resolve()
    require(manifest == SOURCE_BUNDLE / "source_manifest.json", "non-canonical D1 eval source bundle")
    eval_map, eval_manifest = P2.manifest_map(manifest, 2)
    require(eval_map == training["runtime_map"], "D1 train/eval runtime byte map mismatch")
    require(eval_manifest.get("python_environment_sha256") == training["python_environment_sha256"],
            "D1 train/eval Python environment mismatch")


def decision_summary(result: dict, training: dict) -> dict:
    base = BASE.summarize(result)
    base["strata"]["distance_by_pattern"] = V2.enriched_joint(result)
    outcome = result["outcome"]
    q3 = base["strata"]["distance"]["q3"]
    q3_cv = base["strata"]["distance_by_pattern"]["q3"]["cv"]
    gates = {
        "global_crash_lte_27pct": {
            "pass": float(outcome["crash_rate"]) <= 0.27,
            "value": float(outcome["crash_rate"]), "threshold": 0.27,
        },
        "q3_crash_lte_30pct": {
            "pass": float(q3["crash_rate"]) <= 0.30,
            "value": float(q3["crash_rate"]), "threshold": 0.30,
        },
        "q3_cv_timeout_lte_12pct": {
            "pass": float(q3_cv["timeout_rate"]) <= 0.12,
            "value": float(q3_cv["timeout_rate"]), "threshold": 0.12,
        },
    }
    passed = all(item["pass"] for item in gates.values())
    return {
        "schema_version": 1,
        "producer": "tools/run_navrl_ref5in_d1_eval.py",
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "scope": "ref5in_d1_heldout_seed331_decision",
        "verdict": "PASS" if passed else "FAIL",
        "decision_authority": "D1 adaptation probe only",
        "p2_verdict_changed": False,
        "p3_automatically_unlocked": False,
        "checkpoint": str(training["checkpoint"].relative_to(ROOT)),
        "checkpoint_sha256": training["checkpoint_sha256"],
        "training_source_manifest_sha256": training["manifest_sha256"],
        "training_safety": {
            "tensorboard": training["tensorboard"],
            "last100_training_outcomes": training["last100_training_outcomes"],
        },
        "condition": result["condition"],
        "outcome": outcome,
        "strata": base["strata"],
        "gates": gates,
        "limitations": [
            "one warm-start training seed and one held-out evaluation seed",
            "70-bar simulation cell only; no hardware claim",
            "PASS permits manual P3 preregistration review, not automatic P3 launch",
            "the original seed313 P2 decision remains strict FAIL",
        ],
    }


def write_summary(payload: dict) -> None:
    (OUTPUT / "summary.json").write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    q3 = payload["strata"]["distance"]["q3"]
    cv = payload["strata"]["distance_by_pattern"]["q3"]["cv"]
    outcome = payload["outcome"]
    lines = [
        f"# ref5in D1 held-out decision — {payload['verdict']}", "",
        "D1은 P2를 재채점하지 않으며 P3를 자동 해제하지 않는다.", "",
        "| gate | value | limit | pass |", "|---|---:|---:|:---:|",
        f"| global crash | {outcome['crash_rate']*100:.2f}% | ≤27% | {payload['gates']['global_crash_lte_27pct']['pass']} |",
        f"| q3 crash | {q3['crash_rate']*100:.2f}% | ≤30% | {payload['gates']['q3_crash_lte_30pct']['pass']} |",
        f"| q3/CV timeout | {cv['timeout_rate']*100:.2f}% | ≤12% | {payload['gates']['q3_cv_timeout_lte_12pct']['pass']} |",
        "", f"Checkpoint SHA-256: `{payload['checkpoint_sha256']}`", "",
    ]
    (OUTPUT / "summary.md").write_text("\n".join(lines), encoding="utf-8")


def main() -> int:
    mode = sys.argv[1] if len(sys.argv) == 2 else ""
    require(mode in {"preflight", "run", "finalize", "verify"},
            "usage: ... {preflight|run|finalize|verify}")
    if mode == "preflight":
        require(BASE.sha256_file(P1C_CHECKPOINT) == P1C_SHA, "P1c checkpoint identity mismatch")
        configure_base(P1C_CHECKPOINT, P1C_SHA)
        require(not OUTPUT.exists(), f"output already exists: {OUTPUT}")
        BASE.run_evaluator(preflight=True)
        require(not OUTPUT.exists(), "preflight created D1 result output")
        print("[ref5in-d1-eval] PREFLIGHT PASS")
        return 0

    training = verify_training()
    configure_base(training["checkpoint"], training["checkpoint_sha256"])
    if mode == "run":
        status = subprocess.check_output(
            ["git", "-C", str(ROOT), "status", "--porcelain", "--untracked-files=no"], text=True
        )
        require(not status.strip(), "tracked source is dirty; commit before D1 evaluation")
        require(not OUTPUT.exists(), f"refusing overwrite: {OUTPUT}")
        BASE.run_evaluator()
    result = BASE.verify_result()
    V2.verify_joint(result)
    verify_eval_source(training)
    expected = decision_summary(result, training)
    if mode in {"run", "finalize"}:
        write_summary(expected)
        print(f"[ref5in-d1-eval] {expected['verdict']}")
        return 0 if expected["verdict"] == "PASS" else 1
    recorded = BASE.load_json(OUTPUT / "summary.json")
    for key in ("scope", "verdict", "decision_authority", "p2_verdict_changed",
                "p3_automatically_unlocked", "checkpoint_sha256", "condition", "outcome",
                "strata", "gates", "limitations"):
        require(recorded.get(key) == expected.get(key), f"D1 summary changed: {key}")
    print(f"[ref5in-d1-eval] VERIFY {recorded['verdict']}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (BASE.ContractError, OSError, RuntimeError, subprocess.CalledProcessError,
            json.JSONDecodeError) as exc:
        print(f"[ref5in-d1-eval] FAIL: {exc}", file=sys.stderr)
        raise SystemExit(2)
