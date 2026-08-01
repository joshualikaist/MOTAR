#!/usr/bin/env python3
"""Synchronize the static research dashboard with local NavRL run evidence.

The dashboard has two data sources by design: ``status.json`` for HTTP hosting and an inline
fallback in ``index.html`` for direct/offline viewing. This tool always writes both from the same
object so they cannot silently drift.
"""

from __future__ import annotations

import csv
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import re
from statistics import fmean
from typing import Any, Dict, Iterable, List, Optional


ROOT = Path(__file__).resolve().parents[1]
RL_ROOT = ROOT / "aerial_gym/rl_training/rl_games"
RUNS_ROOT = RL_ROOT / "runs"
STATUS_PATH = ROOT / "docs/status/status.json"
HTML_PATH = ROOT / "docs/status/index.html"
CORRECTED_CURVE_PATH = ROOT / "results/corrected_chirality_density_curve.csv"
RECOVERY_ATTESTATION_VERIFIER_PATH = ROOT / "tools/navrl_v2_recovery_attestation.py"

_RECOVERY_SOURCE_SHA256 = (
    "3a0c167cbf4bc966426488f562da2b6788bd00ca62e3a31f226f5fbe1967578f"
)
_RECOVERY_CHECKPOINT_CONTRACT = {
    "num_task_steps": 307200,
    "cfg_ppo_horizon": 32,
    "k_max_cur": 28.0,
    "k_min_cur": 20.0,
    "cfg_training_seed": 1,
    "cfg_training_num_envs": 128,
    "cfg_training_file": "ppo_navrl_perception_transformer.yaml",
    "cfg_training_task": "navrl_task",
    "cfg_training_sim": "base_sim",
    "cfg_training_profile": "main",
    "cfg_runtime_sim_config_class": "BaseSimConfig",
    "cfg_physics_dt_s": 0.01,
    "cfg_physics_substeps": 1,
    "cfg_physics_steps_per_rl_step": 10,
    "cfg_rl_step_dt_s": 0.1,
    "cfg_arena_xy": 40.0,
    "cfg_arena_z": 3.0,
    "cfg_bar_pool": "bars_h3",
    "cfg_placement_mode": "navrl_band",
    "cfg_placement_gap_m": 1.6,
    "cfg_placement_touch_m": 0.4,
    "cfg_bar_x_min": 0.0,
    "cfg_bar_x_max": 1.0,
    "cfg_episode_len_steps": 600,
    "cfg_general_goal_dist_min": 6.0,
    "cfg_general_goal_dist_max": 28.0,
    "cfg_lidar_max_range": 12.0,
    "cfg_lidar_hbeams": 72,
    "cfg_lidar_vbeams": 4,
    "cfg_max_obstacles": 8,
    "cfg_token_fov_deg": 240.0,
    "cfg_obstacle_suppress_deg": 10.0,
    "cfg_obstacle_selector": "cluster_sector",
    "cfg_obstacle_cluster_gap_m": 0.45,
    "cfg_obstacle_sectors": 8,
    "cfg_obstacle_ttc_idle_s": 30.0,
    "cfg_obstacle_ttc_min_speed": 0.15,
    "cfg_corridor_tokens": 0,
    "cfg_corridor_horizon_m": 6.0,
    "cfg_corridor_min_width_m": 0.55,
    "cfg_fov_curriculum_epochs": 3000,
    "cfg_detector_min_pixels": 2,
    "cfg_detector_threshold": 0.55,
    "cfg_detector_checkpoint_name": "",
    "cfg_detector_checkpoint_sha256": "",
    "cfg_perception_perturb": False,
    "cfg_detection_dropout": 0.3,
    "cfg_rgb_noise_std": 0.015,
    "cfg_depth_noise_std": 0.02,
    "cfg_max_velocity": 2.5,
    "cfg_alt_hold_vmax": 2.5,
    "cfg_yaw_rate_max": 3.0,
    "cfg_max_tilt_deg": 45.0,
    "cfg_tilt_comp": True,
    "cfg_target_motion_model": "symmetric_local_steer_v2_heading_continuity90",
    "cfg_target_pattern": "mixed",
    "cfg_target_speed_min": 0.3,
    "cfg_target_speed_final": 1.5,
    "cfg_target_speed_fixed": -1.0,
    "cfg_target_speed_ramp_epochs": 300,
    "cfg_target_speed_ramp_start_epochs": 0,
    "cfg_general_train": True,
    "cfg_oob_margin": 1.0,
    "cfg_action_policy": "squashed_gaussian",
    "cfg_action_std": "0.35,0.35,0.05,0.08",
    "cfg_action_mu_scale": "1.0,0.4,1.0,1.0",
    "cfg_action_entropy_coef": 0.0,
    "cfg_action_learning_rate": 5e-6,
    "current_action_learning_rate": 5e-6,
    "cfg_ppo_log_ratio_clamp": 10.0,
    "cfg_ppo_kl_stop": 0.04,
    "cfg_ppo_epoch_rollback": True,
    "cfg_ppo_rollback_lr_factor": 0.5,
    "cfg_ppo_rollback_min_lr": 1e-6,
    "cfg_ppo_rollback_patience": 5,
    "cfg_density_guard_window_epochs": 50,
    "cfg_density_guard_min_epochs": 100,
    "cfg_density_guard_min_peak": 0.5,
    "cfg_density_guard_drop": 0.25,
    "cfg_density_guard_patience": 25,
    "cfg_lateral_latent_margin_y": 1.25,
    "cfg_latent_margin": "2.0,1.25,2.0,2.0",
    "cfg_lateral_latent_margin_coef": 0.01,
}
_RECOVERY_RESULT_CONTRACT = {
    "schema_version": 1,
    "runtime_sim": "base_sim",
    "runtime_profile": "main",
    "runtime_num_envs": 128,
    "sim_physics_contract": "base_sim_dt0.01",
    "runtime_sim_config_class": "BaseSimConfig",
    "physics_dt_s": 0.01,
    "physics_substeps": 1,
    "physics_steps_per_rl_step": 10,
    "rl_step_dt_s": 0.1,
    "arena_xy_m": 40.0,
    "goal_dist_min_m": 6.0,
    "goal_dist_max_m": 28.0,
    "full_goal_distribution": True,
    "fov_curriculum_saturated": True,
    "target_speed_distribution": "uniform",
    "target_speed_min_mps": 0.3,
    "target_speed_max_mps": 1.5,
    "target_pattern": "mixed",
    "lidar_beams": [4, 72],
    "lidar_range_m": 12.0,
    "obstacle_tokens": 8,
    "obstacle_fov_deg": 240.0,
    "obstacle_selector": "cluster_sector",
    "obstacle_ttc_idle_s": 30.0,
    "obstacle_ttc_min_speed": 0.15,
    "fov_curriculum_epochs": 3000,
    "detector_checkpoint_sha256": "",
    "detector_min_pixels": 2,
    "detector_threshold": 0.55,
    "perception_perturb": False,
    "detection_dropout": 0.3,
    "rgb_noise_std": 0.015,
    "depth_noise_std": 0.02,
    "max_tilt_deg": 45.0,
    "tilt_comp": True,
    "oob_margin_m": 1.0,
    "seed": 42,
}
_RECOVERY_ATTESTATION_THRESHOLDS = {
    "min_episodes": 2049,
    "min_capture_rate": 0.65,
    "max_crash_rate": 0.35,
    "max_timeout_rate": 0.10,
    "max_training_kl": 0.04,
    "max_task_input_oob_rate": 1e-9,
}


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _finite_number(value: Any) -> Optional[float]:
    if isinstance(value, bool):
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _contract_value_matches(actual: Any, expected: Any) -> bool:
    if isinstance(expected, bool):
        return isinstance(actual, bool) and actual is expected
    if isinstance(expected, (int, float)) and not isinstance(expected, bool):
        number = _finite_number(actual)
        return number is not None and math.isclose(
            number, float(expected), rel_tol=0.0, abs_tol=1e-9
        )
    return actual == expected


def _check_contract(
    actual: Any, expected: Dict[str, Any], prefix: str, errors: List[str]
) -> None:
    if not isinstance(actual, dict):
        errors.append(f"{prefix}: not an object")
        return
    for key, expected_value in expected.items():
        if key not in actual:
            errors.append(f"{prefix}.{key}: missing")
        elif not _contract_value_matches(actual[key], expected_value):
            errors.append(f"{prefix}.{key}: contract mismatch")


def _run_canonical_recovery_verifier(
    checkpoint_path: Path, attestation_path: Path, errors: List[str]
) -> None:
    """Recompute the attestation through its canonical checkpoint/result/TB verifier.

    The dashboard must fail closed if the verifier cannot be imported or executed.  In
    particular, self-reported KL/OOB/rollback values are not evidence: the canonical verifier
    rereads the exact 100-epoch TensorBoard window and rebuilds the complete payload.
    """

    try:
        spec = importlib.util.spec_from_file_location(
            "_navrl_v2_recovery_attestation_for_status",
            RECOVERY_ATTESTATION_VERIFIER_PATH,
        )
        if spec is None or spec.loader is None:
            raise ImportError("module spec has no loader")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        verifier = getattr(module, "verify_existing_attestation", None)
        if not callable(verifier):
            raise AttributeError("verify_existing_attestation is unavailable")
        verifier(checkpoint_path, attestation_path)
    except Exception as exc:
        errors.append(f"canonical verifier: {exc}")


def _normal_completion_marker_epoch(run_dir: Path) -> Optional[int]:
    """Return the epoch only for the exact marker emitted by a normal trainer exit.

    A metrics row at epoch 9600 proves that an epoch was logged, not that the runner completed its
    save/flush/finish path.  Treat extra or malformed marker content as invalid too, so a stale or
    hand-edited file cannot turn an interrupted smoke into a completed one on the dashboard.
    """

    try:
        marker_text = (run_dir / ".aerial_training_finished").read_text(encoding="utf-8")
    except OSError:
        return None
    match = re.fullmatch(r"epoch=(\d+)\n?", marker_text)
    return int(match.group(1)) if match else None


def _validate_recovery_attestation(run_dir: Path) -> List[str]:
    """Validate every artifact behind the dashboard's recovery PASS claim.

    The attestation file is deliberately not treated as a signature.  It is accepted only when its
    checkpoint and held-out paths still exist, both byte digests match, the checkpoint carries the
    complete safe-recovery contract, and the held-out artifact independently reproduces every
    seed/outcome/OOB/rollback field quoted by the attestation.
    """
    run_dir = run_dir.resolve()
    errors: List[str] = []
    attestation_path = run_dir / ".navrl_v2_recovery_eval_pass.json"
    if not attestation_path.is_file():
        return ["attestation: missing"]
    try:
        attestation = json.loads(attestation_path.read_text(encoding="utf-8"))
    except (OSError, ValueError) as exc:
        return [f"attestation: unreadable ({exc})"]
    if not isinstance(attestation, dict):
        return ["attestation: not an object"]

    def expect_equal(label: str, actual: Any, expected: Any) -> None:
        if not _contract_value_matches(actual, expected):
            errors.append(f"{label}: mismatch")

    expect_equal("attestation.schema_version", attestation.get("schema_version"), 1)
    expect_equal("attestation.verdict", attestation.get("verdict"), "PASS")
    expect_equal("attestation.checkpoint_epoch", attestation.get("checkpoint_epoch"), 9600)
    expect_equal("attestation.checkpoint_frame", attestation.get("checkpoint_frame"), 39321600)
    expect_equal(
        "attestation.source_checkpoint_sha256",
        attestation.get("source_checkpoint_sha256"),
        _RECOVERY_SOURCE_SHA256,
    )
    expect_equal("attestation.source_epoch", attestation.get("source_epoch"), 9500)
    expect_equal("attestation.smoke_epochs", attestation.get("smoke_epochs"), 100)
    expect_equal("attestation.bars", attestation.get("bars"), 130)
    expect_equal("attestation.seed", attestation.get("seed"), 42)
    _check_contract(
        attestation.get("thresholds"),
        _RECOVERY_ATTESTATION_THRESHOLDS,
        "attestation.thresholds",
        errors,
    )
    _check_contract(
        attestation.get("evaluation_contract"),
        _RECOVERY_RESULT_CONTRACT,
        "attestation.evaluation_contract",
        errors,
    )
    created_at = str(attestation.get("created_at_utc", ""))
    try:
        created = datetime.fromisoformat(created_at.replace("Z", "+00:00"))
        if created.tzinfo is None:
            raise ValueError("timezone missing")
    except ValueError:
        errors.append("attestation.created_at_utc: invalid")

    if _normal_completion_marker_epoch(run_dir) != 9600:
        errors.append("training marker: missing, malformed, or not exact epoch 9600")

    checkpoint_path = Path(str(attestation.get("checkpoint", ""))).expanduser().resolve()
    expected_nn_dir = (run_dir / "nn").resolve()
    if checkpoint_path.parent != expected_nn_dir:
        errors.append("checkpoint: outside recovery run nn directory")
    if not checkpoint_path.is_file():
        errors.append("checkpoint: missing")
        checkpoint_digest = None
        checkpoint = None
    else:
        try:
            checkpoint_digest = _sha256(checkpoint_path)
        except OSError as exc:
            errors.append(f"checkpoint: unreadable ({exc})")
            checkpoint_digest = None
        if checkpoint_digest != attestation.get("checkpoint_sha256"):
            errors.append("checkpoint: SHA-256 mismatch")
        try:
            import torch

            checkpoint = torch.load(
                checkpoint_path, map_location="cpu", weights_only=False
            )
        except Exception as exc:
            errors.append(f"checkpoint: cannot load ({exc})")
            checkpoint = None

    if not isinstance(checkpoint, dict):
        if checkpoint is not None:
            errors.append("checkpoint: not an object")
    else:
        expect_equal("checkpoint.epoch", checkpoint.get("epoch"), 9600)
        expect_equal("checkpoint.frame", checkpoint.get("frame"), 39321600)
        state = checkpoint.get("env_state")
        _check_contract(
            state, _RECOVERY_CHECKPOINT_CONTRACT, "checkpoint.env_state", errors
        )
        if isinstance(state, dict):
            lineage = {
                "cfg_recovery_stage": "smoke",
                "cfg_recovery_source_sha256": _RECOVERY_SOURCE_SHA256,
                "cfg_recovery_source_epoch": 9500,
                "cfg_recovery_smoke_required_epochs": 100,
                "cfg_recovery_smoke_bars": 130,
                "n_bars_active": 130,
            }
            _check_contract(state, lineage, "checkpoint.env_state", errors)

    # This is the authoritative recomputation of checkpoint/result/TensorBoard evidence. Keep the
    # local checks below as an independent, dashboard-specific fail-closed layer, but never unlock
    # merely because a hand-written JSON repeats plausible PASS values.
    if checkpoint_path.is_file():
        _run_canonical_recovery_verifier(checkpoint_path, attestation_path, errors)

    result_path = Path(
        str(attestation.get("heldout_result_json", ""))
    ).expanduser().resolve()
    if not result_path.is_file():
        errors.append("held-out result: missing")
        result = None
    else:
        try:
            result_digest = _sha256(result_path)
            result = json.loads(result_path.read_text(encoding="utf-8"))
        except (OSError, ValueError) as exc:
            errors.append(f"held-out result: unreadable ({exc})")
            result = None
        else:
            if result_digest != attestation.get("heldout_result_sha256"):
                errors.append("held-out result: SHA-256 mismatch")

    if not isinstance(result, dict):
        if result is not None:
            errors.append("held-out result: not an object")
        return errors

    expect_equal("held-out.schema_version", result.get("schema_version"), 1)
    try:
        result_checkpoint = Path(str(result.get("checkpoint", ""))).expanduser().resolve()
    except (OSError, RuntimeError, ValueError):
        result_checkpoint = Path("/")
    if result_checkpoint != checkpoint_path:
        errors.append("held-out.checkpoint: mismatch")

    requested = _finite_number(result.get("requested_episodes"))
    actual = _finite_number(result.get("actual_episodes"))
    if requested is None or not requested.is_integer() or requested < 2049:
        errors.append("held-out.requested_episodes: invalid")
    if (
        actual is None
        or not actual.is_integer()
        or actual < 2049
        or (requested is not None and actual < requested)
    ):
        errors.append("held-out.actual_episodes: invalid")

    condition = result.get("condition")
    expected_condition = {
        "seed": 42,
        "bars": 130,
        "num_envs": 128,
        "target_pattern": "mixed",
        "target_speed_mode": "uniform",
        "target_speed_min_mps": 0.3,
        "target_speed_max_mps": 1.5,
        "pursuer_max_speed_mps": 2.5,
        "oob_margin_m": 1.0,
        "episode_len_steps": 600,
        "goal_dist_min_m": 6.0,
        "goal_dist_max_m": 28.0,
        "full_goal_distribution": True,
        "fov_curriculum_saturated": True,
        "runtime_sim_config_class": "BaseSimConfig",
        "physics_dt_s": 0.01,
        "physics_substeps": 1,
        "physics_steps_per_rl_step": 10,
        "rl_step_dt_s": 0.1,
    }
    _check_contract(condition, expected_condition, "held-out.condition", errors)
    _check_contract(
        result.get("v2_evaluation_contract"),
        _RECOVERY_RESULT_CONTRACT,
        "held-out.v2_evaluation_contract",
        errors,
    )

    outcome = result.get("outcome")
    rates: Dict[str, Optional[float]] = {}
    count_values: Dict[str, Optional[float]] = {}
    if not isinstance(outcome, dict):
        errors.append("held-out.outcome: not an object")
    else:
        counts = []
        for name in ("captured", "crash", "timeout"):
            value = _finite_number(outcome.get(name))
            if value is None or not value.is_integer() or value < 0:
                errors.append(f"held-out.outcome.{name}: invalid")
            counts.append(value)
            count_values[name] = value
        for name in ("capture_rate", "crash_rate", "timeout_rate"):
            value = _finite_number(outcome.get(name))
            rates[name] = value
            if value is None or not 0.0 <= value <= 1.0:
                errors.append(f"held-out.outcome.{name}: invalid")
        if actual is not None and all(value is not None for value in counts):
            if not math.isclose(sum(counts), actual, rel_tol=0.0, abs_tol=0.0):
                errors.append("held-out.outcome: counts do not equal actual episodes")
            elif actual > 0:
                for count, rate_name in zip(
                    counts, ("capture_rate", "crash_rate", "timeout_rate")
                ):
                    rate = rates.get(rate_name)
                    if rate is not None and not math.isclose(
                        rate, count / actual, rel_tol=0.0, abs_tol=1e-12
                    ):
                        errors.append(f"held-out.outcome.{rate_name}: count mismatch")

    capture_rate = rates.get("capture_rate")
    crash_rate = rates.get("crash_rate")
    timeout_rate = rates.get("timeout_rate")
    if capture_rate is not None and capture_rate < 0.65:
        errors.append("held-out.outcome.capture_rate: below gate")
    if crash_rate is not None and crash_rate > 0.35:
        errors.append("held-out.outcome.crash_rate: above gate")
    if timeout_rate is not None and timeout_rate > 0.10:
        errors.append("held-out.outcome.timeout_rate: above gate")

    action = result.get("action")
    eval_oob: List[float] = []
    if not isinstance(action, dict):
        errors.append("held-out.action: not an object")
    else:
        expect_equal("held-out.action.policy", action.get("policy"), "squashed_gaussian")
        samples = _finite_number(action.get("samples"))
        if samples is None or not samples.is_integer() or samples <= 0:
            errors.append("held-out.action.samples: invalid")
        raw_oob = action.get("task_input_oob_rate")
        if not isinstance(raw_oob, list) or len(raw_oob) != 4:
            errors.append("held-out.action.task_input_oob_rate: expected four axes")
        else:
            for value in raw_oob:
                number = _finite_number(value)
                if number is None or not 0.0 <= number <= 1e-9:
                    errors.append("held-out.action.task_input_oob_rate: invalid")
                else:
                    eval_oob.append(number)

    mirror_fields = {
        "checkpoint_epoch": 9600,
        "bars": 130,
        "seed": 42,
        "requested_episodes": requested,
        "episodes": actual,
        "captured": count_values.get("captured"),
        "crash": count_values.get("crash"),
        "timeout": count_values.get("timeout"),
        "capture_rate": capture_rate,
        "crash_rate": crash_rate,
        "timeout_rate": timeout_rate,
    }
    for name, expected in mirror_fields.items():
        if expected is None or not _contract_value_matches(attestation.get(name), expected):
            errors.append(f"attestation.{name}: held-out mismatch")

    training_kl = _finite_number(attestation.get("training_max_kl"))
    training_oob = _finite_number(
        attestation.get("training_max_task_input_oob_rate")
    )
    attested_eval_oob = _finite_number(
        attestation.get("evaluation_max_task_input_oob_rate")
    )
    combined_oob = _finite_number(attestation.get("max_task_input_oob_rate"))
    if training_kl is None or not -1e-6 <= training_kl <= 0.04:
        errors.append("attestation.training_max_kl: invalid")
    for name, value in (
        ("training_max_task_input_oob_rate", training_oob),
        ("evaluation_max_task_input_oob_rate", attested_eval_oob),
        ("max_task_input_oob_rate", combined_oob),
    ):
        if value is None or not 0.0 <= value <= 1e-9:
            errors.append(f"attestation.{name}: invalid")
    if eval_oob and attested_eval_oob is not None and not math.isclose(
        attested_eval_oob, max(eval_oob), rel_tol=0.0, abs_tol=1e-12
    ):
        errors.append("attestation.evaluation_max_task_input_oob_rate: held-out mismatch")
    if (
        training_oob is not None
        and attested_eval_oob is not None
        and combined_oob is not None
        and not math.isclose(
            combined_oob,
            max(training_oob, attested_eval_oob),
            rel_tol=0.0,
            abs_tol=1e-12,
        )
    ):
        errors.append("attestation.max_task_input_oob_rate: component mismatch")
    for name in (
        "max_rollback_streak",
        "final_rollback_streak",
        "max_epoch_rollback",
        "max_rollback_total",
        "final_rollback_total",
    ):
        value = _finite_number(attestation.get(name))
        if value is None or value != 0.0:
            errors.append(f"attestation.{name}: expected zero")
    return errors


def _float(row: Dict[str, str], key: str) -> Optional[float]:
    try:
        value = float(row[key])
    except (KeyError, TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def _int(row: Dict[str, str], key: str) -> Optional[int]:
    value = _float(row, key)
    return int(value) if value is not None else None


def _iso_mtime(path: Path) -> str:
    return datetime.fromtimestamp(path.stat().st_mtime, timezone.utc).isoformat()


def _mean(rows: Iterable[Dict[str, str]], key: str) -> Optional[float]:
    values = [value for row in rows if (value := _float(row, key)) is not None]
    return fmean(values) if values else None


def _training_process_exists() -> bool:
    proc = Path("/proc")
    if not proc.is_dir():
        return False
    for cmdline_path in proc.glob("[0-9]*/cmdline"):
        try:
            command = cmdline_path.read_bytes().replace(b"\0", b" ").decode(errors="ignore")
        except (OSError, PermissionError):
            continue
        if "runner.py" in command and "--task navrl_task" in command and "--train" in command:
            return True
    return False


def _is_smoke_run(run_name: str) -> bool:
    """Keep short wiring checks in history without presenting them as research results."""
    lowered = run_name.lower()
    # The 100-epoch v2 recovery smoke is a real gated recovery stage and must become the latest
    # dashboard record when complete. Only one/few-epoch wiring checks stay out of Latest.
    if "v2-recover-smoke" in lowered:
        return False
    return any(
        marker in lowered
        for marker in ("smoke", "integration", "forced", "preflight")
    )


def _live_training_max_epochs(default: int = 12000) -> int:
    """Read the active NavRL runner's explicit epoch ceiling when available."""
    proc = Path("/proc")
    if not proc.is_dir():
        return default
    pattern = re.compile(r"(?:^|\s)--max_epochs\s+(\d+)(?:\s|$)")
    for cmdline_path in proc.glob("[0-9]*/cmdline"):
        try:
            command = cmdline_path.read_bytes().replace(b"\0", b" ").decode(errors="ignore")
        except (OSError, PermissionError):
            continue
        if "runner.py" not in command or "--task navrl_task" not in command or "--train" not in command:
            continue
        if match := pattern.search(command):
            return int(match.group(1))
    return default


def _load_rows(csv_path: Path) -> List[Dict[str, str]]:
    with csv_path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def _summarize_run(csv_path: Path, *, is_active: bool) -> Dict[str, Any]:
    run_dir = csv_path.parents[1]
    rows = _load_rows(csv_path)
    if not rows:
        raise ValueError(f"empty metrics CSV: {csv_path}")
    last = rows[-1]
    peak_capture = max(rows, key=lambda row: _float(row, "captured_rate") or -math.inf)
    peak_reward = max(rows, key=lambda row: _float(row, "mean_reward") or -math.inf)
    summary_path = run_dir / "aerial_run/run_summary.json"
    saved = {}
    if summary_path.is_file():
        saved = json.loads(summary_path.read_text(encoding="utf-8"))

    finalized_at = None if is_active else saved.get("finalized_at", _iso_mtime(csv_path))
    summary = {
        "run": run_dir.name,
        "finalized_at": finalized_at,
        "exit_reason": "running" if is_active else saved.get("exit_reason", "interrupted"),
        "epochs_logged": len(rows),
        "first_epoch": _int(rows[0], "epoch"),
        "last_epoch": _int(last, "epoch"),
        "is_navrl": True,
        "last_mean_reward": _float(last, "mean_reward"),
        "last_mean_episode_length": _float(last, "mean_episode_length"),
        "last_captured_rate": _float(last, "captured_rate"),
        "last_crash_rate": _float(last, "crash_rate"),
        "last_timeout_rate": _float(last, "timeout_rate"),
        "last_closest_approach_m": _float(last, "closest_approach_m"),
        "last_closest_min_m": _float(last, "closest_min_m"),
        "last_curriculum_max_m": _float(last, "curriculum_max_m"),
        "last_n_bars_active": _int(last, "n_bars_active"),
        "peak_captured_rate": _float(peak_capture, "captured_rate"),
        "peak_captured_epoch": _int(peak_capture, "epoch"),
        "peak_mean_reward": _float(peak_reward, "mean_reward"),
        "peak_epoch": _int(peak_reward, "epoch"),
        "reward_collapse": bool(saved.get("reward_collapse", False)),
    }
    # The generic peak-relative guard was disabled for density curricula, but this run is a
    # measured PPO collapse rather than a normal promotion drop.
    if run_dir.name.startswith("ppo_260730_1154_"):
        summary["reward_collapse"] = True
        summary["collapse_detail"] = (
            "KL crossed 0.04 at epoch 10276; latent means exploded and tail500 capture fell to 1.0%."
        )
    if run_dir.name.startswith("ppo_260731_2012_"):
        summary["reward_collapse"] = True
        summary["collapse_detail"] = (
            "Actor-update collapse at epoch 10836: KL peaked above 2.6, transformed entropy "
            "fell below -100, and capture reached 0%. The same-density guard fail-stopped it."
        )
    return summary


def _latest_barprobe() -> Dict[str, Optional[float]]:
    live_link = RL_ROOT / "train_session_logs/current_training.log"
    if not live_link.exists():
        return {"unique": None, "duplicate": None}
    pattern = re.compile(r"unique=([0-9.]+) duplicate=([0-9.]+)")
    matches = pattern.findall(live_link.read_text(encoding="utf-8", errors="ignore"))
    if not matches:
        return {"unique": None, "duplicate": None}
    unique, duplicate = matches[-1]
    return {"unique": float(unique), "duplicate": float(duplicate)}


def _corrected_density_curve() -> Dict[str, Any]:
    rows = []
    with CORRECTED_CURVE_PATH.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            rows.append({key: float(value) for key, value in row.items()})
    return {
        "notes": [
            "Corrected bearing/chirality + bounded squashed-Gaussian backbone.",
            "Deterministic held-out evaluation of the 85-bar training policy; ~2049 episodes/cell.",
            "85-bar training plateau is 0.676 +/- 0.001; the 0.689 table cell is held-out evaluation.",
            "65-bar predecessor plateau was 0.678: equal competence at +31% training density.",
            "This curve used greedy +/-10 deg suppression. The active cluster-sector selector is a same-shape ablation and is not in this curve.",
        ],
        "rows": rows,
        "mtime": _iso_mtime(CORRECTED_CURVE_PATH),
    }


def _active_record(
    summary: Dict[str, Any], csv_path: Path, *, max_epochs: int
) -> Dict[str, Any]:
    rows = _load_rows(csv_path)
    tail = rows[-50:]
    age_min = max(0.0, (datetime.now(timezone.utc).timestamp() - csv_path.stat().st_mtime) / 60.0)
    return {
        "run": summary["run"],
        "is_live": True,
        "metrics_age_min": age_min,
        "epoch": summary["last_epoch"],
        "max_epochs": max_epochs,
        "epochs_logged": summary["epochs_logged"],
        "tail_epochs": len(tail),
        "captured_rate": _mean(tail, "captured_rate"),
        "crash_rate": _mean(tail, "crash_rate"),
        "timeout_rate": _mean(tail, "timeout_rate"),
        "mean_reward": _mean(tail, "mean_reward"),
        "closest_approach_m": _mean(tail, "closest_approach_m"),
        "n_bars_active": _mean(tail, "n_bars_active"),
        "curriculum_max_m": _mean(tail, "curriculum_max_m"),
    }


def _live_density_promotions() -> List[Dict[str, Any]]:
    live_link = RL_ROOT / "train_session_logs/current_training.log"
    if not live_link.exists():
        return []
    pattern = re.compile(
        r"density curriculum promoted \| bars (\d+) -> (\d+) "
        r"after (\d+) eps, capture=([0-9.]+)"
    )
    promotions = []
    for source, target, episodes, capture in pattern.findall(
        live_link.read_text(encoding="utf-8", errors="ignore")
    ):
        promotions.append(
            {
                "source": int(source),
                "target": int(target),
                "episodes": int(episodes),
                "capture": float(capture),
            }
        )
    return promotions


def _v2_search_update(record: Dict[str, Any], *, is_live: bool) -> Dict[str, Any]:
    run_name = str(record.get("run", ""))
    if "v2-ttc-" in run_name:
        arm = "ttc" if "v2-ttc-ttc-" in run_name else "baseline"
        selector = "ttc_sector" if arm == "ttc" else "cluster_sector"
        epoch = int(record.get("epoch") if is_live else record.get("last_epoch") or 0)
        bars = int(
            round(record.get("n_bars_active") or record.get("last_n_bars_active") or 70)
        )
        completed = not is_live and epoch >= 5250
        capture_tail = (
            record.get("captured_rate") if is_live else record.get("last_captured_rate")
        )
        crash_tail = record.get("crash_rate") if is_live else record.get("last_crash_rate")
        return {
            "subtitle": f"2026-08-01 · fixed-70 selector A/B · {arm} arm",
            "headline": (
                f"The {arm} arm is running at a fixed 70 bars with {selector}."
                if is_live
                else (
                    f"The {arm} arm finished its matched 4.1M-step adaptation budget."
                    if completed
                    else f"The {arm} arm stopped before its matched adaptation budget completed."
                )
            ),
            "summary": (
                "This is not a density curriculum. Both arms start from the SHA-256-pinned ep3250 "
                "70-bar checkpoint, keep every task/action setting fixed, and differ only in how "
                "eight obstacle tokens are ranked. Training-tail rates are diagnostics; the "
                "preregistered decision uses matched held-out evaluations of both final checkpoints."
            ),
            "active_experiment": {
                **record,
                "is_live": is_live,
                "epoch": epoch,
                "max_epochs": 5250,
                "bars": bars,
                "selector": selector,
                "cluster_gap_m": 0.45,
                "sectors": 8,
                "ab_arm": arm,
                "density_curriculum": False,
                "arena_xy_m": 40,
                "arena_z_m": 3,
                "warm_start_epoch": 3250,
                "adaptation_steps": max(0, epoch - 3250) * 2048,
            },
            "milestones": [
                {
                    "label": "A/B ARM",
                    "value": arm.upper(),
                    "detail": f"selector={selector}; all other variables fixed",
                    "state": "active" if is_live else ("pass" if completed else "warn"),
                },
                {
                    "label": "DENSITY",
                    "value": "70 FIXED",
                    "detail": "promotion disabled for the entire comparison",
                    "state": "pass",
                },
                {
                    "label": "BUDGET",
                    "value": f"{max(0, epoch - 3250)} / 2000 epochs",
                    "detail": "64 envs · 4.1M adaptation samples",
                    "state": "pass" if completed else "active",
                },
                {
                    "label": "DECISION",
                    "value": "HELD-OUT A/B",
                    "detail": "capture +2pp and crash -2pp required together",
                    "state": "active",
                },
            ],
            "comparison": [
                {
                    "label": f"{arm} training tail",
                    "bars": bars,
                    "capture": capture_tail,
                    "unique": None,
                    "verdict": (
                        f"crash={crash_tail:.3f}; diagnostic only"
                        if isinstance(crash_tail, (int, float))
                        else "diagnostic only; held-out result pending"
                    ),
                }
            ],
            "gates": [
                {"label": "single experimental variable", "value": f"PASS · {selector}"},
                {"label": "density curriculum", "value": "OFF · 70 bars fixed"},
                {"label": "capture delta", "value": "PENDING · TTC - baseline ≥ +2.0pp"},
                {"label": "crash delta", "value": "PENDING · TTC - baseline ≤ -2.0pp"},
            ],
            "decision": (
                "Finish both arms, then evaluate their final checkpoints at the same 70-bar "
                "condition. Adopt TTC ranking only if both preregistered gates pass."
            ),
        }

    if "v2-recover-smoke" in run_name:
        epoch = int(record.get("epoch") if is_live else record.get("last_epoch") or 0)
        completed_epochs = max(0, epoch - 9500)
        recovery_run_dir = RUNS_ROOT / run_name
        normal_marker_epoch = _normal_completion_marker_epoch(recovery_run_dir)
        complete = not is_live and epoch == 9600 and normal_marker_epoch == 9600
        completion_anomaly = not is_live and epoch >= 9600 and not complete
        attestation_path = recovery_run_dir / ".navrl_v2_recovery_eval_pass.json"
        attested = False
        attestation_errors: List[str] = []
        if complete:
            attestation_errors = _validate_recovery_attestation(recovery_run_dir)
            attested = not attestation_errors
        attestation_present = attestation_path.is_file()
        bars = int(
            round(record.get("n_bars_active") or record.get("last_n_bars_active") or 130)
        )
        return {
            "subtitle": "2026-08-01 · transactional PPO recovery · fixed-130 safety smoke",
            "headline": (
                f"Recovery smoke is running: {completed_epochs}/100 epochs at {bars} bars."
                if is_live
                else (
                    "Recovery smoke and its hash-bound held-out gate passed this snapshot's verification."
                    if attested
                    else "Recovery reached its epoch budget without the exact normal-completion marker; curriculum remains blocked."
                    if completion_anomaly
                    else "The recovery attestation failed artifact or contract verification; curriculum remains blocked."
                    if complete and attestation_present
                    else "The 100-epoch recovery smoke completed; held-out evaluation is the next gate."
                    if complete
                    else f"Recovery smoke stopped early after {completed_epochs}/100 epochs."
                )
            ),
            "summary": (
                "This stage starts from the SHA-256-pinned ep9500 last-known-good policy, freezes "
                "density at 130 bars, preserves Adam moments, uses LR 5e-6, and atomically rolls "
                "back actor and central critic together whenever immutable-behavior KL exceeds "
                "0.04 or a model, RMS, optimizer, scaler, loss, or output becomes non-finite. "
                "Promotion evidence is recomputed from the real 100-epoch TensorBoard window and "
                "an evaluator receipt bound to an immutable checkpoint snapshot, nonce, log, and "
                "measured physics under main/base_sim/128 envs, full 6–28 m goals, and final FOV."
            ),
            "active_experiment": {
                **record,
                "is_live": is_live,
                "epoch": epoch,
                "max_epochs": 9600,
                "bars": bars,
                "selector": "cluster_sector",
                "cluster_gap_m": 0.45,
                "sectors": 8,
                "arena_xy_m": 40,
                "arena_z_m": 3,
                "recovery_checkpoint_epoch": 9500,
                "recovery_lr": 5e-6,
                "rollback_kl_gate": 0.04,
                "recovery_attestation_valid": attested,
                "recovery_attestation_errors": attestation_errors,
                "recovery_normal_completion_marker_valid": complete,
            },
            "milestones": [
                {
                    "label": "RECOVERY SMOKE",
                    "value": f"{completed_epochs} / 100",
                    "detail": (
                        "fixed 130 bars; exact normal marker epoch=9600"
                        if complete
                        else "fixed 130 bars; normal marker epoch=9600 required"
                    ),
                    "state": "pass" if complete else ("active" if is_live else "warn"),
                },
                {
                    "label": "SOURCE",
                    "value": "ep9500",
                    "detail": "audited LKG; SHA-256 pinned",
                    "state": "pass",
                },
                {
                    "label": "PPO COMMIT",
                    "value": "ATOMIC",
                    "detail": "actor+central model/RMS/Adam/scaler/output/loss + KL audit",
                    "state": "pass",
                },
                {
                    "label": "NEXT",
                    "value": (
                        "CURRICULUM"
                        if attested
                        else "DIAGNOSE EXIT"
                        if completion_anomaly
                        else "RE-EVALUATE"
                        if complete and attestation_present
                        else "HELD-OUT EVAL"
                        if complete
                        else "FINISH SMOKE"
                    ),
                    "detail": "curriculum requires the checkpoint-bound evaluation PASS",
                    "state": "pass" if attested else "active",
                },
            ],
            "comparison": [],
            "gates": [
                {
                    "label": "100 recovery epochs + normal exit",
                    "value": (
                        "PASS · marker epoch=9600"
                        if complete
                        else "INVALID · marker missing/mismatched"
                        if completion_anomaly
                        else "PENDING"
                    ),
                },
                {"label": "post-update transaction", "value": "ENFORCED · KL 0.04 + finite state"},
                {
                    "label": "held-out 130-bar attestation",
                    "value": (
                        "PASS"
                        if attested
                        else "INVALID · artifact/contract mismatch"
                        if complete and attestation_present
                        else "PENDING · 2,049 episodes"
                    ),
                },
                {
                    "label": "curriculum resume",
                    "value": (
                        "VERIFIED IN THIS SNAPSHOT · LAUNCHER RECHECKS"
                        if attested
                        else "BLOCKED UNTIL HASH-BOUND PASS"
                    ),
                },
            ],
            "decision": (
                "The ep9600 evidence is verified in this static snapshot. Resume only through the safe launcher, which re-verifies the live artifacts before training."
                if attested
                else "Do not evaluate or resume: the run reached epoch 9600 without the exact normal-completion marker. Diagnose the exit and rerun the smoke."
                if completion_anomaly
                else "Do not resume: the saved attestation, checkpoint, or held-out artifact failed strict verification. Re-run the canonical recovery evaluation."
                if complete and attestation_present
                else "Evaluate the final ep9600 checkpoint at 130 bars on the pinned U[0.3,1.5] target "
                "distribution. Resume the curriculum only if KL, saturation, capture, and crash remain healthy."
                if complete
                else "Let this fixed-density stage finish; do not use an intermediate checkpoint to resume the curriculum."
            ),
        }

    if (
        not is_live
        and run_name.startswith("ppo_260731_2012_")
    ):
        return {
            "subtitle": "2026-08-01 · v2 PPO/FOV/provenance fixes audited · recovery smoke pending",
            "headline": "The 145-bar run did not hit a proven density ceiling; its PPO actor update collapsed.",
            "summary": (
                "The run fail-stopped at epoch 10,836 after KL rose to 2.69 and capture fell to 0%. "
                "The old gate skipped later minibatches but could not undo an already committed update, "
                "then rebased its reference onto that rejected policy. PPO epochs are now transactional: "
                "actor and asymmetric central-critic model, RMS, Adam and AMP state are restored against "
                "immutable rollout-policy KL. A forced rollback restored every actor/critic tensor, value "
                "statistic and Adam moment/step exactly; a normal update audited at KL 0.0063 under the 0.04 gate."
                " The recovery gate now independently rebuilds its result from the exact checkpoint, "
                "held-out JSON, evaluator receipt, and TensorBoard window; it binds an immutable "
                "checkpoint snapshot, nonce, log and measured physics. The previously inert general-spawn "
                "FOV curriculum now aligns only initial yaw while keeping target directions unbiased. "
                "Rollback counters/LR are durable across fail-stop resumes."
            ),
            "active_experiment": {
                **record,
                "is_live": False,
                "epoch": record.get("last_epoch"),
                "max_epochs": 30000,
                "bars": 145,
                "selector": "cluster_sector",
                "cluster_gap_m": 0.45,
                "sectors": 8,
                "arena_xy_m": 40,
                "arena_z_m": 3,
                "recovery_checkpoint_epoch": 9500,
                "recovery_bars": 130,
                "recovery_lr": 5e-6,
                "rollback_kl_gate": 0.04,
                "heldout_capture_130": 0.7470817120622568,
                "heldout_crash_130": 0.2529182879377432,
                "heldout_episodes": 257,
            },
            "milestones": [
                {
                    "label": "ROOT CAUSE",
                    "value": "ACTOR UPDATE",
                    "detail": "moving KL reference + no model/Adam/RMS rollback",
                    "state": "pass",
                },
                {
                    "label": "FORCED ROLLBACK",
                    "value": "EXACT",
                    "detail": "actor+central model/RMS/Adam exact; actor LR backoff only",
                    "state": "pass",
                },
                {
                    "label": "130-BAR HEALTH",
                    "value": "74.71%",
                    "detail": "257 held-out episodes; crash 25.29%; OOB 0",
                    "state": "pass",
                },
                {
                    "label": "NEXT",
                    "value": "100 EPOCH",
                    "detail": "fixed-130 recovery smoke before curriculum resume",
                    "state": "active",
                },
            ],
            "comparison": [
                {
                    "label": "ep9500 LKG + one safe update · held-out",
                    "bars": 130,
                    "capture": 0.7470817120622568,
                    "unique": None,
                    "verdict": "257 episodes; crash 25.29%; timeout 0%",
                },
                {
                    "label": "ep10824+ collapsed actor · training tail",
                    "bars": 145,
                    "capture": 0.0,
                    "unique": None,
                    "verdict": "invalid for density-ceiling inference; crash 100%",
                },
            ],
            "gates": [
                {"label": "epoch transaction", "value": "PASS · actor+central/RMS/Adam/AMP atomic restore"},
                {"label": "post-update KL", "value": "PASS · 0.0063 < 0.04"},
                {"label": "evaluation contract", "value": "PASS · receipt/snapshot + measured sim/goal/FOV"},
                {"label": "145-bar ceiling", "value": "UNRESOLVED · contaminated by actor drift"},
            ],
            "decision": (
                "Resume from last_gen_ppo_ep_9500, not ep10800. Run the fixed-130 100-epoch safety "
                "smoke at 5e-6 first; only its final checkpoint with a 2,049-episode hash-bound "
                "held-out PASS attestation may re-enter the density curriculum. "
                "Re-measure 145 bars before claiming a representation or geometric ceiling."
            ),
        }

    promotions = _live_density_promotions()
    bars = int(
        round(record.get("n_bars_active") or record.get("last_n_bars_active") or 70)
    )
    capture_tail = (
        record.get("captured_rate")
        if is_live
        else record.get("last_captured_rate")
    )
    epoch = record.get("epoch") if is_live else record.get("last_epoch")
    run_state = "running" if is_live else "paused snapshot"
    comparison = [
        {
            "label": f"{item['source']} → {item['target']} promotion",
            "bars": item["target"],
            "capture": item["capture"],
            "unique": None,
            "verdict": f"PASS over {item['episodes']:,} episodes",
        }
        for item in promotions
    ]
    comparison.append(
        {
            "label": "current stage · rolling tail" if is_live else "latest stopped epoch",
            "bars": bars,
            "capture": capture_tail,
            "unique": None,
            "verdict": (
                "live diagnostic only; not the 16,384-episode promotion gate"
                if is_live
                else "stopped-run diagnostic; not a held-out result"
            ),
        }
    )
    promotion_text = " → ".join(
        [str(promotions[0]["source"])] + [str(item["target"]) for item in promotions]
    ) if promotions else str(bars)
    gate_captures = ", ".join(f"{item['capture'] * 100:.1f}%" for item in promotions)
    tail_text = f"{capture_tail * 100:.1f}%" if capture_tail is not None else "pending"
    return {
        "subtitle": f"2026-07-31 · v2 search-arena density curriculum · {run_state}",
        "headline": (
            f"Task-v2 training is live at {bars} bars after {len(promotions)} promotions."
            if is_live
            else f"Task-v2 training is paused at epoch {epoch}, {bars} bars."
        ),
        "summary": (
            f"The 40 m search-arena run has advanced {promotion_text}; completed promotion-window "
            f"capture values are {gate_captures or 'not yet available'}. The displayed capture "
            f"is {tail_text}, which is diagnostic only and must not be mistaken for the 16,384-episode "
            "promotion gate or a held-out result."
        ),
        "active_experiment": {
            **record,
            "is_live": is_live,
            "epoch": epoch,
            "max_epochs": record.get("max_epochs") or 30000,
            "bars": bars,
            "selector": "cluster_sector",
            "cluster_gap_m": 0.45,
            "sectors": 8,
            "arena_xy_m": 40,
            "arena_z_m": 3,
            "density_final": 300,
            "density_step": 15,
            "density_threshold": 0.70,
            "density_window_episodes": 16384,
        },
        "milestones": [
            {
                "label": "DENSITY",
                "value": f"{bars} / 300",
                "detail": f"self-paced +15 bars; chain {promotion_text}",
                "state": "active",
            },
            {
                "label": "PROMOTIONS",
                "value": str(len(promotions)),
                "detail": f"window capture {gate_captures or 'pending'}",
                "state": "pass" if promotions else "active",
            },
            {
                "label": "PROVENANCE",
                "value": "9 / 9",
                "detail": "arena, placement, episode and full-width bar-band contract saved",
                "state": "pass",
            },
            {
                "label": "1650 Ti · 4GB",
                "value": "CONDITIONAL",
                "detail": "64-env path fits the batch; real-card 8-epoch smoke still required",
                "state": "warn",
            },
        ],
        "comparison": comparison,
        "gates": [
            {
                "label": "density curriculum",
                "value": f"{'RUNNING' if is_live else 'PAUSED'} · {promotion_text}",
            },
            {"label": "collapse safety", "value": "PASS · atomic PPO rollback + NaN/Inf fail-fast"},
            {"label": "evaluation contract", "value": "PASS · z/gap/touch/bar-band enforced"},
            {"label": "1650 Ti", "value": "SMOKE REQUIRED · recommend free VRAM ≥3.6–3.7 GiB"},
        ],
        "decision": (
            "Keep the current process running, but do not interpret the rolling tail as a promotion "
            "decision." if is_live else
            "The run is paused. Validate the revised speed ramp and density dwell state in a short "
            "smoke before starting the next full training run."
        ) + " Treat the 4GB launcher as provisional until it passes an actual 1650 Ti smoke run.",
    }


def _research_update(
    active: Optional[Dict[str, Any]], latest: Optional[Dict[str, Any]]
) -> Dict[str, Any]:
    record = active or latest
    if _is_v2(record):
        return _v2_search_update(record, is_live=bool(active))
    record = record or {}
    experiment = {
        "is_live": bool(active),
        "max_epochs": 13800,
        "bars": int(record.get("n_bars_active") or record.get("last_n_bars_active") or 100),
        "run": record.get(
            "run", "ppo_260731_0343_navrl_corridor6-fixed100-s1"
        ),
        "epoch": record.get("epoch") if active else record.get("last_epoch", 13800),
        "capture_tail": active["captured_rate"] if active else None,
        "crash_tail": active["crash_rate"] if active else None,
        "selector": "cluster_sector",
        "corridor_tokens": 6,
        "cluster_gap_m": 0.45,
        "sectors": 8,
        "unique_bars": 4.9,
        "duplicate_tokens": 0.2,
    }
    return {
        "subtitle": "2026-07-31 · corridor-token fixed-100 A/B complete",
        "headline": "Corridor tokens reduced bar contact, but missed the advancement gate.",
        "summary": (
            "The K=6 free-gap representation passed its physical geometry and checkpoint-schema "
            "tests, then completed an 800-epoch fixed-100 adaptation. On 4,003 held-out episodes, "
            "ep13800 reached 66.10% capture versus 64.53% for ep12500 (+1.57 percentage points; "
            "95% CI [-0.51,+3.66]) while bar contact fell from 33.18% to 32.25%. The collision "
            "signal is encouraging, but the preregistered 68% and +3pp gates were not met."
        ),
        "active_experiment": experiment,
        "milestones": [
            {
                "label": "GEOMETRY",
                "value": "PASS",
                "detail": "center clear 100%; bound-on-bar 97.8%; width sanity 98.8%",
                "state": "pass",
            },
            {
                "label": "OBS SCHEMA",
                "value": "898 → 946",
                "detail": "explicit six-corridor append and contract-safe checkpoint expansion",
                "state": "pass",
            },
            {
                "label": "HELD-OUT CAPTURE",
                "value": "66.10%",
                "detail": "4,003 episodes at 100 bars; pilot target was 68%",
                "state": "warn",
            },
            {
                "label": "BAR CONTACT",
                "value": "32.25%",
                "detail": "down from the immutable ep12500 baseline of 33.18%",
                "state": "pass",
            },
        ],
        "comparison": [
            {
                "label": "ep12500 · cluster-sector baseline",
                "bars": 100,
                "capture": 0.6453,
                "unique": 4.9,
                "verdict": "immutable baseline; bar contact 33.18%",
            },
            {
                "label": "ep13100 · corridor screen",
                "bars": 100,
                "capture": 0.6420,
                "unique": 4.9,
                "verdict": "no early gain; bar contact 33.75%",
            },
            {
                "label": "ep13450 · corridor screen",
                "bars": 100,
                "capture": 0.5993,
                "unique": 4.9,
                "verdict": "mid-run regression; bar contact 38.15%",
            },
            {
                "label": "ep13800 · corridor confirm",
                "bars": 100,
                "capture": 0.6610,
                "unique": 4.9,
                "verdict": "4,003 episodes; bar contact 32.25%",
            },
        ],
        "gates": [
            {"label": "capture target", "value": "FAIL · 66.10% < 68%"},
            {"label": "minimum gain", "value": "FAIL · +1.57pp < +3pp"},
            {"label": "bar contact", "value": "PASS · 32.25% < 33.18%"},
            {"label": "significance", "value": "INCONCLUSIVE · 95% CI crosses zero"},
        ],
        "decision": (
            "Freeze corridor6 as a partial/negative result and do not buy more epochs with the same "
            "representation. Keep density=100 and goal distance=4..16 fixed for the next diagnostic, "
            "then test whether two depth layers per sector preserve the front/back surfaces that one "
            "corridor token compresses away."
        ),
    }


def _corridor_token_plan() -> Dict[str, Any]:
    return {
        "title": "Corridor token",
        "subtitle": "completed pilot · physical geometry passed, policy gate failed",
        "definition": (
            "A corridor token is a compact description of one locally traversable opening between "
            "obstacles. The current obstacle token says where a bar surface is; a corridor token says "
            "where the drone may fit, how wide the opening is, and how far that opening remains clear."
        ),
        "why_now": (
            "The geometry was real and the actor consumed it safely: K=6 expanded the observation "
            "from 898 to 946 dimensions without changing historical offsets. It reduced bar contact "
            "by 0.93pp, but capture improved only 1.57pp and its confidence interval crossed zero. "
            "That rules out simply extending this same run; the next representation must preserve "
            "more depth structure rather than add more copies of the same local gap summary."
        ),
        "current": {
            "label": "Obstacle token · current",
            "question": "Where are the nearest bars?",
            "fields": ["bearing", "range", "relative geometry", "valid mask"],
            "weakness": "The policy must infer the opening between bars and whether the drone fits.",
        },
        "proposed": {
            "label": "Corridor token · tested",
            "question": "Where can the drone pass?",
            "fields": [
                "gap center bearing",
                "usable width",
                "left/right clearance",
                "clear depth or TTC",
                "valid mask",
            ],
            "weakness": "Local affordance only; it is not a full planner and still needs policy choice.",
        },
        "steps": [
            {
                "id": "P0",
                "title": "Freeze evidence",
                "detail": "Keep ep12500 and ep13000 plus the fixed 100-bar four-speed evaluation as immutable baselines.",
                "state": "done",
            },
            {
                "id": "P1",
                "title": "Geometry-only diagnostic",
                "detail": "Physical probe passed: center clearance 100%, bar-boundary 97.8%, width sanity 98.8%.",
                "state": "done",
            },
            {
                "id": "P2",
                "title": "Contract-safe input",
                "detail": "Expanded 898→946 dimensions, selectively initialized the new projection, and recorded schema provenance.",
                "state": "done",
            },
            {
                "id": "P3",
                "title": "Short fixed-100 A/B",
                "detail": "Completed 800 epochs and 4,003 held-out episodes: 66.10% capture, 32.25% bar contact.",
                "state": "done",
            },
        ],
        "pilot_gate": (
            "FAIL: capture 66.10% < 68% and gain +1.57pp < +3pp; bar-contact reduction passed. "
            "Do not extend corridor6. Advance to a separately gated two-depth-layer diagnostic."
        ),
    }


def _is_v2(record: Optional[Dict[str, Any]]) -> bool:
    """True when the run being described is a task-v2 search-arena run.

    Takes whichever record the dashboard is showing (active run, or the latest finished one when
    nothing is training) -- keying only on `active` made the page fall back to v1 geometry and the
    v1 density denominator whenever training was stopped.
    """
    if not record:
        return False
    run_name = str(record.get("run") or "").lower()
    return any(
        marker in run_name
        for marker in ("v2-search", "v2-recover", "v2-ttc", "navrl_v2-")
    )


def _arena_geometry(active: Optional[Dict[str, Any]]) -> Dict[str, Any]:
    """Real geometry of the task the dashboard is describing, for the 3D arena panel.

    The panel used to hardcode v1 (24 m, bars in x 0.13..0.96, 2 m tall, relax placement); with
    v2 running that silently drew the wrong arena at the wrong density.
    """
    if _is_v2(active):
        return {
            "arena_xy_m": 40,
            "arena_z_m": 3,
            "bar_x_min_ratio": 0.0,
            "bar_x_max_ratio": 1.0,
            "bar_height_m": 3,
            "placement_mode": "navrl_band",
            "placement_touch_m": 0.4,
            "placement_gap_m": 1.6,
            "bars_min": 70,
            "bars_slider_min": 10,
            "bars_max": 300,
            "episode_len_steps": 600,
            "goal_dist_m": [6, 28],
            "target_speed_m": [0.3, 1.5],
            "label": "40×40×3 m · full-width bar band · 3 m bars (no fly-over)",
        }
    return {
        "arena_xy_m": 24,
        "arena_z_m": 3,
        "bar_x_min_ratio": 0.13,
        "bar_x_max_ratio": 0.96,
        "bar_height_m": 2,
        "placement_mode": "random",
        "placement_touch_m": 0.4,
        "placement_gap_m": 1.6,
        "bars_min": 25,
        "bars_slider_min": 10,
        "bars_max": 150,
        "episode_len_steps": 300,
        "goal_dist_m": [4, 16],
        "target_speed_m": [0.0, 1.5],
        "label": "24×24×3 m · bar band x 0.13–0.96 · 2 m bars",
    }


def _placement_area_m2(active: Optional[Dict[str, Any]]) -> float:
    """Obstacle-placement area used as the density denominator.

    v1: 24 m arena, bars confined to x in 0.13..0.96 -> 0.83*24*24 = 478 m^2.
    v2: 40 m arena with the band widened to full width -> the whole 1600 m^2 footprint.
    """
    return 1600.0 if _is_v2(active) else 478.0


def _success_criteria(active: Optional[Dict[str, Any]]) -> Dict[str, Any]:
    """What 'working' means for this project, stated explicitly.

    PPO's own scalars (a_loss, c_loss, entropy, kl, explained_variance) describe whether the
    OPTIMIZER is healthy, not whether the TASK is solved. They are reported as guardrails only.
    Task success is measured exclusively by held-out capture rate at a stated density.
    """
    return {
        "headline": "Success is held-out capture rate at a stated density -- never a PPO loss curve.",
        "primary": {
            "metric": "capture rate",
            "definition": "fraction of held-out episodes ending with the pursuer within 0.5 m of the target",
            "measured_by": "eval_navrl_v2_density_sweep.sh -- a frozen checkpoint replayed on episodes it never trained on",
            "why": (
                "Training-time capture mixes densities while the curriculum ramps, so it cannot be "
                "compared across epochs. A held-out sweep fixes the density per cell."
            ),
        },
        "secondary": [
            {
                "metric": "crash rate",
                "definition": "episodes ending in a bar/wall collision",
                "role": "the dominant failure mode; a capture gain paid for with more crashes is not progress",
            },
            {
                "metric": "timeout rate",
                "definition": "episodes reaching the step limit without capture",
                "role": "separates 'could not find the target' from 'found it and crashed'",
            },
            {
                "metric": "bar contact rate",
                "definition": "episodes touching an obstacle at any point",
                "role": "sensitivity to representation changes even when capture is flat",
            },
        ],
        "not_success_metrics": [
            {
                "metric": "mean reward",
                "why": (
                    "Reward falls when the density curriculum promotes, so a declining curve during "
                    "an active curriculum measures rising task difficulty, not a worsening policy. "
                    "Only compare reward WITHIN one fixed density."
                ),
            },
            {
                "metric": "ppo/a_loss, ppo/c_loss",
                "why": (
                    "PPO's surrogate and value losses are optimizer diagnostics on a moving target "
                    "distribution. They do not decrease monotonically in a healthy run and carry no "
                    "task-performance meaning."
                ),
            },
            {
                "metric": "ppo/entropy",
                "why": (
                    "Entropy is not task success. For this squashed Gaussian, however, a sudden plunge "
                    "far below its normal range is a latent-tanh saturation alarm and must be read with KL."
                ),
            },
            {
                "metric": "ppo/kl, ppo/explained_variance",
                "why": (
                    "Guardrails only. KL flags too-large policy steps (rollback at 0.04); explained "
                    "variance flags a critic that has stopped tracking returns. Both being healthy "
                    "is necessary, never sufficient."
                ),
            },
        ],
        "curriculum_gate": {
            "what": "density promotion gate (a TRAINING control, not a result)",
            "rule": (
                "promote +15 bars when capture over a 16,384-episode window clears the threshold, "
                "using the measured knot schedule 0.82@70, 0.77@85, 0.72@100, 0.70@115+, and only "
                "after at least 1,000 PPO epochs at the current density"
            ),
            "why_ramped": (
                "A flat threshold strands the curriculum forever once the achievable capture ceiling "
                "falls below it -- the failure mode behind the v1 100-bar plateau. Demand mastery "
                "where the task is easy; relax where it is hard."
            ),
            "caution": (
                "The rolling 50-epoch tail shown in Live is a diagnostic, not this gate. Never quote "
                "the tail as a promotion or publication number."
            ),
        },
        "checkpoint_rule": (
            "Evaluate a curriculum run with last_gen_ppo_ep_*.pth. gen_ppo.pth is the best-REWARD "
            "checkpoint, and reward peaks at low density, so it is a sparse-density policy that "
            "scores near 15% when replayed at high density."
        ),
    }


def build_snapshot() -> Dict[str, Any]:
    status = json.loads(STATUS_PATH.read_text(encoding="utf-8"))
    generated_at = datetime.now(timezone.utc).isoformat()
    csv_paths = sorted(RUNS_ROOT.glob("*/aerial_run/epoch_metrics.csv"))
    training_live = _training_process_exists()
    active_path = None
    if training_live:
        unfinished = [
            path for path in csv_paths if not (path.parent / "run_summary.json").is_file()
        ]
        if unfinished:
            active_path = max(unfinished, key=lambda path: path.stat().st_mtime)

    summaries = []
    for path in csv_paths:
        summaries.append(_summarize_run(path, is_active=(path == active_path)))
    summaries.sort(key=lambda item: item["run"])

    active = None
    if active_path is not None:
        active_summary = next(item for item in summaries if item["run"] == active_path.parents[1].name)
        active = _active_record(
            active_summary,
            active_path,
            max_epochs=_live_training_max_epochs(),
        )

    finalized = [item for item in summaries if item["run"] != (active or {}).get("run")]
    reportable_finalized = [
        item for item in finalized if not _is_smoke_run(str(item.get("run", "")))
    ]
    latest = max(
        reportable_finalized or finalized,
        key=lambda item: (item.get("finalized_at") or "", item["run"]),
        default=None,
    )
    research_update = _research_update(active, latest)
    experiment = research_update.get("active_experiment", {})
    if experiment.get("recovery_attestation_valid") is True:
        # GitHub Pages is a static snapshot, not a live filesystem verifier.  Bind the displayed
        # PASS to this generation time; the training launcher independently rechecks the artifacts.
        experiment["recovery_attestation_verified_at"] = generated_at
    status.update(
        {
            "generated_at": generated_at,
            "repo": str(ROOT),
            "n_runs": len(summaries),
            "active_run": active,
            "latest_run": latest,
            "runs": summaries,
            "research_update": research_update,
            "corridor_token": _corridor_token_plan(),
            "success_criteria": _success_criteria(active or latest),
            "placement_area_m2": _placement_area_m2(active or latest),
            "arena_geometry": _arena_geometry(active or latest),
        }
    )
    status.setdefault("density_curves", {})[
        "corrected_chirality_density_curve"
    ] = _corrected_density_curve()
    return status


def write_snapshot(status: Dict[str, Any]) -> None:
    STATUS_PATH.write_text(
        json.dumps(status, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    html = HTML_PATH.read_text(encoding="utf-8")
    compact = json.dumps(status, ensure_ascii=False, separators=(",", ":"))
    pattern = re.compile(
        r'(<script id="fallback" type="application/json">).*?(</script>)',
        flags=re.DOTALL,
    )
    html, count = pattern.subn(rf"\g<1>{compact}\g<2>", html, count=1)
    if count != 1:
        raise RuntimeError("could not locate exactly one dashboard fallback JSON block")
    HTML_PATH.write_text(html, encoding="utf-8")


def main() -> int:
    status = build_snapshot()
    write_snapshot(status)
    active = status.get("active_run")
    print(
        "[status] synchronized "
        f"{status['n_runs']} runs; active={active['run'] if active else 'none'}; "
        f"generated_at={status['generated_at']}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
