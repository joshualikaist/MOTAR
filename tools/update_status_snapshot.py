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
import sys
from typing import Any, Dict, Iterable, List, Optional


ROOT = Path(__file__).resolve().parents[1]
RL_ROOT = ROOT / "aerial_gym/rl_training/rl_games"
RUNS_ROOT = RL_ROOT / "runs"
STATUS_PATH = ROOT / "docs/status/status.json"
HTML_PATH = ROOT / "docs/status/index.html"
CORRECTED_CURVE_PATH = ROOT / "results/corrected_chirality_density_curve.csv"
RECOVERY_ATTESTATION_VERIFIER_PATH = ROOT / "tools/navrl_v2_recovery_attestation.py"
LIMIT_AUDIT_PATH = ROOT / "results/navrl_v2_ep24000_limit_audit.json"
CAUSAL_1TO3_PATH = ROOT / "results/navrl_v2_ep24000_causal_1to3/summary.json"
FIXED_SPEED_PATH = ROOT / "results/navrl_v2_ep24000_fixed_speed/summary.json"
FORGETTING_PATH = ROOT / "results/navrl_v2_ep19100_vs_ep24000_forgetting/summary.json"
SPEED_GOVERNOR_SCREEN_PATH = ROOT / "results/navrl_v2_ep24000_speed_governor_screen/summary.json"
RISKCAP_SCREEN_PATH = ROOT / "results/navrl_v2_ep24000_riskcap_seed44_screen/summary.json"
RISKCAP_POST_PATH = ROOT / "results/navrl_v2_riskcap_postadapt/summary.json"
TTC_1650_PATH = ROOT / "results/v2_ttc_ab_1650ti.csv"
MAIN_TTC_RESULT_ROOT = ROOT / "results"

_MAIN_TTC_SOURCE_SHA256 = (
    "82f7978b42d9d9e95adcc638a40ae85fb3736fd897ae39cb8aa8333be39cf23f"
)
_MAIN_TTC_BASELINE_SHA256 = (
    "169ddcddb83c9d74df5c79252274660bc9c52e32d7d5144d325698e32b1d9b08"
)
_RISKCAP_TRAINED_SHA256 = (
    "f702213936601860995cf61dcc570247e72543b1976e3716055cd8ec5593ad40"
)

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
        "trained_max_bars": 85,
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


def _main_ttc_result(profile: str, arm: str) -> Optional[Dict[str, Any]]:
    """Load one hash/contract-checked held-out cell for the ep24000 TTC A/B."""

    path = (
        MAIN_TTC_RESULT_ROOT
        / f"navrl_v2_ep24000_ttc_{profile}_{arm}"
        / "205bars.json"
    )
    if not path.is_file():
        return None
    payload = json.loads(path.read_text(encoding="utf-8"))
    condition = payload.get("condition", {})
    contract = payload.get("v2_evaluation_contract", {})
    outcome = payload.get("outcome", {})
    expected_selector = "cluster_sector" if arm == "baseline" else "ttc_sector"
    expected_profile = "main" if profile == "main" else "4gb"
    actual = int(payload.get("actual_episodes", -1))
    expected_sha = _MAIN_TTC_BASELINE_SHA256 if (profile, arm) == ("main", "baseline") else None
    problems = []
    if payload.get("schema_version") != 1:
        problems.append("schema_version must be 1")
    if int(payload.get("requested_episodes", -1)) != 2049 or actual < 2049:
        problems.append("requires requested/actual episodes >= 2049")
    if int(condition.get("bars", -1)) != 205 or int(condition.get("seed", -1)) != 42:
        problems.append("requires bars=205 and seed=42")
    if condition.get("action_selection") != "deterministic":
        problems.append("requires deterministic actions")
    if condition.get("reflection_mode") != "original":
        problems.append("requires original reflection mode")
    if condition.get("target_speed_mode") != "uniform":
        problems.append("requires uniform target speed")
    if contract.get("obstacle_selector") != expected_selector:
        problems.append(f"requires selector={expected_selector}")
    if contract.get("runtime_profile") != expected_profile:
        problems.append(f"requires runtime_profile={expected_profile}")
    counts = sum(int(outcome.get(key, -actual - 1)) for key in ("captured", "crash", "timeout"))
    if counts != actual:
        problems.append("outcome counts do not sum to actual episodes")
    if expected_sha and payload.get("checkpoint_sha256") != expected_sha:
        problems.append("baseline checkpoint SHA-256 mismatch")
    if payload.get("evaluated_checkpoint_snapshot_sha256") != payload.get("checkpoint_sha256"):
        problems.append("evaluated snapshot SHA-256 mismatch")
    if problems:
        raise RuntimeError(f"invalid ep24000 TTC A/B result {path}: " + "; ".join(problems))
    try:
        display_path = str(path.relative_to(ROOT))
    except ValueError:
        display_path = str(path)
    return {
        "path": display_path,
        "checkpoint_sha256": payload["checkpoint_sha256"],
        "episodes": actual,
        "capture_rate": float(outcome["capture_rate"]),
        "crash_rate": float(outcome["crash_rate"]),
        "timeout_rate": float(outcome["timeout_rate"]),
        "bar_contact_rate": float(payload.get("crash_causes", {}).get("bar_contact", 0)) / actual,
    }


def _riskcap_screen_result() -> Optional[Dict[str, Any]]:
    if not RISKCAP_SCREEN_PATH.is_file():
        return None
    payload = json.loads(RISKCAP_SCREEN_PATH.read_text(encoding="utf-8"))
    rows = {row.get("tag"): row for row in payload.get("rows", [])}
    candidate = rows.get("riskcap") or {}
    baseline = rows.get("off") or {}
    checks = {
        "schema": payload.get("schema_version") == 1,
        "experiment": payload.get("experiment") == "ep24000_seed44_riskcap_screen",
        "seed": int(payload.get("heldout_seed", -1)) == 44,
        "sha": payload.get("source_checkpoint_sha256") == _MAIN_TTC_SOURCE_SHA256,
        "go": payload.get("adaptive_go") is True and payload.get("selected_tag") == "riskcap",
        "candidate": candidate.get("screen_pass") is True and int(candidate.get("episodes", 0)) >= 2049,
        "baseline": int(baseline.get("episodes", 0)) >= 2049,
    }
    failed = [name for name, passed in checks.items() if not passed]
    if failed:
        raise RuntimeError(f"invalid riskcap screen {RISKCAP_SCREEN_PATH}: {', '.join(failed)}")
    return {"payload": payload, "baseline": baseline, "candidate": candidate}


def _speed_governor_screen_result() -> Optional[Dict[str, Any]]:
    if not SPEED_GOVERNOR_SCREEN_PATH.is_file():
        return None
    payload = json.loads(SPEED_GOVERNOR_SCREEN_PATH.read_text(encoding="utf-8"))
    rows = {row.get("tag"): row for row in payload.get("rows", [])}
    checks = {
        "schema": payload.get("schema_version") == 1,
        "experiment": payload.get("experiment") == "ep24000_speed_governor_screen",
        "sha": payload.get("source_checkpoint_sha256") == _MAIN_TTC_SOURCE_SHA256,
        "decision": payload.get("adaptive_go") is False and payload.get("selected_tag") is None,
        "cells": set(rows) == {"off", "fixed2p0", "fixed1p5", "clearance", "ttc"},
        "episodes": all(int(row.get("episodes", 0)) >= 2049 for row in rows.values()),
    }
    failed = [name for name, passed in checks.items() if not passed]
    if failed:
        raise RuntimeError(
            f"invalid speed-governor screen {SPEED_GOVERNOR_SCREEN_PATH}: {', '.join(failed)}"
        )
    return {"payload": payload, "rows": rows}


def _riskcap_post_result() -> Optional[Dict[str, Any]]:
    if not RISKCAP_POST_PATH.is_file():
        return None
    payload = json.loads(RISKCAP_POST_PATH.read_text(encoding="utf-8"))
    uniform = {row.get("tag"): row for row in payload.get("uniform_rows", [])}
    fixed = payload.get("fixed_speed_rows", [])
    checks = {
        "schema": payload.get("schema_version") == 1,
        "source_sha": payload.get("source_checkpoint_sha256") == _MAIN_TTC_SOURCE_SHA256,
        "trained_sha": payload.get("trained_checkpoint_sha256") == _RISKCAP_TRAINED_SHA256,
        "winner": payload.get("winner_kind") == "trained"
        and payload.get("winner_checkpoint_sha256") == _RISKCAP_TRAINED_SHA256,
        "gates": payload.get("mechanism_replication_pass") is True
        and payload.get("adaptation_pass") is True
        and payload.get("generalization_pass") is True,
        "uniform_cells": set(uniform)
        == {"uniform_off", "uniform_source_riskcap", "uniform_trained_riskcap"}
        and all(int(row.get("episodes", 0)) >= 2049 for row in uniform.values()),
        "fixed_cells": [float(row.get("target_speed_mps", -1)) for row in fixed] == [0.3, 0.9, 1.5]
        and all(row.get("direction_pass") is True for row in fixed),
    }
    failed = [name for name, passed in checks.items() if not passed]
    if failed:
        raise RuntimeError(f"invalid riskcap post-adaptation result: {', '.join(failed)}")
    return {"payload": payload, "uniform": uniform, "fixed": fixed}


def _v2_search_update(record: Dict[str, Any], *, is_live: bool) -> Dict[str, Any]:
    run_name = str(record.get("run", ""))
    if "v2-speedgov-ep24000-205bars-main-riskcap-s1" in run_name:
        complete_stop = _speed_governor_screen_result()
        if complete_stop is None:
            raise RuntimeError("riskcap training exists without the corrected five-cell screen")
        screen = _riskcap_screen_result()
        if screen is None:
            raise RuntimeError("riskcap training exists without its held-out authorization")
        baseline, candidate = screen["baseline"], screen["candidate"]
        epoch = int(record.get("epoch") if is_live else record.get("last_epoch") or 0)
        completed = not is_live and epoch >= 25000
        post_result = _riskcap_post_result()
        post = post_result["payload"] if post_result else None
        phase = (
            "TRAINING" if is_live else "FINAL PASS" if post and post.get("generalization_pass")
            else "FINAL FAIL" if post else "EVAL PENDING" if completed else "INCOMPLETE"
        )
        comparison = []
        if post:
            for tag, label in (
                ("uniform_off", "seed45 · canonical off"),
                ("uniform_source_riskcap", "seed45 · source + riskcap"),
                ("uniform_trained_riskcap", "seed45 · trained + riskcap"),
            ):
                row = post_result["uniform"][tag]
                comparison.append(
                    {
                        "label": label,
                        "bars": 205,
                        "capture": row["capture_rate"],
                        "unique": None,
                        "verdict": f"crash {row['crash_rate'] * 100:.2f}%; timeout {row['timeout_rate'] * 100:.2f}%",
                    }
                )
            for row in post_result["fixed"]:
                winner = row["winner"]
                comparison.append(
                    {
                        "label": f"final winner · fixed {row['target_speed_mps']:.1f} m/s",
                        "bars": 205,
                        "capture": winner["capture_rate"],
                        "unique": None,
                        "verdict": f"crash {winner['crash_rate'] * 100:.2f}%; {'PASS' if row['direction_pass'] else 'FAIL'} vs off",
                    }
                )
        else:
            comparison.extend(
                [
                    {
                        "label": "seed44 · canonical off",
                        "bars": 205,
                        "capture": baseline["capture_rate"],
                        "unique": None,
                        "verdict": f"crash {baseline['crash_rate'] * 100:.2f}%; timeout {baseline['timeout_rate'] * 100:.2f}%",
                    },
                    {
                        "label": "seed44 · minimum-intervention riskcap",
                        "bars": 205,
                        "capture": candidate["capture_rate"],
                        "unique": None,
                        "verdict": f"crash {candidate['crash_rate'] * 100:.2f}%; timeout {candidate['timeout_rate'] * 100:.2f}%",
                    },
                ]
            )
        adaptation_capture = post["adaptation_deltas"]["capture_rate"][0] if post else None
        mechanism_capture = post["mechanism_deltas"]["capture_rate"][0] if post else None
        mechanism_crash = post["mechanism_deltas"]["crash_rate"][0] if post else None
        return {
            "subtitle": f"2026-08-05 · R2b minimum-intervention riskcap · {phase.lower()}",
            "headline": (
                "Riskcap passed on an unseen seed and its matched adaptation is running."
                if is_live
                else "Riskcap training finished; unseen-seed post-adaptation evaluation is required."
                if completed and not post
                else f"Riskcap final generalization {'passed' if post and post.get('generalization_pass') else 'failed'}."
                if post
                else "Riskcap adaptation stopped before its matched budget."
            ),
            "summary": (
                f"On unseen seed45, sensor-only riskcap improved capture by {mechanism_capture * 100:+.2f} pp "
                f"and crash by {mechanism_crash * 100:+.2f} pp versus the frozen off policy. Exactly "
                f"1,000 adaptation epochs added {adaptation_capture * 100:+.2f} pp capture; the trained "
                "winner passed every seed46 fixed-speed direction check. The layer caps horizontal norm "
                "at 2.0 m/s inside 3 m, releases by 5 m, and never forces a stop."
                if post
                else f"The frozen ep24000 policy improved from {baseline['capture_rate'] * 100:.2f}% to "
                f"{candidate['capture_rate'] * 100:.2f}% capture on seed44 while crash fell "
                f"{(candidate['crash_rate'] - baseline['crash_rate']) * 100:+.2f} pp. The layer caps "
                "horizontal norm at 2.0 m/s inside 3 m and releases it to 3.535 m/s by 5 m; it never "
                "forces a stop. Training-tail values remain diagnostic until seed45/46 evaluation."
            ),
            "active_experiment": {
                **record,
                "is_live": is_live,
                "stage_status": phase,
                "epoch": epoch,
                "max_epochs": 25000,
                "warm_start_epoch": 24000,
                "adaptation_samples": max(0, epoch - 24000) * 32 * 128,
                "bars": 205,
                "density_curriculum": False,
                "selector": "cluster_sector",
                "cluster_gap_m": 0.45,
                "sectors": 8,
                "speed_governor_mode": "riskcap",
                "speed_governor_fixed_mps": 2.0,
                "speed_governor_slow_m": 3.0,
                "speed_governor_release_m": 5.0,
                "screen_pass": True,
                "post_evaluation_complete": post is not None,
                "generalization_pass": post.get("generalization_pass") if post else None,
                "winner_checkpoint_sha256": post.get("winner_checkpoint_sha256") if post else None,
                "heldout_capture": post_result["uniform"]["uniform_trained_riskcap"]["capture_rate"] if post else None,
                "heldout_crash": post_result["uniform"]["uniform_trained_riskcap"]["crash_rate"] if post else None,
                "heldout_timeout": post_result["uniform"]["uniform_trained_riskcap"]["timeout_rate"] if post else None,
                "heldout_episodes": post_result["uniform"]["uniform_trained_riskcap"]["episodes"] if post else None,
            },
            "milestones": [
                {"label": "SEED44 SCREEN", "value": "PASS", "detail": f"capture {candidate['capture_rate'] * 100:.2f}% · crash {candidate['crash_rate'] * 100:.2f}%", "state": "pass"},
                {"label": "NO DEADLOCK", "value": f"{candidate['near_stop_rate'] * 100:.2f}%", "detail": f"intervention {candidate['intervention_rate'] * 100:.1f}% · timeout {candidate['timeout_rate'] * 100:.2f}%", "state": "pass"},
                {"label": "ADAPTATION", "value": f"capture {adaptation_capture * 100:+.2f} pp" if post else f"{max(0, epoch - 24000)} / 1000", "detail": "trained winner · 4.096M samples" if post else "128 envs · 4.096M samples · LR 5e-6", "state": "pass" if post or completed else "active" if is_live else "warn"},
                {"label": "FINAL HELD-OUT", "value": "PASS · 3/3 speeds" if post else "PENDING", "detail": "seed45 uniform + seed46 fixed-speed" if not post else "winner=trained; 0.3/0.9/1.5 m/s all improved", "state": "pass" if post and post.get("generalization_pass") else "warn" if post else "active"},
            ],
            "comparison": comparison,
            "gates": [
                {"label": "R2 complete-stop governors", "value": "REJECTED · 16.59–23.57% timeout"},
                {"label": "R2b seed44", "value": f"PASS · capture {(candidate['capture_rate'] - baseline['capture_rate']) * 100:+.2f} pp; crash {(candidate['crash_rate'] - baseline['crash_rate']) * 100:+.2f} pp"},
                {"label": "PPO adaptation", "value": "RUNNING" if is_live else "COMPLETE" if completed else "INCOMPLETE"},
                {"label": "unseen seed45/46", "value": "PASS" if post and post.get("generalization_pass") else "FAIL" if post else "PENDING"},
            ],
            "decision": (
                "Finish exactly 1,000 epochs, then compare off/source-riskcap/trained-riskcap on seed45."
                if is_live
                else "Run eval_navrl_v2_riskcap_postadapt.sh on the final ep25000 checkpoint."
                if completed and not post
                else "Freeze ep25000 + riskcap as the navigation/control candidate. Do not extend fixed-density PPO or retune the gate post hoc; move next to learned-detector robustness."
            ),
            "speed_governor_screen": complete_stop["payload"],
            "riskcap_screen": screen["payload"],
            "riskcap_postadapt": post,
        }
    if "v2-ep24000-205bars-" in run_name:
        match = re.search(r"v2-ep24000-205bars-(main|4gb)-(baseline|ttc)-s1", run_name)
        if not match:
            raise RuntimeError(f"unrecognized ep24000 TTC A/B run name: {run_name}")
        profile, arm = match.groups()
        selector = "cluster_sector" if arm == "baseline" else "ttc_sector"
        budget_epochs = 1000 if profile == "main" else 2000
        envs = 128 if profile == "main" else 64
        final_epoch = 24000 + budget_epochs
        epoch = int(record.get("epoch") if is_live else record.get("last_epoch") or 0)
        completed = not is_live and epoch >= final_epoch
        baseline_result = _main_ttc_result(profile, "baseline")
        ttc_result = _main_ttc_result(profile, "ttc")
        current_result = baseline_result if arm == "baseline" else ttc_result
        gate_complete = baseline_result is not None and ttc_result is not None
        capture_delta = (
            ttc_result["capture_rate"] - baseline_result["capture_rate"]
            if gate_complete
            else None
        )
        crash_delta = (
            ttc_result["crash_rate"] - baseline_result["crash_rate"]
            if gate_complete
            else None
        )
        gate_pass = bool(
            gate_complete and capture_delta >= 0.020 and crash_delta <= -0.020
        )

        if is_live:
            phase_value = f"{arm.upper()} TRAINING"
            headline = f"The fixed-205 {arm} arm is running; held-out comparison remains locked."
        elif not completed:
            phase_value = f"{arm.upper()} INCOMPLETE"
            headline = f"The fixed-205 {arm} arm stopped before its matched sample budget."
        elif current_result is None:
            phase_value = f"{arm.upper()} EVAL PENDING"
            headline = (
                f"The fixed-205 {arm} adaptation finished normally; its independent held-out score is the next gate."
            )
        elif arm == "baseline" and ttc_result is None:
            phase_value = "TTC ARM READY"
            headline = "The fixed-205 baseline score is frozen; the sample-matched TTC arm may now start."
        elif gate_complete:
            phase_value = "TTC PASS" if gate_pass else "TTC REJECT"
            headline = (
                "The main fixed-205 TTC selector passed both preregistered gates."
                if gate_pass
                else (
                    "The current fixed-205 TTC bundle failed both preregistered gates and remains "
                    "below the canonical ep24000 policy."
                )
            )
        else:
            phase_value = "TTC EVAL PENDING"
            headline = "The fixed-205 TTC adaptation finished; matched held-out evaluation is now required."

        comparison = [
            {
                "label": "frozen source ep24000 · held-out",
                "bars": 205,
                "capture": 0.724390243902439,
                "unique": None,
                "verdict": "n=2,050; context anchor only, not the adapted A/B baseline",
            }
        ]
        for result_arm, result in (("baseline", baseline_result), ("ttc", ttc_result)):
            if result is not None:
                comparison.append(
                    {
                        "label": f"main A/B · {result_arm} held-out",
                        "bars": 205,
                        "capture": result["capture_rate"],
                        "unique": None,
                        "verdict": (
                            f"n={result['episodes']:,}; crash {result['crash_rate'] * 100:.2f}%; "
                            f"bar contact {result['bar_contact_rate'] * 100:.2f}%"
                        ),
                    }
                )
        if current_result is None:
            comparison.append(
                {
                    "label": f"{arm} training final epoch · diagnostic only",
                    "bars": 205,
                    "capture": (
                        record.get("captured_rate") if is_live else record.get("last_captured_rate")
                    ),
                    "unique": None,
                    "verdict": "small on-policy termination sample; never use for the A/B decision",
                }
            )

        return {
            "subtitle": (
                f"{'2026-08-05' if ttc_result else '2026-08-04' if baseline_result else '2026-08-03'} · "
                f"fixed-205 main selector A/B · {phase_value.lower()}"
            ),
            "headline": headline,
            "summary": (
                f"Both arms start from the SHA-256-pinned ep24000 policy, keep 205 bars, seed 1, "
                f"LR 5e-6, action noise and task dynamics fixed, and receive {budget_epochs:,} epochs × "
                f"32 × {envs} = 4,096,000 adaptation samples. The selector mode is the only launcher "
                "knob, but its current semantics bundle ranking with candidate FOV: cluster_sector uses "
                "240° while ttc_sector uses 360°. This therefore tests the deployable representation "
                "bundle, not a pure ranking-only intervention. "
                "Training-tail capture is diagnostic; adoption uses seed-42 deterministic/original "
                "held-out evaluation with at least 2,049 episodes per arm."
            ),
            "active_experiment": {
                **record,
                "is_live": is_live,
                "ab_experiment": True,
                "ab_arm": arm,
                "ab_phase": phase_value,
                "ab_gate_complete": gate_complete,
                "ab_gate_pass": gate_pass if gate_complete else None,
                "representation_bundle": True,
                "baseline_effective_fov_deg": 240,
                "ttc_effective_fov_deg": 360,
                "pure_ranking_isolated": False,
                "epoch": epoch,
                "max_epochs": final_epoch,
                "bars": 205,
                "selector": selector,
                "cluster_gap_m": 0.45,
                "sectors": 8,
                "density_curriculum": False,
                "arena_xy_m": 40,
                "arena_z_m": 3,
                "warm_start_epoch": 24000,
                "adaptation_samples": max(0, epoch - 24000) * 32 * envs,
                "source_checkpoint_sha256": _MAIN_TTC_SOURCE_SHA256,
                "final_checkpoint_sha256": (
                    current_result.get("checkpoint_sha256")
                    if completed and current_result
                    else _MAIN_TTC_BASELINE_SHA256
                    if (profile, arm, completed) == ("main", "baseline", True)
                    else None
                ),
                "training_tail_capture": (
                    record.get("captured_rate") if is_live else record.get("last_captured_rate")
                ),
                "training_tail_crash": (
                    record.get("crash_rate") if is_live else record.get("last_crash_rate")
                ),
                "heldout_complete": current_result is not None,
                "heldout_capture": current_result.get("capture_rate") if current_result else None,
                "heldout_crash": current_result.get("crash_rate") if current_result else None,
                "heldout_episodes": current_result.get("episodes") if current_result else None,
                "capture_delta": capture_delta,
                "crash_delta": crash_delta,
            },
            "milestones": [
                {
                    "label": "CURRENT ARM",
                    "value": phase_value,
                    "detail": (
                        f"selector={selector}; effective candidate FOV "
                        f"{'240' if selector == 'cluster_sector' else '360'}°; density fixed at 205"
                    ),
                    "state": (
                        "active"
                        if is_live or current_result is None
                        else "warn"
                        if gate_complete and not gate_pass
                        else "pass"
                        if completed
                        else "warn"
                    ),
                },
                {
                    "label": "TRAIN BUDGET",
                    "value": f"{max(0, epoch - 24000)} / {budget_epochs}",
                    "detail": f"{envs} envs · 4.096M matched samples",
                    "state": "pass" if completed else ("active" if is_live else "warn"),
                },
                {
                    "label": "HELD-OUT",
                    "value": (
                        f"{current_result['capture_rate'] * 100:.2f}%"
                        if current_result
                        else "PENDING"
                    ),
                    "detail": (
                        f"n={current_result['episodes']:,}; crash {current_result['crash_rate'] * 100:.2f}%"
                        if current_result
                        else "205 bars · seed42 · deterministic · original · ≥2,049 episodes"
                    ),
                    "state": "pass" if current_result else "active",
                },
                {
                    "label": "A/B DECISION",
                    "value": (
                        "PASS" if gate_pass else "REJECT"
                        if gate_complete
                        else "LOCKED"
                    ),
                    "detail": (
                        f"capture {capture_delta * 100:+.2f} pp · crash {crash_delta * 100:+.2f} pp"
                        if gate_complete
                        else "requires baseline + TTC held-out; +2pp capture and -2pp crash"
                    ),
                    "state": "pass" if gate_pass else ("warn" if gate_complete else "active"),
                },
            ],
            "comparison": comparison,
            "gates": [
                {
                    "label": "matched training budget",
                    "value": "PASS · 4.096M samples" if completed else "PENDING",
                },
                {
                    "label": "baseline held-out",
                    "value": "PASS · frozen" if baseline_result else "PENDING · TTC arm blocked",
                },
                {
                    "label": "capture delta",
                    "value": (
                        f"{'PASS' if capture_delta >= 0.020 else 'FAIL'} · {capture_delta * 100:+.2f} pp"
                        if gate_complete
                        else "PENDING · TTC - baseline ≥ +2.0pp"
                    ),
                },
                {
                    "label": "crash delta",
                    "value": (
                        f"{'PASS' if crash_delta <= -0.020 else 'FAIL'} · {crash_delta * 100:+.2f} pp"
                        if gate_complete
                        else "PENDING · TTC - baseline ≤ -2.0pp"
                    ),
                },
                {
                    "label": "canonical replacement floor",
                    "value": (
                        "PASS · beats ep24000 on capture and crash"
                        if gate_complete
                        and ttc_result["capture_rate"] >= 0.724390243902439
                        and ttc_result["crash_rate"] <= 0.25073170731707317
                        else "FAIL · keep ep24000 as deployment default"
                        if gate_complete
                        else "PENDING · capture ≥72.44%; crash ≤25.07%"
                    ),
                },
                {
                    "label": "experimental isolation",
                    "value": (
                        "BUNDLE · candidate FOV 240→360; pure ranking not isolated"
                    ),
                },
            ],
            "decision": (
                "Run the canonical held-out evaluation for the completed baseline and freeze its result. "
                "Do not start TTC training until that artifact exists."
                if completed and baseline_result is None and arm == "baseline"
                else "Run the TTC arm with the same launcher and sample budget; do not change any other knob."
                if baseline_result and arm == "baseline" and ttc_result is None
                else "Adopt TTC and move to replication only; do not resume an open-ended density curriculum."
                if gate_pass
                else (
                    "Close the current TTC-360 representation bundle as a negative result and keep ep24000 "
                    "as the canonical policy. Pure TTC ranking was not isolated: either screen a decoupled "
                    "TTC-240 arm first, or move to the preregistered R2 control-risk screen."
                )
                if gate_complete
                else "Finish the current arm, then evaluate its final checkpoint before any new training."
            ),
        }

    if (
        not is_live
        and run_name == "ppo_260802_0020_navrl_v2-recover-curriculum-continue-s1"
        and LIMIT_AUDIT_PATH.is_file()
    ):
        audit = json.loads(LIMIT_AUDIT_PATH.read_text(encoding="utf-8"))
        checkpoint = audit.get("checkpoint", {})
        if (
            audit.get("schema_version") != 1
            or audit.get("status") != "training-stopped-core-audit-complete"
            or checkpoint.get("epoch") != 24000
            or checkpoint.get("sha256")
            != "82f7978b42d9d9e95adcc638a40ae85fb3736fd897ae39cb8aa8333be39cf23f"
        ):
            raise RuntimeError(f"invalid final v2 limit audit: {LIMIT_AUDIT_PATH}")

        deterministic = audit["action_mode_205"]["deterministic"]
        stochastic = audit["action_mode_205"]["stochastic"]
        difference = audit["action_mode_205"]["difference"]
        training = audit["training"]
        density_rows = audit["density_sweep_deterministic"]
        causal = None
        if CAUSAL_1TO3_PATH.is_file():
            causal = json.loads(CAUSAL_1TO3_PATH.read_text(encoding="utf-8"))
            if (
                causal.get("schema_version") != 1
                or causal.get("training_performed") is not False
                or causal.get("checkpoint_sha256") != checkpoint["sha256"]
                or causal.get("second_seed", {}).get("practical_replication") is not True
            ):
                raise RuntimeError(f"invalid v2 causal 1--3 result: {CAUSAL_1TO3_PATH}")
        mirror = causal.get("mirror", {}) if causal else {}
        seed_check = causal.get("second_seed", {}) if causal else {}
        action_pair = mirror.get("action_pair", {})
        bearing = mirror.get("initial_target_bearing", {})
        causal_done = causal is not None
        fixed_speed = None
        if FIXED_SPEED_PATH.is_file():
            fixed_speed = json.loads(FIXED_SPEED_PATH.read_text(encoding="utf-8"))
            speed_cells = fixed_speed.get("cells", [])
            if (
                fixed_speed.get("schema_version") != 1
                or fixed_speed.get("checkpoint_sha256") != checkpoint["sha256"]
                or [cell.get("target_speed_mps") for cell in speed_cells] != [0.3, 0.9, 1.5]
                or fixed_speed.get("high_minus_low_capture", {}).get(
                    "material_speed_sensitivity"
                )
                is not True
            ):
                raise RuntimeError(f"invalid v2 fixed-speed result: {FIXED_SPEED_PATH}")
        forgetting = None
        if FORGETTING_PATH.is_file():
            forgetting = json.loads(FORGETTING_PATH.read_text(encoding="utf-8"))
            comparisons = forgetting.get("comparisons", {})
            if (
                forgetting.get("schema_version") != 1
                or forgetting.get("checkpoints", {}).get("24000") != checkpoint["sha256"]
                or forgetting.get("material_forgetting_detected") is not False
                or comparisons.get("uniform", {}).get("verdict") != "improvement"
                or comparisons.get("fast1p5", {}).get("verdict") != "improvement"
            ):
                raise RuntimeError(f"invalid v2 forgetting result: {FORGETTING_PATH}")
        ttc_1650_rows = []
        if TTC_1650_PATH.is_file():
            with TTC_1650_PATH.open(encoding="utf-8", newline="") as stream:
                ttc_1650_rows = list(csv.DictReader(stream))
        ttc_1650 = {row.get("arm"): row for row in ttc_1650_rows}
        ttc_1650_pass = (
            set(ttc_1650) == {"baseline", "ttc"}
            and ttc_1650["ttc"].get("gate_pass") == "PASS"
            and float(ttc_1650["ttc"]["capture_pct"])
            - float(ttc_1650["baseline"]["capture_pct"])
            >= 2.0
            and float(ttc_1650["ttc"]["crash_pct"])
            - float(ttc_1650["baseline"]["crash_pct"])
            <= -2.0
        )
        causal_all_done = causal_done and fixed_speed is not None and forgetting is not None
        speed_delta = (
            fixed_speed["high_minus_low_capture"]["delta"] if fixed_speed else None
        )
        speed_bar_delta = (
            fixed_speed["high_minus_low_rates"]["bar_contact"] if fixed_speed else None
        )
        forgetting_uniform = (
            forgetting["comparisons"]["uniform"] if forgetting else {}
        )
        forgetting_fast = forgetting["comparisons"]["fast1p5"] if forgetting else {}
        comparison_rows = [
            {
                "label": f"held-out deterministic · {row['density_per_100m2']:.2f}/100m²",
                "bars": row["bars"],
                "capture": row["capture_rate"],
                "unique": None,
                "verdict": (
                    f"n={row['episodes']:,}; crash {row['crash_rate'] * 100:.2f}%; "
                    f"timeout {row['timeout_rate'] * 100:.2f}%"
                ),
            }
            for row in density_rows
        ]
        if causal_all_done:
            comparison_rows.extend(
                {
                    "label": f"ep24000 · fixed target {cell['target_speed_mps']:.1f} m/s",
                    "bars": 205,
                    "capture": cell["capture_rate"],
                    "unique": None,
                    "verdict": (
                        f"crash {cell['crash_rate'] * 100:.2f}%; "
                        f"bar contact {cell['bar_contact_rate'] * 100:.2f}%"
                    ),
                }
                for cell in fixed_speed["cells"]
            )
        return {
            "subtitle": (
                "2026-08-03 · v2 causal audit complete · fixed-205 TTC A/B ready"
                if causal_all_done and ttc_1650_pass
                else "2026-08-02 · v2 ep24000 causal checks 1–3 complete · fixed-speed checks pending"
                if causal_done
                else "2026-08-02 · v2 ep24000 core limit audit · causal checks pending"
            ),
            "headline": (
                "The 205-bar stage improved rather than forgot; high-speed bar contact is the next controlled bottleneck."
                if causal_all_done
                else "The 205-bar score replicates across seeds, but the policy has a strong learned turn-direction habit."
                if causal_done
                else "The unchanged 205-bar curriculum is closed; mirror and multi-condition evaluations come next."
            ),
            "summary": (
                f"After {training['epochs_at_205']:,} epochs and "
                f"{training['held_windows_at_205']} complete holds at 205 bars, the last seven "
                f"16,384-episode gates averaged {training['last_seven_gate_mean'] * 100:.2f}%. "
                f"The frozen ep24000 policy captures {deterministic['capture_rate'] * 100:.2f}% "
                f"with deterministic deployment actions versus {stochastic['capture_rate'] * 100:.2f}% "
                "with stochastic training actions. PPO divergence and disconnected geometry were rejected; "
                "bar contact on long, fast trajectories is the remaining failure. "
                + (
                    f"Seed 43 reproduced capture within {seed_check['capture_seed43_minus_seed42'] * 100:+.2f} pp. "
                    f"Aggregate mirror performance differed by only {mirror['differences']['capture_conjugate_minus_original'] * 100:+.2f} pp, "
                    f"but exact reflected observations disagreed in lateral action sign {action_pair['lateral_sign_mismatch_rate'] * 100:.1f}% of comparable samples."
                    if causal_done
                    else ""
                )
                + (
                    f" Fixed-speed capture falls {speed_delta * 100:.2f} pp from 0.3 to 1.5 m/s while bar contact rises {speed_bar_delta * 100:.2f} pp. "
                    f"There is no 205-stage forgetting: ep24000 improves over ep19100 by {forgetting_uniform['ep24000_minus_ep19100_capture'] * 100:+.2f} pp on U[0.3,1.5] and {forgetting_fast['ep24000_minus_ep19100_capture'] * 100:+.2f} pp at fixed 1.5 m/s."
                    if causal_all_done
                    else ""
                )
            ),
            "active_experiment": {
                **record,
                "is_live": False,
                "core_audit_complete": True,
                "causal_checks_pending": not causal_all_done,
                "causal_checks_1to3_complete": causal_done,
                "causal_checks_complete": causal_all_done,
                "fixed_speed_complete": fixed_speed is not None,
                "forgetting_complete": forgetting is not None,
                "ttc_1650_gate_pass": ttc_1650_pass,
                "next_training_authorized": causal_all_done and ttc_1650_pass,
                "stage_status": "closed",
                "epoch": 24000,
                "max_epochs": 24000,
                "bars": 205,
                "selector": "cluster_sector",
                "cluster_gap_m": 0.45,
                "sectors": 8,
                "arena_xy_m": 40,
                "arena_z_m": 3,
                "density_curriculum": False,
                "frozen_checkpoint_sha256": checkpoint["sha256"],
                "deterministic_capture": deterministic["capture_rate"],
                "deterministic_crash": deterministic["crash_rate"],
                "deterministic_timeout": deterministic["timeout_rate"],
                "deterministic_episodes": deterministic["episodes"],
                "stochastic_capture": stochastic["capture_rate"],
                "stochastic_crash": stochastic["crash_rate"],
                "stochastic_episodes": stochastic["episodes"],
            },
            "milestones": [
                {
                    "label": "FINAL ARTIFACT",
                    "value": "ep24000 FROZEN",
                    "detail": f"SHA-256 {checkpoint['sha256'][:12]}…; no training active",
                    "state": "pass",
                },
                {
                    "label": "SPEED SENSITIVITY" if causal_all_done else "SEED REPLICATION" if causal_done else "205 · DEPLOY",
                    "value": (
                        f"{speed_delta * 100:+.2f} pp"
                        if causal_all_done
                        else f"Δ {seed_check['capture_seed43_minus_seed42'] * 100:+.2f} pp"
                        if causal_done
                        else f"{deterministic['capture_rate'] * 100:.2f}%"
                    ),
                    "detail": (
                        f"0.3→1.5 m/s; bar contact {speed_bar_delta * 100:+.2f} pp · material"
                        if causal_all_done
                        else f"seed42 {seed_check['seed42']['capture_rate'] * 100:.2f}% · "
                        f"seed43 {seed_check['seed43']['capture_rate'] * 100:.2f}% · PASS"
                        if causal_done
                        else f"n={deterministic['episodes']:,}; crash "
                        f"{deterministic['crash_rate'] * 100:.2f}%"
                    ),
                    "state": "pass",
                },
                {
                    "label": "205-STAGE FORGETTING" if causal_all_done else "MIRROR OUTCOME" if causal_done else "205 · TRAIN POLICY",
                    "value": (
                        "NO · IMPROVED"
                        if causal_all_done
                        else f"{mirror['differences']['capture_conjugate_minus_original'] * 100:+.2f} pp"
                        if causal_done
                        else f"{stochastic['capture_rate'] * 100:.2f}%"
                    ),
                    "detail": (
                        f"uniform {forgetting_uniform['ep24000_minus_ep19100_capture'] * 100:+.2f} pp · fast1.5 {forgetting_fast['ep24000_minus_ep19100_capture'] * 100:+.2f} pp"
                        if causal_all_done
                        else f"4,096/arm; initial-bearing Δ {bearing['positive_minus_negative_capture'] * 100:+.2f} pp; no outcome gap"
                        if causal_done
                        else f"n={stochastic['episodes']:,}; exploration costs "
                        f"{difference['capture_rate_difference_deterministic_minus_stochastic'] * 100:.2f} pp"
                    ),
                    "state": "pass" if causal_done else "warn",
                },
                {
                    "label": "1650 Ti TTC GATE" if causal_all_done else "ACTION EQUIVARIANCE" if causal_done else "NEXT",
                    "value": (
                        "PASS" if ttc_1650_pass else "MISSING"
                        if causal_all_done
                        else f"{action_pair['lateral_sign_mismatch_rate'] * 100:.1f}% MISMATCH"
                        if causal_done
                        else "MIRROR EVAL"
                    ),
                    "detail": (
                        "70 bars · capture +9.86 pp · crash -8.06 pp"
                        if causal_all_done and ttc_1650_pass
                        else "paired 4GB evidence not verified"
                        if causal_all_done
                        else f"lateral MAE {action_pair['mean_abs_error'][1]:.3f}; learned chirality, outcome-neutral in current symmetric arena"
                        if causal_done
                        else "evaluation only; frozen weights; no training"
                    ),
                    "state": "pass" if causal_all_done and ttc_1650_pass else "warn" if causal_done else "active",
                },
                {
                    "label": "NEXT CONTROLLED STEP",
                    "value": "MAIN 205 TTC A/B" if causal_all_done and ttc_1650_pass else "BLOCKED",
                    "detail": "run baseline first; then sample-matched ttc_sector arm"
                    if causal_all_done and ttc_1650_pass
                    else "complete causal and 4GB gates before training",
                    "state": "active" if causal_all_done and ttc_1650_pass else "warn",
                },
            ],
            "comparison": comparison_rows,
            "gates": [
                {"label": "curriculum continuation", "value": "CLOSED · more unchanged epochs rejected"},
                {"label": "PPO divergence", "value": "REJECTED · behavior KL < 0.04; no rollback/OOB"},
                {"label": "geometry disconnection", "value": "REJECTED · 99.83% random-pair connectivity @0.2m"},
                {
                    "label": "mirror + second seed",
                    "value": (
                        "COMPLETE · outcome symmetric, action non-equivariant"
                        if causal_done
                        else "PENDING · frozen checkpoint, no learning"
                    ),
                },
                {
                    "label": "fixed-speed + forgetting",
                    "value": "COMPLETE · speed-sensitive; no forgetting"
                    if causal_all_done
                    else "PENDING · evaluation only",
                },
                {
                    "label": "1650 Ti TTC transfer",
                    "value": "PASS · capture +9.86 pp; crash -8.06 pp"
                    if ttc_1650_pass
                    else "PENDING",
                },
                {
                    "label": "next training",
                    "value": "READY · fixed-205 main baseline → TTC"
                    if causal_all_done and ttc_1650_pass
                    else "BLOCKED · causal checks and 1650 Ti gate pending",
                },
            ],
            "decision": (
                "Do not resume the open-ended 205-bar curriculum. Preserve ep24000 as the canonical "
                "artifact. "
                + (
                    "All frozen-policy causal checks are complete. The 205 stage improved both uniform and fixed-1.5 performance, so replay for catastrophic forgetting is not the next intervention. "
                    "Speed sensitivity is material and paid almost entirely in bar contact; the 1650 Ti TTC selector transfer gate also passed. "
                    "Authorize the sample-matched fixed-205 main baseline arm first, then the TTC arm. Keep reflection/RMS regularization and learned-detector work as separate later experiments. "
                    if causal_all_done and ttc_1650_pass
                    else "Mirror and second-seed evaluation are complete: success is seed-stable and outcome-symmetric, "
                    "but the controller is not action-equivariant. Finish the fixed-speed and forgetting checks before authorizing main training. "
                    if causal_done
                    else "Evaluate original/mirrored layouts without updating the policy, then finish "
                    "the second-seed and fixed-speed checks. "
                )
            ),
            "limit_audit": audit,
            "causal_1to3": causal,
            "fixed_speed": fixed_speed,
            "forgetting": forgetting,
            "ttc_1650": ttc_1650_rows,
        }

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
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--allow-empty",
        action="store_true",
        help="write even when no local run evidence is visible (almost never correct)",
    )
    args = parser.parse_args()

    status = build_snapshot()
    # runs/ is gitignored, so a fresh clone -- a Cursor cloud/mobile agent, or any
    # machine that has not trained -- sees zero runs. Writing then silently strips
    # the published run history and latest_run, and publishing that wipes the site.
    if status["n_runs"] == 0 and not args.allow_empty:
        print(
            "[status] refusing to write: no runs found under\n"
            f"  {RUNS_ROOT}\n"
            "runs/ is gitignored, so this is expected on a fresh clone (cloud/mobile\n"
            "agent). Publishing an empty snapshot would erase the dashboard's run\n"
            "history. Run this on the training workstation, or pass --allow-empty if\n"
            "you really intend an empty dashboard.",
            file=sys.stderr,
        )
        return 2

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
