"""Read-only safety checks for a NavRL density-curriculum resume checkpoint."""

from __future__ import annotations

import argparse
import math
import os
from pathlib import Path
from typing import Any, Dict, Mapping, Optional

import torch


class CheckpointPreflightError(RuntimeError):
    pass


_CONTRACT_ENV = {
    "cfg_lidar_max_range": "NAVRL_LIDAR_RANGE",
    "cfg_max_obstacles": "NAVRL_MAX_OBSTACLES",
    "cfg_token_fov_deg": "NAVRL_OBSTACLE_FOV_DEG",
    "cfg_obstacle_suppress_deg": "NAVRL_OBSTACLE_SUPPRESS_DEG",
    "cfg_lidar_hbeams": "NAVRL_LIDAR_HBEAMS",
    "cfg_lidar_vbeams": "NAVRL_LIDAR_VBEAMS",
}


def _first_nonfinite(value: Any, path: str = "checkpoint") -> Optional[str]:
    if isinstance(value, torch.Tensor):
        if (value.is_floating_point() or value.is_complex()) and not bool(
            torch.isfinite(value).all().item()
        ):
            return path
        return None
    if isinstance(value, Mapping):
        for key, child in value.items():
            found = _first_nonfinite(child, f"{path}.{key}")
            if found is not None:
                return found
        return None
    if isinstance(value, (list, tuple)):
        for index, child in enumerate(value):
            found = _first_nonfinite(child, f"{path}[{index}]")
            if found is not None:
                return found
        return None
    if isinstance(value, float) and not math.isfinite(value):
        return path
    return None


def expected_contract_from_environment() -> Dict[str, float]:
    expected = {}
    for saved_key, env_name in _CONTRACT_ENV.items():
        raw = os.environ.get(env_name, "").strip()
        if raw:
            expected[saved_key] = float(raw)
    return expected


def inspect_checkpoint(
    checkpoint_path: str,
    *,
    max_epochs: int,
    density_final: int,
    expected_contract: Optional[Mapping[str, float]] = None,
) -> Dict[str, Any]:
    path = Path(checkpoint_path)
    if not path.is_file():
        raise CheckpointPreflightError(f"checkpoint not found: {path}")

    try:
        checkpoint = torch.load(str(path), map_location="cpu", weights_only=False)
    except Exception as exc:
        raise CheckpointPreflightError(f"checkpoint load failed: {path}: {exc}") from exc
    if not isinstance(checkpoint, dict):
        raise CheckpointPreflightError("checkpoint root must be a dictionary")

    for section in ("model", "optimizer", "assymetric_vf_nets", "assymetric_vf_optimizer"):
        if section not in checkpoint:
            raise CheckpointPreflightError(f"checkpoint is missing required section: {section}")

    try:
        epoch = int(checkpoint["epoch"])
    except (KeyError, TypeError, ValueError) as exc:
        raise CheckpointPreflightError("checkpoint epoch is missing or invalid") from exc
    if int(max_epochs) <= epoch:
        raise CheckpointPreflightError(
            f"MAX_EPOCHS must exceed checkpoint epoch ({max_epochs} <= {epoch})"
        )

    state = checkpoint.get("env_state")
    if not isinstance(state, dict):
        raise CheckpointPreflightError("checkpoint env_state is missing")
    try:
        bars = int(state["n_bars_active"])
    except (KeyError, TypeError, ValueError) as exc:
        raise CheckpointPreflightError("checkpoint n_bars_active is missing or invalid") from exc
    if bars < 0 or bars > int(density_final):
        raise CheckpointPreflightError(
            f"checkpoint density {bars} is outside requested curriculum range 0..{density_final}"
        )

    for saved_key, expected in (expected_contract or {}).items():
        saved = state.get(saved_key)
        if saved is None:
            raise CheckpointPreflightError(
                f"checkpoint lacks policy-input provenance field: {saved_key}"
            )
        if abs(float(saved) - float(expected)) > 1e-6:
            raise CheckpointPreflightError(
                f"policy-input mismatch for {saved_key}: checkpoint={saved}, requested={expected}"
            )

    nonfinite_path = _first_nonfinite(checkpoint)
    if nonfinite_path is not None:
        raise CheckpointPreflightError(f"non-finite checkpoint value: {nonfinite_path}")

    return {
        "path": str(path),
        "epoch": epoch,
        "bars": bars,
        "task_steps": int(state.get("num_task_steps", 0)),
        "token_fov_deg": state.get("cfg_token_fov_deg"),
        "max_obstacles": state.get("cfg_max_obstacles"),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("checkpoint")
    parser.add_argument("--max-epochs", type=int, required=True)
    parser.add_argument("--density-final", type=int, required=True)
    args = parser.parse_args()
    try:
        info = inspect_checkpoint(
            args.checkpoint,
            max_epochs=args.max_epochs,
            density_final=args.density_final,
            expected_contract=expected_contract_from_environment(),
        )
    except CheckpointPreflightError as exc:
        print(f"[general_repr_density] preflight FAILED | {exc}", file=os.sys.stderr)
        return 2

    print(
        "[general_repr_density] checkpoint state | "
        f"epoch={info['epoch']} bars={info['bars']} task_steps={info['task_steps']} "
        f"tokens={info['max_obstacles']} fov={info['token_fov_deg']}deg"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
