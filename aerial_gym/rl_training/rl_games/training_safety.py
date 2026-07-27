"""Small, dependency-light safety helpers for PPO training."""

from __future__ import annotations

import math
from typing import Iterable, Optional, Tuple

import torch


def _is_finite(value) -> bool:
    if isinstance(value, torch.Tensor):
        return bool(torch.isfinite(value.detach()).all().item())
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def first_nonfinite_training_value(
    metric_groups: Iterable[Tuple[str, Iterable]],
    named_parameters: Iterable[Tuple[str, torch.Tensor]],
) -> Optional[str]:
    """Return the first non-finite metric/parameter path, or ``None`` when all are finite."""
    for group_name, values in metric_groups:
        for index, value in enumerate(values):
            if not _is_finite(value):
                return f"{group_name}[{index}]"

    for name, parameter in named_parameters:
        if not _is_finite(parameter):
            return f"model.{name}"

    return None


def reset_optimizer_learning_rate(optimizer, learning_rate: float) -> None:
    """Override a checkpoint-restored optimizer LR with the active run configuration."""
    lr = float(learning_rate)
    if not math.isfinite(lr) or lr <= 0.0:
        raise ValueError(f"learning_rate must be finite and positive, got {learning_rate!r}")
    for group in optimizer.param_groups:
        group["lr"] = lr
