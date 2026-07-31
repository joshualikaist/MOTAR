"""Pure density-curriculum helpers shared by NavRLTask and CPU unit tests."""

from __future__ import annotations

from typing import Any, Mapping


def density_threshold_at(
    n_bars_active: int,
    n_start: int,
    n_final: int,
    threshold_start: float,
    threshold_end: float,
) -> float:
    """Linearly interpolate the capture gate over the configured density range."""
    if int(n_final) <= int(n_start):
        return float(threshold_start)
    fraction = (int(n_bars_active) - int(n_start)) / float(int(n_final) - int(n_start))
    fraction = min(1.0, max(0.0, fraction))
    return float(threshold_start) + fraction * (
        float(threshold_end) - float(threshold_start)
    )


def density_dwell_epochs(
    num_task_steps: int,
    level_start_steps: int,
    ppo_horizon: int,
) -> float:
    """Return non-negative PPO epochs spent at the current density."""
    elapsed_steps = max(0, int(num_task_steps) - int(level_start_steps))
    return elapsed_steps / float(max(1, int(ppo_horizon)))


def density_dwell_ready(
    num_task_steps: int,
    level_start_steps: int,
    ppo_horizon: int,
    min_epochs: int,
) -> bool:
    return density_dwell_epochs(
        num_task_steps,
        level_start_steps,
        ppo_horizon,
    ) >= max(0, int(min_epochs))


def density_level_start_after_promotion(
    previous_start_steps: int,
    num_task_steps: int,
    promoted: bool,
) -> int:
    """Reset the dwell clock exactly when a new density becomes active."""
    if promoted:
        return max(0, int(num_task_steps))
    return max(0, int(previous_start_steps))


def restore_density_level_start_steps(
    state: Mapping[str, Any],
    num_task_steps: int,
) -> int:
    """Restore a saved dwell clock; old checkpoints conservatively start it now."""
    current_steps = max(0, int(num_task_steps))
    saved = state.get("density_level_start_steps")
    if saved is None:
        return current_steps
    # A malformed/future clock would create a negative dwell. Clamp it into the run history.
    return min(current_steps, max(0, int(saved)))
