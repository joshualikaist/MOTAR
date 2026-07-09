"""
Mean-reward window-band convergence early stop for rl-games PPO (continuous).

After ``min_epoch``, keep the last ``consecutive_epochs`` logged mean episode
rewards. Training stops when that window's span satisfies:

    max(window) - min(window) <= reward_band

This is plateau / band convergence — not pairwise |r[t] - r[t-1]|, which can
trigger while the policy is still drifting or stuck at a bad flat level without
being a meaningful optimum.

Configurable under ``ppo_*.yaml`` → ``params`` → ``config`` → ``early_stop_stable``.
"""

from __future__ import annotations

from typing import Any, Dict, List, Tuple


def parse_early_stop_stable_config(config_section):
    """
    Normalize ``early_stop_stable`` from rl-games config (dict-like).
    Missing or disabled blocks return disable sentinel.
    """
    if config_section is None:
        return None
    if not isinstance(config_section, dict):
        return None
    if not config_section.get("enable", False):
        return None
    return {
        "enable": True,
        "min_epoch": int(config_section.get("min_epoch", 0)),
        "reward_band": float(config_section.get("reward_band", 100.0)),
        "consecutive_epochs": int(config_section.get("consecutive_epochs", 500)),
    }


def window_band_stable_should_stop(
    cfg,
    epoch_num,
    mean_reward_0_scalar,
    prev_state,
) -> Tuple[bool, Dict[str, Any]]:
    """
    Returns (should_stop, next_state_dict).

    ``history`` holds up to ``consecutive_epochs`` mean rewards collected strictly
    after ``min_epoch``. Stop when the window is full and its max−min ≤ band.
    """
    if cfg is None:
        return False, prev_state or {}

    need = cfg["consecutive_epochs"]
    band = cfg["reward_band"]

    if epoch_num <= cfg["min_epoch"]:
        return False, {"history": []}

    history: List[float] = list((prev_state or {}).get("history", []))
    history.append(float(mean_reward_0_scalar))
    if len(history) > need:
        history = history[-need:]

    next_state: Dict[str, Any] = {"history": history}

    if len(history) < need:
        return False, next_state

    spread = max(history) - min(history)
    next_state["last_spread"] = spread
    if spread <= band:
        return True, next_state

    return False, next_state


def parse_early_stop_collapse_config(config_section):
    if config_section is None or not isinstance(config_section, dict):
        return None
    if not config_section.get("enable", False):
        return None
    return {
        "enable": True,
        "min_epoch": int(config_section.get("min_epoch", 400)),
        "drop_from_peak": float(config_section.get("drop_from_peak", 0.55)),
        "patience_epochs": int(config_section.get("patience_epochs", 250)),
        "min_peak_reward": float(config_section.get("min_peak_reward", 400.0)),
    }


def collapse_from_peak_should_stop(cfg, epoch_num, mean_reward_scalar, prev_state):
    """
    Stop when mean reward falls far below a prior peak for many consecutive epochs
    (reward collapse / catastrophic forgetting).
    """
    if cfg is None:
        return False, prev_state or {}

    import math

    mr = float(mean_reward_scalar)
    if math.isnan(mr) or math.isinf(mr):
        return True, {"peak": (prev_state or {}).get("peak", 0.0), "below_count": 0, "nan_stop": True}

    state = dict(prev_state or {})
    peak = float(state.get("peak", mr))
    below_count = int(state.get("below_count", 0))

    if mr > peak:
        peak = mr
        below_count = 0
    elif (
        epoch_num >= cfg["min_epoch"]
        and peak >= cfg["min_peak_reward"]
        and mr < peak * (1.0 - cfg["drop_from_peak"])
    ):
        below_count += 1
    else:
        below_count = 0

    next_state = {"peak": peak, "below_count": below_count}
    if below_count >= cfg["patience_epochs"]:
        next_state["collapse_peak"] = peak
        next_state["collapse_mr"] = mr
        return True, next_state

    return False, next_state
