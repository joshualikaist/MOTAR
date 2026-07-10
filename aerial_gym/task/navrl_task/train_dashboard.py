"""TRAIN: per-epoch NavRL navigation stats for the console dashboard + TensorBoard.

Mirrors the intercept train_dashboard pattern: the task records every finished episode
(reached / success@timeout / crash / timeout / closest approach) into module-level
accumulators each step, and the training agent consumes them once per PPO epoch to
render dashboard lines and TensorBoard scalars (navrl/*).
"""

from __future__ import annotations

from typing import List, Optional, Tuple

_DONE = 0
_REACHED = 0
_SUCC_TIMEOUT = 0
_CRASH = 0
_TIMEOUT = 0
_CLOSEST_SUM = 0.0
_CLOSEST_COUNT = 0
_GOAL_DIST_MAX: Optional[float] = None


def record_navrl_epoch_episodes(
    num_finished: int,
    num_reached: int,
    num_success_timeout: int,
    num_crash: int,
    num_timeout: int,
    closest_sum: float,
    closest_count: int,
    goal_dist_max: Optional[float] = None,
) -> None:
    global _DONE, _REACHED, _SUCC_TIMEOUT, _CRASH, _TIMEOUT
    global _CLOSEST_SUM, _CLOSEST_COUNT, _GOAL_DIST_MAX
    if num_finished <= 0:
        return
    _DONE += int(num_finished)
    _REACHED += int(num_reached)
    _SUCC_TIMEOUT += int(num_success_timeout)
    _CRASH += int(num_crash)
    _TIMEOUT += int(num_timeout)
    if closest_count > 0:
        _CLOSEST_SUM += float(closest_sum)
        _CLOSEST_COUNT += int(closest_count)
    if goal_dist_max is not None:
        _GOAL_DIST_MAX = float(goal_dist_max)


def consume_navrl_epoch_summary() -> Tuple[List[str], dict, int]:
    """Return (dashboard lines, TensorBoard scalars {navrl/<name>: value}, episodes done)."""
    global _DONE, _REACHED, _SUCC_TIMEOUT, _CRASH, _TIMEOUT
    global _CLOSEST_SUM, _CLOSEST_COUNT, _GOAL_DIST_MAX
    done = _DONE
    reached = _REACHED
    succ = _SUCC_TIMEOUT
    crash = _CRASH
    timeout = _TIMEOUT
    closest = _CLOSEST_SUM / float(_CLOSEST_COUNT) if _CLOSEST_COUNT > 0 else None
    goal_dist_max = _GOAL_DIST_MAX
    _DONE = _REACHED = _SUCC_TIMEOUT = _CRASH = _TIMEOUT = 0
    _CLOSEST_SUM = 0.0
    _CLOSEST_COUNT = 0

    if done <= 0:
        return ([], {}, 0)

    def pct(k: int) -> str:
        return f"{100.0 * k / done:5.1f}% ({k}/{done})"

    lines = [
        f"goal reached         : {pct(reached)}",
        f"success @ timeout    : {pct(succ)}",
        f"crash                : {pct(crash)}",
        f"timeout (no reach)   : {pct(timeout)}",
    ]
    if closest is not None:
        lines.append(f"closest to goal (m)  : {closest:.2f}")
    if goal_dist_max is not None:
        lines.append(f"curriculum max (m)   : {goal_dist_max:.1f}")

    metrics = {
        "navrl/ep_finished": float(done),
        "navrl/reach_rate": reached / done,
        "navrl/success_at_timeout_rate": succ / done,
        "navrl/crash_rate": crash / done,
        "navrl/timeout_rate": timeout / done,
    }
    if closest is not None:
        metrics["navrl/mean_closest_approach_m"] = closest
    if goal_dist_max is not None:
        metrics["navrl/curriculum_goal_dist_max_m"] = goal_dist_max
    return (lines, metrics, done)
