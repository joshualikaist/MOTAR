"""Curated TensorBoard scalars for intercept PPO (epoch axis only)."""

from __future__ import annotations

from typing import Any, Optional

from rl_games.algos_torch import torch_ext


def _f(x: Any) -> float:
    if hasattr(x, "detach"):
        return float(x.detach().cpu().item())
    return float(x)


def write_aerial_epoch_scalars(
    writer,
    epoch_num: int,
    *,
    mean_reward: Optional[float] = None,
    mean_episode_length: Optional[float] = None,
    a_losses,
    c_losses,
    entropies,
    kls,
    last_lr: float,
    lr_mul: float,
    step_time: float,
    curr_frames: float,
    intercept_succ: int = 0,
    intercept_done: int = 0,
    intercept_envs_hit: int = 0,
    num_parallel_envs: int = 1,
    mean_target_dist_m: Optional[float] = None,
    mean_closest_target_dist_m: Optional[float] = None,
    explained_variance: Optional[Any] = None,
    extra_intercept_metrics: Optional[dict[str, Any]] = None,
) -> None:
    """Log only metrics useful for intercept training; x-axis = PPO epoch."""
    ep = int(epoch_num)
    w = writer

    if mean_reward is not None:
        w.add_scalar("aerial/mean_reward", mean_reward, ep)
    if mean_episode_length is not None:
        w.add_scalar("aerial/mean_episode_length", mean_episode_length, ep)
    if mean_target_dist_m is not None:
        w.add_scalar("aerial/mean_target_dist_m", mean_target_dist_m, ep)
    if mean_closest_target_dist_m is not None:
        w.add_scalar("aerial/mean_closest_target_dist_m", mean_closest_target_dist_m, ep)
    if extra_intercept_metrics:
        scalar_map = {
            "success_closest_target_dist_m": "aerial/success_closest_target_dist_m",
            "failure_closest_target_dist_m": "aerial/failure_closest_target_dist_m",
            "mean_surface_gap_m": "aerial/mean_surface_gap_m",
            "failure_surface_gap_m": "aerial/failure_surface_gap_m",
            "near_miss_count": "intercept/near_miss_count",
            "near_miss_rate": "intercept/near_miss_rate",
            "radius_no_contact_count": "intercept/radius_no_contact_count",
            "radius_no_contact_rate": "intercept/radius_no_contact_rate",
            "contact_no_radius_count": "intercept/contact_no_radius_count",
            "contact_no_radius_rate": "intercept/contact_no_radius_rate",
        }
        for key, tag in scalar_map.items():
            value = extra_intercept_metrics.get(key)
            if value is not None:
                w.add_scalar(tag, float(value), ep)
        # task-namespaced metrics (e.g. navrl/*) are written under their own tag as-is
        for key, value in extra_intercept_metrics.items():
            if "/" in key and value is not None:
                w.add_scalar(key, float(value), ep)

    w.add_scalar("intercept/ep_finished", int(intercept_done), ep)
    if intercept_done > 0:
        w.add_scalar("intercept/success_rate", float(intercept_succ) / float(intercept_done), ep)
    if num_parallel_envs > 0:
        w.add_scalar("intercept/env_hit_rate", float(intercept_envs_hit) / float(num_parallel_envs), ep)

    w.add_scalar("ppo/a_loss", torch_ext.mean_list(a_losses).item(), ep)
    w.add_scalar("ppo/c_loss", torch_ext.mean_list(c_losses).item(), ep)
    w.add_scalar("ppo/entropy", torch_ext.mean_list(entropies).item(), ep)
    w.add_scalar("ppo/kl", torch_ext.mean_list(kls).item(), ep)
    w.add_scalar("ppo/learning_rate", float(last_lr) * float(lr_mul), ep)

    if explained_variance is not None:
        w.add_scalar("ppo/explained_variance", _f(explained_variance), ep)

    if step_time and step_time > 0:
        w.add_scalar("perf/step_fps", float(curr_frames) / float(step_time), ep)
