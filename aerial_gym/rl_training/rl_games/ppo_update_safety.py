"""Small, simulator-independent PPO safety primitives used by NavRL training."""

import os
import torch


def stable_ppo_actor_loss(
    old_action_neglog_probs_batch,
    action_neglog_probs,
    advantage,
    is_ppo,
    curr_e_clip,
    max_abs_log_ratio,
):
    """PPO surrogate with a finite log-ratio before ``exp``.

    rl-games exponentiates the raw log-ratio. Near a tanh boundary, a very unlikely replayed action
    can make that difference large enough to overflow before PPO's ordinary ratio clip is applied.
    The wide default clamp is only a numerical guard; the agent's separate KL gate rejects updates
    that have moved materially too far.
    """
    if not is_ppo:
        return action_neglog_probs * advantage
    log_ratio = old_action_neglog_probs_batch - action_neglog_probs
    ratio = torch.exp(torch.clamp(log_ratio, -max_abs_log_ratio, max_abs_log_ratio))
    surr1 = advantage * ratio
    surr2 = advantage * torch.clamp(
        ratio, 1.0 - curr_e_clip, 1.0 + curr_e_clip
    )
    return torch.maximum(-surr1, -surr2)


def lateral_latent_margin_loss(mu, margin, axis=1):
    """Mean squared excess of the selected latent mean beyond a soft symmetric margin."""
    if not isinstance(mu, torch.Tensor) or mu.ndim != 2 or mu.shape[1] <= axis:
        raise ValueError("policy mean does not contain the requested lateral action axis")
    return torch.relu(mu[:, axis].abs() - margin).square().mean()


def lateral_batch_bias_loss(mu, axis=1):
    """Penalize a population-wide signed lateral preference, not avoidance magnitude."""
    if not isinstance(mu, torch.Tensor) or mu.ndim != 2 or mu.shape[1] <= axis:
        raise ValueError("policy mean does not contain the requested lateral action axis")
    return mu[:, axis].mean().square()


def mirror_navrl_actions(actions):
    """Reflect body-frame actions across the vehicle x-z plane."""
    if not isinstance(actions, torch.Tensor) or actions.ndim != 2 or actions.shape[1] != 4:
        raise ValueError("NavRL actions must have shape [batch, 4]")
    mirrored = actions.clone()
    mirrored[:, 1] = -mirrored[:, 1]
    mirrored[:, 3] = -mirrored[:, 3]
    return mirrored


def mirror_navrl_structured_observation(obs):
    """Reflect every signed lateral field in the 898-D structured actor observation."""
    # Keep this helper simulator-independent: importing aerial_gym after torch violates Isaac Gym's
    # import-order requirement in CPU unit tests. These are the explicit structured-schema knobs
    # from navrl_perception.py; the dimension check below catches any drift.
    hbeams = int(os.environ.get("NAVRL_LIDAR_HBEAMS", "").strip() or 36)
    vbeams = int(os.environ.get("NAVRL_LIDAR_VBEAMS", "").strip() or 4)
    max_obstacles = int(os.environ.get("NAVRL_MAX_OBSTACLES", "").strip() or 5)
    obstacle_history, obstacle_dim = 5, 12
    robot_history, robot_dim = 5, 10
    target_history, target_dim = 5, 16
    static_dim = vbeams * hbeams
    structured_obs_dim = (
        static_dim
        + obstacle_history * max_obstacles * obstacle_dim
        + robot_history * robot_dim
        + target_history * target_dim
    )

    if (
        not isinstance(obs, torch.Tensor)
        or obs.ndim != 2
        or obs.shape[1] != structured_obs_dim
    ):
        raise ValueError(
            "NavRL structured observation must have shape [batch, %d]"
            % structured_obs_dim
        )
    mirrored = obs.clone()
    offset = 0

    # Bearings are [-180+bin, ..., 180]. Reflection maps index i to H-2-i modulo H;
    # the final +180 ray maps to itself because +/-180 are the same direction.
    scan = obs[:, offset : offset + static_dim].view(-1, vbeams, hbeams)
    reflect_index = (
        hbeams - 2 - torch.arange(hbeams, device=obs.device)
    ) % hbeams
    mirrored[:, offset : offset + static_dim] = scan.index_select(
        2, reflect_index
    ).reshape(obs.shape[0], -1)
    offset += static_dim

    obstacle_size = obstacle_history * max_obstacles * obstacle_dim
    obstacles = mirrored[:, offset : offset + obstacle_size].view(
        -1, obstacle_history, max_obstacles, obstacle_dim
    )
    obstacles[..., 1] = -obstacles[..., 1]  # relative position y
    obstacles[..., 4] = -obstacles[..., 4]  # relative velocity y
    offset += obstacle_size

    robot_size = robot_history * robot_dim
    robot = mirrored[:, offset : offset + robot_size].view(
        -1, robot_history, robot_dim
    )
    # velocity y, yaw rate, previous action y, previous yaw action
    robot[..., (1, 3, 5, 7)] = -robot[..., (1, 3, 5, 7)]
    offset += robot_size

    target = mirrored[:, offset:].view(-1, target_history, target_dim)
    target[..., 1] = -target[..., 1]  # tracked relative position y
    target[..., 4] = -target[..., 4]  # tracked relative velocity y
    return mirrored


def reflection_equivariance_loss(mu, mirrored_mu):
    """Require a reflected observation to produce the reflected latent policy mean."""
    if (
        not isinstance(mu, torch.Tensor)
        or not isinstance(mirrored_mu, torch.Tensor)
        or mu.shape != mirrored_mu.shape
    ):
        raise ValueError("original and mirrored policy means must have matching shapes")
    return (mirrored_mu - mirror_navrl_actions(mu)).square().mean()
