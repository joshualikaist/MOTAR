"""Small, simulator-independent PPO safety primitives used by NavRL training."""

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
