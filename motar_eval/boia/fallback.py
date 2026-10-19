"""07 §6.4 [NEW]: zero translation with permitted-belief yaw; no capture declaration."""
import torch


def safe_hold(inp):
    """Normalized (N,4) brake, active belief yaw only; inactive yaw is exactly zero."""
    n = inp.pos.shape[0]
    assert inp.pos.shape == (n, 3) and inp.mu.shape == (n, 6) and inp.yaw.shape == inp.active.shape == (n,)
    relative = torch.where(inp.active[:, None].bool(), inp.mu[:, :2] - inp.pos[:, :2],
                           torch.zeros_like(inp.pos[:, :2]))
    bearing = torch.atan2(relative[:, 1], relative[:, 0])
    error = torch.atan2((bearing - inp.yaw).sin(), (bearing - inp.yaw).cos())
    valid = inp.active.bool() & ((relative * relative).sum(-1) > 1e-12)
    yaw = torch.where(valid, (2 * error).clamp(-3, 3) / 3, torch.zeros_like(error))
    action = inp.pos.new_zeros((n, 4))
    action[:, 3] = yaw
    return action
