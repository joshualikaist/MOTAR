"""Opt-in camera-safety rule and reward terms for the camera-only from-scratch training CAM-S1
(docs/plans/prereg_camonly_s1_scratch_2026-11-23.md; user decision D-2026-11-23-1).

Why: Policy F flies outside its camera's FOV in 51.8 % of moving frames and, without LiDAR, its crashes are
"consistent with" contacts on bars outside the view (C2 out_fov +39.25 pp; G3 BLIND_SIDE_SUPPORTED). For the trained
camera-only CAM-K2 v3, CAM-T1 found most contacts in view by the footprint profile (67.7 %), the camera inside MinZ at
every valid contact, and no crash change from lower global caps; so (1) is a flight-safety rule ("do not move fast
where the camera cannot see"), not a claimed crash lever. The additions:

  1. blind-direction speed cap (a rule; applied in training; the flight runtime does NOT have it yet, design review r1
     P0-F, so a CAM-S1 checkpoint is not flight-ready until the runtime applies the same cap): after the
     riskcap governor, a horizontal command whose vehicle-frame bearing lies outside the camera's horizontal FOV
     (|bearing| > hfov / 2, perception's own hfov) is scaled, direction kept, to at most BLIND_MPS;
  2. blind-motion penalty (reward, training only): for envs still alive after the step, minus
     BLIND_WEIGHT * excess * gate, where excess = max(0, |bearing(v)| - hfov / 2) / (pi - hfov / 2) of the MEASURED
     vehicle-frame horizontal velocity v, and gate = clamp(|v| / yaw_align_speed_ref, 0, 1). It is <= 0 and zero
     inside the FOV, so it opens no standing income and leaves in-view motion untouched;
  3. collision penalty (reward): task_config.reward_parameters["collision_penalty"] set to COLLISION_PENALTY before
     the task is built (Policy F: -20);
  4. static-safety range (reward): `add_static_safety_reward` recomputed with the ray distances clipped at RSS_CLIP_M
     (NavRL's own 4 m) instead of the 12 m sensor range. The code is the original method with that one constant
     changed; at RSS_CLIP_M = lidar_max_range it is the original bit for bit (tests/test_camsafe.py). Why (reward
     review r1, Fable, ~/motar_tmp/cams1_20261123/review_reward_fable_r1/REVIEW.md): with the 12 m range r_ss is a
     standing cost of about -0.34 / -0.87 per step at 70 / 205 bars, which makes "crash now" (-20) worth more than
     hovering (about -24 / -77 discounted); with the 4 m clip hovering at 205 bars is about -17 and the near-bar
     gradient is unchanged.

Selected only through tools/realveh_entry.py when NAVRL_CAMSAFE=1, installed after the sensor ablation (the ablation
binds the governor method's module globals at its own install; wrapping it first would break that binding, so
install() refuses unless the ablation is already on the class). Default OFF: nothing else imports this module.
navrl_task.py and navrl_perception.py are byte-pinned, so their methods are wrapped at run time.
"""
import math
import os

ENV_SWITCH = "NAVRL_CAMSAFE"
PARAMETERS = {
    # name: (lower, upper)
    "NAVRL_CAMSAFE_BLIND_MPS": (0.3, 2.0),
    "NAVRL_CAMSAFE_BLIND_WEIGHT": (0.0, 2.0),
    "NAVRL_CAMSAFE_COLLISION_PENALTY": (-100.0, -20.0),
    "NAVRL_CAMSAFE_RSS_CLIP_M": (1.0, 12.0),
}
REPORT_AT_CALLS = (32, 320)
REPORT_EVERY_CALLS = 3200
_INSTALLED = {}


def settings_from_env(environ=None):
    """Validated settings (stdlib only). Returns None when the switch is off; refuses a parameter set while off,
    a missing, non-numeric or out-of-range value, and a switch value other than unset / 0 / 1."""
    env = os.environ if environ is None else environ
    switch = env.get(ENV_SWITCH, "").strip()
    if switch not in ("", "0", "1"):
        raise ValueError("%s must be unset, 0 or 1, got %r" % (ENV_SWITCH, switch))
    if switch != "1":
        stray = sorted(k for k in PARAMETERS if env.get(k, "").strip())
        if stray:
            raise ValueError("%s set while %s is off" % (", ".join(stray), ENV_SWITCH))
        return None
    out = {}
    for name, (lo, hi) in PARAMETERS.items():
        raw = env.get(name, "").strip()
        if not raw:
            raise ValueError("%s is required when %s=1" % (name, ENV_SWITCH))
        try:
            value = float(raw)
        except ValueError:
            raise ValueError("%s is not a number: %r" % (name, raw)) from None
        if not (math.isfinite(value) and lo <= value <= hi):
            raise ValueError("%s=%r outside [%g, %g]" % (name, raw, lo, hi))
        out[name] = value
    return out


def blind_mask(command_xy, half_fov_rad):
    """True where a horizontal vehicle-frame command points outside the camera's horizontal FOV and is moving."""
    import torch

    bearing = torch.atan2(command_xy[:, 1], command_xy[:, 0]).abs()
    return (bearing > float(half_fov_rad)) & (command_xy.norm(dim=1) > 1e-6)


def cap_blind(command_xy, half_fov_rad, blind_mps):
    """Scale out-of-FOV commands to at most blind_mps, keeping their direction. Returns (capped, mask)."""
    import torch

    mask = blind_mask(command_xy, half_fov_rad)
    norm = command_xy.norm(dim=1).clamp(min=1e-9)
    scale = torch.where(mask, (float(blind_mps) / norm).clamp(max=1.0), torch.ones_like(norm))
    return command_xy * scale.unsqueeze(1), mask


def blind_penalty(vel_xy, half_fov_rad, weight, speed_ref):
    """Per-env penalty (>= 0) for measured motion outside the camera's horizontal FOV."""
    import torch

    half = float(half_fov_rad)
    speed = vel_xy.norm(dim=1)
    bearing = torch.atan2(vel_xy[:, 1], vel_xy[:, 0]).abs()
    excess = (bearing - half).clamp(min=0.0) / max(1e-6, math.pi - half)
    gate = (speed / max(1e-6, float(speed_ref))).clamp(0.0, 1.0)
    excess = torch.where(speed > 1e-6, excess, torch.zeros_like(excess))
    return float(weight) * excess * gate


def apply_reward_override(task_config, s):
    before = float(task_config.reward_parameters["collision_penalty"])
    task_config.reward_parameters["collision_penalty"] = float(s["NAVRL_CAMSAFE_COLLISION_PENALTY"])
    return {"collision_penalty_before": before,
            "collision_penalty_after": float(task_config.reward_parameters["collision_penalty"])}


def static_safety_r_ss(dist_m, seg, clip_m, target_label=50):
    """The NavRL r_ss of NavRLTask.add_static_safety_reward with the range constant `clip_m` in place of
    lidar_max_range: mean over rays of log(min(d, clip)) - log(clip); target returns count as no return."""
    import torch

    d = dist_m.clamp(min=1e-6, max=float(clip_m))
    if seg is not None:
        d = torch.where(seg == target_label, torch.full_like(d, float(clip_m)), d)
    return (torch.log(d) - math.log(float(clip_m))).mean(dim=1)


def install_task(task_cls, s):
    """Wrap `_apply_sensor_speed_governor` (outermost), `compute_state_reward_and_terminations` and replace
    `add_static_safety_reward` by the clipped-range version."""
    import torch

    if getattr(task_cls, "_camsafe_settings", None) is not None:
        raise RuntimeError("camsafe already installed on %r" % task_cls)
    if getattr(task_cls, "_sensor_ablation_condition", None) is None:
        raise RuntimeError("camsafe must be installed after the sensor ablation")
    blind_mps = float(s["NAVRL_CAMSAFE_BLIND_MPS"])
    weight = float(s["NAVRL_CAMSAFE_BLIND_WEIGHT"])
    clip_m = float(s["NAVRL_CAMSAFE_RSS_CLIP_M"])
    orig_governor = task_cls._apply_sensor_speed_governor
    orig_reward = task_cls.compute_state_reward_and_terminations
    orig_rss = task_cls.add_static_safety_reward
    # Counters live on the tensors' device and are read only when a report is printed (review r1 P3: no host sync
    # per step).
    stats = {"gov_calls": 0, "acc": None}

    def _acc(device):
        if stats["acc"] is None:
            stats["acc"] = torch.zeros(6, dtype=torch.float64, device=device)
        return stats["acc"]

    def _report():
        acc = stats["acc"].tolist() if stats["acc"] is not None else [0.0] * 6
        cmds, capped, alive, penalised, pen_sum, rss_sum = acc
        print("[camsafe] calls=%d capped_frac=%.4f penalised_frac=%.4f mean_penalty=%.5f mean_r_ss_clip=%.4f "
              "blind_mps=%g weight=%g rss_clip_m=%g" % (stats["gov_calls"], capped / max(1.0, cmds),
                                                        penalised / max(1.0, alive), pen_sum / max(1.0, alive),
                                                        rss_sum / max(1.0, alive), blind_mps, weight, clip_m),
              flush=True)

    def _apply_sensor_speed_governor(self, command_xy):
        governed = orig_governor(self, command_xy)
        half = float(self.perception.hfov) * 0.5
        capped, mask = cap_blind(governed, half, blind_mps)
        acc = _acc(capped.device)
        acc[0] += mask.numel()
        acc[1] += mask.sum()
        stats["gov_calls"] += 1
        n = stats["gov_calls"]
        if n in REPORT_AT_CALLS or n % REPORT_EVERY_CALLS == 0:
            _report()
        return capped

    def compute_state_reward_and_terminations(self, *args, **kwargs):
        out = orig_reward(self, *args, **kwargs)
        if weight > 0.0:
            vel_xy = self.obs_dict["robot_vehicle_linvel"][:, :2]
            half = float(self.perception.hfov) * 0.5
            pen = blind_penalty(vel_xy, half, weight, self.task_config.yaw_align_speed_ref)
            alive = (self.terminations <= 0) & (self.truncations <= 0)
            self.rewards[alive] -= pen[alive].to(self.rewards.dtype)
            acc = _acc(pen.device)
            acc[2] += alive.sum()
            acc[3] += ((pen > 0) & alive).sum()
            acc[4] += pen[alive].sum()
        return out

    def add_static_safety_reward(self):
        # NavRLTask.add_static_safety_reward with lidar_max_range -> clip_m (the only change).
        dist_m = self._lidar_distance_m()
        seg = None
        if self.vision_mode:
            seg = self.obs_dict["segmentation_pixels"].squeeze(1).reshape(self.num_envs, -1)
        r_ss = static_safety_r_ss(dist_m, seg, clip_m)
        alive = (self.terminations <= 0) & (self.truncations <= 0)
        self.rewards[alive] += self.rw["safety_static_weight"] * r_ss[alive]
        _acc(r_ss.device)[5] += r_ss[alive].sum()

    task_cls._apply_sensor_speed_governor = _apply_sensor_speed_governor
    task_cls.compute_state_reward_and_terminations = compute_state_reward_and_terminations
    task_cls.add_static_safety_reward = add_static_safety_reward
    task_cls._camsafe_settings = dict(s)
    task_cls._camsafe_stats = stats
    _INSTALLED["task"] = (task_cls, orig_governor, orig_reward, orig_rss)
    return {"blind_mps": blind_mps, "blind_weight": weight, "rss_clip_m": clip_m,
            "wrapped": ["_apply_sensor_speed_governor", "compute_state_reward_and_terminations"],
            "replaced": ["add_static_safety_reward"]}


def install(s, task_config):
    """Production install (tools/realveh_entry.py), after the ablation and the CAM-K1 overrides."""
    from aerial_gym.task.navrl_task import navrl_task

    info = install_task(navrl_task.NavRLTask, s)
    info.update(apply_reward_override(task_config, s))
    info["settings"] = dict(s)
    return info


def uninstall():
    """Tests only."""
    task = _INSTALLED.pop("task", None)
    if task is not None:
        cls, gov, rew, rss = task
        cls._apply_sensor_speed_governor = gov
        cls.compute_state_reward_and_terminations = rew
        cls.add_static_safety_reward = rss
        for name in ("_camsafe_settings", "_camsafe_stats"):
            if name in cls.__dict__:
                delattr(cls, name)
