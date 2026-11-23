"""Opt-in CAM-S2a training rules (docs/plans/prereg_camonly_s2a_2026-11-23.md; rule sheet
~/motar_tmp/cams2_design_20261123/RULES.md R3, R7, R16, R23). Installed by tools/realveh_entry.py only when
NAVRL_CAMS2=1, after the sensor ablation and navrl_camsafe.

  1. Action delay (R7): each environment draws a delay of k control steps, k uniform in [DELAY_MIN, DELAY_MAX], at
     every reset; the command applied at step t is the policy's action of step t - k (zero before the episode's first
     action). Models capture-to-command latency of the real vehicle (camera 35-77 ms + processing + MAVROS).
     NAVRL_ACTION_DELAY_STEPS_MAX must equal DELAY_MAX so the training record carries the delay the flight runtime
     reads (camera_policy_contract.capture_age_bound_s).
  2. Capture and target contact (R3): the task's own swept test runs with success_radius = CONTACT_RADIUS_M and is
     turned into a COLLISION with the target (collision penalty, end of episode). Capture is then: centre distance
     <= CAPTURE_RADIUS_M AND closing speed <= CAPTURE_MAX_CLOSING_MPS AND the target detected this step
     (perception's visibility), for envs still alive; it adds the capture bonus and ends the episode.
  3. Flight altitude and camera mount (R16, R23): task_config.flight_altitude and vision.camera_translation[0].
navrl_task.py and navrl_perception.py are byte-pinned, so their methods are wrapped at run time.
"""
import math
import os

ENV_SWITCH = "NAVRL_CAMS2"
DELAY_RECORD_ENV = "NAVRL_ACTION_DELAY_STEPS_MAX"
PARAMETERS = {
    "NAVRL_CAMS2_DELAY_MIN": (0, 3, int),
    "NAVRL_CAMS2_DELAY_MAX": (0, 3, int),
    "NAVRL_CAMS2_DELAY_SEED": (0, 2 ** 31 - 1, int),
    "NAVRL_CAMS2_CAPTURE_RADIUS_M": (0.5, 3.0, float),
    "NAVRL_CAMS2_CAPTURE_MAX_CLOSING_MPS": (0.1, 5.0, float),
    "NAVRL_CAMS2_CONTACT_RADIUS_M": (0.2, 1.0, float),
    "NAVRL_CAMS2_FLIGHT_ALTITUDE_M": (0.5, 2.5, float),
    "NAVRL_CAMS2_CAMERA_X_M": (0.0, 0.3, float),
}
_INSTALLED = {}


def settings_from_env(environ=None):
    """Validated settings (stdlib only); None when the switch is off. Refuses stray parameters, missing / malformed /
    out-of-range values, DELAY_MIN > DELAY_MAX, CONTACT >= CAPTURE radius and a delay record that differs."""
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
    for name, (lo, hi, kind) in PARAMETERS.items():
        raw = env.get(name, "").strip()
        if not raw:
            raise ValueError("%s is required when %s=1" % (name, ENV_SWITCH))
        try:
            value = int(raw) if kind is int else float(raw)
        except ValueError:
            raise ValueError("%s is not a %s: %r" % (name, kind.__name__, raw)) from None
        if kind is int and not (raw.isascii() and raw.isdigit()):
            raise ValueError("%s must be a non-negative integer, got %r" % (name, raw))
        if not (math.isfinite(value) and lo <= value <= hi):
            raise ValueError("%s=%r outside [%g, %g]" % (name, raw, lo, hi))
        out[name] = value
    if out["NAVRL_CAMS2_DELAY_MIN"] > out["NAVRL_CAMS2_DELAY_MAX"]:
        raise ValueError("NAVRL_CAMS2_DELAY_MIN exceeds NAVRL_CAMS2_DELAY_MAX")
    if out["NAVRL_CAMS2_CONTACT_RADIUS_M"] >= out["NAVRL_CAMS2_CAPTURE_RADIUS_M"]:
        raise ValueError("the contact radius must be smaller than the capture radius")
    if env.get(DELAY_RECORD_ENV, "").strip() != str(out["NAVRL_CAMS2_DELAY_MAX"]):
        raise ValueError("%s must equal NAVRL_CAMS2_DELAY_MAX (%d) so the training record carries the delay"
                         % (DELAY_RECORD_ENV, out["NAVRL_CAMS2_DELAY_MAX"]))
    return out


def apply_config(task_config, s):
    before = {"success_radius": float(task_config.success_radius),
              "flight_altitude": float(task_config.flight_altitude),
              "camera_translation": list(task_config.vision.camera_translation)}
    task_config.success_radius = float(s["NAVRL_CAMS2_CONTACT_RADIUS_M"])
    task_config.flight_altitude = float(s["NAVRL_CAMS2_FLIGHT_ALTITUDE_M"])
    cam = list(task_config.vision.camera_translation)
    cam[0] = float(s["NAVRL_CAMS2_CAMERA_X_M"])
    task_config.vision.camera_translation = cam
    after = {"success_radius": float(task_config.success_radius),
             "flight_altitude": float(task_config.flight_altitude),
             "camera_translation": list(task_config.vision.camera_translation)}
    return {"config_before": before, "config_after": after}


def delayed(history, delay):
    """history [N, K, A] with history[:, 0] the newest action; delay [N] in [0, K-1]. Returns history[n, delay[n]]."""
    import torch

    idx = delay.long().clamp(0, history.shape[1] - 1)
    return history[torch.arange(history.shape[0], device=history.device), idx]


def capture_mask(rel, vel_rel, visible, alive, radius, max_closing):
    """Envs that capture: alive, centre distance <= radius, closing speed <= max_closing, target detected."""
    import torch

    dist = rel.norm(dim=1)
    closing = -(rel * vel_rel).sum(dim=1) / dist.clamp(min=1e-6)
    return alive & visible & (dist <= float(radius)) & (closing <= float(max_closing))


def install_task(task_cls, s, task_config):
    import torch

    if getattr(task_cls, "_cams2_settings", None) is not None:
        raise RuntimeError("cams2 already installed on %r" % task_cls)
    if getattr(task_cls, "_camsafe_settings", None) is None:
        raise RuntimeError("cams2 must be installed after navrl_camsafe")
    dmin, dmax = int(s["NAVRL_CAMS2_DELAY_MIN"]), int(s["NAVRL_CAMS2_DELAY_MAX"])
    radius = float(s["NAVRL_CAMS2_CAPTURE_RADIUS_M"])
    max_closing = float(s["NAVRL_CAMS2_CAPTURE_MAX_CLOSING_MPS"])
    seed = int(s["NAVRL_CAMS2_DELAY_SEED"])
    orig_transform = task_cls.transform_action_to_command
    orig_reset = task_cls.reset_idx
    orig_reward = task_cls.compute_state_reward_and_terminations
    stats = {"contacts": 0, "captures": 0, "steps": 0}

    def _ensure(self, actions):
        hist = getattr(self, "_cams2_hist", None)
        if hist is None or hist.shape[0] != actions.shape[0] or hist.shape[2] != actions.shape[1]:
            self._cams2_hist = torch.zeros((actions.shape[0], dmax + 1, actions.shape[1]),
                                           dtype=actions.dtype, device=actions.device)
            self._cams2_gen = torch.Generator(device=actions.device)
            self._cams2_gen.manual_seed(seed)
            self._cams2_delay = torch.randint(dmin, dmax + 1, (actions.shape[0],), generator=self._cams2_gen,
                                              device=actions.device)

    def transform_action_to_command(self, actions):
        _ensure(self, actions)
        self._cams2_hist = torch.roll(self._cams2_hist, 1, dims=1)
        self._cams2_hist[:, 0] = actions
        return orig_transform(self, delayed(self._cams2_hist, self._cams2_delay))

    def reset_idx(self, env_ids):
        out = orig_reset(self, env_ids)
        hist = getattr(self, "_cams2_hist", None)
        if hist is not None and env_ids is not None and len(env_ids) > 0:
            ids = torch.as_tensor(env_ids, device=hist.device).long()
            hist[ids] = 0.0
            self._cams2_delay[ids] = torch.randint(dmin, dmax + 1, (ids.numel(),), generator=self._cams2_gen,
                                                   device=hist.device)
        return out

    def compute_state_reward_and_terminations(self, *args, **kwargs):
        out = orig_reward(self, *args, **kwargs)
        contact = self.captured_now.clone()            # the task's swept test at the contact radius
        if bool(contact.any()):
            self.rewards[contact] = float(self.rw["collision_penalty"])
            self.captured_now = self.captured_now & ~contact
            self.crashed_now = self.crashed_now | contact
        alive = self.terminations <= 0
        rel = self.obs_dict["robot_position"] - self.target_position
        vel_rel = self.obs_dict["robot_linvel"] - self.target_vel_w
        cap = capture_mask(rel, vel_rel, self._visible_now.bool(), alive, radius, max_closing)
        if bool(cap.any()):
            self.rewards[cap] = self.rewards[cap] + float(self.rw["capture_bonus"])
            self.terminations[cap] = 1
            self.captured_now = self.captured_now | cap
        stats["steps"] += 1
        stats["contacts"] += int(contact.sum().item())
        stats["captures"] += int(cap.sum().item())
        return out

    task_cls.transform_action_to_command = transform_action_to_command
    task_cls.reset_idx = reset_idx
    task_cls.compute_state_reward_and_terminations = compute_state_reward_and_terminations
    task_cls._cams2_settings = dict(s)
    task_cls._cams2_stats = stats
    _INSTALLED["task"] = (task_cls, orig_transform, orig_reset, orig_reward)
    info = {"delay_steps": [dmin, dmax], "capture_radius_m": radius, "capture_max_closing_mps": max_closing,
            "wrapped": ["transform_action_to_command", "reset_idx", "compute_state_reward_and_terminations"]}
    info.update(apply_config(task_config, s))
    return info


def install(s, task_config):
    """Production install (tools/realveh_entry.py), after navrl_camsafe."""
    from aerial_gym.task.navrl_task import navrl_task

    info = install_task(navrl_task.NavRLTask, s, task_config)
    info["settings"] = dict(s)
    return info


def uninstall():
    """Tests only."""
    task = _INSTALLED.pop("task", None)
    if task is not None:
        cls, tr, rs, rw = task
        cls.transform_action_to_command, cls.reset_idx, cls.compute_state_reward_and_terminations = tr, rs, rw
        for name in ("_cams2_settings", "_cams2_stats"):
            if name in cls.__dict__:
                delattr(cls, name)
