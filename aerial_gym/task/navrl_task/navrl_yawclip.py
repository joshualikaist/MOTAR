"""Opt-in yaw-rate command clip for the CAM-S2a evaluation (draft for docs/plans/prereg_camonly_s2a_eval_2026-11-24.md).
Installed by tools/realveh_entry.py only when NAVRL_YAWCLIP=1, after navrl_cams2 (so that it acts on the delayed,
governed command the task is about to execute) and before navrl_cams2diag (so that the recorder sees the clipped
command).

Why: the real vehicle limits its yaw rate to 60 deg/s (1.047 rad/s). CAM-S2a trained with NAVRL_YAW_RATE_MAX=3.0:
action[:, 3] in [-1, 1] maps to +-3.0 rad/s. This module clips the COMMAND, command[:, 3], to +-NAVRL_YAWCLIP_RADPS at
the end of transform_action_to_command. The action scale stays 3.0: the observation normaliser
(robot_body_angvel / yaw_rate_max), the previous-action channel (the raw action a_t, navrl_task.py:5053) and the
controller clamp (lee_controller_config_navrl.max_yaw_rate) are untouched. So the policy still believes it can turn at
3.0 rad/s; the vehicle turns at most at the clip. This is an evaluation condition, not a trained rule.

Telemetry ([yawclip] lines on stderr at REPORT_AT_CALLS and every REPORT_EVERY_CALLS calls; each line is the running
total through its call count, so the last line of a cell is the whole-cell value to within 320 calls): the number of
commands, the fraction clipped, mean |yaw| before and after. The analysis takes the clipped share from the last line and the
exact whole-cell binding share from navrl_cams2diag's per-episode rows (yaw_request_over_ref_steps, recorded in clipped and
unclipped cells alike); the preregistration fixes a minimum binding share for a gate-Y label.
navrl_task.py and navrl_perception.py are byte-pinned, so the method is wrapped at run time.
"""
import math
import os
import sys

ENV_SWITCH = "NAVRL_YAWCLIP"
PARAMETERS = {"NAVRL_YAWCLIP_RADPS": (0.1, 3.0)}
REPORT_AT_CALLS = (32, 320)
REPORT_EVERY_CALLS = 320          # review r1 P1-2 / P2-10: the last line covers the cell to within 320 calls
LOG_PREFIX = "[yawclip]"
_INSTALLED = {}


def settings_from_env(environ=None):
    """Validated settings (stdlib only); None when the switch is off. Refuses a stray parameter while off, a missing,
    non-numeric, non-finite or out-of-range value, and a switch other than unset / 0 / 1."""
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


def clip_yaw(command, limit):
    """command [N, >=4]; clamps column 3 in place to +-limit. Returns the number of rows that were clipped."""
    yaw = command[:, 3]
    clipped = int((yaw.abs() > float(limit)).sum().item())
    command[:, 3] = yaw.clamp(-float(limit), float(limit))
    return clipped


def install_task(task_cls, s, task_config):
    """Wrap transform_action_to_command (outermost at install time). Refuses a second install and a clip that does not
    bind below the task's yaw_rate_max (the condition would be vacuous)."""
    import torch

    if getattr(task_cls, "_yawclip_settings", None) is not None:
        raise RuntimeError("yawclip already installed on %r" % task_cls)
    limit = float(s["NAVRL_YAWCLIP_RADPS"])
    scale = float(task_config.yaw_rate_max)
    if not limit < scale:
        raise RuntimeError("NAVRL_YAWCLIP_RADPS %g must be below task_config.yaw_rate_max %g" % (limit, scale))
    orig_transform = task_cls.transform_action_to_command
    stats = {"calls": 0, "commands": 0, "clipped": 0, "abs_before": 0.0, "abs_after": 0.0}
    stream = sys.stderr          # bound now: rl_games' quiet startup swaps the streams while the task is built

    def _report():
        n = max(1, stats["commands"])
        stream.write("%s calls=%d commands=%d clipped_frac=%.4f mean_abs_yaw_before=%.4f mean_abs_yaw_after=%.4f "
                     "limit_radps=%g scale_radps=%g\n" % (LOG_PREFIX, stats["calls"], stats["commands"],
                                                          stats["clipped"] / n, stats["abs_before"] / n,
                                                          stats["abs_after"] / n, limit, scale))
        stream.flush()

    def transform_action_to_command(self, actions):
        command = orig_transform(self, actions)
        before = command[:, 3].abs().sum(dtype=torch.float64).item()
        clipped = clip_yaw(command, limit)
        stats["calls"] += 1
        stats["commands"] += int(command.shape[0])
        stats["clipped"] += clipped
        stats["abs_before"] += float(before)
        stats["abs_after"] += float(command[:, 3].abs().sum(dtype=torch.float64).item())
        n = stats["calls"]
        if n in REPORT_AT_CALLS or n % REPORT_EVERY_CALLS == 0:
            _report()
        return command

    task_cls.transform_action_to_command = transform_action_to_command
    task_cls._yawclip_settings = dict(s)
    task_cls._yawclip_stats = stats
    _INSTALLED["task"] = (task_cls, orig_transform)
    return {"limit_radps": limit, "scale_radps": scale, "wrapped": ["transform_action_to_command"]}


def install(s, task_config):
    """Production install (tools/realveh_entry.py), after navrl_cams2."""
    from aerial_gym.task.navrl_task import navrl_task

    info = install_task(navrl_task.NavRLTask, s, task_config)
    info["settings"] = dict(s)
    return info


def uninstall():
    """Tests only."""
    task = _INSTALLED.pop("task", None)
    if task is not None:
        cls, tr = task
        cls.transform_action_to_command = tr
        for name in ("_yawclip_settings", "_yawclip_stats"):
            if name in cls.__dict__:
                delattr(cls, name)
