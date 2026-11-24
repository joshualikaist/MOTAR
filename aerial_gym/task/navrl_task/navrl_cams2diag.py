"""Opt-in per-episode recorder for the CAM-S2a evaluation (draft for docs/plans/prereg_camonly_s2a_eval_2026-11-24.md;
review r1 of the trained unit, F4 / F6 / F7, ~/motar_tmp/cams2a_20261123/review_r1/REVIEW_r1.md). Installed by
tools/realveh_entry.py only when NAVRL_CAMS2DIAG=1, after navrl_cams2 (and after navrl_yawclip when that is on), so its
wrappers are the OUTERMOST and see what the task will execute and record. It changes no reward, termination, action or
observation: it reads, accumulates and writes.

Writes one JSON row per finished episode to NAVRL_CAMS2DIAG_JSONL (refusing an existing file), aligned with the task's
episode ledger: the same (env_index, env_episode_index) because both count the same finished set in the same step
(navrl_task.py:5076-5083 derives successes = captured_now, crashes = crashed_now, timeouts from the step limit; the
ledger records them at :5150). Fields:
  outcome          capture | crash | timeout, as the ledger will record them;
  crash_class      target_contact_swept  the cams2 wrapper's conversion of the task's swept test at the contact radius
                                         (its _crash_cause_code stays -1, review F6);
                   contact               the task's cause 0: bar contact (and a PhysX contact with the target mesh);
                   below | above | out_of_bounds   the task's causes 1-3;   none for a non-crash;
  delay_k          the episode's action delay (navrl_cams2);
  episode_steps    control steps of the episode;
  hidden_steps / visible_steps   steps whose observation had no / a detection (the task's _visible_now, the flag the
                   policy acted on and the one the cams2 capture rule uses, review F5);
  hidden_* / visible_*  what the policy ISSUED while the detection channel was empty / filled: means over those steps of
                   the raw action mapped to vehicle-frame velocity (vx, |vy|, speed) and yaw rate (|yaw|, signed yaw),
                   and of the EXECUTED command (after delay, governor, blind cap and yaw clip): speed and |yaw|;
                   hidden_exec_over_blind_cap  steps with an executed horizontal speed above NAVRL_CAMSAFE_BLIND_MPS
                   (an executed command cannot exceed the cap when it points outside the FOV; above it means the command
                   pointed inside the FOV while the target was not detected);
  yaw_request_over_ref_steps / yaw_exec_over_ref_steps   steps whose requested (raw action x yaw_rate_max) / executed yaw
                   rate exceeds NAVRL_CAMS2DIAG_YAW_REF_RADPS (the clip limit): the clip's binding share, measured in the
                   unclipped arm too; under the clip the executed count must be 0;
  capture_dist_m, capture_closing_mps, capture_rel_speed_mps, capture_lateral_mps   at a capture: centre distance,
                   radial closing speed (positive = approaching), |v_robot - v_target| and its tangential part
                   (review F7: the rule bounds only the radial part).
A summary line ([cams2diag] JSON on stderr) at REPORT_AT_CALLS and every REPORT_EVERY_CALLS steps carries the running
totals and navrl_cams2's own counters (task_cls._cams2_stats, which training never wrote out, review F4).
navrl_task.py and navrl_perception.py are byte-pinned, so the methods are wrapped at run time.
"""
import json
import math
import os
import sys

ENV_SWITCH = "NAVRL_CAMS2DIAG"
PATH_ENV = "NAVRL_CAMS2DIAG_JSONL"
YAW_REF_ENV = "NAVRL_CAMS2DIAG_YAW_REF_RADPS"     # the yaw-rate reference the over-reference counters use (the clip limit)
YAW_REF_RANGE = (0.1, 3.0)
YAW_TOL = 1e-5                      # review r2 P0-2: a float32 clip at 1.047 reads 1.0470000505; compare with a tolerance
REPORT_AT_CALLS = (32, 320)
REPORT_EVERY_CALLS = 320          # review r1 P2-10: summary lines are running totals through their step count
LOG_PREFIX = "[cams2diag]"
CAUSES = {-1: "target_contact_swept", 0: "contact", 1: "below", 2: "above", 3: "out_of_bounds"}
_INSTALLED = {}


def settings_from_env(environ=None):
    """None when off; refuses a stray path while off, a missing or relative path, an existing file and a switch other
    than unset / 0 / 1."""
    env = os.environ if environ is None else environ
    switch = env.get(ENV_SWITCH, "").strip()
    if switch not in ("", "0", "1"):
        raise ValueError("%s must be unset, 0 or 1, got %r" % (ENV_SWITCH, switch))
    path = env.get(PATH_ENV, "").strip()
    ref = env.get(YAW_REF_ENV, "").strip()
    if switch != "1":
        stray = [k for k, v in ((PATH_ENV, path), (YAW_REF_ENV, ref)) if v]
        if stray:
            raise ValueError("%s set while %s is off" % (", ".join(stray), ENV_SWITCH))
        return None
    if not path or not os.path.isabs(path):
        raise ValueError("%s must be an absolute path when %s=1" % (PATH_ENV, ENV_SWITCH))
    if os.path.exists(path):
        raise ValueError("%s exists; refusing to overwrite" % path)
    try:
        ref_value = float(ref)
    except ValueError:
        raise ValueError("%s is required (a number) when %s=1, got %r" % (YAW_REF_ENV, ENV_SWITCH, ref)) from None
    if not (math.isfinite(ref_value) and YAW_REF_RANGE[0] <= ref_value <= YAW_REF_RANGE[1]):
        raise ValueError("%s=%r outside [%g, %g]" % (YAW_REF_ENV, ref, YAW_REF_RANGE[0], YAW_REF_RANGE[1]))
    return {PATH_ENV: path, YAW_REF_ENV: ref_value}


def capture_kinematics(rel, vel_rel):
    """rel = robot - target [N, 3], vel_rel = v_robot - v_target [N, 3] -> (dist, closing, rel_speed, lateral)."""
    dist = rel.norm(dim=1)
    closing = -(rel * vel_rel).sum(dim=1) / dist.clamp(min=1e-6)
    rel_speed = vel_rel.norm(dim=1)
    lateral = (rel_speed.pow(2) - closing.pow(2)).clamp(min=0.0).sqrt()
    return dist, closing, rel_speed, lateral


def classify(crashed, captured, cause_code):
    """Per env: 'capture', 'crash:<class>' or '' (not finished here); the cams2 contact keeps cause -1."""
    out = []
    for cr, ca, code in zip(crashed.tolist(), captured.tolist(), cause_code.tolist()):
        if ca:
            out.append("capture")
        elif cr:
            out.append("crash:" + CAUSES.get(int(code), "unknown"))
        else:
            out.append("")
    return out


def install_task(task_cls, s, blind_mps):
    import torch

    if getattr(task_cls, "_cams2diag_settings", None) is not None:
        raise RuntimeError("cams2diag already installed on %r" % task_cls)
    if getattr(task_cls, "_cams2_settings", None) is None:
        raise RuntimeError("cams2diag must be installed after navrl_cams2")
    path = s[PATH_ENV]
    yaw_ref = float(s[YAW_REF_ENV])
    orig_transform = task_cls.transform_action_to_command
    orig_reward = task_cls.compute_state_reward_and_terminations
    stream = sys.stderr
    st = {"steps": 0, "rows": 0, "acc": None, "ep_index": None, "ep_steps": None, "handle": None,
          "totals": {"capture": 0, "crash": 0, "timeout": 0, "hidden_steps": 0, "visible_steps": 0},
          "classes": {}}
    blind = float(blind_mps)
    # per-env accumulators: 0 hidden steps, 1 visible steps, 2 hid raw vx, 3 hid raw |vy|, 4 hid raw speed,
    # 5 hid raw |yaw|, 6 hid raw yaw, 7 hid exec speed, 8 hid exec |yaw|, 9 hid exec over blind cap,
    # 10 vis raw speed, 11 vis raw |yaw|, 12 vis exec speed, 13 vis exec |yaw|,
    # 14 steps with |requested yaw| > yaw_ref (the clip's binding share, clipped and unclipped cells alike, review P1-2),
    # 15 steps with |executed yaw| > yaw_ref (must be 0 under the clip)
    NACC = 16

    def _ensure(self, n, device):
        if st["acc"] is None or st["acc"].shape[0] != n:
            st["acc"] = torch.zeros((n, NACC), dtype=torch.float64, device=device)
            st["ep_index"] = [0] * n
            st["ep_steps"] = torch.zeros(n, dtype=torch.long, device=device)
        if st["handle"] is None:
            st["handle"] = open(path, "x", encoding="utf-8")

    def transform_action_to_command(self, actions):
        command = orig_transform(self, actions)
        _ensure(self, actions.shape[0], actions.device)
        acc = st["acc"]
        vmax = float(self.task_config.max_velocity)
        ymax = float(self.task_config.yaw_rate_max)
        raw_v = actions[:, 0:3].clamp(-1.0, 1.0).to(torch.float64) * vmax
        raw_yaw = actions[:, 3].clamp(-1.0, 1.0).to(torch.float64) * ymax
        exec_speed = command[:, 0:2].to(torch.float64).norm(dim=1)
        exec_yaw = command[:, 3].to(torch.float64).abs()
        hidden = ~self._visible_now.bool()
        h = hidden.to(torch.float64)
        v = 1.0 - h
        acc[:, 0] += h
        acc[:, 1] += v
        acc[:, 2] += h * raw_v[:, 0]
        acc[:, 3] += h * raw_v[:, 1].abs()
        acc[:, 4] += h * raw_v[:, 0:2].norm(dim=1)
        acc[:, 5] += h * raw_yaw.abs()
        acc[:, 6] += h * raw_yaw
        acc[:, 7] += h * exec_speed
        acc[:, 8] += h * exec_yaw
        acc[:, 9] += h * (exec_speed > blind).to(torch.float64)
        acc[:, 10] += v * raw_v[:, 0:2].norm(dim=1)
        acc[:, 11] += v * raw_yaw.abs()
        acc[:, 12] += v * exec_speed
        acc[:, 13] += v * exec_yaw
        acc[:, 14] += (raw_yaw.abs() > yaw_ref + YAW_TOL).to(torch.float64)
        acc[:, 15] += (exec_yaw > yaw_ref + YAW_TOL).to(torch.float64)
        st["ep_steps"] += 1
        return command

    def _mean(acc_row, idx, n):
        return (acc_row[idx] / n) if n > 0 else None

    def compute_state_reward_and_terminations(self, *args, **kwargs):
        out = orig_reward(self, *args, **kwargs)
        n = self.captured_now.shape[0]
        _ensure(self, n, self.captured_now.device)
        st["steps"] += 1
        captured = self.captured_now.bool()
        crashed = self.crashed_now.bool() if self.crashed_now.dtype == torch.bool else self.crashed_now > 0
        limit = sys.modules[type(self).__module__]._episode_limit_reached(self.sim_env.sim_steps,
                                                                           self.task_config.episode_len_steps)
        timeout = limit.bool() & ~captured & ~crashed
        finished = captured | crashed | timeout
        if not bool(finished.any()):
            n_call = st["steps"]
            if n_call in REPORT_AT_CALLS or n_call % REPORT_EVERY_CALLS == 0:
                _report(type(self))
            return out
        rel = self.obs_dict["robot_position"] - self.target_position
        vel_rel = self.obs_dict["robot_linvel"] - self.target_vel_w
        dist, closing, rel_speed, lateral = capture_kinematics(rel.to(torch.float64), vel_rel.to(torch.float64))
        classes = classify(crashed, captured, self._crash_cause_code)
        delay = getattr(self, "_cams2_delay", None)
        acc = st["acc"].tolist()
        ep_steps = st["ep_steps"].tolist()
        for env in torch.nonzero(finished, as_tuple=False).flatten().tolist():
            a = acc[env]
            hid, vis = int(a[0]), int(a[1])
            cls = classes[env]
            outcome = "capture" if cls == "capture" else ("crash" if cls.startswith("crash:") else "timeout")
            row = {"env_index": env, "env_episode_index": st["ep_index"][env], "completion_step": st["steps"],
                   "outcome": outcome, "crash_class": cls[6:] if cls.startswith("crash:") else "none",
                   "delay_k": int(delay[env].item()) if delay is not None else None, "episode_steps": int(ep_steps[env]),
                   "hidden_steps": hid, "visible_steps": vis,
                   "hidden_raw_vx_mps": _mean(a, 2, hid), "hidden_raw_abs_vy_mps": _mean(a, 3, hid),
                   "hidden_raw_speed_mps": _mean(a, 4, hid), "hidden_raw_abs_yaw_radps": _mean(a, 5, hid),
                   "hidden_raw_yaw_radps": _mean(a, 6, hid), "hidden_exec_speed_mps": _mean(a, 7, hid),
                   "hidden_exec_abs_yaw_radps": _mean(a, 8, hid), "hidden_exec_over_blind_cap": int(a[9]),
                   "visible_raw_speed_mps": _mean(a, 10, vis), "visible_raw_abs_yaw_radps": _mean(a, 11, vis),
                   "visible_exec_speed_mps": _mean(a, 12, vis), "visible_exec_abs_yaw_radps": _mean(a, 13, vis),
                   "yaw_request_over_ref_steps": int(a[14]), "yaw_exec_over_ref_steps": int(a[15]), "yaw_ref_radps": yaw_ref,
                   "capture_dist_m": float(dist[env]) if outcome == "capture" else None,
                   "capture_closing_mps": float(closing[env]) if outcome == "capture" else None,
                   "capture_rel_speed_mps": float(rel_speed[env]) if outcome == "capture" else None,
                   "capture_lateral_mps": float(lateral[env]) if outcome == "capture" else None}
            st["handle"].write(json.dumps(row, sort_keys=True) + "\n")
            st["rows"] += 1
            st["ep_index"][env] += 1
            st["totals"][outcome] += 1
            st["totals"]["hidden_steps"] += hid
            st["totals"]["visible_steps"] += vis
            st["classes"][row["crash_class"]] = st["classes"].get(row["crash_class"], 0) + 1
        st["handle"].flush()
        fin = finished.to(torch.float64).unsqueeze(1)
        st["acc"].mul_(1.0 - fin)
        st["ep_steps"][finished] = 0
        n_call = st["steps"]
        if n_call in REPORT_AT_CALLS or n_call % REPORT_EVERY_CALLS == 0:
            _report(type(self))
        return out

    def _report(cls):
        payload = {"event": "summary", "snapshot": "running totals through this step count, not a final line",
                   "steps": st["steps"], "rows": st["rows"], "totals": dict(st["totals"]),
                   "crash_classes": dict(st["classes"]), "cams2_stats": dict(getattr(cls, "_cams2_stats", None) or {})}
        stream.write("%s %s\n" % (LOG_PREFIX, json.dumps(payload, sort_keys=True)))
        stream.flush()

    task_cls.transform_action_to_command = transform_action_to_command
    task_cls.compute_state_reward_and_terminations = compute_state_reward_and_terminations
    task_cls._cams2diag_settings = dict(s)
    task_cls._cams2diag_stats = st
    _INSTALLED["task"] = (task_cls, orig_transform, orig_reward)
    return {"jsonl": path, "blind_mps": blind, "yaw_ref_radps": yaw_ref,
            "wrapped": ["transform_action_to_command", "compute_state_reward_and_terminations"]}


def telemetry_lines(texts, prefix=LOG_PREFIX):
    """The JSON payloads of the recorder's summary lines in `texts`."""
    out = []
    for text in texts:
        for line in text.splitlines():
            if line.startswith(prefix + " {"):
                try:
                    out.append(json.loads(line[len(prefix) + 1:]))
                except ValueError:
                    pass
    return out


def install(s, blind_mps):
    """Production install (tools/realveh_entry.py), after navrl_cams2 (and navrl_yawclip when on)."""
    from aerial_gym.task.navrl_task import navrl_task

    info = install_task(navrl_task.NavRLTask, s, blind_mps)
    info["settings"] = dict(s)
    return info


def uninstall():
    """Tests only."""
    task = _INSTALLED.pop("task", None)
    if task is not None:
        cls, tr, rw = task
        cls.transform_action_to_command, cls.compute_state_reward_and_terminations = tr, rw
        st = getattr(cls, "_cams2diag_stats", None)
        if st and st.get("handle"):
            st["handle"].close()
        for name in ("_cams2diag_settings", "_cams2diag_stats"):
            if name in cls.__dict__:
                delattr(cls, name)
