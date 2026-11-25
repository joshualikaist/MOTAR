"""Opt-in CAM-S2 task rules without the camera-blind cap, and a per-episode outcome recorder, for CAM-REF (draft
preregistration ~/motar_tmp/camref_20261126/draft/prereg_camonly_ref_matched_protocol_2026-11-26.md; design survey
~/motar_tmp/camref_20261126/FEASIBILITY.md). Default OFF: nothing imports this module except tools/camref_entry.py, and it
installs nothing unless one of the switches below is 1.

Why a new module: navrl_cams2 refuses to install without navrl_camsafe (navrl_cams2.py install_task), and camsafe's
camera-blind speed cap cannot be made neutral for Policy F (camera + 360 deg LiDAR). This module carries the same three
CAM-S2 rules, each with its own switch, numerically equal to navrl_cams2 (+ navrl_cams2_contactcode) when configured the
same (tests/test_camref.py on synthetic tensors), with no camsafe dependency:

  1. CAPTURE (NAVRL_CAMREF_CAPTURE=1; CAPTURE_RADIUS_M, CAPTURE_MAX_CLOSING_MPS, CONTACT_RADIUS_M, DETECT_SOURCE):
     task_config.success_radius = CONTACT_RADIUS_M, so the task's own swept capture test runs at the contact radius; every
     env it marks is turned into a CRASH with the target (collision penalty, captured_now cleared, crashed_now set), its
     crash-cause code set to 0, and, when the task's crash diagnostics are on, the descriptive tallies updated exactly as
     navrl_cams2_contactcode does (the task's strata validator accepts codes 0..3 only and requires the per-stratum cause
     sum to equal the crash count, navrl_task.py:8514-8545 / :9045, so a target contact cannot carry a fifth code there;
     its own class is recorded per episode by the recorder below from the mask self._camref_contact_now). Capture is then:
     alive AND centre distance <= CAPTURE_RADIUS_M AND radial closing speed <= CAPTURE_MAX_CLOSING_MPS AND detected, where
     "detected" is the task's own per-step flag named by DETECT_SOURCE: visible_now = self._visible_now (perception's
     fused visibility: camera, or LiDAR association when the LiDAR is in perception; the flag navrl_cams2 uses) or
     camera_visible_now = self._camera_visible_now (camera detection only).
  2. DELAY (NAVRL_CAMREF_DELAY=1; DELAY_MIN, DELAY_MAX, DELAY_SEED): each env draws k ~ U{DELAY_MIN..DELAY_MAX} control
     steps at every reset from this module's own torch.Generator; the command applied at step t is the action of step
     t - k (zeros before the episode's first action). The task's previous-action channel is not touched (as navrl_cams2).
  3. GEOMETRY (NAVRL_CAMREF_GEOMETRY=1; FLIGHT_ALTITUDE_M, CAMERA_X_M): task_config.flight_altitude and
     task_config.vision.camera_translation[0], set before the task is built.
Recorder (NAVRL_CAMREF_RECORD=1, NAVRL_CAMREF_RECORD_JSONL): one JSON row per finished episode, aligned with the task's
episode ledger exactly as navrl_cams2diag is (the same finished set in the same step, the same per-env counter), installed
OUTSIDE every wrapper that changes captured_now / crashed_now (the capture rule here; never inside navrl_cams2, which the
entry refuses). Fields: outcome; crash_class (target_contact_swept from this module's contact mask, else the task's cause
code: 0 contact (bar, or a physical contact with the target body), 1 below, 2 above, 3 out_of_bounds, -1 uncoded); cause_code;
delay_k (this module's or navrl_cams2's draw, None without a delay rule); episode_steps; detected_steps (steps whose
observation had the DETECT_SOURCE flag, visible_now when the capture rule is off); capture_dist_m / capture_closing_mps at
a capture. It changes no reward, termination, action or observation.
Telemetry: "[camref] {json}" lines on stderr at REPORT_AT_CALLS and every REPORT_EVERY_CALLS steps (running totals).
navrl_task.py and navrl_perception.py are byte-pinned, so methods are wrapped at run time.
"""
import json
import math
import os
import sys

CAPTURE_SWITCH = "NAVRL_CAMREF_CAPTURE"
DELAY_SWITCH = "NAVRL_CAMREF_DELAY"
GEOMETRY_SWITCH = "NAVRL_CAMREF_GEOMETRY"
RECORD_SWITCH = "NAVRL_CAMREF_RECORD"
RECORD_PATH_ENV = "NAVRL_CAMREF_RECORD_JSONL"
DETECT_ENV = "NAVRL_CAMREF_DETECT_SOURCE"
DETECT_SOURCES = {"visible_now": "_visible_now", "camera_visible_now": "_camera_visible_now"}
RULES = {
    CAPTURE_SWITCH: {"NAVRL_CAMREF_CAPTURE_RADIUS_M": (0.5, 3.0, float),
                     "NAVRL_CAMREF_CAPTURE_MAX_CLOSING_MPS": (0.1, 5.0, float),
                     "NAVRL_CAMREF_CONTACT_RADIUS_M": (0.2, 1.0, float)},
    DELAY_SWITCH: {"NAVRL_CAMREF_DELAY_MIN": (0, 3, int), "NAVRL_CAMREF_DELAY_MAX": (0, 3, int),
                   "NAVRL_CAMREF_DELAY_SEED": (0, 2 ** 31 - 1, int)},
    GEOMETRY_SWITCH: {"NAVRL_CAMREF_FLIGHT_ALTITUDE_M": (0.5, 2.5, float), "NAVRL_CAMREF_CAMERA_X_M": (0.0, 0.3, float)},
}
# The keys of tools/camref_entry.py (not this module's); every other NAVRL_CAMREF_* key is refused.
ENTRY_KEYS = ("NAVRL_CAMREF_POLICY", "NAVRL_CAMREF_RECEIPT")
KNOWN_KEYS = (set(RULES) | {k for p in RULES.values() for k in p} | {DETECT_ENV, RECORD_SWITCH, RECORD_PATH_ENV}
              | set(ENTRY_KEYS))
CONTACT_CODE = 0
CAUSES = {-1: "uncoded", 0: "contact", 1: "below", 2: "above", 3: "out_of_bounds"}
LOG_PREFIX = "[camref]"
REPORT_AT_CALLS = (1, 32, 320)
REPORT_EVERY_CALLS = 320
_INSTALLED = {}


def _switch(env, name):
    value = env.get(name, "").strip()
    if value not in ("", "0", "1"):
        raise ValueError("%s must be unset, 0 or 1, got %r" % (name, value))
    return value == "1"


def _number(env, name, lo, hi, kind):
    raw = env.get(name, "").strip()
    if not raw:
        raise ValueError("%s is required when its rule is on" % name)
    if kind is int and not (raw.isascii() and raw.isdigit()):
        raise ValueError("%s must be a non-negative integer, got %r" % (name, raw))
    try:
        value = int(raw) if kind is int else float(raw)
    except ValueError:
        raise ValueError("%s is not a %s: %r" % (name, kind.__name__, raw)) from None
    if not (math.isfinite(value) and lo <= value <= hi):
        raise ValueError("%s=%r outside [%g, %g]" % (name, raw, lo, hi))
    return value


def settings_from_env(environ=None):
    """Validated settings (stdlib only): {"capture": {...} | None, "delay": ... | None, "geometry": ... | None,
    "record": ... | None}, or None when every switch is off. Refuses an unknown NAVRL_CAMREF_* key, a parameter set while
    its switch is off, a missing / malformed / non-finite / out-of-range value, DELAY_MIN > DELAY_MAX, a contact radius not
    below the capture radius, an unknown detection source, and a missing, relative or existing recorder path."""
    env = os.environ if environ is None else environ
    unknown = sorted(k for k in env if k.startswith("NAVRL_CAMREF") and k not in KNOWN_KEYS and env[k].strip())
    if unknown:
        raise ValueError("unknown CAM-REF keys %s" % unknown)
    out = {}
    for switch, params in RULES.items():
        on = _switch(env, switch)
        names = list(params) + ([DETECT_ENV] if switch == CAPTURE_SWITCH else [])
        if not on:
            stray = sorted(k for k in names if env.get(k, "").strip())
            if stray:
                raise ValueError("%s set while %s is off" % (", ".join(stray), switch))
            out[switch] = None
            continue
        s = {k: _number(env, k, lo, hi, kind) for k, (lo, hi, kind) in params.items()}
        if switch == CAPTURE_SWITCH:
            src = env.get(DETECT_ENV, "").strip()
            if src not in DETECT_SOURCES:
                raise ValueError("%s must be one of %s, got %r" % (DETECT_ENV, sorted(DETECT_SOURCES), src))
            s[DETECT_ENV] = src
            if s["NAVRL_CAMREF_CONTACT_RADIUS_M"] >= s["NAVRL_CAMREF_CAPTURE_RADIUS_M"]:
                raise ValueError("the contact radius must be smaller than the capture radius")
        if switch == DELAY_SWITCH and s["NAVRL_CAMREF_DELAY_MIN"] > s["NAVRL_CAMREF_DELAY_MAX"]:
            raise ValueError("NAVRL_CAMREF_DELAY_MIN exceeds NAVRL_CAMREF_DELAY_MAX")
        out[switch] = s
    path = env.get(RECORD_PATH_ENV, "").strip()
    if _switch(env, RECORD_SWITCH):
        if not path or not os.path.isabs(path):
            raise ValueError("%s must be an absolute path when %s=1" % (RECORD_PATH_ENV, RECORD_SWITCH))
        if os.path.exists(path):
            raise ValueError("%s exists; refusing to overwrite" % path)
        out[RECORD_SWITCH] = {RECORD_PATH_ENV: path}
    else:
        if path:
            raise ValueError("%s set while %s is off" % (RECORD_PATH_ENV, RECORD_SWITCH))
        out[RECORD_SWITCH] = None
    if all(v is None for v in out.values()):
        return None
    return {"capture": out[CAPTURE_SWITCH], "delay": out[DELAY_SWITCH], "geometry": out[GEOMETRY_SWITCH],
            "record": out[RECORD_SWITCH]}


def rules_on(s):
    return bool(s) and any(s.get(k) is not None for k in ("capture", "delay", "geometry"))


# ---------------------------------------------------------------- pure (torch tensors; navrl_cams2's own definitions)
def delayed(history, delay):
    """history [N, K, A] with history[:, 0] the newest action; delay [N] in [0, K-1]. Returns history[n, delay[n]]."""
    import torch

    idx = delay.long().clamp(0, history.shape[1] - 1)
    return history[torch.arange(history.shape[0], device=history.device), idx]


def closing_speed(rel, vel_rel):
    """Radial closing speed (positive = approaching) of rel = robot - target, vel_rel = v_robot - v_target."""
    dist = rel.norm(dim=1)
    return dist, -(rel * vel_rel).sum(dim=1) / dist.clamp(min=1e-6)


def capture_mask(rel, vel_rel, detected, alive, radius, max_closing):
    """Envs that capture: alive, centre distance <= radius, closing speed <= max_closing, target detected."""
    dist, closing = closing_speed(rel, vel_rel)
    return alive & detected & (dist <= float(radius)) & (closing <= float(max_closing))


def apply_config(task_config, s):
    """Set the task configuration the switched-on rules need, before the task is built. Returns before / after."""
    def snap():
        return {"success_radius": float(task_config.success_radius),
                "flight_altitude": float(task_config.flight_altitude),
                "camera_translation": [float(x) for x in task_config.vision.camera_translation]}

    before = snap()
    if s.get("capture") is not None:
        task_config.success_radius = float(s["capture"]["NAVRL_CAMREF_CONTACT_RADIUS_M"])
    if s.get("geometry") is not None:
        task_config.flight_altitude = float(s["geometry"]["NAVRL_CAMREF_FLIGHT_ALTITUDE_M"])
        cam = list(task_config.vision.camera_translation)
        cam[0] = float(s["geometry"]["NAVRL_CAMREF_CAMERA_X_M"])
        task_config.vision.camera_translation = cam
    return {"config_before": before, "config_after": snap()}


def classify(crashed, captured, cause_code, contact_mask):
    """Per env: 'capture', 'crash:<class>' or '' (not finished here)."""
    out = []
    for cr, ca, code, tc in zip(crashed.tolist(), captured.tolist(), cause_code.tolist(), contact_mask.tolist()):
        if ca:
            out.append("capture")
        elif cr and tc:
            out.append("crash:target_contact_swept")
        elif cr:
            out.append("crash:" + CAUSES.get(int(code), "unknown"))
        else:
            out.append("")
    return out


# ---------------------------------------------------------------- rules
def install_rules_task(task_cls, s, task_config):
    """Wrap the methods the switched-on rules need and apply their configuration. Refuses a second install, an install
    on a class that already carries navrl_cams2 or its contact code (two rule sets), and settings with no rule on."""
    import torch

    if not rules_on(s):
        raise RuntimeError("no CAM-REF rule is switched on")
    if getattr(task_cls, "_camref_settings", None) is not None:
        raise RuntimeError("CAM-REF rules already installed on %r" % task_cls)
    for attr in ("_cams2_settings", "_cams2_contactcode_settings"):
        if getattr(task_cls, attr, None) is not None:
            raise RuntimeError("CAM-REF rules refuse a class that carries %s (navrl_cams2's rules)" % attr)
    cap, dly = s.get("capture"), s.get("delay")
    stats = {"steps": 0, "contacts": 0, "captures": 0, "delay_draws": {}}
    wrapped = []
    saved = {}

    if dly is not None:
        dmin, dmax = int(dly["NAVRL_CAMREF_DELAY_MIN"]), int(dly["NAVRL_CAMREF_DELAY_MAX"])
        seed = int(dly["NAVRL_CAMREF_DELAY_SEED"])
        orig_transform, orig_reset = task_cls.transform_action_to_command, task_cls.reset_idx
        saved.update(transform_action_to_command=orig_transform, reset_idx=orig_reset)

        def _count(draw):
            for k in draw.tolist():
                stats["delay_draws"][str(k)] = stats["delay_draws"].get(str(k), 0) + 1

        def _ensure(self, actions):
            hist = getattr(self, "_camref_hist", None)
            if hist is None or hist.shape[0] != actions.shape[0] or hist.shape[2] != actions.shape[1]:
                self._camref_hist = torch.zeros((actions.shape[0], dmax + 1, actions.shape[1]),
                                                dtype=actions.dtype, device=actions.device)
                self._camref_gen = torch.Generator(device=actions.device)
                self._camref_gen.manual_seed(seed)
                self._camref_delay = torch.randint(dmin, dmax + 1, (actions.shape[0],), generator=self._camref_gen,
                                                   device=actions.device)
                _count(self._camref_delay)

        def transform_action_to_command(self, actions):
            _ensure(self, actions)
            self._camref_hist = torch.roll(self._camref_hist, 1, dims=1)
            self._camref_hist[:, 0] = actions
            return orig_transform(self, delayed(self._camref_hist, self._camref_delay))

        def reset_idx(self, env_ids):
            out = orig_reset(self, env_ids)
            hist = getattr(self, "_camref_hist", None)
            if hist is not None and env_ids is not None and len(env_ids) > 0:
                ids = torch.as_tensor(env_ids, device=hist.device).long()
                hist[ids] = 0.0
                draw = torch.randint(dmin, dmax + 1, (ids.numel(),), generator=self._camref_gen, device=hist.device)
                self._camref_delay[ids] = draw
                _count(draw)
            return out

        task_cls.transform_action_to_command = transform_action_to_command
        task_cls.reset_idx = reset_idx
        wrapped += ["transform_action_to_command", "reset_idx"]

    if cap is not None:
        radius = float(cap["NAVRL_CAMREF_CAPTURE_RADIUS_M"])
        max_closing = float(cap["NAVRL_CAMREF_CAPTURE_MAX_CLOSING_MPS"])
        source = DETECT_SOURCES[cap[DETECT_ENV]]
        orig_reward = task_cls.compute_state_reward_and_terminations
        saved.update(compute_state_reward_and_terminations=orig_reward)

        def compute_state_reward_and_terminations(self, *args, **kwargs):
            out = orig_reward(self, *args, **kwargs)
            # navrl_cams2_contactcode: the task's swept test at the contact radius gets cause code 0 and the tallies
            contact = self.captured_now.bool().clone()
            self._camref_contact_now = contact
            if bool(contact.any()):
                n = int(contact.sum().item())
                self._crash_cause_code[contact] = CONTACT_CODE
                if getattr(self, "_crash_diag", False):
                    self._diag["contact"] += n
                    steps = self.sim_env.sim_steps.to(torch.float32)
                    self._diag_steps["contact"] += float(steps[contact].sum().item())
                    self._diag_x_sum += float(self.obs_dict["robot_position"][contact, 0].sum().item())
                # navrl_cams2: the contact becomes a collision with the target
                self.rewards[contact] = float(self.rw["collision_penalty"])
                self.captured_now = self.captured_now & ~contact
                self.crashed_now = self.crashed_now | contact
            alive = self.terminations <= 0
            rel = self.obs_dict["robot_position"] - self.target_position
            vel_rel = self.obs_dict["robot_linvel"] - self.target_vel_w
            got = capture_mask(rel, vel_rel, getattr(self, source).bool(), alive, radius, max_closing)
            if bool(got.any()):
                self.rewards[got] = self.rewards[got] + float(self.rw["capture_bonus"])
                self.terminations[got] = 1
                self.captured_now = self.captured_now | got
            stats["steps"] += 1
            stats["contacts"] += int(contact.sum().item())
            stats["captures"] += int(got.sum().item())
            return out

        task_cls.compute_state_reward_and_terminations = compute_state_reward_and_terminations
        wrapped.append("compute_state_reward_and_terminations")

    task_cls._camref_settings = {k: (dict(v) if v is not None else None) for k, v in s.items() if k != "record"}
    task_cls._camref_stats = stats
    _INSTALLED["rules"] = (task_cls, saved)
    info = {"wrapped": wrapped, "contact_code": CONTACT_CODE if cap is not None else None,
            "detect_attribute": DETECT_SOURCES[cap[DETECT_ENV]] if cap is not None else None,
            "delay_steps": [int(dly["NAVRL_CAMREF_DELAY_MIN"]), int(dly["NAVRL_CAMREF_DELAY_MAX"])] if dly else None}
    info.update(apply_config(task_config, s))
    return info


# ---------------------------------------------------------------- recorder
def install_record_task(task_cls, s):
    """Wrap transform_action_to_command and compute_state_reward_and_terminations (outermost at install time). Refuses a
    second install and an install inside navrl_cams2 (the recorder must sit outside every rule that changes the flags)."""
    import torch

    rec = (s or {}).get("record")
    if rec is None:
        raise RuntimeError("the CAM-REF recorder is not switched on")
    if getattr(task_cls, "_camref_record_settings", None) is not None:
        raise RuntimeError("CAM-REF recorder already installed on %r" % task_cls)
    if getattr(task_cls, "_cams2_settings", None) is not None:
        raise RuntimeError("the CAM-REF recorder would sit inside navrl_cams2's wrapper; use navrl_cams2diag there")
    path = rec[RECORD_PATH_ENV]
    cap = s.get("capture")
    source = DETECT_SOURCES[cap[DETECT_ENV]] if cap is not None else "_visible_now"
    orig_transform = task_cls.transform_action_to_command
    orig_reward = task_cls.compute_state_reward_and_terminations
    stream = sys.stderr
    st = {"steps": 0, "rows": 0, "handle": None, "ep_index": None, "ep_steps": None, "det_steps": None,
          "totals": {"capture": 0, "crash": 0, "timeout": 0}, "classes": {}}

    def _ensure(self, n, device):
        if st["ep_steps"] is None or st["ep_steps"].shape[0] != n:
            st["ep_steps"] = torch.zeros(n, dtype=torch.long, device=device)
            st["det_steps"] = torch.zeros(n, dtype=torch.long, device=device)
            st["ep_index"] = [0] * n
        if st["handle"] is None:
            st["handle"] = open(path, "x", encoding="utf-8")

    def transform_action_to_command(self, actions):
        command = orig_transform(self, actions)
        _ensure(self, actions.shape[0], actions.device)
        st["ep_steps"] += 1
        st["det_steps"] += getattr(self, source).bool().to(torch.long)
        return command

    def _report(cls):
        payload = {"event": "summary", "snapshot": "running totals through this step count, not a final line",
                   "steps": st["steps"], "rows": st["rows"], "totals": dict(st["totals"]),
                   "crash_classes": dict(st["classes"]), "rules": dict(getattr(cls, "_camref_stats", None) or {})}
        stream.write("%s %s\n" % (LOG_PREFIX, json.dumps(payload, sort_keys=True)))
        stream.flush()

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
        if bool(finished.any()):
            contact = getattr(self, "_camref_contact_now", None)
            if contact is None or contact.shape[0] != n:
                contact = torch.zeros(n, dtype=torch.bool, device=captured.device)
            classes = classify(crashed, captured, self._crash_cause_code, contact.bool())
            rel = (self.obs_dict["robot_position"] - self.target_position).to(torch.float64)
            vel_rel = (self.obs_dict["robot_linvel"] - self.target_vel_w).to(torch.float64)
            dist, closing = closing_speed(rel, vel_rel)
            delay = getattr(self, "_camref_delay", None)
            if delay is None:
                delay = getattr(self, "_cams2_delay", None)
            codes = self._crash_cause_code.tolist()
            ep_steps, det_steps = st["ep_steps"].tolist(), st["det_steps"].tolist()
            for env in torch.nonzero(finished, as_tuple=False).flatten().tolist():
                cls = classes[env]
                outcome = "capture" if cls == "capture" else ("crash" if cls.startswith("crash:") else "timeout")
                row = {"env_index": env, "env_episode_index": st["ep_index"][env], "completion_step": st["steps"],
                       "outcome": outcome, "crash_class": cls[6:] if cls.startswith("crash:") else "none",
                       "cause_code": int(codes[env]) if outcome == "crash" else None,
                       "delay_k": int(delay[env].item()) if delay is not None else None,
                       "episode_steps": int(ep_steps[env]), "detected_steps": int(det_steps[env]),
                       "detect_attribute": source,
                       "capture_dist_m": float(dist[env]) if outcome == "capture" else None,
                       "capture_closing_mps": float(closing[env]) if outcome == "capture" else None}
                st["handle"].write(json.dumps(row, sort_keys=True) + "\n")
                st["rows"] += 1
                st["ep_index"][env] += 1
                st["totals"][outcome] += 1
                st["classes"][row["crash_class"]] = st["classes"].get(row["crash_class"], 0) + 1
            st["handle"].flush()
            st["ep_steps"][finished] = 0
            st["det_steps"][finished] = 0
        if st["steps"] in REPORT_AT_CALLS or st["steps"] % REPORT_EVERY_CALLS == 0:
            _report(type(self))
        return out

    task_cls.transform_action_to_command = transform_action_to_command
    task_cls.compute_state_reward_and_terminations = compute_state_reward_and_terminations
    task_cls._camref_record_settings = dict(rec)
    task_cls._camref_record_stats = st
    _INSTALLED["record"] = (task_cls, orig_transform, orig_reward)
    return {"jsonl": path, "detect_attribute": source,
            "wrapped": ["transform_action_to_command", "compute_state_reward_and_terminations"]}


def telemetry_lines(texts, prefix=LOG_PREFIX):
    """The JSON payloads of the "[camref] {...}" lines in `texts`."""
    out = []
    for text in texts:
        for line in text.splitlines():
            if line.startswith(prefix + " {"):
                try:
                    out.append(json.loads(line[len(prefix) + 1:]))
                except ValueError:
                    pass
    return out


def install_rules(s, task_config):
    """Production install of the rules (tools/camref_entry.py)."""
    from aerial_gym.task.navrl_task import navrl_task

    info = install_rules_task(navrl_task.NavRLTask, s, task_config)
    info["settings"] = {k: v for k, v in s.items() if k != "record"}
    return info


def install_record(s):
    """Production install of the recorder (tools/camref_entry.py), after the rules."""
    from aerial_gym.task.navrl_task import navrl_task

    info = install_record_task(navrl_task.NavRLTask, s)
    info["settings"] = dict(s["record"])
    return info


def uninstall():
    """Tests only: restores the wrapped methods (recorder first, then the rules) and removes the class attributes."""
    rec = _INSTALLED.pop("record", None)
    if rec is not None:
        cls, tr, rw = rec
        cls.transform_action_to_command, cls.compute_state_reward_and_terminations = tr, rw
        st = getattr(cls, "_camref_record_stats", None)
        if st and st.get("handle"):
            st["handle"].close()
        for name in ("_camref_record_settings", "_camref_record_stats"):
            if name in cls.__dict__:
                delattr(cls, name)
    rules = _INSTALLED.pop("rules", None)
    if rules is not None:
        cls, saved = rules
        for name, fn in saved.items():
            setattr(cls, name, fn)
        for name in ("_camref_settings", "_camref_stats"):
            if name in cls.__dict__:
                delattr(cls, name)
