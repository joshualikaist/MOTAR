"""Opt-in, evaluation-only crash-cause code for the CAM-S2a target contacts (amendment 1 of
docs/plans/prereg_camonly_s2a_eval_2026-11-24.md). Installed by tools/realveh_entry.py only when NAVRL_CAMS2_CONTACTCODE=1,
after navrl_camsafe and BEFORE navrl_cams2, so that its wrapper is the INNER one: it runs on the task's own result, where
`captured_now` is the swept test at success_radius = NAVRL_CAMS2_CONTACT_RADIUS_M, which is exactly the set navrl_cams2
turns into a collision one layer further out (navrl_cams2.py:147-151).

Why: the task fills `_crash_cause_code` with -1 and then 0..3 for its own crashes (navrl_task.py:6137-6141). A cams2
converted contact is a crash with code -1. The bulk-eval strata tallies count only codes 0..3 (navrl_task.py:8514-8520,
8539-8545) and `_validate_eval_outcome_strata` requires the per-stratum cause sum to equal the crash count
(navrl_task.py:9045-9050), so every CAM-S2a-E identity cell of 2026-11-24 raised "crash-cause mismatch: 286 != 372" at the
export (:9127) and wrote no result.json (raw/identity_01, IDENTITY_FAIL).

What it does, per step, after the task's reward/termination method and before cams2's wrapper:
  1. contact = captured_now (the swept test at the contact radius; cams2 converts exactly this set);
  2. _crash_cause_code[contact] = 0 ("contact": the same bucket as a bar contact, as the strata have no other code);
  3. when the task's crash diagnostics are on (always in bulk eval, navrl_task.py:1900-1902) the descriptive tallies that
     feed result.json's top-level crash_causes get the same envs: _diag["contact"] (+count), _diag_steps["contact"]
     (+sim_steps) and _diag_x_sum (+death x), so that "bar_contact" there reads "contact, target contacts included";
  4. exposes the mask as self._cams2_contact_now for navrl_cams2diag, which classifies target_contact_swept from it.
Passivity: every reader of _crash_cause_code and of the _diag tallies is a diagnostic or an export (the strata tallies
and their validator :8514/:8539/:9048; the joint speed telemetry :5110; the episode forensics :5135; result.json
:9132-9134, :9685-9694, :9741; the epoch dashboard :10012-10021). None of them writes rewards, terminations,
observations or commands. navrl_task.py and navrl_cams2.py are byte-pinned, so the method is wrapped at run time.
A [cams2contact] JSON line on stderr every REPORT_EVERY_CALLS steps carries the running counts.
"""
import json
import os
import sys

ENV_SWITCH = "NAVRL_CAMS2_CONTACTCODE"
CONTACT_CODE = 0
REPORT_AT_CALLS = (32, 320)
REPORT_EVERY_CALLS = 320
LOG_PREFIX = "[cams2contact]"
_INSTALLED = {}


def settings_from_env(environ=None):
    """{} when on, None when off; refuses a switch other than unset / 0 / 1."""
    env = os.environ if environ is None else environ
    switch = env.get(ENV_SWITCH, "").strip()
    if switch not in ("", "0", "1"):
        raise ValueError("%s must be unset, 0 or 1, got %r" % (ENV_SWITCH, switch))
    return {} if switch == "1" else None


def install_task(task_cls):
    """Wrap compute_state_reward_and_terminations. Refuses a second install, an install before navrl_camsafe (the
    production order) and an install after navrl_cams2 (the wrapper must sit inside cams2's)."""
    import torch

    if getattr(task_cls, "_cams2_contactcode_settings", None) is not None:
        raise RuntimeError("cams2 contact code already installed on %r" % task_cls)
    if getattr(task_cls, "_camsafe_settings", None) is None:
        raise RuntimeError("cams2 contact code must be installed after navrl_camsafe")
    if getattr(task_cls, "_cams2_settings", None) is not None:
        raise RuntimeError("cams2 contact code must be installed BEFORE navrl_cams2 (it reads the task's swept test)")
    orig_reward = task_cls.compute_state_reward_and_terminations
    stats = {"steps": 0, "contacts": 0, "contact_steps": 0}
    stream = sys.stderr

    def _report():
        stream.write("%s %s\n" % (LOG_PREFIX, json.dumps({"event": "summary", "snapshot": "running totals", **stats},
                                                         sort_keys=True)))
        stream.flush()

    def compute_state_reward_and_terminations(self, *args, **kwargs):
        out = orig_reward(self, *args, **kwargs)
        contact = self.captured_now.bool().clone()          # the task's swept test at the contact radius (cams2.py:147)
        self._cams2_contact_now = contact
        stats["steps"] += 1
        if bool(contact.any()):
            n = int(contact.sum().item())
            self._crash_cause_code[contact] = CONTACT_CODE
            if getattr(self, "_crash_diag", False):
                self._diag["contact"] += n
                steps = self.sim_env.sim_steps.to(torch.float32)
                self._diag_steps["contact"] += float(steps[contact].sum().item())
                self._diag_x_sum += float(self.obs_dict["robot_position"][contact, 0].sum().item())
            stats["contacts"] += n
            stats["contact_steps"] += 1
        if stats["steps"] in REPORT_AT_CALLS or stats["steps"] % REPORT_EVERY_CALLS == 0:
            _report()
        return out

    task_cls.compute_state_reward_and_terminations = compute_state_reward_and_terminations
    task_cls._cams2_contactcode_settings = {}
    task_cls._cams2_contactcode_stats = stats
    _INSTALLED["task"] = (task_cls, orig_reward)
    return {"contact_code": CONTACT_CODE, "wrapped": ["compute_state_reward_and_terminations"],
            "exposes": "_cams2_contact_now", "updates_diag": ["contact", "_diag_steps.contact", "_diag_x_sum"]}


def telemetry_lines(texts, prefix=LOG_PREFIX):
    out = []
    for text in texts:
        for line in text.splitlines():
            if line.startswith(prefix + " {"):
                try:
                    out.append(json.loads(line[len(prefix) + 1:]))
                except ValueError:
                    pass
    return out


def install(s):
    """Production install (tools/realveh_entry.py), after navrl_camsafe and before navrl_cams2."""
    from aerial_gym.task.navrl_task import navrl_task

    info = install_task(navrl_task.NavRLTask)
    info["settings"] = dict(s)
    return info


def uninstall():
    """Tests only."""
    task = _INSTALLED.pop("task", None)
    if task is not None:
        cls, rw = task
        cls.compute_state_reward_and_terminations = rw
        for name in ("_cams2_contactcode_settings", "_cams2_contactcode_stats"):
            if name in cls.__dict__:
                delattr(cls, name)
