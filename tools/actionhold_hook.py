"""G7 action hold / delay hook (docs/plans/prereg_actionhold_g7_v1.md). Opt-in, default OFF.

Loaded only by ``tools/actionhold_site/sitecustomize.py`` when ``MOTAR_ACTION_HOLD_K`` or ``MOTAR_ACTION_DELAY_D`` is set.
It registers a post-import patch for ``aerial_gym.rl_training.rl_games.navrl_players`` (the B1-R hook's mechanism):
the patched ``NavRLPpoPlayerContinuous.get_action`` runs the original method first (Policy F's deterministic action),
then returns a held or delayed copy. Everything downstream -- the speed governor, the controller, the altitude PI,
the recorders and the episode ledger -- is inside ``task.step`` and unchanged; the governor still runs every step.

  hold k (k >= 1):  at an env's episode start and every k-th step after it the policy action is latched; in between
                    the latched action is returned (open-loop chunks of k steps, the premise of action chunking).
  delay d (d >= 0): the action computed d steps earlier is returned; for the first d steps of an episode the zero
                    action (hover: zero velocity and yaw rate) is returned (a d x 100 ms command latency).
k = 1 and d = 0 return the policy action itself (the identity control). An episode start is an env whose
``sim_env.sim_steps`` is 0 when the action is requested (reset_idx zeroes it; EnvManager.step increments it).
A sidecar ``actionhold_hook.json`` (config, call counts, latched and delayed steps) is written next to
``NAVRL_BULK_EVAL_JSON`` when the player exits.
"""
from __future__ import annotations

import atexit
import importlib.abc
import json
import os
from pathlib import Path
import sys

TARGET_MODULE = "aerial_gym.rl_training.rl_games.navrl_players"
TARGET_CLASS = "NavRLPpoPlayerContinuous"
SIDECAR = "actionhold_hook.json"
STATE = {"controller": None, "finder": None, "written_by": None}


def read_config(environ=os.environ):
    k_raw = environ.get("MOTAR_ACTION_HOLD_K", "1").strip() or "1"
    d_raw = environ.get("MOTAR_ACTION_DELAY_D", "0").strip() or "0"
    k, d = int(k_raw), int(d_raw)
    if k < 1 or d < 0 or k > 50 or d > 50:
        raise ValueError("MOTAR_ACTION_HOLD_K must be in [1, 50] and MOTAR_ACTION_DELAY_D in [0, 50]")
    if k > 1 and d > 0:
        raise ValueError("hold and delay are separate arms; set only one")
    return {"hold_k": k, "delay_d": d}


class _PatchingLoader(importlib.abc.Loader):
    def __init__(self, inner, patch):
        self.inner, self.patch = inner, patch

    def create_module(self, spec):
        return self.inner.create_module(spec)

    def exec_module(self, module):
        self.inner.exec_module(module)
        self.patch(module)


class PostImportPatch(importlib.abc.MetaPathFinder):
    def __init__(self, name, patch):
        self.name, self.patch, self.applied = name, patch, False

    def find_spec(self, fullname, path=None, target=None):
        if fullname != self.name or self.applied:
            return None
        for finder in sys.meta_path:
            if finder is self or not hasattr(finder, "find_spec"):
                continue
            spec = finder.find_spec(fullname, path, target)
            if spec is not None and spec.loader is not None:
                spec.loader = _PatchingLoader(spec.loader, self._apply)
                return spec
        return None

    def _apply(self, module):
        self.patch(module)
        self.applied = True


class Controller:
    """Per-process state. ``step_source(player)`` returns the per-env episode step tensor (sim_steps)."""

    def __init__(self, config, step_source=None):
        self.config = config
        self.step_source = step_source or _sim_steps
        self.calls = 0
        self.latched = 0
        self.held = 0
        self.delayed_zero = 0
        self.held_action = None
        self.buffer = None          # [d, num_envs, A] ring of past policy actions, oldest first

    def act(self, player, action):
        import torch
        self.calls += 1
        k, d = self.config["hold_k"], self.config["delay_d"]
        if k == 1 and d == 0:
            return action
        steps = self.step_source(player).to(action.device)
        new = steps == 0
        if k > 1:
            if self.held_action is None:
                self.held_action = action.detach().clone()
                latch = torch.ones_like(new)
            else:
                latch = new | (torch.remainder(steps, k) == 0)
            self.held_action[latch] = action.detach()[latch]
            self.latched += int(latch.sum().item())
            self.held += int((~latch).sum().item())
            return self.held_action.clone()
        if self.buffer is None:
            self.buffer = torch.zeros((d,) + tuple(action.shape), dtype=action.dtype, device=action.device)
        self.buffer[:, new] = 0.0                       # a new episode starts with an empty (hover) pipeline
        out = self.buffer[0].clone()
        self.buffer = torch.cat([self.buffer[1:], action.detach().unsqueeze(0)], dim=0)
        # first d steps of an episode: zeros came from the reset above, count them
        self.delayed_zero += int((steps < d).sum().item())
        return out

    def summary(self):
        return {"config": dict(self.config), "calls": self.calls, "latched_env_steps": self.latched,
                "held_env_steps": self.held, "delayed_zero_env_steps": self.delayed_zero}


def _sim_steps(player):
    obj = player.env
    for _ in range(6):
        if hasattr(obj, "sim_env") and hasattr(obj.sim_env, "sim_steps"):
            return obj.sim_env.sim_steps
        obj = getattr(obj, "env")
    raise RuntimeError("actionhold: task not reachable from the player")


def patch_player_module(module, controller_factory):
    cls = getattr(module, TARGET_CLASS)
    original = cls.get_action

    def get_action(self, obs, is_deterministic=False):
        policy_action = original(self, obs, is_deterministic)
        return controller_factory().act(self, policy_action)

    get_action.__actionhold_original__ = original
    cls.get_action = get_action
    original_run = cls.run

    def run(self, *args, **kwargs):
        try:
            return original_run(self, *args, **kwargs)
        finally:
            _write_sidecar("player_run_exit")

    run.__actionhold_original__ = original_run
    cls.run = run


def _write_sidecar(trigger="atexit"):
    ctrl = STATE["controller"]
    if ctrl is None or STATE["written_by"] is not None:
        return
    STATE["written_by"] = trigger
    try:
        destination = os.environ.get("NAVRL_BULK_EVAL_JSON")
        if not destination:
            raise RuntimeError("actionhold acted without NAVRL_BULK_EVAL_JSON")
        payload = ctrl.summary()
        payload["patch_applied"] = bool(STATE["finder"] and STATE["finder"].applied)
        payload["written_by"] = trigger
        with (Path(destination).parent / SIDECAR).open("x", encoding="utf-8") as stream:
            json.dump(payload, stream, indent=2, sort_keys=True)
            stream.write("\n")
    except BaseException as exc:
        os.write(2, ("actionhold sidecar failed: %r\n" % exc).encode())
        os._exit(3)


def install(environ=os.environ):
    config = read_config(environ)

    def factory():
        if STATE["controller"] is None:
            STATE["controller"] = Controller(config)
        return STATE["controller"]

    finder = PostImportPatch(TARGET_MODULE, lambda module: patch_player_module(module, factory))
    sys.meta_path.insert(0, finder)
    STATE["finder"] = finder
    atexit.register(_write_sidecar)
    return finder
