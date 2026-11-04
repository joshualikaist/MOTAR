"""G8 state recorder hook (docs/plans/prereg_sitl_reference_g8g9_v1_amendment1_2026-11-05.md). Opt-in, default OFF.

Loaded only by ``tools/staterec_site/sitecustomize.py`` when ``MOTAR_STATE_RECORD`` (an .npz path that must not exist)
is set. It patches ``NavRLPpoPlayerContinuous.get_action`` after import (the B1-R hook's mechanism) to read, for every
env at every action request, the task tensors the observation was built from and the policy's deterministic action,
then returns that action unchanged. It reads detached copies only; nothing is written back.

Recorded per call (arrays stacked over calls; env axis = all envs):
  sim_steps (episode step at request), robot_position, robot_linvel (world), robot_vehicle_orientation,
  robot_orientation (xyzw), robot_body_angvel, env_bounds_min / env_bounds_max (passed to perception), target_position,
  target_orientation (passed to the renderer), prev_action (the task's, as fed to perception),
  perception_step_count (scalar per call), action (the policy output returned to the player).
The npz is written when the player exits (atexit as a fallback).
"""
from __future__ import annotations

import atexit
import importlib.abc
import os
from pathlib import Path
import sys

TARGET_MODULE = "aerial_gym.rl_training.rl_games.navrl_players"
TARGET_CLASS = "NavRLPpoPlayerContinuous"
STATE = {"recorder": None, "finder": None, "written_by": None}
FIELDS = ("robot_position", "robot_linvel", "robot_vehicle_orientation", "robot_orientation", "robot_body_angvel",
          "env_bounds_min", "env_bounds_max")


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


def find_task(player):
    obj = player.env
    for _ in range(6):
        if hasattr(obj, "obs_dict") and hasattr(obj, "prev_action") and hasattr(obj, "sim_env"):
            return obj
        obj = getattr(obj, "env")
    raise RuntimeError("staterec: task not reachable from the player")


class Recorder:
    def __init__(self, path):
        self.path = Path(path)
        self.rows = {k: [] for k in FIELDS + ("sim_steps", "target_position", "target_orientation", "prev_action",
                                              "action")}
        self.step_counts = []
        self.task = None

    def record(self, player, action):
        if self.task is None:
            self.task = find_task(player)
        t = self.task
        for k in FIELDS:
            self.rows[k].append(t.obs_dict[k].detach().to("cpu").clone())
        self.rows["sim_steps"].append(t.sim_env.sim_steps.detach().to("cpu").clone())
        self.rows["target_position"].append(t.target_position.detach().to("cpu").clone())
        tq = getattr(t, "target_orientation", None)
        if tq is not None:
            self.rows["target_orientation"].append(tq.detach().to("cpu").clone())
        self.rows["prev_action"].append(t.prev_action.detach().to("cpu").clone())
        self.rows["action"].append(action.detach().to("cpu").clone())
        perception = getattr(t, "perception", None)
        self.step_counts.append(int(getattr(perception, "step_count", -1)))

    def save(self):
        import numpy as np
        import torch
        arrays = {k: torch.stack(v).numpy() for k, v in self.rows.items() if v}
        arrays["perception_step_count"] = np.array(self.step_counts, dtype=np.int64)
        with self.path.open("xb") as fh:
            np.savez_compressed(fh, **arrays)


def patch_player_module(module, recorder_factory):
    cls = getattr(module, TARGET_CLASS)
    original = cls.get_action

    def get_action(self, obs, is_deterministic=False):
        action = original(self, obs, is_deterministic)
        recorder_factory().record(self, action)
        return action

    get_action.__staterec_original__ = original
    cls.get_action = get_action
    original_run = cls.run

    def run(self, *args, **kwargs):
        try:
            return original_run(self, *args, **kwargs)
        finally:
            _save("player_run_exit")

    run.__staterec_original__ = original_run
    cls.run = run


def _save(trigger="atexit"):
    rec = STATE["recorder"]
    if rec is None or STATE["written_by"] is not None:
        return
    STATE["written_by"] = trigger
    try:
        rec.save()
    except BaseException as exc:
        os.write(2, ("staterec save failed: %r\n" % exc).encode())
        os._exit(3)


def install(environ=os.environ):
    path = environ.get("MOTAR_STATE_RECORD", "").strip()
    if not path.endswith(".npz"):
        raise ValueError("MOTAR_STATE_RECORD must name an .npz file")
    if Path(path).exists():
        raise ValueError("MOTAR_STATE_RECORD already exists: %s" % path)

    def factory():
        if STATE["recorder"] is None:
            STATE["recorder"] = Recorder(path)
        return STATE["recorder"]

    finder = PostImportPatch(TARGET_MODULE, lambda module: patch_player_module(module, factory))
    sys.meta_path.insert(0, finder)
    STATE["finder"] = finder
    atexit.register(_save)
    return finder
