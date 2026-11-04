"""G8 zero-action hook (docs/plans/prereg_sitl_reference_g8g9_v1_amendment2_2026-11-05.md). Opt-in, default OFF.

Loaded only by ``tools/zeroaction_site/sitecustomize.py`` when ``MOTAR_ACTION_ZERO=1``. It patches
``NavRLPpoPlayerContinuous.get_action`` after import (the B1-R hook's mechanism) to run the original method and return
zeros of the same shape: the pursuer hovers (zero velocity and yaw rate; the altitude PI holds 1.0 m), so each episode
runs to its 600-step timeout and the occlusion trace records the E2 target's full 60 s trajectory. E2 does not read
the pursuer's state (target_motion.resolve_target_behavior_contract: uses_privileged_pursuer_gt False), so the runner
checks that these trajectories equal the ones recorded with Policy F flying, over their common frames.
"""
from __future__ import annotations

import importlib.abc
import os
import sys

TARGET_MODULE = "aerial_gym.rl_training.rl_games.navrl_players"
TARGET_CLASS = "NavRLPpoPlayerContinuous"
STATE = {"finder": None, "calls": 0}


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


def patch_player_module(module):
    cls = getattr(module, TARGET_CLASS)
    original = cls.get_action

    def get_action(self, obs, is_deterministic=False):
        action = original(self, obs, is_deterministic)
        STATE["calls"] += 1
        return action * 0.0

    get_action.__zeroaction_original__ = original
    cls.get_action = get_action


def install(environ=os.environ):
    if environ.get("MOTAR_ACTION_ZERO", "").strip() != "1":
        raise ValueError("MOTAR_ACTION_ZERO must be 1 to install the zero-action hook")
    finder = PostImportPatch(TARGET_MODULE, patch_player_module)
    sys.meta_path.insert(0, finder)
    STATE["finder"] = finder
    return finder
