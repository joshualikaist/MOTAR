"""Shared helpers for the EQ-1 CPU validation package (validation only; nothing here changes source).

``production(env)`` is a context manager that yields ``load_production(name)``, which imports a
frozen production module by file path without executing the ``aerial_gym`` package ``__init__``
(which would start Isaac Gym). Parent packages are registered as empty namespace stubs, and the
Policy F LiDAR grid (72 x 4, as in every KROC/OCC receipt) plus any receipt environment are set
before import because ``navrl_perception`` reads them at module level. On exit ``sys.modules`` and
``os.environ`` are restored exactly, so the stubs and the 72-beam modules never leak into another
test in the same process; every production call therefore runs inside the context. The production
code itself is executed unmodified; the independent references in the EQ-1 tools never import it.
"""

from __future__ import annotations

import contextlib
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import types

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))  # this tree's motar_eval, never the main checkout's editable install
MAIN_CHECKOUT = Path("/home/fair/workspaces/aerial_gym_ws/src/aerial_gym_simulator")
OCC_P2 = MAIN_CHECKOUT / "results/occlusion_confirmatory_v1/OCC-P2"
OCC_P1 = MAIN_CHECKOUT / "results/occlusion_pilot_v1/OCC-P1_rerun1"
POLICY_F = ("aerial_gym/rl_training/rl_games/runs/ppo_260805_0413_navrl_v2-speedgov-ep24000-205bars-main-riskcap-s1/"
            "nn/last_gen_ppo_ep_25000_rew_39.742134.pth")
POLICY_F_SHA256 = "f702213936601860995cf61dcc570247e72543b1976e3716055cd8ec5593ad40"
MEASUREMENT_SOURCE = "5ff285ad053f181c5a1c86dddb2462bb44e6327b"

# Policy F evaluation settings as recorded in the receipts (env_snapshot_navrl).
POLICY_F_ENV = {"NAVRL_LIDAR_HBEAMS": "72", "NAVRL_LIDAR_VBEAMS": "4", "NAVRL_MAX_OBSTACLES": "8"}
CAMERA = {"width": 160, "hfov_deg": 87.0, "translation_m": (0.10, 0.0, 0.03), "target_radius_m": 0.15}
GOVERNOR = {"mode": "riskcap", "fixed_cap_mps": 2.0, "free_speed_cap_mps": 3.53553390593, "slow_distance_m": 3.0,
            "release_distance_m": 5.0, "hard_margin_m": 0.45, "brake_mps2": 2.9608856678, "reaction_s": 0.1}

_PACKAGES = ("aerial_gym", "aerial_gym.task", "aerial_gym.task.navrl_task", "aerial_gym.utils", "aerial_gym.control")
_FILES = {
    "aerial_gym.task.navrl_task.navrl_corridor": "aerial_gym/task/navrl_task/navrl_corridor.py",
    "aerial_gym.task.navrl_task.navrl_search_state": "aerial_gym/task/navrl_task/navrl_search_state.py",
    "aerial_gym.task.navrl_task.navrl_perception": "aerial_gym/task/navrl_task/navrl_perception.py",
    "aerial_gym.task.navrl_task.speed_governor": "aerial_gym/task/navrl_task/speed_governor.py",
    "aerial_gym.utils.math": "aerial_gym/utils/math.py",
    "aerial_gym.control.motor_model": "aerial_gym/control/motor_model.py",
}
_DEPENDS = {"aerial_gym.task.navrl_task.navrl_perception": ("aerial_gym.task.navrl_task.navrl_corridor",
                                                           "aerial_gym.task.navrl_task.navrl_search_state"),
            "aerial_gym.control.motor_model": ("aerial_gym.utils.math",)}


_ACTIVE = []


@contextlib.contextmanager
def production(env=None):
    """Context for executing production code; restores sys.modules and os.environ on exit."""
    saved_modules, saved_env = dict(sys.modules), dict(os.environ)
    os.environ.update(POLICY_F_ENV)
    os.environ.update({k: str(v) for k, v in (env or {}).items()})
    _ACTIVE.append(True)
    try:
        yield load_production
    finally:
        _ACTIVE.pop()
        for key in [k for k in sys.modules if k not in saved_modules]:
            del sys.modules[key]
        sys.modules.update(saved_modules)
        os.environ.clear()
        os.environ.update(saved_env)


def load_production(name):
    """Production module ``name`` (dotted) from this tree, loaded by path inside ``production()``."""
    if not _ACTIVE:
        raise RuntimeError("load_production must run inside eq1_common.production()")
    if name in sys.modules and getattr(sys.modules[name], "__eq1_loaded__", False):
        return sys.modules[name]
    for package in _PACKAGES:
        if package not in sys.modules or getattr(sys.modules[package], "__eq1_stub__", False):
            stub = types.ModuleType(package)
            stub.__path__ = [str(ROOT / package.replace(".", "/"))]
            stub.__eq1_stub__ = True
            sys.modules[package] = stub
    for dep in _DEPENDS.get(name, ()):
        load_production(dep)
    path = ROOT / _FILES[name]
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    module.__eq1_loaded__ = True
    return module


def load_config(name, relpath):
    """A config module by path under a private name (inside ``production()``; navrl_task_config needs
    the path-loaded perception module for its observation width)."""
    if name in sys.modules:
        return sys.modules[name]
    spec = importlib.util.spec_from_file_location(name, ROOT / relpath)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


RECEIPT = OCC_P2 / "static/A1/E0_static__115bars__seed4301/receipt.json"


def receipt_env():
    """Policy F's recorded NAVRL_* environment (OCC-P2 receipt); the minimal LiDAR grid if absent."""
    if RECEIPT.exists():
        return json.loads(RECEIPT.read_text())["env_snapshot_navrl"]
    return dict(POLICY_F_ENV)


def build_perception(num_envs, step_dt=0.1):
    """A fresh production NavRLPerceptionModule on CPU, configured from navrl_task_config under the
    current environment (call inside ``production(receipt_env())``)."""
    import torch

    perception = load_production("aerial_gym.task.navrl_task.navrl_perception")
    tc = load_config("eq1_navrl_task_config", "aerial_gym/config/task_config/navrl_task_config.py").task_config
    module = perception.NavRLPerceptionModule(num_envs=num_envs, device=torch.device("cpu"), cfg=tc.perception,
                                              step_dt=step_dt, camera_cfg=tc.vision)
    return module, tc, perception


def assert_in_tree(module):
    """Refuse a module imported from outside this worktree (the editable-install hazard)."""
    path = Path(module.__file__).resolve()
    if ROOT not in path.parents:
        raise RuntimeError("%s imported from %s, outside %s" % (module.__name__, path, ROOT))
    return str(path.relative_to(ROOT))


def sha256_file(path) -> str:
    path = Path(path)
    if path.is_dir():  # a directory digest: each file's relative path and SHA-256, in sorted order
        h = hashlib.sha256()
        for f in sorted(p for p in path.rglob("*") if p.is_file()):
            h.update(("%s %s\n" % (f.relative_to(path).as_posix(), sha256_file(f))).encode())
        return h.hexdigest()
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        for block in iter(lambda: fh.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()


def source_binding(paths):
    """SHA-256 of each production file exercised, plus HEAD and whether they differ from OCC's source."""
    head = subprocess.run(["git", "-C", str(ROOT), "rev-parse", "HEAD"], capture_output=True, text=True).stdout.strip()
    diff = subprocess.run(["git", "-C", str(ROOT), "diff", "--quiet", MEASUREMENT_SOURCE, "HEAD", "--", *paths]).returncode
    return {"head": head, "measurement_source": MEASUREMENT_SOURCE,
            "identical_to_measurement_source": diff == 0,
            "files_sha256": {p: sha256_file(ROOT / p) for p in paths}}


def verdict(passed, note=""):
    return {"status": "PASS" if passed else "FAIL", "note": note}


def write_record(path, record):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(record, indent=1, sort_keys=True, default=float) + "\n")
    return path
