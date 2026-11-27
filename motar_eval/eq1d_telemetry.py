"""EQ-1D diagnostic telemetry (evaluation only; no simulator import).

* ``d1_command`` turns a preregistered D1 fixture into the held vehicle-frame command of an action.
* ``SubstepRecorder`` is an ``env_manager.set_physics_step_callback`` object: ``__call__`` runs after
  the controller and motor model have produced this substep's forces, ``post_physics_step`` after
  PhysX has advanced and the tensors were refreshed. It copies tensors into preallocated device
  buffers and never writes back.
* ``MotorTap`` wraps a motor model's ``update_motor_thrusts`` so the reference thrust it receives is
  kept; the call goes through unchanged.
* ``snapshot`` / ``restore`` save and put back every tensor and scalar reachable from an object
  (the perception module) so one observation can be recomputed from the same pre-step state.
"""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path

import numpy as np
import torch

PLAN = Path(__file__).resolve().parents[1] / "docs/plans/eq1d_qualification_v1.json"


def load_plan(path=PLAN):
    return json.loads(Path(path).read_text())


# ------------------------------------------------------------------ D1 commands
def d1_command(fixture, t):
    """Vehicle-frame [vx, vy, vz, yaw_rate] held during the action that starts at time ``t``."""
    if "sinusoid" in fixture:
        s = fixture["sinusoid"]
        cmd = [0.0, 0.0, 0.0, 0.0]
        if t >= s["start_s"] - 1e-9:
            cmd[s["axis"]] = s["amplitude"] * math.sin(2.0 * math.pi * s["hz"] * (t - s["start_s"]))
        return cmd
    if "contact" in fixture:
        c = fixture["contact"]
        if t < 0.5 - 1e-9:
            return [0.0, 0.0, 0.0, 0.0]
        h = math.radians(c["heading_deg"])
        return [c["speed_mps"] * math.cos(h), c["speed_mps"] * math.sin(h), 0.0, 0.0]
    cmd = [0.0, 0.0, 0.0, 0.0]
    for start, value in fixture["segments"]:
        if t >= start - 1e-9:
            cmd = [float(v) for v in value]
    return cmd


def d1_commands(fixtures, t, device="cpu"):
    return torch.tensor([d1_command(f, t) for f in fixtures], dtype=torch.float32, device=device)


# ------------------------------------------------------------------ substep recording
SUBSTEP_FIELDS = {
    "action_index": (), "position": (3,), "orientation_xyzw": (4,), "linvel_w": (3,), "angvel_w": (3,),
    "command": (4,), "motor_ref_thrust": (4,), "motor_thrust": (4,), "robot_force": (3,), "robot_torque": (3,),
    "contact_force": (3,),
}


class SubstepRecorder:
    """Per-substep log for ``num_envs`` envs and up to ``max_substeps`` substeps (device buffers)."""

    def __init__(self, num_envs, max_substeps, device, sources):
        """``sources``: name -> zero-argument callable returning the current tensor ([N, ...])."""
        self.num_envs, self.max_substeps, self.device = int(num_envs), int(max_substeps), device
        self.sources = sources
        self.buffers = {k: torch.zeros((self.max_substeps, self.num_envs) + shape, dtype=torch.float32, device=device)
                        for k, shape in SUBSTEP_FIELDS.items() if k != "action_index"}
        self.action_index = torch.full((self.max_substeps,), -1, dtype=torch.int64)
        self.k = 0
        self.current_action = -1
        self.enabled = True

    def __call__(self):  # pre-physics: controller output and motor update of this substep exist
        if not self.enabled:
            return
        if self.k >= self.max_substeps:
            raise RuntimeError("SubstepRecorder buffer full (%d substeps)" % self.max_substeps)
        for name in ("command", "motor_ref_thrust", "motor_thrust", "robot_force", "robot_torque"):
            self.buffers[name][self.k] = self.sources[name]()
        self.action_index[self.k] = self.current_action

    def post_physics_step(self):  # PhysX advanced; root-state tensors refreshed
        if not self.enabled:
            return
        for name in ("position", "orientation_xyzw", "linvel_w", "angvel_w", "contact_force"):
            self.buffers[name][self.k] = self.sources[name]()
        self.k += 1

    def arrays(self):
        out = {k: v[: self.k].detach().cpu().numpy() for k, v in self.buffers.items()}
        out["action_index"] = self.action_index[: self.k].numpy()
        return out


class MotorTap:
    """Call-through wrapper of ``motor_model.update_motor_thrusts`` keeping the last reference."""

    def __init__(self, motor_model):
        self.motor_model = motor_model
        self.original = motor_model.update_motor_thrusts
        self.last_ref = None
        motor_model.update_motor_thrusts = self

    def __call__(self, ref_thrust):
        self.last_ref = ref_thrust.detach().clone()
        return self.original(ref_thrust)

    def detach(self):
        self.motor_model.update_motor_thrusts = self.original


# ------------------------------------------------------------------ state snapshot
_SCALARS = (bool, int, float, str, type(None))


def _walk(obj, prefix, depth, seen, out, module_prefixes):
    if id(obj) in seen or depth < 0:
        return
    seen.add(id(obj))
    items = obj.items() if isinstance(obj, dict) else vars(obj).items() if hasattr(obj, "__dict__") else ()
    for key, value in list(items):
        path = prefix + (("[%r]" % (key,)) if isinstance(obj, dict) else "." + str(key))
        if isinstance(value, torch.nn.Module):
            continue  # weights are fixed; their buffers are not observation state
        if isinstance(value, torch.Tensor):
            out[path] = ("tensor", obj, key, value.detach().clone())
        elif isinstance(value, _SCALARS):
            out[path] = ("scalar", obj, key, value)
        elif isinstance(value, dict):
            out[path] = ("dict_keys", obj, key, list(value.keys()))
            _walk(value, path, depth - 1, seen, out, module_prefixes)
        elif isinstance(value, (list, tuple)) and all(isinstance(v, (torch.Tensor,) + _SCALARS) for v in value):
            out[path] = ("seq", obj, key, [v.detach().clone() if isinstance(v, torch.Tensor) else v for v in value])
        elif type(value).__module__.startswith(module_prefixes):
            _walk(value, path, depth - 1, seen, out, module_prefixes)


def snapshot(obj, depth=4, module_prefixes=("aerial_gym", "motar_eval")):
    """Every tensor / scalar reachable from ``obj`` (nested aerial_gym objects, dicts, lists)."""
    out = {}
    _walk(obj, "", depth, set(), out, module_prefixes)
    return out


def _set(owner, key, value):
    if isinstance(owner, dict):
        owner[key] = value
    else:
        setattr(owner, key, value)


def _get(owner, key):
    return owner[key] if isinstance(owner, dict) else getattr(owner, key)


def restore(snap):
    """Put the snapshot back. Same-shape tensors are copied in place so aliases stay aliases."""
    for path, (kind, owner, key, saved) in snap.items():
        if kind == "tensor":
            current = _get(owner, key) if (key in owner if isinstance(owner, dict) else hasattr(owner, key)) else None
            if (isinstance(current, torch.Tensor) and current.shape == saved.shape and current.dtype == saved.dtype
                    and current.device == saved.device):
                current.copy_(saved)
            else:
                _set(owner, key, saved.clone())
        elif kind == "scalar":
            _set(owner, key, saved)
        elif kind == "seq":
            cur = _get(owner, key)
            rebuilt = [s.clone() if isinstance(s, torch.Tensor) else s for s in saved]
            _set(owner, key, type(cur)(rebuilt) if isinstance(cur, (list, tuple)) else rebuilt)
        elif kind == "dict_keys":
            d = _get(owner, key)
            for extra in [k for k in d if k not in saved]:
                del d[extra]


def tensors_equal(a, b):
    return a.shape == b.shape and a.dtype == b.dtype and bool(torch.equal(a, b))


# ------------------------------------------------------------------ files
def sha256_file(path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        for block in iter(lambda: fh.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()


def write_npz(path, arrays):
    path = Path(path)
    if path.exists():
        raise FileExistsError("refusing to overwrite %s" % path)
    np.savez_compressed(path, **{k: np.asarray(v) for k, v in arrays.items()})
    return {"file": path.name, "bytes": path.stat().st_size, "sha256": sha256_file(path)}


def write_json(path, obj):
    path = Path(path)
    if path.exists():
        raise FileExistsError("refusing to overwrite %s" % path)
    path.write_text(json.dumps(obj, indent=1, sort_keys=True, default=float) + "\n")
    return {"file": path.name, "bytes": path.stat().st_size, "sha256": sha256_file(path)}


def rms_digest(module):
    """SHA-256 of running_mean, running_var, count (the EQ-1C C2 digest convention)."""
    h = hashlib.sha256()
    for name in ("running_mean", "running_var", "count"):
        h.update(getattr(module, name).detach().cpu().contiguous().numpy().tobytes())
    return h.hexdigest()


__all__ = ["PLAN", "load_plan", "d1_command", "d1_commands", "SUBSTEP_FIELDS", "SubstepRecorder", "MotorTap",
           "snapshot", "restore", "tensors_equal", "sha256_file", "write_npz", "write_json", "rms_digest"]
