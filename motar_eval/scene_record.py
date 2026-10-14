"""Episode keys and content hashes of the scene and initial state (recording only).

KROC2027 P1a does not replay scenes: the arms are independent samples from one sampler, and every
exogenous stream is labelled ``UNPAIRED_STREAM`` (plan v2, section C). This module only makes each
episode identifiable and its starting scene auditable after the fact:

* ``episode_key`` names an episode by ``(seed, density, env_id, episode_index)``;
* ``scene_hash`` hashes the active obstacle boxes, quantised to ``resolution_m``;
* ``initial_state_record`` stores the starting geometry and the treatment, plus a hash of it.

Quantisation makes the hashes independent of float32 noise below the resolution, so two runs that
built the same scene produce the same hash.
"""

from __future__ import annotations

import hashlib
import json
import math
from typing import Dict, Iterable, Sequence

SCENE_CONTRACT = "KROC2027_SCENE_RECORD_V1"
STREAM_LABEL = "UNPAIRED_STREAM"
DEFAULT_RESOLUTION_M = 1e-3


def episode_key(seed: int, density: int, env_id: int, episode_index: int) -> str:
    for name, value in (("seed", seed), ("density", density), ("env_id", env_id),
                        ("episode_index", episode_index)):
        if int(value) != value or int(value) < 0:
            raise ValueError("%s must be a non-negative integer, got %r" % (name, value))
    return "s%d/b%d/e%03d/k%03d" % (int(seed), int(density), int(env_id), int(episode_index))


def _quantise(values: Iterable[float], resolution: float):
    out = []
    for value in values:
        value = float(value)
        if not math.isfinite(value):
            raise ValueError("cannot hash a non-finite value")
        out.append(int(round(value / resolution)))
    return out


def _digest(payload) -> str:
    text = json.dumps(payload, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(text.encode("utf-8")).hexdigest()


def scene_hash(
    obstacle_center: Sequence[Sequence[float]],
    obstacle_half: Sequence[Sequence[float]],
    resolution_m: float = DEFAULT_RESOLUTION_M,
) -> str:
    """Order-independent hash of the active obstacle boxes."""
    if len(obstacle_center) != len(obstacle_half):
        raise ValueError("obstacle_center and obstacle_half must have the same length")
    boxes = sorted(
        tuple(_quantise(c, resolution_m) + _quantise(h, resolution_m))
        for c, h in zip(obstacle_center, obstacle_half)
    )
    return _digest({"contract": SCENE_CONTRACT, "resolution_m": resolution_m, "boxes": boxes})


def initial_state_record(
    key: str,
    scene: str,
    pursuer_xyz: Sequence[float],
    pursuer_yaw: float,
    target_xyz: Sequence[float],
    target_velocity_xy: Sequence[float],
    target_nominal_speed: float,
    treatment: Dict,
    resolution_m: float = DEFAULT_RESOLUTION_M,
) -> Dict:
    """Starting geometry and treatment of one episode, with a content hash."""
    dx = float(target_xyz[0]) - float(pursuer_xyz[0])
    dy = float(target_xyz[1]) - float(pursuer_xyz[1])
    record = {
        "episode_key": key,
        "scene_hash": scene,
        "pursuer_xyz": [float(v) for v in pursuer_xyz],
        "pursuer_yaw_rad": float(pursuer_yaw),
        "target_xyz": [float(v) for v in target_xyz],
        "target_velocity_xy": [float(v) for v in target_velocity_xy],
        "target_nominal_speed_mps": float(target_nominal_speed),
        "initial_range_xy_m": math.hypot(dx, dy),
        "initial_world_bearing_rad": math.atan2(dy, dx),
        "treatment": dict(treatment),
        "stream": STREAM_LABEL,
    }
    hashed = {
        "key": key,
        "scene": scene,
        "pursuer": _quantise(record["pursuer_xyz"], resolution_m),
        "yaw_mrad": int(round(record["pursuer_yaw_rad"] * 1000.0)),
        "target": _quantise(record["target_xyz"], resolution_m),
        "target_velocity": _quantise(record["target_velocity_xy"], resolution_m),
        "nominal_speed": _quantise([record["target_nominal_speed_mps"]], resolution_m),
        "treatment": record["treatment"],
    }
    record["initial_state_hash"] = _digest(hashed)
    return record
