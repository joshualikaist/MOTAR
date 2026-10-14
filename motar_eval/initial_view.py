"""Experiment A initial condition: fixed range, clear line of sight, pursuer yaw 0 or 180 degrees.

The treatment is the pursuer's initial heading only. Both arms draw the scene from the same sampler
with the same acceptance rule:

* the target sits at exactly ``distance_m`` from the pursuer, at the same altitude, in a uniformly
  random direction;
* it is inside the arena inset by ``wall_margin_m``;
* it passes the caller's footprint-clearance predicate (the NavRL task passes the same
  ``_spawn_footprint_clearance_accepted`` rule it uses for drone spawns, for every arm);
* the ray from the camera to the target centre is clear of every obstacle box, evaluated with the
  camera facing the target. Acceptance never depends on the arm, so for one RNG state the visible
  and hidden arms accept the same scene.

Then ``visible`` points the camera boresight at the target and ``hidden`` turns the pursuer by
180 degrees. "Hidden" therefore means outside the field of view, not occluded by a tree.

Acceptance never looks at a detector, a confidence or an outcome (KROC2027 plan v2, section C).
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Callable

import torch

from motar_eval.geometry import camera_eye, los_clear, wrap_angle, yaw_quat_xyzw

VIEWS = ("visible", "hidden")


@dataclass(frozen=True)
class InitialViewSpec:
    view: str
    distance_m: float = 10.0
    wall_margin_m: float = 1.0
    candidates_per_round: int = 64
    rounds: int = 32

    def __post_init__(self):
        if self.view not in VIEWS:
            raise ValueError("view must be one of %s, got %r" % (VIEWS, self.view))
        if not (self.distance_m > 0.0 and math.isfinite(self.distance_m)):
            raise ValueError("distance_m must be positive and finite")
        if self.candidates_per_round < 1 or self.rounds < 1:
            raise ValueError("candidates_per_round and rounds must be positive")


def view_yaw(bearing, view: str, camera_yaw_offset_rad: float = 0.0):
    """Pursuer yaw for a world bearing to the target. ``hidden`` faces directly away."""
    if view not in VIEWS:
        raise ValueError("unknown view %r" % (view,))
    yaw = bearing - camera_yaw_offset_rad
    if view == "hidden":
        yaw = yaw + math.pi
    return wrap_angle(yaw)


def sample_initial_view(
    pursuer_xyz,
    obstacle_center,
    obstacle_half,
    lo_xy,
    hi_xy,
    camera_translation,
    spec: InitialViewSpec,
    clearance_ok: Callable,
    rand: Callable,
):
    """Draw a target position for each pursuer; return the arm's yaw for the accepted ones.

    Shapes: ``pursuer_xyz`` [n, 3]; ``obstacle_center``/``obstacle_half`` [n, B, 3]; ``lo_xy``/
    ``hi_xy`` [n, 2] (the arena bounds before the wall inset). ``clearance_ok(candidates)`` maps
    [n, K, 2] candidates to an [n, K] bool mask. ``rand(shape)`` returns uniform [0, 1) samples on
    the right device; it is the only source of randomness, so the caller decides which generator
    is consumed.

    Returns a dict: ``target_xyz`` [n, 3], ``bearing`` [n], ``yaw`` [n], ``quat_xyzw`` [n, 4],
    ``accepted`` [n] bool and ``rounds_used`` [n] int. Rows that were not accepted keep NaN
    targets; the caller must respawn the pursuer and call again, or fail closed.
    """
    n = pursuer_xyz.shape[0]
    device = pursuer_xyz.device
    dtype = pursuer_xyz.dtype
    lo = lo_xy + spec.wall_margin_m
    hi = hi_xy - spec.wall_margin_m
    target_xyz = torch.full((n, 3), math.nan, dtype=dtype, device=device)
    bearing = torch.full((n,), math.nan, dtype=dtype, device=device)
    rounds_used = torch.zeros(n, dtype=torch.long, device=device)
    todo = torch.ones(n, dtype=torch.bool, device=device)
    k = spec.candidates_per_round
    for round_index in range(spec.rounds):
        if not bool(todo.any()):
            break
        ids = todo.nonzero(as_tuple=False).squeeze(-1)
        m = len(ids)
        theta = -math.pi + 2.0 * math.pi * rand((m, k))
        theta = theta.to(dtype)
        base = pursuer_xyz[ids]
        cand_xy = base[:, None, 0:2] + spec.distance_m * torch.stack(
            (torch.cos(theta), torch.sin(theta)), dim=-1
        )
        accepted = (
            (cand_xy[..., 0] >= lo[ids, 0:1])
            & (cand_xy[..., 0] <= hi[ids, 0:1])
            & (cand_xy[..., 1] >= lo[ids, 1:2])
            & (cand_xy[..., 1] <= hi[ids, 1:2])
        )
        accepted &= clearance_ok(cand_xy, ids)
        # Line of sight with the camera facing the candidate, whatever the arm.
        cand_xyz = torch.cat((cand_xy, base[:, None, 2:3].expand(m, k, 1)), dim=-1)
        facing = yaw_quat_xyzw(theta)
        eye = camera_eye(base[:, None, :].expand(m, k, 3), facing, camera_translation)
        clear = los_clear(
            eye,
            cand_xyz,
            obstacle_center[ids].unsqueeze(1).expand(m, k, -1, -1),
            obstacle_half[ids].unsqueeze(1).expand(m, k, -1, -1),
        )
        accepted &= clear
        has = accepted.any(dim=1)
        rounds_used[ids] = round_index + 1
        if bool(has.any()):
            first = accepted.to(torch.int64).argmax(dim=1)
            rows = torch.arange(m, device=device)
            resolved = ids[has]
            target_xyz[resolved] = cand_xyz[rows[has], first[has]]
            bearing[resolved] = theta[rows[has], first[has]]
            todo[resolved] = False
    yaw = view_yaw(bearing, spec.view)
    return {
        "target_xyz": target_xyz,
        "bearing": bearing,
        "yaw": yaw,
        "quat_xyzw": yaw_quat_xyzw(yaw),
        "accepted": ~todo,
        "rounds_used": rounds_used,
    }
