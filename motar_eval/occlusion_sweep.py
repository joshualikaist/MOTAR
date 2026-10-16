"""Geometry-only occlusion sweep: fixed kinematics through obstacle layouts (CPU, evaluation only).

Only the layout changes between densities; the pursuer and target follow scripted kinematics, so the
result is the geometric occlusion opportunity of the layouts, not the behaviour of Policy F or of the
E2 target. A geometric fact the families make visible: with straight pursuit of a static target, a
clear sight line stays clear (every later segment lies on the earlier one), so post-acquisition
occlusion of a static target needs lateral relative motion -- a detour, a crossing pursuer or a
moving target. Obstacle collisions are ignored (paths may pass through boxes), the pursuer flies pure
pursuit at constant speed and altitude, and line of sight is omnidirectional (no camera frustum).
Label every result ``GEOMETRY_ONLY_KINEMATIC_BASELINE``.

Intended input: layouts recorded by the occlusion trace of real cells (``occlusion_layouts.npz``).
``synthetic_layout`` exists for CPU tests; it is a plain rejection sampler, not the simulator's
navrl_band placement rule, and its results must never be reported as the experiment's scenes.
"""

from __future__ import annotations

import math
from typing import Dict, List, Sequence

import numpy as np

from motar_eval import occlusion_events as ev
from motar_eval import occlusion_geometry as og

LABEL = "GEOMETRY_ONLY_KINEMATIC_BASELINE"


def synthetic_layout(count, rng, sizes_xy, bounds=((2.0, 38.0), (2.0, 38.0)), height=3.0, min_gap=1.6,
                     tries=200):
    """``count`` upright boxes with footprints from ``sizes_xy``; centre spacing >= ``min_gap``."""
    centers, halves = [], []
    for _ in range(count):
        for _ in range(tries):
            x, y = rng.uniform(*bounds[0]), rng.uniform(*bounds[1])
            if all((x - c[0]) ** 2 + (y - c[1]) ** 2 >= min_gap ** 2 for c in centers):
                sx, sy = sizes_xy[rng.integers(0, len(sizes_xy))]
                centers.append((x, y, height / 2))
                halves.append((sx / 2, sy / 2, height / 2))
                break
    return np.array(centers, dtype=np.float64).reshape(-1, 3), np.array(halves, dtype=np.float64).reshape(-1, 3)


def _inside_any(point, center, half, margin=0.0):
    if len(center) == 0:
        return False
    return bool(np.any(np.all(np.abs(point[None, :2] - center[:, :2]) <= half[:, :2] + margin, axis=1)))


FAMILIES = (
    {"name": "static_pursuit", "target_speed": 0.0, "pursuer_mode": "pursuit"},
    {"name": "static_crossing", "target_speed": 0.0, "pursuer_mode": "crossing", "horizon_s": 10.0},
    {"name": "moving_pursuit", "target_speed": 1.5, "pursuer_mode": "pursuit"},
)


def simulate_path(center, half, rng, *, pursuer_speed=2.7, target_speed=0.0, start_distance=10.0, dt=0.1,
                  horizon_s=60.0, capture_radius=0.5, altitude=1.0, arena=(0.0, 40.0), eye_forward=0.10,
                  eye_up=0.03, target_radius=0.15, rim_count=0, tries=200, pursuer_mode="pursuit"):
    """One scripted episode; returns per-frame centre blockage and the terminal frame and outcome.

    ``pursuit`` flies at the target; ``crossing`` keeps a constant heading perpendicular to the initial
    sight line (random side) and ends at the horizon, so the sight line sweeps across the layout.
    """
    if pursuer_mode not in ("pursuit", "crossing"):
        raise ValueError("pursuer_mode must be pursuit or crossing")
    lo, hi = arena[0] + 1.0, arena[1] - 1.0
    for _ in range(tries):
        p = np.array([rng.uniform(lo, hi), rng.uniform(lo, hi), altitude])
        bearing = rng.uniform(-math.pi, math.pi)
        t = p + start_distance * np.array([math.cos(bearing), math.sin(bearing), 0.0])
        if lo <= t[0] <= hi and lo <= t[1] <= hi and not _inside_any(p, center, half, 0.3) \
                and not _inside_any(t, center, half, 0.3):
            break
    else:
        raise RuntimeError("no admissible start in this layout")
    heading = rng.uniform(-math.pi, math.pi)
    tv = target_speed * np.array([math.cos(heading), math.sin(heading), 0.0])
    side = 1.0 if rng.uniform() < 0.5 else -1.0
    cross = side * np.array([-math.sin(bearing), math.cos(bearing), 0.0])
    eyes, targets = [], []
    steps = int(round(horizon_s / dt))
    outcome, terminal = "timeout", steps
    for k in range(steps):
        rel = t - p
        dist = float(np.linalg.norm(rel[:2]))
        direction = rel / max(np.linalg.norm(rel), 1e-9)
        eyes.append(p + eye_forward * np.array([direction[0], direction[1], 0.0]) + np.array([0.0, 0.0, eye_up]))
        targets.append(t.copy())
        if dist <= capture_radius:
            outcome, terminal = "capture", k + 1
            break
        step = cross if pursuer_mode == "crossing" else np.array([direction[0], direction[1], 0.0])
        p = p + pursuer_speed * dt * step
        p[:2] = np.clip(p[:2], lo, hi)
        t = t + tv * dt
        for axis in (0, 1):  # reflect the scripted target at the arena margin
            if t[axis] < lo or t[axis] > hi:
                tv[axis] = -tv[axis]
                t[axis] = min(max(t[axis], lo), hi)
    los = og.line_of_sight(np.array(eyes), np.array(targets), center, half, radius=target_radius,
                           rim_count=rim_count)
    return {"center_blocked": los["center_blocked"], "terminal_frame": terminal, "outcome": outcome}


def sweep_families(layouts_by_density, paths_per_layout: int, seed: int, families=FAMILIES) -> Dict:
    """Every kinematic family over the same layouts, each with its own seeded stream."""
    out = {"label": LABEL, "families": {}}
    for index, family in enumerate(families):
        kinematics = {k: v for k, v in family.items() if k != "name"}
        out["families"][family["name"]] = sweep(layouts_by_density, paths_per_layout, seed + index, **kinematics)
    return out


def sweep(layouts_by_density: Dict[int, Sequence], paths_per_layout: int, seed: int, dt: float = 0.1,
          at: Sequence[float] = ev.THRESHOLDS_S, **kinematics) -> Dict:
    """Per density: occlusion intervals after the first clear line of sight, summarised."""
    rng = np.random.default_rng(seed)
    out = {"label": LABEL, "seed": seed, "paths_per_layout": paths_per_layout, "dt_s": dt,
           "kinematics": {k: v for k, v in kinematics.items()}, "densities": {}}
    for density in sorted(layouts_by_density):
        intervals: List[Dict] = []
        exposures: List[float] = []
        episode = 0
        for center, half in layouts_by_density[density]:
            for _ in range(paths_per_layout):
                path = simulate_path(center, half, rng, dt=dt, **kinematics)
                blocked = path["center_blocked"]
                frames = np.arange(len(blocked))
                first_clear = ev.first_true(~blocked)
                rows = ev.episode_intervals(blocked, frames, first_clear, path["terminal_frame"], dt,
                                            path["outcome"])
                for row in rows:
                    row["episode"] = episode
                intervals += rows
                exposures.append(ev.exposure_s(first_clear, path["terminal_frame"], dt))
                episode += 1
        out["densities"][str(density)] = ev.summarise(intervals, exposures, at)
    return out


__all__ = ["LABEL", "FAMILIES", "synthetic_layout", "simulate_path", "sweep", "sweep_families"]
