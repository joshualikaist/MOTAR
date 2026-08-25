"""Deterministic, simulator-independent global routes for the physical NavRL target.

The planner is intentionally outside the policy.  It gives the *target actor* a collision-safe
velocity reference; it never exposes ground-truth geometry to the pursuer actor or critic.

Planning is performed only at reset, goal replacement, or explicit invalidation.  Selected
environment tensors are copied to CPU once per planning batch, while route following uses a
padded GPU waypoint cache and remains vectorised at every ordinary RL step.
"""

from __future__ import annotations

from dataclasses import dataclass
import heapq
import math
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
import torch


TARGET_ROUTE_MODE_OFF = "off"
TARGET_ROUTE_MODE_GLOBAL_ASTAR = "global_astar_v1"
TARGET_ROUTE_MODES = (TARGET_ROUTE_MODE_OFF, TARGET_ROUTE_MODE_GLOBAL_ASTAR)
TARGET_ROUTE_MODEL = "physx_ref5in_6dof_global_astar_aabb_v1"

_NEIGHBORS = (
    (-1, 0, 1.0),
    (0, -1, 1.0),
    (0, 1, 1.0),
    (1, 0, 1.0),
    (-1, -1, math.sqrt(2.0)),
    (-1, 1, math.sqrt(2.0)),
    (1, -1, math.sqrt(2.0)),
    (1, 1, math.sqrt(2.0)),
)


@dataclass(frozen=True)
class RoutePlannerConfig:
    resolution_m: float = 0.25
    tracking_margin_m: float = 0.45
    boundary_margin_m: float = 1.25
    max_expansions: int = 50000
    max_waypoints: int = 128
    replan_cooldown_steps: int = 10
    goal_tolerance_m: float = 0.05

    def validate(self) -> None:
        if not math.isfinite(self.resolution_m) or self.resolution_m <= 0.0:
            raise ValueError("route resolution must be finite and positive")
        if not math.isfinite(self.tracking_margin_m) or self.tracking_margin_m < 0.0:
            raise ValueError("route tracking margin must be finite and non-negative")
        if not math.isfinite(self.boundary_margin_m) or self.boundary_margin_m < 0.0:
            raise ValueError("route boundary margin must be finite and non-negative")
        if self.max_expansions <= 0 or self.max_waypoints < 2:
            raise ValueError("route max_expansions and max_waypoints must be positive")
        if self.replan_cooldown_steps < 1:
            raise ValueError("route replan cooldown must be at least one step")
        if not math.isfinite(self.goal_tolerance_m) or self.goal_tolerance_m < 0.0:
            raise ValueError("route goal tolerance must be finite and non-negative")


@dataclass(frozen=True)
class RoutePlan:
    status: str
    waypoints_xy: np.ndarray
    expanded_nodes: int
    raw_grid_nodes: int
    smoothed_nodes: int
    path_length_m: float

    @property
    def valid(self) -> bool:
        return self.status == "ok"


def conservative_xy_support_from_box(box_xyz: Sequence[float]) -> np.ndarray:
    """Return a finite XY support envelope safe under every future tilt/yaw orientation.

    The world-axis support of an oriented box cannot exceed its 3-D half-diagonal. Using this
    circumscribed radius on both horizontal axes is slightly conservative at the declared 45-deg
    tilt limit, but unlike current-pose support it cannot become invalid after the route is cached.
    """
    size = np.asarray(box_xyz, dtype=np.float64)
    if size.shape != (3,) or not np.isfinite(size).all() or np.any(size <= 0.0):
        raise ValueError("physical target box_xyz must contain three finite positive sizes")
    radius = 0.5 * float(np.linalg.norm(size))
    return np.array([radius, radius], dtype=np.float64)


def _finite_xy(value, name: str) -> np.ndarray:
    result = np.asarray(value, dtype=np.float64)
    if result.shape != (2,) or not np.isfinite(result).all():
        raise ValueError(f"{name} must be a finite XY pair")
    return result


def _segment_intersects_closed_aabb(p0, p1, lo, hi, epsilon=1e-12) -> bool:
    """Exact slab test. Touching an inflated AABB is unsafe (closed-set collision)."""
    direction = p1 - p0
    enter, leave = 0.0, 1.0
    for axis in (0, 1):
        if abs(direction[axis]) <= epsilon:
            if p0[axis] < lo[axis] or p0[axis] > hi[axis]:
                return False
            continue
        ta = (lo[axis] - p0[axis]) / direction[axis]
        tb = (hi[axis] - p0[axis]) / direction[axis]
        if ta > tb:
            ta, tb = tb, ta
        enter = max(enter, ta)
        leave = min(leave, tb)
        if enter > leave:
            return False
    return enter <= leave and leave >= 0.0 and enter <= 1.0


def segment_is_safe(
    p0,
    p1,
    admissible_lo,
    admissible_hi,
    bars_xy,
    inflated_half_extents_xy,
) -> bool:
    """Continuous line-of-sight contract used by smoothing and unit tests."""
    p0 = _finite_xy(p0, "p0")
    p1 = _finite_xy(p1, "p1")
    admissible_lo = _finite_xy(admissible_lo, "admissible_lo")
    admissible_hi = _finite_xy(admissible_hi, "admissible_hi")
    if np.any(admissible_hi <= admissible_lo):
        return False
    # The admissible arena is convex; endpoint inclusion proves the entire segment is inside.
    if np.any(p0 <= admissible_lo) or np.any(p0 >= admissible_hi):
        return False
    if np.any(p1 <= admissible_lo) or np.any(p1 >= admissible_hi):
        return False
    bars_xy = np.asarray(bars_xy, dtype=np.float64).reshape((-1, 2))
    half = np.asarray(inflated_half_extents_xy, dtype=np.float64).reshape((-1, 2))
    if bars_xy.shape != half.shape or not np.isfinite(bars_xy).all() or not np.isfinite(half).all():
        return False
    if np.any(half < 0.0):
        return False
    for center, extent in zip(bars_xy, half):
        if _segment_intersects_closed_aabb(p0, p1, center - extent, center + extent):
            return False
    return True


class DeterministicAStarRoutePlanner:
    """CPU A* with exact AABB inflation and safe line-of-sight smoothing."""

    def __init__(self, config: RoutePlannerConfig):
        config.validate()
        self.config = config

    def _grid(self, arena_lo, arena_hi):
        extent = arena_hi - arena_lo
        shape = np.maximum(1, np.floor(extent / self.config.resolution_m).astype(np.int64))
        axes = tuple(
            arena_lo[axis]
            + (np.arange(int(shape[axis]), dtype=np.float64) + 0.5)
            * self.config.resolution_m
            for axis in (0, 1)
        )
        return axes, (int(shape[0]), int(shape[1]))

    @staticmethod
    def _cell(point, axes):
        return tuple(
            int(np.argmin(np.abs(axis - point[k])))
            for k, axis in enumerate(axes)
        )

    def _anchor_cell(
        self, point, axes, free, admissible_lo, admissible_hi, bars, inflated_half
    ):
        """Map a continuous safe endpoint to a nearby grid node with a proven safe connector.

        A safe point may lie just outside the inflated AABB while its nearest cell centre lies
        inside it. Rejecting that endpoint is a rasterisation artefact, not a geometric failure.
        Search a bounded deterministic neighbourhood and retain the continuous slab test.
        """
        base_i, base_j = self._cell(point, axes)
        candidates = []
        for di in range(-3, 4):
            for dj in range(-3, 4):
                i, j = base_i + di, base_j + dj
                if 0 <= i < free.shape[0] and 0 <= j < free.shape[1] and free[i, j]:
                    cell_point = np.array([axes[0][i], axes[1][j]], dtype=np.float64)
                    candidates.append((float(np.sum((cell_point - point) ** 2)), i, j))
        for _, i, j in sorted(candidates):
            cell_point = np.array([axes[0][i], axes[1][j]], dtype=np.float64)
            if segment_is_safe(
                point, cell_point, admissible_lo, admissible_hi, bars, inflated_half
            ):
                return (i, j)
        return None

    def plan(
        self,
        start_xy,
        goal_xy,
        bars_xy,
        bars_half_extents_xy,
        arena_lo_xy,
        arena_hi_xy,
        target_support_xy,
    ) -> RoutePlan:
        try:
            start = _finite_xy(start_xy, "start_xy")
            goal = _finite_xy(goal_xy, "goal_xy")
            arena_lo = _finite_xy(arena_lo_xy, "arena_lo_xy")
            arena_hi = _finite_xy(arena_hi_xy, "arena_hi_xy")
            support = _finite_xy(target_support_xy, "target_support_xy")
            bars = np.asarray(bars_xy, dtype=np.float64).reshape((-1, 2))
            half = np.asarray(bars_half_extents_xy, dtype=np.float64).reshape((-1, 2))
        except (TypeError, ValueError):
            return RoutePlan("invalid_input", np.empty((0, 2)), 0, 0, 0, 0.0)
        if (
            bars.shape != half.shape
            or not np.isfinite(bars).all()
            or not np.isfinite(half).all()
            or np.any(half < 0.0)
            or np.any(support < 0.0)
            or np.any(arena_hi <= arena_lo)
        ):
            return RoutePlan("invalid_input", np.empty((0, 2)), 0, 0, 0, 0.0)

        admissible_lo = arena_lo + self.config.boundary_margin_m + support
        admissible_hi = arena_hi - self.config.boundary_margin_m - support
        inflated_half = half + support[None, :] + self.config.tracking_margin_m
        if not segment_is_safe(start, start, admissible_lo, admissible_hi, bars, inflated_half):
            return RoutePlan("unsafe_start", np.empty((0, 2)), 0, 0, 0, 0.0)
        if not segment_is_safe(goal, goal, admissible_lo, admissible_hi, bars, inflated_half):
            return RoutePlan("unsafe_goal", np.empty((0, 2)), 0, 0, 0, 0.0)
        if segment_is_safe(start, goal, admissible_lo, admissible_hi, bars, inflated_half):
            points = np.stack((start, goal))
            return RoutePlan("ok", points, 0, 2, 2, float(np.linalg.norm(goal - start)))

        axes, shape = self._grid(arena_lo, arena_hi)
        xx, yy = np.meshgrid(axes[0], axes[1], indexing="ij")
        free = (
            (xx > admissible_lo[0])
            & (xx < admissible_hi[0])
            & (yy > admissible_lo[1])
            & (yy < admissible_hi[1])
        )
        for center, extent in zip(bars, inflated_half):
            free &= ~(
                (np.abs(xx - center[0]) <= extent[0])
                & (np.abs(yy - center[1]) <= extent[1])
            )
        start_cell = self._anchor_cell(
            start, axes, free, admissible_lo, admissible_hi, bars, inflated_half
        )
        goal_cell = self._anchor_cell(
            goal, axes, free, admissible_lo, admissible_hi, bars, inflated_half
        )
        if start_cell is None:
            return RoutePlan("unsafe_start_cell", np.empty((0, 2)), 0, 0, 0, 0.0)
        if goal_cell is None:
            return RoutePlan("unsafe_goal_cell", np.empty((0, 2)), 0, 0, 0, 0.0)

        g_score = np.full(shape, np.inf, dtype=np.float64)
        parent_i = np.full(shape, -1, dtype=np.int32)
        parent_j = np.full(shape, -1, dtype=np.int32)
        g_score[start_cell] = 0.0

        def heuristic(i, j):
            return math.hypot(i - goal_cell[0], j - goal_cell[1])

        queue = [(heuristic(*start_cell), 0.0, start_cell[0], start_cell[1])]
        expanded = 0
        while queue:
            _, cost, i, j = heapq.heappop(queue)
            if cost != g_score[i, j]:
                continue
            expanded += 1
            if expanded > self.config.max_expansions:
                return RoutePlan("expansion_limit", np.empty((0, 2)), expanded, 0, 0, 0.0)
            if (i, j) == goal_cell:
                break
            for di, dj, step_cost in _NEIGHBORS:
                ii, jj = i + di, j + dj
                if not (0 <= ii < shape[0] and 0 <= jj < shape[1] and free[ii, jj]):
                    continue
                # No diagonal corner cutting. Exact smoothing below applies the stronger AABB test.
                if di and dj and (not free[i + di, j] or not free[i, j + dj]):
                    continue
                candidate = cost + step_cost
                if candidate < g_score[ii, jj]:
                    g_score[ii, jj] = candidate
                    parent_i[ii, jj], parent_j[ii, jj] = i, j
                    priority = candidate + heuristic(ii, jj)
                    heapq.heappush(queue, (priority, candidate, ii, jj))
        if not np.isfinite(g_score[goal_cell]):
            return RoutePlan("no_path", np.empty((0, 2)), expanded, 0, 0, 0.0)

        cells = [goal_cell]
        cursor = goal_cell
        while cursor != start_cell:
            cursor = (int(parent_i[cursor]), int(parent_j[cursor]))
            if cursor[0] < 0 or cursor[1] < 0:
                return RoutePlan("parent_chain_invalid", np.empty((0, 2)), expanded, 0, 0, 0.0)
            cells.append(cursor)
        cells.reverse()
        # Preserve both endpoint anchor cells. Omitting an anchor can join the continuous
        # endpoint directly to the *second* grid node and cut an inflated AABB corner even though
        # endpoint->anchor and anchor->neighbour are each safe.
        raw = [start]
        for i, j in cells:
            point = np.array([axes[0][i], axes[1][j]])
            if np.linalg.norm(point - raw[-1]) > 1e-9:
                raw.append(point)
        if np.linalg.norm(goal - raw[-1]) > 1e-9:
            raw.append(goal)

        # Greedy farthest-visible smoothing. Every accepted edge passes the continuous exact-AABB
        # contract; no waypoint is removed merely because the raster path looked clear.
        smoothed = [raw[0]]
        anchor = 0
        while anchor < len(raw) - 1:
            candidate = len(raw) - 1
            while candidate > anchor + 1 and not segment_is_safe(
                raw[anchor], raw[candidate], admissible_lo, admissible_hi, bars, inflated_half
            ):
                candidate -= 1
            if not segment_is_safe(
                raw[anchor], raw[candidate], admissible_lo, admissible_hi, bars, inflated_half
            ):
                return RoutePlan("smoothing_invalid", np.empty((0, 2)), expanded, len(raw), 0, 0.0)
            smoothed.append(raw[candidate])
            anchor = candidate
        points = np.asarray(smoothed, dtype=np.float64)
        if points.shape[0] > self.config.max_waypoints:
            return RoutePlan("waypoint_limit", np.empty((0, 2)), expanded, len(raw), len(points), 0.0)
        length = float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum())
        return RoutePlan("ok", points, expanded, len(raw), len(points), length)

    def plan_to_connected_goal(
        self,
        start_xy,
        bars_xy,
        bars_half_extents_xy,
        arena_lo_xy,
        arena_hi_xy,
        target_support_xy,
        min_goal_distance_m: float,
        selector: float,
    ) -> RoutePlan:
        """Choose a sufficiently distant goal proven reachable by the same planner contract.

        Candidate order is deterministic for ``selector``. A returned goal is in the start's
        connected component by construction because ``plan`` has produced a valid path to it.
        Failure returns no route; it never falls back to an unproved random waypoint.
        """
        try:
            start = _finite_xy(start_xy, "start_xy")
            arena_lo = _finite_xy(arena_lo_xy, "arena_lo_xy")
            arena_hi = _finite_xy(arena_hi_xy, "arena_hi_xy")
            selector = float(selector)
            minimum = float(min_goal_distance_m)
        except (TypeError, ValueError):
            return RoutePlan("invalid_input", np.empty((0, 2)), 0, 0, 0, 0.0)
        if (
            not math.isfinite(selector)
            or not math.isfinite(minimum)
            or minimum <= 0.0
        ):
            return RoutePlan("invalid_input", np.empty((0, 2)), 0, 0, 0, 0.0)
        try:
            bars = np.asarray(bars_xy, dtype=np.float64).reshape((-1, 2))
            half = np.asarray(bars_half_extents_xy, dtype=np.float64).reshape((-1, 2))
            support = _finite_xy(target_support_xy, "target_support_xy")
        except (TypeError, ValueError):
            return RoutePlan("invalid_input", np.empty((0, 2)), 0, 0, 0, 0.0)
        if (
            bars.shape != half.shape
            or not np.isfinite(bars).all()
            or not np.isfinite(half).all()
            or np.any(half < 0.0)
            or np.any(support < 0.0)
        ):
            return RoutePlan("invalid_input", np.empty((0, 2)), 0, 0, 0, 0.0)

        admissible_lo = arena_lo + self.config.boundary_margin_m + support
        admissible_hi = arena_hi - self.config.boundary_margin_m - support
        inflated_half = half + support[None, :] + self.config.tracking_margin_m
        if not segment_is_safe(start, start, admissible_lo, admissible_hi, bars, inflated_half):
            return RoutePlan("unsafe_start", np.empty((0, 2)), 0, 0, 0, 0.0)
        axes, shape = self._grid(arena_lo, arena_hi)
        xx, yy = np.meshgrid(axes[0], axes[1], indexing="ij")
        free = (
            (xx > admissible_lo[0])
            & (xx < admissible_hi[0])
            & (yy > admissible_lo[1])
            & (yy < admissible_hi[1])
        )
        for center, extent in zip(bars, inflated_half):
            free &= ~(
                (np.abs(xx - center[0]) <= extent[0])
                & (np.abs(yy - center[1]) <= extent[1])
            )
        start_cell = self._anchor_cell(
            start, axes, free, admissible_lo, admissible_hi, bars, inflated_half
        )
        if start_cell is None:
            return RoutePlan("unsafe_start_cell", np.empty((0, 2)), 0, 0, 0, 0.0)

        # One deterministic Dijkstra expansion gives both the connected component and the parent
        # tree. Unlike the previous candidate loop, occupancy and search are never repeated up to
        # 64 times for one reset.
        distance = np.full(shape, np.inf, dtype=np.float64)
        parent_i = np.full(shape, -1, dtype=np.int32)
        parent_j = np.full(shape, -1, dtype=np.int32)
        distance[start_cell] = 0.0
        queue = [(0.0, start_cell[0], start_cell[1])]
        expanded = 0
        while queue:
            cost, i, j = heapq.heappop(queue)
            if cost != distance[i, j]:
                continue
            expanded += 1
            if expanded > self.config.max_expansions:
                return RoutePlan(
                    "expansion_limit", np.empty((0, 2)), expanded, 0, 0, 0.0
                )
            for di, dj, step_cost in _NEIGHBORS:
                ii, jj = i + di, j + dj
                if not (0 <= ii < shape[0] and 0 <= jj < shape[1] and free[ii, jj]):
                    continue
                if di and dj and (not free[i + di, j] or not free[i, j + dj]):
                    continue
                candidate = cost + step_cost
                if candidate < distance[ii, jj]:
                    distance[ii, jj] = candidate
                    parent_i[ii, jj], parent_j[ii, jj] = i, j
                    heapq.heappush(queue, (candidate, ii, jj))

        reachable_i, reachable_j = np.nonzero(np.isfinite(distance))
        reachable_points = np.stack(
            (axes[0][reachable_i], axes[1][reachable_j]), axis=1
        )
        distant = np.linalg.norm(reachable_points - start[None, :], axis=1) >= minimum
        reachable_i, reachable_j = reachable_i[distant], reachable_j[distant]
        if not len(reachable_i):
            return RoutePlan("no_connected_goal", np.empty((0, 2)), 0, 0, 0, 0.0)
        choice = min(len(reachable_i) - 1, int(math.floor((selector % 1.0) * len(reachable_i))))
        goal_cell = (int(reachable_i[choice]), int(reachable_j[choice]))
        goal = np.array([axes[0][goal_cell[0]], axes[1][goal_cell[1]]])
        cells = [goal_cell]
        cursor = goal_cell
        while cursor != start_cell:
            cursor = (int(parent_i[cursor]), int(parent_j[cursor]))
            if cursor[0] < 0 or cursor[1] < 0:
                return RoutePlan(
                    "parent_chain_invalid", np.empty((0, 2)), expanded, 0, 0, 0.0
                )
            cells.append(cursor)
        cells.reverse()
        raw = [start]
        for i, j in cells:
            point = np.array([axes[0][i], axes[1][j]])
            if np.linalg.norm(point - raw[-1]) > 1e-9:
                raw.append(point)
        smoothed = [raw[0]]
        anchor = 0
        while anchor < len(raw) - 1:
            candidate = len(raw) - 1
            while candidate > anchor + 1 and not segment_is_safe(
                raw[anchor], raw[candidate], admissible_lo, admissible_hi, bars, inflated_half
            ):
                candidate -= 1
            if not segment_is_safe(
                raw[anchor], raw[candidate], admissible_lo, admissible_hi, bars, inflated_half
            ):
                return RoutePlan(
                    "smoothing_invalid", np.empty((0, 2)), expanded, len(raw), 0, 0.0
                )
            smoothed.append(raw[candidate])
            anchor = candidate
        points = np.asarray(smoothed, dtype=np.float64)
        if len(points) > self.config.max_waypoints:
            return RoutePlan(
                "waypoint_limit", np.empty((0, 2)), expanded, len(raw), len(points), 0.0
            )
        length = float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum())
        return RoutePlan("ok", points, expanded, len(raw), len(points), length)


class BatchedTargetRouteManager:
    """Per-environment cached routes with GPU-vectorised waypoint following."""

    STATUS_CODES = {
        "unplanned": 0,
        "ok": 1,
        "invalid_input": 2,
        "unsafe_start": 3,
        "unsafe_goal": 4,
        "unsafe_start_cell": 5,
        "unsafe_goal_cell": 6,
        "no_path": 7,
        "expansion_limit": 8,
        "parent_chain_invalid": 9,
        "smoothing_invalid": 10,
        "waypoint_limit": 11,
        "local_step_infeasible": 12,
        "support_contract_changed": 13,
        "goal_changed": 14,
        "no_connected_goal": 15,
    }

    def __init__(self, num_envs: int, device, config: RoutePlannerConfig):
        config.validate()
        self.num_envs = int(num_envs)
        self.device = device
        self.config = config
        self.planner = DeterministicAStarRoutePlanner(config)
        self.waypoints = torch.zeros(
            (self.num_envs, config.max_waypoints, 2), dtype=torch.float32, device=device
        )
        self.length = torch.zeros(self.num_envs, dtype=torch.long, device=device)
        self.cursor = torch.zeros(self.num_envs, dtype=torch.long, device=device)
        self.valid = torch.zeros(self.num_envs, dtype=torch.bool, device=device)
        self.goal = torch.zeros((self.num_envs, 2), dtype=torch.float32, device=device)
        self.segment_start = torch.zeros_like(self.goal)
        self.planned_support = torch.zeros_like(self.goal)
        self.next_replan_step = torch.zeros(self.num_envs, dtype=torch.long, device=device)
        self.status_code = torch.zeros(self.num_envs, dtype=torch.long, device=device)
        self.plan_attempts = 0
        self.plan_successes = 0
        self.replan_attempts = 0
        self.no_path_count = 0
        self.invalid_count = 0
        # Runtime counters remain device-side so the normal control path does not
        # force a GPU synchronization every interval.  They are materialized only
        # when bulk-evaluation diagnostics are exported.
        self.runtime_invalid_count = torch.zeros((), dtype=torch.long, device=device)
        self.fallback_intervals = torch.zeros((), dtype=torch.long, device=device)
        self.expanded_nodes = 0
        self.raw_waypoints = 0
        self.smoothed_waypoints = 0

    def reset_idx(self, env_ids) -> None:
        if len(env_ids) == 0:
            return
        self.length[env_ids] = 0
        self.cursor[env_ids] = 0
        self.valid[env_ids] = False
        self.status_code[env_ids] = self.STATUS_CODES["unplanned"]
        self.next_replan_step[env_ids] = 0

    def invalidate(self, mask, reason: str, current_step: int) -> None:
        if reason not in self.STATUS_CODES:
            raise ValueError(f"unknown route invalidation reason {reason!r}")
        if mask.dtype != torch.bool or mask.shape != (self.num_envs,):
            raise ValueError("route invalidation mask must have shape [N] and bool dtype")
        self.valid[mask] = False
        self.status_code[mask] = self.STATUS_CODES[reason]
        self.next_replan_step[mask] = int(current_step) + self.config.replan_cooldown_steps
        self.runtime_invalid_count += mask.sum()

    def needs_replan(self, goal_xy, support_xy, current_step: int):
        if goal_xy.shape != self.goal.shape or support_xy.shape != self.planned_support.shape:
            raise ValueError("goal/support must have shape [N, 2]")
        goal_changed = (goal_xy - self.goal).norm(dim=1) > self.config.goal_tolerance_m
        support_changed = (support_xy > self.planned_support + 1e-6).any(dim=1)
        self.status_code[goal_changed] = self.STATUS_CODES["goal_changed"]
        self.status_code[support_changed] = self.STATUS_CODES["support_contract_changed"]
        self.valid[goal_changed | support_changed] = False
        cooldown_done = self.next_replan_step <= int(current_step)
        return (~self.valid) & cooldown_done

    def plan_idx(
        self,
        env_ids,
        start_xy,
        goal_xy,
        bars_xy,
        bars_half_extents_xy,
        arena_lo_xy,
        arena_hi_xy,
        support_xy,
        current_step: int,
        is_replan: bool = False,
        connected_goal_selector=None,
        min_goal_distance_m: float = 0.0,
    ) -> Dict[str, int]:
        if len(env_ids) == 0:
            return {}
        ids = env_ids.detach().to("cpu", dtype=torch.long).tolist()
        selected = [
            value.detach().to("cpu", dtype=torch.float64).numpy()
            for value in (
                start_xy[env_ids],
                goal_xy[env_ids],
                bars_xy[env_ids],
                bars_half_extents_xy[env_ids],
                arena_lo_xy[env_ids],
                arena_hi_xy[env_ids],
                support_xy[env_ids],
            )
        ]
        starts, goals, bars, half, arena_lo, arena_hi, support = selected
        batch_counts: Dict[str, int] = {}
        for local, env_id in enumerate(ids):
            if connected_goal_selector is None:
                result = self.planner.plan(
                    starts[local], goals[local], bars[local], half[local],
                    arena_lo[local], arena_hi[local], support[local]
                )
            else:
                selector = float(connected_goal_selector[env_ids[local]].item())
                result = self.planner.plan_to_connected_goal(
                    starts[local], bars[local], half[local], arena_lo[local],
                    arena_hi[local], support[local], min_goal_distance_m, selector
                )
            self.plan_attempts += 1
            self.replan_attempts += int(is_replan)
            self.expanded_nodes += result.expanded_nodes
            self.raw_waypoints += result.raw_grid_nodes
            self.smoothed_waypoints += result.smoothed_nodes
            batch_counts[result.status] = batch_counts.get(result.status, 0) + 1
            selected_goal = (
                torch.as_tensor(
                    result.waypoints_xy[-1], dtype=self.goal.dtype, device=self.device
                )
                if result.valid
                else goal_xy[env_id]
            )
            self.goal[env_id] = selected_goal
            self.planned_support[env_id] = support_xy[env_id]
            self.next_replan_step[env_id] = int(current_step) + self.config.replan_cooldown_steps
            self.status_code[env_id] = self.STATUS_CODES[result.status]
            if not result.valid:
                self.valid[env_id] = False
                self.length[env_id] = 0
                self.no_path_count += int(result.status == "no_path")
                self.invalid_count += int(result.status != "no_path")
                continue
            count = result.waypoints_xy.shape[0] - 1  # exclude current start
            route = torch.as_tensor(
                result.waypoints_xy[1:], dtype=self.waypoints.dtype, device=self.device
            )
            self.waypoints[env_id].zero_()
            self.waypoints[env_id, :count] = route
            self.segment_start[env_id] = start_xy[env_id]
            self.length[env_id] = count
            self.cursor[env_id] = 0
            self.valid[env_id] = count > 0
            self.plan_successes += 1
        return batch_counts

    def velocity_reference(self, position_xy, speed, reach_m: float):
        if position_xy.shape != self.goal.shape or speed.shape != (self.num_envs,):
            raise ValueError("position must be [N,2] and speed [N]")
        rows = torch.arange(self.num_envs, device=self.device)
        # Smoothed routes contain few points, but advance repeatedly so a slow/reset target cannot
        # spend extra control intervals commanding a waypoint already inside the reach radius.
        for _ in range(4):
            safe_cursor = torch.minimum(self.cursor, (self.length - 1).clamp(min=0))
            waypoint = self.waypoints[rows, safe_cursor]
            segment = waypoint - self.segment_start
            offset = position_xy - waypoint
            cross_track = torch.abs(offset[:, 0] * segment[:, 1] - offset[:, 1] * segment[:, 0])
            cross_track /= segment.norm(dim=1).clamp(min=1e-6)
            passed = ((offset * segment).sum(dim=1) >= 0.0) & (
                cross_track <= float(reach_m)
            )
            reached = self.valid & (
                ((waypoint - position_xy).norm(dim=1) <= float(reach_m)) | passed
            )
            can_advance = reached & (self.cursor + 1 < self.length)
            self.segment_start[can_advance] = waypoint[can_advance]
            self.cursor += can_advance.long()
        safe_cursor = torch.minimum(self.cursor, (self.length - 1).clamp(min=0))
        waypoint = self.waypoints[rows, safe_cursor]
        delta = waypoint - position_xy
        final_segment = waypoint - self.segment_start
        final_offset = position_xy - waypoint
        final_cross_track = torch.abs(
            final_offset[:, 0] * final_segment[:, 1]
            - final_offset[:, 1] * final_segment[:, 0]
        ) / final_segment.norm(dim=1).clamp(min=1e-6)
        passed_final = ((final_offset * final_segment).sum(dim=1) >= 0.0) & (
            final_cross_track <= float(reach_m)
        )
        complete = self.valid & (self.cursor + 1 >= self.length) & (
            (delta.norm(dim=1) <= float(reach_m)) | passed_final
        )
        active = self.valid & ~complete
        direction = delta / delta.norm(dim=1, keepdim=True).clamp(min=1e-6)
        velocity = torch.where(
            active.unsqueeze(1), direction * speed.clamp(min=0.0).unsqueeze(1),
            torch.zeros_like(position_xy)
        )
        self.fallback_intervals += (~self.valid & (speed > 1e-6)).sum()
        return velocity, active, complete

    def diagnostics(self) -> Dict[str, object]:
        reverse = {value: key for key, value in self.STATUS_CODES.items()}
        codes, counts = torch.unique(self.status_code, return_counts=True)
        status_counts = {
            reverse.get(int(code), f"unknown_{int(code)}"): int(count)
            for code, count in zip(codes.detach().cpu(), counts.detach().cpu())
        }
        return {
            "mode": TARGET_ROUTE_MODE_GLOBAL_ASTAR,
            "plan_attempts": self.plan_attempts,
            "plan_successes": self.plan_successes,
            "replan_attempts": self.replan_attempts,
            "no_path_count": self.no_path_count,
            "invalid_count": self.invalid_count + int(self.runtime_invalid_count.item()),
            "fallback_intervals": int(self.fallback_intervals.item()),
            "expanded_nodes": self.expanded_nodes,
            "raw_waypoints": self.raw_waypoints,
            "smoothed_waypoints": self.smoothed_waypoints,
            "currently_valid": int(self.valid.sum().item()),
            "status_counts": status_counts,
        }
