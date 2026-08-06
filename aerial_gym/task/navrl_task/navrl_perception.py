"""Perception front-end for the NavRL++-Target policy.

The actor never receives RGB-D pixels, LiDAR semantic IDs, or simulator target state.  This
module consumes raw RGB-D and raw LiDAR ranges, detects the target from appearance, associates
camera and LiDAR measurements, tracks target position/velocity/covariance, and emits a fixed-size
structured history for the Transformer policy.

The simulator may use ground truth to *render* sensor pixels, just as a real scene produces sensor
measurements.  Ground truth is intentionally absent from every public method in this file.
"""

import math
import os

import torch
import torch.nn as nn

from aerial_gym.task.navrl_task.navrl_corridor import CORRIDOR_DIM, extract_corridor_tokens


ROBOT_HISTORY = 5
TARGET_HISTORY = 5
OBSTACLE_HISTORY = 5
# Obstacle-token capacity. Env-overridable so it can be swept; changing it changes
# STRUCTURED_OBS_DIM and therefore requires a FRESH policy (no warm-start across capacities).
MAX_OBSTACLES = int(os.environ.get("NAVRL_MAX_OBSTACLES", "").strip() or 5)
ROBOT_DIM = 10
TARGET_DIM = 16
OBSTACLE_DIM = 12

# Agreement window that declares a sensor return "the target, not static geometry", used to carve
# the target out of the obstacle map. Named constants because P2's latency-aware variant must
# reproduce the same window from the PREDICTED target pose; two literals would silently drift.
TARGET_LIKE_ANGLE_RAD = math.radians(15.0)
TARGET_LIKE_RANGE_TOL_M = 0.55

# LiDAR scan resolution. HBEAMS is THE source of truth for the horizontal beam count and must equal
# navrl_lidar_config.width (both read NAVRL_LIDAR_HBEAMS). Changing it changes STRUCTURED_OBS_DIM,
# hence a fresh policy is required. The old claim that token error was dominated by angular
# quantization was refuted by evaluation; see WORKLOG 2026-07-27 and bar-probe v2.
HBEAMS = int(os.environ.get("NAVRL_LIDAR_HBEAMS", "").strip() or 36)
VBEAMS = int(os.environ.get("NAVRL_LIDAR_VBEAMS", "").strip() or 4)
# Angular half-width blanked around an accepted obstacle token, in DEGREES (not bins: the bin size
# changes with HBEAMS). It stops one wide bar from consuming several token slots, but it also caps
# how many tokens can EVER be filled at 360/(2*this). The original +-2 bins at 36 beams was +-20 deg
# = at most ~7 tokens, so raising MAX_OBSTACLES alone silently wasted the extra slots. At +-10 deg
# the ceiling is 18, comfortably above any capacity we intend to sweep.
OBSTACLE_SUPPRESS_DEG = float(os.environ.get("NAVRL_OBSTACLE_SUPPRESS_DEG", "").strip() or 10.0)
# Obstacle proposal policy. ``greedy_suppress`` preserves the historical nearest-bearing selector.
# ``cluster_sector`` first joins adjacent ray endpoints that plausibly lie on one physical surface,
# then reserves one nearest cluster per angular sector. Empty sectors are filled from the remaining
# nearest clusters. Both modes emit the same [MAX_OBSTACLES, OBSTACLE_DIM] shape.
OBSTACLE_SELECTOR = (
    os.environ.get("NAVRL_OBSTACLE_SELECTOR", "").strip().lower() or "greedy_suppress"
)
OBSTACLE_CLUSTER_GAP_M = float(
    os.environ.get("NAVRL_OBSTACLE_CLUSTER_GAP_M", "").strip() or 0.45
)
OBSTACLE_SECTORS = int(
    os.environ.get("NAVRL_OBSTACLE_SECTORS", "").strip() or MAX_OBSTACLES
)
# Angular sector (centered on body-forward, in DEGREES) the obstacle tokens are selected from.
# 360 = the original behavior: pick the nearest MAX_OBSTACLES bars anywhere around the drone.
# Why narrowing helps: the probe measured ~16 bars inside the horizon against 8 token slots, so a
# 360-degree search represents only ~50% of them -- which is exactly the measured hit_in_tokens
# (0.40-0.53). Spending slots on bars BEHIND the drone is what starves the ones ahead of it.
# Restricting the sector raises coverage geometrically (240 deg -> ~75%, 180 deg -> ~100%) WITHOUT
# changing the observation width, so a policy can warm-start across this change.
# Omnidirectional awareness is not lost: the static scan token still carries all HBEAMS bearings.
OBSTACLE_FOV_DEG = float(os.environ.get("NAVRL_OBSTACLE_FOV_DEG", "").strip() or 360.0)
if MAX_OBSTACLES <= 0:
    raise ValueError("NAVRL_MAX_OBSTACLES must be positive")
if HBEAMS <= 0 or VBEAMS <= 0:
    raise ValueError("NAVRL_LIDAR_HBEAMS and NAVRL_LIDAR_VBEAMS must be positive")
if not math.isfinite(OBSTACLE_SUPPRESS_DEG) or OBSTACLE_SUPPRESS_DEG < 0.0:
    raise ValueError("NAVRL_OBSTACLE_SUPPRESS_DEG must be finite and non-negative")
if OBSTACLE_SELECTOR not in ("greedy_suppress", "cluster_sector", "ttc_sector"):
    raise ValueError(
        "NAVRL_OBSTACLE_SELECTOR must be 'greedy_suppress', 'cluster_sector' or 'ttc_sector'"
    )
# ttc_sector only: a cluster that the drone is not closing on is ranked by this fallback time
# instead of being dropped, so a slow/hovering drone still tokenizes its surroundings by proximity
# rather than emitting an empty set.
OBSTACLE_TTC_IDLE_S = float(
    os.environ.get("NAVRL_OBSTACLE_TTC_IDLE_S", "").strip() or 30.0
)
# Speed below which closing rate is treated as unreliable and selection degrades to nearest-first.
OBSTACLE_TTC_MIN_SPEED = float(
    os.environ.get("NAVRL_OBSTACLE_TTC_MIN_SPEED", "").strip() or 0.15
)
if not math.isfinite(OBSTACLE_TTC_IDLE_S) or OBSTACLE_TTC_IDLE_S <= 0.0:
    raise ValueError("NAVRL_OBSTACLE_TTC_IDLE_S must be finite and positive")
if not math.isfinite(OBSTACLE_TTC_MIN_SPEED) or OBSTACLE_TTC_MIN_SPEED < 0.0:
    raise ValueError("NAVRL_OBSTACLE_TTC_MIN_SPEED must be finite and non-negative")
if not math.isfinite(OBSTACLE_CLUSTER_GAP_M) or OBSTACLE_CLUSTER_GAP_M <= 0.0:
    raise ValueError("NAVRL_OBSTACLE_CLUSTER_GAP_M must be finite and positive")
if OBSTACLE_SECTORS <= 0 or OBSTACLE_SECTORS > MAX_OBSTACLES:
    raise ValueError("NAVRL_OBSTACLE_SECTORS must be in 1..NAVRL_MAX_OBSTACLES")
if not math.isfinite(OBSTACLE_FOV_DEG) or not 0.0 < OBSTACLE_FOV_DEG <= 360.0:
    raise ValueError("NAVRL_OBSTACLE_FOV_DEG must be in (0, 360]")


def obstacle_selector_provenance(selector, configured_fov_deg):
    """Return the candidate FOV and whether angular suppression actually executes."""
    selector = str(selector).strip().lower()
    configured_fov_deg = float(configured_fov_deg)
    if selector not in ("greedy_suppress", "cluster_sector", "ttc_sector"):
        raise ValueError("unsupported obstacle selector: %s" % selector)
    if not math.isfinite(configured_fov_deg) or not 0.0 < configured_fov_deg <= 360.0:
        raise ValueError("configured_fov_deg must be in (0, 360]")
    return {
        "effective_fov_deg": 360.0 if selector == "ttc_sector" else configured_fov_deg,
        "suppress_active": selector == "greedy_suppress",
    }


# Provenance must describe the selector's actual candidate set. TTC intentionally ranks all 360
# degrees, while cluster_sector/greedy_suppress obey the configured FOV. Suppression is only used
# by the legacy greedy selector; logging it as active for cluster/TTC previously misdescribed runs.
_OBSTACLE_PROVENANCE = obstacle_selector_provenance(OBSTACLE_SELECTOR, OBSTACLE_FOV_DEG)
OBSTACLE_EFFECTIVE_FOV_DEG = _OBSTACLE_PROVENANCE["effective_fov_deg"]
OBSTACLE_SUPPRESS_ACTIVE = _OBSTACLE_PROVENANCE["suppress_active"]
# Corridor (free-gap) affordance tokens -- see navrl_corridor.py. 0 (the default) keeps the
# historical 898-D schema byte-identical; any positive count APPENDS
# CORRIDOR_TOKENS * CORRIDOR_DIM features at the END of the observation, so every existing
# segment keeps its offset and a checkpoint can be warm-started by expanding only the input
# projections (see runner._expand_corridor_checkpoint). Changing this is a schema change:
# provenance is recorded in env_state as cfg_corridor_tokens.
CORRIDOR_TOKENS = int(os.environ.get("NAVRL_CORRIDOR_TOKENS", "").strip() or 0)
CORRIDOR_HORIZON_M = float(os.environ.get("NAVRL_CORRIDOR_HORIZON_M", "").strip() or 6.0)
CORRIDOR_MIN_WIDTH_M = float(os.environ.get("NAVRL_CORRIDOR_MIN_WIDTH_M", "").strip() or 0.55)
if CORRIDOR_TOKENS < 0 or CORRIDOR_TOKENS > 16:
    raise ValueError("NAVRL_CORRIDOR_TOKENS must be in 0..16")
if not math.isfinite(CORRIDOR_HORIZON_M) or CORRIDOR_HORIZON_M <= 0.0:
    raise ValueError("NAVRL_CORRIDOR_HORIZON_M must be finite and positive")
if not math.isfinite(CORRIDOR_MIN_WIDTH_M) or CORRIDOR_MIN_WIDTH_M < 0.0:
    raise ValueError("NAVRL_CORRIDOR_MIN_WIDTH_M must be finite and non-negative")
CORRIDOR_OBS_DIM = CORRIDOR_TOKENS * CORRIDOR_DIM
STATIC_DIM = VBEAMS * HBEAMS


def lidar_bin_bearings(device=None):
    """Physical body-frame bearing of each horizontal scan bin, in radians.

    THE single source of truth for bin -> bearing. It mirrors the warp ray generator
    (warp_lidar.py:67: azimuth = hfov_max - span * j/(W-1)), so the bearing DECREASES with the bin
    index: bin 0 = +180 deg, bin W-1 = -(180 - 360/W) deg.

    History: this module used to assume the INCREASING convention (linspace(-180+bin, +180)),
    which is the mirror image (assumed = bin_deg - true). Every obstacle token was therefore
    emitted on the wrong side of the drone, the camera fusion ghosted forward bars onto the
    opposite bearing, and the LiDAR target re-association read the mirrored bin. Physically
    adjudicated with tools/probe_lidar_bearing.py: back-projecting real returns through the
    increasing table lands on a GT bar only 13.9% of the time (mean error 2.5 m); through this
    decreasing table 94.8% (mean error 0.47 m ~= bar surface-to-center). Any edit here must keep
    tests/test_lidar_bearing_convention.py and that probe passing.
    """
    bin_rad = 2.0 * math.pi / HBEAMS
    return torch.linspace(math.pi, -math.pi + bin_rad, HBEAMS, device=device)


def camera_no_return_to_lidar_range(camera_col_min, camera_max_range, lidar_max_range):
    """Map camera columns that saw NOTHING to the LiDAR no-return value before min-fusion.

    The obstacle depth camera fills no-hit rays with its own far plane (camera_obstacle_max_range,
    10 m). When the LiDAR range exceeds that (12 m), min-fusing the raw fill value stamps a phantom
    10 m wall across the whole forward sector: the actor can never observe true free space ahead,
    and the token selector manufactures obstacles out of the camera's horizon. A column at (or
    beyond) the camera far plane carries no obstacle information, so it must fuse as lidar_max
    (= no return), leaving the LiDAR's own measurement untouched.
    """
    no_return = camera_col_min >= float(camera_max_range) - 1e-3
    return torch.where(
        no_return, torch.full_like(camera_col_min, float(lidar_max_range)), camera_col_min
    )


def select_cluster_sector_obstacles(
    nearest,
    bearings,
    *,
    max_range,
    max_obstacles,
    token_fov_deg,
    cluster_gap_m,
    num_sectors,
):
    """Select distinct, angularly distributed LiDAR surface clusters.

    ``nearest`` is [batch, horizontal_beams]. Adjacent valid ray endpoints are assigned to the same
    cluster when their Euclidean separation is at most ``cluster_gap_m``. One nearest cluster is
    reserved per fixed body-frame angular sector, then empty sector capacity is filled with the
    nearest still-unselected clusters. Returned proposals are sorted by range so a same-shape
    warm-start retains the historical nearest-first slot convention as closely as possible.

    The intended experiment uses a forward 240-degree FOV, so its eligible bearings form one
    contiguous interval away from the +/-pi scan seam. Full-360 operation deliberately treats the
    seam as a boundary; the static scan token still retains the complete circular geometry.
    """
    if nearest.ndim != 2 or bearings.ndim != 1 or nearest.shape[1] != bearings.shape[0]:
        raise ValueError("nearest must be [batch, beams] and bearings must be [beams]")
    if max_obstacles <= 0 or num_sectors <= 0 or num_sectors > max_obstacles:
        raise ValueError("sector count must be in 1..max_obstacles")
    if not 0.0 < float(token_fov_deg) <= 360.0:
        raise ValueError("token_fov_deg must be in (0, 360]")
    if not math.isfinite(float(cluster_gap_m)) or float(cluster_gap_m) <= 0.0:
        raise ValueError("cluster_gap_m must be finite and positive")

    batch, beams = nearest.shape
    device = nearest.device
    max_range = float(max_range)
    half_fov = math.radians(float(token_fov_deg) * 0.5)
    eligible = nearest < max_range * 0.995
    if token_fov_deg < 359.9:
        eligible &= bearings.abs().view(1, beams) <= half_fov + 1e-7

    # A range jump alone is a poor object boundary: two rays on one nearby cylinder can differ in
    # range noticeably. Endpoint distance combines angular separation and radial change in metres.
    x = nearest * torch.cos(bearings).view(1, beams)
    y = nearest * torch.sin(bearings).view(1, beams)
    endpoint_gap = torch.sqrt(
        (x[:, 1:] - x[:, :-1]).square() + (y[:, 1:] - y[:, :-1]).square()
    )
    linked_to_previous = (
        eligible[:, 1:]
        & eligible[:, :-1]
        & (endpoint_gap <= float(cluster_gap_m))
    )
    boundary = torch.ones((batch, beams), dtype=torch.bool, device=device)
    boundary[:, 1:] = ~linked_to_previous
    cluster_id = boundary.long().cumsum(dim=1)

    remaining = eligible.clone()
    rows = torch.arange(batch, device=device)
    sector_ranges = []
    sector_indices = []
    sector_valid = []
    sector_width = (2.0 * half_fov) / float(num_sectors)

    for sector in range(num_sectors):
        low = -half_fov + sector * sector_width
        high = low + sector_width
        if sector == num_sectors - 1:
            in_sector = (bearings >= low - 1e-7) & (bearings <= high + 1e-7)
        else:
            in_sector = (bearings >= low - 1e-7) & (bearings < high - 1e-7)
        work = nearest.masked_fill(~(remaining & in_sector.view(1, beams)), max_range)
        picked_range, picked_idx = work.min(dim=1)
        picked_valid = picked_range < max_range * 0.995
        sector_ranges.append(picked_range)
        sector_indices.append(picked_idx)
        sector_valid.append(picked_valid)

        picked_cluster = cluster_id[rows, picked_idx]
        remove = cluster_id.eq(picked_cluster.unsqueeze(1)) & picked_valid.unsqueeze(1)
        remaining &= ~remove

    # Generate enough global candidates to fill every empty sector. They are lower-priority than
    # all valid sector representatives, irrespective of range.
    fallback_ranges = []
    fallback_indices = []
    fallback_valid = []
    for _ in range(max_obstacles):
        work = nearest.masked_fill(~remaining, max_range)
        picked_range, picked_idx = work.min(dim=1)
        picked_valid = picked_range < max_range * 0.995
        fallback_ranges.append(picked_range)
        fallback_indices.append(picked_idx)
        fallback_valid.append(picked_valid)

        picked_cluster = cluster_id[rows, picked_idx]
        remove = cluster_id.eq(picked_cluster.unsqueeze(1)) & picked_valid.unsqueeze(1)
        remaining &= ~remove

    ranges = torch.stack(sector_ranges + fallback_ranges, dim=1)
    indices = torch.stack(sector_indices + fallback_indices, dim=1)
    valid = torch.stack(sector_valid + fallback_valid, dim=1)
    source_is_fallback = torch.cat(
        [
            torch.zeros(num_sectors, device=device, dtype=nearest.dtype),
            torch.ones(max_obstacles, device=device, dtype=nearest.dtype),
        ]
    ).view(1, -1)
    priority = source_is_fallback * 2.0 + ranges / max_range
    priority = priority.masked_fill(~valid, float("inf"))
    keep = priority.argsort(dim=1)[:, :max_obstacles]
    ranges = ranges.gather(1, keep)
    indices = indices.gather(1, keep)
    valid = valid.gather(1, keep)

    # Preserve nearest-first slot semantics for the downstream flattened MLP.
    range_order = ranges.masked_fill(~valid, float("inf")).argsort(dim=1)
    ranges = ranges.gather(1, range_order)
    indices = indices.gather(1, range_order)
    valid = valid.gather(1, range_order)
    return ranges, indices, valid


def select_ttc_obstacles(
    nearest,
    bearings,
    body_vel_xy,
    *,
    max_range,
    max_obstacles,
    cluster_gap_m,
    idle_ttc_s=30.0,
    min_speed=0.15,
):
    """Select the obstacle clusters the drone is most imminently going to hit.

    ``cluster_sector`` allocates slots by BEARING: one cluster per fixed forward sector. That is a
    proxy for threat, and the crash probe measured where the proxy breaks (run ppo_260731_1722,
    2300 epochs, 70 bars): 23.6% of the bars actually struck were outside the 240-degree token
    window entirely, and of those inside it, 11.7% still received no token because a sector only
    reserves its single nearest cluster. Both losses share one cause -- a bar is dangerous because
    the drone is MOVING INTO it, and bearing alone does not encode that. While searching, the drone
    yaws hard, so a bar that was forward moments ago sits behind the window while the velocity
    vector still points at it.

    This selector ranks clusters by time-to-collision instead:

        closing = -d(range)/dt ~= body_velocity . unit_vector_to_cluster
        ttc     = range / closing            when closing > 0
                = idle_ttc_s + range/max_range  otherwise (receding: ordered last, by proximity)

    Consequences: a receding bar dead ahead yields its slot to an approaching bar off to the side;
    a second cluster in one sector can be tokenized when it is the more urgent one; and no bearing
    is excluded a priori, so the rear blind sector disappears without widening any sensor. Below
    ``min_speed`` the closing estimate is dominated by noise, so ranking degrades continuously to
    nearest-first -- which is exactly ``cluster_sector`` behaviour at a standstill.

    Shape and semantics of the return match ``select_cluster_sector_obstacles`` exactly
    ([batch, max_obstacles] ranges/indices/valid, sorted nearest-first), so this is a same-width
    swap that a trained policy can warm-start across.
    """
    if nearest.ndim != 2 or bearings.ndim != 1 or nearest.shape[1] != bearings.shape[0]:
        raise ValueError("nearest must be [batch, beams] and bearings must be [beams]")
    if body_vel_xy.ndim != 2 or body_vel_xy.shape[0] != nearest.shape[0] or body_vel_xy.shape[1] != 2:
        raise ValueError("body_vel_xy must be [batch, 2]")
    if max_obstacles <= 0:
        raise ValueError("max_obstacles must be positive")
    if not math.isfinite(float(cluster_gap_m)) or float(cluster_gap_m) <= 0.0:
        raise ValueError("cluster_gap_m must be finite and positive")

    batch, beams = nearest.shape
    device = nearest.device
    max_range = float(max_range)
    eligible = nearest < max_range * 0.995

    # Identical clustering to cluster_sector: adjacent endpoints within cluster_gap_m are one
    # physical surface. Keeping this shared means the A/B isolates the RANKING, not the grouping.
    cos_b = torch.cos(bearings).view(1, beams)
    sin_b = torch.sin(bearings).view(1, beams)
    x = nearest * cos_b
    y = nearest * sin_b
    endpoint_gap = torch.sqrt(
        (x[:, 1:] - x[:, :-1]).square() + (y[:, 1:] - y[:, :-1]).square()
    )
    linked_to_previous = eligible[:, 1:] & eligible[:, :-1] & (endpoint_gap <= float(cluster_gap_m))
    boundary = torch.ones((batch, beams), dtype=torch.bool, device=device)
    boundary[:, 1:] = ~linked_to_previous
    cluster_id = boundary.long().cumsum(dim=1)

    # Closing speed toward each bearing. Bars are static, so the drone's own motion is the whole
    # closing rate; a moving target is never in this scan (it is a virtual point, no mesh).
    closing = body_vel_xy[:, 0:1] * cos_b + body_vel_xy[:, 1:2] * sin_b
    speed = body_vel_xy.norm(dim=1, keepdim=True)
    approaching = (closing > 1e-3) & (speed > float(min_speed))
    ttc = torch.where(
        approaching,
        nearest / closing.clamp(min=1e-3),
        float(idle_ttc_s) + nearest / max_range,
    )
    ttc = ttc.masked_fill(~eligible, float("inf"))

    remaining = eligible.clone()
    rows = torch.arange(batch, device=device)
    picked_ranges, picked_indices, picked_valid = [], [], []
    for _ in range(max_obstacles):
        work = ttc.masked_fill(~remaining, float("inf"))
        _, idx = work.min(dim=1)
        r = nearest[rows, idx]
        valid = remaining[rows, idx] & (r < max_range * 0.995)
        picked_ranges.append(r)
        picked_indices.append(idx)
        picked_valid.append(valid)
        # Consume the whole cluster so one wide surface cannot occupy several slots.
        picked_cluster = cluster_id[rows, idx]
        remaining &= ~(cluster_id.eq(picked_cluster.unsqueeze(1)) & valid.unsqueeze(1))

    ranges = torch.stack(picked_ranges, dim=1)
    indices = torch.stack(picked_indices, dim=1)
    valid = torch.stack(picked_valid, dim=1)

    # Preserve nearest-first slot semantics for the downstream flattened MLP.
    order = ranges.masked_fill(~valid, float("inf")).argsort(dim=1)
    return ranges.gather(1, order), indices.gather(1, order), valid.gather(1, order)


STRUCTURED_OBS_DIM = (
    ROBOT_HISTORY * ROBOT_DIM
    + TARGET_HISTORY * TARGET_DIM
    + OBSTACLE_HISTORY * MAX_OBSTACLES * OBSTACLE_DIM
    + STATIC_DIM
    + CORRIDOR_OBS_DIM
)


def _quat_rotate_xyzw(q, v):
    qvec = q[:, :3]
    uv = torch.cross(qvec, v, dim=1)
    uuv = torch.cross(qvec, uv, dim=1)
    return v + 2.0 * (q[:, 3:4] * uv + uuv)


def _quat_rotate_inverse_xyzw(q, v):
    qi = torch.cat([-q[:, :3], q[:, 3:4]], dim=1)
    return _quat_rotate_xyzw(qi, v)


class AppearanceTargetSegmenter(nn.Module):
    """Tiny learnable RGB-D pixel classifier with a usable red-target bootstrap.

    A trained checkpoint can replace the bootstrap weights through ``load_state_dict``.  The
    default is deterministic and recognizes appearance only; it has no access to simulator class
    IDs.  Keeping this head tiny makes 256 parallel camera streams practical on an 8 GB GPU.
    """

    def __init__(self):
        super().__init__()
        self.classifier = nn.Conv2d(4, 1, kernel_size=1)
        with torch.no_grad():
            self.classifier.weight.zero_()
            self.classifier.weight[0, 0, 0, 0] = 3.0
            self.classifier.weight[0, 1, 0, 0] = -2.0
            self.classifier.weight[0, 2, 0, 0] = -2.0
            self.classifier.bias.fill_(-0.9)

    def forward(self, rgb, depth, max_depth):
        depth_channel = (depth / max_depth).clamp(0.0, 1.0).unsqueeze(1)
        return torch.sigmoid(self.classifier(torch.cat([rgb, depth_channel], dim=1))).squeeze(1)


class BatchedConstantVelocityTracker:
    """GPU-vectorized 3-D constant-velocity Kalman filter."""

    def __init__(self, num_envs, device, dt, memory_s):
        self.num_envs = int(num_envs)
        self.device = device
        self.dt = float(dt)
        self.memory_s = float(memory_s)
        self.state = torch.zeros(self.num_envs, 6, device=device)
        self.cov = torch.eye(6, device=device).unsqueeze(0).repeat(self.num_envs, 1, 1)
        self.active = torch.zeros(self.num_envs, dtype=torch.bool, device=device)
        self.age = torch.full((self.num_envs,), self.memory_s, device=device)

        self.F = torch.eye(6, device=device)
        self.F[0, 3] = self.dt
        self.F[1, 4] = self.dt
        self.F[2, 5] = self.dt
        self.H = torch.zeros(3, 6, device=device)
        self.H[:, :3] = torch.eye(3, device=device)
        self.I = torch.eye(6, device=device)
        q_pos = 0.025 * self.dt * self.dt
        q_vel = 0.25 * self.dt
        self.Q = torch.diag(torch.tensor([q_pos] * 3 + [q_vel] * 3, device=device))

    def reset_idx(self, env_ids):
        self.state[env_ids] = 0.0
        self.cov[env_ids] = torch.eye(6, device=self.device)
        self.active[env_ids] = False
        self.age[env_ids] = self.memory_s

    def step(self, measurement_world, visible, measurement_var):
        active = self.active
        if bool(active.any()):
            x = self.state[active]
            p = self.cov[active]
            self.state[active] = torch.matmul(x, self.F.t())
            self.cov[active] = self.F.unsqueeze(0).matmul(p).matmul(self.F.t()) + self.Q
            self.age[active] += self.dt

        new = visible & ~self.active
        if bool(new.any()):
            self.state[new, :3] = measurement_world[new]
            self.state[new, 3:] = 0.0
            p0 = torch.zeros(int(new.sum()), 6, 6, device=self.device)
            p0[:, :3, :3] = torch.diag_embed(measurement_var[new])
            p0[:, 3:, 3:] = torch.eye(3, device=self.device).unsqueeze(0) * 1.0
            self.cov[new] = p0
            self.active[new] = True
            self.age[new] = 0.0

        self.correct(measurement_world, visible & ~new, measurement_var)

        expired = self.active & (self.age > self.memory_s)
        self.active[expired] = False
        return self.state, self.cov, self.active, self.age

    def correct(self, measurement_world, visible, measurement_var):
        """Apply a second sensor correction without advancing time again."""
        update = visible & self.active
        if bool(update.any()):
            x = self.state[update]
            p = self.cov[update]
            z = measurement_world[update]
            innovation = z - x[:, :3]
            r = torch.diag_embed(measurement_var[update])
            s = p[:, :3, :3] + r
            k = p[:, :, :3].matmul(torch.inverse(s))
            self.state[update] = x + k.matmul(innovation.unsqueeze(2)).squeeze(2)
            ikh = self.I.unsqueeze(0) - k.matmul(self.H.unsqueeze(0))
            self.cov[update] = ikh.matmul(p).matmul(ikh.transpose(1, 2)) + k.matmul(r).matmul(
                k.transpose(1, 2)
            )
            self.age[update] = 0.0


class NavRLPerceptionModule:
    """RGB-D/LiDAR fusion, tracking, uncertainty, and 2-second history encoding."""

    def __init__(self, num_envs, device, cfg, step_dt, camera_cfg):
        self.num_envs = int(num_envs)
        self.device = device
        self.cfg = cfg
        self.step_dt = float(step_dt)
        self.max_camera_range = float(camera_cfg.detector_max_range)
        self.lidar_max_range = float(getattr(cfg, "lidar_max_range", 4.0))
        # Far plane of the obstacle depth camera: columns at this value saw NOTHING and must not
        # min-fuse into a longer-range LiDAR scan (see camera_no_return_to_lidar_range).
        self.camera_obstacle_max_range = float(
            getattr(camera_cfg, "camera_obstacle_max_range", self.lidar_max_range)
        )
        self.hfov = math.radians(float(camera_cfg.detector_hfov_deg))
        self.vfov = math.radians(float(camera_cfg.detector_vfov_deg))
        self.width = int(camera_cfg.camera_width)
        self.height = int(camera_cfg.camera_height)
        self.fx = self.width / (2.0 * math.tan(self.hfov * 0.5))
        self.fy = self.height / (2.0 * math.tan(self.vfov * 0.5))
        self.cx = (self.width - 1) * 0.5
        self.cy = (self.height - 1) * 0.5
        self.camera_offset = torch.tensor(
            camera_cfg.camera_translation, dtype=torch.float32, device=device
        ).view(1, 3)
        self.target_radius = float(camera_cfg.camera_target_radius)
        self.min_pixels = int(getattr(cfg, "min_target_pixels", 2))
        self.pixel_threshold = float(getattr(cfg, "pixel_threshold", 0.55))
        self.dropout_prob = float(getattr(cfg, "detection_dropout_prob", 0.0))
        self.detection_latency_s = float(getattr(cfg, "detection_latency_s", 0.0))
        self.range_error_m = float(getattr(cfg, "range_error_m", 0.0))
        # Latency compensation (P0/P1, WORKLOG 2026-08-05). Both default off; both are no-ops
        # when detection_latency_s is 0, so a clean run is byte-identical with them set.
        self.latency_compensate = bool(getattr(cfg, "latency_compensate", False))
        self.latency_lidar_backup = bool(getattr(cfg, "latency_lidar_backup", False))
        # P2: how the obstacle map is edited while the detection is stale.
        self.latency_obstacle_fix = (
            str(getattr(cfg, "latency_obstacle_fix", "off") or "off").strip().lower()
        )
        if self.latency_obstacle_fix not in ("off", "predict", "skip"):
            raise ValueError(
                "latency_obstacle_fix must be off|predict|skip, got "
                f"{self.latency_obstacle_fix!r}"
            )
        # P3: lift the delayed measurement to world with the pose it was TAKEN at, not the
        # current one. Set alongside the ring-buffer read so the index math happens exactly once.
        self.latency_ego_motion_fix = bool(getattr(cfg, "latency_ego_motion_fix", False))
        self._latency_delayed_pose = None
        self.rgb_noise_std = float(getattr(cfg, "rgb_noise_std", 0.015))
        self.depth_noise_std = float(getattr(cfg, "depth_noise_std", 0.02))
        self.history_stride = max(1, int(round(float(cfg.history_interval_s) / self.step_dt)))
        self.step_count = 0
        self._latency_steps = max(0, int(round(self.detection_latency_s / self.step_dt)))
        self._latency_slots = self._latency_steps + 1
        self._latency_step = torch.zeros(self.num_envs, dtype=torch.long, device=device)
        self._latency_meas_vehicle = torch.zeros(
            self.num_envs, self._latency_slots, 3, dtype=torch.float32, device=device
        )
        self._latency_surface_range = torch.zeros(
            self.num_envs, self._latency_slots, dtype=torch.float32, device=device
        )
        self._latency_bearing = torch.zeros(
            self.num_envs, self._latency_slots, dtype=torch.float32, device=device
        )
        self._latency_visible = torch.zeros(
            self.num_envs, self._latency_slots, dtype=torch.bool, device=device
        )
        self._latency_confidence = torch.zeros(
            self.num_envs, self._latency_slots, dtype=torch.float32, device=device
        )
        self._latency_mask = torch.zeros(
            self.num_envs,
            self._latency_slots,
            self.height,
            self.width,
            dtype=torch.bool,
            device=device,
        )
        # Pose of the observer at the moment each buffered detection was taken (P3). The quat
        # buffer holds identity rather than zeros so an unread slot still rotates validly.
        self._latency_drone_pos = torch.zeros(
            self.num_envs, self._latency_slots, 3, dtype=torch.float32, device=device
        )
        self._latency_drone_quat = torch.zeros(
            self.num_envs, self._latency_slots, 4, dtype=torch.float32, device=device
        )
        self._latency_drone_quat[..., 3] = 1.0
        self._env_ids = torch.arange(self.num_envs, device=device)

        self.segmenter = AppearanceTargetSegmenter().to(device).eval()
        checkpoint = str(getattr(cfg, "detector_checkpoint", "") or "").strip()
        if checkpoint:
            state = torch.load(checkpoint, map_location=device)
            self.segmenter.load_state_dict(state.get("model", state), strict=True)

        self.tracker = BatchedConstantVelocityTracker(
            num_envs, device, step_dt, float(camera_cfg.tracker_memory_s)
        )
        self.robot_history = torch.zeros(
            self.num_envs, ROBOT_HISTORY, ROBOT_DIM, device=device
        )
        self.target_history = torch.zeros(
            self.num_envs, TARGET_HISTORY, TARGET_DIM, device=device
        )
        self.obstacle_history = torch.zeros(
            self.num_envs, OBSTACLE_HISTORY, MAX_OBSTACLES, OBSTACLE_DIM, device=device
        )
        self.last_visible = torch.zeros(self.num_envs, dtype=torch.bool, device=device)
        self.last_confidence = torch.zeros(self.num_envs, device=device)
        # Exact target-return association used to build the actor's obstacle representation.
        # Safety layers may reuse this sensor-derived mask; they must never inspect the simulator
        # semantic-ID buffer independently of the perception front-end.
        self.last_target_like = torch.zeros(
            self.num_envs, VBEAMS, HBEAMS, dtype=torch.bool, device=device
        )

        self._u = torch.arange(self.width, device=device, dtype=torch.float32).view(1, 1, -1)
        self._v = torch.arange(self.height, device=device, dtype=torch.float32).view(1, -1, 1)
        # Vehicle-frame bearing of each depth COLUMN -- the exact inverse of the lidar-angle ->
        # column map used below (camera_u = (hfov/2 - angle)/hfov * (width-1)). Only P2's
        # "predict" mode reads it, to rebuild the target pixel mask from the predicted bearing.
        self._pixel_angles = (
            self.hfov * 0.5
            - torch.arange(self.width, device=device, dtype=torch.float32)
            / max(self.width - 1, 1)
            * self.hfov
        )
        # Bearings of the horizontal beams. The span stops 360/HBEAMS short of a full turn so the
        # first and last ray are not the same direction (matching navrl_lidar_config's fov choice):
        # HBEAMS=36 -> [-170, 180] deg at 10 deg spacing, HBEAMS=72 -> [-175, 180] at 5 deg.
        self._bin_deg = 360.0 / HBEAMS
        # Physical bin bearings (DECREASING with index, matching the warp ray generator).
        # See lidar_bin_bearings() for the mirror-bug history and the adjudication probe.
        self._lidar_angles = lidar_bin_bearings(device)
        # Suppression window converted from degrees to bins (at least 1 so a token always blanks
        # its own bin, never selecting the same bearing twice).
        self._suppress_bins = max(1, int(round(OBSTACLE_SUPPRESS_DEG / self._bin_deg)))
        # Bearings eligible to become obstacle tokens (see OBSTACLE_FOV_DEG). None = all of them.
        if OBSTACLE_FOV_DEG < 359.9:
            self._token_bearing_mask = (
                self._lidar_angles.abs() <= math.radians(OBSTACLE_FOV_DEG * 0.5)
            )
        else:
            self._token_bearing_mask = None

    def reset_idx(self, env_ids):
        self.tracker.reset_idx(env_ids)
        self.robot_history[env_ids] = 0.0
        self.target_history[env_ids] = 0.0
        self.obstacle_history[env_ids] = 0.0
        self.last_visible[env_ids] = False
        self.last_confidence[env_ids] = 0.0
        self.last_target_like[env_ids] = False
        self._latency_step[env_ids] = 0
        self._latency_meas_vehicle[env_ids] = 0.0
        self._latency_surface_range[env_ids] = 0.0
        self._latency_bearing[env_ids] = 0.0
        self._latency_visible[env_ids] = False
        self._latency_confidence[env_ids] = 0.0
        self._latency_mask[env_ids] = False
        self._latency_drone_pos[env_ids] = 0.0
        self._latency_drone_quat[env_ids] = 0.0
        self._latency_drone_quat[env_ids, :, 3] = 1.0

    def _apply_detection_latency(
        self,
        measurement_vehicle,
        surface_range,
        bearing,
        visible,
        confidence,
        mask,
        drone_pos_w=None,
        vehicle_quat=None,
    ):
        self._latency_delayed_pose = None
        if self._latency_steps <= 0:
            return measurement_vehicle, surface_range, bearing, visible, confidence, mask

        write_idx = self._latency_step % self._latency_slots
        self._latency_meas_vehicle[self._env_ids, write_idx] = measurement_vehicle
        self._latency_surface_range[self._env_ids, write_idx] = surface_range
        self._latency_bearing[self._env_ids, write_idx] = bearing
        self._latency_visible[self._env_ids, write_idx] = visible
        self._latency_confidence[self._env_ids, write_idx] = confidence
        self._latency_mask[self._env_ids, write_idx] = mask
        if drone_pos_w is not None and vehicle_quat is not None:
            self._latency_drone_pos[self._env_ids, write_idx] = drone_pos_w
            self._latency_drone_quat[self._env_ids, write_idx] = vehicle_quat
        self._latency_step += 1
        ready = self._latency_step > self._latency_steps
        read_idx = (self._latency_step - self._latency_steps - 1) % self._latency_slots
        delayed_visible = self._latency_visible[self._env_ids, read_idx] & ready
        delayed_mask = self._latency_mask[self._env_ids, read_idx] & delayed_visible.view(
            -1, 1, 1
        )
        delayed_confidence = self._latency_confidence[self._env_ids, read_idx]
        # Published from the SAME read index as the measurement, so the pose and the detection
        # can never come from different steps. observe() consumes it only when P3 is enabled.
        if drone_pos_w is not None and vehicle_quat is not None:
            self._latency_delayed_pose = (
                self._latency_drone_pos[self._env_ids, read_idx],
                self._latency_drone_quat[self._env_ids, read_idx],
            )
        return (
            self._latency_meas_vehicle[self._env_ids, read_idx],
            self._latency_surface_range[self._env_ids, read_idx],
            self._latency_bearing[self._env_ids, read_idx],
            delayed_visible,
            torch.where(
                delayed_visible,
                delayed_confidence,
                torch.zeros_like(confidence),
            ),
            delayed_mask,
        )

    def _detect_rgbd(self, rgb, depth, training, drone_pos_w=None, vehicle_quat=None):
        if training and self.rgb_noise_std > 0.0:
            rgb = (rgb + torch.randn_like(rgb) * self.rgb_noise_std).clamp(0.0, 1.0)
        if training and self.depth_noise_std > 0.0:
            depth = (depth + torch.randn_like(depth) * self.depth_noise_std).clamp(
                0.0, self.max_camera_range
            )
        with torch.no_grad():
            score = self.segmenter(rgb, depth, self.max_camera_range)
        mask = (score >= self.pixel_threshold) & (depth < self.max_camera_range)
        count = mask.sum(dim=(1, 2))
        visible = count >= self.min_pixels
        if training and self.dropout_prob > 0.0:
            visible &= torch.rand(self.num_envs, device=self.device) >= self.dropout_prob
        mask &= visible.view(-1, 1, 1)
        denom = count.clamp(min=1).float()
        mf = mask.float()
        u = (mf * self._u).sum(dim=(1, 2)) / denom
        v = (mf * self._v).sum(dim=(1, 2)) / denom
        surface_range = (depth * mf).sum(dim=(1, 2)) / denom
        if training and self.range_error_m != 0.0:
            surface_range = (surface_range + self.range_error_m).clamp(
                0.0, self.max_camera_range
            )
        confidence = (score * mf).sum(dim=(1, 2)) / denom
        confidence *= (count.float() / max(1.0, float(self.min_pixels * 4))).clamp(max=1.0)
        confidence = torch.where(visible, confidence, torch.zeros_like(confidence))

        ray = torch.stack(
            [torch.ones_like(u), -(u - self.cx) / self.fx, -(v - self.cy) / self.fy], dim=1
        )
        ray = ray / ray.norm(dim=1, keepdim=True).clamp(min=1e-6)
        center_range = surface_range + self.target_radius
        measurement_vehicle = self.camera_offset + ray * center_range.unsqueeze(1)
        bearing = torch.atan2(measurement_vehicle[:, 1], measurement_vehicle[:, 0])
        return self._apply_detection_latency(
            measurement_vehicle,
            surface_range,
            bearing,
            visible,
            confidence,
            mask,
            drone_pos_w=drone_pos_w,
            vehicle_quat=vehicle_quat,
        )

    def _latency_corrected_map_inputs(
        self,
        depth,
        target_pixels,
        target_surface_range,
        target_bearing,
        visible,
        drone_pos_w,
        vehicle_quat,
    ):
        """P2: fix WHERE the obstacle map gets the target carved out of it under latency.

        `_fuse_static_and_extract_obstacles` deletes sensor returns that look like the target --
        LiDAR bins inside the target_like window, and depth pixels inside `target_pixels` -- so
        that the target is never proposed as an obstacle. Every one of those inputs is DELAYED,
        so under latency the deletion happens at the target's OLD bearing/range, where a real bar
        may stand: the map loses that bar and the drone flies into it. This is the channel that
        tripled bar contacts (337 -> 931) in the 0.1 s arm even though LiDAR itself is undelayed,
        and neither P0 (policy-facing output only) nor P1 (association gate) touched it.

        "predict" rebuilds the deletion window from the tracker's forward-predicted target pose,
        i.e. deletes at where the target is NOW. "skip" stops deleting while the detection is
        stale: the target survives in the map as a phantom obstacle (costing capture), but no
        real bar is ever erased (bounding crash). Returns the inputs unchanged when off or when
        there is no latency, so clean runs are bit-identical.
        """
        if self.latency_obstacle_fix == "off" or self._latency_steps <= 0:
            return target_pixels, target_surface_range, target_bearing, visible
        if self.latency_obstacle_fix == "skip":
            # visible=False disables both the target_like carve-out and (via the empty pixel
            # mask) the depth blanking, without touching the fused scan geometry itself.
            return (
                torch.zeros_like(target_pixels),
                target_surface_range,
                target_bearing,
                torch.zeros_like(visible),
            )

        # "predict": same window, relocated to the tracker's estimate of the target NOW. This is
        # the P0 extrapolation reused on the map path -- deliberately the same expression, so the
        # two compensations cannot disagree about where the target is.
        state = self.tracker.state
        pred_pos_w = state[:, :3] + state[:, 3:] * self.detection_latency_s
        rel = _quat_rotate_inverse_xyzw(vehicle_quat, pred_pos_w - drone_pos_w)
        pred_bearing = torch.atan2(rel[:, 1], rel[:, 0])
        # surface_range is measured from the CAMERA to the near face, matching _detect_rgbd's
        # center_range = surface_range + target_radius decomposition.
        pred_surface = (rel - self.camera_offset).norm(dim=1) - self.target_radius
        pred_surface = pred_surface.clamp(min=0.0)
        # Only relocate where the tracker actually has a target; elsewhere carve nothing rather
        # than carve at an arbitrary place.
        pred_visible = visible & self.tracker.active
        # Rebuild the depth blanking mask at the predicted pose with the same agreement window
        # the LiDAR carve-out uses (the delayed segmenter mask points at the wrong place).
        pixel_delta = torch.atan2(
            torch.sin(self._pixel_angles.view(1, 1, -1) - pred_bearing.view(-1, 1, 1)),
            torch.cos(self._pixel_angles.view(1, 1, -1) - pred_bearing.view(-1, 1, 1)),
        ).abs()
        pred_pixels = (
            pred_visible.view(-1, 1, 1)
            & (pixel_delta < TARGET_LIKE_ANGLE_RAD)
            & ((depth - pred_surface.view(-1, 1, 1)).abs() < TARGET_LIKE_RANGE_TOL_M)
        )
        return pred_pixels, pred_surface, pred_bearing, pred_visible

    def _fuse_static_and_extract_obstacles(
        self,
        lidar_m,
        raw_depth,
        target_pixels,
        target_surface_range,
        target_bearing,
        visible,
        drone_vel_w=None,
        vehicle_quat=None,
    ):
        scan = lidar_m.view(self.num_envs, VBEAMS, HBEAMS).clone()

        # Camera-associated LiDAR returns are target evidence, not static obstacles. No semantic
        # ID is consulted: association uses only bearing and metric range agreement.
        angle_delta = torch.atan2(
            torch.sin(self._lidar_angles.view(1, 1, HBEAMS) - target_bearing.view(-1, 1, 1)),
            torch.cos(self._lidar_angles.view(1, 1, HBEAMS) - target_bearing.view(-1, 1, 1)),
        ).abs()
        target_like = (
            visible.view(-1, 1, 1)
            & (angle_delta < TARGET_LIKE_ANGLE_RAD)
            & ((scan - target_surface_range.view(-1, 1, 1)).abs() < TARGET_LIKE_RANGE_TOL_M)
        )
        self.last_target_like[:] = target_like
        scan = torch.where(target_like, torch.full_like(scan, self.lidar_max_range), scan)

        # Fuse forward RGB-D geometry into the 360-degree LiDAR distance representation.
        depth_no_target = torch.where(
            target_pixels, torch.full_like(raw_depth, self.lidar_max_range), raw_depth
        )
        camera_col_min = camera_no_return_to_lidar_range(
            depth_no_target.amin(dim=1), self.camera_obstacle_max_range, self.lidar_max_range
        ).clamp(0.0, self.lidar_max_range)
        inside = self._lidar_angles.abs() <= self.hfov * 0.5
        camera_u = ((self.hfov * 0.5 - self._lidar_angles) / self.hfov * (self.width - 1)).round()
        camera_u = camera_u.long().clamp(0, self.width - 1)
        camera_ranges = camera_col_min[:, camera_u]
        fused_camera = camera_ranges.unsqueeze(1).expand(-1, VBEAMS, -1)
        scan[:, :, inside] = torch.minimum(scan[:, :, inside], fused_camera[:, :, inside])
        static_state = (scan / self.lidar_max_range).clamp(0.0, 1.0)

        # Nearest angularly separated obstacle proposals. Bars are static, hence velocity=0;
        # position and covariance come from range/angle geometry rather than simulator positions.
        nearest = scan.amin(dim=1)
        # Diagnostics only (no grad, no copy cost on the hot path): the per-bearing obstacle range
        # actually seen this step. The bar_contact probe uses it to measure how crowded the scene was
        # at the moment of a collision -- i.e. whether MAX_OBSTACLES truncated away the bar that hit.
        self.last_scan_nearest = nearest
        tokens = torch.zeros(
            self.num_envs, MAX_OBSTACLES, OBSTACLE_DIM, device=self.device
        )
        rows = torch.arange(self.num_envs, device=self.device)
        if OBSTACLE_SELECTOR == "ttc_sector":
            # Body-frame planar velocity drives the closing-rate ranking. Same rotation the robot
            # observation uses below, so the two views of the drone's motion cannot disagree.
            if vehicle_quat is None or drone_vel_w is None:
                raise ValueError(
                    "ttc_sector requires vehicle_quat and drone_vel_w for closing-rate ranking"
                )
            body_vel = _quat_rotate_inverse_xyzw(vehicle_quat, drone_vel_w)
            selected_ranges, selected_indices, selected_valid = select_ttc_obstacles(
                nearest,
                self._lidar_angles,
                body_vel[:, :2],
                max_range=self.lidar_max_range,
                max_obstacles=MAX_OBSTACLES,
                cluster_gap_m=OBSTACLE_CLUSTER_GAP_M,
                idle_ttc_s=OBSTACLE_TTC_IDLE_S,
                min_speed=OBSTACLE_TTC_MIN_SPEED,
            )
        elif OBSTACLE_SELECTOR == "cluster_sector":
            selected_ranges, selected_indices, selected_valid = (
                select_cluster_sector_obstacles(
                    nearest,
                    self._lidar_angles,
                    max_range=self.lidar_max_range,
                    max_obstacles=MAX_OBSTACLES,
                    token_fov_deg=OBSTACLE_FOV_DEG,
                    cluster_gap_m=OBSTACLE_CLUSTER_GAP_M,
                    num_sectors=OBSTACLE_SECTORS,
                )
            )
        else:
            work = nearest.clone()
            if self._token_bearing_mask is not None:
                # Blank bearings outside the token sector so they can never win a slot. Applied to
                # the SELECTION copy only -- `static_state` still encodes the full 360-degree scan.
                work = work.masked_fill(
                    ~self._token_bearing_mask.view(1, -1), self.lidar_max_range
                )
            selected_ranges = torch.full(
                (self.num_envs, MAX_OBSTACLES), self.lidar_max_range, device=self.device
            )
            selected_indices = torch.zeros(
                (self.num_envs, MAX_OBSTACLES), dtype=torch.long, device=self.device
            )
            selected_valid = torch.zeros(
                (self.num_envs, MAX_OBSTACLES), dtype=torch.bool, device=self.device
            )
            for slot in range(MAX_OBSTACLES):
                r, idx = work.min(dim=1)
                valid = r < self.lidar_max_range * 0.995
                selected_ranges[:, slot] = r
                selected_indices[:, slot] = idx
                selected_valid[:, slot] = valid
                for off in range(-self._suppress_bins, self._suppress_bins + 1):
                    work[rows, (idx + off) % HBEAMS] = self.lidar_max_range

        for slot in range(MAX_OBSTACLES):
            r = selected_ranges[:, slot]
            idx = selected_indices[:, slot]
            valid = selected_valid[:, slot]
            a = self._lidar_angles[idx]
            pos = torch.stack([r * torch.cos(a), r * torch.sin(a), torch.zeros_like(r)], dim=1)
            radial_sigma = 0.04 + 0.02 * r
            lateral_sigma = 0.05 + r * math.tan(math.radians(5.0))
            cov = torch.stack(
                [radial_sigma.square(), lateral_sigma.square(), torch.full_like(r, 0.12)], dim=1
            )
            feat = torch.cat(
                [
                    pos / self.lidar_max_range,
                    torch.zeros_like(pos),
                    torch.full_like(r.unsqueeze(1), 0.30 / self.lidar_max_range),
                    (1.0 - r / self.lidar_max_range).clamp(0.0, 1.0).unsqueeze(1),
                    cov / (self.lidar_max_range * self.lidar_max_range),
                    torch.ones_like(r.unsqueeze(1)),
                ],
                dim=1,
            )
            tokens[:, slot] = feat * valid.unsqueeze(1)
        return static_state.reshape(self.num_envs, -1), tokens

    def _associate_lidar_target(self, lidar_m, drone_pos_w, vehicle_quat, camera_visible):
        """Associate a raw LiDAR return with the predicted camera-initialized target track."""
        rel = _quat_rotate_inverse_xyzw(
            vehicle_quat, self.tracker.state[:, :3] - drone_pos_w
        )
        predicted_center_range = rel.norm(dim=1).clamp(min=1e-6)
        predicted_surface_range = (predicted_center_range - self.target_radius).clamp(min=0.0)
        bearing = torch.atan2(rel[:, 1], rel[:, 0])
        angular_error = torch.atan2(
            torch.sin(self._lidar_angles.view(1, -1) - bearing.unsqueeze(1)),
            torch.cos(self._lidar_angles.view(1, -1) - bearing.unsqueeze(1)),
        ).abs()
        h_idx = angular_error.argmin(dim=1)
        raw_scan = lidar_m.view(self.num_envs, VBEAMS, HBEAMS)
        rows = torch.arange(self.num_envs, device=self.device)
        ray_ranges = raw_scan[rows, :, h_idx]
        measured_surface, _ = ray_ranges.min(dim=1)
        pos_sigma = torch.diagonal(self.tracker.cov[:, :3, :3], dim1=1, dim2=2).sum(
            dim=1
        ).sqrt()
        gate = (0.35 + 2.0 * pos_sigma).clamp(max=1.0)
        valid = (
            self.tracker.active
            & ~camera_visible
            & (predicted_surface_range < self.lidar_max_range)
            & (measured_surface < self.lidar_max_range * 0.995)
            & ((measured_surface - predicted_surface_range).abs() < gate)
        )
        center_range = measured_surface + self.target_radius
        measured_vehicle = torch.stack(
            [
                center_range * torch.cos(bearing),
                center_range * torch.sin(bearing),
                rel[:, 2],
            ],
            dim=1,
        )
        measurement_world = drone_pos_w + _quat_rotate_xyzw(vehicle_quat, measured_vehicle)
        lidar_var = torch.tensor(
            [0.08**2, 0.15**2, 0.20**2], device=self.device
        ).view(1, 3).expand(self.num_envs, -1)
        self.tracker.correct(measurement_world, valid, lidar_var)
        confidence = torch.exp(
            -(measured_surface - predicted_surface_range).abs() / gate.clamp(min=1e-3)
        ) * valid.float()
        return valid, confidence, measured_surface, bearing

    def _target_features(
        self, drone_pos_w, drone_vel_w, vehicle_quat, camera_confidence, lidar_confidence
    ):
        state, cov, active, age = (
            self.tracker.state,
            self.tracker.cov,
            self.tracker.active,
            self.tracker.age,
        )
        pos_world = state[:, :3]
        # P0 latency compensation: the KF is corrected with measurements that are
        # detection_latency_s OLD, so its position estimate trails the true target by ~v*tau.
        # Extrapolate the POLICY-FACING position to "now" with the filter's own velocity.
        # Output-side only: KF internals, covariance, age, LiDAR association, and diagnostics
        # are untouched, and the observation stays the same width.
        if self.latency_compensate and self._latency_steps > 0:
            pos_world = pos_world + state[:, 3:] * self.detection_latency_s
        rel_pos = _quat_rotate_inverse_xyzw(vehicle_quat, pos_world - drone_pos_w)
        rel_vel = _quat_rotate_inverse_xyzw(vehicle_quat, state[:, 3:] - drone_vel_w)
        diag = torch.diagonal(cov, dim1=1, dim2=2)
        pos_var = diag[:, :3] / (self.max_camera_range * self.max_camera_range)
        vel_var = diag[:, 3:] / 4.0
        features = torch.cat(
            [
                rel_pos / self.max_camera_range,
                rel_vel / 2.0,
                pos_var.clamp(0.0, 1.0),
                vel_var.clamp(0.0, 1.0),
                camera_confidence.unsqueeze(1),
                lidar_confidence.unsqueeze(1),
                (age / self.tracker.memory_s).clamp(0.0, 1.0).unsqueeze(1),
                torch.full_like(age.unsqueeze(1), self.target_radius / self.lidar_max_range),
            ],
            dim=1,
        )
        return features * active.unsqueeze(1)

    def _update_histories(self, robot_now, target_now, obstacles_now):
        if self.step_count % self.history_stride == 0:
            self.robot_history = torch.roll(self.robot_history, shifts=-1, dims=1)
            self.target_history = torch.roll(self.target_history, shifts=-1, dims=1)
            self.obstacle_history = torch.roll(self.obstacle_history, shifts=-1, dims=1)
        self.robot_history[:, -1] = robot_now
        self.target_history[:, -1] = target_now
        self.obstacle_history[:, -1] = obstacles_now
        self.step_count += 1

    def observe(
        self,
        rgb,
        depth,
        lidar_m,
        drone_pos_w,
        drone_vel_w,
        vehicle_quat,
        yaw_rate,
        previous_action,
        max_velocity,
        flight_altitude,
        training=True,
    ):
        """Return actor-safe structured observation and sensor diagnostics."""
        meas_vehicle, surface_range, bearing, visible, confidence, pixels = self._detect_rgbd(
            rgb, depth, training, drone_pos_w=drone_pos_w, vehicle_quat=vehicle_quat
        )
        # P3 ego-motion compensation: a DELAYED detection was taken in the vehicle frame of
        # t-tau, so lifting it to world with the pose at t injects the drone's own motion over
        # tau into every KF correction -- 0.23 m of translation at the measured 2.33 m/s mean
        # speed, plus yaw applied straight to the bearing. Both dominate the <=0.15 m target lag
        # that P0 removes, which is why P0 alone changed nothing (WORKLOG 2026-08-06). Lifting
        # with the buffered capture-time pose leaves a measurement that is accurate but tau old,
        # i.e. exactly the residual P0 was written for.
        meas_pos_w, meas_quat = drone_pos_w, vehicle_quat
        if self.latency_ego_motion_fix and self._latency_delayed_pose is not None:
            meas_pos_w, meas_quat = self._latency_delayed_pose
        meas_world = meas_pos_w + _quat_rotate_xyzw(meas_quat, meas_vehicle)
        pixel_count = pixels.sum(dim=(1, 2)).float().clamp(min=1.0)
        sigma_r = 0.04 + 0.012 * surface_range + 0.15 / pixel_count.sqrt()
        sigma_lat = 0.03 + surface_range / max(self.fx, 1.0)
        measurement_var = torch.stack(
            [sigma_r.square(), sigma_lat.square(), sigma_lat.square()], dim=1
        )
        self.tracker.step(meas_world, visible, measurement_var)
        # P1 latency compensation: `visible` here is the DELAYED camera flag (the ring buffer in
        # _apply_detection_latency already ran inside _detect_rgbd). The historical
        # `~camera_visible` gate inside _associate_lidar_target then blocks the fresh-LiDAR
        # correction exactly when the stale camera claims sight -- the structural reason latency
        # was catastrophic (-42.7 pp) while range error was benign. With the backup enabled a
        # delayed detection no longer vetoes the LiDAR path; sim LiDAR has no latency.
        lidar_camera_gate = visible
        if self.latency_lidar_backup and self._latency_steps > 0:
            lidar_camera_gate = torch.zeros_like(visible)
        lidar_visible, lidar_confidence, lidar_surface, lidar_bearing = (
            self._associate_lidar_target(lidar_m, drone_pos_w, vehicle_quat, lidar_camera_gate)
        )
        fused_visible = visible | lidar_visible
        fused_surface = torch.where(lidar_visible, lidar_surface, surface_range)
        fused_bearing = torch.where(lidar_visible, lidar_bearing, bearing)

        map_pixels, map_surface, map_bearing, map_visible = self._latency_corrected_map_inputs(
            depth, pixels, fused_surface, fused_bearing, fused_visible, drone_pos_w, vehicle_quat
        )
        static_state, obstacles_now = self._fuse_static_and_extract_obstacles(
            lidar_m,
            depth,
            map_pixels,
            map_surface,
            map_bearing,
            map_visible,
            drone_vel_w=drone_vel_w,
            vehicle_quat=vehicle_quat,
        )
        target_now = self._target_features(
            drone_pos_w, drone_vel_w, vehicle_quat, confidence, lidar_confidence
        )
        robot_now = torch.cat(
            [
                _quat_rotate_inverse_xyzw(vehicle_quat, drone_vel_w) / max_velocity,
                yaw_rate.unsqueeze(1),
                previous_action,
                (drone_pos_w[:, 2:3] / max(flight_altitude * 3.0, 1e-6)),
                torch.ones(self.num_envs, 1, device=self.device),
            ],
            dim=1,
        )
        self._update_histories(robot_now, target_now, obstacles_now)
        self.last_visible[:] = fused_visible
        self.last_confidence[:] = torch.maximum(confidence, lidar_confidence)

        obs_parts = [
            static_state,
            self.obstacle_history.reshape(self.num_envs, -1),
            self.robot_history.reshape(self.num_envs, -1),
            self.target_history.reshape(self.num_envs, -1),
        ]
        if CORRIDOR_TOKENS > 0:
            # Free-gap affordance from the SAME fused profile the obstacle tokens use
            # (last_scan_nearest is set inside _fuse_static_and_extract_obstacles above).
            # Appended LAST so all pre-existing segment offsets stay checkpoint-compatible.
            corridor_tokens, _ = extract_corridor_tokens(
                self.last_scan_nearest,
                self._lidar_angles,
                max_range=self.lidar_max_range,
                num_corridors=CORRIDOR_TOKENS,
                fov_deg=OBSTACLE_FOV_DEG,
                horizon_m=CORRIDOR_HORIZON_M,
                min_width_m=CORRIDOR_MIN_WIDTH_M,
            )
            obs_parts.append(corridor_tokens.reshape(self.num_envs, -1))
        obs = torch.cat(obs_parts, dim=1)
        if obs.shape[1] != STRUCTURED_OBS_DIM:
            raise RuntimeError(
                "perception observation schema drift: got %d, expected %d"
                % (obs.shape[1], STRUCTURED_OBS_DIM)
            )
        diagnostics = {
            "visible": fused_visible,
            "camera_visible": visible,
            "lidar_visible": lidar_visible,
            "confidence": torch.maximum(confidence, lidar_confidence),
            "target_pixels": pixel_count,
            "track_age": self.tracker.age,
            "track_covariance": self.tracker.cov,
        }
        return obs, diagnostics
