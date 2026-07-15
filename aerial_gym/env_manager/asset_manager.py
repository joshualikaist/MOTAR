import math
from typing import Any
from aerial_gym.utils.math import *

from aerial_gym.utils.logging import CustomLogger

logger = CustomLogger("asset_manager")
logger.setLevel("DEBUG")


class AssetManager:
    def __init__(
        self,
        global_tensor_dict,
        num_keep_in_env,
        min_xy_spacing=0.0,
        placement_mode="grid",
        placement_attempts_before_relax=128,
        placement_relax_factor=0.8,
    ):
        # min_xy_spacing > 0 enforces a minimum ground-plane (XY) center-to-center distance
        # between the kept obstacles so they never overlap. Default 0.0 = off (unchanged behavior).
        self.min_xy_spacing = float(min_xy_spacing)
        self.placement_mode = str(placement_mode).strip().lower()
        self.placement_attempts_before_relax = max(1, int(placement_attempts_before_relax))
        self.placement_relax_factor = float(placement_relax_factor)
        if self.placement_relax_factor <= 0.0 or self.placement_relax_factor >= 1.0:
            self.placement_relax_factor = 0.8
        self.init_tensors(global_tensor_dict, num_keep_in_env)

    def init_tensors(self, global_tensor_dict, num_keep_in_env):
        self.env_asset_state_tensor = global_tensor_dict["env_asset_state_tensor"]
        self.asset_min_state_ratio = global_tensor_dict["asset_min_state_ratio"]
        self.asset_max_state_ratio = global_tensor_dict["asset_max_state_ratio"]
        self.env_bounds_min = (
            global_tensor_dict["env_bounds_min"]
            .unsqueeze(1)
            .expand(-1, self.env_asset_state_tensor.shape[1], -1)
        )
        self.env_bounds_max = (
            global_tensor_dict["env_bounds_max"]
            .unsqueeze(1)
            .expand(-1, self.env_asset_state_tensor.shape[1], -1)
        )
        self.num_keep_in_env = num_keep_in_env

    def prepare_for_sim(self):
        self.reset(self.num_keep_in_env)
        logger.warning(f"Number of obstacles to be kept in the environment: {self.num_keep_in_env}")

    def pre_physics_step(self, actions):
        pass

    def post_physics_step(self):
        pass

    def step(self, actions):
        pass
        # Implement this function if needed.
        # this functionality can do speciic things with the environment assets on stepping.
        # nothing really needs to be done for static environments.
        # if force needs to be applied, it should be done in the other classes and it's
        # better to leave this class to manipulate the state tensors.

    def reset(self, num_obstacles_per_env):
        self.reset_idx(torch.arange(self.env_asset_state_tensor.shape[0]), num_obstacles_per_env)

    def reset_idx(self, env_ids, num_obstacles_per_env=0):
        env_ids = self._env_ids_tensor(env_ids)
        num_obstacles_per_env = int(num_obstacles_per_env)
        if num_obstacles_per_env < self.num_keep_in_env:
            logger.info(
                "Number of obstacles required in the environment by the \
                  code is lesser than the minimum number of obstacles that the environment configuration specifies."
            )
            num_obstacles_per_env = self.num_keep_in_env

        sampled_asset_state_ratio = torch_rand_float_tensor(
            self.asset_min_state_ratio, self.asset_max_state_ratio
        )
        positions = torch_interpolate_ratio(
            min=self.env_bounds_min,
            max=self.env_bounds_max,
            ratio=sampled_asset_state_ratio[..., 0:3],
        )
        if self.min_xy_spacing > 0.0:
            positions = self._enforce_min_xy_spacing(positions, num_obstacles_per_env, env_ids)
        self.env_asset_state_tensor[env_ids, :, 0:3] = positions[env_ids, :, 0:3]
        self.env_asset_state_tensor[env_ids, :, 3:7] = quat_from_euler_xyz_tensor(
            sampled_asset_state_ratio[env_ids, :, 3:6]
        )
        # put those obstacles not needed in the environment outside
        self.env_asset_state_tensor[env_ids, num_obstacles_per_env:, 0:3] = -1000.0

    def _env_ids_tensor(self, env_ids):
        num_envs = self.env_asset_state_tensor.shape[0]
        device = self.env_asset_state_tensor.device
        if env_ids is None:
            return torch.arange(num_envs, device=device, dtype=torch.long)
        if isinstance(env_ids, torch.Tensor):
            return env_ids.to(device=device, dtype=torch.long).view(-1)
        return torch.as_tensor(env_ids, device=device, dtype=torch.long).view(-1)

    def _enforce_min_xy_spacing(self, positions, num_used, env_ids):
        if self.placement_mode in ("random", "rejection", "navrl_random"):
            return self._random_rejection_xy_spacing(positions, num_used, env_ids)
        return self._grid_xy_spacing(positions, num_used, env_ids)

    def _placement_band(self):
        span_x = self.env_bounds_max[:, 0, 0] - self.env_bounds_min[:, 0, 0]
        span_y = self.env_bounds_max[:, 0, 1] - self.env_bounds_min[:, 0, 1]
        bx0 = self.env_bounds_min[:, 0, 0] + self.asset_min_state_ratio[:, 0, 0] * span_x
        bx1 = self.env_bounds_min[:, 0, 0] + self.asset_max_state_ratio[:, 0, 0] * span_x
        by0 = self.env_bounds_min[:, 0, 1] + self.asset_min_state_ratio[:, 0, 1] * span_y
        by1 = self.env_bounds_min[:, 0, 1] + self.asset_max_state_ratio[:, 0, 1] * span_y
        return bx0, bx1, by0, by1

    def _grid_xy_spacing(self, positions, num_used, env_ids):
        """Place the kept obstacles on a per-env jittered grid so that no two are closer than
        min_xy_spacing (center-to-center, XY).

        A grid guarantees the spacing deterministically. Pure rejection sampling was tried first
        but stalls once the obstacles fill a large fraction of the field (e.g. 16 bars at a 1.3 m
        minimum spacing). Each obstacle keeps a fixed grid cell across resets; the per-reset
        jitter (bounded so the spacing guarantee always holds) together with the obstacles'
        random footprints gives per-env variety. Only the XY position is set here; z and
        orientation are left as sampled. Requires the placement band to be at least
        (cols * min_xy_spacing) x (rows * min_xy_spacing) -- otherwise the jitter clamps to 0 and
        the nominal cell spacing (still the best achievable) is used.
        """
        _, num_assets, _ = positions.shape
        num_envs = len(env_ids)
        n = int(min(num_used, num_assets))
        if n <= 1 or self.min_xy_spacing <= 0.0:
            return positions
        device = positions.device
        min_dist = self.min_xy_spacing
        # per-env XY placement band from the asset ratio window (identical for every obstacle)
        bx0, bx1, by0, by1 = self._placement_band()
        bx0, bx1, by0, by1 = bx0[env_ids], bx1[env_ids], by0[env_ids], by1[env_ids]
        cols = int(math.ceil(math.sqrt(n)))
        rows = int(math.ceil(n / cols))
        k = torch.arange(n, device=device)
        col = (k % cols).float()
        row = torch.div(k, cols, rounding_mode="floor").float()
        cell_x = (bx1 - bx0) / cols  # (num_envs,)
        cell_y = (by1 - by0) / rows
        cx = bx0.unsqueeze(1) + (col.unsqueeze(0) + 0.5) * cell_x.unsqueeze(1)  # (num_envs, n)
        cy = by0.unsqueeze(1) + (row.unsqueeze(0) + 0.5) * cell_y.unsqueeze(1)
        jitter_x = torch.clamp((cell_x - min_dist) * 0.5, min=0.0).unsqueeze(1)  # (num_envs, 1)
        jitter_y = torch.clamp((cell_y - min_dist) * 0.5, min=0.0).unsqueeze(1)
        cx = cx + (2.0 * torch.rand(num_envs, n, device=device) - 1.0) * jitter_x
        cy = cy + (2.0 * torch.rand(num_envs, n, device=device) - 1.0) * jitter_y
        positions = positions.clone()
        positions[env_ids, :n, 0] = cx
        positions[env_ids, :n, 1] = cy
        return positions

    def _random_rejection_xy_spacing(self, positions, num_used, env_ids):
        """NavRL-style random scatter: sample uniform XY positions and reject candidates that are
        too close to already placed obstacles. If the local field is saturated, progressively relax
        the spacing so high-density runs do not stall, mirroring NavRL's obstacle placement trick.
        """
        num_assets = positions.shape[1]
        n = int(min(num_used, num_assets))
        if n <= 1 or self.min_xy_spacing <= 0.0 or len(env_ids) == 0:
            return positions

        device = positions.device
        num_envs = len(env_ids)
        bx0, bx1, by0, by1 = self._placement_band()
        bx0, bx1, by0, by1 = bx0[env_ids], bx1[env_ids], by0[env_ids], by1[env_ids]
        width = (bx1 - bx0).clamp(min=1e-6)
        height = (by1 - by0).clamp(min=1e-6)
        placed_x = torch.empty((num_envs, n), device=device)
        placed_y = torch.empty((num_envs, n), device=device)
        min_dist = torch.full((num_envs,), self.min_xy_spacing, device=device)
        relax_factor = min(max(self.placement_relax_factor, 0.0), 1.0)

        for k in range(n):
            pending = torch.ones(num_envs, dtype=torch.bool, device=device)
            attempts = torch.zeros(num_envs, dtype=torch.int32, device=device)
            while pending.any():
                idx = pending.nonzero(as_tuple=False).squeeze(-1)
                cand_x = bx0[idx] + width[idx] * torch.rand(len(idx), device=device)
                cand_y = by0[idx] + height[idx] * torch.rand(len(idx), device=device)
                if k == 0:
                    valid = torch.ones(len(idx), dtype=torch.bool, device=device)
                else:
                    dx = placed_x[idx, :k] - cand_x.unsqueeze(1)
                    dy = placed_y[idx, :k] - cand_y.unsqueeze(1)
                    nearest_sq = (dx * dx + dy * dy).min(dim=1).values
                    valid = nearest_sq >= min_dist[idx].pow(2)

                accepted = idx[valid]
                if len(accepted) > 0:
                    placed_x[accepted, k] = cand_x[valid]
                    placed_y[accepted, k] = cand_y[valid]
                    pending[accepted] = False

                rejected = idx[~valid]
                if len(rejected) > 0:
                    attempts[rejected] += 1
                    relax = attempts[rejected] >= self.placement_attempts_before_relax
                    if relax.any():
                        relax_ids = rejected[relax]
                        min_dist[relax_ids] *= relax_factor
                        attempts[relax_ids] = 0

        positions = positions.clone()
        positions[env_ids, :n, 0] = placed_x
        positions[env_ids, :n, 1] = placed_y
        return positions
