"""B1: an external yaw-scanning supervisor around the unchanged policy (KROC2027 Experiment B).

It rewrites the policy's normalised action before the task turns it into a command, so the
existing speed governor, altitude hold, command and previous-action feedback all see what is
executed (priority: policy -> supervisor -> governor -> final command -> feedback).

Inputs are the actor-side signals only: the fused target measurement, whether the tracker is
active, and the measured LiDAR clearance. Ground-truth target pose, visibility, range and obstacle
geometry are never inputs. Rules (docs/plans/kroc2027_protocol_v1.json, experiment_B.supervisor):

* initial search: from 1.0 s after reset, if the target was never measured and no track is active;
  at most 10.5 s;
* re-search: when an established track expires (active -> inactive; the tracker coasts through
  missed frames, so a single miss never triggers); at most 4.5 s per re-search;
* at most 15 s of searching per episode, after which the policy keeps control;
* while searching: horizontal action 0 and a constant yaw rate of 0.6 rad/s in a direction fixed
  per episode from (env, episode index) parity -- balanced, never from ground truth; if the LiDAR
  clearance is at or below the hard margin or invalid, the yaw is 0 as well (hold);
* control returns to the policy on the first step that sees a fused measurement with an active
  track.

Imports only torch.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Dict, List

import torch

SEARCH_CONTRACT = "KROC2027_B1_YAW_SCAN_V1"
MODE_NONE, MODE_INITIAL, MODE_RESEARCH = 0, 1, 2


@dataclass(frozen=True)
class SearchSpec:
    yaw_rate_rad_s: float = 0.6
    initial_trigger_s: float = 1.0
    initial_search_max_s: float = 10.5
    research_max_s: float = 4.5
    episode_search_max_s: float = 15.0
    dt_s: float = 0.1

    def steps(self, seconds: float) -> int:
        return int(round(seconds / self.dt_s))


class YawScanSupervisor:
    def __init__(self, num_envs: int, device, yaw_rate_max_rad_s: float, hard_margin_m: float,
                 spec: SearchSpec = SearchSpec()):
        if spec.yaw_rate_rad_s > yaw_rate_max_rad_s:
            raise ValueError("scan yaw rate exceeds the vehicle's yaw-rate scale")
        self.spec = spec
        self.num_envs = int(num_envs)
        self.device = torch.device(device)
        self.yaw_action = spec.yaw_rate_rad_s / float(yaw_rate_max_rad_s)
        self.hard_margin_m = float(hard_margin_m)
        n, dev = self.num_envs, self.device
        long = torch.long

        def zeros(dtype=long):
            return torch.zeros(n, dtype=dtype, device=dev)

        self.mode = zeros()
        self.steps_since_reset = zeros()
        self.current = zeros()
        self.total = zeros()
        self.ever_measured = zeros(torch.bool)
        self.initial_done = zeros(torch.bool)
        self.prev_track = zeros(torch.bool)
        self.sign = torch.ones(n, device=dev)
        self.episode_index = torch.full((n,), -1, dtype=long, device=dev)
        self.active = zeros(torch.bool)
        self.suppressed = zeros(torch.bool)
        self.stats = {key: zeros() for key in (
            "initial_searches", "research_count", "search_steps", "initial_search_steps",
            "research_steps", "suppressed_steps", "ended_by_acquisition", "ended_by_limit")}

    def on_reset(self, env_ids) -> None:
        env_ids = torch.as_tensor(env_ids, dtype=torch.long, device=self.device)
        if env_ids.numel() == 0:
            return
        self.episode_index[env_ids] += 1
        parity = (env_ids + self.episode_index[env_ids]) % 2
        self.sign[env_ids] = torch.where(parity == 0, 1.0, -1.0).to(self.sign.dtype)
        for tensor in (self.mode, self.steps_since_reset, self.current, self.total):
            tensor[env_ids] = 0
        for tensor in (self.ever_measured, self.initial_done, self.prev_track, self.active,
                       self.suppressed):
            tensor[env_ids] = False
        for tensor in self.stats.values():
            tensor[env_ids] = 0

    def step(self, actions, fused_valid, track_active, lidar_clearance_m):
        """Return the executed normalised actions for this step; record why."""
        spec = self.spec
        fused = fused_valid.to(self.device).bool()
        track = track_active.to(self.device).bool()
        clearance = lidar_clearance_m.to(self.device)
        self.ever_measured |= fused
        acquired = fused & track
        budget_left = self.total < spec.steps(spec.episode_search_max_s)

        # End an ongoing search: acquisition hands control back on this very step.
        searching = self.mode != MODE_NONE
        returned = searching & acquired
        limit = torch.where(self.mode == MODE_INITIAL,
                            torch.full_like(self.current, spec.steps(spec.initial_search_max_s)),
                            torch.full_like(self.current, spec.steps(spec.research_max_s)))
        exhausted = searching & ~acquired & ((self.current >= limit) | ~budget_left)
        self.stats["ended_by_acquisition"] += returned.long()
        self.stats["ended_by_limit"] += exhausted.long()
        self.mode = torch.where(returned | exhausted, torch.zeros_like(self.mode), self.mode)

        # Start a search.
        idle = self.mode == MODE_NONE
        start_initial = (idle & ~self.initial_done & (self.steps_since_reset >= spec.steps(spec.initial_trigger_s))
                         & ~self.ever_measured & ~track & budget_left)
        expired = self.prev_track & ~track
        start_research = idle & ~start_initial & expired & ~acquired & budget_left
        self.initial_done |= start_initial | self.ever_measured
        self.mode = torch.where(start_initial, torch.full_like(self.mode, MODE_INITIAL), self.mode)
        self.mode = torch.where(start_research, torch.full_like(self.mode, MODE_RESEARCH), self.mode)
        self.current = torch.where(start_initial | start_research, torch.zeros_like(self.current),
                                   self.current)
        self.stats["initial_searches"] += start_initial.long()
        self.stats["research_count"] += start_research.long()

        active = self.mode != MODE_NONE
        suppressed = active & (~torch.isfinite(clearance) | (clearance <= self.hard_margin_m))
        out = actions.clone()
        out[:, 0:2] = torch.where(active.unsqueeze(1), torch.zeros_like(out[:, 0:2]), out[:, 0:2])
        yaw = torch.where(suppressed, torch.zeros_like(self.sign), self.sign * self.yaw_action)
        out[:, 3] = torch.where(active, yaw.to(out.dtype), out[:, 3])

        self.current += active.long()
        self.total += active.long()
        self.stats["search_steps"] += active.long()
        self.stats["initial_search_steps"] += (active & (self.mode == MODE_INITIAL)).long()
        self.stats["research_steps"] += (active & (self.mode == MODE_RESEARCH)).long()
        self.stats["suppressed_steps"] += suppressed.long()
        self.active = active
        self.suppressed = suppressed
        self.prev_track = track
        self.steps_since_reset += 1
        return out

    def episode_records(self, env_ids) -> List[Dict]:
        env_ids = torch.as_tensor(env_ids, dtype=torch.long, device=self.device)
        host = {k: v[env_ids].cpu().tolist() for k, v in self.stats.items()}
        active = self.active[env_ids].cpu().tolist()
        sign = self.sign[env_ids].cpu().tolist()
        out = []
        for row in range(len(active)):
            record = {"search_" + k: host[k][row] for k in host}
            record["search_active_at_end"] = bool(active[row])
            record["search_scan_sign"] = int(math.copysign(1, sign[row]))
            out.append(record)
        return out
