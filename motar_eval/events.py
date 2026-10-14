"""Per-episode acquisition chronology for the KROC2027 recorder.

Measurement, track and ground-truth geometry are recorded as separate signals so that "the detector
measured the target", "the tracker carried it" and "the target was in view" are never conflated:

* ``camera_valid`` -- the camera detector produced a valid target measurement for this observation
* ``fused_valid``  -- camera, or LiDAR associated with the camera-started track (the task's
  ``diagnostics["visible"]``; LiDAR can never start a track)
* ``track_active`` -- the constant-velocity tracker is active after this observation's update; it
  stays active while coasting and expires after its memory
* ``geo_in_fov``   -- ground truth: target centre inside the camera frustum (evaluation only)
* ``geo_visible``  -- ground truth: inside the frustum and the centre ray unoccluded

``observe`` is called once per observation, including the first one after a reset; observation
indices are 1-based within the episode, so the observation that drives action ``k`` (0-based) has
index ``k + 1``. A never-acquired episode reports ``None`` as its first index, never 0. A loss that
is still open when the episode ends is reported as censored, with the observations spent in it.

Imports only torch; never ``aerial_gym``.
"""

from __future__ import annotations

from typing import Dict, List

import torch

EVENTS_CONTRACT = "KROC2027_ACQUISITION_EVENTS_V1"

_SIGNALS = ("camera_valid", "fused_valid", "track_active", "geo_in_fov", "geo_visible")


class AcquisitionEventMonitor:
    def __init__(self, num_envs: int, device="cpu"):
        self.num_envs = int(num_envs)
        self.device = torch.device(device)
        n, dev = self.num_envs, self.device
        long = torch.long

        def zeros():
            return torch.zeros(n, dtype=long, device=dev)

        def minus_one():
            return torch.full((n,), -1, dtype=long, device=dev)

        self.obs = zeros()
        self.first = {name: minus_one() for name in _SIGNALS}
        self.count = {name: zeros() for name in _SIGNALS}
        self.geo_visible_undetected = zeros()
        # Losses for the fused measurement ("detection") and for the track.
        self._prev = {
            "fused_valid": torch.zeros(n, dtype=torch.bool, device=dev),
            "track_active": torch.zeros(n, dtype=torch.bool, device=dev),
        }
        self.losses = {"fused_valid": zeros(), "track_active": zeros()}
        self.reacquisitions = {"fused_valid": zeros(), "track_active": zeros()}
        self.reacq_latency_sum = {"fused_valid": zeros(), "track_active": zeros()}
        self.reacq_latency_max = {"fused_valid": zeros(), "track_active": zeros()}
        self.loss_start = {"fused_valid": minus_one(), "track_active": minus_one()}

    def observe(self, camera_valid, fused_valid, track_active, geo_in_fov, geo_visible) -> None:
        signals = {
            "camera_valid": camera_valid,
            "fused_valid": fused_valid,
            "track_active": track_active,
            "geo_in_fov": geo_in_fov,
            "geo_visible": geo_visible,
        }
        signals = {k: v.to(self.device).bool() for k, v in signals.items()}
        self.obs += 1
        index = self.obs
        for name, value in signals.items():
            fresh = value & (self.first[name] < 0)
            self.first[name] = torch.where(fresh, index, self.first[name])
            self.count[name] += value.long()
        self.geo_visible_undetected += (signals["geo_visible"] & ~signals["camera_valid"]).long()
        for name in ("fused_valid", "track_active"):
            now = signals[name]
            prev = self._prev[name]
            acquired_before = self.first[name] >= 0
            lost = prev & ~now
            self.losses[name] += lost.long()
            self.loss_start[name] = torch.where(lost, index, self.loss_start[name])
            regained = ~prev & now & acquired_before & (self.loss_start[name] >= 0)
            latency = index - self.loss_start[name]
            self.reacquisitions[name] += regained.long()
            self.reacq_latency_sum[name] += torch.where(regained, latency, torch.zeros_like(latency))
            self.reacq_latency_max[name] = torch.where(
                regained, torch.maximum(self.reacq_latency_max[name], latency),
                self.reacq_latency_max[name],
            )
            self.loss_start[name] = torch.where(
                regained, torch.full_like(index, -1), self.loss_start[name]
            )
            self._prev[name] = now

    def finish(self, env_ids) -> List[Dict]:
        """Close the episodes of ``env_ids``: return one record each, then clear their state."""
        env_ids = torch.as_tensor(env_ids, dtype=torch.long, device=self.device)
        if env_ids.numel() == 0:
            return []
        host = {
            "obs": self.obs[env_ids].tolist(),
            "geo_visible_undetected": self.geo_visible_undetected[env_ids].tolist(),
        }
        for name in _SIGNALS:
            host["first_" + name] = self.first[name][env_ids].tolist()
            host["count_" + name] = self.count[name][env_ids].tolist()
        for name in ("fused_valid", "track_active"):
            for key, store in (
                ("losses", self.losses),
                ("reacq", self.reacquisitions),
                ("reacq_sum", self.reacq_latency_sum),
                ("reacq_max", self.reacq_latency_max),
                ("open", self.loss_start),
            ):
                host[key + "_" + name] = store[name][env_ids].tolist()
        records = []
        for row in range(len(host["obs"])):
            obs = host["obs"][row]
            record = {"observations": obs}
            for name in _SIGNALS:
                first = host["first_" + name][row]
                record["first_obs_" + name] = first if first >= 0 else None
                record["ever_" + name] = first >= 0
                record["obs_" + name] = host["count_" + name][row]
            record["geo_visible_undetected_obs"] = host["geo_visible_undetected"][row]
            for name, label in (("fused_valid", "detection"), ("track_active", "track")):
                reacq = host["reacq_" + name][row]
                open_start = host["open_" + name][row]
                record[label + "_losses"] = host["losses_" + name][row]
                record[label + "_reacquisitions"] = reacq
                record[label + "_reacq_latency_mean_obs"] = (
                    host["reacq_sum_" + name][row] / reacq if reacq > 0 else None
                )
                record[label + "_reacq_latency_max_obs"] = (
                    host["reacq_max_" + name][row] if reacq > 0 else None
                )
                # A loss still open at the end is censored, not closed.
                record[label + "_open_loss_censored_obs"] = (
                    obs - open_start + 1 if open_start >= 0 else None
                )
            records.append(record)
        self._clear(env_ids)
        return records

    def _clear(self, env_ids) -> None:
        self.obs[env_ids] = 0
        self.geo_visible_undetected[env_ids] = 0
        for name in _SIGNALS:
            self.first[name][env_ids] = -1
            self.count[name][env_ids] = 0
        for name in ("fused_valid", "track_active"):
            self._prev[name][env_ids] = False
            self.losses[name][env_ids] = 0
            self.reacquisitions[name][env_ids] = 0
            self.reacq_latency_sum[name][env_ids] = 0
            self.reacq_latency_max[name][env_ids] = 0
            self.loss_start[name][env_ids] = -1
