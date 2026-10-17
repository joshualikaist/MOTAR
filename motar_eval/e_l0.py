"""E-L0 evaluation-only recorder (``NAVRL_KROC2027_E_L0=1``), attached at runtime to the KROC2027 adapter.

``attach(adapter)`` wraps the hook methods of an existing ``KrocRecorder`` *instance* (no source file is
changed; ``kroc_eval_adapter.py`` stays byte-identical to the frozen OCC G0 source), builds the
recorder below and returns it. Every wrapped hook still calls the original with the same arguments.

Records, without writing any task state:

* 100 Hz contact chronology from the robot contact-force tensor, collected by a callback that is
  *chained* onto the env manager's physics-step callback (an existing callback still runs first);
* 10 Hz timing: env-manager step counters when an observation is staged, consumed and its action
  has been simulated, plus the applied action and command;
* camera identity provenance (renderer target mask vs perception detection) and LiDAR association
  provenance (renderer semantic buffer, id 50 = target, on the beam and row the association used),
  captured by a call-through wrapper of ``perception._associate_lidar_target`` that returns the
  original outputs untouched;
* pixel-centre silhouette classes: target-alone support computed on the detector's own ray table and
  pose (analytic sphere, far plane), visible support = the renderer's target mask, a centre-ray mesh
  query; RANGE_EXIT / FOV_EXIT / SCALE_LOSS / FULL / CENTRE_ONLY / PARTIAL / VISIBLE;
* tracker age, active and expiry; per-episode outcome columns: the task outcome unchanged,
  ``capture_with_contact``, ``collision_free_capture``, evidence-based crash causes,
  ``timeout_at_rest`` and ``safe_abort = NOT_IMPLEMENTED``.

GT read here goes to files only; nothing returned to the task depends on it.
"""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path

import numpy as np
import torch

SCHEMA = "motar.e-l0.v1"
FRAMES_FILE, STEPS_FILE, EPISODES_FILE, SUMMARY_FILE = (
    "e_l0_frames.bin", "e_l0_steps.jsonl", "e_l0_episodes.jsonl", "e_l0_summary.json")
CLASSES = ("NONE", "RANGE_EXIT", "FOV_EXIT", "SCALE_LOSS", "FULL", "CENTRE_ONLY", "PARTIAL", "VISIBLE")
CAMERA_PROV = ("NONE", "TRUE_TARGET", "FALSE_DETECTION", "MISSED")
LIDAR_PROV = ("NONE", "CORRECT", "FALSE_ASSOCIATION")
TARGET_SEMANTIC = 50
NOT_IMPLEMENTED = "NOT_IMPLEMENTED"
BODY_R_CONS = math.sqrt(0.14 ** 2 * 2 + 0.04 ** 2)
REST_SPEED_MPS, REST_CLEARANCE_M, REST_FRAMES = 0.1, 0.45, 10

FRAME_DTYPE = np.dtype([
    ("env", "<u2"), ("episode", "<u4"), ("frame", "<u2"),
    ("cls", "u1"), ("centre_in_frustum", "u1"), ("centre_blocked", "u1"), ("below_min_pixels", "u1"),
    ("alone_px", "<u2"), ("visible_px", "<u2"), ("visible_outside_alone_px", "<u2"), ("perception_px", "<u2"),
    ("centre_range_m", "<f4"),
    ("camera_visible", "u1"), ("lidar_visible", "u1"), ("fused_visible", "u1"), ("track_active", "u1"),
    ("camera_prov", "u1"), ("lidar_prov", "u1"), ("assoc_row", "i1"), ("assoc_beam", "i1"),
    ("assoc_semantic", "<i2"), ("assoc_surface_m", "<f4"), ("assoc_consistent", "u1"),
    ("track_age_s", "<f4"), ("expired", "u1"),
    ("contact_substeps", "u1"), ("first_contact_substep", "i1"), ("peak_contact_force_n", "<f4"),
    ("action", "<f4", (4,)), ("command", "<f4", (4,)),
])


# ------------------------------------------------------------------ pure label functions
def quat_rotate(q, v):
    """Rotate v [..., 3] by unit quaternions q [..., 4] (x, y, z, w); broadcasting."""
    shape = torch.broadcast_shapes(q[..., :3].shape, v.shape)
    qv, w, v = q[..., :3].expand(shape), q[..., 3:4].expand(shape[:-1] + (1,)), v.expand(shape)
    t = 2.0 * torch.cross(qv, v, dim=-1)
    return v + w * t + torch.cross(qv, t, dim=-1)


def quat_rotate_inverse(q, v):
    return quat_rotate(torch.cat([-q[..., :3], q[..., 3:4]], dim=-1), v)


def target_alone_mask(rays, origins, orientations, targets, radius, far):
    """[N, H, W] pixel-centre rays (camera frame [H, W, 3], rotated by the camera pose) that hit the
    analytic target sphere before the far plane, ignoring every occluder (the renderer's rule)."""
    rd = quat_rotate(orientations[:, None, None, :], rays[None])
    rd = rd / rd.norm(dim=-1, keepdim=True).clamp(min=1e-9)
    oc = (origins - targets)[:, None, None, :]
    b = (oc * rd).sum(dim=-1)
    c = (oc * oc).sum(dim=-1) - radius * radius
    disc = b * b - c
    root = torch.sqrt(disc.clamp(min=0.0))
    t = -b - root
    t = torch.where(t < 0.0, -b + root, t)
    return (disc >= 0.0) & (t >= 0.0) & (t < far)


def centre_in_frustum(origins, orientations, targets, hfov, vfov, far):
    rel = quat_rotate_inverse(orientations, targets - origins)
    x, y, z = rel[:, 0], rel[:, 1], rel[:, 2]
    return (x > 0) & (y.abs() <= x * math.tan(hfov / 2)) & (z.abs() <= x * math.tan(vfov / 2)) & \
        (rel.norm(dim=1) <= far)


def classify(alone_px, visible_px, in_frustum, centre_range, centre_blocked, radius, far):
    """Silhouette class codes (index into CLASSES)."""
    cls = torch.full_like(alone_px, CLASSES.index("VISIBLE"))
    none = alone_px == 0
    cls = torch.where(none & (centre_range - radius >= far), torch.full_like(cls, CLASSES.index("RANGE_EXIT")),
                      torch.where(none & ~in_frustum, torch.full_like(cls, CLASSES.index("FOV_EXIT")),
                                  torch.where(none, torch.full_like(cls, CLASSES.index("SCALE_LOSS")), cls)))
    some = ~none
    full = some & (visible_px == 0)
    partial = some & (visible_px > 0) & (visible_px < alone_px)
    cls = torch.where(full, torch.full_like(cls, CLASSES.index("FULL")), cls)
    cls = torch.where(partial & centre_blocked, torch.full_like(cls, CLASSES.index("CENTRE_ONLY")), cls)
    cls = torch.where(partial & ~centre_blocked, torch.full_like(cls, CLASSES.index("PARTIAL")), cls)
    return cls


def camera_provenance(camera_visible, renderer_px, min_pixels):
    out = torch.zeros_like(renderer_px)
    out = torch.where(camera_visible & (renderer_px >= 1), torch.full_like(out, CAMERA_PROV.index("TRUE_TARGET")), out)
    out = torch.where(camera_visible & (renderer_px == 0), torch.full_like(out, CAMERA_PROV.index("FALSE_DETECTION")), out)
    out = torch.where(~camera_visible & (renderer_px >= min_pixels), torch.full_like(out, CAMERA_PROV.index("MISSED")), out)
    return out


def lidar_provenance(valid, bearing, measured, scan, semantic, angles):
    """Beam nearest the association bearing (the production rule), the row of its minimum return,
    the renderer semantic id there and the label; ``scan``/``semantic`` are [N, V, H]."""
    err = torch.atan2(torch.sin(angles.view(1, -1) - bearing.view(-1, 1)),
                      torch.cos(angles.view(1, -1) - bearing.view(-1, 1))).abs()
    beam = err.argmin(dim=1)
    rows = torch.arange(scan.shape[0], device=scan.device)
    column = scan[rows, :, beam]                               # [N, V]
    low = column.min(dim=1).values
    row = (column == low.unsqueeze(1)).float().argmax(dim=1)   # first row holding the minimum
    sem = semantic[rows, row, beam]
    label = torch.zeros_like(beam)
    label = torch.where(valid & (sem == TARGET_SEMANTIC), torch.full_like(label, LIDAR_PROV.index("CORRECT")), label)
    label = torch.where(valid & (sem != TARGET_SEMANTIC), torch.full_like(label, LIDAR_PROV.index("FALSE_ASSOCIATION")), label)
    consistent = (low - measured).abs() <= 1e-6
    return {"beam": beam, "row": row, "semantic": sem, "surface": low, "label": label, "consistent": consistent}


def centre_ray_blocked(mesh_ids, origins, targets):
    """Mesh query along camera -> target centre in each env's Warp scene (the renderer's mesh)."""
    import warp as wp

    from motar_eval.e_l0_kernels import centre_ray

    o = origins.contiguous().float()
    t = targets.contiguous().float()
    out = torch.zeros(o.shape[0], dtype=torch.int32, device=o.device)
    wp.launch(kernel=centre_ray, dim=o.shape[0], inputs=[mesh_ids, wp.from_torch(o, dtype=wp.vec3),
                                                         wp.from_torch(t, dtype=wp.vec3), wp.from_torch(out, dtype=wp.int32)],
              device=str(o.device))
    return out.bool()


def silhouette_from_task(task):
    """Silhouette support and class of the frame the detector rendered last (zero-latency delivery)."""
    det = task.detector
    if getattr(det, "target_use_oriented_box", False):
        raise NotImplementedError("E-L0 silhouette needs the analytic sphere target")
    import warp as wp

    rays = wp.to_torch(det._ray_vectors_wp)
    origins, orientations = det._origins, det._orientations
    targets = task.target_position
    radius, far = float(det.target_radius), float(det.max_range)
    alone = target_alone_mask(rays, origins, orientations, targets, radius, far)
    visible = det.target_mask > 0
    vis = task.vis_cfg
    hfov = math.radians(float(vis.detector_hfov_deg))
    vfov = math.radians(float(getattr(vis, "detector_vfov_deg", 58.0)))
    in_frustum = centre_in_frustum(origins, orientations, targets, hfov, vfov, far)
    centre_range = (targets - origins).norm(dim=1)
    blocked = centre_ray_blocked(det.mesh_ids, origins, targets)
    alone_px = alone.sum(dim=(1, 2)).long()
    visible_px = visible.sum(dim=(1, 2)).long()
    outside = (visible & ~alone).sum(dim=(1, 2)).long()
    cls = classify(alone_px, visible_px, in_frustum, centre_range, blocked, radius, far)
    return {"alone_px": alone_px, "visible_px": visible_px, "visible_outside_alone_px": outside,
            "centre_in_frustum": in_frustum, "centre_range": centre_range, "centre_blocked": blocked, "cls": cls}


# ------------------------------------------------------------------ contact chain
class ContactChain:
    """Chained physics-step callback: runs an existing callback first, then records the contact force."""

    def __init__(self, existing, force_getter, threshold, num_envs, device):
        self.existing = existing
        self.force_getter = force_getter
        self.threshold = float(threshold)
        self.count = torch.zeros(num_envs, dtype=torch.long, device=device)
        self.first = torch.full((num_envs,), -1, dtype=torch.long, device=device)
        self.peak = torch.zeros(num_envs, dtype=torch.float32, device=device)
        self.substep = 0

    def __call__(self):
        if self.existing is not None:
            self.existing()

    def post_physics_step(self):
        if self.existing is not None and hasattr(self.existing, "post_physics_step"):
            self.existing.post_physics_step()
        force = self.force_getter().detach().norm(dim=1)
        hit = force > self.threshold
        self.count += hit.long()
        self.first = torch.where((self.first < 0) & hit, torch.full_like(self.first, self.substep), self.first)
        self.peak = torch.maximum(self.peak, force.float())
        self.substep += 1

    def begin_action(self):
        self.count.zero_()
        self.first.fill_(-1)
        self.peak.zero_()
        self.substep = 0


# ------------------------------------------------------------------ recorder
def _sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        for block in iter(lambda: fh.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()


class EL0Recorder:
    """Built by ``KrocRecorder`` when ``NAVRL_KROC2027_E_L0=1``; hooks mirror the adapter's."""

    def __init__(self, parent):
        self.parent, self.task = parent, parent.task
        task = self.task
        n, dev = parent.num_envs, parent.device
        self.n, self.device = n, dev
        self.cell_dir = Path(parent.cell_dir)
        for name in (FRAMES_FILE, STEPS_FILE, EPISODES_FILE, SUMMARY_FILE):
            if (self.cell_dir / name).exists():
                raise FileExistsError("E-L0 needs a fresh cell directory: %s exists" % name)
        self.perception = getattr(task, "perception", None)
        self.min_pixels = int(getattr(self.perception, "min_pixels", 2))
        self.angles = getattr(self.perception, "_lidar_angles", None)
        env_cfg = task.sim_env.cfg.env
        self.n_sub = int(env_cfg.num_physics_steps_per_env_step_mean)
        self.contact = ContactChain(task.sim_env.physics_step_callback,
                                    lambda: task.obs_dict["robot_contact_force_tensor"],
                                    float(env_cfg.collision_force_threshold), n, dev)
        task.sim_env.set_physics_step_callback(self.contact)
        self._assoc = None
        if self.perception is not None and hasattr(self.perception, "_associate_lidar_target"):
            original = self.perception._associate_lidar_target

            def associate(lidar_m, drone_pos_w, vehicle_quat, camera_visible):
                out = original(lidar_m, drone_pos_w, vehicle_quat, camera_visible)
                valid, _conf, measured, bearing = out
                self._assoc = (valid.detach().clone(), measured.detach().clone(), bearing.detach().clone(),
                               lidar_m.detach().clone())
                return out

            self.perception._associate_lidar_target = associate
        self._frames = open(self.cell_dir / FRAMES_FILE, "wb")
        self._steps = open(self.cell_dir / STEPS_FILE, "w", encoding="utf-8")
        self._episodes = open(self.cell_dir / EPISODES_FILE, "w", encoding="utf-8")
        z = lambda dtype=torch.long: torch.zeros(n, dtype=dtype, device=dev)  # noqa: E731
        self.ep = {"contact_substeps": z(), "first_contact_frame": torch.full((n,), -1, dtype=torch.long, device=dev),
                   "first_contact_substep": torch.full((n,), -1, dtype=torch.long, device=dev),
                   "correct": z(), "false": z(), "lidar_only": z(), "camera_false": z(), "expiries": z(),
                   "max_age": z(torch.float32), "frames": z(), "cls": torch.zeros(n, len(CLASSES), dtype=torch.long, device=dev)}
        self.speed_ring = torch.full((n, REST_FRAMES), math.inf, device=dev)
        self.prev_active = torch.zeros(n, dtype=torch.bool, device=dev)
        self.staged = None
        self.current = None
        self.counts = {"frames": 0, "episodes": 0, "steps": 0}
        self.action_index = 0
        self._closed = False

    # ---------------------------------------------------------------- hooks
    def on_reset(self, env_ids):
        for v in self.ep.values():
            v[env_ids] = 0
        self.ep["first_contact_frame"][env_ids] = -1
        self.ep["first_contact_substep"][env_ids] = -1
        self.speed_ring[env_ids] = math.inf
        self.prev_active[env_ids] = False

    def after_perception(self, diagnostics):
        t = self.task
        dev = self.device
        camera = diagnostics.get("camera_visible", diagnostics["visible"]).detach().bool()
        lidar = diagnostics.get("lidar_visible", torch.zeros_like(camera)).detach().bool()
        fused = diagnostics["visible"].detach().bool()
        tracker = getattr(self.perception, "tracker", None)
        active = tracker.active.detach().clone() if tracker is not None else torch.zeros_like(camera)
        age = diagnostics.get("track_age", torch.zeros(self.n, device=dev)).detach().float().clone()
        sil = silhouette_from_task(t)
        renderer_px = sil["visible_px"]
        cam_prov = camera_provenance(camera, renderer_px, self.min_pixels)
        if self._assoc is not None and self.angles is not None:
            valid, measured, bearing, lidar_m = self._assoc
            scan = lidar_m.view(self.n, -1, self.angles.numel())
            seg = t.obs_dict["segmentation_pixels"].view(self.n, -1, self.angles.numel())
            prov = lidar_provenance(valid.bool(), bearing, measured, scan, seg, self.angles)
        else:
            zeros = torch.zeros(self.n, dtype=torch.long, device=dev)
            prov = {"beam": zeros - 1, "row": zeros - 1, "semantic": zeros - 3, "surface": torch.full((self.n,), math.nan, device=dev),
                    "label": zeros, "consistent": torch.ones(self.n, dtype=torch.bool, device=dev)}
        self._assoc = None
        expired = self.prev_active & ~active
        self.prev_active = active.clone()
        self.staged = {"sil": sil, "camera": camera, "lidar": lidar, "fused": fused, "active": active, "age": age,
                       "cam_prov": cam_prov, "prov": prov, "expired": expired,
                       "perception_px": diagnostics.get("target_pixels", torch.zeros(self.n, device=dev)).detach().long(),
                       "counter": int(t.sim_env.step_counter)}

    def before_action(self):
        self.current = self.staged
        self.staged = None
        if self.current is not None:
            self.current["consumed_counter"] = int(self.task.sim_env.step_counter)
        self.contact.begin_action()

    def after_physics(self, actions, command):
        cur = self.current
        if cur is None:
            return
        t = self.task
        sil, prov = cur["sil"], cur["prov"]
        frame_idx = self.parent._steps.detach().clone()
        rec = np.zeros(self.n, dtype=FRAME_DTYPE)
        host = lambda x: x.detach().cpu().numpy()  # noqa: E731
        rec["env"] = np.arange(self.n)
        rec["episode"] = np.asarray(self.parent.episode_index, dtype=np.uint32)
        rec["frame"] = host(frame_idx)
        rec["cls"] = host(sil["cls"])
        rec["centre_in_frustum"] = host(sil["centre_in_frustum"])
        rec["centre_blocked"] = host(sil["centre_blocked"])
        rec["below_min_pixels"] = host(sil["alone_px"] < self.min_pixels)
        rec["alone_px"] = host(sil["alone_px"].clamp(max=65535))
        rec["visible_px"] = host(sil["visible_px"].clamp(max=65535))
        rec["visible_outside_alone_px"] = host(sil["visible_outside_alone_px"].clamp(max=65535))
        rec["perception_px"] = host(cur["perception_px"].clamp(max=65535))
        rec["centre_range_m"] = host(sil["centre_range"])
        for key in ("camera", "lidar", "fused", "active"):
            rec[{"camera": "camera_visible", "lidar": "lidar_visible", "fused": "fused_visible", "active": "track_active"}[key]] = host(cur[key])
        rec["camera_prov"] = host(cur["cam_prov"])
        rec["lidar_prov"] = host(prov["label"])
        rec["assoc_row"] = host(prov["row"])
        rec["assoc_beam"] = host(prov["beam"])
        rec["assoc_semantic"] = host(prov["semantic"])
        rec["assoc_surface_m"] = host(prov["surface"])
        rec["assoc_consistent"] = host(prov["consistent"])
        rec["track_age_s"] = host(cur["age"])
        rec["expired"] = host(cur["expired"])
        rec["contact_substeps"] = host(self.contact.count.clamp(max=255))
        rec["first_contact_substep"] = host(self.contact.first)
        rec["peak_contact_force_n"] = host(self.contact.peak)
        rec["action"] = host(actions[:, 0:4].float())
        rec["command"] = host(command[:, 0:4].float())
        self._frames.write(rec.tobytes())
        self.counts["frames"] += self.n
        self._steps.write(json.dumps({"action": self.action_index, "staged_counter": cur["counter"],
                                      "consumed_counter": cur.get("consumed_counter"),
                                      "simulated_counter": int(t.sim_env.step_counter)}) + "\n")
        self.action_index += 1
        self.counts["steps"] += 1
        # Episode accumulators.
        ep = self.ep
        hit = self.contact.count > 0
        newly = hit & (ep["first_contact_frame"] < 0)
        ep["first_contact_frame"] = torch.where(newly, frame_idx, ep["first_contact_frame"])
        ep["first_contact_substep"] = torch.where(newly, self.contact.first, ep["first_contact_substep"])
        ep["contact_substeps"] += self.contact.count
        ep["correct"] += (prov["label"] == LIDAR_PROV.index("CORRECT")).long()
        ep["false"] += (prov["label"] == LIDAR_PROV.index("FALSE_ASSOCIATION")).long()
        ep["lidar_only"] += (cur["lidar"] & ~cur["camera"]).long()
        ep["camera_false"] += (cur["cam_prov"] == CAMERA_PROV.index("FALSE_DETECTION")).long()
        ep["expiries"] += cur["expired"].long()
        ep["max_age"] = torch.maximum(ep["max_age"], cur["age"])
        ep["frames"] += 1
        ep["cls"].scatter_add_(1, sil["cls"].view(-1, 1), torch.ones(self.n, 1, dtype=torch.long, device=self.device))
        speed = t.obs_dict["robot_linvel"][:, 0:2].detach().norm(dim=1)
        self.speed_ring = torch.cat([self.speed_ring[:, 1:], speed.unsqueeze(1)], dim=1)
        self.current["contact_now"] = self.contact.count.clone()

    def _clearance(self, env_ids):
        center, half = self.parent._obstacles()
        pos = self.task.obs_dict["robot_position"][env_ids, None, 0:2]
        q = (pos - center[env_ids, :, 0:2]).abs() - half[env_ids, :, 0:2]
        d = q.clamp(min=0.0).norm(dim=-1)
        return d.min(dim=1).values - BODY_R_CONS if d.shape[1] else torch.full((len(env_ids),), math.inf, device=self.device)

    def on_episode_end(self, env_ids, capture, crash, timeout):
        """``env_ids`` tensor; capture/crash/timeout host lists aligned with it (before any reset)."""
        t = self.task
        ids = env_ids.cpu().tolist()
        pos = t.obs_dict["robot_position"][env_ids].detach()
        b_min, b_max = t.obs_dict["env_bounds_min"][env_ids, 0:2], t.obs_dict["env_bounds_max"][env_ids, 0:2]
        margin = float(getattr(t.vis_cfg, "oob_margin", 0.5))
        oob = ((pos[:, 0:2] < b_min - margin) | (pos[:, 0:2] > b_max + margin)).any(dim=1)
        low = pos[:, 2] < float(t.task_config.lower_height_bound)
        high = pos[:, 2] > float(t.task_config.upper_height_bound)
        contact_final = (self.current["contact_now"][env_ids] > 0) if self.current and "contact_now" in self.current \
            else torch.zeros(len(ids), dtype=torch.bool, device=self.device)
        target_contact = t.obs_dict.get("navrl_target_contact")
        target_contact = target_contact[env_ids].bool() if isinstance(target_contact, torch.Tensor) else None
        clearance = self._clearance(env_ids)
        at_rest = (self.speed_ring[env_ids].max(dim=1).values < REST_SPEED_MPS) & (clearance >= REST_CLEARANCE_M)
        ep = {k: v[env_ids].detach().cpu() for k, v in self.ep.items()}
        hostb = lambda x: x.detach().cpu().tolist()  # noqa: E731
        oob_l, low_l, high_l, cf_l, rest_l, clr_l = (hostb(x) for x in (oob, low, high, contact_final, at_rest, clearance))
        tc_l = hostb(target_contact) if target_contact is not None else [False] * len(ids)
        for row, env in enumerate(ids):
            flags = (bool(capture[row]), bool(crash[row]), bool(timeout[row]))
            outcome = ("capture", "crash", "timeout")[flags.index(True)] if sum(flags) == 1 else "invalid"
            contact_total = int(ep["contact_substeps"][row])
            causes = [name for name, on in (("contact", cf_l[row]), ("out_of_bounds", oob_l[row]),
                                            ("below_height", low_l[row]), ("above_height", high_l[row]),
                                            ("target_contact", tc_l[row])) if on]
            first_frame = int(ep["first_contact_frame"][row])
            record = {
                "env_index": env, "env_episode_index": self.parent.episode_index[env], "outcome": outcome,
                "frames": int(ep["frames"][row]),
                "contact_substeps": contact_total,
                "first_contact_frame": first_frame if first_frame >= 0 else None,
                "first_contact_substep": int(ep["first_contact_substep"][row]) if first_frame >= 0 else None,
                "first_contact_time_s": ((first_frame + (int(ep["first_contact_substep"][row]) + 1) / self.n_sub)
                                         * float(self.parent.dt)) if first_frame >= 0 else None,
                "capture_with_contact": outcome == "capture" and contact_total > 0,
                "collision_free_capture": outcome == "capture" and contact_total == 0,
                "crash_causes": causes if outcome == "crash" else [],
                "timeout_at_rest": (bool(rest_l[row]) if outcome == "timeout" else None),
                "final_clearance_m": float(clr_l[row]) if math.isfinite(clr_l[row]) else None,
                "safe_abort": NOT_IMPLEMENTED,
                "lidar_correct_associations": int(ep["correct"][row]),
                "lidar_false_associations": int(ep["false"][row]),
                "lidar_only_frames": int(ep["lidar_only"][row]),
                "camera_false_detections": int(ep["camera_false"][row]),
                "track_expiries": int(ep["expiries"][row]),
                "max_track_age_s": float(ep["max_age"][row]),
                "class_frames": {CLASSES[i]: int(ep["cls"][row, i]) for i in range(len(CLASSES)) if int(ep["cls"][row, i])},
            }
            self._episodes.write(json.dumps(record, sort_keys=True) + "\n")
            self.counts["episodes"] += 1

    def finalize(self, reason, metadata):
        if self._closed:
            return
        self._closed = True
        for fh in (self._frames, self._steps, self._episodes):
            fh.close()
        files = {name: {"bytes": (self.cell_dir / name).stat().st_size, "sha256": _sha256(self.cell_dir / name)}
                 for name in (FRAMES_FILE, STEPS_FILE, EPISODES_FILE)}
        summary = {"schema": SCHEMA, "reason": reason, "evaluation_only": True, "counts": self.counts, "files": files,
                   "frame_dtype": FRAME_DTYPE.descr, "frame_itemsize": FRAME_DTYPE.itemsize,
                   "classes": list(CLASSES), "camera_provenance": list(CAMERA_PROV), "lidar_provenance": list(LIDAR_PROV),
                   "target_semantic_id": TARGET_SEMANTIC, "min_pixels": self.min_pixels, "substeps_per_action": self.n_sub,
                   "collision_force_threshold_n": self.contact.threshold, "safe_abort": NOT_IMPLEMENTED,
                   "rest_rule": {"speed_mps": REST_SPEED_MPS, "clearance_m": REST_CLEARANCE_M, "frames": REST_FRAMES,
                                 "body_radius_m": BODY_R_CONS},
                   "frame_contract": "frame k = observation k (staged after perception, consumed by action k) plus action k, "
                                     "its command and the contacts during its physics substeps",
                   **(metadata or {})}
        (self.cell_dir / SUMMARY_FILE).write_text(json.dumps(summary, indent=1, sort_keys=True, default=str) + "\n")


SWITCH = "NAVRL_KROC2027_E_L0"


def switch_on(environ):
    value = str(environ.get(SWITCH, "0")).strip().lower()
    if value in ("", "0", "off", "false", "none"):
        return False
    if value == "1":
        return True
    raise ValueError("%s must be 0 or 1, got %r" % (SWITCH, value))


def attach(adapter, recorder=None):
    """Wrap ``adapter``'s hooks so the E-L0 recorder runs alongside them; returns the recorder.

    Order: after_physics -> E-L0 first (the adapter then advances its step counter); on_episode_end
    -> E-L0 first (the adapter then advances its episode index); every other hook -> original first.
    """
    if getattr(adapter, "_e_l0", None) is not None:
        raise RuntimeError("E-L0 already attached")
    rec = recorder if recorder is not None else EL0Recorder(adapter)
    originals = {name: getattr(adapter, name) for name in
                 ("on_reset", "after_perception", "before_action", "after_physics", "on_episode_end", "finalize")}

    def on_reset(env_ids):
        out = originals["on_reset"](env_ids)
        ids = torch.as_tensor(env_ids, dtype=torch.long, device=rec.device)
        if ids.numel():
            rec.on_reset(ids)
        return out

    def after_perception(diagnostics):
        out = originals["after_perception"](diagnostics)
        rec.after_perception(diagnostics)
        return out

    def before_action():
        out = originals["before_action"]()
        rec.before_action()
        return out

    def after_physics(actions, command):
        rec.after_physics(actions, command)
        return originals["after_physics"](actions, command)

    def on_episode_end(finished, successes, crashes, timeouts):
        fin = finished.detach().to(torch.bool)
        if bool(fin.any()):
            ids = torch.nonzero(fin, as_tuple=False).flatten()
            capture = successes.detach().to(torch.bool)[ids].cpu().tolist()
            crash = (crashes.detach() if crashes.dtype == torch.bool else crashes.detach() > 0)[ids].cpu().tolist()
            timeout = timeouts.detach().to(torch.bool)[ids].cpu().tolist()
            rec.on_episode_end(ids, capture, crash, timeout)
        return originals["on_episode_end"](finished, successes, crashes, timeouts)

    def finalize(reason="complete"):
        out = originals["finalize"](reason)
        rec.finalize(reason, {"context": dict(getattr(adapter, "context", {})),
                              "import_origins": dict(getattr(adapter, "import_origins", {})),
                              "attachment": "runtime wrap of the KrocRecorder instance (motar_eval.e_l0.attach)"})
        return out

    for name, fn in (("on_reset", on_reset), ("after_perception", after_perception), ("before_action", before_action),
                     ("after_physics", after_physics), ("on_episode_end", on_episode_end), ("finalize", finalize)):
        setattr(adapter, name, fn)
    adapter._e_l0 = rec
    return rec


def read_frames(directory):
    directory = Path(directory)
    summary = json.loads((directory / SUMMARY_FILE).read_text())
    if summary.get("schema") != SCHEMA or summary.get("frame_itemsize") != FRAME_DTYPE.itemsize:
        raise ValueError("E-L0 schema or frame size differs from this reader")
    frames = np.fromfile(directory / FRAMES_FILE, dtype=FRAME_DTYPE)
    with open(directory / EPISODES_FILE) as fh:
        episodes = [json.loads(line) for line in fh]
    with open(directory / STEPS_FILE) as fh:
        steps = [json.loads(line) for line in fh]
    return {"summary": summary, "frames": frames, "episodes": episodes, "steps": steps}


__all__ = ["SCHEMA", "CLASSES", "CAMERA_PROV", "LIDAR_PROV", "FRAME_DTYPE", "TARGET_SEMANTIC", "NOT_IMPLEMENTED",
           "quat_rotate", "quat_rotate_inverse", "target_alone_mask", "centre_in_frustum", "classify",
           "camera_provenance", "lidar_provenance", "centre_ray_blocked", "silhouette_from_task", "ContactChain",
           "EL0Recorder", "SWITCH", "switch_on", "attach", "read_frames"]
