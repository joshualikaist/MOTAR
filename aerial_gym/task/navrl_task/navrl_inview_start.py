"""Opt-in evaluation condition and recorder of CAM-DX1 (docs/plans/prereg_camonly_dx1_capture_diag_2026-11-25.md).
Installed by tools/realveh_entry.py only when its switch is on, after every other module (outermost). Both parts are
evaluation-only: they never train, and the recorder never feeds the actor.

1. Visible start (NAVRL_INVIEW_START=1, NAVRL_INVIEW_START_HALF_DEG, NAVRL_INVIEW_START_SEED). Wraps
   NavRLTask._align_general_spawn_yaw_to_target (navrl_task.py:2460-2481). The original runs first; it rotates the
   initial yaw only before FOV-curriculum saturation and is a no-op in evaluation (full distribution forces saturation).
   Then, for the reset envs, this module sets the drone's initial yaw so that the bearing of the sampled target, seen from
   the drone's centre, is uniform in [-HALF_DEG, +HALF_DEG], whatever the saturation flag, from its OWN torch.Generator
   seeded with NAVRL_INVIEW_START_SEED (the global torch RNG stream is not touched). The orientation is written into
   obs_dict["robot_orientation"] exactly where the original writes it; reset_idx commits it in the same indexed write
   (navrl_task.py:4840 calls the aligner, :4841-4846 update_states + write_to_sim in general-spawn mode), so it is the
   pose of the first rendered observation. Target placement is the task's own (NAVRL_GENERAL_GOAL_DIST_MAX is the
   caller's); positions and the target are not moved. Refused: a second install, HALF_DEG not below half the
   detector's horizontal FOV, and a reset outside general-spawn vision mode (where the task would not commit the pose).

2. Recorder (NAVRL_INVIEW_DIAG=1, NAVRL_INVIEW_DIAG_JSONL): one JSON row per finished episode, aligned with the task's
   episode ledger exactly as navrl_cams2diag is (same finished set, the same per-env episode counter, written in the
   outermost compute_state_reward_and_terminations wrapper before any reset). It wraps reset_idx (start geometry, after
   the task committed the pose and the target), _process_obs_perception (the pose and target the camera renders with:
   the step-0 bearing from robot_vehicle_orientation, review r1 P1-1, and every frame's distance, r1 P2-7),
   transform_action_to_command (one frame per control step: the observation the policy acted on, i.e. the last
   perception render) and compute_state_reward_and_terminations. Fields:
     start_bearing_deg      bearing of the target from the drone's centre in the drone's yaw frame at the episode start
                            (reset), wrapped to (-180, 180];   start_dist_xy_m / start_dist_m  horizontal / 3-D distance;
     step0_render_bearing_deg / step0_render_dist_xy_m   the same, read from the pose the first observation was rendered
                            with (robot_vehicle_orientation at _process_obs_perception): the checked quantity;
     start_detected         the task's detection flag (_visible_now, the one the policy and the cams2 capture rule use)
                            on the episode's first observation;  start_target_pixels  rendered target pixels then;
     first_detected_step    control step of the first detected observation (None if never);
     frames / detected_frames / detected_pixel_sum   observations, detected observations, and the RENDERED target pixel
                            count (the simulator's private target mask, detector.target_mask > 0, including occlusion)
                            summed over the detected observations: "target pixel area per detected frame";
     det_frames_by_dist_bin / det_pixels_by_dist_bin   the same two sums by the horizontal distance of that observation
                            (2 m bins from 0 m, the last bin >= 30 m; the distance at the frame's render);
     exec_speed_mean_mps    mean horizontal norm of the EXECUTED command (after delay, governor, blind cap);
     robot_speed_mean_mps   mean horizontal speed of the vehicle after each step.
   A summary line ([inviewdiag] JSON on stderr) at REPORT_AT_CALLS and every REPORT_EVERY_CALLS steps carries running
   totals and the camera / detection resolution the detector actually built (camera_wh, detect_wh).
   The target mask is renderer-private ground truth (CLAUDE.md observation contract): read here for an evaluation
   metric only, after the observation was written; nothing is returned to the task.
navrl_task.py and navrl_perception.py are byte-pinned, so the methods are wrapped at run time.
"""
import json
import math
import os
import sys

START_SWITCH = "NAVRL_INVIEW_START"
HALF_DEG_ENV = "NAVRL_INVIEW_START_HALF_DEG"
SEED_ENV = "NAVRL_INVIEW_START_SEED"
HALF_DEG_RANGE = (0.5, 89.0)
SEED_RANGE = (0, 2 ** 31 - 1)
DIAG_SWITCH = "NAVRL_INVIEW_DIAG"
DIAG_PATH_ENV = "NAVRL_INVIEW_DIAG_JSONL"
START_PREFIX = "[inview]"
DIAG_PREFIX = "[inviewdiag]"
DIST_BIN_M = 2.0
N_DIST_BINS = 16                    # [0, 2), [2, 4), ..., [28, 30), [30, inf)
REPORT_AT_CALLS = (1, 32, 320)
REPORT_EVERY_CALLS = 320
_INSTALLED = {}


def _switch(env, name):
    value = env.get(name, "").strip()
    if value not in ("", "0", "1"):
        raise ValueError("%s must be unset, 0 or 1, got %r" % (name, value))
    return value == "1"


def start_settings_from_env(environ=None):
    """Visible-start settings (stdlib only); None when NAVRL_INVIEW_START is off. Refuses a parameter set while off, a
    missing, malformed, non-finite or out-of-range value, and a switch other than unset / 0 / 1."""
    env = os.environ if environ is None else environ
    if not _switch(env, START_SWITCH):
        stray = sorted(k for k in (HALF_DEG_ENV, SEED_ENV) if env.get(k, "").strip())
        if stray:
            raise ValueError("%s set while %s is off" % (", ".join(stray), START_SWITCH))
        return None
    raw_half, raw_seed = env.get(HALF_DEG_ENV, "").strip(), env.get(SEED_ENV, "").strip()
    if not raw_half or not raw_seed:
        raise ValueError("%s and %s are required when %s=1" % (HALF_DEG_ENV, SEED_ENV, START_SWITCH))
    try:
        half = float(raw_half)
    except ValueError:
        raise ValueError("%s is not a number: %r" % (HALF_DEG_ENV, raw_half)) from None
    if not (math.isfinite(half) and HALF_DEG_RANGE[0] <= half <= HALF_DEG_RANGE[1]):
        raise ValueError("%s=%r outside [%g, %g]" % (HALF_DEG_ENV, raw_half, HALF_DEG_RANGE[0], HALF_DEG_RANGE[1]))
    if not (raw_seed.isascii() and raw_seed.isdigit()):
        raise ValueError("%s must be a non-negative integer, got %r" % (SEED_ENV, raw_seed))
    seed = int(raw_seed)
    if not SEED_RANGE[0] <= seed <= SEED_RANGE[1]:
        raise ValueError("%s=%r outside [%d, %d]" % (SEED_ENV, raw_seed, SEED_RANGE[0], SEED_RANGE[1]))
    return {HALF_DEG_ENV: half, SEED_ENV: seed}


def diag_settings_from_env(environ=None):
    """Recorder settings; None when NAVRL_INVIEW_DIAG is off. Refuses a stray path while off, a missing or relative
    path, an existing file and a switch other than unset / 0 / 1."""
    env = os.environ if environ is None else environ
    path = env.get(DIAG_PATH_ENV, "").strip()
    if not _switch(env, DIAG_SWITCH):
        if path:
            raise ValueError("%s set while %s is off" % (DIAG_PATH_ENV, DIAG_SWITCH))
        return None
    if not path or not os.path.isabs(path):
        raise ValueError("%s must be an absolute path when %s=1" % (DIAG_PATH_ENV, DIAG_SWITCH))
    if os.path.exists(path):
        raise ValueError("%s exists; refusing to overwrite" % path)
    return {DIAG_PATH_ENV: path}


# ---------------------------------------------------------------- pure geometry (torch tensors)
def wrap_pi(angle):
    """Angles wrapped to (-pi, pi]."""
    import torch

    return torch.atan2(torch.sin(angle), torch.cos(angle))


def yaw_from_quat_xyzw(q):
    """Yaw of [N, 4] xyzw quaternions."""
    import torch

    x, y, z, w = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    return torch.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quat_from_yaw(yaw):
    """Level xyzw quaternions of yaw angles (the form _randomize_general_drone_spawn and the original aligner write)."""
    import torch

    quat = torch.zeros((yaw.shape[0], 4), dtype=yaw.dtype, device=yaw.device)
    quat[:, 2] = torch.sin(0.5 * yaw)
    quat[:, 3] = torch.cos(0.5 * yaw)
    return quat


def inview_yaw(start_xy, goal_xy, u, half_rad):
    """Initial yaw putting the target bearing at rel = (2u - 1) * half_rad (u in [0, 1)): yaw = world bearing - rel.
    Returns (yaw, rel)."""
    delta = goal_xy - start_xy
    world = (delta[:, 1]).atan2(delta[:, 0])
    rel = (2.0 * u - 1.0) * float(half_rad)
    return world - rel, rel


def start_geometry(robot_pos, robot_quat, target_pos):
    """(bearing in the drone's yaw frame [rad, wrapped], horizontal distance, 3-D distance) of the target."""
    rel = target_pos - robot_pos
    world = rel[:, 1].atan2(rel[:, 0])
    bearing = wrap_pi(world - yaw_from_quat_xyzw(robot_quat))
    return bearing, rel[:, 0:2].norm(dim=1), rel.norm(dim=1)


def dist_bin(dxy):
    """Index of the 2 m horizontal-distance bin (last bin open)."""
    return (dxy / DIST_BIN_M).floor().clamp(min=0, max=N_DIST_BINS - 1).long()


# ---------------------------------------------------------------- 1. visible start
def install_start_task(task_cls, s, task_config):
    """Wrap _align_general_spawn_yaw_to_target. Refuses a second install and a half-angle not inside the FOV."""
    import torch

    if getattr(task_cls, "_inview_start_settings", None) is not None:
        raise RuntimeError("inview start already installed on %r" % task_cls)
    half_deg, seed = float(s[HALF_DEG_ENV]), int(s[SEED_ENV])
    hfov = float(task_config.vision.detector_hfov_deg)
    if not half_deg < 0.5 * hfov:
        raise RuntimeError("%s %g must be below half the detector FOV (%g / 2)" % (HALF_DEG_ENV, half_deg, hfov))
    half = math.radians(half_deg)
    orig_align = task_cls._align_general_spawn_yaw_to_target
    stats = {"calls": 0, "envs": 0, "max_abs_rel_deg": 0.0}
    stream = sys.stderr          # bound now: rl_games' quiet startup swaps the streams while the task is built

    def _report():
        stream.write("%s calls=%d envs=%d max_abs_rel_deg=%.4f half_deg=%g seed=%d\n"
                     % (START_PREFIX, stats["calls"], stats["envs"], stats["max_abs_rel_deg"], half_deg, seed))
        stream.flush()

    def _align_general_spawn_yaw_to_target(self, env_ids, start_pos, goal):
        out = orig_align(self, env_ids, start_pos, goal)
        if not self.general_spawn_mode or not self.vision_mode:
            raise RuntimeError("inview start needs general-spawn vision mode (the task commits the pose only there)")
        n = len(env_ids)
        if n == 0:
            return out
        gen = getattr(self, "_inview_gen", None)
        if gen is None:
            gen = torch.Generator(device=start_pos.device)
            gen.manual_seed(seed)
            self._inview_gen = gen
        u = torch.rand(n, generator=gen, device=start_pos.device, dtype=start_pos.dtype)
        yaw, rel = inview_yaw(start_pos[:, 0:2], goal[:, 0:2], u, half)
        self.obs_dict["robot_orientation"][env_ids] = quat_from_yaw(yaw).to(self.obs_dict["robot_orientation"].dtype)
        stats["calls"] += 1
        stats["envs"] += n
        stats["max_abs_rel_deg"] = max(stats["max_abs_rel_deg"], math.degrees(float(rel.abs().max().item())))
        if stats["calls"] in REPORT_AT_CALLS or stats["calls"] % REPORT_EVERY_CALLS == 0:
            _report()
        return out

    task_cls._align_general_spawn_yaw_to_target = _align_general_spawn_yaw_to_target
    task_cls._inview_start_settings = dict(s)
    task_cls._inview_start_stats = stats
    _INSTALLED["start"] = (task_cls, orig_align)
    return {"half_deg": half_deg, "seed": seed, "detector_hfov_deg": hfov,
            "wrapped": ["_align_general_spawn_yaw_to_target"]}


# ---------------------------------------------------------------- 2. recorder
def install_diag_task(task_cls, s):
    import torch

    if getattr(task_cls, "_inview_diag_settings", None) is not None:
        raise RuntimeError("inview recorder already installed on %r" % task_cls)
    path = s[DIAG_PATH_ENV]
    orig_reset = task_cls.reset_idx
    orig_transform = task_cls.transform_action_to_command
    orig_reward = task_cls.compute_state_reward_and_terminations
    orig_obs = task_cls._process_obs_perception
    stream = sys.stderr
    st = {"steps": 0, "rows": 0, "t": None, "ep_index": None, "handle": None, "camera_wh": None, "detect_wh": None,
          "totals": {"capture": 0, "crash": 0, "timeout": 0, "start_detected": 0, "starts_recorded": 0,
                     "detected_frames": 0, "frames": 0}}
    FLOAT = ("start_bearing", "start_dxy", "start_d3", "det_pix_sum", "exec_speed_sum", "robot_speed_sum", "r_bearing",
             "r_dxy")
    LONG = ("ep_steps", "frames", "det_frames", "start_pix", "robot_steps")

    def _ensure(self, device):
        n = int(self.num_envs)
        if st["t"] is None:
            t = {k: torch.zeros(n, dtype=torch.float64, device=device) for k in FLOAT}
            t.update({k: torch.zeros(n, dtype=torch.long, device=device) for k in LONG})
            t["first_det"] = torch.full((n,), -1, dtype=torch.long, device=device)
            t["start_det"] = torch.zeros(n, dtype=torch.bool, device=device)
            t["start_known"] = torch.zeros(n, dtype=torch.bool, device=device)
            t["render_known"] = torch.zeros(n, dtype=torch.bool, device=device)
            t["need_render"] = torch.zeros(n, dtype=torch.bool, device=device)
            t["render_dxy"] = torch.zeros(n, dtype=torch.float64, device=device)
            t["bin_frames"] = torch.zeros((n, N_DIST_BINS), dtype=torch.float64, device=device)
            t["bin_pix"] = torch.zeros((n, N_DIST_BINS), dtype=torch.float64, device=device)
            st["t"] = t
            st["ep_index"] = [0] * n
        if st["handle"] is None:
            st["handle"] = open(path, "x", encoding="utf-8")
        return st["t"]

    def _clear(t, ids):
        for k in FLOAT + LONG:
            t[k][ids] = 0
        t["first_det"][ids] = -1
        t["start_det"][ids] = False
        t["render_known"][ids] = False
        t["need_render"][ids] = False
        t["bin_frames"][ids] = 0.0
        t["bin_pix"][ids] = 0.0

    def reset_idx(self, env_ids):
        out = orig_reset(self, env_ids)
        if env_ids is None or len(env_ids) == 0:
            return out
        t = _ensure(self, self.target_position.device)
        ids = torch.as_tensor(env_ids, device=self.target_position.device).long()
        _clear(t, ids)
        bearing, dxy, d3 = start_geometry(self.obs_dict["robot_position"][ids].to(torch.float64),
                                          self.obs_dict["robot_orientation"][ids].to(torch.float64),
                                          self.target_position[ids].to(torch.float64))
        t["start_bearing"][ids] = bearing
        t["start_dxy"][ids] = dxy
        t["start_d3"][ids] = d3
        t["start_known"][ids] = True
        t["need_render"][ids] = True          # the step-0 pose is read where the camera renders it
        return out

    def _process_obs_perception(self, *args, **kwargs):
        """The rendered pose: the detector renders with robot_position / robot_vehicle_orientation / target_position as
        they are when the observation is built (navrl_task.py:6459-6461); nothing moves during the call."""
        t = _ensure(self, self.target_position.device)
        bearing, dxy, _ = start_geometry(self.obs_dict["robot_position"].to(torch.float64),
                                         self.obs_dict["robot_vehicle_orientation"].to(torch.float64),
                                         self.target_position.to(torch.float64))
        out = orig_obs(self, *args, **kwargs)
        need = t["need_render"]
        t["r_bearing"] = torch.where(need, bearing, t["r_bearing"])
        t["r_dxy"] = torch.where(need, dxy, t["r_dxy"])
        t["render_known"] = t["render_known"] | need
        t["need_render"] = torch.zeros_like(need)
        t["render_dxy"] = dxy                 # the distance of the frame the next action is computed from
        return out

    def transform_action_to_command(self, actions):
        command = orig_transform(self, actions)
        t = _ensure(self, actions.device)
        det = getattr(self, "detector", None)
        mask = getattr(det, "target_mask", None)
        if mask is None:
            raise RuntimeError("inview recorder needs the camera detector's target mask (vision mode)")
        if st["camera_wh"] is None:
            st["camera_wh"] = [int(det.width), int(det.height)]
            st["detect_wh"] = [int(getattr(det, "detect_width", det.width)), int(getattr(det, "detect_height", det.height))]
        vis = self._visible_now.bool()
        pix = (mask > 0).sum(dim=(1, 2)).to(torch.long)
        first = t["ep_steps"] == 0
        t["start_det"] = torch.where(first, vis, t["start_det"])
        t["start_pix"] = torch.where(first, pix, t["start_pix"])
        t["first_det"] = torch.where((t["first_det"] < 0) & vis, t["ep_steps"], t["first_det"])
        visf = vis.to(torch.float64)
        t["frames"] += 1
        t["det_frames"] += vis.to(torch.long)
        t["det_pix_sum"] += visf * pix.to(torch.float64)
        b = dist_bin(t["render_dxy"]).unsqueeze(1)    # the rendered frame's distance (the target advances before this)
        t["bin_frames"].scatter_add_(1, b, visf.unsqueeze(1))
        t["bin_pix"].scatter_add_(1, b, (visf * pix.to(torch.float64)).unsqueeze(1))
        t["exec_speed_sum"] += command[:, 0:2].to(torch.float64).norm(dim=1)
        t["ep_steps"] += 1
        return command

    def compute_state_reward_and_terminations(self, *args, **kwargs):
        out = orig_reward(self, *args, **kwargs)
        t = _ensure(self, self.captured_now.device)
        st["steps"] += 1
        t["robot_speed_sum"] += self.obs_dict["robot_linvel"][:, 0:2].to(torch.float64).norm(dim=1)
        t["robot_steps"] += 1
        captured = self.captured_now.bool()
        crashed = self.crashed_now.bool() if self.crashed_now.dtype == torch.bool else self.crashed_now > 0
        limit = sys.modules[type(self).__module__]._episode_limit_reached(self.sim_env.sim_steps,
                                                                           self.task_config.episode_len_steps)
        timeout = limit.bool() & ~captured & ~crashed
        finished = captured | crashed | timeout
        if bool(finished.any()):
            ids = torch.nonzero(finished, as_tuple=False).flatten()
            cols = {k: t[k][ids].tolist() for k in FLOAT + LONG + ("first_det", "start_det", "start_known", "render_known")}
            bf, bp = t["bin_frames"][ids].tolist(), t["bin_pix"][ids].tolist()
            cap, cra = captured[ids].tolist(), crashed[ids].tolist()
            for j, env in enumerate(ids.tolist()):
                outcome = "capture" if cap[j] else ("crash" if cra[j] else "timeout")
                frames, rsteps = int(cols["frames"][j]), int(cols["robot_steps"][j])
                known = bool(cols["start_known"][j])
                row = {"env_index": env, "env_episode_index": st["ep_index"][env], "completion_step": st["steps"],
                       "outcome": outcome, "episode_steps": int(cols["ep_steps"][j]), "start_recorded": known,
                       "start_bearing_deg": math.degrees(cols["start_bearing"][j]) if known else None,
                       "start_dist_xy_m": cols["start_dxy"][j] if known else None,
                       "start_dist_m": cols["start_d3"][j] if known else None,
                       "step0_render_bearing_deg": math.degrees(cols["r_bearing"][j]) if cols["render_known"][j] else None,
                       "step0_render_dist_xy_m": cols["r_dxy"][j] if cols["render_known"][j] else None,
                       "start_detected": bool(cols["start_det"][j]) if frames else None,
                       "start_target_pixels": int(cols["start_pix"][j]) if frames else None,
                       "first_detected_step": int(cols["first_det"][j]) if cols["first_det"][j] >= 0 else None,
                       "frames": frames, "detected_frames": int(cols["det_frames"][j]),
                       "detected_pixel_sum": float(cols["det_pix_sum"][j]),
                       "det_frames_by_dist_bin": [int(x) for x in bf[j]], "det_pixels_by_dist_bin": bp[j],
                       "exec_speed_mean_mps": cols["exec_speed_sum"][j] / frames if frames else None,
                       "robot_speed_mean_mps": cols["robot_speed_sum"][j] / rsteps if rsteps else None}
                st["handle"].write(json.dumps(row, sort_keys=True) + "\n")
                st["rows"] += 1
                st["ep_index"][env] += 1
                tot = st["totals"]
                tot[outcome] += 1
                tot["start_detected"] += int(bool(row["start_detected"]))
                tot["starts_recorded"] += int(known)
                tot["detected_frames"] += row["detected_frames"]
                tot["frames"] += frames
            st["handle"].flush()
            _clear(t, ids)
            t["start_known"][ids] = False
        if st["steps"] in REPORT_AT_CALLS or st["steps"] % REPORT_EVERY_CALLS == 0:
            _report()
        return out

    def _report():
        payload = {"event": "summary", "snapshot": "running totals through this step count, not a final line",
                   "steps": st["steps"], "rows": st["rows"], "totals": dict(st["totals"]),
                   "camera_wh": st["camera_wh"], "detect_wh": st["detect_wh"]}
        stream.write("%s %s\n" % (DIAG_PREFIX, json.dumps(payload, sort_keys=True)))
        stream.flush()

    task_cls.reset_idx = reset_idx
    task_cls.transform_action_to_command = transform_action_to_command
    task_cls.compute_state_reward_and_terminations = compute_state_reward_and_terminations
    task_cls._process_obs_perception = _process_obs_perception
    task_cls._inview_diag_settings = dict(s)
    task_cls._inview_diag_stats = st
    _INSTALLED["diag"] = (task_cls, orig_reset, orig_transform, orig_reward, orig_obs)
    return {"jsonl": path, "dist_bin_m": DIST_BIN_M, "n_dist_bins": N_DIST_BINS,
            "wrapped": ["reset_idx", "_process_obs_perception", "transform_action_to_command",
                        "compute_state_reward_and_terminations"]}


def telemetry_lines(texts, prefix=DIAG_PREFIX):
    """The JSON payloads of the recorder's summary lines in `texts`."""
    out = []
    for text in texts:
        for line in text.splitlines():
            if line.startswith(prefix + " {"):
                try:
                    out.append(json.loads(line[len(prefix) + 1:]))
                except ValueError:
                    pass
    return out


def install_start(s, task_config):
    """Production install (tools/realveh_entry.py), after every other module."""
    from aerial_gym.task.navrl_task import navrl_task

    info = install_start_task(navrl_task.NavRLTask, s, task_config)
    info["settings"] = dict(s)
    return info


def install_diag(s):
    """Production install (tools/realveh_entry.py), outermost."""
    from aerial_gym.task.navrl_task import navrl_task

    info = install_diag_task(navrl_task.NavRLTask, s)
    info["settings"] = dict(s)
    return info


def uninstall():
    """Tests only."""
    diag = _INSTALLED.pop("diag", None)
    if diag is not None:
        cls, rs, tr, rw, ob = diag
        cls.reset_idx, cls.transform_action_to_command, cls.compute_state_reward_and_terminations = rs, tr, rw
        cls._process_obs_perception = ob
        st = getattr(cls, "_inview_diag_stats", None)
        if st and st.get("handle"):
            st["handle"].close()
        for name in ("_inview_diag_settings", "_inview_diag_stats"):
            if name in cls.__dict__:
                delattr(cls, name)
    start = _INSTALLED.pop("start", None)
    if start is not None:
        cls, al = start
        cls._align_general_spawn_yaw_to_target = al
        for name in ("_inview_start_settings", "_inview_start_stats"):
            if name in cls.__dict__:
                delattr(cls, name)
