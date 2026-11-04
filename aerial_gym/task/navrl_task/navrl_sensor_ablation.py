"""Opt-in LiDAR ablation by bearing for the camera-only study (D-2026-11-04-3; plan
docs/plans/camonly_study_plan_v1.md). Default OFF: nothing imports this module except the entry point
tools/camonly_entry.py, which a camera-only launcher selects explicitly.

Conditions (the camera FOV is perception's own |bearing| <= hfov / 2 mask, the one used for fusion):

  full               LiDAR everywhere (transparent; the identity control)
  no_lidar_in_fov    LiDAR removed inside the camera FOV, kept outside
  no_lidar_out_fov   LiDAR kept inside the camera FOV, removed outside
  no_lidar           LiDAR removed everywhere (camera only)
  no_lidar_obs_only  LiDAR removed from perception everywhere; the speed governor keeps the raw LiDAR

Removal means every online consumer sees no LiDAR return in the removed bins:
  - perception: `observe` receives `lidar_m` with removed bins set to `lidar_max_range` ("no return"). In the FOV
    the min-fusion then yields the camera's own value; outside it the static scan holds the existing no-return
    encoding 1.0 (K1). `_associate_lidar_target` requires a return below 0.995 * max range, so the LiDAR target
    association, its confidence field and the LiDAR part of fused visibility switch off without other changes.
  - speed governor (riskcap, directional corridor clearance only): the clearance is computed on a governor scan
    with the raw LiDAR in kept bins, the camera-fused range in removed in-FOV bins and no return in removed
    out-of-FOV bins. When out-of-FOV LiDAR is removed, a moving command whose bearing lies outside the FOV gets
    clearance 0, so riskcap caps it at its 2.0 m/s floor (K2). Any other governor feature raises.
Not changed: rewards (r_ss reads the raw LiDAR; a training-only signal), terminations (physics contacts) and the
evaluation recorders, which may read the LiDAR and ground truth but do not feed control.

navrl_task.py and navrl_perception.py are byte-pinned by the H-R3 tests, so this module wraps their methods at run
time instead of editing them.
"""
import inspect
import math
import os

import torch

ENV = "NAVRL_SENSOR_ABLATION"
CONDITIONS = {
    "full": {"in_fov": True, "out_fov": True, "governor": "same"},
    "no_lidar_in_fov": {"in_fov": False, "out_fov": True, "governor": "same"},
    "no_lidar_out_fov": {"in_fov": True, "out_fov": False, "governor": "same"},
    "no_lidar": {"in_fov": False, "out_fov": False, "governor": "same"},
    "no_lidar_obs_only": {"in_fov": False, "out_fov": False, "governor": "raw"},
}
_LINEAGE_FLAGS = ("sensor_lineage_s_l1a", "sensor_lineage_s_l1b", "sensor_lineage_s_l1c", "sensor_lineage_s_l1d")
_INSTALLED = {}


def condition_from_env(environ=None):
    value = (environ if environ is not None else os.environ).get(ENV, "")
    if value not in CONDITIONS:
        raise ValueError("%s must be one of %s, got %r" % (ENV, sorted(CONDITIONS), value))
    return value


def fov_inside(perception):
    """Perception's own fusion mask (navrl_perception.py, `_fuse_static_and_extract_obstacles`)."""
    return perception._lidar_angles.abs() <= perception.hfov * 0.5


def removed_bins(perception, condition):
    spec = CONDITIONS[condition]
    inside = fov_inside(perception)
    keep = torch.where(inside, torch.full_like(inside, spec["in_fov"]), torch.full_like(inside, spec["out_fov"]))
    return ~keep


def _check_perception(perception):
    for flag in _LINEAGE_FLAGS:
        if getattr(perception, flag, False):
            raise RuntimeError("sensor ablation is defined for the Policy F perception path only; %s is on" % flag)


def install_perception(module, condition):
    """Wrap `observe` and `_fuse_static_and_extract_obstacles` of `module.NavRLPerceptionModule`."""
    if condition not in CONDITIONS:
        raise ValueError("unknown condition %r" % condition)
    cls = module.NavRLPerceptionModule
    if getattr(cls, "_sensor_ablation_condition", None) is not None:
        raise RuntimeError("sensor ablation already installed on %r" % cls)
    vb, hb = int(module.VBEAMS), int(module.HBEAMS)
    angle_tol, range_tol = float(module.TARGET_LIKE_ANGLE_RAD), float(module.TARGET_LIKE_RANGE_TOL_M)
    orig_observe, orig_fuse = cls.observe, cls._fuse_static_and_extract_obstacles
    sig_observe, sig_fuse = inspect.signature(orig_observe), inspect.signature(orig_fuse)

    def arg(bound, name):
        return bound.arguments[name]

    def observe(self, *args, **kwargs):
        _check_perception(self)
        bound = sig_observe.bind(self, *args, **kwargs)
        raw = bound.arguments["lidar_m"]
        removed = removed_bins(self, condition)
        self._ablation_raw_lidar_m = raw
        self._ablation_removed = removed
        if bool(removed.any()):
            masked = raw.reshape(self.num_envs, vb, hb).clone()
            masked[:, :, removed] = float(self.lidar_max_range)
            bound.arguments["lidar_m"] = masked.reshape(raw.shape)
        return orig_observe(*bound.args, **bound.kwargs)

    def fuse(self, *args, **kwargs):
        bound = sig_fuse.bind(self, *args, **kwargs)
        static_state, tokens = orig_fuse(*bound.args, **bound.kwargs)
        self._ablation_fused_m = (static_state.reshape(self.num_envs, vb, hb) * float(self.lidar_max_range)).detach()
        raw = getattr(self, "_ablation_raw_lidar_m", None)
        if raw is not None:
            # The historical association window (`_fuse_static_and_extract_obstacles`, Frozen-F branch), on the RAW
            # LiDAR: the governor's target mask when it keeps the raw LiDAR (no_lidar_obs_only).
            a = arg(bound, "target_bearing").reshape(-1, 1, 1)
            delta = torch.atan2(torch.sin(self._lidar_angles.view(1, 1, hb) - a),
                                torch.cos(self._lidar_angles.view(1, 1, hb) - a)).abs()
            scan = raw.reshape(self.num_envs, vb, hb)
            self._ablation_raw_target_like = (
                arg(bound, "visible").reshape(-1, 1, 1)
                & (delta < angle_tol)
                & ((scan - arg(bound, "target_surface_range").reshape(-1, 1, 1)).abs() < range_tol)
            )
        return static_state, tokens

    cls.observe = observe
    cls._fuse_static_and_extract_obstacles = fuse
    cls._sensor_ablation_condition = condition
    _INSTALLED["perception"] = (cls, orig_observe, orig_fuse)
    return cls


def governor_scan(perception, condition, scan_m, max_range_m):
    """The scan the governor's clearance is computed on, [N, V, H] metres."""
    removed = getattr(perception, "_ablation_removed", None)
    if removed is None or not bool(removed.any()):
        return scan_m
    inside = fov_inside(perception)
    out = scan_m.clone()
    rin, rout = removed & inside, removed & ~inside
    if bool(rin.any()):
        out[:, :, rin] = perception._ablation_fused_m[:, :, rin].to(out.dtype)
    if bool(rout.any()):
        out[:, :, rout] = float(max_range_m)
    return out


def out_of_fov_commands(perception, command_xy):
    bearing = torch.atan2(command_xy[:, 1], command_xy[:, 0])
    return (bearing.abs() > perception.hfov * 0.5) & (command_xy.norm(dim=1) > 1e-6)


def install_governor(task_cls, condition):
    """Wrap `task_cls._apply_sensor_speed_governor`; inside that call only, `directional_lidar_clearance` (looked up
    in the method's module globals) computes the clearance on the condition's governor scan."""
    if condition not in CONDITIONS:
        raise ValueError("unknown condition %r" % condition)
    if getattr(task_cls, "_sensor_ablation_condition", None) is not None:
        raise RuntimeError("sensor ablation already installed on %r" % task_cls)
    spec = CONDITIONS[condition]
    orig_method = task_cls._apply_sensor_speed_governor
    module_globals = orig_method.__globals__
    orig_dlc = module_globals["directional_lidar_clearance"]
    active = {"task": None}

    def directional_lidar_clearance(scan_m, bearings_rad, command_xy, **kwargs):
        task = active["task"]
        if task is None or condition == "full":
            return orig_dlc(scan_m, bearings_rad, command_xy, **kwargs)
        perception = task.perception
        if spec["governor"] == "raw":
            kwargs = dict(kwargs, target_return_mask=perception._ablation_raw_target_like)
            return orig_dlc(scan_m, bearings_rad, command_xy, **kwargs)
        scan = governor_scan(perception, condition, scan_m, kwargs["max_range_m"])
        clearance = orig_dlc(scan, bearings_rad, command_xy, **kwargs)
        if not spec["out_fov"]:
            clearance = torch.where(out_of_fov_commands(perception, command_xy), torch.zeros_like(clearance),
                                    clearance)
        return clearance

    def _apply_sensor_speed_governor(self, command_xy):
        if condition != "full":
            cfg = self.speed_governor_cfg
            bad = [name for name in ("open_width_enabled", "lateral_channel_enabled", "yaw_cap_enabled")
                   if getattr(cfg, name, False)]
            if cfg.mode != "riskcap" or bad:
                raise RuntimeError("sensor ablation supports the riskcap directional governor only (mode %r, %s)"
                                   % (cfg.mode, bad))
            if self.perception is None or getattr(self.perception, "_ablation_removed", None) is None:
                raise RuntimeError("sensor ablation governor ran before the wrapped perception observed")
        active["task"] = self
        try:
            return orig_method(self, command_xy)
        finally:
            active["task"] = None

    module_globals["directional_lidar_clearance"] = directional_lidar_clearance
    task_cls._apply_sensor_speed_governor = _apply_sensor_speed_governor
    task_cls._sensor_ablation_condition = condition
    _INSTALLED["governor"] = (task_cls, orig_method, orig_dlc)
    return task_cls


def install(condition):
    """Production install (tools/camonly_entry.py): import isaacgym before calling this."""
    from aerial_gym.task.navrl_task import navrl_perception
    from aerial_gym.task.navrl_task import navrl_task

    install_perception(navrl_perception, condition)
    install_governor(navrl_task.NavRLTask, condition)
    return {"condition": condition, "perception": navrl_perception.__file__, "task": navrl_task.__file__,
            "hbeams": int(navrl_perception.HBEAMS), "vbeams": int(navrl_perception.VBEAMS)}
