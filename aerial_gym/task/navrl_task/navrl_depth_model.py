"""Opt-in stereo depth model of the Luxonis OAK-D Pro for the real-vehicle study (2026-11-08; survey
~/motar_tmp/realveh_design_20261108/SURVEY.md section 2).

Default OFF: nothing imports this module unless NAVRL_DEPTH_MODEL=oakd, and then only tools/realveh_entry.py does.
Every parameter is required from the environment (no defaults here; a missing or out-of-range value refuses):

  NAVRL_DEPTH_MINZ_M      MinZ [m] in [0, 2]: pixels whose true z-depth is below it are invalid (e.g. 0.4)
  NAVRL_DEPTH_MAXZ_M      MaxZ [m] in (0, 40]: pixels whose true z-depth is beyond it are invalid; must not exceed
                          perception's detection far plane (checked on the first frame). Equal to it = no cut.
  NAVRL_DEPTH_SIGMA_D_PX  disparity noise sigma_d [px] in [0, 2] (e.g. 0.15, INFERRED for subpixel)
  NAVRL_DEPTH_FB_PXM      focal length x baseline f*B [px m] in (0, 1000]. CAM-K2: OAK-D Pro 800P with extended
                          disparity + subpixel, f = 882.5 px (Luxonis' rectified 800P focal length), B = 0.075 m:
                          66.1875 px m. The unit's own calibration decides the real value.
  NAVRL_DEPTH_HOLE_P      per-pixel probability of a random hole in [0, 0.5]
  NAVRL_DEPTH_SEED        int >= 0, seeds this module's own torch.Generator (the global RNG is never consumed)

install() wraps NavRLPerceptionModule.observe (navrl_perception.py is byte-pinned, so it is wrapped at run time) with
functools.wraps, so inspect.signature still reports the original parameters: the LiDAR ablation, installed AFTER this
module, binds observe's signature at install. Per frame, on the `depth` argument only (RGB, LiDAR, everything else
pass through):
  1. z = range * cos(ray), from perception's own camera intrinsics (fx, fy, cx, cy at camera resolution); the renderer
     stores the Euclidean range along normalized rays, not z-depth (navrl_detector.py).
  2. range += n / cos(ray), n ~ N(0, sigma_z), sigma_z = z^2 * sigma_d / (f*B): the stereo z-noise, per pixel, iid.
  3. invalid where z < MinZ, z > MaxZ or a hole (Bernoulli HOLE_P); invalid and range >= the detection far plane are
     encoded as perception.max_camera_range (20 m in F's config). That value is "no return" for every consumer: the
     detector mask needs depth < max_camera_range, and the obstacle fusion maps a column whose minimum is >= the
     obstacle far plane camera_obstacle_max_range (10 m) to the LiDAR no-return (camera_no_return_to_lidar_range).
     The renderer's own no-hit pixels (range within 1e-3 m of camera_obstacle_max_range) pass through unchanged.
The real OAK outputs 0 for invalid pixels; a deployment pipeline must map 0 to "no return" the same way. This
encoding is optimistic: a bar closer than MinZ disappears instead of being clamped to MinZ. Which convention is used
is an open decision for the user (CAM-K2 preregistration); the conservative clamp is not built.
With sigma_d = 0, HOLE_P = 0, MinZ = 0 and MaxZ = max_camera_range, observe is bitwise identical to OFF.

Telemetry: one JSON object per line, prefix LOG_PREFIX, written to the stream that was sys.stderr when
install_perception() ran (review r2 N-1: rl_games' quiet startup swaps sys.stderr for /dev/null while it builds the
env; binding at install keeps the lines independent of call order). Lines: "install" with the settings; "first" after the first frame and "summary" after STATS_CALLS frames, each with the
cumulative fractions of eligible pixels (not renderer no-hit) invalidated by MinZ, by MaxZ and by holes, the mean
|range change| of the valid noised pixels and sigma_z at their median z; then "heartbeat" every HEARTBEAT_EVERY calls
with the call count only. The statistics need a device synchronisation, so they are computed for the first
STATS_CALLS frames only. tools/camk2_train.py requires these lines after a run.
"""
import functools
import inspect
import json
import os
import sys

# torch is imported inside the functions: tools/realveh_entry.py validates the settings before Isaac Gym is imported,
# and Isaac Gym must be imported before torch.

ENV = "NAVRL_DEPTH_MODEL"
MODEL = "oakd"
LIMITS = {"NAVRL_DEPTH_MINZ_M": (0.0, 2.0), "NAVRL_DEPTH_MAXZ_M": (1e-6, 40.0),
          "NAVRL_DEPTH_SIGMA_D_PX": (0.0, 2.0), "NAVRL_DEPTH_FB_PXM": (1e-6, 1000.0),
          "NAVRL_DEPTH_HOLE_P": (0.0, 0.5)}
REQUIRED = tuple(LIMITS) + ("NAVRL_DEPTH_SEED",)
NO_HIT_TOL_M = 1e-3            # the tolerance of navrl_perception.camera_no_return_to_lidar_range
LOG_PREFIX = "[depth-model] "
STATS_CALLS = 50
HEARTBEAT_EVERY = 5000
_STREAM = {"stream": None}                 # bound at install_perception(); see the module docstring
_LINEAGE_FLAGS = ("sensor_lineage_s_l1a", "sensor_lineage_s_l1b", "sensor_lineage_s_l1c", "sensor_lineage_s_l1d")
_INSTALLED = {}


def settings_from_env(environ=None):
    """Validated settings; refuses (ValueError) unless NAVRL_DEPTH_MODEL=oakd and every parameter is in bounds."""
    env = os.environ if environ is None else environ
    if env.get(ENV, "").strip() != MODEL:
        raise ValueError("%s must be %s to use navrl_depth_model, got %r" % (ENV, MODEL, env.get(ENV)))
    missing = [k for k in REQUIRED if not env.get(k, "").strip()]
    if missing:
        raise ValueError("NAVRL_DEPTH_MODEL=oakd needs %s" % ", ".join(missing))
    out = {}
    for k, (lo, hi) in LIMITS.items():
        try:
            v = float(env[k])
        except ValueError:
            raise ValueError("%s is not a number: %r" % (k, env[k])) from None
        if not lo <= v <= hi:          # also refuses nan
            raise ValueError("%s=%r outside [%g, %g]" % (k, v, lo, hi))
        out[k] = v
    if out["NAVRL_DEPTH_MAXZ_M"] <= out["NAVRL_DEPTH_MINZ_M"]:
        raise ValueError("NAVRL_DEPTH_MAXZ_M must exceed NAVRL_DEPTH_MINZ_M")
    try:
        seed = int(env["NAVRL_DEPTH_SEED"])
    except ValueError:
        raise ValueError("NAVRL_DEPTH_SEED is not an integer: %r" % env["NAVRL_DEPTH_SEED"]) from None
    if seed < 0:
        raise ValueError("NAVRL_DEPTH_SEED must be >= 0")
    out["NAVRL_DEPTH_SEED"] = seed
    return out


def ray_cosine(perception, device):
    """cos of the angle between each camera pixel's ray and the optical axis, [H, W] (cached per perception)."""
    import torch

    cached = getattr(perception, "_depth_model_cos", None)
    if cached is not None and cached.device == torch.device(device):
        return cached
    h, w = int(perception.height), int(perception.width)
    vv, uu = torch.meshgrid(torch.arange(h, dtype=torch.float32), torch.arange(w, dtype=torch.float32), indexing="ij")
    tan_u = (uu - float(perception.cx)) / float(perception.fx)
    tan_v = (vv - float(perception.cy)) / float(perception.fy)
    cos = (1.0 / torch.sqrt(1.0 + tan_u * tan_u + tan_v * tan_v)).to(device)
    perception._depth_model_cos = cos
    return cos


def _generator(perception, s, device):
    import torch

    gen = getattr(perception, "_depth_model_generator", None)
    if gen is None:
        gen = torch.Generator(device=device)
        gen.manual_seed(int(s["NAVRL_DEPTH_SEED"]))
        perception._depth_model_generator = gen
    return gen


def _check(perception, depth, s):
    for flag in _LINEAGE_FLAGS:
        if getattr(perception, flag, False):
            raise RuntimeError("the depth model is defined for the Policy F perception path only; %s is on" % flag)
    if getattr(perception, "detect_decoupled", False):
        raise RuntimeError("the depth model does not reach a decoupled detect-resolution frame")
    shape = (int(perception.num_envs), int(perception.height), int(perception.width))
    if tuple(depth.shape) != shape:
        raise RuntimeError("depth model expects depth %s, got %s" % (shape, tuple(depth.shape)))
    far = float(perception.max_camera_range)
    if s["NAVRL_DEPTH_MAXZ_M"] > far:
        raise RuntimeError("NAVRL_DEPTH_MAXZ_M=%g exceeds perception max_camera_range %g" % (s["NAVRL_DEPTH_MAXZ_M"], far))
    if far < float(perception.camera_obstacle_max_range):
        raise RuntimeError("max_camera_range %g < camera_obstacle_max_range %g: the invalid encoding would not read "
                           "as no return in the obstacle fusion" % (far, perception.camera_obstacle_max_range))


def apply(perception, depth, s):
    """The modelled depth frame (a new tensor; `depth` is not modified). Records the invalid mask on perception."""
    import torch

    _check(perception, depth, s)
    far = float(perception.max_camera_range)
    cos = ray_cosine(perception, depth.device)
    z = depth * cos
    no_hit = (depth - float(perception.camera_obstacle_max_range)).abs() <= NO_HIT_TOL_M
    gen = _generator(perception, s, depth.device)
    sigma_d, fb = float(s["NAVRL_DEPTH_SIGMA_D_PX"]), float(s["NAVRL_DEPTH_FB_PXM"])
    noisy = depth
    if sigma_d > 0.0:
        n = torch.randn(depth.shape, generator=gen, device=depth.device, dtype=depth.dtype)
        noisy = (depth + n * (z * z * (sigma_d / fb)) / cos).clamp(0.0, far)
    minz = z < float(s["NAVRL_DEPTH_MINZ_M"])
    maxz = z > float(s["NAVRL_DEPTH_MAXZ_M"])
    invalid = minz | maxz
    hole = None
    if float(s["NAVRL_DEPTH_HOLE_P"]) > 0.0:
        u = torch.rand(depth.shape, generator=gen, device=depth.device, dtype=depth.dtype)
        hole = u < float(s["NAVRL_DEPTH_HOLE_P"])
        invalid = invalid | hole
    invalid = invalid & ~no_hit
    out = torch.where(invalid, torch.full_like(depth, far), noisy)
    out = torch.where(no_hit, depth, out)
    perception._depth_model_invalid = invalid
    _telemetry(perception, depth, out, z, no_hit, minz, maxz, hole, invalid, s)
    return out


def log(event, **fields):
    stream = _STREAM["stream"] if _STREAM["stream"] is not None else sys.stderr
    stream.write(LOG_PREFIX + json.dumps(dict(event=event, **fields), sort_keys=True) + "\n")
    stream.flush()


def _telemetry(perception, depth, out, z, no_hit, minz, maxz, hole, invalid, s):
    """Cumulative statistics over the first STATS_CALLS frames; a heartbeat after that (see the module docstring)."""
    st = getattr(perception, "_depth_model_stats", None)
    if st is None:
        st = perception._depth_model_stats = {"calls": 0, "eligible": 0, "minz": 0, "maxz": 0, "hole": 0,
                                              "valid": 0, "abs_noise_sum": 0.0, "median_z": []}
    st["calls"] += 1
    if st["calls"] <= STATS_CALLS:
        eligible = ~no_hit
        valid = eligible & ~invalid
        st["eligible"] += int(eligible.sum())
        st["minz"] += int((minz & eligible).sum())
        st["maxz"] += int((maxz & eligible).sum())
        st["hole"] += int((hole & eligible).sum()) if hole is not None else 0
        st["valid"] += int(valid.sum())
        st["abs_noise_sum"] += float((out - depth).abs()[valid].sum())
        if bool(valid.any()):
            st["median_z"].append(float(z[valid].median()))
        if st["calls"] in (1, STATS_CALLS):
            k = float(s["NAVRL_DEPTH_SIGMA_D_PX"]) / float(s["NAVRL_DEPTH_FB_PXM"])
            med = sorted(st["median_z"])[len(st["median_z"]) // 2] if st["median_z"] else None
            e = max(1, st["eligible"])
            log("first" if st["calls"] == 1 else "summary", calls=st["calls"],
                frames=st["calls"] * int(perception.num_envs), eligible_pixels=st["eligible"],
                invalid_minz=st["minz"] / e, invalid_maxz=st["maxz"] / e, invalid_hole=st["hole"] / e,
                mean_abs_noise_m=st["abs_noise_sum"] / max(1, st["valid"]), median_valid_z_m=med,
                sigma_z_at_median_m=(k * med * med) if med is not None else None)
    elif st["calls"] % HEARTBEAT_EVERY == 0:
        log("heartbeat", calls=st["calls"])


def install_perception(module, s):
    """Wrap `module.NavRLPerceptionModule.observe`. Must run before the LiDAR ablation's install."""
    cls = module.NavRLPerceptionModule
    if getattr(cls, "_depth_model_settings", None) is not None:
        raise RuntimeError("depth model already installed on %r" % cls)
    if getattr(cls, "_sensor_ablation_condition", None) is not None:
        raise RuntimeError("install the depth model before the sensor ablation (it binds observe's signature)")
    s = dict(s)
    _STREAM["stream"] = sys.stderr             # bound now, outside rl_games' quiet construction (review r2 N-1)
    orig_observe = cls.observe
    sig = inspect.signature(orig_observe)
    if "depth" not in sig.parameters:
        raise RuntimeError("observe has no `depth` parameter: %s" % sig)

    @functools.wraps(orig_observe)
    def observe(self, *args, **kwargs):
        bound = sig.bind(self, *args, **kwargs)
        bound.arguments["depth"] = apply(self, bound.arguments["depth"], s)
        return orig_observe(*bound.args, **bound.kwargs)

    cls.observe = observe
    cls._depth_model_settings = s
    _INSTALLED["perception"] = (cls, orig_observe)
    log("install", settings=s)
    return cls


def install(s, perception_module=None):
    """Production install (tools/realveh_entry.py): import isaacgym before calling this."""
    if perception_module is None:
        from aerial_gym.task.navrl_task import navrl_perception as perception_module
    install_perception(perception_module, s)
    k = float(s["NAVRL_DEPTH_SIGMA_D_PX"]) / float(s["NAVRL_DEPTH_FB_PXM"])
    return {"model": MODEL, "settings": dict(s), "perception": perception_module.__file__,
            "sigma_z_m_at": {"%gm" % z: k * z * z for z in (1.0, 2.0, 4.0, 6.0, 10.0)},
            "invalid_encoding": "perception.max_camera_range"}

