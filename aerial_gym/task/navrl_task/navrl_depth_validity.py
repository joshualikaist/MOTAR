"""Opt-in invalid-depth validity rule for the OAK-D Pro depth model (CAM-K2 v3, 2026-11-08; preregistration
docs/plans/prereg_camonly_k2_train_v3_2026-11-08.md; diagnosis ~/motar_tmp/camk2_depthprobe_20261108/probe_r1).

Why: navrl_depth_model.py encodes every invalid pixel (z < MinZ, z > MaxZ, random hole) as "no return"
(perception.max_camera_range, 20 m). A bar face closer than MinZ 0.4 m then reads as free space, so the bar about to be
hit is missing from the obstacle tokens. This rule turns a dense blob of invalid pixels into a surface at MinZ (the
nearest distance the stereo sensor could have reported), while isolated invalid pixels stay no return.

The simulation wrapper defaults OFF: tools/realveh_entry.py installs it only with NAVRL_DEPTH_VALIDITY=blob_near
and NAVRL_DEPTH_MODEL=oakd. External sensor adapters may call apply_mask directly with their measured invalid mask;
this pure API neither installs a wrapper nor reapplies the simulation depth model. Every parameter is explicit:

  NAVRL_DEPTH_VALIDITY_WINDOW   odd int in [3, 31]: side of the square box filter [px] (e.g. 5)
  NAVRL_DEPTH_VALIDITY_DENSITY  float in (0, 1]: an invalid pixel whose local invalid density is >= this becomes near

Per frame, on the `depth` argument only, AFTER navrl_depth_model has modelled it (same delivered frame):
  1. invalid = perception._depth_model_invalid, the mask navrl_depth_model.apply records for this very frame (the
     sensor's own invalid mask; a real OAK-D reports it as depth 0). Equality with the 20 m sentinel is NOT used: a
     valid pixel whose noised range reaches the far plane is clamped to exactly the same value by the depth model.
     Freshness is checked: the depth model's call counter must have advanced by exactly one since this wrapper's last
     call, else RuntimeError.
  2. density = box mean of invalid over WINDOW x WINDOW (stride 1, same padding, padding not counted:
     avg_pool2d(count_include_pad=True) times the window area, divided by the in-image count); dense = invalid & (count >= DENSITY x in-image area - DENSITY_TOL).
  3. dense pixels are set to optical z = MinZ, converted back to ray range with the depth model's own per-pixel cosine
     (perception._depth_model_cos, from navrl_depth_model.ray_cosine): range = MinZ / cos. Every other pixel (valid,
     isolated invalid, the renderer's own 10 m no-hit pixels, target paint) passes through bit for bit.
It uses no random number (neither the global RNG nor the depth model's generator) and no simulator-only quantity.

Installation order (wrapper nesting): a later-installed wrapper runs FIRST. To run after the depth model on the frame
it produced, this wrapper must be the inner one, so install_perception() runs BEFORE navrl_depth_model's install
(tools/realveh_entry.py: validity install, then the depth model, then the LiDAR ablation). It keeps observe's signature
(functools.wraps), so the depth model and the ablation bind the original parameters.

Telemetry: JSON lines with prefix LOG_PREFIX on the stream that was sys.stderr at install (rl_games' quiet startup
swaps sys.stderr later): "install" with the settings; "first" and "summary" (after STATS_CALLS frames) with cumulative
eligible / invalid / near pixel counts and the fractions near/invalid and near/eligible; "heartbeat" every
HEARTBEAT_EVERY calls.
"""
import functools
import inspect
import json
import math
import os
import sys

ENV = "NAVRL_DEPTH_VALIDITY"
RULE = "blob_near"
REQUIRED = ("NAVRL_DEPTH_VALIDITY_WINDOW", "NAVRL_DEPTH_VALIDITY_DENSITY")
WINDOW_RANGE = (3, 31)
DENSITY_TOL = 1e-4             # counts and areas are exact small integers; this only absorbs float rounding of x*area
LOG_PREFIX = "[depth-validity] "
STATS_CALLS = 50
HEARTBEAT_EVERY = 5000
_STREAM = {"stream": None}
_INSTALLED = {}


def settings_from_env(environ=None):
    """Validated settings; refuses (ValueError) unless NAVRL_DEPTH_VALIDITY=blob_near with both parameters valid."""
    env = os.environ if environ is None else environ
    if env.get(ENV, "").strip() != RULE:
        raise ValueError("%s must be %s to use navrl_depth_validity, got %r" % (ENV, RULE, env.get(ENV)))
    missing = [k for k in REQUIRED if not env.get(k, "").strip()]
    if missing:
        raise ValueError("NAVRL_DEPTH_VALIDITY=blob_near needs %s" % ", ".join(missing))
    raw = env["NAVRL_DEPTH_VALIDITY_WINDOW"].strip()
    try:
        window = int(raw)
    except ValueError:
        raise ValueError("NAVRL_DEPTH_VALIDITY_WINDOW is not an integer: %r" % raw) from None
    if not (WINDOW_RANGE[0] <= window <= WINDOW_RANGE[1]) or window % 2 != 1:
        raise ValueError("NAVRL_DEPTH_VALIDITY_WINDOW=%d must be odd in [%d, %d]" % ((window,) + WINDOW_RANGE))
    try:
        density = float(env["NAVRL_DEPTH_VALIDITY_DENSITY"])
    except ValueError:
        raise ValueError("NAVRL_DEPTH_VALIDITY_DENSITY is not a number: %r"
                         % env["NAVRL_DEPTH_VALIDITY_DENSITY"]) from None
    if not 0.0 < density <= 1.0:       # also refuses nan
        raise ValueError("NAVRL_DEPTH_VALIDITY_DENSITY=%r outside (0, 1]" % density)
    return {"NAVRL_DEPTH_VALIDITY": RULE, "NAVRL_DEPTH_VALIDITY_WINDOW": window,
            "NAVRL_DEPTH_VALIDITY_DENSITY": density}


def dense_mask(invalid, window, density):
    """invalid [N, H, W] bool -> pixels of invalid whose WINDOW x WINDOW in-image invalid share is >= density."""
    import torch
    import torch.nn.functional as F

    x = invalid.to(torch.float32).unsqueeze(1)
    pad = window // 2
    area = float(window * window)
    count = F.avg_pool2d(x, window, stride=1, padding=pad, count_include_pad=True) * area     # padding adds zeros
    inside = F.avg_pool2d(torch.ones_like(x[:1]), window, stride=1, padding=pad, count_include_pad=True) * area
    dense = count.round() >= density * inside.round() - DENSITY_TOL
    return invalid & dense.squeeze(1)


def _apply_mask(depth, invalid, cosine, min_z_m, window, density):
    """Shared numerical kernel; callers own validation and same-frame mask provenance."""
    import torch

    near = dense_mask(invalid, window, density)
    surface = (min_z_m / cosine).to(depth.dtype).expand_as(depth)
    return torch.where(near, surface, depth), near


def apply_mask(depth, invalid, cosine, *, min_z_m, window, density):
    """Apply blob_near to an explicit measured mask; return (new ray-range depth, near mask).

    depth and invalid are [N, H, W], cosine is [H, W], and all tensors share a device. Depth must already be finite
    nonnegative ray range in metres, with isolated invalid pixels encoded as the caller's no-return range. The mask
    must come from that same delivered frame, before replacing invalid sensor values. No sentinel inference, noise,
    random draw, perception state, wrapper installation or input mutation occurs here. This function checks shapes
    and values; the sensor adapter remains responsible for timestamp alignment and calibrated pixel rays.
    """
    import torch

    if not isinstance(depth, torch.Tensor) or not depth.is_floating_point() or depth.ndim != 3 \
            or any(n <= 0 for n in depth.shape):
        raise ValueError("depth must be a nonempty floating tensor [N, H, W]")
    if not isinstance(invalid, torch.Tensor) or invalid.dtype != torch.bool or invalid.shape != depth.shape:
        raise ValueError("invalid must be a bool tensor with the same [N, H, W] shape as depth")
    if not isinstance(cosine, torch.Tensor) or not cosine.is_floating_point() \
            or cosine.shape != depth.shape[-2:]:
        raise ValueError("cosine must be a floating tensor [H, W] matching depth")
    if invalid.device != depth.device or cosine.device != depth.device:
        raise ValueError("depth, invalid and cosine must share a device")
    if not bool(torch.isfinite(depth).all()) or bool((depth < 0).any()):
        raise ValueError("depth ray ranges must be finite and nonnegative")
    if not bool(torch.isfinite(cosine).all()) or bool(((cosine <= 0) | (cosine > 1)).any()):
        raise ValueError("cosine must be finite and in (0, 1]")
    if isinstance(min_z_m, bool) or not isinstance(min_z_m, (int, float)) \
            or not math.isfinite(min_z_m) or min_z_m <= 0:
        raise ValueError("min_z_m must be finite and positive")
    if type(window) is not int or not WINDOW_RANGE[0] <= window <= WINDOW_RANGE[1] or window % 2 != 1:
        raise ValueError("window must be an odd integer in [%d, %d]" % WINDOW_RANGE)
    if isinstance(density, bool) or not isinstance(density, (int, float)) or not 0 < density <= 1:
        raise ValueError("density must be in (0, 1]")
    surface = (float(min_z_m) / cosine).to(depth.dtype)
    if not bool(torch.isfinite(surface).all()) or bool((surface <= 0).any()):
        raise ValueError("min_z_m / cosine must be finite and positive in depth's dtype")
    return _apply_mask(depth, invalid, cosine, float(min_z_m), window, float(density))


def apply(perception, depth, s):
    """The validity-ruled frame (a new tensor; `depth` is not modified)."""
    dm = getattr(type(perception), "_depth_model_settings", None)
    if dm is None:
        raise RuntimeError("the validity rule needs navrl_depth_model installed (outside this wrapper)")
    st = getattr(perception, "_depth_model_stats", None)
    calls = None if st is None else st.get("calls")
    last = getattr(perception, "_depth_validity_last_dm_call", 0)
    if calls != last + 1:
        raise RuntimeError("the depth model's invalid mask is not this frame's (depth-model calls %r, last %r): "
                           "install the validity rule before (inside) the depth model" % (calls, last))
    perception._depth_validity_last_dm_call = calls
    invalid = getattr(perception, "_depth_model_invalid", None)
    cos = getattr(perception, "_depth_model_cos", None)
    if invalid is None or cos is None or tuple(invalid.shape) != tuple(depth.shape) \
            or tuple(cos.shape) != tuple(depth.shape[-2:]):
        raise RuntimeError("depth model mask %r or cosine %r does not match depth %s"
                           % (None if invalid is None else tuple(invalid.shape),
                              None if cos is None else tuple(cos.shape), tuple(depth.shape)))
    minz = float(dm["NAVRL_DEPTH_MINZ_M"])
    if minz <= 0.0:
        raise RuntimeError("the validity rule needs NAVRL_DEPTH_MINZ_M > 0 (got %g)" % minz)
    # Keep the simulation's established guards and device execution; external-only value checks in apply_mask
    # would add new device synchronizations to every training frame.
    out, near = _apply_mask(depth, invalid, cos, minz, int(s["NAVRL_DEPTH_VALIDITY_WINDOW"]),
                            float(s["NAVRL_DEPTH_VALIDITY_DENSITY"]))
    perception._depth_validity_near = near
    _telemetry(perception, depth, invalid, near)
    return out


def log(event, **fields):
    stream = _STREAM["stream"] if _STREAM["stream"] is not None else sys.stderr
    stream.write(LOG_PREFIX + json.dumps(dict(event=event, **fields), sort_keys=True) + "\n")
    stream.flush()


def _telemetry(perception, depth, invalid, near):
    st = getattr(perception, "_depth_validity_stats", None)
    if st is None:
        st = perception._depth_validity_stats = {"calls": 0, "pixels": 0, "invalid": 0, "near": 0,
                                                 "sentinel_mismatch": 0}
    st["calls"] += 1
    if st["calls"] <= STATS_CALLS:
        far = float(perception.max_camera_range)
        st["pixels"] += int(depth.numel())
        st["invalid"] += int(invalid.sum())
        st["near"] += int(near.sum())
        st["sentinel_mismatch"] += int((invalid & (depth != far)).sum())   # the mask and the encoding must agree
        if st["calls"] in (1, STATS_CALLS):
            log("first" if st["calls"] == 1 else "summary", calls=st["calls"],
                frames=st["calls"] * int(perception.num_envs), pixels=st["pixels"], invalid_pixels=st["invalid"],
                near_pixels=st["near"], near_of_invalid=st["near"] / max(1, st["invalid"]),
                near_of_pixels=st["near"] / max(1, st["pixels"]), sentinel_mismatch=st["sentinel_mismatch"])
    elif st["calls"] % HEARTBEAT_EVERY == 0:
        log("heartbeat", calls=st["calls"])


def install_perception(module, s):
    """Wrap `module.NavRLPerceptionModule.observe`. Must run BEFORE the depth model's install (inner wrapper)."""
    cls = module.NavRLPerceptionModule
    if getattr(cls, "_depth_validity_settings", None) is not None:
        raise RuntimeError("depth validity already installed on %r" % cls)
    if getattr(cls, "_depth_model_settings", None) is not None:
        raise RuntimeError("install the depth validity rule before the depth model: it must be the inner wrapper to "
                           "run on the depth model's output")
    if getattr(cls, "_sensor_ablation_condition", None) is not None:
        raise RuntimeError("install the depth validity rule before the sensor ablation")
    s = dict(s)
    _STREAM["stream"] = sys.stderr             # bound now, outside rl_games' quiet construction
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
    cls._depth_validity_settings = s
    _INSTALLED["perception"] = (cls, orig_observe)
    log("install", settings=s)
    return cls


def install(s, perception_module=None):
    """Production install (tools/realveh_entry.py): import isaacgym first; call BEFORE navrl_depth_model.install."""
    if perception_module is None:
        from aerial_gym.task.navrl_task import navrl_perception as perception_module
    install_perception(perception_module, s)
    return {"rule": RULE, "settings": dict(s), "perception": perception_module.__file__,
            "near_surface": "optical z = NAVRL_DEPTH_MINZ_M, range = MinZ / perception._depth_model_cos",
            "invalid_source": "perception._depth_model_invalid (navrl_depth_model.apply, same frame)",
            "order": "installed before the depth model (inner wrapper), runs after it"}
