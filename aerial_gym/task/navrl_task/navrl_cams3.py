"""CAM-S3 opt-in rules (docs/plans/prereg_camonly_s3w_2026-11-27.md). navrl_perception.py and navrl_depth_model.py are
byte-pinned by the CAM-S2 flight-runtime contract (tools/camera_policy_contract_cams2.py TRAINING_SOURCE_PINS), so both
rules are installed at run time from this module and leave those files unchanged. Default OFF: nothing imports this
module unless tools/realveh_entry.py sees NAVRL_DEPTH_MEAS=1 or NAVRL_CAMS3_SECTOR_K.

1. Measured depth (training only; NAVRL_DEPTH_MEAS=1, needs NAVRL_DEPTH_MODEL=oakd). A wrapper of
   NavRLPerceptionModule.observe installed BEFORE navrl_depth_model (so it is the inner wrapper and runs on the frame the
   depth model produced). Per frame, on `depth` only:
   a. false near (RULES.md R5b, the OAK-D on a textureless grey wall, 2026-11-26): every valid return (not invalid in
      the depth model, not the renderer's no-hit) that the perception's own target segmenter does not score as target
      is shortened with probability P20(z): by a factor U(0.3, 0.6) with probability P40(z), else U(0.6, 0.8).
      P40 / P20 are the measured shares of valid pixels > 40 % / > 20 % too near at 1, 2 and 3 m
      (0.29 / 0.51, 0.024 / 0.18, 0.011 / 0.14), linear in z between the knots and constant outside them. A shortened
      z below the depth model's MinZ becomes invalid (the sensor reports nothing below MinZ).
   b. measured invalid masks (R5): each env holds a pointer into the mask library (tools/cams3_build_mask_library.py:
      1,849 frames of 6 recorded scenes at the camera grid); per frame the pointer advances one recorded frame with
      probability ADVANCE_P (the recording ran at 5 fps, control at 10 Hz) and jumps to a uniformly drawn frame with
      probability JUMP_P or at the end of its scene. Pixels invalid in the current mask, and the top TOP_ROWS rows,
      become invalid unless they are the renderer's no-hit (which already reads as no return).
   Invalid = perception.max_camera_range, the depth model's encoding; perception._depth_model_invalid is extended.
   Settings (all required when on): NAVRL_DEPTH_MEAS_LIBRARY (path), NAVRL_DEPTH_MEAS_SHA256, NAVRL_DEPTH_MEAS_SEED,
   NAVRL_DEPTH_MEAS_TOP_ROWS, NAVRL_DEPTH_MEAS_ADVANCE_P, NAVRL_DEPTH_MEAS_JUMP_P, NAVRL_DEPTH_MEAS_FALSE_NEAR (0/1).
   The NAVRL_DEPTH_ prefix makes the flight runtime drop them (TRAINING_ONLY_PREFIXES): the real sensor supplies them.

2. Camera sector reduction (training AND runtime; NAVRL_CAMS3_SECTOR_K=k, an integer >= 1). The camera-only obstacle
   scan of navrl_perception._fuse_static_and_extract_obstacles reads, for each 5-degree bin inside the camera FOV, the
   column minimum of ONE pixel column (camera_u, about every 23rd column of 312). A thin obstacle between two sampled
   columns is invisible (CPU geometry ~/motar_tmp/cams3_20261127/column_gap.py: a 0.25 m trunk at 3-6 m is seen 65 %
   of the time, at 6-10 m 37 %). This rule replaces the value at each sampled column by the k-th smallest range over
   EVERY pixel (all rows) of every column whose pinhole bearing atan((cx - u) / fx) lies in that bin
   [bearing - 2.5 deg, bearing + 2.5 deg), on the same target-carved frame (depth_no_target) the original reads, writes
   it into every column of that bin (the sampled column lies inside it), then passes the columns to the original
   camera_no_return_to_lidar_range unchanged. k > 1 ignores up to k - 1 isolated
   near pixels (false-near speckle, R27b); k = 1 is the plain sector minimum. It is installed by replacing the module
   global camera_no_return_to_lidar_range (looked up at call time); the replacement reads depth_no_target from its
   caller's frame, refuses any caller other than _fuse_static_and_extract_obstacles, and checks for the first
   VERIFY_CALLS calls that the column minima it was given are exactly depth_no_target.amin(dim=1).
"""
import functools
import hashlib
import inspect
import json
import math
import os
import sys

# torch is imported inside the functions (tools/realveh_entry.py parses settings before Isaac Gym is imported).

LOG_PREFIX = "[cams3] "
MEAS_ENV = "NAVRL_DEPTH_MEAS"
SECTOR_ENV = "NAVRL_CAMS3_SECTOR_K"
MEAS_KEYS = ("NAVRL_DEPTH_MEAS_LIBRARY", "NAVRL_DEPTH_MEAS_SHA256", "NAVRL_DEPTH_MEAS_SEED", "NAVRL_DEPTH_MEAS_TOP_ROWS",
             "NAVRL_DEPTH_MEAS_ADVANCE_P", "NAVRL_DEPTH_MEAS_JUMP_P", "NAVRL_DEPTH_MEAS_FALSE_NEAR")
FALSE_NEAR_Z = (1.0, 2.0, 3.0)
FALSE_NEAR_P40 = (0.29, 0.024, 0.011)
FALSE_NEAR_P20 = (0.51, 0.18, 0.14)
NO_HIT_TOL_M = 1e-3            # navrl_perception.camera_no_return_to_lidar_range / navrl_depth_model
SECTOR_K_MAX = 1000
VERIFY_CALLS = 50
STATS_CALLS = 200
CALLER = "_fuse_static_and_extract_obstacles"
_STREAM = {"stream": None}
_INSTALLED = {}


def log(event, **fields):
    stream = _STREAM["stream"] if _STREAM["stream"] is not None else sys.stderr
    stream.write(LOG_PREFIX + json.dumps(dict(event=event, **fields), sort_keys=True) + "\n")
    stream.flush()


def _file_sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def meas_settings_from_env(environ=None):
    """None when NAVRL_DEPTH_MEAS is unset/0 (refusing stray NAVRL_DEPTH_MEAS_* keys); else validated settings."""
    env = os.environ if environ is None else environ
    switch = env.get(MEAS_ENV, "").strip()
    if switch in ("", "0"):
        stray = sorted(k for k in MEAS_KEYS if env.get(k, "").strip())
        if stray:
            raise ValueError("%s set while %s is off" % (", ".join(stray), MEAS_ENV))
        return None
    if switch != "1":
        raise ValueError("%s must be unset, 0 or 1, got %r" % (MEAS_ENV, switch))
    missing = [k for k in MEAS_KEYS if not env.get(k, "").strip()]
    if missing:
        raise ValueError("%s=1 needs %s" % (MEAS_ENV, ", ".join(missing)))
    out = {"NAVRL_DEPTH_MEAS_LIBRARY": os.path.abspath(env["NAVRL_DEPTH_MEAS_LIBRARY"].strip())}
    sha = env["NAVRL_DEPTH_MEAS_SHA256"].strip().lower()
    if len(sha) != 64 or any(c not in "0123456789abcdef" for c in sha):
        raise ValueError("NAVRL_DEPTH_MEAS_SHA256 is not a SHA-256 hex digest")
    if not os.path.isfile(out["NAVRL_DEPTH_MEAS_LIBRARY"]):
        raise ValueError("NAVRL_DEPTH_MEAS_LIBRARY not found: %s" % out["NAVRL_DEPTH_MEAS_LIBRARY"])
    actual = _file_sha256(out["NAVRL_DEPTH_MEAS_LIBRARY"])
    if actual != sha:
        raise ValueError("NAVRL_DEPTH_MEAS_LIBRARY SHA-256 %s != NAVRL_DEPTH_MEAS_SHA256 %s" % (actual, sha))
    out["NAVRL_DEPTH_MEAS_SHA256"] = sha
    for key in ("NAVRL_DEPTH_MEAS_SEED", "NAVRL_DEPTH_MEAS_TOP_ROWS"):
        try:
            v = int(env[key])
        except ValueError:
            raise ValueError("%s is not an integer: %r" % (key, env[key])) from None
        if v < 0 or (key == "NAVRL_DEPTH_MEAS_TOP_ROWS" and v > 100):
            raise ValueError("%s out of range: %d" % (key, v))
        out[key] = v
    for key in ("NAVRL_DEPTH_MEAS_ADVANCE_P", "NAVRL_DEPTH_MEAS_JUMP_P"):
        try:
            v = float(env[key])
        except ValueError:
            raise ValueError("%s is not a number: %r" % (key, env[key])) from None
        if not (0.0 <= v <= 1.0) or v != v:
            raise ValueError("%s must be in [0, 1], got %r" % (key, v))
        out[key] = v
    fn = env["NAVRL_DEPTH_MEAS_FALSE_NEAR"].strip()
    if fn not in ("0", "1"):
        raise ValueError("NAVRL_DEPTH_MEAS_FALSE_NEAR must be 0 or 1, got %r" % fn)
    out["NAVRL_DEPTH_MEAS_FALSE_NEAR"] = int(fn)
    return out


def sector_settings_from_env(environ=None):
    """None when NAVRL_CAMS3_SECTOR_K is unset; else {"NAVRL_CAMS3_SECTOR_K": k}."""
    env = os.environ if environ is None else environ
    raw = env.get(SECTOR_ENV, "").strip()
    if not raw:
        return None
    try:
        k = int(raw)
    except ValueError:
        raise ValueError("%s is not an integer: %r" % (SECTOR_ENV, raw)) from None
    if not 1 <= k <= SECTOR_K_MAX:
        raise ValueError("%s must be in [1, %d], got %d" % (SECTOR_ENV, SECTOR_K_MAX, k))
    return {SECTOR_ENV: k}


# ----------------------------------------------------------------------------------------------- 1. measured depth
def false_near_probabilities(z):
    """(P40, P20) per pixel for z-depth tensor z: linear between the knots, constant outside."""
    import torch

    zc = z.clamp(FALSE_NEAR_Z[0], FALSE_NEAR_Z[-1])
    p40 = torch.empty_like(zc)
    p20 = torch.empty_like(zc)
    for (z0, z1), (a0, a1), (b0, b1) in zip(zip(FALSE_NEAR_Z[:-1], FALSE_NEAR_Z[1:]),
                                            zip(FALSE_NEAR_P40[:-1], FALSE_NEAR_P40[1:]),
                                            zip(FALSE_NEAR_P20[:-1], FALSE_NEAR_P20[1:])):
        sel = (zc >= z0) & (zc <= z1)
        t = (zc - z0) / (z1 - z0)
        p40 = torch.where(sel, a0 + (a1 - a0) * t, p40)
        p20 = torch.where(sel, b0 + (b1 - b0) * t, p20)
    return p40, p20


def load_library(path, device):
    """(masks bool [F, H, W], scene_end_of_frame int64 [F]) on `device`."""
    import numpy as np
    import torch

    with np.load(path) as z:
        masks = np.asarray(z["masks"], dtype=bool)
        starts = np.asarray(z["scene_start"], dtype=np.int64)
    if masks.ndim != 3 or starts[0] != 0 or starts[-1] != masks.shape[0] or np.any(np.diff(starts) <= 0):
        raise RuntimeError("mask library malformed: masks %s, scene_start %s" % (masks.shape, starts.tolist()))
    end = np.repeat(starts[1:], np.diff(starts))
    return torch.from_numpy(masks).to(device), torch.from_numpy(end).to(device)


class MeasuredDepth:
    """Per-perception state of rule 1 (pointer, generator, library)."""

    def __init__(self, perception, s, device):
        import torch

        self.s = s
        self.masks, self.scene_end = load_library(s["NAVRL_DEPTH_MEAS_LIBRARY"], device)
        h, w = int(perception.height), int(perception.width)
        if tuple(self.masks.shape[1:]) != (h, w):
            raise RuntimeError("mask library frames %s differ from the camera %s" % (tuple(self.masks.shape[1:]), (h, w)))
        self.gen = torch.Generator(device=device)
        self.gen.manual_seed(int(s["NAVRL_DEPTH_MEAS_SEED"]))
        n = int(perception.num_envs)
        self.frames = int(self.masks.shape[0])
        self.ptr = torch.randint(0, self.frames, (n,), generator=self.gen, device=device)
        self.top = torch.zeros((h, w), dtype=torch.bool, device=device)
        self.top[: int(s["NAVRL_DEPTH_MEAS_TOP_ROWS"])] = True
        self.stats = {"calls": 0, "eligible": 0, "mask_invalid": 0, "false_near": 0, "false_near_to_invalid": 0,
                      "target_like": 0, "jumps": 0, "advances": 0}

    def advance(self):
        import torch

        n = self.ptr.shape[0]
        dev = self.ptr.device
        step = (torch.rand(n, generator=self.gen, device=dev) < float(self.s["NAVRL_DEPTH_MEAS_ADVANCE_P"])).long()
        nxt = self.ptr + step
        jump = (torch.rand(n, generator=self.gen, device=dev) < float(self.s["NAVRL_DEPTH_MEAS_JUMP_P"]))
        jump = jump | (nxt >= self.scene_end[self.ptr])
        fresh = torch.randint(0, self.frames, (n,), generator=self.gen, device=dev)
        self.ptr = torch.where(jump, fresh, nxt)
        return step, jump


def apply_measured(perception, rgb, depth, s):
    """The measured-depth frame (a new tensor). Needs navrl_depth_model to have run on this frame."""
    import torch

    invalid = getattr(perception, "_depth_model_invalid", None)
    cos = getattr(perception, "_depth_model_cos", None)
    if invalid is None or cos is None or tuple(invalid.shape) != tuple(depth.shape):
        raise RuntimeError("measured depth needs navrl_depth_model's invalid mask for this frame (install order)")
    state = getattr(perception, "_cams3_meas", None)
    if state is None:
        state = perception._cams3_meas = MeasuredDepth(perception, s, depth.device)
    far = float(perception.max_camera_range)
    no_hit = (depth - float(perception.camera_obstacle_max_range)).abs() <= NO_HIT_TOL_M
    out = depth
    new_invalid = invalid.clone()
    fn_count = fn_inv = tl_count = 0
    if int(s["NAVRL_DEPTH_MEAS_FALSE_NEAR"]):
        with torch.no_grad():
            score = perception.segmenter(rgb, depth, far)
        target_like = score >= float(perception.pixel_threshold)
        eligible = ~invalid & ~no_hit & ~target_like
        z = depth * cos
        p40, p20 = false_near_probabilities(z)
        u = torch.rand(depth.shape, generator=state.gen, device=depth.device, dtype=depth.dtype)
        v = torch.rand(depth.shape, generator=state.gen, device=depth.device, dtype=depth.dtype)
        factor = torch.where(u < p40, 0.3 + 0.3 * v, torch.where(u < p20, 0.6 + 0.2 * v, torch.ones_like(v)))
        hit = eligible & (u < p20)
        out = torch.where(hit, depth * factor, depth)
        minz = float(perception._depth_model_settings["NAVRL_DEPTH_MINZ_M"])
        below = hit & (out * cos < minz)
        new_invalid = new_invalid | below
        if state.stats["calls"] < STATS_CALLS:
            fn_count, fn_inv, tl_count = int(hit.sum()), int(below.sum()), int((target_like & ~no_hit).sum())
    step, jump = state.advance()
    mask = state.masks[state.ptr] | state.top
    measured = mask & ~no_hit
    new_invalid = new_invalid | measured
    out = torch.where(new_invalid, torch.full_like(depth, far), out)
    perception._depth_model_invalid = new_invalid
    st = state.stats
    st["calls"] += 1
    if st["calls"] <= STATS_CALLS:
        st["eligible"] += int((~no_hit).sum())
        st["mask_invalid"] += int((measured & ~invalid).sum())
        st["false_near"] += fn_count
        st["false_near_to_invalid"] += fn_inv
        st["target_like"] += tl_count
        st["jumps"] += int(jump.sum())
        st["advances"] += int(step.sum())
        if st["calls"] in (1, STATS_CALLS):
            e = max(1, st["eligible"])
            frames = st["calls"] * int(depth.shape[0])
            log("meas_first" if st["calls"] == 1 else "meas_summary", calls=st["calls"], frames=frames,
                eligible_pixels=st["eligible"], mask_invalid_share=st["mask_invalid"] / e,
                false_near_share=st["false_near"] / e, false_near_to_invalid_share=st["false_near_to_invalid"] / e,
                target_like_pixels=st["target_like"], jump_rate=st["jumps"] / frames,
                advance_rate=st["advances"] / frames, library_frames=state.frames)
    return out


def install_measured(module, s):
    """Wrap observe; must run BEFORE navrl_depth_model's install (this is the inner wrapper)."""
    cls = module.NavRLPerceptionModule
    if getattr(cls, "_cams3_meas_settings", None) is not None:
        raise RuntimeError("measured depth already installed on %r" % cls)
    if getattr(cls, "_depth_model_settings", None) is not None:
        raise RuntimeError("install measured depth before navrl_depth_model (it must be the inner wrapper)")
    if getattr(cls, "_sensor_ablation_condition", None) is not None:
        raise RuntimeError("install measured depth before the sensor ablation")
    s = dict(s)
    _STREAM["stream"] = sys.stderr
    orig_observe = cls.observe
    sig = inspect.signature(orig_observe)
    if "depth" not in sig.parameters or "rgb" not in sig.parameters:
        raise RuntimeError("observe has no rgb/depth parameter: %s" % sig)

    @functools.wraps(orig_observe)
    def observe(self, *args, **kwargs):
        bound = sig.bind(self, *args, **kwargs)
        bound.arguments["depth"] = apply_measured(self, bound.arguments["rgb"], bound.arguments["depth"], s)
        return orig_observe(*bound.args, **bound.kwargs)

    cls.observe = observe
    cls._cams3_meas_settings = s
    _INSTALLED["measured"] = (cls, orig_observe)
    profile = ({"z": FALSE_NEAR_Z, "p40": FALSE_NEAR_P40, "p20": FALSE_NEAR_P20}
               if int(s["NAVRL_DEPTH_MEAS_FALSE_NEAR"]) else "off")
    log("install_measured", settings=s, false_near=profile)
    return {"settings": s, "module": __file__}


# ----------------------------------------------------------------------------------------------- 2. sector reduction
def sector_columns(perception, device):
    """(camera_u [B] long, columns [B, C] long, valid [B, C] bool, flat_columns [M] long, flat_bins [M] long,
    min_columns int) for the B bins inside the FOV, cached (no per-step host synchronisation)."""
    import torch

    cached = getattr(perception, "_cams3_sector_cols", None)
    if cached is not None and cached[0].device == torch.device(device):
        return cached
    angles = perception._lidar_angles.detach().to("cpu", torch.float64)
    hfov, width = float(perception.hfov), int(perception.width)
    inside = angles.abs() <= hfov * 0.5
    # the caller's own sampled column (navrl_perception._fuse_static_and_extract_obstacles), float32 as there
    cam_u = ((hfov * 0.5 - perception._lidar_angles.detach().cpu()) / hfov * (width - 1)).round().long().clamp(0, width - 1)
    half = math.pi / angles.numel()                  # half of one bin (2.5 deg at 72 bins)
    u = torch.arange(width, dtype=torch.float64)
    alpha = torch.atan((float(perception.cx) - u) / float(perception.fx))
    rows = []
    for b in torch.nonzero(inside).view(-1).tolist():
        th = float(angles[b])
        cols = torch.nonzero((alpha >= th - half) & (alpha < th + half)).view(-1)
        if cols.numel() == 0:
            raise RuntimeError("bin %.2f deg has no camera column" % math.degrees(th))
        rows.append((int(cam_u[b]), cols))
    for c, cols in rows:
        if c not in set(cols.tolist()):
            raise RuntimeError("sampled column %d lies outside its bin's columns" % c)
    cmax = max(c.numel() for _, c in rows)
    columns = torch.zeros((len(rows), cmax), dtype=torch.long)
    valid = torch.zeros((len(rows), cmax), dtype=torch.bool)
    for i, (_, c) in enumerate(rows):
        columns[i, : c.numel()] = c
        valid[i, : c.numel()] = True
    bins = torch.arange(len(rows)).view(-1, 1).expand_as(columns)[valid]
    out = (torch.tensor([c for c, _ in rows], dtype=torch.long, device=device), columns.to(device), valid.to(device),
           columns[valid].to(device), bins.to(device), int(valid.sum(dim=1).min()))
    perception._cams3_sector_cols = out
    return out


def sector_values(perception, depth_no_target, k):
    """[N, B] k-th smallest range over every pixel of each in-FOV bin's columns, and the sampled columns [B]."""
    import torch

    cam_u, columns, valid, _, _, min_cols = sector_columns(perception, depth_no_target.device)
    n, h, _ = depth_no_target.shape
    b, c = columns.shape
    vals = depth_no_target[:, :, columns.reshape(-1)].reshape(n, h, b, c)
    vals = torch.where(valid.view(1, 1, b, c), vals, torch.full_like(vals, float("inf")))
    vals = vals.permute(0, 2, 1, 3).reshape(n, b, h * c)
    kk = min(int(k), h * min_cols)
    return vals.kthvalue(kk, dim=2).values, cam_u


def make_sector_function(original, k):
    state = {"calls": 0}

    def camera_no_return_to_lidar_range(camera_col_min, camera_max_range, lidar_max_range):
        import torch

        frame = sys._getframe(1)
        try:
            if frame.f_code.co_name != CALLER:
                raise RuntimeError("cams3 sector rule called from %s, only %s is supported"
                                   % (frame.f_code.co_name, CALLER))
            loc = frame.f_locals
            perception, dnt = loc.get("self"), loc.get("depth_no_target")
        finally:
            del frame
        if dnt is None or perception is None:
            raise RuntimeError("cams3 sector rule: caller has no depth_no_target")
        if tuple(camera_col_min.shape) != (dnt.shape[0], dnt.shape[2]):
            raise RuntimeError("cams3 sector rule: column minima %s vs frame %s" % (tuple(camera_col_min.shape),
                                                                                  tuple(dnt.shape)))
        state["calls"] += 1
        if state["calls"] <= VERIFY_CALLS and not torch.equal(dnt.amin(dim=1), camera_col_min):
            raise RuntimeError("cams3 sector rule: column minima are not depth_no_target.amin(dim=1)")
        vals, cam_u = sector_values(perception, dnt, k)
        flat_columns, flat_bins = perception._cams3_sector_cols[3:5]
        cols = camera_col_min.clone()
        cols[:, flat_columns] = vals[:, flat_bins].to(cols.dtype)  # every column of the bin, so any sampled one reads it
        perception._cams3_sector_last = vals
        if state["calls"] == 1:
            log("sector_first", k=int(k), bins=int(cam_u.numel()), sampled_columns=cam_u.tolist(),
                columns_per_bin=[int(x) for x in perception._cams3_sector_cols[2].sum(dim=1).tolist()])
        return original(cols, camera_max_range, lidar_max_range)

    camera_no_return_to_lidar_range.cams3_sector_k = int(k)
    camera_no_return_to_lidar_range.cams3_original = original
    return camera_no_return_to_lidar_range


def install_sector(module, s):
    """Replace module.camera_no_return_to_lidar_range (navrl_perception) by the sector rule with k."""
    current = module.camera_no_return_to_lidar_range
    if getattr(current, "cams3_sector_k", None) is not None:
        raise RuntimeError("cams3 sector rule already installed")
    if getattr(module.NavRLPerceptionModule, CALLER, None) is None:
        raise RuntimeError("navrl_perception has no %s" % CALLER)
    _STREAM["stream"] = sys.stderr
    k = int(s[SECTOR_ENV])
    module.camera_no_return_to_lidar_range = make_sector_function(current, k)
    _INSTALLED["sector"] = (module, current)
    log("install_sector", settings=dict(s))
    return {"settings": dict(s), "module": __file__}


def uninstall_all():
    """Tests only: restore everything this module replaced."""
    if "sector" in _INSTALLED:
        module, original = _INSTALLED.pop("sector")
        module.camera_no_return_to_lidar_range = original
    if "measured" in _INSTALLED:
        cls, orig = _INSTALLED.pop("measured")
        cls.observe = orig
        cls._cams3_meas_settings = None
