"""Metadata-only flight contract for the CAM-S2 camera-only policies (CAM-S2a; CAM-S2b, 60 deg/s yaw ceiling).

Runtime v1 (2026-11-24). Separate from the CAM-K2 v3 contract (camera_policy_contract.py, its PINS and its fixed
160x90 / blob_near schema): a CAM-S2 contract is derived from a training RECEIPT written by tools/realveh_entry.py
(key env_navrl = the exact training environment) and a checkpoint that the receipt's run directory lists by SHA-256.
Nothing here deserializes the checkpoint or starts a simulator, camera or flight link.

What the contract fixes, each read from the receipt (never typed by hand):
  * camera 312x234 at NAVRL_CAMTRAIN_HFOV_DEG x NAVRL_CAMTRAIN_VFOV_DEG (66.7 x 52.6), detection at the camera
    resolution (no decoupling), mount = receipt cams2.config_after.camera_translation ([0.08, 0, 0.03]);
  * depth: OAK-D model MinZ/MaxZ (0.315 / 20 m); NO blob_near (NAVRL_DEPTH_VALIDITY unset): every invalid return is
    encoded as perception.max_camera_range = "no return" (navrl_depth_model.py step 3). The runtime reduces each
    trained pixel to the NEAREST valid sample that lands on it (min over valid); a pixel with no valid sample is
    no-return (camera_policy_runtime.project_rgbd_min_valid);
  * height observation z / (3 x NAVRL_CAMS2_FLIGHT_ALTITUDE_M) (navrl_perception.py:2337; review r1 F2);
  * action scale NAVRL_MAX_VELOCITY, yaw-rate scale NAVRL_YAW_RATE_MAX (3.0 = CAM-S2a, 1.0472 = CAM-S2b);
  * trained action delay k ~ U{NAVRL_CAMS2_DELAY_MIN..MAX} (1..2 control steps) and the previous-action channel as
    trained (review r1 F1): xy = the DELAYED command a_{t-k} after the riskcap governor and the camsafe blind cap,
    z and yaw = the CURRENT raw action a_t. The runtime cannot know the vehicle's k, so the contract carries one
    explicit operator choice previous_action_delay_steps (k_hat, inside the trained support), see
    PREVIOUS_ACTION_NOTE;
  * the camsafe blind-direction cap NAVRL_CAMSAFE_BLIND_MPS (1 m/s outside +-hfov/2, after the governor);
  * the capture-age bound (NAVRL_ACTION_DELAY_STEPS_MAX + 1) x 0.1 s + 0.05 s = 0.35 s for 2 steps
    (camera_policy_contract.capture_age_bound_s, which accepts delays up to 2 for this lineage only);
  * the flight environment = the training environment minus training-only keys (TRAINING_ONLY_*): the P9 detection-
    error injector, the simulated depth noise, vehicle/gain randomisation and the receipt path. A contract whose
    runtime_env carries any of them is refused (the P9 injector would otherwise run inside flight perception:
    navrl_perception.py builds it from NAVRL_P9_ERROR_MODEL regardless of `training`).
Timing (review r1 P1-2). Three numbers relate the trained delay to flight:
  * k (training): the command applied during [t + k dt, t + (k+1) dt] is the action computed from the observation at t,
    k ~ U{1, 2}, dt = 0.1 s;
  * k_hat (contract action.previous_action_delay_steps): the operator's estimate of the vehicle's k, i.e. the measured
    capture -> PX4 setpoint latency L in control periods, round(L / dt). It only selects which past action the
    previous-action channel shows; it must lie in the trained support;
  * the capture-age bound 0.35 s = (DELAY_MAX + 1) x dt + 0.05 s: a command may still be USED while its capture is at
    most this old (it is used until the next proposal, one period later). The runtime therefore refuses a step whose
    capture-to-proposal age already implies more than DELAY_MAX steps, floor(age / dt + 0.5) > 2, i.e. age >= 0.25 s
    (camera_policy_runtime.CameraPolicyRuntime.latency), and reports the implied steps against k_hat on every step.
    The runtime sees the latency only up to its proposal; the gateway, MAVROS and PX4 add to it. The real L is
    measured on the vehicle (NOT_MEASURED here).
Open items (review r1; also ~/motar_tmp/runtime_v1_20261124/NOTES.md), not fixed by runtime v1:
  * P1-3: training obstacle depth is a 40x24 ray render bilinearly upsampled to 312x234 and clamped at 10 m
    (navrl_detector.py:927-938); real min-valid depth is sharper, and textureless/IR-off invalid regions read as free
    space (training saw 2 % iid holes). The parity covers identical depth images only. User decision: emulate the
    coarse render on real depth, or accept and measure the gap on recordings;
  * P2-2: the CPU parity's training-side governor is the runtime's replica; only the GPU check covers the real one;
  * P2-4: training applies the governor and blind cap at application time on the newest frame; flight applies them
    at capture time, k steps earlier. The previous-action channel follows the trained formula, not what the vehicle
    executed; a PHASE 7 filter should re-apply the cap on the newest frame;
  * P2-5: the height feature (sigma about 6 cm at 1.5 m) assumes estimator z = height above the floor; the operator
    assertion is enforced only when a floor/ceiling filter is on;
  * P2-6: the specified GPU parity check must also require blind-cap/governor events, handle episode resets, state a
    TF32-aware tolerance and include non-identity attitude before it is preregistered;
  * P3-4: training-time byte pins omit navrl_task.py, the task config, speed_governor.py and navrl_detector.py
    (identical to the training head 22ed16f on 2026-11-24, verified by the reviewer).
The deployment floor/ceiling filters and the operator estimator-height assertion are the CAM-K2 ones
(camera_policy_contract.GROUND_DEFAULT etc.; default: floor removal ON, refusing every frame until asserted). The outdoor
profile is NOT supported for CAM-S2 in runtime v1 (refused): its sky rule exists to undo blob_near, which CAM-S2 has
not, and its ground estimate has not been reviewed against the no-return encoding.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import camera_policy_contract as K

ROOT = K.ROOT
FORMAT = "motar-camera-policy-cams2-v1"
LINEAGE = "CAM-S2"
# Variant by the trained yaw-rate ceiling (receipt NAVRL_YAW_RATE_MAX, exact string).
VARIANTS = {"3.0": "CAM-S2a", "1.0472": "CAM-S2b"}
CAMERA_SIZE = (312, 234)
# Delays this lineage's flight timing accepts (camera_policy_contract.trained_action_delay_steps dispatches here). The
# CAM-K2 set SUPPORTED_TRAINED_DELAY_STEPS stays (0,): only a receipt written by the delay code itself
# (navrl_cams2.settings_from_env refuses NAVRL_ACTION_DELAY_STEPS_MAX != NAVRL_CAMS2_DELAY_MAX) may widen the bound.
SUPPORTED_TRAINED_DELAY_STEPS = (0, 1, 2)
# Exact training settings the runtime implements; anything else is a different policy contract.
REQUIRED_ENV = {"NAVRL_SENSOR_ABLATION": "no_lidar", "NAVRL_LIDAR_HBEAMS": "72", "NAVRL_LIDAR_VBEAMS": "4",
                "NAVRL_LIDAR_RANGE": "12", "NAVRL_MAX_OBSTACLES": "8", "NAVRL_CORRIDOR_TOKENS": "0",
                "NAVRL_MAX_VELOCITY": "2.5", "NAVRL_SPEED_GOVERNOR": "riskcap", "NAVRL_PERCEPTION_PERTURB": "0",
                "NAVRL_ACTION_POLICY": "squashed_gaussian", "NAVRL_DEPTH_MODEL": "oakd", "NAVRL_CAMS2": "1",
                "NAVRL_CAMSAFE": "1", "NAVRL_VISION": "1", "NAVRL_PERCEPTION": "1",
                "NAVRL_CAMERA_WIDTH": str(CAMERA_SIZE[0]), "NAVRL_CAMERA_HEIGHT": str(CAMERA_SIZE[1]),
                "NAVRL_DETECT_WIDTH": str(CAMERA_SIZE[0]), "NAVRL_DETECT_HEIGHT": str(CAMERA_SIZE[1])}
# Training settings that would make the trained observation differ from what this runtime builds (blob_near, the
# evaluation-only yaw clip, the CAM-S2a-E recorders): a receipt with any of them set is refused.
FORBIDDEN_IN_TRAINING = ("NAVRL_DEPTH_VALIDITY", "NAVRL_DEPTH_VALIDITY_WINDOW", "NAVRL_DEPTH_VALIDITY_DENSITY",
                         "NAVRL_YAWCLIP", "NAVRL_CAMS2DIAG", "NAVRL_CAMS2_CONTACTCODE", "NAVRL_DETECTOR_CHECKPOINT")
# Training-only keys: removed from the flight environment and refused if present in a contract's runtime_env.
TRAINING_ONLY_PREFIXES = ("NAVRL_P9_", "NAVRL_DEPTH_", "NAVRL_VEH_RAND", "NAVRL_GAIN_RAND", "NAVRL_DETNOISE_")
TRAINING_ONLY_KEYS = ("NAVRL_REALVEH_RECEIPT", "NAVRL_DETECTION_DROPOUT", "NAVRL_RGB_NOISE_STD")
# Sources a CAM-S2 contract binds: the CAM-K2 set plus this module and the two training modules the runtime imports.
SOURCE_PATHS = K.SOURCE_PATHS + ("tools/camera_policy_contract_cams2.py",
                                 "aerial_gym/task/navrl_task/navrl_cams2.py",
                                 "aerial_gym/task/navrl_task/navrl_camsafe.py")
# Training-time module hashes the receipt records, which the current tree must reproduce byte for byte.
TRAINING_SOURCE_PINS = (("sha256", "navrl_perception", "aerial_gym/task/navrl_task/navrl_perception.py"),
                        ("sha256", "sensor_ablation", "aerial_gym/task/navrl_task/navrl_sensor_ablation.py"),
                        ("sha256", "depth_model", "aerial_gym/task/navrl_task/navrl_depth_model.py"),
                        (None, "cams2_sha256", "aerial_gym/task/navrl_task/navrl_cams2.py"),
                        (None, "camsafe_sha256", "aerial_gym/task/navrl_task/navrl_camsafe.py"))
# The training run directory's own listings, or the SHA256SUMS of a handoff bundle that carries the receipt copy.
CHECKPOINT_LISTS = ("checkpoint_sha256_all.txt", "checkpoint_sha256.txt", "SHA256SUMS")
NEAR_CONSTANT_VAR = 1e-6
PREVIOUS_ACTION_NOTE = (
    "Training (navrl_task.py:5053-5057 with :4967-4970, navrl_cams2.py:129-133, navrl_camsafe.py:154-158): "
    "prev[0:2] = clamp(capped(governed(a_{t-k} xy * v_max)) / v_max), prev[2:4] = clamp(a_t z, yaw), k ~ U{min..max} "
    "per episode, zero before the episode's first action. The runtime keeps the history of its own clamped actions and "
    "feeds the same formula with the fixed k_hat = previous_action_delay_steps, the governor and blind cap evaluated on "
    "the current frame's perception exactly as training evaluates them at the step that applies a_{t-k}. The published "
    "command is the CURRENT governed and capped a_t: the vehicle's own capture-to-actuation latency plays the role of "
    "the simulated delay, so k_hat should be the vehicle's measured latency in control periods, rounded, "
    "(capture stamp -> PX4 velocity setpoint applied) / 0.1 s, inside the trained support. Limit (review r1 P2-4): "
    "training governs and caps the delayed action on the frame at APPLICATION time; flight governs and caps at capture "
    "time, so the channel follows the trained formula, not the command the vehicle executed.")


def _receipt(path):
    path = Path(path).resolve()
    data = json.loads(path.read_text())
    env = data.get("env_navrl") if isinstance(data, dict) else None
    if (not isinstance(env, dict) or any(not isinstance(k, str) or not k.startswith("NAVRL_") or not isinstance(v, str)
                                         for k, v in env.items())):
        raise ValueError("training receipt env_navrl is malformed")
    return path, data, env


def _training_only(key):
    return key in TRAINING_ONLY_KEYS or key.startswith(TRAINING_ONLY_PREFIXES)


def flight_environment(training_env):
    """The training environment without its training-only keys (sorted copy)."""
    return {k: v for k, v in sorted(training_env.items()) if not _training_only(k)}


def _float(env, key):
    try:
        value = float(env[key])
    except (KeyError, ValueError):
        raise ValueError("training receipt lacks a numeric %s" % key) from None
    if not math.isfinite(value):
        raise ValueError("%s is not finite" % key)
    return value


def _private(name, path):
    """The settings parser of a stdlib-only training module, loaded by path and not registered in sys.modules (as
    tools/realveh_entry.py does), so the training's own validation (ranges, delay record) is applied verbatim."""
    import importlib.util
    spec = importlib.util.spec_from_file_location(name, str(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _checkpoint_listed(receipt_path, digest):
    for name in CHECKPOINT_LISTS:
        listing = receipt_path.parent / name
        if listing.is_file():
            rows = [line.split() for line in listing.read_text().splitlines() if line.strip()]
            if any(row and row[0] == digest for row in rows):
                return str(listing)
    return None


def _metadata(receipt_path, checkpoint_path, obs_norm_path, previous_action_delay_steps, root=ROOT):
    """Every lineage field, recomputed from the receipt and the bound files (validate_contract compares with it)."""
    receipt_path, receipt, env = _receipt(receipt_path)
    bad = sorted(k for k, v in REQUIRED_ENV.items() if env.get(k) != v)
    if bad:
        raise ValueError("training environment differs from the CAM-S2 runtime contract: %s" % ", ".join(bad))
    present = sorted(k for k in FORBIDDEN_IN_TRAINING if env.get(k, "").strip())
    if present:
        raise ValueError("training used settings this runtime does not implement: %s" % ", ".join(present))
    variant = VARIANTS.get(env.get("NAVRL_YAW_RATE_MAX"))
    if variant is None:
        raise ValueError("NAVRL_YAW_RATE_MAX=%r is neither CAM-S2a (3.0) nor CAM-S2b (1.0472)"
                         % env.get("NAVRL_YAW_RATE_MAX"))
    cams2 = _private("cams2_contract_settings", Path(root) / "aerial_gym/task/navrl_task/navrl_cams2.py").settings_from_env(env)
    camsafe = _private("camsafe_contract_settings",
                       Path(root) / "aerial_gym/task/navrl_task/navrl_camsafe.py").settings_from_env(env)
    if cams2 is None or camsafe is None:
        raise ValueError("training receipt has NAVRL_CAMS2 or NAVRL_CAMSAFE off")
    record = receipt.get("cams2") if isinstance(receipt.get("cams2"), dict) else {}
    after = record.get("config_after") if isinstance(record.get("config_after"), dict) else {}
    translation = after.get("camera_translation")
    altitude = float(cams2["NAVRL_CAMS2_FLIGHT_ALTITUDE_M"])
    if (not isinstance(translation, list) or len(translation) != 3
            or translation[0] != cams2["NAVRL_CAMS2_CAMERA_X_M"] or after.get("flight_altitude") != altitude):
        raise ValueError("receipt cams2.config_after disagrees with its NAVRL_CAMS2_* settings")
    if record.get("delay_steps") != [cams2["NAVRL_CAMS2_DELAY_MIN"], cams2["NAVRL_CAMS2_DELAY_MAX"]]:
        raise ValueError("receipt cams2.delay_steps disagrees with its settings")
    overrides = receipt.get("overrides_after") or {}
    hfov, vfov = _float(env, "NAVRL_CAMTRAIN_HFOV_DEG"), _float(env, "NAVRL_CAMTRAIN_VFOV_DEG")
    if overrides.get("detector_hfov_deg") != hfov or overrides.get("detector_vfov_deg") != vfov:
        raise ValueError("receipt overrides_after disagrees with the trained FOV")
    depth_record = receipt.get("depth_model") or {}
    if depth_record.get("invalid_encoding") != "perception.max_camera_range":
        raise ValueError("receipt depth model does not encode invalid depth as no return")
    minz, maxz = _float(env, "NAVRL_DEPTH_MINZ_M"), _float(env, "NAVRL_DEPTH_MAXZ_M")
    pins = []
    for group, key, rel in TRAINING_SOURCE_PINS:
        recorded = (receipt.get(group) or {}).get(key) if group else receipt.get(key)
        if recorded != K.sha256_file(Path(root) / rel):
            pins.append(rel)
    if pins:
        raise ValueError("current source differs from the training-time bytes the receipt pins: %s" % ", ".join(pins))
    dmin, dmax = int(cams2["NAVRL_CAMS2_DELAY_MIN"]), int(cams2["NAVRL_CAMS2_DELAY_MAX"])
    if env.get(K.TRAINED_DELAY_ENV) != str(dmax) or dmax not in SUPPORTED_TRAINED_DELAY_STEPS:
        raise ValueError("%s must equal NAVRL_CAMS2_DELAY_MAX within %s" % (K.TRAINED_DELAY_ENV,
                                                                           SUPPORTED_TRAINED_DELAY_STEPS))
    k_hat = previous_action_delay_steps
    if isinstance(k_hat, bool) or not isinstance(k_hat, int) or not dmin <= k_hat <= dmax or k_hat < 1:
        raise ValueError("previous_action_delay_steps must be an explicit integer in the trained support [%d, %d] "
                         "(and >= 1)" % (max(dmin, 1), dmax))
    checkpoint_path, obs_norm_path = Path(checkpoint_path).resolve(), Path(obs_norm_path).resolve()
    ck_sha = K.sha256_file(checkpoint_path)
    listing = _checkpoint_listed(receipt_path, ck_sha)
    if listing is None:
        raise ValueError("checkpoint SHA-256 is not listed in the receipt run directory (%s)" % ", ".join(CHECKPOINT_LISTS))
    width, height = CAMERA_SIZE
    max_velocity, yaw_rate_max = _float(env, "NAVRL_MAX_VELOCITY"), _float(env, "NAVRL_YAW_RATE_MAX")
    return {
        "format": FORMAT, "lineage": LINEAGE, "variant": variant,
        "checkpoint": {"path": str(checkpoint_path), "sha256": ck_sha, "listing": listing,
                       "obs_norm_path": str(obs_norm_path), "obs_norm_sha256": K.sha256_file(obs_norm_path),
                       "near_constant_var": NEAR_CONSTANT_VAR},
        "training_record": {"path": str(receipt_path), "sha256": K.sha256_file(receipt_path)},
        "runtime_env": flight_environment(env), "environment_inheritance": "forbidden",
        "camera": {"width": width, "height": height, "hfov_deg": hfov, "vfov_deg": vfov,
                   "fx": width / (2.0 * math.tan(math.radians(hfov / 2.0))),
                   "fy": height / (2.0 * math.tan(math.radians(vfov / 2.0))),
                   "cx": (width - 1) / 2.0, "cy": (height - 1) / 2.0,
                   "translation_m": [float(x) for x in translation], "rgb_depth_alignment": "required"},
        "observation": {"dim": 898, "lidar_storage_shape": [1, 288], "no_lidar": True, "history_steps": 5,
                        "history_interval_s": 0.5, "control_dt_s": 0.1, "flight_altitude_m": altitude,
                        "height_feature": "z_enu_m / (3 x flight_altitude_m)"},
        "action": {"dim": 4, "max_velocity_mps": max_velocity, "yaw_rate_max_rps": yaw_rate_max,
                   "trained_delay_steps": [dmin, dmax], "previous_action_delay_steps": k_hat,
                   "previous_action": "delayed_governed_blind_capped_xy_then_current_raw_z_yaw",
                   "previous_action_note": PREVIOUS_ACTION_NOTE,
                   "blind_cap_mps": float(camsafe["NAVRL_CAMSAFE_BLIND_MPS"]), "blind_half_fov_deg": hfov / 2.0,
                   "published_command": "current_governed_blind_capped_xy_and_current_yaw_rate"},
        "depth": {"input": "optical_z_m", "output": "ray_range_m", "invalid_mask": "explicit_same_frame",
                  "minz_m": minz, "maxz_m": maxz, "validity": "none", "invalid_encoding": "no_return",
                  "source_reduction": "min_over_valid_samples_per_trained_pixel",
                  "synthetic_noise_on_real_input": False},
        "detector": {"kind": "bootstrap_red", "scope": "red-target appearance only; physical NOT_TESTED",
                     "training_only_error_injection": "P9 removed from the flight environment"},
        "state": {"position_velocity_frame": "world_enu", "vehicle_quaternion": "xyzw",
                  "yaw_rate_unit": "rad/s", "target_truth_required": False},
        "qualification": {"cpu": "NOT_TESTED", "gazebo": "NOT_TESTED", "hardware": "NOT_TESTED"},
        "ground": dict(K.GROUND_DEFAULT), "ceiling": dict(K.CEILING_DEFAULT), "estimator_height": None,
    }


# Review r1 P2-3: the checkpoint's own training snapshot (state["env_state"], written by the task) must agree with the
# receipt the contract was built from; a bundle that lists a different lineage's checkpoint is refused. Pairs of
# (env_state key, receipt env key, kind); "float" compares numerically, "int" as integers, "str" exactly.
CHECKPOINT_ENV_PAIRS = (
    ("cfg_yaw_rate_max", "NAVRL_YAW_RATE_MAX", "float"), ("cfg_max_velocity", "NAVRL_MAX_VELOCITY", "float"),
    ("cfg_detect_width", "NAVRL_DETECT_WIDTH", "int"), ("cfg_detect_height", "NAVRL_DETECT_HEIGHT", "int"),
    ("cfg_lidar_max_range", "NAVRL_LIDAR_RANGE", "float"), ("cfg_lidar_hbeams", "NAVRL_LIDAR_HBEAMS", "int"),
    ("cfg_lidar_vbeams", "NAVRL_LIDAR_VBEAMS", "int"), ("cfg_max_obstacles", "NAVRL_MAX_OBSTACLES", "int"),
    ("cfg_detector_min_pixels", "NAVRL_DETECTOR_MIN_PIXELS", "int"),
    ("cfg_detector_threshold", "NAVRL_DETECTOR_THRESHOLD", "float"),
    ("cfg_speed_governor_mode", "NAVRL_SPEED_GOVERNOR", "str"),
    ("cfg_speed_governor_fixed_mps", "NAVRL_SPEED_GOVERNOR_FIXED_MPS", "float"),
    ("cfg_speed_governor_free_mps", "NAVRL_SPEED_GOVERNOR_FREE_MPS", "float"),
    ("cfg_speed_governor_half_width_m", "NAVRL_SPEED_GOVERNOR_HALF_WIDTH_M", "float"),
    ("cfg_speed_governor_margin_m", "NAVRL_SPEED_GOVERNOR_MARGIN_M", "float"),
    ("cfg_speed_governor_slow_m", "NAVRL_SPEED_GOVERNOR_SLOW_M", "float"),
    ("cfg_speed_governor_release_m", "NAVRL_SPEED_GOVERNOR_RELEASE_M", "float"),
    ("cfg_speed_governor_ttc_s", "NAVRL_SPEED_GOVERNOR_TTC_S", "float"),
    ("cfg_speed_governor_brake_mps2", "NAVRL_SPEED_GOVERNOR_BRAKE_MPS2", "float"),
    ("cfg_speed_governor_reaction_s", "NAVRL_SPEED_GOVERNOR_REACTION_S", "float"),
    ("cfg_action_policy", "NAVRL_ACTION_POLICY", "str"), ("cfg_action_std", "NAVRL_ACTION_STD", "str"),
    ("cfg_action_mu_scale", "NAVRL_ACTION_MU_SCALE", "str"),
    ("cfg_p9_empirical_error_sha256", "NAVRL_P9_EXPECTED_SHA256", "str"),
    ("cfg_training_seed", "NAVRL_SEED", "int"),
)
# Constant expectations on the snapshot (no receipt key; the runtime implements exactly these).
CHECKPOINT_ENV_FIXED = {"cfg_detector_checkpoint_name": "", "cfg_perception_perturb": False,
                        "cfg_detector_max_range": 20.0, "cfg_rl_step_dt_s": 0.1, "cfg_corridor_tokens": 0,
                        "cfg_speed_governor_yaw_cap_radps": 0.0, "cfg_search_state": "off"}


def checkpoint_env_problems(env_state, receipt_env):
    """Pure: the disagreements between a checkpoint's env_state and the training receipt's env_navrl ([] = agree).
    A missing key on either side counts as a disagreement."""
    if not isinstance(env_state, dict) or not isinstance(receipt_env, dict):
        return ["checkpoint env_state or receipt env is missing"]
    problems = []
    for key, env_key, kind in CHECKPOINT_ENV_PAIRS:
        have, want = env_state.get(key), receipt_env.get(env_key)
        try:
            if kind == "float":
                same = (not isinstance(have, bool) and isinstance(have, (int, float))
                        and math.isclose(float(have), float(want), rel_tol=1e-9, abs_tol=1e-12))
            elif kind == "int":
                same = not isinstance(have, bool) and int(have) == int(want) and float(have) == int(have)
            else:
                same = isinstance(have, str) and have == want
        except (TypeError, ValueError):
            same = False
        if not same:
            problems.append("checkpoint %s=%r differs from receipt %s=%r" % (key, have, env_key, want))
    for key, want in CHECKPOINT_ENV_FIXED.items():
        have = env_state.get(key)
        if type(have) is not type(want) and not (isinstance(want, float) and type(have) is int) or have != want:
            problems.append("checkpoint %s=%r, the runtime implements %r" % (key, have, want))
    return problems


def is_cams2(contract):
    return isinstance(contract, dict) and contract.get("lineage") == LINEAGE


def build_contract(receipt_path, checkpoint_path, obs_norm_path, *, previous_action_delay_steps, root=ROOT,
                   ground=None, ceiling=None, estimator_height=None):
    contract = _metadata(receipt_path, checkpoint_path, obs_norm_path, previous_action_delay_steps, root)
    if ground is not None:
        contract["ground"] = dict(ground)
    if ceiling is not None:
        contract["ceiling"] = dict(ceiling)
    if estimator_height is not None:
        contract["estimator_height"] = json.loads(json.dumps(estimator_height))
    contract["sources"] = {rel: K.sha256_file(Path(root) / rel) for rel in SOURCE_PATHS}
    problems = validate_contract(contract, root=root)
    if problems:
        raise ValueError("invalid CAM-S2 camera policy contract: " + "; ".join(problems))
    return contract


def _same(a, b):
    try:
        return json.dumps(a, sort_keys=True, allow_nan=False) == json.dumps(b, sort_keys=True)
    except (TypeError, ValueError):
        return False


def validate_contract(contract, verify_files=True, *, root=ROOT):
    """All problems of a CAM-S2 contract; [] accepts it. The receipt is always re-read (its SHA-256 checked), so
    environment additions, training-only keys and edited lineage fields cannot reach the flight runtime."""
    if not is_cams2(contract):
        return ["not a CAM-S2 camera policy contract"]
    problems = []
    env = contract.get("runtime_env")
    if isinstance(env, dict):
        leaked = sorted(k for k in env if _training_only(k))
        if leaked:
            problems.append("training-only keys in the flight environment: %s" % ", ".join(leaked))
    record, ck, action = (contract.get(k) if isinstance(contract.get(k), dict) else {}
                          for k in ("training_record", "checkpoint", "action"))
    try:
        if K.sha256_file(record.get("path", "")) != record.get("sha256"):
            return problems + ["training receipt SHA-256 differs from the contract"]
        # Always recomputed (verify_files only skips the runtime-source and evidence hashes, as in CAM-K2).
        expected = _metadata(record["path"], ck.get("path", ""), ck.get("obs_norm_path", ""),
                             action.get("previous_action_delay_steps"), root)
    except (OSError, ValueError, KeyError, TypeError) as exc:
        return problems + ["CAM-S2 training provenance: %s" % exc]
    if any(key in contract for key in K.OUTDOOR_KEYS) or "physical" in (contract.get("ceiling") or {}):
        problems.append("the outdoor profile is not supported for CAM-S2 in runtime v1")
    keys = {"format", "lineage", "variant", "checkpoint", "training_record", "runtime_env", "environment_inheritance",
            "camera", "observation", "action", "depth", "detector", "state", "qualification", "ground", "ceiling",
            "estimator_height", "sources"}
    if set(contract) != keys:
        problems.append("contract keys differ from the CAM-S2 schema")
    problems.extend(K.height_filter_problems(contract))
    for block in ("ground", "ceiling"):
        if isinstance(contract.get(block), dict) and contract[block].get("invalid_plane_rule") is True:
            problems.append("%s.invalid_plane_rule is meaningless for CAM-S2 (invalid depth is already no return)"
                            % block)
    try:
        K.capture_age_bound_s(contract)
    except (ValueError, TypeError) as exc:
        problems.append("timing: %s" % exc)
    for key, value in expected.items():
        if key in K.HEIGHT_FILTER_KEYS:
            continue
        if not _same(contract.get(key), value):
            problems.append("%s differs from the CAM-S2 training receipt" % key)
    sources = contract.get("sources")
    if not isinstance(sources, dict) or set(sources) != set(SOURCE_PATHS):
        problems.append("runtime source set differs from the CAM-S2 contract")
    elif verify_files:
        for rel, digest in sources.items():
            try:
                if K.sha256_file(Path(root) / rel) != digest:
                    problems.append("runtime source changed: %s" % rel)
            except OSError as exc:
                problems.append("runtime source unreadable: %s (%s)" % (rel, exc))
    asserted = contract.get("estimator_height")
    if verify_files and isinstance(asserted, dict) and K._evidence_problem(asserted.get("evidence")) is None:
        try:
            if K.sha256_file(asserted["evidence"]["path"]) != asserted["evidence"]["sha256"]:
                problems.append("estimator_height evidence SHA256 changed")
        except OSError as exc:
            problems.append("estimator_height evidence unreadable: %s" % exc)
    return problems


def near_constant_slots(obs_norm_path, threshold=NEAR_CONSTANT_VAR):
    """(indices, mean, var, epsilon) of the observation slots whose trained running variance is below threshold."""
    import numpy as np
    with np.load(obs_norm_path, allow_pickle=False) as data:
        mean, var, eps = data["running_mean"], data["running_var"], float(data["epsilon"])
    if mean.shape != (898,) or var.shape != (898,):
        raise ValueError("observation normaliser must hold 898 slots")
    idx = np.flatnonzero(var < threshold)
    return idx, mean, var, eps


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--check", type=Path, help="verify a previously generated CAM-S2 contract and all bound bytes")
    mode.add_argument("--output", type=Path, help="create a new contract; refuses an existing file")
    parser.add_argument("--receipt", type=Path, help="training receipt (realveh_entry receipt.json)")
    parser.add_argument("--checkpoint", type=Path, help="checkpoint listed by SHA-256 in the receipt run directory")
    parser.add_argument("--obs-norm", type=Path, help="the checkpoint's exported observation normaliser (.npz)")
    parser.add_argument("--previous-action-delay-steps", type=int,
                        help="k_hat: the vehicle's capture-to-actuation latency in 0.1 s control steps, inside the "
                             "trained support (CAM-S2a: 1 or 2); no default")
    parser.add_argument("--no-ground-removal", action="store_true", help="disable the floor height filter")
    parser.add_argument("--ground-margin-m", type=float)
    parser.add_argument("--ceiling-height-m", type=float)
    parser.add_argument("--ceiling-margin-m", type=float)
    parser.add_argument("--height-reference-evidence", type=Path)
    args = parser.parse_args(argv)
    try:
        if args.check:
            contract = K.load_contract(args.check)
        else:
            if None in (args.receipt, args.checkpoint, args.obs_norm, args.previous_action_delay_steps):
                parser.error("--output needs --receipt, --checkpoint, --obs-norm and --previous-action-delay-steps")
            ground = dict(K.GROUND_DEFAULT, enabled=not args.no_ground_removal)
            if args.ground_margin_m is not None:
                ground["margin_m"] = args.ground_margin_m
            ceiling = dict(K.CEILING_DEFAULT)
            if args.ceiling_height_m is not None:
                ceiling.update(enabled=True, height_m=args.ceiling_height_m)
                if args.ceiling_margin_m is not None:
                    ceiling["margin_m"] = args.ceiling_margin_m
            asserted = (None if args.height_reference_evidence is None
                        else K.estimator_height_assertion(args.height_reference_evidence))
            contract = build_contract(args.receipt, args.checkpoint, args.obs_norm,
                                      previous_action_delay_steps=args.previous_action_delay_steps,
                                      ground=ground, ceiling=ceiling, estimator_height=asserted)
            with args.output.open("x") as stream:
                json.dump(contract, stream, indent=2, sort_keys=True, allow_nan=False)
                stream.write("\n")
        print(json.dumps({"status": "CONTRACT_VALID", "lineage": contract["lineage"], "variant": contract["variant"],
                          "checkpoint_sha256": contract["checkpoint"]["sha256"],
                          "previous_action_delay_steps": contract["action"]["previous_action_delay_steps"],
                          "capture_age_bound_s": K.capture_age_bound_s(contract),
                          "height_filters": ("OFF" if not (contract["ground"]["enabled"] or contract["ceiling"]["enabled"])
                                             else "REFUSES_EVERY_FRAME_UNTIL_ASSERTED"
                                             if K.height_reference_problem(contract) else "ASSERTED"),
                          "qualification": contract["qualification"]}, sort_keys=True))
    except (OSError, ValueError, KeyError, TypeError) as exc:
        raise SystemExit("CAM-S2 camera policy contract refused: %s" % exc) from None


if __name__ == "__main__":
    main()
