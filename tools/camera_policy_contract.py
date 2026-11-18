"""Metadata-only contract for the trained CAM-K2 v3 RGB-D policy.

This tool neither deserializes checkpoints nor starts a simulator, camera or flight link.
The two training seeds remain distinct candidates. Choosing seed 1 is an engineering
choice, not a measured claim that it is better; the command line requires the choice.
Historical training and evaluation records are read-only. Contracts bind the current
runtime source and must be rebuilt after a source change.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
FORMAT = "motar-camera-policy-v1"
# Deployment height filters (not training metadata): the training arena had neither a
# floor nor a ceiling. Floor removal of VALID returns by height defaults ON with the S4
# margin (sitl_s4_gazebo.py GROUND_Z_M = 0.10). Ceiling removal defaults OFF because it
# needs a measured room height; no placeholder height is ever written. Heights are above
# the floor, which the estimator ENU z must also use.
# invalid_plane_rule (repair r1, F1): turning a SENSOR-INVALID pixel into no-return because
# its ray meets the floor/ceiling plane is opt-in and OFF by default. Row geometry cannot
# tell a textureless floor from a real obstacle closer than MinZ below camera height, so
# with the rule off a textureless floor or ceiling reads near again (an open item to decide
# from bench data; the OAK-D Pro IR projector usually gives indoor surfaces texture).
GROUND_DEFAULT = {"enabled": True, "margin_m": 0.10, "height_reference": "floor", "invalid_plane_rule": False}
CEILING_DEFAULT = {"enabled": False, "height_m": None, "margin_m": 0.10, "height_reference": "floor",
                   "invalid_plane_rule": False}
MAX_FILTER_MARGIN_M = 0.5
MAX_CEILING_HEIGHT_M = 100.0
# Repair r1 (F6): the floor/ceiling filters compare hit heights with the estimator ENU z, so
# they may run only after the operator asserts that this z is the height above the floor,
# bound to an evidence file by SHA-256. Default None: no assertion is ever invented, and a
# runtime with an enabled filter refuses every frame (fail closed) until one is given.
ESTIMATOR_HEIGHT_KEYS = ("z_is_height_above_floor", "evidence")
HEIGHT_FILTER_KEYS = ("ground", "ceiling", "estimator_height")
# Outdoor profile (2026-11-13, opt-in; every contract without it behaves exactly as before). Outdoors the OAK-D
# reports open sky as depth 0, so the runtime (camera_policy_runtime.py) verifies a ground plane from the camera on
# every frame, cross-checks it with PX4's estimate and refuses the frame (FrameRefused) when it cannot; the floor and
# the virtual ceiling are measured from that plane and the sky rule explains sky and far-ground sensor-invalid pixels
# as no-return. The three optional top-level keys come together. The operator estimator-height assertion is not used
# outdoors: PX4 z is relative to the take-off point and the per-frame camera check replaces it.
# ceiling.physical (optional key, accepted only together with the outdoor profile; absent = the historical physical
# ceiling): false marks a virtual ceiling (outdoor 3.0 m), which removes VALID returns above height_m - margin_m and
# touches nothing else: it never explains a sensor-invalid pixel (physical=false together with invalid_plane_rule=true
# is refused) and, since outdoor repair r1 (review P0-2b), its removed pixels never connect the sky rule either.
# Every threshold below is PROVISIONAL: chosen on synthetic scenes only, to be calibrated on real OAK-D outdoor
# recordings (tools/oakd_outdoor_replay.py). Each is stored as {"value": v, "status": "PROVISIONAL"}.
OUTDOOR_PROFILE = "outdoor"
OUTDOOR_KEYS = ("profile", "sky", "ground_estimate")
OUTDOOR_CEILING_HEIGHT_M = 3.0
# Outdoor repair r1 (review P3-3): the virtual ceiling removes valid returns, so a low one deletes real obstacles near
# the flight altitude; 2.5 m keeps at least 0.97 m above a 1.53 m camera. PROVISIONAL bounds (inclusive).
OUTDOOR_CEILING_BOUNDS_M = (2.5, 6.0)
PROVISIONAL = "PROVISIONAL"
ESTIMATOR_VERTICAL_FIELDS = ("z_valid", "epv", "dead_reckoning")
# The sky flood starts at the top row of the trained image and passes only through sky candidates and through
# pixels outside the source camera's field of view (unimaged). Virtual-ceiling-removed pixels are NOT connectors.
SKY_CONNECTIVITY = "top_border_4_through_sky_or_unimaged"


def provisional(value):
    """A threshold chosen on synthetic scenes only, pending calibration on real OAK-D data."""
    return {"value": value, "status": PROVISIONAL}


SKY_OUTDOOR_DEFAULT = {
    "enabled": True,
    "luma_min": provisional(0.5),          # Rec.601 luma of the projected RGB in [0, 1]
    "gradient_max": provisional(0.03),     # max(|dL/dx|, |dL/dy|) by central differences, luma per trained pixel
    "connectivity": SKY_CONNECTIVITY,
    # Outdoor repair r1 (review P0-1): rule 2 (sensor-invalid pixels whose ray meets the verified ground only beyond
    # MaxZ, or points below the horizon and never meets it, become no-return without any RGB test) is OFF. Depth
    # alone cannot tell invalid far ground from a dark, thin, sensor-invalid structure inside the vehicle's altitude
    # band (a rail, a barrier arm, a branch); with the rule on such structures are lost. Enabling it needs real OAK-D
    # evidence that far ground drops out to depth 0 AND the user's explicit acceptance of that loss
    # (ros2/motar_camera_policy/README.md). Without it, invalid far ground stays a near (phantom) band.
    "far_ground_rule": provisional(False),
    # Outdoor repair r1 (review P0-2/P0-3), edge-enclosure veto (camera_policy_runtime.edge_enclosure): a sensor-
    # invalid pixel lying between two near valid returns in its row or column is never freed. Range: the trained
    # riskcap governor releases beyond 5.0 m (training env NAVRL_SPEED_GOVERNOR_RELEASE_M) and the worst-case stop
    # from its free-speed cap 3.54 m/s at its 2.96 m/s^2 with a 0.3 s delay is about 3.2 m. Span: the whole trained
    # HFOV (66 deg) plus margin, so that a 2.4 m-wide face at 2 m (64.6 deg) is enclosed. Tolerance: the two
    # silhouette edges of one face 2.5 m wide seen up to about 25 deg off its normal. Measured trade-off: REPAIR_r1.md.
    "edge_veto_range_m": provisional(5.0),
    "edge_veto_span_deg": provisional(70.0),
    "edge_veto_range_tolerance_m": provisional(1.0),
}
GROUND_ESTIMATE_OUTDOOR_DEFAULT = {
    "enabled": True,
    "fit_max_range_m": provisional(10.0),
    "inlier_threshold_m": provisional(0.06),
    "min_inliers": provisional(300),
    "min_inlier_share": provisional(0.25),
    # 3.0 = PX4 attitude error budget (2 deg) + slope of a flat site (1 deg). A wrong plane's tilt t lifts the floor
    # band by r tan(t) at range r (0.87 m at 10 m for 5 deg); synthetic bridging planes across a step were 3.3-3.8 deg.
    "tilt_max_deg": provisional(3.0),
    "height_tolerance_m": provisional(0.30),
    "epv_max_m": provisional(0.5),
    "ambiguity_support_ratio": provisional(0.5),
    "ambiguity_separation_m": provisional(0.15),
    # Outdoor floor band: a valid return at optical z is floor when its height above the verified plane is below
    # ground.margin_m + z * (0.5 / fy + floor_noise_margin_per_m): half a trained pixel of angular quantisation
    # (z-buffer path) plus stereo depth noise and the z-buffer's nearest-sample bias, which grow with range.
    "floor_noise_margin_per_m": provisional(0.015),
    # The ground is the lowest surface: a candidate plane with more than this share of the fitted points below it
    # by more than inlier_threshold_m + floor_noise_margin_per_m * range (a bridging plane across a step, a box top
    # whose front face reaches lower) is not accepted (ground_not_lowest_surface).
    "max_below_share": provisional(0.02),
    # Outdoor repair r1 (review P1-1), wrong-surface checks (camera_policy_runtime.estimate_ground):
    # any other found horizontal plane holding this share of the points at a different height -> ground_ambiguous;
    "rival_min_share": provisional(0.10),
    # the accepted plane's inliers must hold this share of each image half and span this range (10th-90th pct);
    "min_half_share": provisional(0.10),
    "min_range_span_m": provisional(1.0),
    # planes fitted to the nearest/farthest third and to each half of its inliers must tilt less than this from it.
    "planarity_max_angle_deg": provisional(1.0),
    "refuse_dead_reckoning": True,
    "estimator_fields": list(ESTIMATOR_VERTICAL_FIELDS),
}
# (lower bound exclusive, upper bound inclusive, integer) for each provisional value.
SKY_BOUNDS = {"luma_min": (0.0, 1.0, False), "gradient_max": (0.0, 1.0, False),
              "edge_veto_range_m": (0.4, 20.0, False), "edge_veto_span_deg": (0.0, 90.0, False),
              "edge_veto_range_tolerance_m": (0.0, 20.0, False)}
# Provisional JSON-boolean switches of the sky block ({"value": true/false, "status": "PROVISIONAL"}).
SKY_FLAGS = ("far_ground_rule",)
# Outdoor repair r1 (review P3-3): tilt_max_deg and height_tolerance_m narrowed from (0, 30] and (0, 2] m.
GROUND_ESTIMATE_BOUNDS = {"fit_max_range_m": (0.5, 20.0, False), "inlier_threshold_m": (0.0, 0.5, False),
                          "min_inliers": (10, 2000, True), "min_inlier_share": (0.0, 1.0, False),
                          "tilt_max_deg": (0.0, 5.0, False), "height_tolerance_m": (0.0, 0.5, False),
                          "epv_max_m": (0.0, 5.0, False), "ambiguity_support_ratio": (0.0, 1.0, False),
                          "ambiguity_separation_m": (0.0, 2.0, False), "floor_noise_margin_per_m": (0.0, 0.1, False),
                          "max_below_share": (0.0, 1.0, False), "rival_min_share": (0.0, 1.0, False),
                          "min_half_share": (0.0, 0.5, False), "min_range_span_m": (0.0, 20.0, False),
                          "planarity_max_angle_deg": (0.0, 10.0, False)}
PINS = {
    1: {"checkpoint": "888e8056b2a6768318e6d9fa4136c5ed01f68b7480954f7f339801b99fafc8e1",
        "report": "b4af1444fcc3a070be7dda545f636d9710c1ffbef88ab708966608b650f57a39",
        "manifest": "30ff4a68653758a12f1229ce410a1479bd5ee285de6821616213a47a76c76575"},
    2: {"checkpoint": "203deea459ef1d937239281c46f07d96e797839708337666de1ecdf9492e4ea2",
        "report": "42076b65646d0cb7c8b4b106e919c8fe1656a1c0998eb179c8bced7d0078751b",
        "manifest": "f28cfa2f7d4bef04582ec8e8c1e53f26a58c3ec3a14104c39c173ebd9e20084f"},
}
SOURCE_PATHS = (
    "tools/camera_policy_contract.py", "tools/camera_policy_runtime.py", "tools/camera_estimator_link.py",
    "tools/camera_companion.py",
    "tools/sitl_camera.py", "tools/sitl_s4_gazebo.py", "tools/sitl_s4_model.py",
    "tools/sitl_s4/motar_s4_bridge.cc",
    "tools/sitl_s4/motar_rgbd_bridge.cc", "tools/sitl_s4/CMakeLists.txt",
    "tools/oakd_camera.py",
    # The ROS adapter and session define observation semantics too (REP 117 depth, CameraInfo
    # coverage, pose interpolation, altitude gating), so a contract also binds them (repair r1, F11).
    "tools/camera_ros2_adapter.py", "tools/camera_ros2_session.py",
    "tools/navrl_reflection_offline_audit.py",
    "aerial_gym/task/navrl_task/navrl_perception.py",
    "aerial_gym/task/navrl_task/navrl_sensor_ablation.py",
    "aerial_gym/task/navrl_task/navrl_depth_validity.py",
    "aerial_gym/task/navrl_task/navrl_corridor.py",
    "aerial_gym/task/navrl_task/navrl_search_state.py",
    "aerial_gym/task/navrl_task/speed_governor.py",
    "aerial_gym/config/task_config/navrl_task_config.py",
    "aerial_gym/rl_training/rl_games/navrl_players.py",
    "aerial_gym/rl_training/rl_games/navrl_transformer_network.py",
    "aerial_gym/rl_training/rl_games/navrl_action_models.py",
    "aerial_gym/rl_training/rl_games/ppo_update_safety.py",
    "aerial_gym/rl_training/rl_games/navrl_mode_probe.py",
    "aerial_gym/rl_training/rl_games/ppo_navrl_perception_transformer.yaml",
)


def sha256_file(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _report_path(seed, root):
    return Path(root) / ("docs/evidence/camonly_2026-11-04/camk2/train_report_v3_s%d.json" % seed)


def _read_report(seed, root):
    if type(seed) is not int or seed not in PINS:
        raise ValueError("CAM-K2 v3 seed must be explicitly 1 or 2")
    path = _report_path(seed, root)
    if sha256_file(path) != PINS[seed]["report"]:
        raise ValueError("training report differs from its historical SHA256 pin")
    report = json.loads(path.read_text())
    if (report.get("verdict") != "TRAINED" or report.get("development_smoke") is not False
            or report.get("seed") != seed or report.get("checkpoint_epoch") != 29000):
        raise ValueError("training report is not the selected trained non-smoke CAM-K2 v3 seed")
    return path, report


def _metadata(seed, root):
    path, report = _read_report(seed, root)
    ck = report["checkpoint"]
    env = report["realveh_receipt"]["env_navrl"]
    if (ck["sha256"] != PINS[seed]["checkpoint"] or ck["copy_sha256"] != ck["sha256"]
            or ck["manifest"]["sha256"] != PINS[seed]["manifest"]):
        raise ValueError("training checkpoint/manifest provenance does not match the selected seed")
    if (not isinstance(env, dict) or any(not isinstance(k, str) or not k.startswith("NAVRL_")
                                       or not isinstance(v, str) for k, v in env.items())):
        raise ValueError("training inference environment is malformed")
    # Checkpoint-compatible storage stays 72x4 even though its LiDAR returns are absent.
    required = {"NAVRL_SENSOR_ABLATION": "no_lidar", "NAVRL_LIDAR_HBEAMS": "72",
                "NAVRL_LIDAR_VBEAMS": "4", "NAVRL_LIDAR_RANGE": "12",
                "NAVRL_MAX_OBSTACLES": "8", "NAVRL_CORRIDOR_TOKENS": "0",
                "NAVRL_MAX_VELOCITY": "2.5", "NAVRL_YAW_RATE_MAX": "3.0",
                "NAVRL_CAMTRAIN_HFOV_DEG": "66", "NAVRL_CAMTRAIN_VFOV_DEG": "54",
                "NAVRL_DEPTH_VALIDITY": "blob_near", "NAVRL_DEPTH_VALIDITY_WINDOW": "5",
                "NAVRL_DEPTH_VALIDITY_DENSITY": "0.5", "NAVRL_DEPTH_MINZ_M": "0.4",
                "NAVRL_DEPTH_MAXZ_M": "20", "NAVRL_SPEED_GOVERNOR": "riskcap",
                "NAVRL_PERCEPTION_PERTURB": "0", "NAVRL_ACTION_POLICY": "squashed_gaussian",
                "NAVRL_ACTION_MU_SCALE": "1.0,0.4,1.0,1.0"}
    if any(env.get(k) != v for k, v in required.items()):
        raise ValueError("training environment lacks a required camera policy setting")
    width, height, hfov, vfov = 160, 90, 66.0, 54.0
    return {
        "format": FORMAT, "lineage": "CAM-K2-v3", "seed": seed,
        "checkpoint": {"path": ck["copy"], "sha256": PINS[seed]["checkpoint"], "epoch": 29000,
                       "manifest_path": str(Path(ck["copy"]).with_name(ck["file"] + ".manifest.json")),
                       "manifest_sha256": PINS[seed]["manifest"]},
        "training_record": {"path": str(path.resolve()), "sha256": PINS[seed]["report"],
                            "commit": report["commit"]},
        "runtime_env": dict(env), "environment_inheritance": "forbidden",
        "camera": {"width": width, "height": height, "hfov_deg": hfov, "vfov_deg": vfov,
                   "fx": width / (2.0 * math.tan(math.radians(hfov / 2.0))),
                   "fy": height / (2.0 * math.tan(math.radians(vfov / 2.0))),
                   "cx": (width - 1) / 2.0, "cy": (height - 1) / 2.0,
                   "translation_m": [0.10, 0.0, 0.03], "rgb_depth_alignment": "required"},
        "observation": {"dim": 898, "lidar_storage_shape": [1, 288], "no_lidar": True,
                        "history_steps": 5, "history_interval_s": 0.5, "control_dt_s": 0.1},
        "action": {"dim": 4, "max_velocity_mps": 2.5, "yaw_rate_max_rps": 3.0,
                   "previous_action": "governed_xy_divided_by_max_velocity_then_clamped_z_yaw"},
        "depth": {"input": "optical_z_m", "output": "ray_range_m",
                  "invalid_mask": "explicit_same_frame", "minz_m": 0.4, "maxz_m": 20.0,
                  "obstacle_far_m": 10.0, "validity": "blob_near", "window": 5, "density": 0.5,
                  "synthetic_noise_on_real_input": False},
        "detector": {"kind": "bootstrap_red", "scope": "red-target appearance only; physical NOT_TESTED"},
        "state": {"position_velocity_frame": "world_enu", "vehicle_quaternion": "xyzw",
                  "yaw_rate_unit": "rad/s", "target_truth_required": False},
        "qualification": {"cpu": "NOT_TESTED", "gazebo": "NOT_TESTED", "hardware": "NOT_TESTED"},
        "ground": dict(GROUND_DEFAULT), "ceiling": dict(CEILING_DEFAULT), "estimator_height": None,
    }


def _margin_ok(value):
    return (not isinstance(value, bool) and isinstance(value, (int, float)) and math.isfinite(value)
            and 0.0 < value <= MAX_FILTER_MARGIN_M)


def _evidence_problem(value):
    """None for a {path: absolute str, sha256: 64 lowercase hex} binding, else the problem."""
    if not isinstance(value, dict) or set(value) != {"path", "sha256"}:
        return "estimator_height.evidence must be exactly {path, sha256}"
    path, digest = value["path"], value["sha256"]
    if not isinstance(path, str) or not path or not Path(path).is_absolute():
        return "estimator_height.evidence.path must be an absolute file path"
    if (not isinstance(digest, str) or len(digest) != 64
            or any(c not in "0123456789abcdef" for c in digest)):
        return "estimator_height.evidence.sha256 must be 64 lowercase hex digits"
    return None


def estimator_height_assertion(evidence_path):
    """Operator assertion that the estimator ENU z is the height above the floor, bound to a file."""
    path = Path(evidence_path).resolve()
    return {"z_is_height_above_floor": True, "evidence": {"path": str(path), "sha256": sha256_file(path)}}


def height_reference_problem(contract):
    """None, or why a frame must be refused: an enabled floor/ceiling filter without the assertion.

    The filters compare hit heights with the estimator ENU z; if that z is not the height above the
    floor (a VIO or mocap origin off the floor, a take-off platform) they erase real obstacles or keep
    the floor as a phantom wall. Pure; the runtime and the ROS node call it (fail closed)."""
    ground, ceiling = contract["ground"], contract["ceiling"]
    if not (ground["enabled"] is True or ceiling["enabled"] is True):
        return None
    if is_outdoor(contract):
        # Outdoor profile: the runtime verifies the ground plane from the camera on every frame and refuses
        # frames it cannot verify (FrameRefused); the operator assertion is not used (validate_contract). Only a
        # well-formed outdoor profile is exempt (outdoor repair r1, review P3-3).
        problems = height_filter_problems(contract) + outdoor_problems(contract)
        return ("invalid outdoor profile: %s; frame refused" % "; ".join(problems)) if problems else None
    asserted = contract["estimator_height"]
    if (isinstance(asserted, dict) and asserted.get("z_is_height_above_floor") is True
            and _evidence_problem(asserted.get("evidence")) is None):
        return None
    return ("floor/ceiling filter enabled without the operator assertion that estimator z is the height "
            "above the floor (contract estimator_height, rebuild with --height-reference-evidence); frame refused")


def height_filter_problems(contract):
    """Structural checks of the deployment `ground`, `ceiling` and `estimator_height` fields (all required)."""
    problems = []
    ground, ceiling = contract.get("ground"), contract.get("ceiling")
    if not isinstance(ground, dict) or set(ground) != set(GROUND_DEFAULT):
        problems.append("ground must have exactly enabled, margin_m, height_reference and invalid_plane_rule")
    else:
        if not isinstance(ground["enabled"], bool):
            problems.append("ground.enabled must be a JSON boolean")
        if not _margin_ok(ground["margin_m"]):
            problems.append("ground.margin_m must be finite in (0, %g] m" % MAX_FILTER_MARGIN_M)
        if ground["height_reference"] != "floor":
            problems.append("ground.height_reference must be floor")
        if not isinstance(ground["invalid_plane_rule"], bool):
            problems.append("ground.invalid_plane_rule must be a JSON boolean")
        elif ground["invalid_plane_rule"] and ground["enabled"] is not True:
            problems.append("ground.invalid_plane_rule needs the floor filter enabled")
    if not isinstance(ceiling, dict) or set(ceiling) not in (set(CEILING_DEFAULT), set(CEILING_DEFAULT) | {"physical"}):
        problems.append("ceiling must have exactly enabled, height_m, margin_m, height_reference and "
                        "invalid_plane_rule (and optionally physical)")
    else:
        if "physical" in ceiling:
            if not any(key in contract for key in OUTDOOR_KEYS):
                # Outdoor repair r1 (review P3-3): the key exists for the outdoor profile only; an indoor contract
                # carrying it is refused exactly as before the outdoor profile existed.
                problems.append("ceiling.physical is accepted only with the outdoor profile")
            if not isinstance(ceiling["physical"], bool):
                problems.append("ceiling.physical must be a JSON boolean")
            elif ceiling["physical"] is False:
                if ceiling["enabled"] is not True:
                    problems.append("a virtual ceiling (physical=false) needs the ceiling filter enabled")
                if ceiling["invalid_plane_rule"] is not False:
                    problems.append("a virtual ceiling (physical=false) must never explain sensor-invalid pixels: "
                                    "invalid_plane_rule must be false")
        if not isinstance(ceiling["enabled"], bool):
            problems.append("ceiling.enabled must be a JSON boolean")
        if not _margin_ok(ceiling["margin_m"]):
            problems.append("ceiling.margin_m must be finite in (0, %g] m" % MAX_FILTER_MARGIN_M)
        if ceiling["height_reference"] != "floor":
            problems.append("ceiling.height_reference must be floor")
        if not isinstance(ceiling["invalid_plane_rule"], bool):
            problems.append("ceiling.invalid_plane_rule must be a JSON boolean")
        elif ceiling["invalid_plane_rule"] and ceiling["enabled"] is not True:
            problems.append("ceiling.invalid_plane_rule needs the ceiling filter enabled")
        height = ceiling["height_m"]
        if ceiling["enabled"] is True:
            if (isinstance(height, bool) or not isinstance(height, (int, float)) or not math.isfinite(height)
                    or not 2.0 * MAX_FILTER_MARGIN_M < height <= MAX_CEILING_HEIGHT_M):
                problems.append("enabled ceiling needs a measured height_m in (%g, %g] m"
                                % (2.0 * MAX_FILTER_MARGIN_M, MAX_CEILING_HEIGHT_M))
        elif height is not None:
            problems.append("disabled ceiling must not carry a height_m")
    if "estimator_height" not in contract:
        problems.append("estimator_height is required (null until the operator asserts it)")
    else:
        asserted = contract["estimator_height"]
        if asserted is not None:
            if not isinstance(asserted, dict) or set(asserted) != set(ESTIMATOR_HEIGHT_KEYS):
                problems.append("estimator_height must be null or exactly {z_is_height_above_floor, evidence}")
            else:
                if asserted["z_is_height_above_floor"] is not True:
                    problems.append("estimator_height.z_is_height_above_floor must be JSON true")
                problem = _evidence_problem(asserted["evidence"])
                if problem:
                    problems.append(problem)
    return problems


def is_outdoor(contract):
    """True only for a contract carrying the opt-in outdoor profile."""
    return isinstance(contract, dict) and contract.get("profile") == OUTDOOR_PROFILE


def threshold_value(block, key):
    """The value of a provisional threshold {"value", "status"} (plain values are returned unchanged)."""
    value = block[key]
    return value["value"] if isinstance(value, dict) else value


def _provisional_problems(block, name, bounds):
    problems = []
    for key, (low, high, integer) in bounds.items():
        item = block.get(key)
        if not isinstance(item, dict) or set(item) != {"value", "status"}:
            problems.append("%s.%s must be exactly {value, status}" % (name, key))
            continue
        if item["status"] != PROVISIONAL:
            problems.append("%s.%s.status must be %s (no calibrated value exists yet)" % (name, key, PROVISIONAL))
        value = item["value"]
        kind_ok = (type(value) is int) if integer else (not isinstance(value, bool) and isinstance(value, (int, float)))
        if not kind_ok or not math.isfinite(value) or not low < value <= high:
            problems.append("%s.%s.value must be %s in (%g, %g]" % (name, key, "an integer" if integer else "finite",
                                                                   low, high))
    return problems


def outdoor_problems(contract):
    """Structural checks of the optional outdoor profile (profile, sky, ground_estimate, ceiling.physical)."""
    present = [key for key in OUTDOOR_KEYS if key in contract]
    if not present:
        return []
    if len(present) != len(OUTDOOR_KEYS):
        return ["the outdoor profile needs all of %s together" % ", ".join(OUTDOOR_KEYS)]
    problems = []
    if contract["profile"] != OUTDOOR_PROFILE:
        problems.append("profile must be %r" % OUTDOOR_PROFILE)
    sky, estimate = contract["sky"], contract["ground_estimate"]
    if not isinstance(sky, dict) or set(sky) != set(SKY_OUTDOOR_DEFAULT):
        problems.append("sky must have exactly %s" % ", ".join(sorted(SKY_OUTDOOR_DEFAULT)))
    else:
        if not isinstance(sky["enabled"], bool):
            problems.append("sky.enabled must be a JSON boolean")
        if sky["connectivity"] != SKY_CONNECTIVITY:
            problems.append("sky.connectivity must be %r" % SKY_CONNECTIVITY)
        problems.extend(_provisional_problems(sky, "sky", SKY_BOUNDS))
        for key in SKY_FLAGS:
            item = sky[key]
            if (not isinstance(item, dict) or set(item) != {"value", "status"} or not isinstance(item["value"], bool)
                    or item["status"] != PROVISIONAL):
                problems.append("sky.%s must be exactly {value: JSON boolean, status: %s}" % (key, PROVISIONAL))
    if not isinstance(estimate, dict) or set(estimate) != set(GROUND_ESTIMATE_OUTDOOR_DEFAULT):
        problems.append("ground_estimate must have exactly %s" % ", ".join(sorted(GROUND_ESTIMATE_OUTDOOR_DEFAULT)))
    else:
        if estimate["enabled"] is not True:
            problems.append("ground_estimate.enabled must be true in the outdoor profile (the floor filter and "
                            "the sky rule use the camera-verified plane)")
        if estimate["refuse_dead_reckoning"] is not True:
            # Outdoor repair r1 (review P3-3): dead reckoning has no vertical reference; it is always refused.
            problems.append("ground_estimate.refuse_dead_reckoning must be JSON true")
        if estimate["estimator_fields"] != list(ESTIMATOR_VERTICAL_FIELDS):
            problems.append("ground_estimate.estimator_fields must be %s" % list(ESTIMATOR_VERTICAL_FIELDS))
        problems.extend(_provisional_problems(estimate, "ground_estimate", GROUND_ESTIMATE_BOUNDS))
    ground, ceiling = contract.get("ground"), contract.get("ceiling")
    if isinstance(ground, dict) and (ground.get("enabled") is not True or ground.get("invalid_plane_rule") is not False):
        problems.append("the outdoor profile needs the floor filter enabled with invalid_plane_rule false")
    if isinstance(ceiling, dict) and (ceiling.get("enabled") is not True or ceiling.get("physical") is not False):
        problems.append("the outdoor profile needs an enabled virtual ceiling (physical=false)")
    elif isinstance(ceiling, dict):
        height = ceiling.get("height_m")
        low, high = OUTDOOR_CEILING_BOUNDS_M
        if (isinstance(height, bool) or not isinstance(height, (int, float)) or not math.isfinite(height)
                or not low <= height <= high):
            problems.append("the outdoor virtual ceiling height_m must lie in [%g, %g] m (PROVISIONAL)" % (low, high))
    if contract.get("estimator_height") is not None:
        problems.append("the outdoor profile verifies the ground from the camera on every frame; estimator_height "
                        "(the operator floor assertion) must be null")
    return problems


def outdoor_blocks(ceiling_height_m=OUTDOOR_CEILING_HEIGHT_M, ground_margin_m=None, ceiling_margin_m=None):
    """The outdoor profile's deployment blocks: floor filter on the camera-verified plane (plane rule off),
    virtual ceiling (physical=false, default 3.0 m above the verified ground), sky rule on with the far-ground rule
    OFF (sky.far_ground_rule, outdoor repair r1), ground estimate on."""
    ground = dict(GROUND_DEFAULT, enabled=True, invalid_plane_rule=False)
    if ground_margin_m is not None:
        ground["margin_m"] = ground_margin_m
    ceiling = dict(CEILING_DEFAULT, enabled=True, height_m=ceiling_height_m, invalid_plane_rule=False, physical=False)
    if ceiling_margin_m is not None:
        ceiling["margin_m"] = ceiling_margin_m
    return {"ground": ground, "ceiling": ceiling, "profile": OUTDOOR_PROFILE,
            "sky": json.loads(json.dumps(SKY_OUTDOOR_DEFAULT)),
            "ground_estimate": json.loads(json.dumps(GROUND_ESTIMATE_OUTDOOR_DEFAULT))}


def build_contract(seed=1, *, root=ROOT, ground=None, ceiling=None, estimator_height=None, outdoor=None):
    """Bind a selected trained checkpoint to the current inference source, without loading it.

    ground/ceiling override the deployment height filters; the ceiling height must be
    the measured room height above the floor. estimator_height is the operator's assertion
    (estimator_height_assertion(evidence)) that the estimator z is the height above the floor;
    without it an enabled filter refuses every frame. outdoor (opt-in, e.g. outdoor_blocks())
    adds the outdoor profile: its ground/ceiling blocks replace the indoor ones and
    estimator_height must stay None.
    """
    contract = _metadata(seed, root)
    if ground is not None:
        contract["ground"] = dict(ground)
    if ceiling is not None:
        contract["ceiling"] = dict(ceiling)
    if estimator_height is not None:
        contract["estimator_height"] = json.loads(json.dumps(estimator_height))
    if outdoor is not None:
        if ground is not None or ceiling is not None:
            raise ValueError("the outdoor profile carries its own ground and ceiling blocks")
        contract.update(json.loads(json.dumps(outdoor)))
    contract["sources"] = {rel: sha256_file(Path(root) / rel) for rel in SOURCE_PATHS}
    problems = validate_contract(contract, root=root)
    if problems:
        raise ValueError("invalid camera policy contract: " + "; ".join(problems))
    return contract


def validate_contract(contract, verify_files=True, *, root=ROOT):
    """Return all structural/provenance problems; an empty list accepts the fixed contract.

    verify_files=False omits hashing current runtime/checkpoint bytes only. The pinned
    historical training record is always read so arbitrary NAVRL environment additions,
    omitted settings and cross-seed metadata cannot enter the inference path.
    """
    problems = []
    if not isinstance(contract, dict):
        return ["camera policy contract must be a JSON object"]
    try:
        expected = _metadata(contract.get("seed"), root)
    except (OSError, ValueError, KeyError, TypeError) as exc:
        return ["camera policy training provenance: %s" % exc]
    keys = set(expected) | {"sources"}
    if any(key in contract for key in OUTDOOR_KEYS):
        keys |= set(OUTDOOR_KEYS)          # the opt-in outdoor profile; checked by outdoor_problems
    if set(contract) != keys:
        problems.append("contract keys differ from the supported schema")
    problems.extend(height_filter_problems(contract))
    problems.extend(outdoor_problems(contract))
    for key, value in expected.items():
        if key in HEIGHT_FILTER_KEYS:
            continue  # deployment settings, checked structurally above
        try:
            # JSON's booleans must not be accepted through Python's True == 1 rule.
            same = json.dumps(contract.get(key), sort_keys=True, allow_nan=False) == json.dumps(value, sort_keys=True)
        except (TypeError, ValueError):
            same = False
        if not same:
            problems.append("%s differs from the selected trained camera policy contract" % key)
    sources = contract.get("sources")
    if not isinstance(sources, dict) or set(sources) != set(SOURCE_PATHS):
        problems.append("runtime source set differs from the supported contract")
    elif any(not isinstance(value, str) or len(value) != 64
             or any(c not in "0123456789abcdef" for c in value) for value in sources.values()):
        problems.append("runtime source SHA256 is malformed")
    elif verify_files:
        for rel, digest in sources.items():
            try:
                if sha256_file(Path(root) / rel) != digest:
                    problems.append("runtime source changed: %s" % rel)
            except OSError as exc:
                problems.append("runtime source unreadable: %s (%s)" % (rel, exc))
    asserted = contract.get("estimator_height")
    if verify_files and isinstance(asserted, dict) and _evidence_problem(asserted.get("evidence")) is None:
        evidence = asserted["evidence"]
        try:
            if sha256_file(evidence["path"]) != evidence["sha256"]:
                problems.append("estimator_height evidence SHA256 changed")
        except OSError as exc:
            problems.append("estimator_height evidence unreadable: %s" % exc)
    if verify_files:
        ck = expected["checkpoint"]
        for label, path_key, digest_key in (("checkpoint", "path", "sha256"),
                                          ("checkpoint manifest", "manifest_path", "manifest_sha256")):
            try:
                if sha256_file(ck[path_key]) != ck[digest_key]:
                    problems.append("%s SHA256 differs from its historical pin" % label)
            except OSError as exc:
                problems.append("%s unreadable: %s" % (label, exc))
        try:
            manifest = json.loads(Path(ck["manifest_path"]).read_text())
            if (manifest.get("verdict") != "TRAINED" or manifest.get("unit") != "CAM-K2 v3"
                    or manifest.get("seed") != expected["seed"] or manifest.get("robot") != "navrl_quad"
                    or manifest.get("checkpoint_sha256") != ck["sha256"]):
                problems.append("checkpoint sidecar does not name the selected trained CAM-K2 v3 seed")
        except (OSError, ValueError, AttributeError) as exc:
            problems.append("checkpoint sidecar unreadable: %s" % exc)
    return problems


def load_contract(path, verify_files=True, *, root=ROOT):
    contract = json.loads(Path(path).read_text())
    problems = validate_contract(contract, verify_files=verify_files, root=root)
    if problems:
        raise ValueError("invalid camera policy contract: " + "; ".join(problems))
    return contract


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--check", type=Path, help="verify a previously generated contract and all bound bytes")
    mode.add_argument("--output", type=Path, help="create a new contract; refuses an existing file")
    parser.add_argument("--seed", type=int, choices=(1, 2), help="explicit training seed; no measured winner implied")
    parser.add_argument("--no-ground-removal", action="store_true", help="disable the floor height filter")
    parser.add_argument("--ground-margin-m", type=float, help="floor band above the floor [m], default 0.10")
    parser.add_argument("--ceiling-height-m", type=float,
                        help="measured ceiling height above the floor [m]; enables the ceiling filter")
    parser.add_argument("--ceiling-margin-m", type=float, help="ceiling band below the ceiling [m], default 0.10")
    parser.add_argument("--ground-invalid-plane-rule", action="store_true",
                        help="also explain sensor-invalid pixels whose ray meets the floor as no-return (default off)")
    parser.add_argument("--ceiling-invalid-plane-rule", action="store_true",
                        help="the same for the ceiling plane; needs --ceiling-height-m (default off)")
    parser.add_argument("--height-reference-evidence", type=Path,
                        help="operator assertion that the estimator ENU z is the height above the floor, bound "
                             "to this evidence file by SHA-256; required before an enabled filter accepts frames")
    parser.add_argument("--profile", choices=(OUTDOOR_PROFILE,),
                        help="outdoor: floor filter on the camera-verified ground plane (plane rule off), virtual "
                             "ceiling %.1f m (physical=false; --ceiling-height-m overrides), sky rule on, per-frame "
                             "ground estimate cross-checked with PX4 (z_valid, epv, dead_reckoning); thresholds are "
                             "PROVISIONAL. No operator height assertion is used." % OUTDOOR_CEILING_HEIGHT_M)
    args = parser.parse_args(argv)
    filters = (args.no_ground_removal or args.ground_invalid_plane_rule or args.ceiling_invalid_plane_rule
               or any(x is not None for x in (args.ground_margin_m, args.ceiling_height_m, args.ceiling_margin_m,
                                              args.height_reference_evidence, args.profile)))
    try:
        if args.check:
            if args.seed is not None or filters:
                parser.error("--seed and height-filter options apply only to --output")
            contract = load_contract(args.check)
        elif args.profile == OUTDOOR_PROFILE:
            if args.seed is None:
                parser.error("--output requires an explicit --seed")
            if (args.no_ground_removal or args.ground_invalid_plane_rule or args.ceiling_invalid_plane_rule
                    or args.height_reference_evidence is not None):
                parser.error("--profile outdoor fixes the floor filter on, both invalid plane rules off and uses no "
                             "operator height assertion")
            blocks = outdoor_blocks(OUTDOOR_CEILING_HEIGHT_M if args.ceiling_height_m is None else args.ceiling_height_m,
                                    args.ground_margin_m, args.ceiling_margin_m)
            contract = build_contract(args.seed, outdoor=blocks)
            with args.output.open("x") as stream:
                json.dump(contract, stream, indent=2, sort_keys=True, allow_nan=False)
                stream.write("\n")
        else:
            if args.seed is None:
                parser.error("--output requires an explicit --seed")
            if args.ceiling_margin_m is not None and args.ceiling_height_m is None:
                parser.error("--ceiling-margin-m needs a measured --ceiling-height-m")
            if args.ceiling_invalid_plane_rule and args.ceiling_height_m is None:
                parser.error("--ceiling-invalid-plane-rule needs a measured --ceiling-height-m")
            ground = dict(GROUND_DEFAULT, enabled=not args.no_ground_removal)
            if args.ground_margin_m is not None:
                ground["margin_m"] = args.ground_margin_m
            if args.ground_invalid_plane_rule:
                ground["invalid_plane_rule"] = True
            ceiling = dict(CEILING_DEFAULT)
            if args.ceiling_height_m is not None:
                ceiling.update(enabled=True, height_m=args.ceiling_height_m)
                if args.ceiling_margin_m is not None:
                    ceiling["margin_m"] = args.ceiling_margin_m
                if args.ceiling_invalid_plane_rule:
                    ceiling["invalid_plane_rule"] = True
            asserted = (None if args.height_reference_evidence is None
                        else estimator_height_assertion(args.height_reference_evidence))
            contract = build_contract(args.seed, ground=ground, ceiling=ceiling, estimator_height=asserted)
            with args.output.open("x") as stream:
                json.dump(contract, stream, indent=2, sort_keys=True, allow_nan=False)
                stream.write("\n")
        summary = {"status": "CONTRACT_VALID", "seed": contract["seed"],
                   "checkpoint_sha256": contract["checkpoint"]["sha256"],
                   "ground": contract["ground"], "ceiling": contract["ceiling"],
                   "estimator_height": contract["estimator_height"],
                   "height_filters": ("OFF" if not (contract["ground"]["enabled"] or contract["ceiling"]["enabled"])
                                      else "REFUSES_EVERY_FRAME_UNTIL_ASSERTED"
                                      if height_reference_problem(contract) else "ASSERTED"),
                   "qualification": contract["qualification"]}
        if is_outdoor(contract):
            summary.update(profile=contract["profile"], sky=contract["sky"], ground_estimate=contract["ground_estimate"],
                           height_filters="CAMERA_VERIFIED_PER_FRAME", thresholds=PROVISIONAL)
        print(json.dumps(summary, sort_keys=True))
    except (OSError, ValueError, KeyError, TypeError) as exc:
        raise SystemExit("camera policy contract refused: %s" % exc) from None


if __name__ == "__main__":
    main()
