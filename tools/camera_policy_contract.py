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
    }


def build_contract(seed=1, *, root=ROOT):
    """Bind a selected trained checkpoint to the current inference source, without loading it."""
    contract = _metadata(seed, root)
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
    if set(contract) != set(expected) | {"sources"}:
        problems.append("contract keys differ from the supported schema")
    for key, value in expected.items():
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
    args = parser.parse_args(argv)
    try:
        if args.check:
            if args.seed is not None:
                parser.error("--seed applies only to --output")
            contract = load_contract(args.check)
        else:
            if args.seed is None:
                parser.error("--output requires an explicit --seed")
            contract = build_contract(args.seed)
            with args.output.open("x") as stream:
                json.dump(contract, stream, indent=2, sort_keys=True, allow_nan=False)
                stream.write("\n")
        print(json.dumps({"status": "CONTRACT_VALID", "seed": contract["seed"],
                          "checkpoint_sha256": contract["checkpoint"]["sha256"],
                          "qualification": contract["qualification"]}, sort_keys=True))
    except (OSError, ValueError, KeyError, TypeError) as exc:
        raise SystemExit("camera policy contract refused: %s" % exc) from None


if __name__ == "__main__":
    main()
