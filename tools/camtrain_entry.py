"""Entry point of the CAM-K1 camera-only continuation training (docs/plans/prereg_camonly_k1_train_v1_2026-11-06.md).

Installs, before runner.py builds the task, everything that makes this lineage differ from Policy F's training:
  1. the opt-in LiDAR ablation NAVRL_SENSOR_ABLATION (must be `no_lidar`: perception and governor see no LiDAR; the
     simulator's LiDAR still exists and is read by the reward's static-safety term only, which the observation
     contract allows);
  2. the camera field of view of the real RGB camera (Luxonis OAK-D Pro, autofocus IMX378: 66 x 54 deg), shared by the
     detector image and the obstacle depth image, as the user fixed it;
  3. two reward weights that keep the motion and the target in the camera's view: yaw_align_weight (the speed-gated
     crab penalty, <= 0, so it opens no standing income) and visibility_bonus (per-step, positive, raised a little).
Every value is required from the environment; nothing has a default here, so a missing setting refuses the run.
The installed values and this file's SHA-256 are printed and written to NAVRL_CAMTRAIN_RECEIPT (refusing to
overwrite). Then runner.py runs unchanged as __main__.
usage: camtrain_python runner.py ARGS...
"""
import hashlib
import json
import os
import pathlib
import runpy
import sys

REQUIRED = ("NAVRL_CAMTRAIN_HFOV_DEG", "NAVRL_CAMTRAIN_VFOV_DEG", "NAVRL_CAMTRAIN_YAW_ALIGN_WEIGHT",
            "NAVRL_CAMTRAIN_VISIBILITY_BONUS", "NAVRL_CAMTRAIN_RECEIPT")
CONDITION = "no_lidar"
LIMITS = {"NAVRL_CAMTRAIN_HFOV_DEG": (30.0, 120.0), "NAVRL_CAMTRAIN_VFOV_DEG": (20.0, 90.0),
          "NAVRL_CAMTRAIN_YAW_ALIGN_WEIGHT": (0.0, 3.0), "NAVRL_CAMTRAIN_VISIBILITY_BONUS": (0.0, 0.2)}


def settings(environ=None):
    """The required values, validated: refuse a missing, non-numeric or out-of-range value and any condition but
    no_lidar."""
    env = os.environ if environ is None else environ
    missing = [k for k in REQUIRED if not env.get(k, "").strip()]
    if missing:
        raise SystemExit("camtrain: missing %s" % ", ".join(missing))
    if env.get("NAVRL_SENSOR_ABLATION", "") != CONDITION:
        raise SystemExit("camtrain: NAVRL_SENSOR_ABLATION must be %s, got %r" % (CONDITION,
                                                                                 env.get("NAVRL_SENSOR_ABLATION")))
    out = {}
    for k, (lo, hi) in LIMITS.items():
        try:
            v = float(env[k])
        except ValueError:
            raise SystemExit("camtrain: %s is not a number: %r" % (k, env[k])) from None
        if not lo <= v <= hi:
            raise SystemExit("camtrain: %s=%g outside [%g, %g]" % (k, v, lo, hi))
        out[k] = v
    return out


def apply_overrides(task_config, s):
    """Set the camera FOV and the two reward weights on the task configuration class, before the task is built.
    Returns the values before and after, for the receipt."""
    before = {"detector_hfov_deg": float(task_config.vision.detector_hfov_deg),
              "detector_vfov_deg": float(task_config.vision.detector_vfov_deg),
              "yaw_align_weight": float(task_config.reward_parameters["yaw_align_weight"]),
              "visibility_bonus": float(task_config.vision.visibility_bonus)}
    task_config.vision.detector_hfov_deg = s["NAVRL_CAMTRAIN_HFOV_DEG"]
    task_config.vision.detector_vfov_deg = s["NAVRL_CAMTRAIN_VFOV_DEG"]
    task_config.reward_parameters["yaw_align_weight"] = s["NAVRL_CAMTRAIN_YAW_ALIGN_WEIGHT"]
    task_config.vision.visibility_bonus = s["NAVRL_CAMTRAIN_VISIBILITY_BONUS"]
    after = {"detector_hfov_deg": float(task_config.vision.detector_hfov_deg),
             "detector_vfov_deg": float(task_config.vision.detector_vfov_deg),
             "yaw_align_weight": float(task_config.reward_parameters["yaw_align_weight"]),
             "visibility_bonus": float(task_config.vision.visibility_bonus)}
    return before, after


def main():
    if len(sys.argv) < 2:
        raise SystemExit("usage: camtrain_entry.py SCRIPT [ARGS...]")
    s = settings()
    script = os.path.abspath(sys.argv[1])
    if os.path.basename(script) != "runner.py":
        raise SystemExit("camtrain: only runner.py may be run through this entry, got %s" % script)
    from aerial_gym.rl_training.rl_games.navrl_import_origin import assert_runtime_origin
    from aerial_gym.rl_training.rl_games.quiet_rl_io import devnull_stdio, quiet_startup_enabled

    assert_runtime_origin()
    if quiet_startup_enabled():
        with devnull_stdio():
            import isaacgym  # noqa: F401  (Isaac Gym must be imported before torch)
    else:
        import isaacgym  # noqa: F401
    from aerial_gym.task.navrl_task import navrl_sensor_ablation as ablation
    from aerial_gym.config.task_config.navrl_task_config import task_config

    info = {"ablation": ablation.install(ablation.condition_from_env())}
    before, after = apply_overrides(task_config, s)
    info.update({"overrides_before": before, "overrides_after": after, "script": script,
                 "entry": os.path.abspath(__file__),
                 "entry_sha256": hashlib.sha256(pathlib.Path(__file__).read_bytes()).hexdigest(),
                 "ablation_module_sha256": hashlib.sha256(pathlib.Path(ablation.__file__).read_bytes()).hexdigest()})
    print("[camtrain] " + json.dumps(info, sort_keys=True), flush=True)
    with open(os.environ["NAVRL_CAMTRAIN_RECEIPT"], "x") as fh:
        json.dump(info, fh, indent=1, sort_keys=True)
        fh.write("\n")
    sys.argv = [script] + sys.argv[2:]
    sys.path[0] = os.path.dirname(script)      # as `python runner.py` would set it
    runpy.run_path(script, run_name="__main__")


if __name__ == "__main__":
    main()
