"""Entry point of the real-vehicle simulation study (2026-11-08; design ~/motar_tmp/realveh_design_20261108/SURVEY.md,
IMPLEMENTATION.md part 3).

Opt-in. Before runner.py builds the task it installs, in this fixed order:
  1. vehicle randomization, only if NAVRL_VEH_RAND=1 (aerial_gym/task/navrl_task/navrl_vehicle_rand.py): the nominal
     per-motor thrust cap TW_NOMINAL x m x g / 4, which makes navrl_quad fly like the 0.50 kg "Option A" vehicle
     (T/W 2.25), and the per-env / per-motor thrust factors and motor time constants around it;
  2. the OAK-D Pro depth model, only if NAVRL_DEPTH_MODEL=oakd (navrl_depth_model.py), BEFORE the ablation, which
     binds observe's signature at install;
  2b. CAM-K2 v3 only, if NAVRL_DEPTH_VALIDITY=blob_near (navrl_depth_validity.py; requires the depth model): the
     invalid-depth validity rule, installed just BEFORE the depth model so that its wrapper is the inner one and runs
     on the frame the depth model produced. Unset: the module is not loaded and nothing below changes (the receipt
     has no depth_validity keys);
  3. the camera-only LiDAR ablation (NAVRL_SENSOR_ABLATION must be no_lidar) and the CAM-K1 camera FOV / reward
     overrides, through tools/camtrain_entry.py's `settings` and `apply_overrides`, used unchanged.
The vehicle stays navrl_quad. Its robot identity (the checkpoint's cfg_robot_* fields), URDF, config and Lee gains
are Policy F's and CAM-K1's, so checkpoints keep their lineage and NavRLTask's robot-lineage guard is untouched. The
Option A files (quad_navrl_rv05.urdf and its robot config) are documentation and the oracle of the
equivalence test in tests/test_realveh.py; nothing here uses them.
Refused: a missing or malformed value; NAVRL_ROBOT other than unset / navrl_quad; any NAVRL_REALVEH_* variable other
than NAVRL_REALVEH_RECEIPT (NAVRL_REALVEH_ROBOT and NAVRL_REALVEH_WARMSTART_FROM are retired); vehicle randomization
whose worst-case thrust-to-weight TW_NOMINAL x (1 - TW_SCALE) x (1 - MOTOR_IMBAL) is below
MIN_WORST_CASE_THRUST_TO_WEIGHT; the depth model together with NAVRL_PERCEPTION_PERTURB on (two depth noises); a
module parameter set while its switch is off; a script other than runner.py; an existing receipt or record.
NAVRL_CAMTRAIN_RECEIPT must be unset or equal to NAVRL_REALVEH_RECEIPT: this entry writes one receipt (refusing to
overwrite) with every module's SHA-256, all parsed settings and the install records; vehicle randomization writes
the cap it computed to NAVRL_REALVEH_RECEIPT + VEH_RAND_RECORD_SUFFIX. Then runner.py runs unchanged as __main__.
usage: realveh_python runner.py ARGS...
"""
import hashlib
import importlib.util
import json
import os
import pathlib
import runpy
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
TOOLS = ROOT / "tools"
ROBOT = "navrl_quad"
DEPTH_MODEL = "oakd"
REQUIRED = ("NAVRL_REALVEH_RECEIPT",)
RETIRED = ("NAVRL_REALVEH_ROBOT", "NAVRL_REALVEH_WARMSTART_FROM")
VEH_RAND = ROOT / "aerial_gym/task/navrl_task/navrl_vehicle_rand.py"
DEPTH = ROOT / "aerial_gym/task/navrl_task/navrl_depth_model.py"
VALIDITY = ROOT / "aerial_gym/task/navrl_task/navrl_depth_validity.py"     # CAM-K2 v3; hashed only when on
CAMSAFE = ROOT / "aerial_gym/task/navrl_task/navrl_camsafe.py"     # CAM-S1; hashed only when on
CAMS2 = ROOT / "aerial_gym/task/navrl_task/navrl_cams2.py"     # CAM-S2a; hashed only when on
VALIDITY_RULE = "blob_near"
VALIDITY_PARAMETERS = ("NAVRL_DEPTH_VALIDITY_WINDOW", "NAVRL_DEPTH_VALIDITY_DENSITY")   # navrl_depth_validity.REQUIRED
# Hashed into the receipt (all of them, whether or not their switch is on).
HASHED = {"realveh_entry": pathlib.Path(__file__).resolve(), "realveh_python": TOOLS / "realveh_python",
          "camtrain_entry": TOOLS / "camtrain_entry.py",
          "sensor_ablation": ROOT / "aerial_gym/task/navrl_task/navrl_sensor_ablation.py",
          "vehicle_rand": VEH_RAND, "depth_model": DEPTH,
          "robot_config": ROOT / "aerial_gym/config/robot_config/navrl_quad_config.py",
          "robot_urdf": ROOT / "resources/robots/quad/quad_navrl_collide.urdf",
          # inherited or wrapped at run time, not edited
          "base_quad_config": ROOT / "aerial_gym/config/robot_config/base_quad_config.py",
          "lee_controller_config_navrl": ROOT / "aerial_gym/config/controller_config/lee_controller_config_navrl.py",
          "base_multirotor": ROOT / "aerial_gym/robots/base_multirotor.py",
          "motor_model": ROOT / "aerial_gym/control/motor_model.py",
          "control_allocation": ROOT / "aerial_gym/control/control_allocation.py",
          "navrl_perception": ROOT / "aerial_gym/task/navrl_task/navrl_perception.py",
          "navrl_task": ROOT / "aerial_gym/task/navrl_task/navrl_task.py"}
# navrl_depth_model.REQUIRED (tests/test_realveh.py checks the two agree); listed here so that the module is not loaded
# while NAVRL_DEPTH_MODEL is off. Other NAVRL_DEPTH_* knobs (NAVRL_DEPTH_NOISE_*) belong to perception, not to it.
DEPTH_PARAMETERS = ("NAVRL_DEPTH_MINZ_M", "NAVRL_DEPTH_MAXZ_M", "NAVRL_DEPTH_SIGMA_D_PX", "NAVRL_DEPTH_FB_PXM",
                    "NAVRL_DEPTH_HOLE_P", "NAVRL_DEPTH_SEED")
MIN_WORST_CASE_THRUST_TO_WEIGHT = 1.8
PERTURB_OFF = ("", "0", "false", "no", "off")
VEH_RAND_RECORD_SUFFIX = ".veh_rand_init.json"


def _load_private(name, path):
    """A stdlib-only module by file path, under a private name and not registered in sys.modules."""
    spec = importlib.util.spec_from_file_location(name, str(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


camtrain = _load_private("realveh_camtrain_entry", TOOLS / "camtrain_entry.py")


def sha256(path):
    return hashlib.sha256(pathlib.Path(path).read_bytes()).hexdigest()


def settings(environ=None):
    """Every setting, validated without importing torch, Isaac Gym or aerial_gym. Refuses (SystemExit) a missing,
    malformed or conflicting value, and a module parameter set while its switch is off. The randomization and depth
    modules are loaded (privately, for their settings parser only) only when their switch is on."""
    env = dict(os.environ if environ is None else environ)
    missing = [k for k in REQUIRED if not env.get(k, "").strip()]
    if missing:
        raise SystemExit("realveh: missing %s" % ", ".join(missing))
    unknown = sorted(k for k in env if k.startswith("NAVRL_REALVEH_") and k not in REQUIRED and env[k].strip())
    if unknown:
        raise SystemExit("realveh: %s not supported (retired: %s); the vehicle stays navrl_quad"
                         % (", ".join(unknown), ", ".join(RETIRED)))
    if env.get("NAVRL_ROBOT", "").strip() not in ("", ROBOT):
        raise SystemExit("realveh: NAVRL_ROBOT must be unset or %s, got %r" % (ROBOT, env["NAVRL_ROBOT"]))
    receipt = env["NAVRL_REALVEH_RECEIPT"]
    camtrain_receipt = env.get("NAVRL_CAMTRAIN_RECEIPT", "").strip()
    if camtrain_receipt and camtrain_receipt != receipt:
        raise SystemExit("realveh: NAVRL_CAMTRAIN_RECEIPT must be unset or equal NAVRL_REALVEH_RECEIPT (this entry "
                         "writes one receipt)")
    out = {"receipt": receipt, "camtrain": camtrain.settings(dict(env, NAVRL_CAMTRAIN_RECEIPT=receipt))}
    veh, depth = env.get("NAVRL_VEH_RAND", "").strip(), env.get("NAVRL_DEPTH_MODEL", "").strip()
    if veh not in ("", "0", "1"):
        raise SystemExit("realveh: NAVRL_VEH_RAND must be unset, 0 or 1, got %r" % veh)
    if depth not in ("", DEPTH_MODEL):
        raise SystemExit("realveh: NAVRL_DEPTH_MODEL must be unset or %s, got %r" % (DEPTH_MODEL, depth))
    if veh == "1":
        module = _load_private("realveh_vehicle_rand_settings", VEH_RAND)
        try:
            out["vehicle_rand"] = module.settings_from_env(env)
        except ValueError as exc:
            raise SystemExit("realveh: %s" % exc) from None
        worst = module.worst_case_thrust_to_weight(out["vehicle_rand"])
        if worst < MIN_WORST_CASE_THRUST_TO_WEIGHT:
            raise SystemExit("realveh: worst-case thrust-to-weight %.4f = %g x (1 - %g) x (1 - %g) is below %g"
                             % (worst, out["vehicle_rand"]["NAVRL_VEH_RAND_TW_NOMINAL"],
                                out["vehicle_rand"]["NAVRL_VEH_RAND_TW_SCALE"],
                                out["vehicle_rand"]["NAVRL_VEH_RAND_MOTOR_IMBAL"], MIN_WORST_CASE_THRUST_TO_WEIGHT))
        out["worst_case_thrust_to_weight"] = worst
        if os.path.exists(receipt + VEH_RAND_RECORD_SUFFIX):
            raise SystemExit("realveh: vehicle-randomization record %s exists" % (receipt + VEH_RAND_RECORD_SUFFIX))
    else:
        stray = sorted(k for k in env if k.startswith("NAVRL_VEH_RAND_") and env[k].strip())
        if stray:
            raise SystemExit("realveh: %s set while NAVRL_VEH_RAND is off" % ", ".join(stray))
        out["vehicle_rand"] = None
    if depth == DEPTH_MODEL:
        perturb = env.get("NAVRL_PERCEPTION_PERTURB", "")
        if perturb.strip().lower() not in PERTURB_OFF:
            raise SystemExit("realveh: NAVRL_PERCEPTION_PERTURB=%r together with the depth model would add a second "
                             "depth noise to the detection depth" % perturb)
        try:
            out["depth_model"] = _load_private("realveh_depth_model_settings", DEPTH).settings_from_env(env)
        except ValueError as exc:
            raise SystemExit("realveh: %s" % exc) from None
    else:
        stray = sorted(k for k in DEPTH_PARAMETERS if env.get(k, "").strip())
        if stray:
            raise SystemExit("realveh: %s set while NAVRL_DEPTH_MODEL is off" % ", ".join(stray))
        out["depth_model"] = None
    validity = env.get("NAVRL_DEPTH_VALIDITY", "").strip()
    if validity:                                   # CAM-K2 v3; unset leaves `out` exactly as before
        if validity != VALIDITY_RULE:
            raise SystemExit("realveh: NAVRL_DEPTH_VALIDITY must be unset or %s, got %r" % (VALIDITY_RULE, validity))
        if out["depth_model"] is None:
            raise SystemExit("realveh: NAVRL_DEPTH_VALIDITY needs NAVRL_DEPTH_MODEL=%s" % DEPTH_MODEL)
        try:
            out["depth_validity"] = _load_private("realveh_depth_validity_settings", VALIDITY).settings_from_env(env)
        except ValueError as exc:
            raise SystemExit("realveh: %s" % exc) from None
    else:
        stray = sorted(k for k in VALIDITY_PARAMETERS if env.get(k, "").strip())
        if stray:
            raise SystemExit("realveh: %s set while NAVRL_DEPTH_VALIDITY is off" % ", ".join(stray))
    # CAM-S1 only (NAVRL_CAMSAFE=1): blind-direction speed cap and reward terms. Off: the module is not loaded and
    # `out` is exactly as before (no camsafe key), so CAM-K2 receipts are unchanged.
    if env.get("NAVRL_CAMSAFE", "").strip() or any(k.startswith("NAVRL_CAMSAFE_") and env[k].strip() for k in env):
        try:
            camsafe = _load_private("realveh_camsafe_settings", CAMSAFE).settings_from_env(env)
        except ValueError as exc:
            raise SystemExit("realveh: %s" % exc) from None
        if camsafe is not None:
            out["camsafe"] = camsafe
    # CAM-S2a only (NAVRL_CAMS2=1, needs NAVRL_CAMSAFE=1): action delay, capture/contact rule, altitude, camera mount.
    if env.get("NAVRL_CAMS2", "").strip() or any(k.startswith("NAVRL_CAMS2_") and env[k].strip() for k in env):
        try:
            cams2 = _load_private("realveh_cams2_settings", CAMS2).settings_from_env(env)
        except ValueError as exc:
            raise SystemExit("realveh: %s" % exc) from None
        if cams2 is not None:
            if out.get("camsafe") is None:
                raise SystemExit("realveh: NAVRL_CAMS2 needs NAVRL_CAMSAFE=1")
            out["cams2"] = cams2
    return out


def write_receipt(path, info):
    try:
        with open(path, "x") as fh:
            json.dump(info, fh, indent=1, sort_keys=True)
            fh.write("\n")
    except FileExistsError:
        raise SystemExit("realveh: receipt %s exists; refusing to overwrite" % path) from None


def main():
    if len(sys.argv) < 2:
        raise SystemExit("usage: realveh_entry.py SCRIPT [ARGS...]")
    s = settings()
    script = os.path.abspath(sys.argv[1])
    if os.path.basename(script) != "runner.py":
        raise SystemExit("realveh: only runner.py may be run through this entry, got %s" % script)
    if os.path.exists(s["receipt"]):
        raise SystemExit("realveh: receipt %s exists; refusing to overwrite" % s["receipt"])
    from aerial_gym.rl_training.rl_games.navrl_import_origin import assert_runtime_origin
    from aerial_gym.rl_training.rl_games.quiet_rl_io import devnull_stdio, quiet_startup_enabled

    origin = assert_runtime_origin()
    if quiet_startup_enabled():
        with devnull_stdio():
            import isaacgym  # noqa: F401  (Isaac Gym must be imported before torch)
    else:
        import isaacgym  # noqa: F401
    import aerial_gym

    package = pathlib.Path(aerial_gym.__file__).resolve()
    if ROOT not in package.parents:
        raise SystemExit("realveh: aerial_gym imported from %s, outside %s (set PYTHONPATH)" % (package, ROOT))
    from aerial_gym.config.task_config.navrl_task_config import task_config

    if task_config.robot_name != ROBOT:
        raise SystemExit("realveh: task_config.robot_name is %r, not %s" % (task_config.robot_name, ROBOT))
    info = {"settings": s, "aerial_gym": str(package), "import_origin": origin,
            "task_robot_name": task_config.robot_name,
            "sha256": {k: sha256(v) for k, v in HASHED.items()},
            "env_navrl": {k: v for k, v in sorted(os.environ.items()) if k.startswith("NAVRL_")}}
    # 1. vehicle randomization, 2. depth model (before the ablation), 3. ablation + CAM-K1 overrides
    if s["vehicle_rand"] is not None:
        from aerial_gym.task.navrl_task import navrl_vehicle_rand as veh

        info["vehicle_rand"] = veh.install(s["vehicle_rand"], record_path=s["receipt"] + VEH_RAND_RECORD_SUFFIX)
        info["vehicle_rand_module"] = veh.__file__
    if s.get("depth_validity") is not None:     # 2b: inner wrapper, so installed before the depth model
        from aerial_gym.task.navrl_task import navrl_depth_validity as validity

        info["depth_validity"] = validity.install(s["depth_validity"])
        info["depth_validity_module"] = validity.__file__
        info["depth_validity_sha256"] = sha256(VALIDITY)
    if s["depth_model"] is not None:
        from aerial_gym.task.navrl_task import navrl_depth_model as depth

        info["depth_model"] = depth.install(s["depth_model"])
        info["depth_model_module"] = depth.__file__
    # This opt-in entry is on the allow-list of the no-production-import guards (tests/test_navrl_sensor_ablation.py,
    # as camtrain_entry.py is, and tests/test_realveh.py).
    from aerial_gym.task.navrl_task import navrl_sensor_ablation as ablation

    info["ablation"] = ablation.install(ablation.condition_from_env())
    before, after = camtrain.apply_overrides(task_config, s["camtrain"])
    if s.get("camsafe") is not None:        # 4. CAM-S1: after the ablation, whose governor wrapper must be inner
        from aerial_gym.task.navrl_task import navrl_camsafe as camsafe

        info["camsafe"] = camsafe.install(s["camsafe"], task_config)
        info["camsafe_module"] = camsafe.__file__
        info["camsafe_sha256"] = sha256(CAMSAFE)
    if s.get("cams2") is not None:          # 5. CAM-S2a: after camsafe (whose wrappers it wraps)
        from aerial_gym.task.navrl_task import navrl_cams2 as cams2

        info["cams2"] = cams2.install(s["cams2"], task_config)
        info["cams2_module"] = cams2.__file__
        info["cams2_sha256"] = sha256(CAMS2)
    info.update({"overrides_before": before, "overrides_after": after, "script": script,
                 "entry": str(pathlib.Path(__file__).resolve())})
    print("[realveh] " + json.dumps(info, sort_keys=True), flush=True)
    write_receipt(s["receipt"], info)
    sys.argv = [script] + sys.argv[2:]
    sys.path[0] = os.path.dirname(script)      # as `python runner.py` would set it
    runpy.run_path(script, run_name="__main__")


if __name__ == "__main__":
    main()
