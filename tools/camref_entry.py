"""Entry point of CAM-REF, the matched-protocol evaluation of Policy F, CAM-K1 and CAM-S2b-FT (draft preregistration
~/motar_tmp/camref_20261126/draft/prereg_camonly_ref_matched_protocol_2026-11-26.md). Selected only through its wrapper
tools/camref_python, by tools/camref_eval_run.py's launcher copy eval_navrl_camref_arms.sh.

Every policy keeps its own perception and its own trained control stack; this entry adds, per switch, the protocol
modules. NAVRL_CAMREF_POLICY names the policy and must match the checkpoint the launcher loads (NAVRL_EVAL_CHECKPOINT):

  F    Policy F (camera 87 x 58 deg at 160 x 90 + 360 deg LiDAR). Native = nothing installed (GR-0 / KROC2027 path).
       This entry installs, in this order, only what is switched on: navrl_vehicle_rand (NAVRL_VEH_RAND=1), the CAM-REF
       rules (aerial_gym/task/navrl_task/navrl_camref_rules.py: capture/contact, delay, geometry), the CAM-REF recorder
       (NAVRL_CAMREF_RECORD=1), navrl_yawclip (NAVRL_YAWCLIP=1: clips the delayed, governed command), navrl_inview_start
       (visible start; its recorder NAVRL_INVIEW_DIAG=1). No sensor ablation, no camera-FOV / reward override, no camsafe,
       cams2, depth model, P9 injector or camera-resolution key: each refuses the run. Then runner.py runs unchanged.
  K1   CAM-K1 seed 1 (camera only, 66 x 54 deg at 160 x 90). The CAM-REF rules and recorder are installed here, then
       tools/realveh_entry.py runs unchanged (main(), loaded by path): it validates and installs vehicle randomisation,
       the no_lidar ablation and the CAM-K1 FOV / reward overrides (camtrain_entry's settings), navrl_yawclip and
       navrl_inview_start, writes its own receipt (NAVRL_REALVEH_RECEIPT) and runs runner.py. Refused here: camsafe, cams2
       and its contact code / recorder, the depth model and validity rule, P9, camera-resolution keys, any FOV or reward
       weight other than CAM-K1's.
  S2B  CAM-S2b-FT (camera only, 66.7 x 52.6 deg at 312 x 234, its own stack). tools/realveh_entry.py runs unchanged with
       the native stack (vehicle randomisation, OAK-D depth model, no_lidar, camsafe, cams2 contact code, cams2,
       cams2diag, inview): the CAM-DX1 path. The CAM-REF rules are refused (cams2 is S2b-FT's own rule set); the CAM-REF
       recorder only with cams2 off (inside cams2's wrapper it would count a target contact as a capture). No yaw clip
       (1.0472 rad/s is S2b-FT's own scale).
  S3W  CAM-S3 S3-W ep18000 (stage 3, docs/plans/prereg_camonly_s3w_eval_2026-11-27.md): S2B's stack and path in its own
       trained runtime: NAVRL_YAW_RATE_MAX 1.5708 (its action scale; no yaw clip), navrl_cams3 measured depth
       (NAVRL_DEPTH_MEAS=1, false near 0, NAVRL_DEPTH_HOLE_P=0) and the sector rule (NAVRL_CAMS3_SECTOR_K=24), which
       realveh_entry validates and installs. S2B refuses the sector rule (S2b-FT's runtime reads the sampled columns);
       it may carry the measured depth (a sensor model, stage 3's S0M).
Why K1 and S2B delegate: realveh_entry is the explicit, tested installer of navrl_vehicle_rand, navrl_depth_model and
navrl_sensor_ablation (tests/test_realveh.py TestIsolation). F cannot use it (camtrain_entry's settings refuse any
condition but no_lidar), so this file installs navrl_vehicle_rand for F itself: the one new importer of that module
(see the CAM-REF package report; the isolation test's allowlist is an existing file and is not edited here).
All rules OFF and the recorder OFF: F runs exactly the native task, K1 exactly realveh_entry's K1 stack (= CAM-K1-E's
camtrain_entry stack), S2B exactly CAM-DX1's realveh stack. Unknown policies, unknown NAVRL_CAMREF_* keys and every
combination listed above refuse the run (SystemExit) before Isaac Gym is imported.
Writes NAVRL_CAMREF_RECEIPT (refusing to overwrite): every setting, the SHA-256 of every module it may load, the NAVRL_*
environment, the install records and the delegate.
usage: camref_python runner.py ARGS...
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
TASK_DIR = ROOT / "aerial_gym/task/navrl_task"
RULES = TASK_DIR / "navrl_camref_rules.py"
ROBOT = "navrl_quad"
POLICY_ENV, RECEIPT_ENV = "NAVRL_CAMREF_POLICY", "NAVRL_CAMREF_RECEIPT"
# The checkpoint each policy name binds (NAVRL_EVAL_CHECKPOINT, set by the launcher from the file it SHA-checked).
POLICY_CKPT = {"F": "f702213936601860995cf61dcc570247e72543b1976e3716055cd8ec5593ad40",
               "K1": "d7bf51a89b0e8124687e618cf8d95735713d36af2eb34fc5b8819efb14715beb",
               "S2B": "f2c80fcc2b2eaeb24e0b05d2284e74594a7ed8e7953f6b17b7dce49b09f2bbab",
               "S3W": "2626a4b24eb29b11c469904ac1a4e06de0349a04e9c83164b23a1e6308fa98ee"}
DELEGATE = {"F": None, "K1": "tools/realveh_entry.py", "S2B": "tools/realveh_entry.py", "S3W": "tools/realveh_entry.py"}
K1_PERCEPTION = {"NAVRL_SENSOR_ABLATION": "no_lidar", "NAVRL_CAMTRAIN_HFOV_DEG": 66.0, "NAVRL_CAMTRAIN_VFOV_DEG": 54.0,
                 "NAVRL_CAMTRAIN_YAW_ALIGN_WEIGHT": 0.6, "NAVRL_CAMTRAIN_VISIBILITY_BONUS": 0.03}
S2B_PERCEPTION = {"NAVRL_SENSOR_ABLATION": "no_lidar", "NAVRL_CAMSAFE": "1", "NAVRL_DEPTH_MODEL": "oakd",
                  "NAVRL_CAMERA_WIDTH": "312", "NAVRL_CAMERA_HEIGHT": "234", "NAVRL_DETECT_WIDTH": "312",
                  "NAVRL_DETECT_HEIGHT": "234", "NAVRL_CAMTRAIN_HFOV_DEG": "66.7", "NAVRL_CAMTRAIN_VFOV_DEG": "52.6",
                  "NAVRL_YAW_RATE_MAX": "1.0472"}
S3W_PERCEPTION = dict(S2B_PERCEPTION, NAVRL_YAW_RATE_MAX="1.5708")
S3W_REQUIRED = {"NAVRL_DEPTH_MEAS": "1", "NAVRL_DEPTH_MEAS_FALSE_NEAR": "0", "NAVRL_DEPTH_HOLE_P": "0",
                "NAVRL_CAMS3_SECTOR_K": "24"}
# Keys that change a policy's perception or bring in S2b-FT's own modules; refused for F (all) and K1 (all but the
# ablation / camtrain keys, which K1 must carry with its own values).
PERCEPTION_PREFIXES = ("NAVRL_CAMTRAIN_", "NAVRL_CAMSAFE", "NAVRL_CAMS2", "NAVRL_DEPTH_VALIDITY", "NAVRL_P9_",
                       "NAVRL_DEPTH_MEAS", "NAVRL_CAMS3_")              # the last two: CAM-S3 rules (stage 3)
PERCEPTION_KEYS = ("NAVRL_SENSOR_ABLATION", "NAVRL_DEPTH_MODEL", "NAVRL_DEPTH_MINZ_M", "NAVRL_DEPTH_MAXZ_M",
                   "NAVRL_DEPTH_SIGMA_D_PX", "NAVRL_DEPTH_FB_PXM", "NAVRL_DEPTH_HOLE_P", "NAVRL_DEPTH_SEED",
                   "NAVRL_CAMERA_WIDTH", "NAVRL_CAMERA_HEIGHT", "NAVRL_DETECT_WIDTH", "NAVRL_DETECT_HEIGHT",
                   "NAVRL_ACTION_DELAY_STEPS_MAX")


def _load_private(name, path):
    """A stdlib-only module by file path, under a private name and not registered in sys.modules."""
    spec = importlib.util.spec_from_file_location(name, str(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


RV = _load_private("camref_realveh_entry", TOOLS / "realveh_entry.py")      # the delegate and its module paths
R = _load_private("camref_rules_settings", RULES)                          # settings parser only (stdlib)
HASHED = dict(RV.HASHED, camref_entry=pathlib.Path(__file__).resolve(), camref_python=TOOLS / "camref_python",
              camref_rules=RULES, yawclip=RV.YAWCLIP, inview=RV.INVIEW, camsafe=RV.CAMSAFE, cams2=RV.CAMS2,
              cams2diag=RV.CAMS2DIAG, contactcode=RV.CONTACTCODE)


def sha256(path):
    return hashlib.sha256(pathlib.Path(path).read_bytes()).hexdigest()


def _set(env, key):
    return bool(env.get(key, "").strip())


def _stray(env, keys=(), prefixes=()):
    return sorted(k for k in env if env[k].strip() and (k in keys or k.startswith(tuple(prefixes))))


def _module_settings(env, path, *readers):
    """Run the named settings readers of a module loaded by path; ValueError becomes SystemExit."""
    module = _load_private("camref_settings_" + pathlib.Path(path).stem, path)
    try:
        return [getattr(module, r)(env) for r in readers], module
    except ValueError as exc:
        raise SystemExit("camref: %s" % exc) from None


def f_settings(env, receipt):
    """Policy F's optional modules (it has no perception module of its own here)."""
    stray = _stray(env, PERCEPTION_KEYS, PERCEPTION_PREFIXES + ("NAVRL_REALVEH_",))
    if stray:
        raise SystemExit("camref: policy F refuses %s (F keeps its own perception and stack)" % stray)
    out = {}
    veh = env.get("NAVRL_VEH_RAND", "").strip()
    if veh not in ("", "0", "1"):
        raise SystemExit("camref: NAVRL_VEH_RAND must be unset, 0 or 1, got %r" % veh)
    if veh == "1":
        (vs,), module = _module_settings(env, RV.VEH_RAND, "settings_from_env")
        worst = module.worst_case_thrust_to_weight(vs)
        if worst < RV.MIN_WORST_CASE_THRUST_TO_WEIGHT:
            raise SystemExit("camref: worst-case thrust-to-weight %.4f is below %g" % (worst,
                                                                                      RV.MIN_WORST_CASE_THRUST_TO_WEIGHT))
        if os.path.exists(receipt + RV.VEH_RAND_RECORD_SUFFIX):
            raise SystemExit("camref: vehicle-randomization record %s exists" % (receipt + RV.VEH_RAND_RECORD_SUFFIX))
        out["vehicle_rand"], out["worst_case_thrust_to_weight"] = vs, worst
    else:
        stray = _stray(env, prefixes=("NAVRL_VEH_RAND_",))
        if stray:
            raise SystemExit("camref: %s set while NAVRL_VEH_RAND is off" % ", ".join(stray))
        out["vehicle_rand"] = None
    (out["yawclip"],), _ = _module_settings(env, RV.YAWCLIP, "settings_from_env")
    (out["inview_start"], out["inview_diag"]), _ = _module_settings(env, RV.INVIEW, "start_settings_from_env",
                                                                    "diag_settings_from_env")
    return out


def settings(environ=None):
    """Every setting, validated without importing torch, Isaac Gym or aerial_gym (SystemExit on any refusal)."""
    env = dict(os.environ if environ is None else environ)
    policy = env.get(POLICY_ENV, "").strip()
    if policy not in POLICY_CKPT:
        raise SystemExit("camref: %s must be one of %s, got %r" % (POLICY_ENV, sorted(POLICY_CKPT), policy))
    receipt = env.get(RECEIPT_ENV, "").strip()
    if not receipt or not os.path.isabs(receipt):
        raise SystemExit("camref: %s must be an absolute path" % RECEIPT_ENV)
    if env.get("NAVRL_EVAL_CHECKPOINT", "").strip() != POLICY_CKPT[policy]:
        raise SystemExit("camref: NAVRL_EVAL_CHECKPOINT %r is not policy %s's checkpoint %s..."
                         % (env.get("NAVRL_EVAL_CHECKPOINT"), policy, POLICY_CKPT[policy][:12]))
    if env.get("NAVRL_ROBOT", "").strip() not in ("", ROBOT):
        raise SystemExit("camref: NAVRL_ROBOT must be unset or %s" % ROBOT)
    if _set(env, "NAVRL_CAMTRAIN_RECEIPT"):
        raise SystemExit("camref: NAVRL_CAMTRAIN_RECEIPT must be unset (realveh_entry writes the one receipt)")
    try:
        rules = R.settings_from_env(env)
    except ValueError as exc:
        raise SystemExit("camref: %s" % exc) from None
    out = {"policy": policy, "receipt": receipt, "camref": rules, "delegate": DELEGATE[policy]}
    if policy == "F":
        out.update(f_settings(env, receipt))
        return out
    realveh_receipt = env.get("NAVRL_REALVEH_RECEIPT", "").strip()
    if not realveh_receipt or realveh_receipt == receipt:
        raise SystemExit("camref: policy %s needs its own NAVRL_REALVEH_RECEIPT (the delegate's receipt)" % policy)
    if policy == "K1":
        stray = _stray(env, set(PERCEPTION_KEYS) - set(K1_PERCEPTION),
                       tuple(p for p in PERCEPTION_PREFIXES if p != "NAVRL_CAMTRAIN_"))
        if stray:
            raise SystemExit("camref: policy K1 refuses %s (K1 keeps its own perception and stack)" % stray)
        for k, v in K1_PERCEPTION.items():
            got = env.get(k, "").strip()
            try:
                ok = (got == v) if isinstance(v, str) else (got != "" and abs(float(got) - v) < 1e-9)
            except ValueError:
                ok = False
            if not ok:
                raise SystemExit("camref: policy K1 needs %s=%s (its own perception), got %r" % (k, v, got))
    else:                                                     # S2B, S3W
        for k, v in (S2B_PERCEPTION if policy == "S2B" else dict(S3W_PERCEPTION, **S3W_REQUIRED)).items():
            if env.get(k, "").strip() != v:
                raise SystemExit("camref: policy %s needs %s=%s (its own perception and stack), got %r"
                                 % (policy, k, v, env.get(k)))
        if policy == "S2B" and _set(env, "NAVRL_CAMS3_SECTOR_K"):
            raise SystemExit("camref: policy S2B refuses NAVRL_CAMS3_SECTOR_K (its runtime reads the sampled columns)")
        if not _set(env, "NAVRL_P9_ERROR_MODEL"):
            raise SystemExit("camref: policy %s needs its P9 injector (NAVRL_P9_ERROR_MODEL)" % policy)
        if R.rules_on(rules):
            raise SystemExit("camref: policy %s refuses the CAM-REF rules (cams2 is its own rule set)" % policy)
        if _stray(env, prefixes=("NAVRL_YAWCLIP",)):
            raise SystemExit("camref: policy %s refuses NAVRL_YAWCLIP (its action scale is its own)" % policy)
        if rules and rules.get("record") is not None and env.get("NAVRL_CAMS2", "").strip() == "1":
            raise SystemExit("camref: the CAM-REF recorder would sit inside navrl_cams2; use navrl_cams2diag")
    out["realveh"] = RV.settings(env)                       # the delegate's own validation (SystemExit on a refusal)
    return out


def write_receipt(path, info):
    try:
        with open(path, "x") as fh:
            json.dump(info, fh, indent=1, sort_keys=True, default=str)
            fh.write("\n")
    except FileExistsError:
        raise SystemExit("camref: receipt %s exists; refusing to overwrite" % path) from None


def main():
    if len(sys.argv) < 2:
        raise SystemExit("usage: camref_entry.py SCRIPT [ARGS...]")
    s = settings()
    script = os.path.abspath(sys.argv[1])
    if os.path.basename(script) != "runner.py":
        raise SystemExit("camref: only runner.py may be run through this entry, got %s" % script)
    if os.path.exists(s["receipt"]):
        raise SystemExit("camref: receipt %s exists; refusing to overwrite" % s["receipt"])
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
        raise SystemExit("camref: aerial_gym imported from %s, outside %s (set PYTHONPATH)" % (package, ROOT))
    from aerial_gym.config.task_config.navrl_task_config import task_config

    if task_config.robot_name != ROBOT:
        raise SystemExit("camref: task_config.robot_name is %r, not %s" % (task_config.robot_name, ROBOT))
    policy, rules = s["policy"], s["camref"]
    info = {"settings": s, "policy": policy, "aerial_gym": str(package), "import_origin": origin,
            "task_robot_name": task_config.robot_name, "sha256": {k: sha256(v) for k, v in HASHED.items()},
            "env_navrl": {k: v for k, v in sorted(os.environ.items()) if k.startswith("NAVRL_")},
            "delegate": s["delegate"], "installed": [], "script": script, "entry": str(pathlib.Path(__file__).resolve())}
    if policy == "F" and s["vehicle_rand"] is not None:          # 1. vehicle randomisation (F only; K1 / S2B: delegate)
        from aerial_gym.task.navrl_task import navrl_vehicle_rand as veh

        info["vehicle_rand"] = veh.install(s["vehicle_rand"], record_path=s["receipt"] + RV.VEH_RAND_RECORD_SUFFIX)
        info["installed"].append("vehicle_rand")
    if R.rules_on(rules) or (rules and rules.get("record") is not None):
        from aerial_gym.task.navrl_task import navrl_camref_rules as camref

        if R.rules_on(rules):                                    # 2. the CAM-REF rules (F, K1)
            info["camref_rules"] = camref.install_rules(rules, task_config)
            info["installed"].append("camref_rules")
        if rules.get("record") is not None:                      # 3. the recorder, outside the rules
            info["camref_record"] = camref.install_record(rules)
            info["installed"].append("camref_record")
        info["camref_module"] = camref.__file__
    if policy == "F":
        if s["yawclip"] is not None:                             # 4. yaw clip on the delayed, governed command
            from aerial_gym.task.navrl_task import navrl_yawclip as yawclip

            info["yawclip"] = yawclip.install(s["yawclip"], task_config)
            info["installed"].append("yawclip")
        if s["inview_start"] is not None or s["inview_diag"] is not None:   # 5. visible start and its recorder
            from aerial_gym.task.navrl_task import navrl_inview_start as inview

            if s["inview_start"] is not None:
                info["inview_start"] = inview.install_start(s["inview_start"], task_config)
                info["installed"].append("inview_start")
            if s["inview_diag"] is not None:
                info["inview_diag"] = inview.install_diag(s["inview_diag"])
                info["installed"].append("inview_diag")
    print("[camref-entry] " + json.dumps({k: info[k] for k in ("policy", "installed", "delegate")}, sort_keys=True),
          flush=True)
    write_receipt(s["receipt"], info)
    if s["delegate"] is not None:                                # K1, S2B: realveh_entry, unchanged, runs runner.py
        RV.main()
        return
    sys.argv = [script] + sys.argv[2:]
    sys.path[0] = os.path.dirname(script)      # as `python runner.py` would set it
    runpy.run_path(script, run_name="__main__")


if __name__ == "__main__":
    main()
