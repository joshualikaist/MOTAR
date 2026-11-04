"""Entry point of the camera-only study's evaluations (D-2026-11-04-3): install the opt-in LiDAR ablation named by
NAVRL_SENSOR_ABLATION, then run the given script (runner.py) unchanged as __main__.

Selected only through tools/camonly_python, which a camera-only launcher passes to play_navrl.sh as PYTHON; every
other launcher and the pinned files are untouched, so the ablation is OFF everywhere else. The order mirrors
runner.py: the import-origin guard first, Isaac Gym before torch, then the task modules are imported and wrapped.
The installed condition and the module's SHA-256 are printed and, if NAVRL_SENSOR_ABLATION_RECEIPT is set, written
there (refusing to overwrite).
usage: camonly_python runner.py ARGS...
"""
import hashlib
import json
import os
import pathlib
import runpy
import sys


def main():
    if len(sys.argv) < 2:
        raise SystemExit("usage: camonly_entry.py SCRIPT [ARGS...]")
    script = os.path.abspath(sys.argv[1])
    from aerial_gym.rl_training.rl_games.navrl_import_origin import assert_runtime_origin
    from aerial_gym.rl_training.rl_games.quiet_rl_io import devnull_stdio, quiet_startup_enabled

    assert_runtime_origin()
    if quiet_startup_enabled():
        with devnull_stdio():
            import isaacgym  # noqa: F401  (Isaac Gym must be imported before torch)
    else:
        import isaacgym  # noqa: F401
    from aerial_gym.task.navrl_task import navrl_sensor_ablation as ablation

    info = ablation.install(ablation.condition_from_env())
    info["module_sha256"] = hashlib.sha256(pathlib.Path(ablation.__file__).read_bytes()).hexdigest()
    info["module"] = ablation.__file__
    info["script"] = script
    print("[camonly] " + json.dumps(info, sort_keys=True), flush=True)
    receipt = os.environ.get("NAVRL_SENSOR_ABLATION_RECEIPT", "").strip()
    if receipt:
        with open(receipt, "x") as fh:
            json.dump(info, fh, indent=1, sort_keys=True)
            fh.write("\n")
    sys.argv = [script] + sys.argv[2:]
    sys.path[0] = os.path.dirname(script)      # as `python runner.py` would set it
    runpy.run_path(script, run_name="__main__")


if __name__ == "__main__":
    main()
