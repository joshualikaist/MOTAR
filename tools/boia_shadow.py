#!/usr/bin/env python3
"""Q01 CPU preflight / bounded GPU supervisor. No simulator import outside _worker.

GPU subcommands require --execute-gpu and exact --approved-commit. This switch
is an accidental-launch guard; actual user approval is still required separately.
"""
import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
PLAN = ROOT / "docs/plans/boia_shadow_q01_v1.json"
PYTHON = "/home/fair/miniconda3/envs/aerialgym/bin/python"
CODE = ("motar_eval/boia_shadow.py", "tools/boia_shadow.py", "tests/test_boia_shadow.py",
        "tests/test_boia_shadow_runner.py", "docs/plans/boia_shadow_q01_v1.json",
        "docs/plans/boia_shadow_q01_v1.md")
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


def sha(path):
    h = hashlib.sha256()
    with Path(path).open("rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def load(path):
    return json.loads(Path(path).read_text())


def write_new(path, value):
    with Path(path).open("x") as f:
        json.dump(value, f, indent=2, sort_keys=True, allow_nan=False)
        f.write("\n")


def git(*args, root=ROOT):
    return subprocess.check_output(["git", "-C", str(root), *args], text=True).rstrip("\n")


def nearest_existing(path):
    path = Path(path)
    while not path.exists():
        path = path.parent
    return path


def check(condition, message):
    if not condition:
        raise ValueError(message)


def validate_plan(p):
    check(p["schema"] == "motar.boia.shadow.q01.v1" and p["run_id"] == "BOIA-SHADOW-Q01", "unknown plan")
    expected = dict(num_envs=1, seed=4901, density=70, target="E2_direct_l2_v1", initial_view="visible",
                    initial_distance_m=10, step_dt_s=.1, physics_dt_s=.01, physics_substeps=10,
                    episode_actions=600, lineage="off")
    check(p["profile"] == expected, "Q01 profile changed")
    check(p["limits"] == dict(campaign_wall_s=720, disk_preflight_bytes=3300000000,
          disk_stop_bytes=3000000000, output_bytes=256000000, vram_ceiling_mib=6656,
          vram_reserve_mib=768, poll_s=.5, shadow_stride=5, sample_cap_s=2., slow_sample_ms=500.,
          slow_streak=3, ended_episodes_per_process=1, no_retry=True), "Q01 resource contract changed")
    check(p["stages"] == {"S0": dict(actions=20, wall_s=120, shadow=True, snapshot_cap=2),
                         "S1_OFF": dict(actions=200, wall_s=300, shadow=False, snapshot_cap=3),
                         "S1_ON": dict(actions=200, wall_s=300, shadow=True, snapshot_cap=3)}, "stage caps changed")
    required = dict(NAVRL_SEED="4901", NAVRL_NUM_BARS="70", NAVRL_SENSOR_LINEAGE="off",
                    NAVRL_TARGET_DISTANCE_BACKEND="direct_l2_v1", NAVRL_PERCEPTION_PERTURB="0",
                    NAVRL_TARGET_DYNAMICS="bounded", NAVRL_TARGET_BEHAVIOR_LEVEL="e2_obstacle_aware",
                    NAVRL_TARGET_PATTERN="waypoint", NAVRL_EPISODE_LEN_STEPS="600",
                    NAVRL_KROC2027_INITIAL_VIEW="visible", NAVRL_KROC2027_INITIAL_DISTANCE_M="10.0",
                    NAVRL_VISION="1", NAVRL_PERCEPTION="1", NAVRL_LIDAR_HBEAMS="72",
                    NAVRL_LIDAR_VBEAMS="4", NAVRL_LIDAR_RANGE="12", NAVRL_SPEED_GOVERNOR="riskcap",
                    NAVRL_BULK_EVAL="0", NAVRL_KROC2027_RECORDER="0")
    check(all(p["runtime_env"].get(k) == v for k, v in required.items()), "runtime env/profile mismatch")
    check(all(k.startswith("NAVRL_") and type(v) is str for k, v in p["runtime_env"].items()), "unexpected environment key/type")
    check(p["frozen"]["policy_f_sha256"] == "f702213936601860995cf61dcc570247e72543b1976e3716055cd8ec5593ad40", "checkpoint binding changed")
    out = Path(p["output_root"])
    check(out.is_absolute() and out.name == p["run_id"] and ROOT not in out.resolve().parents,
          "output must be dedicated persistent directory outside source")


def checkpoint(p):
    return ROOT / "aerial_gym/rl_training/rl_games" / p["frozen"]["policy_f_rel_to_rl_games"]


def process_env(p, parent_pid=None, config_sha=None):
    prefixes = ("NAVRL_", "AERIAL_GYM_", "AERIAL_RUN", "AERIAL_TASK", "BOIA_SHADOW_")
    env = {k: v for k, v in os.environ.items() if not k.startswith(prefixes)
           and k not in ("TASK", "FILE", "NUM_ENVS", "HEADLESS", "CKPT", "PYTHONPATH", "PYTHONHOME")}
    env.update(p["runtime_env"])
    env.update(PYTHONPATH=str(ROOT), PYTHONNOUSERSITE="1", PYTHONDONTWRITEBYTECODE="1",
               PYTHONHASHSEED="4901", OMP_NUM_THREADS="1", MKL_NUM_THREADS="1",
               TASK=p["frozen"]["task"], FILE=p["frozen"]["player_yaml"],
               AERIAL_GYM_SIM_NAME=p["frozen"]["sim_name"], HEADLESS="True")
    env["PATH"] = str(Path(PYTHON).parent) + os.pathsep + env.get("PATH", "")
    if parent_pid is not None:
        env["BOIA_SHADOW_PARENT_PID"] = str(parent_pid)
        env["BOIA_SHADOW_CONFIG_SHA"] = config_sha
    return env


def preflight(p, approved):
    """CPU only: no torch/Isaac imports, checkpoint unpickling or GPU queries."""
    validate_plan(p)
    head = git("rev-parse", "HEAD")
    dirty = git("status", "--porcelain", "--untracked-files=no")
    extra = git("ls-files", "--others", "--exclude-standard", "--", "aerial_gym", "motar_eval", "tools", "tests")
    tracked = set(git("ls-files").splitlines())
    checks = dict(branch=git("branch", "--show-current") == "codex/mt-p1", exact_HEAD=head == approved,
                  clean_tracked_tree=not dirty, no_untracked_runtime=not extra,
                  committed_inputs=all(x in tracked for x in CODE),
                  checkpoint_sha=sha(checkpoint(p)) == p["frozen"]["policy_f_sha256"],
                  python_available=Path(PYTHON).is_file(),
                  disk=shutil.disk_usage(nearest_existing(p["output_root"])).free >= p["limits"]["disk_preflight_bytes"])
    origins = {}
    for name in ("aerial_gym", "motar_eval"):
        spec = importlib.util.find_spec(name)  # top-level resolution does not execute __init__.
        origin = None if spec is None else spec.origin
        origins[name] = origin
        checks[name + "_origin"] = bool(origin and ROOT in Path(origin).resolve().parents)
    return dict(status="PASS_CPU_PREFLIGHT" if all(checks.values()) else "HOLD_PRE_GPU", checks=checks,
                head=head, config_sha256=sha(PLAN), checkpoint_sha256=p["frozen"]["policy_f_sha256"],
                code_sha256={x: sha(ROOT / x) for x in CODE if (ROOT / x).is_file()},
                import_origins=origins, free_bytes=shutil.disk_usage(ROOT).free,
                GPU="NOT_RUN", runtime_parity="NOT_TESTED", physical_safety="NOT_QUALIFIED", G5="HOLD")


def guard_sources(approved, config_sha):
    check(git("rev-parse", "HEAD") == approved, "SOURCE_HEAD_CHANGED")
    check(not git("status", "--porcelain", "--untracked-files=no"), "TRACKED_TREE_CHANGED")
    check(not git("ls-files", "--others", "--exclude-standard", "--", "aerial_gym", "motar_eval", "tools", "tests"),
          "UNTRACKED_RUNTIME_SOURCE")
    check(sha(PLAN) == config_sha, "CONFIG_CHANGED")


def directory_bytes(path):
    return sum(x.stat().st_size for x in Path(path).rglob("*") if x.is_file())


def gpu_state():
    text = subprocess.check_output(["nvidia-smi", "--query-gpu=name,memory.total,memory.used",
                                    "--format=csv,noheader,nounits"], text=True, timeout=5)
    rows = text.strip().splitlines()
    check(len(rows) == 1, "Q01 requires an unambiguous single GPU")
    name, total, used = [x.strip() for x in rows[0].split(",")]
    check("3070" in name, "unqualified GPU platform")
    return dict(name=name, total_mib=int(total), used_mib=int(used))


def manifest(directory, destination):
    products = {str(x.relative_to(directory)): dict(bytes=x.stat().st_size, sha256=sha(x))
                for x in sorted(Path(directory).rglob("*")) if x.is_file() and x != destination}
    write_new(destination, dict(files=products))


def read_rows(path):
    with Path(path).open() as f:
        return [json.loads(line) for line in f]


def compare(off, on):
    a, b = read_rows(Path(off) / "parity.jsonl"), read_rows(Path(on) / "parity.jsonl")
    check(a and len(a) == len(b), "OFF_ON_LENGTH_OR_EMPTY_TRACE")
    for i, (left, right) in enumerate(zip(a, b)):
        check(left == right, "OFF_ON_FIRST_MISMATCH_ACTION_%d" % i)
    return dict(status="PASS_EXACT_OFF_ON_PARITY", actions=len(a), tolerance=0, excluded_rows=0)


def compare_qualified(p, approved):
    root = Path(p["output_root"])
    guard_sources(approved, sha(PLAN))
    for stage in ("S1_OFF", "S1_ON"):
        meta = load(root / stage / "process_done.json")
        check(meta["status"] == "COMPLETED" and meta["source"] == approved and
              meta["config_sha"] == sha(PLAN) and meta["checkpoint"] == p["frozen"]["policy_f_sha256"],
              "PARITY_METADATA_BINDING_MISMATCH")
    return compare(root / "S1_OFF", root / "S1_ON")


def latency_summary(values):
    if not values:
        return dict(n=0, P50=None, P95=None, P99=None, maximum=None)
    x = sorted(values)
    def quantile(q):
        z = (len(x)-1)*q; lo = int(z); hi = min(lo+1, len(x)-1)
        return x[lo] + (x[hi]-x[lo])*(z-lo)
    return dict(n=len(x), P50=quantile(.5), P95=quantile(.95), P99=quantile(.99), maximum=x[-1])


def _terminate(proc):
    if proc.poll() is None:
        os.killpg(proc.pid, signal.SIGTERM)
        try:
            proc.wait(timeout=1)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
            proc.wait(timeout=1)


def run_stage(p, stage, approved):
    report = preflight(p, approved)
    check(report["status"] == "PASS_CPU_PREFLIGHT", "CPU_PREFLIGHT_FAILED: " + repr(report["checks"]))
    root, spec = Path(p["output_root"]), p["stages"][stage]
    if stage == "S0":
        check(not root.exists(), "CAMPAIGN_PATH_EXISTS_NO_RETRY")
        root.mkdir(parents=True, exist_ok=False)
        write_new(root / "campaign_start.json", dict(preflight=report, runtime_env=p["runtime_env"],
                                                     run_id=p["run_id"], source=approved))
    else:
        start = load(root / "campaign_start.json")
        check(start["source"] == approved and start["preflight"]["config_sha256"] == report["config_sha256"], "CAMPAIGN_BINDING_CHANGED")
        for previous in (("S0",) if stage == "S1_OFF" else ("S0", "S1_OFF")):
            check(load(root / (previous + "_supervisor.json"))["status"] == "COMPLETED", "PREVIOUS_STAGE_NOT_PASS")
    check(not (root / stage).exists() and not (root / (stage + "_supervisor.json")).exists(), "STAGE_EXISTS_NO_RETRY")
    used = sum(load(x)["wall_s"] for x in root.glob("*_supervisor.json"))
    remaining = min(spec["wall_s"], p["limits"]["campaign_wall_s"] - used)
    check(remaining > 2, "CAMPAIGN_BUDGET_EXHAUSTED")
    command = [PYTHON, "-B", str(Path(__file__).resolve()), "_worker", "--stage", stage,
               "--approved-commit", approved]
    start_time, peak, error, proc, ceiling = time.monotonic(), 0, None, None, None
    try:
        gpu = gpu_state()
        ceiling = min(p["limits"]["vram_ceiling_mib"], gpu["total_mib"] - p["limits"]["vram_reserve_mib"])
        check(gpu["used_mib"] < ceiling, "INSUFFICIENT_VRAM_HEADROOM")
        peak = gpu["used_mib"]
        with (root / (stage + ".log")).open("x") as log:
            proc = subprocess.Popen(command, cwd=str(ROOT), env=process_env(p, os.getpid(), report["config_sha256"]),
                                    stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            while proc.poll() is None:
                if time.monotonic() - start_time >= remaining - 2:
                    raise ValueError("WALL_WATCHDOG_CENSORING_NOT_TASK_TIMEOUT")
                guard_sources(approved, report["config_sha256"])
                check(shutil.disk_usage(root).free >= p["limits"]["disk_stop_bytes"], "DISK_BELOW_3GB")
                check(directory_bytes(root) <= p["limits"]["output_bytes"], "CAMPAIGN_OUTPUT_CAP")
                peak = max(peak, gpu_state()["used_mib"])
                check(peak <= ceiling, "DEVICE_VRAM_CEILING")
                time.sleep(p["limits"]["poll_s"])
        check(proc.returncode == 0, "WORKER_NONZERO_EXIT_%s" % proc.returncode)
        guard_sources(approved, report["config_sha256"])
        done = load(root / stage / "process_done.json")
        check(done["status"] == "COMPLETED" and done["source"] == approved and
              done["config_sha"] == report["config_sha256"], "WORKER_BINDING_OR_COMPLETION")
        if stage == "S1_ON":
            write_new(root / "parity_result.json", compare_qualified(p, approved))
    except BaseException as exc:
        error = type(exc).__name__ + ": " + str(exc)
    finally:
        if proc is not None:
            _terminate(proc)
        report = dict(stage=stage, status="STOPPED_INTEGRITY" if error else "COMPLETED", reason=error,
                      wall_s=time.monotonic() - start_time, peak_device_mib=peak, vram_cap_mib=ceiling,
                      source=approved, config_sha=sha(PLAN), command=command, GPU_parity="NOT_TESTED" if stage != "S1_ON" or error else "PASS",
                      checkpoint_sha256_after=sha(checkpoint(p)), physical_safety="NOT_QUALIFIED", G5="HOLD")
        if report["checkpoint_sha256_after"] != p["frozen"]["policy_f_sha256"]:
            report.update(status="STOPPED_INTEGRITY", reason="CHECKPOINT_CHANGED")
        write_new(root / (stage + "_supervisor.json"), report)
        manifest(root, root / (stage + "_RAW_MANIFEST.json"))
    return report


def runtime_profile(task, player, p):
    """Read-only runtime attestation, intentionally fails closed on new profiles."""
    pc, detector, tc = task.perception, task.detector, task.task_config
    cfg = task.sim_env.robot_manager.warp_sensor.cfg
    checks = dict(N=int(task.num_envs) == 1, actor=tuple(task.task_obs["observations"].shape) == (1, 898),
                  inference=not player.model.training and not bool(player.is_rnn),
                  step_dt=float(task.step_dt) == .1, episode_cap=int(tc.episode_len_steps) == 600,
                  vision=bool(task.vision_mode) and bool(task.perception_mode),
                  lineage=os.environ.get("NAVRL_SENSOR_LINEAGE") == "off",
                  target_backend=task._target_distance_backend == "direct_l2_v1",
                  target_dynamics=task._target_dynamics == "bounded" and not task._physical_target,
                  target_level=task._target_behavior_level == "e2_obstacle_aware",
                  bars=int(task.n_bars_active) == 70,
                  initial_view=(getattr(task._kroc_initial_view, "view", None) == "visible" and
                                getattr(task._kroc_initial_view, "distance_m", None) == 10.),
                  perturbations=not bool(task.perception_cfg.enable_perturbations),
                  latency=float(pc.detection_latency_s) == 0.,
                  raster=(detector.height, detector.width, detector.detect_height, detector.detect_width) == (90, 160, 90, 160),
                  lidar=(int(cfg.height), int(cfg.width), float(cfg.max_range), float(cfg.min_range)) == (4, 72, 12., .2),
                  governor=task.speed_governor_cfg.mode == "riskcap",
                  no_supervisor=getattr(task, "_kroc_search", None) is None,
                  no_manual=not task.interactive_mode,
                  no_other_recorder=getattr(task, "_kroc_adapter", None) is None)
    checks["physics_dt"] = float(task.sim_env.sim_config.sim.dt) == .01
    checks["physics_substeps"] = (int(task.sim_env.cfg.env.num_physics_steps_per_env_step_mean) == 10 and
                                  float(task.sim_env.cfg.env.num_physics_steps_per_env_step_std) == 0.)
    checks["camera_mount"] = tuple(pc.camera_offset.reshape(-1).detach().cpu().tolist()) == tuple(
        torch_value for torch_value in (.10000000149011612, 0., .029999999329447746))
    checks["pose_error"] = all(float(getattr(pc, name)) == 0. for name in ("pose_noise_pos_m", "pose_noise_yaw_deg"))
    checks["no_empirical_model"] = getattr(pc, "empirical_error", None) is None and not pc._detector_noise_active
    checks["native_lidar_noise_off"] = not bool(cfg.sensor_noise.enable_sensor_noise)
    checks["native_lidar_geometry"] = (cfg.yaw_only_attach and not cfg.randomize_placement and
        cfg.normalize_range and not cfg.return_pointcloud and not cfg.pointcloud_in_world_frame and
        tuple(cfg.nominal_position) == (0.,0.,0.) and tuple(cfg.euler_frame_rot_deg) == (0.,0.,0.) and
        (cfg.horizontal_fov_deg_min,cfg.horizontal_fov_deg_max,cfg.vertical_fov_deg_min,cfg.vertical_fov_deg_max) == (-175.,180.,-10.,20.))
    checks["target_proxy_radii"] = float(cfg.target_radius) == .20 and float(detector.target_radius) == .15
    checks["appearance_off"] = all(float(getattr(detector,name)) == 0. for name in
        ("app_hue_deg","app_light_gain","app_albedo_jitter","app_texture_std","app_motion_blur","mount_rot_deg","mount_trans_m"))
    check(all(checks.values()), "RUNTIME_PROFILE_FAILED: " + repr(checks))
    return checks


def worker(p, stage, approved):
    check(os.environ.get("BOIA_SHADOW_PARENT_PID") == str(os.getppid()), "WORKER_REQUIRES_SUPERVISOR")
    check(os.environ.get("BOIA_SHADOW_CONFIG_SHA") == sha(PLAN), "WORKER_CONFIG_SHA")
    guard_sources(approved, sha(PLAN))
    import isaacgym  # must precede torch; this function is never called by CPU preflight/tests.
    import torch
    if str(ROOT / "tools") not in sys.path:
        sys.path.insert(0, str(ROOT / "tools"))
    import eq1d_fixture_launcher as eq
    from motar_eval.eq1d_telemetry import rms_digest
    from motar_eval.boia_shadow import Binding, EvaluatorTap, EventLogger, Recorder, TensorBytes, rng_fingerprint
    root, spec = Path(p["output_root"]), p["stages"][stage]
    log, tap, error, recorder = None, None, None, None
    times, start = [], time.perf_counter()
    try:
        check(sha(checkpoint(p)) == p["frozen"]["policy_f_sha256"], "CHECKPOINT_BEFORE_LOAD")
        player, _ = eq.build_player(1, checkpoint(p))
        task = eq.find_task(player)
        origins = eq.check_origins()
        profile = runtime_profile(task, player, p)
        rms = eq.find_rms(player.model)
        rms0 = rms_digest(rms)
        check(rms0 == p["frozen"]["rms_digest_sha256"], "RMS_BINDING")
        log = EventLogger(root / stage, max_bytes=p["limits"]["output_bytes"] // 3)
        reference = read_rows(root / "S1_OFF/parity.jsonl") if stage == "S1_ON" else None
        recorder = Recorder(log, spec["shadow"], 5, spec["snapshot_cap"], reference)
        binding = Binding(approved, sha(PLAN), p["frozen"]["policy_f_sha256"])
        torch.cuda.reset_peak_memory_stats()
        with EvaluatorTap(task, recorder, binding) as tap:
            obs = player.env_reset(player.env)
            player.get_batch_size(obs, 1)  # N1 otherwise loses its actor/action batch dimension.
            for step in range(spec["actions"]):
                before = time.perf_counter()
                tap.actor_rng_before = rng_fingerprint(task.device)
                actor = obs["obs"] if isinstance(obs, dict) else obs
                check(TensorBytes.copy(actor) == TensorBytes.copy(task.task_obs["observations"]), "ACTOR_INPUT_NOT_CURRENT_TASK_PACKET")
                action = player.get_action(obs, is_deterministic=True)
                check(rms_digest(rms) == rms0 and not player.model.training, "NORMALIZATION_OR_POLICY_STATE_MUTATED")
                obs, reward, done, info = player.env_step(player.env, action)
                times.append((time.perf_counter() - before) * 1000)
                if recorder.episodes_ended >= 1:
                    break
            recorder.censor("EPISODE_LIMIT" if recorder.episodes_ended else "PROCESS_ACTION_CAP")
        torch.cuda.synchronize()
        check(rms_digest(rms) == rms0, "FINAL_RMS_CHANGED")
        write_new(root / stage / "process_done.json", dict(status="COMPLETED", source=approved, config_sha=sha(PLAN),
                  checkpoint=p["frozen"]["policy_f_sha256"], rms_start=rms0, rms_end=rms_digest(rms),
                  profile=profile, origins=origins, actions=recorder.completed, ended_episodes=recorder.episodes_ended,
                  observe_calls=tap.observe_calls, transform_calls=tap.transform_calls, compute_calls=tap.compute_calls,
                  lidar_calls=tap.lidar_calls, rgbd_calls=tap.rgbd_calls,
                  loop_ms=times, shadow_diagnostic_only_ms=recorder.latencies,
                  observer_phase_times=tap.observer_times,
                  cold_first_observer_ms=tap.observer_times[0]["total_observer_ms"] if tap.observer_times else None,
                  warm_observer_ms=latency_summary([x["total_observer_ms"] for x in tap.observer_times[1:]]),
                  warm_full_step_ms=latency_summary(times[1:]),
                  latency_scope="HOST_SYNC_COPY_SCHEMA_DIAGNOSTIC_SHA_LOG; native physics/inference excluded from observer; no CUDA kernel-only claim",
                  peak_torch_allocated_bytes=torch.cuda.max_memory_allocated(), snapshots=recorder.snapshots,
                  wall_s=time.perf_counter()-start, full_planner="SKIPPED_UNSUPPORTED",
                  runtime_parity="PENDING_SUPERVISOR_COMPARISON", physical_safety="NOT_QUALIFIED", G5="HOLD"))
    except BaseException as exc:
        error = type(exc).__name__ + ": " + str(exc)
        (root / stage).mkdir(parents=True, exist_ok=True)
        write_new(root / stage / "failure.json", dict(status="STOPPED_INTEGRITY", reason=error,
                  source=approved, config_sha=sha(PLAN), actions_completed=None if recorder is None else recorder.completed,
                  censoring="UNFINISHED_OR_ABORTED_PROCESS_NOT_TASK_TIMEOUT", wall_s=time.perf_counter()-start))
    finally:
        if log is not None:
            log.close()
    if error:
        raise RuntimeError(error)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("command", choices=("preflight", "s0", "s1-off", "s1-on", "campaign", "parity", "_worker"))
    ap.add_argument("--approved-commit", required=True)
    ap.add_argument("--execute-gpu", action="store_true")
    ap.add_argument("--stage", choices=("S0", "S1_OFF", "S1_ON"))
    args = ap.parse_args(argv)
    p = load(PLAN)
    validate_plan(p)
    if args.command == "preflight":
        result = preflight(p, args.approved_commit)
        print(json.dumps(result, indent=2)); return 0 if result["status"] == "PASS_CPU_PREFLIGHT" else 2
    if args.command == "parity":
        result = compare_qualified(p, args.approved_commit)
        print(json.dumps(result, indent=2)); return 0
    if args.command == "_worker":
        check(args.stage is not None, "worker stage required")
        worker(p, args.stage, args.approved_commit); return 0
    check(args.execute_gpu, "GPU execution requires separate user approval and explicit --execute-gpu")
    stages = ("S0", "S1_OFF", "S1_ON") if args.command == "campaign" else ({"s0":"S0", "s1-off":"S1_OFF", "s1-on":"S1_ON"}[args.command],)
    for stage in stages:
        result = run_stage(p, stage, args.approved_commit)
        print(json.dumps(result, indent=2))
        if result["status"] != "COMPLETED":
            return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
