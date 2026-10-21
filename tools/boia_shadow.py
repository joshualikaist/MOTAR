#!/usr/bin/env python3
"""Q01 v2 CPU preflight / bounded GPU supervisor. No simulator import outside _worker.

GPU subcommands require --execute-gpu and exact --approved-commit. This switch
is an accidental-launch guard; actual user approval is still required separately.
Stages run in the preregistered order S0, S1_NATIVE, S1_OFF, S1_ON. `verify` compares
their files on the CPU, exactly, and launches nothing.
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
PLAN = ROOT / "docs/plans/boia_shadow_q01_v2.json"
PYTHON = "/home/fair/miniconda3/envs/aerialgym/bin/python"
CODE = ("motar_eval/boia_shadow.py", "tools/boia_shadow.py", "tests/test_boia_shadow.py",
        "tests/test_boia_shadow_runner.py", "docs/plans/boia_shadow_q01_v2.json",
        "docs/plans/boia_shadow_q01_v2.md", "docs/plans/boia_shadow_q01_v1_amendment_2026-10-21.md")
# Q01 v1 was withdrawn before execution; its committed files and absent output stay as evidence.
V1 = dict(config="docs/plans/boia_shadow_q01_v1.json",
          config_sha256="759fef83ebbc4365c9080cb8dc15ebbb2209cc4d097f7dc5a235f1a41bc54df8",
          preregistration="docs/plans/boia_shadow_q01_v1.md",
          preregistration_sha256="cf5da98367770a1f6ebaf83461a533ed74e40c3469402b3e2f269e8a179f36b0",
          source_commit="6bd986eb585600ece1f8c59a7348ebfd8093fd42",
          output_root="/home/fair/workspaces/aerial_gym_ws/boia_shadow_runs/BOIA-SHADOW-Q01")
STAGES = ("S0", "S1_NATIVE", "S1_OFF", "S1_ON")
COMMANDS = {"s0": "S0", "s1-native": "S1_NATIVE", "s1-off": "S1_OFF", "s1-on": "S1_ON"}
COMPARISONS = {
    "native_vs_off": dict(left="S1_NATIVE", right="S1_OFF", files=["host.jsonl"], mode="full"),
    "off_vs_on": dict(left="S1_OFF", right="S1_ON", files=["parity.jsonl", "host.jsonl"], mode="full"),
    "replay_s0_vs_on": dict(left="S0", right="S1_ON", files=["parity.jsonl", "host.jsonl"], mode="prefix")}
PASS = "PASS_EXACT_NATIVE_OFF_ON_PARITY_WITH_REPLAY_CONTROL"
MISMATCH = "REPLAY_INSTABILITY_OR_PROFILE_MISMATCH"
INCOMPLETE = "INCOMPLETE_CAMPAIGN_NOT_EVALUATED"
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
    check(p["schema"] == "motar.boia.shadow.q01.v2" and p["run_id"] == "BOIA-SHADOW-Q01-V2", "unknown plan")
    expected = dict(num_envs=1, seed=4901, density=70, target="E2_direct_l2_v1", initial_view="visible",
                    initial_distance_m=10, step_dt_s=.1, physics_dt_s=.01, physics_substeps=10,
                    episode_actions=600, lineage="off")
    check(p["profile"] == expected, "Q01 profile changed")
    check(p["limits"] == dict(campaign_wall_s=1080, disk_preflight_bytes=3300000000,
          disk_stop_bytes=3000000000, output_bytes=256000000, host_trace_bytes=8000000,
          vram_ceiling_mib=6656, vram_reserve_mib=768, poll_s=2., source_guard_s=10.,
          watchdog_margin_s=4., term_grace_s=3., shadow_stride=5, sample_cap_s=2., slow_sample_ms=500.,
          slow_streak=3, ended_episodes_per_process=1, no_retry=True), "Q01 resource contract changed")
    check(p["stage_order"] == list(STAGES), "stage order changed")
    check(p["stages"] == {"S0": dict(actions=20, wall_s=180, observer=True, shadow=True, snapshot_cap=2),
                         "S1_NATIVE": dict(actions=200, wall_s=300, observer=False, shadow=False, snapshot_cap=0),
                         "S1_OFF": dict(actions=200, wall_s=300, observer=True, shadow=False, snapshot_cap=3),
                         "S1_ON": dict(actions=200, wall_s=300, observer=True, shadow=True, snapshot_cap=3)},
          "stage caps changed")
    check(sum(x["wall_s"] for x in p["stages"].values()) == p["limits"]["campaign_wall_s"],
          "campaign cap must equal the stage caps")
    check({k: {f: v[f] for f in ("left", "right", "files", "mode")} for k, v in p["comparisons"].items()}
          == COMPARISONS, "comparison contract changed")
    check(p["verdicts"] == [PASS, MISMATCH, INCOMPLETE], "verdict vocabulary changed")
    check(p["supersedes"] == dict(V1, status="WITHDRAWN_BEFORE_EXECUTION_NOT_RUN",
                                  amendment="docs/plans/boia_shadow_q01_v1_amendment_2026-10-21.md"),
          "superseded v1 binding changed")
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
    inactive = p["inactive_runtime_env"]["keys"]
    check(inactive == ["NAVRL_DETECTION_DROPOUT", "NAVRL_RGB_NOISE_STD", "NAVRL_DEPTH_NOISE_STD"] and
          all(k in p["runtime_env"] for k in inactive) and p["runtime_env"]["NAVRL_PERCEPTION_PERTURB"] == "0",
          "inactive perturbation variables must stay gated by NAVRL_PERCEPTION_PERTURB=0")
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
                  v1_preserved=(sha(ROOT / V1["config"]) == V1["config_sha256"] and
                                sha(ROOT / V1["preregistration"]) == V1["preregistration_sha256"]),
                  superseded_v1_not_run=not Path(V1["output_root"]).exists(),
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


def diff_fields(left, right, prefix=""):
    if isinstance(left, dict) and isinstance(right, dict):
        out = []
        for key in sorted(set(left) | set(right)):
            if key not in left or key not in right:
                out.append(prefix + key)
            elif left[key] != right[key]:
                out += diff_fields(left[key], right[key], prefix + key + ".")
        return out
    return [] if left == right else [prefix[:-1] or "<row>"]


def compare_rows(left, right, mode="full", left_censored=False):
    """Exact equality: no tolerance, exclusion or salvage. A missing trace is None.

    `prefix` compares the shorter replay control `left` with `right`: equal rows over all of
    `left`, and equal lengths unless `left` stopped at its own action cap.
    """
    if not left or not right:
        return dict(status="MISMATCH", reason="MISSING_OR_EMPTY_TRACE",
                    rows=[None if left is None else len(left), None if right is None else len(right)])
    rows = [len(left), len(right)]
    for i in range(min(rows)):
        if left[i] != right[i]:
            return dict(status="MISMATCH", reason="FIRST_ROW_MISMATCH", index=i, rows=rows,
                        fields=diff_fields(left[i], right[i]), left_row=left[i], right_row=right[i])
    if mode == "full":
        same_length = rows[0] == rows[1]
    else:
        same_length = rows[0] <= rows[1] if left_censored else rows[0] == rows[1]
    if not same_length:
        return dict(status="MISMATCH", reason="TRACE_LENGTH_MISMATCH", rows=rows)
    return dict(status="PASS", rows=rows, compared=rows[0])


def stage_record(root, stage, p, approved):
    done, failure = root / stage / "process_done.json", root / stage / "failure.json"
    if done.is_file():
        meta = load(done)
        bound = (meta.get("stage") == stage and meta.get("status") == "COMPLETED" and
                 meta.get("source") == approved and meta.get("config_sha") == sha(PLAN) and
                 meta.get("checkpoint") == p["frozen"]["policy_f_sha256"] and
                 meta.get("rms_start") == meta.get("rms_end") == p["frozen"]["rms_digest_sha256"])
        return dict(status="COMPLETED" if bound else "BINDING_MISMATCH", meta=meta)
    if failure.is_file():
        return dict(status="FAILED", meta=load(failure))
    return dict(status="MISSING", meta=None)


def _rows_or_none(path):
    return read_rows(path) if Path(path).is_file() else None


def verify(p, approved):
    """CPU-only exact comparison of the stage files. Writes parity_result.json once.

    A mismatch is never attributed to the observer or to CUDA nondeterminism here: without a
    verified cause it is REPLAY_INSTABILITY_OR_PROFILE_MISMATCH, with the first row recorded.
    """
    root = Path(p["output_root"])
    guard_sources(approved, sha(PLAN))
    check(root.is_dir(), "CAMPAIGN_ROOT_MISSING")
    stages = {stage: stage_record(root, stage, p, approved) for stage in STAGES}
    comparisons = {}
    for name, spec in COMPARISONS.items():
        left, right = stages[spec["left"]], stages[spec["right"]]
        if left["status"] != "COMPLETED" or right["status"] != "COMPLETED":
            comparisons[name] = dict(status="NOT_AVAILABLE",
                                     stages={spec["left"]: left["status"], spec["right"]: right["status"]})
            continue
        censored = left["meta"]["ended_episodes"] == 0
        files = {f: compare_rows(_rows_or_none(root / spec["left"] / f), _rows_or_none(root / spec["right"] / f),
                                 spec["mode"], censored) for f in spec["files"]}
        comparisons[name] = dict(status="PASS" if all(x["status"] == "PASS" for x in files.values()) else "MISMATCH",
                                 files=files)
    completed = [s for s in STAGES if stages[s]["status"] == "COMPLETED"]
    attestation = {s: stages[s]["meta"]["runtime_attestation_start"] for s in completed}
    hooks = {s: stages[s]["meta"]["hooks_during"] for s in completed}
    same_numerics = len({json.dumps(v, sort_keys=True) for v in attestation.values()}) <= 1
    hooks_as_planned = all(all(v is p["stages"][s]["observer"] for v in hooks[s].values()) for s in completed)
    comparisons["runtime_profile"] = dict(status="PASS" if same_numerics and hooks_as_planned else "MISMATCH",
                                          attestation=attestation, hooks_during=hooks)
    failed_on_mismatch = [s for s in STAGES if stages[s]["status"] == "FAILED" and
                          "MISMATCH" in str(stages[s]["meta"].get("reason"))]
    if (any(c["status"] == "MISMATCH" for c in comparisons.values()) or failed_on_mismatch or
            any(stages[s]["status"] == "BINDING_MISMATCH" for s in STAGES)):
        verdict = MISMATCH
    elif any(stages[s]["status"] != "COMPLETED" for s in STAGES):
        verdict = INCOMPLETE
    else:
        verdict = PASS
    def supported(*names):
        return all(comparisons[n]["status"] == "PASS" for n in names + ("replay_s0_vs_on", "runtime_profile"))
    claims = dict(
        observer_noninterference="SUPPORTED_EXACT_HOST_TRACE_THIS_PROFILE" if supported("native_vs_off") else "NOT_SUPPORTED",
        diagnostic_noninterference="SUPPORTED_EXACT_THIS_PROFILE" if supported("off_vs_on") else "NOT_SUPPORTED",
        replay_reproducibility=("PASS_ON_CONFIGURATION_S0_PREFIX_ONLY"
                                if comparisons["replay_s0_vs_on"]["status"] == "PASS" else "NOT_SUPPORTED"),
        mismatch_cause="NOT_VERIFIED" if verdict == MISMATCH else None,
        host_instrumentation="READ_ONLY_HOST_TRACE_IN_EVERY_STAGE_NO_UNINSTRUMENTED_RUN",
        boia_computation=p["qualification_scope"]["wait_intervene_computation"])
    result = dict(verdict=verdict, tolerance=0, excluded_rows=0, source=approved, config_sha=sha(PLAN),
                  stages={s: dict(status=r["status"], reason=None if r["meta"] is None else r["meta"].get("reason"),
                                  actions=None if r["meta"] is None else r["meta"].get("actions",
                                          r["meta"].get("actions_completed")),
                                  ended_episodes=None if r["meta"] is None else r["meta"].get("ended_episodes"))
                          for s, r in stages.items()},
                  failed_on_mismatch=failed_on_mismatch, comparisons=comparisons, claims=claims,
                  physical_safety="NOT_QUALIFIED", G5="HOLD")
    write_new(root / "parity_result.json", result)
    return result


def latency_summary(values):
    if not values:
        return dict(n=0, P50=None, P95=None, P99=None, maximum=None)
    x = sorted(values)
    def quantile(q):
        z = (len(x)-1)*q; lo = int(z); hi = min(lo+1, len(x)-1)
        return x[lo] + (x[hi]-x[lo])*(z-lo)
    return dict(n=len(x), P50=quantile(.5), P95=quantile(.95), P99=quantile(.99), maximum=x[-1])


def _terminate(proc, grace=3.):
    if proc.poll() is None:
        os.killpg(proc.pid, signal.SIGTERM)
        try:
            proc.wait(timeout=grace)  # lets the worker's SIGTERM handler leave failure.json
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
            proc.wait(timeout=1)


def run_stage(p, stage, approved):
    report = preflight(p, approved)
    check(report["status"] == "PASS_CPU_PREFLIGHT", "CPU_PREFLIGHT_FAILED: " + repr(report["checks"]))
    root, spec, limits = Path(p["output_root"]), p["stages"][stage], p["limits"]
    if stage == STAGES[0]:
        check(not root.exists(), "CAMPAIGN_PATH_EXISTS_NO_RETRY")
        root.mkdir(parents=True, exist_ok=False)
        write_new(root / "campaign_start.json", dict(preflight=report, runtime_env=p["runtime_env"],
                                                     run_id=p["run_id"], source=approved, stage_order=list(STAGES)))
    else:
        start = load(root / "campaign_start.json")
        check(start["source"] == approved and start["preflight"]["config_sha256"] == report["config_sha256"], "CAMPAIGN_BINDING_CHANGED")
        for previous in STAGES[:STAGES.index(stage)]:
            check(load(root / (previous + "_supervisor.json"))["status"] == "COMPLETED", "PREVIOUS_STAGE_NOT_PASS")
    check(not (root / stage).exists() and not (root / (stage + "_supervisor.json")).exists(), "STAGE_EXISTS_NO_RETRY")
    used = sum(load(x)["wall_s"] for x in root.glob("*_supervisor.json"))
    remaining = min(spec["wall_s"], limits["campaign_wall_s"] - used)
    check(remaining > limits["watchdog_margin_s"], "CAMPAIGN_BUDGET_EXHAUSTED")
    command = [PYTHON, "-B", str(Path(__file__).resolve()), "_worker", "--stage", stage,
               "--approved-commit", approved]
    start_time, peak, error, proc, ceiling = time.monotonic(), 0, None, None, None
    try:
        gpu = gpu_state()
        ceiling = min(limits["vram_ceiling_mib"], gpu["total_mib"] - limits["vram_reserve_mib"])
        check(gpu["used_mib"] < ceiling, "INSUFFICIENT_VRAM_HEADROOM")
        peak = gpu["used_mib"]
        with (root / (stage + ".log")).open("x") as log:
            proc = subprocess.Popen(command, cwd=str(ROOT), env=process_env(p, os.getpid(), report["config_sha256"]),
                                    stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            # Shutdown starts watchdog_margin_s before the cap so SIGTERM + grace end inside it.
            deadline, guarded = start_time + remaining - limits["watchdog_margin_s"], None
            while proc.poll() is None:
                now = time.monotonic()
                if now >= deadline:
                    raise ValueError("WALL_WATCHDOG_CENSORING_NOT_TASK_TIMEOUT")
                if guarded is None or now - guarded >= limits["source_guard_s"]:
                    guard_sources(approved, report["config_sha256"])  # git subprocesses: slower cadence
                    guarded = now
                check(shutil.disk_usage(root).free >= limits["disk_stop_bytes"], "DISK_BELOW_3GB")
                check(directory_bytes(root) <= limits["output_bytes"], "CAMPAIGN_OUTPUT_CAP")
                peak = max(peak, gpu_state()["used_mib"])
                check(peak <= ceiling, "DEVICE_VRAM_CEILING")
                time.sleep(max(.05, min(limits["poll_s"], deadline - time.monotonic())))
        check(proc.returncode == 0, "WORKER_NONZERO_EXIT_%s" % proc.returncode)
        guard_sources(approved, report["config_sha256"])
        done = load(root / stage / "process_done.json")
        check(done["status"] == "COMPLETED" and done["source"] == approved and done["stage"] == stage and
              done["config_sha"] == report["config_sha256"], "WORKER_BINDING_OR_COMPLETION")
    except BaseException as exc:
        error = type(exc).__name__ + ": " + str(exc)
    finally:
        if proc is not None:
            _terminate(proc, limits["term_grace_s"])
        worker_failure, failure = None, root / stage / "failure.json"
        if failure.is_file():  # keep the worker's own first cause visible in the supervisor receipt
            try:
                worker_failure = load(failure)
            except ValueError as exc:
                worker_failure = dict(status="UNREADABLE_FAILURE_RECEIPT", reason=str(exc))
        report = dict(stage=stage, status="STOPPED_INTEGRITY" if error else "COMPLETED", reason=error,
                      first_cause=(worker_failure or {}).get("reason") or error, worker_failure=worker_failure,
                      worker_returncode=None if proc is None else proc.returncode,
                      wall_s=time.monotonic() - start_time, peak_device_mib=peak, vram_cap_mib=ceiling,
                      vram_sampling="SUPERVISOR_POLL_%gS_NOT_A_TRUE_PEAK" % limits["poll_s"],
                      source=approved, config_sha=sha(PLAN), command=command, GPU_parity="PENDING_VERIFY",
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


class SupervisorStop(BaseException):
    """SIGTERM from the supervisor; caught only so the worker leaves failure.json."""


def _stop(signum, frame):
    raise SupervisorStop("SIGTERM_FROM_SUPERVISOR")


def worker(p, stage, approved):
    check(os.environ.get("BOIA_SHADOW_PARENT_PID") == str(os.getppid()), "WORKER_REQUIRES_SUPERVISOR")
    check(os.environ.get("BOIA_SHADOW_CONFIG_SHA") == sha(PLAN), "WORKER_CONFIG_SHA")
    guard_sources(approved, sha(PLAN))
    try:
        previous_handler, handler_installed = signal.signal(signal.SIGTERM, _stop), True
    except ValueError:  # only off the main thread, i.e. never in the supervised process
        previous_handler, handler_installed = None, False
    import isaacgym  # must precede torch; this function is never called by CPU preflight/tests.
    import torch
    from contextlib import nullcontext
    if str(ROOT / "tools") not in sys.path:
        sys.path.insert(0, str(ROOT / "tools"))
    import eq1d_fixture_launcher as eq
    from motar_eval.eq1d_telemetry import rms_digest
    from motar_eval.boia_shadow import (Binding, EvaluatorTap, EventLogger, HostTrace, Recorder, drive,
                                        hook_audit, runtime_attestation)
    root, spec = Path(p["output_root"]), p["stages"][stage]
    log, trace, tap, recorder, error = None, None, None, None, None
    start = time.perf_counter()
    try:
        reference = read_rows(root / "S1_OFF/parity.jsonl") if stage == "S1_ON" else None
        check(sha(checkpoint(p)) == p["frozen"]["policy_f_sha256"], "CHECKPOINT_BEFORE_LOAD")
        player, _ = eq.build_player(1, checkpoint(p))
        task = eq.find_task(player)
        origins = eq.check_origins()
        profile = runtime_profile(task, player, p)
        numerics = runtime_attestation(task.device)  # recorded, never changed (Fable G4-R6 F3)
        rms = eq.find_rms(player.model)
        rms0 = rms_digest(rms)
        check(rms0 == p["frozen"]["rms_digest_sha256"], "RMS_BINDING")
        hooks_before = hook_audit(task)
        check(not any(hooks_before.values()), "OBSERVER_HOOK_PRESENT_BEFORE_STAGE")
        callback_type = type(task.sim_env.physics_step_callback).__name__
        if spec["observer"]:
            log = EventLogger(root / stage, max_bytes=p["limits"]["output_bytes"] // len(STAGES))
            recorder = Recorder(log, spec["shadow"], p["limits"]["shadow_stride"], spec["snapshot_cap"], reference)
            tap = EvaluatorTap(task, recorder, Binding(approved, sha(PLAN), p["frozen"]["policy_f_sha256"]))
        else:
            (root / stage).mkdir(parents=False, exist_ok=False)
        trace = HostTrace(root / stage, p["limits"]["host_trace_bytes"])
        torch.cuda.reset_peak_memory_stats()

        def reset():
            obs = player.env_reset(player.env)
            player.get_batch_size(obs, 1)  # N1 otherwise loses its actor/action batch dimension.
            return obs

        def policy_state():
            check(rms_digest(rms) == rms0 and not player.model.training, "NORMALIZATION_OR_POLICY_STATE_MUTATED")

        with (tap if tap is not None else nullcontext()):
            hooks_during = hook_audit(task)
            check(all(v is spec["observer"] for v in hooks_during.values()), "OBSERVER_HOOK_STATE_MISMATCH")
            times, ended = drive(task, reset, lambda obs: player.get_action(obs, is_deterministic=True),
                                 lambda action: player.env_step(player.env, action), trace, spec["actions"],
                                 tap, recorder, policy_state)
            if recorder is not None:
                check(recorder.episodes_ended == ended, "HOST_OBSERVER_EPISODE_COUNT_MISMATCH")
                recorder.censor("EPISODE_LIMIT" if ended else "PROCESS_ACTION_CAP")
        hooks_after = hook_audit(task)
        check(hooks_after == hooks_before, "OBSERVER_HOOKS_NOT_RESTORED")
        torch.cuda.synchronize()
        check(rms_digest(rms) == rms0, "FINAL_RMS_CHANGED")
        numerics_end = runtime_attestation(task.device)
        check(numerics_end == numerics, "RUNTIME_NUMERICS_CHANGED_DURING_PROCESS")
        observer_times = [] if tap is None else tap.observer_times
        write_new(root / stage / "process_done.json", dict(stage=stage, status="COMPLETED",
                  observer=spec["observer"], shadow=spec["shadow"], source=approved, config_sha=sha(PLAN),
                  checkpoint=p["frozen"]["policy_f_sha256"], rms_start=rms0, rms_end=rms_digest(rms),
                  profile=profile, origins=origins, runtime_attestation_start=numerics,
                  runtime_attestation_end=numerics_end, hooks_before=hooks_before, hooks_during=hooks_during,
                  hooks_after=hooks_after, physics_step_callback_type=callback_type,
                  actions=len(times), ended_episodes=ended, host_rows=trace.rows,
                  observe_calls=None if tap is None else tap.observe_calls,
                  transform_calls=None if tap is None else tap.transform_calls,
                  compute_calls=None if tap is None else tap.compute_calls,
                  lidar_calls=None if tap is None else tap.lidar_calls,
                  rgbd_calls=None if tap is None else tap.rgbd_calls,
                  loop_ms=times, shadow_diagnostic_only_ms=[] if recorder is None else recorder.latencies,
                  shadow_latency_scope="DIAGNOSTIC_ONLY_NOT_INFORMATIVE_FOR_FULL_PLANNER",
                  observer_phase_times=observer_times,
                  cold_first_observer_ms=observer_times[0]["total_observer_ms"] if observer_times else None,
                  warm_observer_ms=latency_summary([x["total_observer_ms"] for x in observer_times[1:]]),
                  warm_full_step_ms=latency_summary(times[1:]),
                  latency_scope="HOST_SYNC_COPY_SCHEMA_DIAGNOSTIC_SHA_LOG; native physics/inference excluded from observer; no CUDA kernel-only claim",
                  peak_torch_allocated_bytes=torch.cuda.max_memory_allocated(),
                  snapshots=0 if recorder is None else recorder.snapshots,
                  wall_s=time.perf_counter()-start, full_planner="SKIPPED_UNSUPPORTED",
                  qualification_scope=p["qualification_scope"],
                  host_instrumentation="READ_ONLY_HOST_TRACE_PRESENT",
                  runtime_parity="PENDING_SUPERVISOR_VERIFY", physical_safety="NOT_QUALIFIED", G5="HOLD"))
    except BaseException as exc:
        error = type(exc).__name__ + ": " + str(exc)
        (root / stage).mkdir(parents=True, exist_ok=True)
        write_new(root / stage / "failure.json", dict(stage=stage, status="STOPPED_INTEGRITY", reason=error,
                  source=approved, config_sha=sha(PLAN), actions_completed=None if recorder is None else recorder.completed,
                  host_rows=None if trace is None else trace.rows,
                  censoring="UNFINISHED_OR_ABORTED_PROCESS_NOT_TASK_TIMEOUT", wall_s=time.perf_counter()-start))
    finally:
        for stream in (log, trace):
            if stream is not None:
                stream.close()
        if handler_installed:
            signal.signal(signal.SIGTERM, signal.SIG_DFL if previous_handler is None else previous_handler)
    if error:
        raise RuntimeError(error)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("command", choices=("preflight", *COMMANDS, "campaign", "verify", "_worker"))
    ap.add_argument("--approved-commit", required=True)
    ap.add_argument("--execute-gpu", action="store_true")
    ap.add_argument("--stage", choices=STAGES)
    args = ap.parse_args(argv)
    p = load(PLAN)
    validate_plan(p)
    if args.command == "preflight":
        result = preflight(p, args.approved_commit)
        print(json.dumps(result, indent=2)); return 0 if result["status"] == "PASS_CPU_PREFLIGHT" else 2
    if args.command == "verify":
        result = verify(p, args.approved_commit)
        print(json.dumps(result, indent=2)); return 0 if result["verdict"] == PASS else 1
    if args.command == "_worker":
        check(args.stage is not None, "worker stage required")
        worker(p, args.stage, args.approved_commit); return 0
    check(args.execute_gpu, "GPU execution requires separate user approval and explicit --execute-gpu")
    status = 0
    for stage in (STAGES if args.command == "campaign" else (COMMANDS[args.command],)):
        result = run_stage(p, stage, args.approved_commit)
        print(json.dumps(result, indent=2))
        if result["status"] != "COMPLETED":
            status = 1
            break
    if args.command == "campaign":  # CPU file comparison, also after a stopped stage; never a retry
        result = verify(p, args.approved_commit)
        print(json.dumps(result, indent=2))
        status = status or (0 if result["verdict"] == PASS else 1)
    return status


if __name__ == "__main__":
    raise SystemExit(main())
