"""S3a-P1: Frozen Policy F on the stored G8 r2 observations, CPU versus GPU with TF32 ON and OFF (motar-z5f).

Protocol: docs/plans/prereg_sitl_s3a_policy_precision_v1_2026-11-06.md. One factor changes: float32 matmul precision
on the GPU, set and read back AFTER player construction (build_player enables TF32, as rl_games' Runner does). The
observations, checkpoint, normalisation, deterministic action selection and the batch shape (one call's 128
environments) are the recording's. Frozen F's weights are only read. Nothing here touches a simulator.

  --cpu-only --out FILE        CPU condition and the pairing controls (no approval; CUDA must be hidden; FILE new and
                               outside every Git work tree)
  --execute --approval-file F  adds the two GPU conditions (needs the user's scoped approval). Writes only to the
                               fixed single-use directory RUN_DIR, created before CUDA starts; the kernel fault
                               count is read before and after and a rise voids the run.
"""
import argparse
import hashlib
import json
import math
import os
import re
import pathlib
import sys
import threading
import time

TOOLS = pathlib.Path(__file__).resolve().parent
ROOT = TOOLS.parent
PREREG = "docs/plans/prereg_sitl_s3a_policy_precision_v1_2026-11-06.md"
SCOPE = "S3a-P1 policy precision: stored G8 r2 observations, Frozen F, CPU vs GPU TF32 ON vs GPU TF32 OFF (one GPU process)"
G8 = pathlib.Path("/home/fair/workspaces/aerial_gym_ws/src/aerial_gym_simulator/results/sitl_ref_v1_r2/g8")
INPUTS = {  # name -> (relative path below G8, SHA-256)
    "obs_70": ("obs_dump_70bars_seed4501.npz", "e74b9de30c1cf5341802a433de3c5eef846870f6089a523f84550b29cc15fb2f"),
    "obs_205": ("obs_dump_205bars_seed4501.npz", "c50299d4b9074cf6afc2d6fd8a3e8722781dc23e411edd7823936590e806a461"),
    "state_70": ("state_record_70bars_seed4501.npz", "0cb6897114d22f6e6f11c9a272366aec123a8ac7c015574ba35bf9b5878a9fe9"),
    "state_205": ("state_record_205bars_seed4501.npz", "ae25c2c9b3cf6305c36ffa14649ddba9364b10c8e8c294ead4a6d9807f935c92"),
    "receipt_70": ("full/E2_obstacle_aware__70bars__seed4501/receipt.json",
                   "9de38dae0cceb1cf1eef64c51bba1aa0682e3bfb712caa566d358f12e9d6123b"),
    "receipt_205": ("full/E2_obstacle_aware__205bars__seed4501/receipt.json",
                    "d2e977b419102c4c2d3d858495ce7be79959046637aaf3f91dd7c0bd14aee6c8"),
}
F_SHA = "f702213936601860995cf61dcc570247e72543b1976e3716055cd8ec5593ad40"
DENSITIES = (70, 205)
CALLS, ENVS, OBS_DIM = 84, 128, 898
TOL_ACTION = 1e-4          # S3a's action tolerance, unchanged
TOL_REPRODUCE = 1e-5       # GPU TF32 ON must reproduce the recorded actions this closely to count as faithful
NEG_MIN = 1e-2             # the shifted-pairing negative control must differ at least this much
RSS_CAP_BYTES = 8 * 1024 ** 3
APPROVAL_KEYS = ("approved_by", "approved_at", "scope", "commit")
RUN_DIR = pathlib.Path.home() / "motar_tmp" / "s3a_policy_precision_20261106" / "gpu_run01"
FAULT_PATTERN = ("Machine check", "segfault", "general protection")
CAUSE_READINGS = ("TF32_EXPLAINS_POLICY_RESIDUAL", "TF32_SWITCH_NO_EFFECT", "RESIDUAL_NOT_TF32", "MIXED")


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for block in iter(lambda: f.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()


def verify_inputs(g8=G8, inputs=INPUTS):
    out, bad = {}, []
    for name, (rel, want) in inputs.items():
        p = pathlib.Path(g8) / rel
        got = sha256_file(p) if p.is_file() else None
        out[name] = {"path": str(p), "sha256": got}
        if got != want:
            bad.append(name)
    if bad:
        raise ValueError("inputs differ from their pins: %s" % bad)
    return out


def batches(obs_npz, state_npz, calls=CALLS, envs=ENVS):
    """(obs [calls, envs, 898] float32, recorded actions [calls, envs, 4]) with exactly one dump row per
    (call, env); the recording's batch is one call's envs in env order."""
    import numpy as np
    d = np.load(obs_npz)
    s = np.load(state_npz)
    obs, call, env = d["obs"], d["call_index"], d["env_id"]
    if obs.shape != (calls * envs, OBS_DIM) or s["action"].shape != (calls, envs, 4):
        raise ValueError("unexpected shapes %s %s" % (obs.shape, s["action"].shape))
    out = np.full((calls, envs, OBS_DIM), np.nan, np.float32)
    seen = np.zeros((calls, envs), bool)
    for i, (c, e) in enumerate(zip(call, env)):
        if not (0 <= c < calls and 0 <= e < envs) or seen[c, e]:
            raise ValueError("dump row %d (call %d, env %d) is outside the grid or repeated" % (i, c, e))
        seen[c, e] = True
        out[c, e] = obs[i]
    if not seen.all() or not np.isfinite(out).all() or not np.isfinite(s["action"]).all():
        raise ValueError("incomplete or non-finite grid")
    return out, s["action"].astype(np.float32)


def compare(a, b, tol=TOL_ACTION):
    import numpy as np
    a, b = np.asarray(a, np.float64), np.asarray(b, np.float64)
    finite = bool(np.isfinite(a).all() and np.isfinite(b).all())
    if not finite:                                                # never compared, never "faithful"
        return {"finite": False, "max": None, "argmax_call_env_dim": None, "mean": None,
                "n_nonfinite": int((~np.isfinite(a)).sum() + (~np.isfinite(b)).sum()), "n": int(a.size), "tol": tol}
    d = np.abs(a - b)
    i = np.unravel_index(int(d.argmax()), d.shape)
    return {"finite": True, "max": float(d.max()), "argmax_call_env_dim": [int(x) for x in i], "mean": float(d.mean()),
            "n_above_tol": int((d > tol).sum()), "n": int(d.size), "tol": tol}


def run(act, obs, device):
    """Actions for every call, one call's 128 environments per forward pass, as the recording."""
    import numpy as np
    import torch
    out = []
    for c in range(obs.shape[0]):
        x = torch.from_numpy(obs[c]).to(device)
        out.append(act(x).detach().to("cpu").numpy().astype(np.float32))
    return np.stack(out)


def backend_flags():
    import torch
    return {"cuda_matmul_allow_tf32": bool(torch.backends.cuda.matmul.allow_tf32),
            "cudnn_allow_tf32": bool(torch.backends.cudnn.allow_tf32),
            "cudnn_benchmark": bool(torch.backends.cudnn.benchmark),
            "cudnn_deterministic": bool(torch.backends.cudnn.deterministic),
            "float32_matmul_precision": torch.get_float32_matmul_precision()}


def set_tf32(on):
    import torch
    torch.backends.cuda.matmul.allow_tf32 = bool(on)
    torch.backends.cudnn.allow_tf32 = bool(on)
    torch.set_float32_matmul_precision("high" if on else "highest")
    flags = backend_flags()
    want = (bool(on), bool(on), "high" if on else "highest")
    if (flags["cuda_matmul_allow_tf32"], flags["cudnn_allow_tf32"], flags["float32_matmul_precision"]) != want:
        raise RuntimeError("backend flags did not take: %s" % flags)
    return flags


def controls(cpu, recorded):
    """Pairing controls on the CPU actions: the true pairing (positive; reproduces the S3a audit's order of
    magnitude) and the pairing shifted by one call (negative; must differ by at least NEG_MIN)."""
    pos = compare(cpu, recorded)
    neg = compare(cpu[:-1], recorded[1:])
    return {"positive_true_pairing": pos, "negative_shifted_pairing": neg,
            "ok": pos["finite"] and neg["finite"] and pos["max"] < NEG_MIN and neg["max"] >= NEG_MIN}


def _density_reading(m):
    if m["gpu_on_vs_recorded"]["max"] > TOL_REPRODUCE:
        return "GPU_REPLAY_NOT_FAITHFUL"
    if m["gpu_off_vs_cpu"]["max"] <= TOL_ACTION:
        return "TF32_EXPLAINS_POLICY_RESIDUAL"
    if m["gpu_on_vs_gpu_off"]["max"] == 0.0:
        return "TF32_SWITCH_NO_EFFECT"                            # flags read back, outputs bitwise unchanged
    return "RESIDUAL_NOT_TF32"


def reading(metrics):
    """The protocol's fixed reading. GPU_REPLAY_NOT_FAITHFUL if either density's replay is not faithful; otherwise
    each density is classified on its own (density_readings) and the reading is that label when both agree, MIXED
    when they differ. metrics[bars] holds the comparisons."""
    if not all(metrics[b]["controls"]["ok"] for b in metrics):
        return "CONTROL_FAILED"
    if any("gpu_on_vs_recorded" not in metrics[b] for b in metrics):
        return "CPU_ONLY"
    keys = ("gpu_on_vs_recorded", "gpu_off_vs_cpu", "gpu_on_vs_gpu_off")
    if not all(metrics[b][k].get("finite", True) for b in metrics for k in keys):
        return "NONFINITE_OUTPUT"
    labels = {_density_reading(metrics[b]) for b in metrics}
    if "GPU_REPLAY_NOT_FAITHFUL" in labels:                      # one unfaithful replay: the apparatus is not shown
        return "GPU_REPLAY_NOT_FAITHFUL"
    return labels.pop() if len(labels) == 1 else "MIXED"


def density_readings(metrics):
    return {str(b): (_density_reading(m) if "gpu_on_vs_recorded" in m and all(
        m[k].get("finite", True) for k in ("gpu_on_vs_recorded", "gpu_off_vs_cpu", "gpu_on_vs_gpu_off")) else None)
            for b, m in metrics.items()}


def _rss_watchdog(cap=RSS_CAP_BYTES, period=0.5):
    def watch():
        page = os.sysconf("SC_PAGE_SIZE")
        while True:
            with open("/proc/self/statm") as f:
                rss = int(f.read().split()[1]) * page
            if rss > cap:
                sys.stderr.write("RSS %d > cap %d: abort\n" % (rss, cap))
                sys.stderr.flush()
                os._exit(3)
            time.sleep(period)
    threading.Thread(target=watch, name="rss-watchdog", daemon=True).start()


def players(B, device):
    """The bridge's CPU player and, for a CUDA device, the same construction on the GPU."""
    import torch
    audit = B._LOADED["audit"]
    if device == "cpu":
        p = B.Policy()
        return p.act, p.schema
    audit.install_cpu_only_aerial_gym_packages()
    state = torch.load(str(B.F_CKPT), map_location=device, weights_only=False)
    schema = audit.read_checkpoint_schema(state)
    audit.apply_schema_environment(schema)
    original = torch.load

    def mapped(*a, **kw):
        kw.setdefault("map_location", device)
        return original(*a, **kw)
    torch.load = mapped
    try:
        player = audit.build_player(B.F_CKPT, schema, device, B.TRAIN_CONFIG)
    finally:
        torch.load = original
    return (lambda x: audit.deterministic_actions(player, x)), schema


def _navrl_env():
    return {k: v for k, v in sorted(os.environ.items()) if k.startswith("NAVRL_")}


def _loaded_files():
    """SHA-256 of every loaded aerial_gym / rl_games module file (what the players actually ran)."""
    out = {}
    for name, mod in sorted(sys.modules.items()):
        f = getattr(mod, "__file__", None)
        if f and name.split(".")[0] in ("aerial_gym", "rl_games") and f.endswith(".py") and os.path.isfile(f):
            out[name] = {"path": f, "sha256": sha256_file(f)}
    return out


def _require_worktree_aerial_gym():
    resolved = pathlib.Path(list(sys.modules["aerial_gym"].__path__)[0]).resolve()
    if not str(resolved).startswith(str(ROOT.resolve()) + "/"):
        raise RuntimeError("aerial_gym resolved outside this worktree: %s" % resolved)
    return str(resolved)


def measure(gpu, *, g8=G8, out_npz=None, gpu_device="cuda:0"):
    import numpy as np
    import torch
    sys.path.insert(0, str(TOOLS))
    import sitl_bridge as B
    torch.set_num_threads(1)
    inputs = verify_inputs(g8)
    if sha256_file(B.F_CKPT) != F_SHA:
        raise ValueError("Policy F checkpoint differs from its pin")
    result = {"protocol": PREREG, "inputs": inputs, "policy_f": {"path": str(B.F_CKPT), "sha256": F_SHA},
              "torch": torch.__version__, "cuda_available": torch.cuda.is_available(), "densities": {}}
    arrays = {}
    for bars in DENSITIES:
        receipt = json.loads((pathlib.Path(g8) / INPUTS["receipt_%d" % bars][0]).read_text())
        B.setup(receipt["env_snapshot_navrl"])
        obs, recorded = batches(pathlib.Path(g8) / INPUTS["obs_%d" % bars][0],
                                pathlib.Path(g8) / INPUTS["state_%d" % bars][0])
        act_cpu, schema = players(B, "cpu")
        _require_worktree_aerial_gym()                            # before any GPU work (review r3 N14)
        cpu = run(act_cpu, obs, "cpu")
        m = {"cpu_vs_recorded": compare(cpu, recorded), "controls": controls(cpu, recorded),
             "schema_policy": schema.get("cfg_action_policy"), "navrl_env_cpu": _navrl_env()}
        arrays["cpu_%d" % bars] = cpu
        if gpu:
            act_gpu, _ = players(B, gpu_device)
            m["navrl_env_gpu"] = _navrl_env()
            m["flags_after_construction"] = backend_flags()
            m["flags_on"] = set_tf32(True)
            on = run(act_gpu, obs, gpu_device)
            m["flags_off"] = set_tf32(False)
            off = run(act_gpu, obs, gpu_device)
            set_tf32(True)
            m.update(gpu_on_vs_recorded=compare(on, recorded), gpu_off_vs_recorded=compare(off, recorded),
                     gpu_off_vs_cpu=compare(off, cpu), gpu_on_vs_cpu=compare(on, cpu),
                     gpu_on_vs_gpu_off=compare(on, off), device=gpu_device,
                     device_name=torch.cuda.get_device_name(0) if gpu_device.startswith("cuda") else None)
            arrays.update({"gpu_on_%d" % bars: on, "gpu_off_%d" % bars: off})
        result["densities"][str(bars)] = m
    result["reading"] = reading(result["densities"])
    # per-density labels only when the headline reading assigns a cause per density (review r3 N12)
    result["density_readings"] = (density_readings(result["densities"]) if result["reading"] in CAUSE_READINGS
                                  else None)
    result["loaded_module_files"] = _loaded_files()
    result["aerial_gym_path"] = _require_worktree_aerial_gym()
    if out_npz is not None:
        np.savez(out_npz, **arrays)
        result["actions_npz"] = {"path": str(out_npz), "sha256": sha256_file(out_npz)}
    return result


def git_state(root=ROOT):
    import subprocess
    run_ = lambda *a: subprocess.run(["git", "-C", str(root), *a], capture_output=True, text=True).stdout.strip()  # noqa: E731
    return {"head": run_("rev-parse", "HEAD"), "dirty": run_("status", "--porcelain")}


def approval_problems(approval, git):
    p = ["missing %s" % k for k in APPROVAL_KEYS if not isinstance(approval, dict) or approval.get(k) in (None, "")
         or not isinstance(approval.get(k), str)]
    if p:
        return p
    if approval["scope"] != SCOPE:
        p.append("scope differs")
    if approval["commit"] != git["head"]:
        p.append("approval names %s, HEAD is %s" % (approval["commit"], git["head"]))
    if git["dirty"]:
        p.append("worktree is not clean")
    return p


def inside_git_tree(path):
    path = pathlib.Path(path).absolute()
    return any(os.path.lexists(q / ".git") for q in (path, *path.parents))


def kernel_faults():
    import subprocess
    r = subprocess.run(["journalctl", "-k", "-b", "0", "--no-pager"], capture_output=True, timeout=30,
                       env=dict(os.environ, LC_ALL="C"))
    if r.returncode != 0 or not r.stdout:
        raise RuntimeError("kernel log unreadable (rc %d): the fault check would be vacuous" % r.returncode)
    text = r.stdout.decode("utf-8", errors="replace")
    return sum(1 for line in text.splitlines() if any(k in line for k in FAULT_PATTERN))


def _write_json(path, obj):
    text = json.dumps(obj, indent=1, allow_nan=False)            # serialise fully before the file exists
    with open(path, "x") as fh:
        fh.write(text + "\n")


def _f_checkpoint():
    saved = list(sys.path)
    try:
        sys.path.insert(0, str(TOOLS))
        import sitl_bridge as B                                   # module constants only: no torch, no setup
    finally:
        sys.path[:] = saved
    return B.F_CKPT


def _strict_json(raw):
    def bad(c):
        raise ValueError("non-finite constant %s in the approval file" % c)

    def number(text):
        x = float(text)
        if not math.isfinite(x):
            bad(text)
        return x
    return json.loads(raw, parse_constant=bad, parse_float=number)


def main(argv=None, *, run_dir=RUN_DIR, git=None, faults=kernel_faults):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--cpu-only", action="store_true")
    mode.add_argument("--execute", action="store_true")
    parser.add_argument("--approval-file", type=pathlib.Path)
    parser.add_argument("--out", type=pathlib.Path, help="new result file for --cpu-only")
    args = parser.parse_args(argv)
    approval = None
    if args.cpu_only:
        if args.out is None or args.approval_file is not None:
            parser.error("--cpu-only needs --out and takes no approval")
        if os.environ.get("CUDA_VISIBLE_DEVICES") != "":
            parser.error("--cpu-only requires CUDA_VISIBLE_DEVICES to be the empty string")
        out = args.out.absolute()
        if os.path.lexists(out) or os.path.lexists(out.with_suffix(".npz")):
            parser.error("--out must be new")
        if inside_git_tree(out.parent):
            parser.error("--out must be outside every Git work tree")
        faults0 = faults()
    else:
        if args.approval_file is None or args.out is not None:
            parser.error("--execute needs --approval-file and writes only to %s" % run_dir)
        raw = args.approval_file.read_bytes()
        try:
            approval = _strict_json(raw)
        except ValueError as e:
            print(json.dumps({"status": "REFUSED", "problems": [str(e)]}))
            return 2
        problems = approval_problems(approval, git or git_state())
        if not re.fullmatch(r"[0-9]+", os.environ.get("CUDA_VISIBLE_DEVICES") or ""):
            problems.append("CUDA_VISIBLE_DEVICES must name one GPU index")
        try:
            verify_inputs()                                       # the pins, before the directory is spent
            if sha256_file(_f_checkpoint()) != F_SHA:
                problems.append("Policy F checkpoint differs from its pin")
        except Exception as e:                                    # noqa: BLE001
            problems.append("inputs: %s" % e)
        if os.path.lexists(run_dir):
            problems.append("run directory %s already exists: the GPU run is single use" % run_dir)
        if inside_git_tree(run_dir.parent):
            problems.append("run directory inside a Git work tree")
        try:
            faults0 = faults()                                    # an unreadable kernel log refuses before anything
        except Exception as e:                                    # noqa: BLE001
            problems.append("kernel log: %s" % e)
        if problems:
            print(json.dumps({"status": "REFUSED", "problems": problems}))
            return 2
        approval["approval_file"] = {"path": str(args.approval_file.resolve()), "sha256": hashlib.sha256(raw).hexdigest()}
        run_dir.mkdir(parents=False, exist_ok=False)              # single use, before CUDA starts
        _write_json(run_dir / "attempt.json", {"approval": approval, "git": git or git_state(), "started": time.time(),
                                               "kernel_faults_start": faults0, "scope": SCOPE, "prereg": PREREG})
        out = run_dir / "result.json"
    _rss_watchdog()
    t0 = time.time()
    try:
        result = measure(gpu=args.execute, out_npz=out.with_suffix(".npz"))
    except BaseException as e:                                    # noqa: BLE001 - recorded, then re-raised
        if args.execute:
            _write_json(run_dir / "aborted.json", {"error": "%s: %s" % (type(e).__name__, e), "wall_s": time.time() - t0})
        raise
    try:
        faults1, fault_error = faults(), None
    except Exception as e:                                        # noqa: BLE001 - still write the result
        faults1, fault_error = None, "%s: %s" % (type(e).__name__, e)
    try:
        git_after = git_state()
    except Exception as e:                                        # noqa: BLE001 - still write the result
        git_after = {"error": "%s: %s" % (type(e).__name__, e)}
    result.update(approval=approval, git=git_after, wall_s=time.time() - t0, run_dir=str(run_dir) if args.execute
                  else None, kernel_faults_start=faults0, kernel_faults_end=faults1, fault_check_error=fault_error,
                  environment={k: os.environ.get(k) for k in ("CUDA_VISIBLE_DEVICES", "PYTHONNOUSERSITE",
                                                             "OMP_NUM_THREADS", "MKL_NUM_THREADS")},
                  affinity=sorted(os.sched_getaffinity(0)), nice=os.nice(0))
    if faults1 is None or faults1 != faults0:
        result.update(reading_if_valid=result["reading"], reading="VOID_HOST_FAULT",
                      density_readings_if_valid=result.get("density_readings"), density_readings=None)
    _write_json(out, result)
    print(json.dumps({"reading": result["reading"], "out": str(out)}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
