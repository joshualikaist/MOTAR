"""dev08 attempt recorder (amendment1 runtime-stack row "Attempt recorder"; motar-5md).

The fixed 42-episode sequence of the proposed observed-prefix unit and an append-only attempt record:
  - schedule(): two native smoke episodes (env0 at 70 then 205 bars; never main observations), then the forty main
    episodes: 70 bars before 205, env0..9, even envs native then candidate, odd envs candidate then native.
  - AttemptRecorder: a new attempt directory outside every Git work tree; attempt.json binds the protocol, the
    clean source HEAD, every runtime source file, Policy F's checkpoint, boot id, kernel fault count and the process
    environment (CUDA hidden, PYTHONNOUSERSITE=1, CPU affinity within 6,7,12,13, nice >= 10), refusing a mismatch.
    ledger.jsonl is append-only: each record names the next scheduled member (no skip, rerun or reorder) and the
    SHA-256 of the previous line. A record that is not operationally VALID, a smoke failure, a budget overrun or a
    raised fault count ends the attempt; final.json lists every unrun member. Host faults void the attempt.
  - main_records(): the verified ledger's main records for tools/sitl_dev08_prefix_contract.summarize_complete_pairs.
This module starts no simulator and grants no approval: execution of the unit needs a new scoped approval.
"""
import contextlib
import hashlib
import json
import math
import numbers
import os
import signal
import threading
import pathlib
import subprocess
import sys
import time

STATUS = {"component": "attempt_recorder", "contract": "amendment1 runtime-stack: Attempt recorder",
          "live_verified": False, "execution_supported": False, "launch_ready": False}

ROOT = pathlib.Path(__file__).resolve().parents[1]
PROTOCOL = "observed-prefix-v1"
AMENDMENT = "docs/plans/prereg_sitl_dev08_candidate_gains_v1_amendment1_2026-11-05.md"
AMENDMENT2 = "docs/plans/prereg_sitl_dev08_candidate_gains_v1_amendment2_2026-11-05.md"
AMENDMENT3 = "docs/plans/prereg_sitl_dev08_candidate_gains_v1_amendment3_2026-11-05.md"
AMENDMENT4 = "docs/plans/prereg_sitl_dev08_candidate_gains_v1_amendment4_2026-11-05.md"
AMENDMENT5 = "docs/plans/prereg_sitl_dev08_candidate_gains_v1_amendment5_2026-11-05.md"
# Every file the unit runs or reads as code/configuration: the runtime stack, the historical runner and bridge, and
# the production modules the bridge loads. Each must be tracked and equal to HEAD.
SOURCES = ("tools/sitl_owned_session.py", "tools/sitl_dev08_link.py", "tools/sitl_dev08_step.py",
           "tools/sitl_dev08_attempt.py", "tools/sitl_s3b_dev08_prefix.py", "tools/sitl_dev08_prefix_contract.py",
           "tools/sitl_s3b_dev08.py", "tools/sitl_s3b_closed_loop.py", "tools/sitl_bridge.py",
           "tools/navrl_reflection_offline_audit.py", "motar_eval/occlusion_trace.py",
           "aerial_gym/task/navrl_task/navrl_perception.py", "aerial_gym/task/navrl_task/navrl_detector.py",
           "aerial_gym/task/navrl_task/speed_governor.py", "aerial_gym/config/task_config/navrl_task_config.py",
           "aerial_gym/sensors/warp/warp_kernels/warp_lidar_kernels.py",
           "aerial_gym/rl_training/rl_games/ppo_navrl_perception_transformer.yaml",
           "aerial_gym/rl_training/rl_games/navrl_players.py", "aerial_gym/rl_training/rl_games/navrl_transformer_network.py",
           "aerial_gym/rl_training/rl_games/navrl_action_models.py", "aerial_gym/rl_training/rl_games/ppo_update_safety.py",
           "aerial_gym/rl_training/rl_games/navrl_mode_probe.py", "aerial_gym/task/navrl_task/navrl_corridor.py",
           "aerial_gym/task/navrl_task/navrl_search_state.py", "aerial_gym/utils/math.py", AMENDMENT, AMENDMENT2,
           AMENDMENT3, AMENDMENT4, AMENDMENT5)
# Code loaded from another work tree (tools/sitl_bridge.py CPLANNER): bound by hash and that tree's HEAD/state.
EXTERNAL_SOURCES = (("/home/fair/workspaces/aerial_gym_ws/src/MOTAR-cplanner", "cplanner/motar/replay.py"),)
SEED, EPISODE = 4501, 0
DENSITIES = (70, 205)
ENVS = tuple(range(10))
CORES = frozenset({6, 7, 12, 13})
BUDGET_S = {"density_main": 2400.0, "overall": 5400.0}
FAULT_PATTERN = ("Machine check", "segfault", "general protection")


def schedule():
    out = [{"phase": "smoke", "bars": b, "env": 0, "arm": "native"} for b in DENSITIES]
    for b in DENSITIES:
        for e in ENVS:
            arms = ("native", "candidate") if e % 2 == 0 else ("candidate", "native")
            out.extend({"phase": "main", "bars": b, "env": e, "arm": a} for a in arms)
    for i, m in enumerate(out):
        m.update(index=i, name="%02d_%s_%dbars_env%d_%s" % (i, m["phase"], m["bars"], m["env"], m["arm"]))
    return out


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for block in iter(lambda: f.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()


def kernel_faults():
    try:
        r = subprocess.run(["journalctl", "-k", "-b", "0", "--no-pager"], capture_output=True, timeout=30,
                           env=dict(os.environ, LC_ALL="C"))
    except subprocess.TimeoutExpired:
        raise AttemptError("kernel log read timed out: the fault check would be vacuous") from None
    if r.returncode != 0 or not r.stdout:
        raise AttemptError("kernel log unreadable (journalctl rc %d): the fault check would be vacuous" % r.returncode)
    out = r.stdout.decode("utf-8", errors="replace")
    return sum(1 for line in out.splitlines() if any(k in line for k in FAULT_PATTERN))


def git_state(root):
    run = lambda *a: subprocess.run(["git", "-C", str(root), *a], capture_output=True, text=True,  # noqa: E731
                                    env=dict(os.environ, LC_ALL="C")).stdout.strip()
    return {"head": run("rev-parse", "HEAD"), "branch": run("rev-parse", "--abbrev-ref", "HEAD"),
            "dirty": run("status", "--porcelain", "--untracked-files=no")}


def _git(root, *a):
    r = subprocess.run(["git", "-C", str(root), *a], capture_output=True, text=True, env=dict(os.environ, LC_ALL="C"))
    return r.returncode, r.stdout.strip()


def source_problems(root, sources):
    """A source that is untracked, or whose bytes differ from HEAD's blob, is not part of the commit that is run."""
    problems = []
    for rel in sources:
        rc, _ = _git(root, "ls-files", "--error-unmatch", rel)
        if rc != 0:
            problems.append("untracked source %s" % rel)
            continue
        rc1, work = _git(root, "hash-object", rel)
        rc2, head = _git(root, "rev-parse", "HEAD:" + rel)
        if rc1 or rc2 or work != head:
            problems.append("source %s differs from HEAD" % rel)
    return problems


def external_binding(external):
    out, problems = {}, []
    for tree, rel in external:
        p = pathlib.Path(tree) / rel
        if not p.is_file():
            problems.append("missing external source %s" % p)
            continue
        _, head = _git(tree, "rev-parse", "HEAD")
        _, dirty = _git(tree, "status", "--porcelain", "--", rel)
        out[str(p)] = {"sha256": sha256_file(p), "tree_head": head, "dirty": dirty}
        if dirty:
            problems.append("external source %s has local changes" % p)
    return out, problems


def platform_info():
    info = {"python": sys.version.split()[0], "cpu_count": os.cpu_count(),
            "cpus_online": pathlib.Path("/sys/devices/system/cpu/online").read_text().strip(),
            "cpus_offline": pathlib.Path("/sys/devices/system/cpu/offline").read_text().strip(),
            "OMP_NUM_THREADS": os.environ.get("OMP_NUM_THREADS"), "MKL_NUM_THREADS": os.environ.get("MKL_NUM_THREADS")}
    for name in ("numpy", "torch", "warp"):
        mod = sys.modules.get(name)
        info[name] = getattr(mod, "__version__", None) if mod is not None else "not imported"
    return info


def record_problems(record):
    """amendment1's per-record rules for an operationally VALID record (smoke too)."""
    p = []
    k, frames = record.get("final_frame_index"), record.get("recorded_frames")
    if type(frames) is not int or not 1 <= frames <= 600:
        return ["recorded_frames missing or outside 1..600"]
    if type(k) is not int or not 0 <= k <= frames - 1:
        p.append("final_frame_index outside the admitted prefix")
        return p
    if record.get("observation_checks") != k + 1 or record.get("commands_issued") != k:
        p.append("counts are not k+1 checks and k commands")
    outcome = record.get("outcome")
    if outcome not in ("capture", "crash", "CENSORED_TRACE_END"):
        p.append("outcome %r is not an observed-prefix outcome" % outcome)
    if outcome == "CENSORED_TRACE_END" and k != frames - 1:
        p.append("censor before the last admitted checkpoint")
    times = [record.get("start_sim_time_s"), record.get("final_sim_time_s"), record.get("scheduled_elapsed_s")]
    if any(type(t) not in (int, float) or t != t or t < 0 for t in times):
        p.append("missing or non-finite times")
    elif times[1] < times[0] or times[2] != round(k * 0.1, 10):
        p.append("non-monotonic simulator time or wrong nominal checkpoint time")
    return p


def core_idle(cores=CORES, seconds=1.0):
    """Idle fraction of each core over a short window, from /proc/stat (idle + iowait over total jiffies)."""
    def read():
        out = {}
        for line in pathlib.Path("/proc/stat").read_text().splitlines():
            name = line.split()[0]
            if name.startswith("cpu") and name[3:].isdigit() and int(name[3:]) in cores:
                v = [int(x) for x in line.split()[1:]]
                out[int(name[3:])] = (v[3] + v[4], sum(v))
        return out
    a = read()
    time.sleep(seconds)
    b = read()
    return {c: round((b[c][0] - a[c][0]) / max(1, b[c][1] - a[c][1]), 4) for c in sorted(a) if c in b}


def idle_check(cores=CORES, min_idle=0.9, seconds=1.0, tries=3):
    """The runner's cores must be idle before the unit and before each member (review r2: another job on them
    multiplied the step time by 25). Up to `tries` 1 s samples; the first passing one is enough, all are recorded."""
    samples = []
    for _ in range(tries):
        idle = core_idle(cores, seconds)
        samples.append(idle)
        if len(idle) == len(cores) and all(v >= min_idle for v in idle.values()):
            return True, {"core_idle": idle, "min_idle": min_idle, "samples": samples}
    return False, {"core_idle": samples[-1], "min_idle": min_idle, "samples": samples}


@contextlib.contextmanager
def signals_held():
    """Hold SIGTERM/SIGHUP/SIGINT while a ledger line or final.json is written; a held signal is re-delivered when
    the write is complete. The Python handlers are swapped for a recorder, so this holds whichever thread the kernel
    delivers the signal to: pthread_sigmask would block the calling thread only, and the runner has more threads
    (torch, watchdog, MAVLink receiver; review r5 MJ1-r5). Main thread only (Python sets handlers only there)."""
    held = (signal.SIGTERM, signal.SIGHUP, signal.SIGINT)
    if threading.current_thread() is not threading.main_thread():
        yield
        return
    pending = []
    saved = {s: signal.signal(s, lambda signum, frame: pending.append(signum)) for s in held}
    try:
        yield
    finally:
        for s, h in saved.items():
            signal.signal(s, h)
        if pending:
            signal.raise_signal(pending[0])


def _finite_json(record):
    """JSON numeric primitives, including numpy scalars/arrays, and explicit record problems.

    Non-finite and unsupported values become None so the failed record can still be retained. No numpy import
    is needed before the environment gate; numpy's tolist() returns ordinary scalar/list values recursively.
    """
    bad = []

    def clean(x, path):
        if isinstance(x, dict):
            return {k: clean(v, path + "." + str(k)) for k, v in x.items()}
        if isinstance(x, (list, tuple)):
            return [clean(v, "%s[%d]" % (path, i)) for i, v in enumerate(x)]
        if x is None or isinstance(x, (bool, str)):
            return x
        if isinstance(x, numbers.Integral):
            return int(x)
        if isinstance(x, numbers.Real):
            if not math.isfinite(float(x)):
                bad.append("non-finite value at %s" % path)
                return None
            return float(x)
        if type(x).__module__.split(".")[0] == "numpy" and hasattr(x, "tolist"):
            return clean(x.tolist(), path)
        bad.append("unsupported value at %s (%s)" % (path, type(x).__name__))
        return None
    return clean(record, "record"), bad


def process_environment():
    return {"CUDA_VISIBLE_DEVICES": os.environ.get("CUDA_VISIBLE_DEVICES"),
            "PYTHONNOUSERSITE": os.environ.get("PYTHONNOUSERSITE"),
            "OMP_NUM_THREADS": os.environ.get("OMP_NUM_THREADS"), "MKL_NUM_THREADS": os.environ.get("MKL_NUM_THREADS"),
            "affinity": sorted(os.sched_getaffinity(0)), "nice": os.nice(0)}


def environment_problems(env, cores=CORES, min_nice=10):
    p = []
    if env["CUDA_VISIBLE_DEVICES"] != "":
        p.append("CUDA_VISIBLE_DEVICES must be the empty string")
    if env["PYTHONNOUSERSITE"] != "1":
        p.append("PYTHONNOUSERSITE must be 1")
    for key in ("OMP_NUM_THREADS", "MKL_NUM_THREADS"):        # amendment4: 4 threads on two cores gave 160-350 ms
        if env.get(key) != "1":                                 # compute spikes; 1 thread: max 11.8 ms
            p.append("%s must be 1" % key)
    if not env["affinity"] or not set(env["affinity"]) <= set(cores):
        p.append("CPU affinity %s outside %s" % (env["affinity"], sorted(cores)))
    if env["nice"] < min_nice:
        p.append("nice %d < %d" % (env["nice"], min_nice))
    return p


def _git_tree(path):
    for p in (path, *path.parents):
        if os.path.lexists(p / ".git"):
            return p
    return None


class AttemptError(RuntimeError):
    pass


class AttemptRecorder:
    def __init__(self, attempt_dir, *, scope, root=ROOT, f_checkpoint=None, f_sha=None, sources=SOURCES,
                 external=EXTERNAL_SOURCES, cores=CORES, min_nice=10, budget_s=BUDGET_S, faults=kernel_faults,
                 clock=time.monotonic, require_clean=True, extra_header=None, host_check=idle_check):
        self.dir = pathlib.Path(attempt_dir)
        self.scope, self.root, self.sources = scope, pathlib.Path(root), sources
        self.f_checkpoint, self.f_sha = f_checkpoint, f_sha
        self.cores, self.min_nice = cores, min_nice
        self.budget_s, self.faults, self.clock = dict(budget_s), faults, clock
        self.require_clean = require_clean
        self.external = tuple(external)
        self.extra_header = dict(extra_header or {})
        self.host_check = host_check
        self.plan = schedule()
        self.next = 0
        self.prev_sha = None
        self.status = None
        self.t0 = None
        self.block_t0 = {}

    # -- open
    def open(self):
        if not self.dir.is_absolute() or os.path.lexists(self.dir):
            raise AttemptError("attempt directory must be new and absolute: %s" % self.dir)
        parent = self.dir.parent.resolve(strict=True)
        if _git_tree(parent / self.dir.name) is not None:
            raise AttemptError("attempt directory inside a Git work tree")
        if not self.scope or not isinstance(self.scope, dict) or not self.scope.get("approval"):
            raise AttemptError("no scoped approval recorded for this attempt")
        env = process_environment()
        problems = environment_problems(env, self.cores, self.min_nice)
        git = git_state(self.root)
        if self.require_clean and git["dirty"]:
            problems.append("tracked source is not clean: %s" % git["dirty"].splitlines()[:5])
        if self.require_clean:
            problems += source_problems(self.root, self.sources)
        external, ext_problems = external_binding(self.external)
        problems += ext_problems
        host_ok, host = self.host_check()
        if not host_ok:
            problems.append("runner cores are not idle: %s" % host)
        files = {}
        for rel in self.sources:
            p = self.root / rel
            if not p.is_file():
                problems.append("missing source %s" % rel)
            else:
                files[rel] = sha256_file(p)
        f = None
        if self.f_checkpoint is not None:
            actual = sha256_file(self.f_checkpoint)
            if actual != self.f_sha:
                problems.append("Policy F checkpoint differs from its pinned SHA-256")
            f = {"path": str(self.f_checkpoint), "sha256": actual}
        else:
            problems.append("Policy F checkpoint not bound")
        if problems:
            raise AttemptError("attempt refused: " + "; ".join(problems))
        self.faults0 = self.faults()                    # an unreadable kernel log refuses before anything exists
        self.dir.mkdir()
        self.t0 = self.clock()
        header = {"status": STATUS, "protocol": PROTOCOL, "amendment": AMENDMENT,
                  "amendment_sha256": files.get(AMENDMENT), "scope": self.scope, "git": git, "sources": files,
                  "external_sources": external, "platform": platform_info(), "extra": self.extra_header,
                  "host_idle_at_open": host,
                  "policy_f": f, "boot_id": pathlib.Path("/proc/sys/kernel/random/boot_id").read_text().strip(),
                  "kernel_faults_start": self.faults0, "environment": env, "budget_s": self.budget_s,
                  "schedule": self.plan, "wall_start": time.time()}
        with open(self.dir / "attempt.json", "x") as fh:
            json.dump(header, fh, indent=1, default=str)
            fh.write("\n")
        open(self.dir / "ledger.jsonl", "x").close()
        self.prev_sha = sha256_file(self.dir / "attempt.json")          # the chain's root binds the header
        return header

    # -- sequence
    def expected(self):
        return None if self.status or self.next >= len(self.plan) else self.plan[self.next]

    def check_budget(self):
        """Raise (after ending the attempt) when the overall or the current density's main budget is exceeded."""
        m = self.expected()
        now = self.clock()
        if now - self.t0 > self.budget_s["overall"]:
            self.abort("BUDGET_OVERALL")
            raise AttemptError("overall budget exceeded")
        if m is not None and m["phase"] == "main":
            t = self.block_t0.setdefault(m["bars"], now)
            if now - t > self.budget_s["density_main"]:
                self.abort("BUDGET_DENSITY_%d" % m["bars"])
                raise AttemptError("density %d main budget exceeded" % m["bars"])

    def check_host(self):
        """Before a member: the runner's cores must still be idle; otherwise the attempt ends (HOST_NOT_IDLE)."""
        ok, info = self.host_check()
        if not ok:
            self.abort("HOST_NOT_IDLE", host=info)
            raise AttemptError("runner cores are not idle: %s" % info)
        return info

    def append(self, record):
        m = self.expected()
        if m is None:
            raise AttemptError("attempt is closed (%s)" % self.status)
        key = {k: record.get(k) for k in ("phase", "bars", "env", "arm")}
        if key != {k: m[k] for k in ("phase", "bars", "env", "arm")}:
            raise AttemptError("record %s is not the next scheduled member %s" % (key, m["name"]))
        if m["phase"] == "main":
            self.block_t0.setdefault(m["bars"], self.clock())
        record, serialization_problems = _finite_json(record)
        problems = record_problems(record) if record.get("operational_status") == "VALID" else []
        problems += serialization_problems
        line = dict(record, schedule_index=m["index"], name=m["name"], seq=self.next, prev_sha256=self.prev_sha,
                    protocol=PROTOCOL, seed=SEED, env_episode_index=EPISODE, record_problems=problems)
        text = json.dumps(line, sort_keys=True, allow_nan=False)
        with signals_held():                       # the line and its chain update land together (review r4 Q1)
            with open(self.dir / "ledger.jsonl", "a") as fh:
                fh.write(text + "\n")
            self.prev_sha = hashlib.sha256(text.encode()).hexdigest()
            self.next += 1
        try:
            faults = self.faults()
        except Exception as e:                                    # noqa: BLE001 - an unverifiable host ends it
            self.abort("FAULT_CHECK_ERROR", error=str(e))
            return line
        if faults != self.faults0:
            self.abort("HOST_FAULT", kernel_faults_now=faults)
        elif record.get("operational_status") != "VALID" or problems:
            self.abort("SMOKE_FAILED" if m["phase"] == "smoke" else
                       ("RECORD_INVALID" if problems else "MEMBER_NOT_VALID"))
        return line                                                 # the caller finishes (after its own checks)

    def abort(self, reason, **extra):
        if self.status is None:
            self._final("VOID_HOST_FAULT" if reason == "HOST_FAULT" else "ABORTED", reason, extra)

    def finish(self):
        if self.status is None:
            if self.next != len(self.plan):
                self._final("INCOMPLETE", "finish before the last scheduled member", {})
            else:
                self._final("COMPLETE", None, {})

    def _final(self, status, reason, extra):
        with signals_held():                       # status, fault read and write together (review r4 Q2)
            self._final_held(status, reason, extra)

    def _final_held(self, status, reason, extra):
        self.status = status
        try:
            faults_end, fault_error = self.faults(), None
        except Exception as e:                                    # noqa: BLE001 - still write final.json
            faults_end, fault_error = None, str(e)
        final = dict(extra, status=status, reason=reason, recorded=self.next,
                     unrun=[m["name"] for m in self.plan[self.next:]], kernel_faults_start=self.faults0,
                     kernel_faults_end=faults_end, fault_check_error=fault_error, last_record_sha256=self.prev_sha,
                     wall_end=time.time(), elapsed_s=self.clock() - self.t0)
        with open(self.dir / "final.json", "x") as fh:
            json.dump(final, fh, indent=1, default=str)
            fh.write("\n")


def read_ledger(attempt_dir):
    """Ledger lines after verifying the hash chain and the schedule order; AttemptError on any break."""
    plan = schedule()
    prev, out = sha256_file(pathlib.Path(attempt_dir) / "attempt.json"), []
    with open(pathlib.Path(attempt_dir) / "ledger.jsonl") as fh:
        for i, text in enumerate(fh.read().splitlines()):
            line = json.loads(text)
            if line.get("prev_sha256") != prev or line.get("seq") != i or line.get("name") != plan[i]["name"]:
                raise AttemptError("ledger chain or order broken at line %d" % i)
            prev = hashlib.sha256(text.encode()).hexdigest()
            out.append(line)
    final = json.loads((pathlib.Path(attempt_dir) / "final.json").read_text())
    if final["last_record_sha256"] != prev or final["recorded"] != len(out):
        raise AttemptError("final record does not close the ledger")
    return out, final


def main_records(attempt_dir):
    lines, final = read_ledger(attempt_dir)
    if final["status"] != "COMPLETE":
        raise AttemptError("attempt is %s (%s); no complete-pair summary" % (final["status"], final["reason"]))
    if final["kernel_faults_end"] != final["kernel_faults_start"]:
        raise AttemptError("kernel fault count changed during the attempt")
    if any(ln.get("record_problems") for ln in lines):
        raise AttemptError("a ledger record has problems")
    return [ln for ln in lines if ln["phase"] == "main"]


if __name__ == "__main__":
    print(json.dumps(STATUS))
