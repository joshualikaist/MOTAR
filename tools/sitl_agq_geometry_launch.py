"""Launch the reviewed AGQ geometry-v1 unit once (docs/plans/prereg_sitl_agq_geometry_v1_2026-11-06.md).

Default: a read-only preparation check that prints the fixed command and refuses nothing it cannot verify.
--launch: requires the recorded approval file for this exact clean HEAD, starts the unit once under the fixed
envelope (outer timeout, nice, cores 6/7/12/13, 16 GiB address space, CUDA hidden, one compute thread) and writes
an append-only process record. It never repeats: an existing state file or attempt directory refuses.
"""
import datetime
import json
import os
from pathlib import Path
import re
import resource
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
UNIT = Path.home() / "motar_tmp" / "agq_geometry_diag_20261106"
APPROVAL = UNIT / "execution_approval.json"
ATTEMPT = UNIT / "attempt01"
RUN = UNIT / "run01"
PYTHON = "/home/fair/miniconda3/envs/aerialgym/bin/python"
AS_LIMIT = 16 * 1024 ** 3
OUTER_TIMEOUT_S, OUTER_KILL_S = 1200, 120      # -k exceeds the worst-case cleanup (review r1 m11: about 95 s)
COMMAND = ["timeout", "-s", "TERM", "-k", str(OUTER_KILL_S), str(OUTER_TIMEOUT_S), "nice", "-n", "10", "taskset",
           "-c", "6,7,12,13", PYTHON, "-B", "tools/sitl_agq_geometry_diag.py", "--execute", "--approval-file",
           str(APPROVAL), "--attempt-dir", str(ATTEMPT)]
SCOPE = ("AGQ geometry-v1: 4 fixed development members (205/env5 native gains: native then training-box collision; "
         "205/env2 candidate gains: training-box then native collision)")


def utc():
    return datetime.datetime.now(datetime.timezone.utc).isoformat()


def git(*args):
    return subprocess.check_output(["git", "-C", str(ROOT), *args], text=True).strip()


def faults():
    p = subprocess.run(["journalctl", "-k", "-b", "0", "--no-pager"], capture_output=True, text=True, timeout=30)
    if p.returncode or not p.stdout:
        raise RuntimeError("kernel log unreadable: %s" % p.stderr[:200])
    return len(re.findall(r"Machine check|segfault|general protection", p.stdout))


def save(path, record):
    tmp = path.with_suffix(".pending.json")
    with tmp.open("x") as f:
        json.dump(record, f, indent=2)
        f.write("\n")
    os.replace(str(tmp), str(path))


def prepared():
    """The checks before any launch; raises SystemExit with a reason on the first failure."""
    if not APPROVAL.is_file():
        raise SystemExit("no approval file %s" % APPROVAL)
    approval = json.loads(APPROVAL.read_text())
    problems = []
    if approval.get("commit") != git("rev-parse", "HEAD"):
        problems.append("approval commit is not HEAD")
    if git("status", "--porcelain"):
        problems.append("worktree is not clean")
    if approval.get("scope") != SCOPE:
        problems.append("approval scope differs")
    if type(approval.get("skip_max_s")) is not float or approval["skip_max_s"] != 0.15:
        problems.append("skip_max_s must be 0.15")
    if os.path.lexists(ATTEMPT):
        problems.append("attempt directory already exists")
    if problems:
        raise SystemExit("refused: " + "; ".join(problems))
    env = dict(os.environ, CUDA_VISIBLE_DEVICES="", PYTHONNOUSERSITE="1", OMP_NUM_THREADS="1", MKL_NUM_THREADS="1",
               PYTHONPATH=str(ROOT), PYTHONDONTWRITEBYTECODE="1")
    record = {"phase": "PREPARED", "command": COMMAND, "cwd": str(ROOT), "source_commit": approval["commit"],
              "attempt_dir": str(ATTEMPT), "approval_file": str(APPROVAL), "rlimit_as_bytes": AS_LIMIT,
              "env": {k: env[k] for k in ("CUDA_VISIBLE_DEVICES", "PYTHONNOUSERSITE", "OMP_NUM_THREADS",
                                          "MKL_NUM_THREADS", "PYTHONPATH")},
              "kernel_faults_before": faults(), "prepared_at": utc(), "source_frozen": False}
    return approval, env, record


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    if argv not in ([], ["--launch"]):
        raise SystemExit("usage: sitl_agq_geometry_launch.py [--launch]")
    approval, env, record = prepared()
    if argv != ["--launch"]:
        print(json.dumps(record, indent=2))
        return 0

    def limits():
        resource.setrlimit(resource.RLIMIT_AS, (AS_LIMIT, AS_LIMIT))

    RUN.mkdir(parents=True, exist_ok=False)                      # never automatically repeat
    state = RUN / "execution_state.json"
    log_path = RUN / "execution_stdout.log"
    record.update(phase="STARTING", source_frozen=True, started_at=utc(), stdout=str(log_path))
    save(state, record)
    with log_path.open("x") as log:
        proc = subprocess.Popen(COMMAND, cwd=str(ROOT), env=env, preexec_fn=limits, start_new_session=True,
                                stdout=log, stderr=subprocess.STDOUT)
        record.update(phase="RUNNING", timeout_pid=proc.pid)
        try:
            record["timeout_proc_limits"] = Path("/proc/%d/limits" % proc.pid).read_text()
        except FileNotFoundError:
            record["timeout_proc_limits"] = "NOT_RECORDED_PROCESS_ALREADY_EXITED"
        save(state, record)
        code = proc.wait()
    record.update(phase="ENDED", exit_code=code, ended_at=utc(), source_frozen=False,
                  kernel_faults_after=faults(), head_after=git("rev-parse", "HEAD"),
                  dirty_after=git("status", "--porcelain"))
    save(state, record)
    with (RUN / "execution_process_record.json").open("x") as f:
        json.dump(record, f, indent=2)
        f.write("\n")
    print(json.dumps({k: record[k] for k in ("phase", "exit_code", "started_at", "ended_at", "kernel_faults_before",
                                             "kernel_faults_after", "head_after", "dirty_after")}, indent=2))
    ok = (code == 0 and record["kernel_faults_before"] == record["kernel_faults_after"]
          and not record["dirty_after"] and record["head_after"] == approval["commit"])
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
