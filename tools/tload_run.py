"""T-LOAD: does concurrent CPU work change a GPU evaluation? (docs/plans/prereg_tload_v1.md; D-2026-11-05-1)

Runs three of the camera-only study's cells through the unchanged camonly cell path (tools/camonly_run.run_cell:
camera-only launcher copy, ablation entry point, GR-0's grid condition, quota 16) five times each, under
interleaved load arms, and compares the per-episode files byte for byte:

  L0a  idle
  L1   stress-ng --cpu 4 --hdd 1 on the disjoint cores 6,7,12,13 (CPU and disk)
  L2   stress-ng --cpu 18 on the runner's own cores 0-3,8-11,14-23 (the worst case)
  L3   a real job on the disjoint cores: the camera-only sensor check (Warp CPU renders), looped, nice 10
  L0b  idle again

Cells: full 205/4501, no_lidar 205/4501, full 70/4501. Every load runs with CUDA_VISIBLE_DEVICES="", an address-
space cap and its own process group; it writes only under --load-scratch (outside every repository). The runner
itself must be pinned to 0-3,8-11,14-23 (taskset by the caller); preflight checks the affinity.

Manipulation check (the load must be present): each load process is alive from before the cell starts until it ends,
and its CPU time per cell second is at least MIN_CORES[arm]. Reading (pure function `reading`):
  INCOMPLETE / VOID_HOST_FAULT   a cell did not finish OK
  REFERENCE_MISMATCH             L0a of a 205-bar cell differs from its idle historical references
  NONDETERMINISTIC_IDLE          L0a and L0b differ in some cell
  LOAD_NOT_APPLIED               a load arm failed its manipulation check (no reading for that arm)
  LOAD_SENSITIVE                 L1 or L3 differs from L0a in some cell
  DISJOINT_ONLY                  only L2 differs
  LOAD_INVARIANT                 every arm equals L0a in every cell
Equality = episodes.jsonl and kroc2027_episodes.jsonl byte-identical and gr_run.identity_compare result_equal.
usage: tools/tload_run.py --approved-commit SHA --out DIR --load-scratch DIR --cplanner-tree DIR [--max-hours H]
       tools/tload_run.py --dry-run --out DIR --load-scratch DIR --cplanner-tree DIR   (CPU smoke: fake cells)
"""
import argparse
import hashlib
import json
import os
import pathlib
import resource
import shutil
import signal
import subprocess
import sys
import time

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(HERE))
import camonly_run as C                                     # noqa: E402

G, K = C.G, C.K
PREREG = "docs/plans/prereg_tload_v1.md"
SENSOR_CHECK = "docs/evidence/camonly_2026-11-04/sensor/sensor_check.py"
RUNNER_CORES = "0-3,8-11,14-23"
DISJOINT = "6,7,12,13"
CELLS = (("full", 205, 4501), ("no_lidar", 205, 4501), ("full", 70, 4501))
ARMS = ("L0a", "L1", "L2", "L3", "L0b")
LOAD_ARMS = ("L1", "L2", "L3")
MIN_CORES = {"L1": 3.0, "L2": 12.0, "L3": 0.8}
AS_CAP = {"L1": 4 << 30, "L2": 4 << 30, "L3": 24 << 30}
RES = pathlib.Path("/home/fair/workspaces/aerial_gym_ws/src/aerial_gym_simulator/results")
CELL_ID = "E2_obstacle_aware__%dbars__seed%d"
# Historical references (prereg section 4): idle ones gate the pipeline; the GR-0 70/4501 cell ran under a heavy
# same-core job on 2026-11-04 and is reported only.
REFS = {
    ("full", 205, 4501): {"gate": [RES / "camonly_c1c2_v1/c1/full", RES / "camonly_c1c2_v1/c1_r2/full"],
                          "report": [RES / "gain_randomization_gr0_v1/gr0_main/S00"]},
    ("no_lidar", 205, 4501): {"gate": [RES / "camonly_c1c2_v1/c1/no_lidar", RES / "camonly_c1c2_v1/c1_r2/no_lidar",
                                       RES / "camonly_c1c2_v1/c2/no_lidar"], "report": []},
    ("full", 70, 4501): {"gate": [], "report": [RES / "gain_randomization_gr0_v1/gr0_main/S00"]},
}
PY = "/home/fair/miniconda3/envs/aerialgym/bin/python"


def sha(p):
    return hashlib.sha256(pathlib.Path(p).read_bytes()).hexdigest()


def digest(cell_dir):
    cell_dir = pathlib.Path(cell_dir)
    return {f: (sha(cell_dir / f) if (cell_dir / f).is_file() else None)
            for f in ("episodes.jsonl", "kroc2027_episodes.jsonl")}


def equal(a_dir, b_dir):
    da, db = digest(a_dir), digest(b_dir)
    files_equal = None not in da.values() and da == db
    try:
        comp = G.identity_compare(pathlib.Path(a_dir), pathlib.Path(b_dir))
        result_equal = bool(comp["result_equal"])
    except (OSError, ValueError, KeyError) as e:
        comp, result_equal = {"error": str(e)}, False
    return {"files_equal": files_equal, "result_equal": result_equal, "equal": bool(files_equal and result_equal),
            "a": da, "b": db}


def load_command(arm, scratch):
    if arm == "L1":
        return ["taskset", "-c", DISJOINT, "stress-ng", "--cpu", "4", "--hdd", "1", "--hdd-bytes", "1G",
                "--temp-path", str(scratch), "--metrics-brief", "--timeout", "3600s"], signal.SIGINT
    if arm == "L2":
        return ["taskset", "-c", RUNNER_CORES, "stress-ng", "--cpu", "18", "--metrics-brief",
                "--timeout", "3600s"], signal.SIGINT
    if arm == "L3":
        sc = scratch / "sensor_check.py"
        shutil.copyfile(ROOT / SENSOR_CHECK, sc)
        loop = ('i=0; while true; do "%s" -I -B "%s" "%s" "$CPLANNER_TREE" "%s/sc_$i.json" 300 > "%s/sc_$i.out" 2>&1; '
                'i=$((i+1)); done' % (PY, sc, ROOT, scratch, scratch))
        return ["taskset", "-c", DISJOINT, "nice", "-n", "10", "bash", "-c", loop], signal.SIGTERM
    raise ValueError(arm)


def group_cpu_s(pgid):
    """CPU seconds of every live process in a process group, including each member's reaped children."""
    tick = os.sysconf("SC_CLK_TCK")
    total = 0
    for d in pathlib.Path("/proc").iterdir():
        if not d.name.isdigit():
            continue
        try:
            fields = (d / "stat").read_text().rsplit(")", 1)[1].split()
        except OSError:
            continue
        if int(fields[2]) == pgid:                  # fields[0] = state; pgrp is stat field 5
            total += sum(int(fields[i]) for i in (11, 12, 13, 14))   # utime stime cutime cstime
    return total / tick


class Load:
    def __init__(self, arm, scratch, cplanner_tree):
        self.arm, self.scratch = arm, pathlib.Path(scratch)
        self.scratch.mkdir(parents=True, exist_ok=False)
        self.cmd, self.stop_signal = load_command(arm, self.scratch)
        env = {k: v for k, v in os.environ.items() if not k.startswith("NAVRL_")}
        env.update({"CUDA_VISIBLE_DEVICES": "", "PYTHONDONTWRITEBYTECODE": "1", "PYTHONNOUSERSITE": "1",
                    "CPLANNER_TREE": str(cplanner_tree)})
        cap = AS_CAP[arm]

        def pre():
            os.setsid()
            resource.setrlimit(resource.RLIMIT_AS, (cap, cap))
        self.log = open(self.scratch / "load.log", "w")
        self.t0 = time.time()
        self.proc = subprocess.Popen(self.cmd, stdout=self.log, stderr=subprocess.STDOUT, env=env, preexec_fn=pre,
                                     cwd=str(self.scratch))

    def alive(self):
        return self.proc.poll() is None

    def stop(self):
        alive_at_stop = self.alive()
        cpu_group = group_cpu_s(self.proc.pid) if alive_at_stop else None
        sampled_wall = time.time() - self.t0
        if alive_at_stop:
            os.killpg(self.proc.pid, self.stop_signal)
        rus = None
        deadline = time.time() + 30
        while time.time() < deadline:
            pid, status, rus = os.wait4(self.proc.pid, os.WNOHANG)
            if pid:
                break
            time.sleep(0.2)
        else:
            os.killpg(self.proc.pid, signal.SIGKILL)
            pid, status, rus = os.wait4(self.proc.pid, 0)
        try:                                                 # leftover group members (bash loop children)
            os.killpg(self.proc.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        self.log.close()
        return {"arm": self.arm, "cmd": self.cmd, "alive_at_stop": alive_at_stop, "wall_s": time.time() - self.t0,
                "sampled_wall_s": sampled_wall,
                "cpu_s": cpu_group, "cpu_s_wait4": (rus.ru_utime + rus.ru_stime) if rus else None,
                "scratch": str(self.scratch)}


def manipulation_ok(load, cell_wall_s):
    if load is None:
        return True, None
    cores = (load["cpu_s"] or 0.0) / max(1e-9, load["sampled_wall_s"])
    ok = bool(load["alive_before_cell"] and load["alive_at_stop"] and cores >= MIN_CORES[load["arm"]])
    return ok, cores


def reading(runs):
    """runs: list of {cell: (cond, d, s), arm, verdict, eq_L0a: bool|None, load_ok: bool, ref_gate_equal: bool|None}."""
    if any(r["verdict"] == "VOID_HOST_FAULT" for r in runs):
        return "VOID_HOST_FAULT", {}
    expected = {(c, a) for c in CELLS for a in ARMS}
    got = {(tuple(r["cell"]), r["arm"]) for r in runs if r["verdict"] == "OK"}
    if got != expected:
        return "INCOMPLETE", {"missing": sorted(map(str, expected - got))}
    by = {(tuple(r["cell"]), r["arm"]): r for r in runs}
    if any(by[(c, "L0a")]["ref_gate_equal"] is False for c in CELLS):
        return "REFERENCE_MISMATCH", {}
    if not all(by[(c, "L0b")]["eq_L0a"] for c in CELLS):
        return "NONDETERMINISTIC_IDLE", {}
    arm_state = {}
    for a in LOAD_ARMS:
        if not all(by[(c, a)]["load_ok"] for c in CELLS):
            arm_state[a] = "LOAD_NOT_APPLIED"
        else:
            arm_state[a] = "EQUAL" if all(by[(c, a)]["eq_L0a"] for c in CELLS) else "DIFFERS"
    if "LOAD_NOT_APPLIED" in arm_state.values():
        return "LOAD_NOT_APPLIED", arm_state
    if arm_state["L1"] == "DIFFERS" or arm_state["L3"] == "DIFFERS":
        return "LOAD_SENSITIVE", arm_state
    if arm_state["L2"] == "DIFFERS":
        return "DISJOINT_ONLY", arm_state
    return "LOAD_INVARIANT", arm_state


def fake_run_cell(protocol, run_root, condition, density, seed, run_id, head, start, records, module_sha):
    """--dry-run only: copy the idle reference cell instead of launching (exercises loads, readout and reading)."""
    cid = CELL_ID % (density, seed)
    src = (REFS[(condition, density, seed)]["gate"] or REFS[(condition, density, seed)]["report"])[0] / cid
    dst = run_root / condition / cid
    dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.copytree(src, dst)
    time.sleep(15)
    rec = {"level": condition, "cell_id": cid, "density": density, "seed": seed, "wall_s": 15.0, "verdict": "OK",
           "problems": [], "dry_run": True}
    records.append(rec)
    return rec


def preflight_extra(out_root, scratch_root, cplanner_tree, approved):
    problems = []
    if subprocess.run(["git", "-C", str(ROOT), "ls-files", "--error-unmatch", PREREG],
                      capture_output=True).returncode:
        problems.append("%s is not committed" % PREREG)
    if not shutil.which("stress-ng"):
        problems.append("stress-ng not found")
    want = set()
    for part in RUNNER_CORES.split(","):
        a, _, b = part.partition("-")
        want |= set(range(int(a), int(b or a) + 1))
    if os.sched_getaffinity(0) != want:
        problems.append("runner affinity %s != %s (start it under taskset -c %s)" % (
            sorted(os.sched_getaffinity(0)), RUNNER_CORES, RUNNER_CORES))
    online = pathlib.Path("/sys/devices/system/cpu/online").read_text().strip()
    if online != "0-3,6-23":
        problems.append("online CPUs %s, expected 0-3,6-23" % online)
    if not (pathlib.Path(cplanner_tree) / "cplanner/motar/replay.py").is_file():
        problems.append("--cplanner-tree has no cplanner/motar/replay.py")
    if scratch_root.exists():
        problems.append("load scratch already exists: %s" % scratch_root)
    if K.is_inside(str(scratch_root), ROOT) or K.is_inside(str(scratch_root), RES.parent):
        problems.append("load scratch must be outside every repository")
    free = shutil.disk_usage(str(RES)).free
    if free < (3 << 30):
        problems.append("disk free %.1f GB < 3 GB" % (free / 2 ** 30))
    for cell, refs in REFS.items():
        for p in refs["gate"] + refs["report"]:
            if not (p / (CELL_ID % cell[1:]) / "episodes.jsonl").is_file():
                problems.append("reference missing: %s" % p)
    return problems


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--approved-commit")
    ap.add_argument("--out", required=True)
    ap.add_argument("--load-scratch", required=True)
    ap.add_argument("--cplanner-tree", required=True)
    ap.add_argument("--max-hours", type=float, default=2.0)
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()
    out_root = pathlib.Path(args.out).resolve()
    scratch_root = pathlib.Path(args.load_scratch).resolve()
    protocol = json.loads(K.PROTOCOL.read_text())
    if args.dry_run:
        head, problems, origins = K.git("rev-parse", "HEAD", root=ROOT).strip(), [], {}
        problems_extra = [p for p in preflight_extra(out_root, scratch_root, args.cplanner_tree, None)
                          if "not committed" not in p]
        if out_root.exists():
            problems_extra.append("output already exists: %s" % out_root)
    else:
        if not args.approved_commit:
            raise SystemExit("--approved-commit is required")
        head, problems, origins = C.preflight(protocol, args.approved_commit, out_root)
        problems_extra = preflight_extra(out_root, scratch_root, args.cplanner_tree, args.approved_commit)
    problems = list(problems) + problems_extra
    module_sha = G.sha(ROOT / C.MODULE)
    start = G.host()
    report = {"unit": "T-LOAD", "prereg": PREREG, "tree": str(ROOT), "commit": head, "dry_run": args.dry_run,
              "started": G.now(), "host_start": start, "import_origins": origins, "module_sha256": module_sha,
              "launcher_sha256": C.LAUNCHER_SHA, "runner_cores": RUNNER_CORES, "disjoint_cores": DISJOINT,
              "min_cores": MIN_CORES, "preflight_problems": problems,
              "env_allowlisted": {k: os.environ[k] for k in sorted(os.environ) if C.ENV_ALLOW.match(k)}}
    if problems:
        report["reading"] = "PREFLIGHT_FAIL"
        print(json.dumps(report, indent=1, default=str))
        return 2
    out_root.mkdir(parents=True)
    scratch_root.mkdir(parents=True)
    budget = K.GpuBudget(args.max_hours)
    run_cell = fake_run_cell if args.dry_run else C.run_cell
    records, runs, loads = [], [], []
    stop = False
    idx = 0
    for cell in CELLS:
        cond, d, s = cell
        for arm in ARMS:
            if budget.exhausted():
                report["budget_exhausted"] = True
                stop = True
                break
            idx += 1
            run_root = out_root / ("%02d_%s" % (idx, arm))
            load = None
            if arm in LOAD_ARMS:
                load_obj = Load(arm, scratch_root / ("%02d_%s" % (idx, arm)), args.cplanner_tree)
                time.sleep(5)
                alive_before = load_obj.alive()
            print("== T-LOAD run %02d %s %s %s" % (idx, arm, cell, G.now()), flush=True)
            try:
                rec = run_cell(protocol, run_root, cond, d, s, "TLOAD", head, start, records, module_sha)
            finally:
                if arm in LOAD_ARMS:
                    load = load_obj.stop()
                    load["alive_before_cell"] = alive_before
                    loads.append(load)
            cell_dir = run_root / cond / (CELL_ID % (d, s))
            ok, cores = manipulation_ok(load, rec.get("wall_s", 0.0))
            run = {"idx": idx, "cell": list(cell), "arm": arm, "verdict": rec["verdict"], "wall_s": rec.get("wall_s"),
                   "dir": str(cell_dir), "digest": digest(cell_dir), "load": load, "load_cores": cores,
                   "load_ok": ok, "eq_L0a": None, "ref_gate_equal": None, "ref": {}}
            if rec["verdict"] == "OK":
                if arm == "L0a":
                    gates = [equal(p / (CELL_ID % (d, s)), cell_dir) for p in REFS[cell]["gate"]]
                    run["ref"] = {"gate": [dict(g, path=str(p)) for g, p in zip(gates, REFS[cell]["gate"])],
                                  "report": [dict(equal(p / (CELL_ID % (d, s)), cell_dir), path=str(p))
                                             for p in REFS[cell]["report"]]}
                    run["ref_gate_equal"] = all(g["equal"] for g in gates) if gates else None
                else:
                    l0a = next((r for r in runs if tuple(r["cell"]) == cell and r["arm"] == "L0a"), None)
                    run["eq_L0a"] = (equal(l0a["dir"], cell_dir)["equal"] if l0a and l0a["verdict"] == "OK"
                                     else None)
            runs.append(run)
            print("   %s eq_L0a=%s ref=%s load_ok=%s cores=%s wall=%.0fs" % (
                rec["verdict"], run["eq_L0a"], run["ref_gate_equal"], ok,
                None if cores is None else round(cores, 2), rec.get("wall_s", 0.0)), flush=True)
            if rec["verdict"] != "OK":
                stop = True
                break
        if stop:
            break
    report["reading"], report["arm_state"] = reading(runs)
    report["runs"] = runs
    report["cell_records"] = records
    report["host_end"] = G.host()
    report["finished"] = G.now()
    name = "tload_dryrun_report.json" if args.dry_run else "tload_report.json"
    (out_root / name).write_text(json.dumps(report, indent=1, default=str) + "\n")
    print(json.dumps({k: report[k] for k in ("unit", "reading", "arm_state", "commit", "dry_run")}, indent=1))
    return 0 if report["reading"] in ("LOAD_INVARIANT", "DISJOINT_ONLY", "LOAD_SENSITIVE") else 1


if __name__ == "__main__":
    raise SystemExit(main())
