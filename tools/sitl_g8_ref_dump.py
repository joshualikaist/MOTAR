"""G8: reference dump for the SITL bridge's observation-equivalence check S3a (docs/plans/prereg_sitl_reference_g8g9_v1.md).

Runs Policy F through the unchanged camonly cell path (`full` condition = native F; GR-0's grid condition: E2, A3,
128 envs) on 70 and 205 bars, seed 4501, quota 1 (each env's first episode), with two evaluation-only recorders:
  - NAVRL_OBS_DUMP (stride 1, cap 80,000 rows so it never decimates): the exact 898-D actor observation per env per
    step, with episode uid, call index and episode step;
  - NAVRL_KROC2027_OCCLUSION_TRACE=1: both UAVs' states per consumed frame and every reset's bar layout.
Noninterference gate: every env's first-episode row in episodes.jsonl (outcome, completion step, min relative
distance) must equal the untraced GR-0 S00 cell's (quota 16) first-episode row. Reading DUMP_OK (both cells OK and
equal), RECORDER_INTERFERES, or INCOMPLETE / VOID_HOST_FAULT.
Fail-closed preflight as tools/camonly_run.py, plus this preregistration committed.
usage: tools/sitl_g8_ref_dump.py --approved-commit SHA --out DIR [--max-hours H]
"""
import argparse
import hashlib
import json
import pathlib
import subprocess
import sys

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(HERE))
import camonly_run as C                                     # noqa: E402

G, K = C.G, C.K
PREREG = "docs/plans/prereg_sitl_reference_g8g9_v1.md"
AMENDMENT = "docs/plans/prereg_sitl_reference_g8g9_v1_amendment1_2026-11-05.md"
AMENDMENT2 = "docs/plans/prereg_sitl_reference_g8g9_v1_amendment2_2026-11-05.md"
ZSITE = ROOT / "tools/zeroaction_site"
SITE = ROOT / "tools/staterec_site"
HOOK_FILES = ("tools/staterec_hook.py", "tools/staterec_site/sitecustomize.py", "tools/zeroaction_hook.py",
              "tools/zeroaction_site/sitecustomize.py")
CELLS = ((70, 4501), (205, 4501))
QUOTA = 1
REF = pathlib.Path("/home/fair/workspaces/aerial_gym_ws/src/aerial_gym_simulator/results/gain_randomization_gr0_v1/gr0_main/S00")
FIELDS = ("outcome", "completion_step", "min_relative_distance_m", "env_index", "env_episode_index")


def first_rows(path):
    rows = {}
    with open(path) as f:
        for line in f:
            e = json.loads(line)
            if e["env_episode_index"] == 0:
                rows[e["env_index"]] = tuple(e[k] for k in FIELDS)
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--approved-commit", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--max-hours", type=float, default=1.0)
    args = ap.parse_args()
    out_root = pathlib.Path(args.out).resolve()
    protocol = json.loads(K.PROTOCOL.read_text())
    head, problems, origins = C.preflight(protocol, args.approved_commit, out_root)
    for rel in (PREREG, AMENDMENT, AMENDMENT2) + HOOK_FILES:
        if subprocess.run(["git", "-C", str(ROOT), "ls-files", "--error-unmatch", rel], capture_output=True).returncode:
            problems.append("%s is not committed" % rel)
    for d, s in CELLS:
        if not (REF / ("E2_obstacle_aware__%dbars__seed%d" % (d, s)) / "episodes.jsonl").is_file():
            problems.append("reference missing %d/%d" % (d, s))
    module_sha = G.sha(ROOT / C.MODULE)
    start = G.host()
    report = {"unit": "G8", "prereg": PREREG, "amendment": AMENDMENT,
              "hook_sha256": {f: G.sha(ROOT / f) for f in HOOK_FILES}, "commit": head, "started": G.now(), "host_start": start,
              "import_origins": origins, "preflight_problems": problems, "quota": QUOTA}
    if problems:
        report["reading"] = "PREFLIGHT_FAIL"
        print(json.dumps(report, indent=1, default=str))
        return 2
    out_root.mkdir(parents=True)
    budget = K.GpuBudget(args.max_hours)
    records, cells, stop, void = [], [], False, False
    for d, s in CELLS:
        if budget.exhausted():
            report["budget_exhausted"] = True
            break
        dump = out_root / ("obs_dump_%dbars_seed%d.npz" % (d, s))
        staterec = out_root / ("state_record_%dbars_seed%d.npz" % (d, s))
        extra = {"NAVRL_SENSOR_ABLATION": "full", "CAMONLY_PY": str(C.WRAPPER), "NAVRL_KROC2027_OCCLUSION_TRACE": "1",
                 "NAVRL_OBS_DUMP": str(dump), "NAVRL_OBS_DUMP_STRIDE": "1", "NAVRL_OBS_DUMP_MAX": "80000",
                 "PYTHONPATH": "%s:%s" % (ROOT, SITE), "MOTAR_STATE_RECORD": str(staterec)}
        rec = G.run_cell(ROOT, protocol, out_root, "full", d, s, QUOTA, "SITL-G8", None, head, start, records,
                         G.CELL_TIMEOUT_S, extra, C.LAUNCHER, G.F_SHA)
        cell_dir = out_root / "full" / rec["cell_id"]
        if rec["verdict"] == "OK":
            extra_problems = C.ablation_problems(cell_dir, "full", module_sha)
            if not dump.is_file():
                extra_problems.append("observation dump missing")
            if not staterec.is_file():
                extra_problems.append("state record missing")
            if extra_problems:
                rec["problems"].extend(extra_problems)
                rec["verdict"] = "FAIL"
        ni = None
        if rec["verdict"] == "OK":
            a, b = first_rows(cell_dir / "episodes.jsonl"), first_rows(REF / rec["cell_id"] / "episodes.jsonl")
            diff = sorted(k for k in a if a.get(k) != b.get(k))
            ni = {"first_episode_rows": len(a), "mismatches": len(diff), "mismatch_envs": diff[:10],
                  "pass": len(a) == len(b) == protocol["evaluation"]["num_envs"] and not diff}
            rec["dump_sha256"] = hashlib.sha256(dump.read_bytes()).hexdigest()
            rec["dump_bytes"] = dump.stat().st_size
            rec["state_record_sha256"] = hashlib.sha256(staterec.read_bytes()).hexdigest()
        cells.append({"density": d, "seed": s, "dir": str(cell_dir), "verdict": rec["verdict"], "noninterference": ni,
                      "dump": str(dump), "dump_sha256": rec.get("dump_sha256"), "dump_bytes": rec.get("dump_bytes"),
                      "state_record": str(staterec), "state_record_sha256": rec.get("state_record_sha256")})
        print("   G8", d, s, rec["verdict"], ni, flush=True)
        if rec["verdict"] != "OK":
            void = rec["verdict"] == "VOID_HOST_FAULT"
            break
    # Amendment 2: the same cells with the pursuer held at zero action (hover to the 600-step timeout), occlusion
    # trace only, for the E2 target's full trajectories; they must equal the F-flying trajectories on common frames.
    zero = []
    if len(cells) == len(CELLS) and all(c["verdict"] == "OK" for c in cells) and not budget.exhausted():
        OT = __import__("importlib.util").util
        spec = OT.spec_from_file_location("ot_g8", str(ROOT / "motar_eval/occlusion_trace.py"))
        otm = OT.module_from_spec(spec)
        spec.loader.exec_module(otm)
        for d, s in CELLS:
            extra = {"NAVRL_SENSOR_ABLATION": "full", "CAMONLY_PY": str(C.WRAPPER), "NAVRL_KROC2027_OCCLUSION_TRACE": "1",
                     "PYTHONPATH": "%s:%s" % (ROOT, ZSITE), "MOTAR_ACTION_ZERO": "1"}
            rec = G.run_cell(ROOT, protocol, out_root, "zero", d, s, QUOTA, "SITL-G8Z", None, head, start, records,
                             G.CELL_TIMEOUT_S, extra, C.LAUNCHER, G.F_SHA)
            zdir, fdir = out_root / "zero" / rec["cell_id"], out_root / "full" / rec["cell_id"]
            ind = None
            if rec["verdict"] == "OK":
                zt, ft = otm.read_trace(zdir), otm.read_trace(fdir)
                zf, ff = otm.episode_frames(zt["frames"]), otm.episode_frames(ft["frames"])
                n_env, worst, frames = 0, 0.0, 0
                for (env, epi), fr in ff.items():
                    if epi != 0 or (env, 0) not in zf:
                        continue
                    zr = zf[(env, 0)]
                    m = min(len(fr), len(zr))
                    diff = float(abs(fr["t_pos"][:m] - zr["t_pos"][:m]).max()) if m else 0.0
                    worst, n_env, frames = max(worst, diff), n_env + 1, frames + m
                zlen = [len(v) for (env, epi), v in zf.items() if epi == 0]
                ind = {"envs": n_env, "common_frames": frames, "max_abs_t_pos_diff_m": worst, "pass": n_env > 0 and worst == 0.0,
                       "zero_episode_frames_min": min(zlen) if zlen else None, "zero_episode_frames_max": max(zlen) if zlen else None}
            zero.append({"density": d, "seed": s, "dir": str(zdir), "verdict": rec["verdict"], "target_independence": ind})
            print("   G8 zero", d, s, rec["verdict"], ind, flush=True)
            if rec["verdict"] != "OK":
                break
    report["zero_action_cells"] = zero
    complete = len(cells) == len(CELLS) and all(c["verdict"] == "OK" for c in cells)
    zero_ok = len(zero) == len(CELLS) and all(z["verdict"] == "OK" for z in zero)
    reading = ("VOID_HOST_FAULT" if void else "INCOMPLETE" if not (complete and zero_ok) else
               "RECORDER_INTERFERES" if not all(c["noninterference"]["pass"] for c in cells) else
               "TARGET_DEPENDS_ON_PURSUER" if not all(z["target_independence"]["pass"] for z in zero) else "DUMP_OK")
    report.update(reading=reading, cells=cells, records=records, host_end=G.host(), finished=G.now())
    (out_root / "g8_report.json").write_text(json.dumps(report, indent=1, default=str) + "\n")
    print(json.dumps({k: report[k] for k in ("unit", "reading", "commit")}, indent=1))
    return 0 if reading == "DUMP_OK" else 1


if __name__ == "__main__":
    raise SystemExit(main())
