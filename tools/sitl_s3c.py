"""S3c v1: the S3b loop with a heavier vehicle (PX4 iris) and with a one-step command delay (plan stage S3c).

Preregistration: docs/plans/prereg_sitl_s3c_v1_2026-11-06.md. Reported, not gated: the plan's S3c reads "the gap
reported; it decides whether GR-1b/GR-2 policies are needed".

Everything is tools/sitl_s3b_formal.py's: the same episode function, owned fresh-rootfs session, parameter read-back,
lateness bound, scene inputs and validity rules. Only the condition differs:
  - iris:   model navrl_iris (PX4 iris 1.5 kg, T/W about 1.9, PX4's own tuned rate loops; plus the rangefinder height
            reference and MOTAR's speed/tilt/yaw limits as airframe 10091); the velocity loop set as S3b
            (MPC_XY_VEL_P_ACC 2.5, _I_ACC 0.1, _D_ACC 0); rate gains and THR_MDL_FAC left at iris defaults;
  - delay1: navrl_quad exactly as S3b, every governed command sent one 0.1 s step late (hover on the first step).
Scenes: the first 40 eligible 70-bar G8 r2 scenes (S3b block A's first 40), one boot each per arm.

  tools/sitl_s3c.py --check | --launch | --analyze [ATTEMPT_DIR]
"""
import argparse
import collections
import json
import os
import pathlib
import sys
import time
import types

TOOLS = pathlib.Path(__file__).resolve().parent
ROOT = TOOLS.parent
sys.path.insert(0, str(TOOLS))
import sitl_s3b_formal as SF                                    # noqa: E402

PROTOCOL = "s3c-v1"
PREREG = "docs/plans/prereg_sitl_s3c_v1_2026-11-06.md"
SCOPE = ("S3c v1: Frozen Policy F through PX4 v1.14.3 SITL, G8 r2 seed 4501, first 40 eligible 70-bar scenes; arm iris "
         "(navrl_iris, delay 0) then arm delay1 (navrl_quad, one-step command delay); development smoke01 (the first "
         "scheduled episode once, not read) then attempt01; episode lateness bound 0.15 s; no retries")
UNIT_DIR = pathlib.Path.home() / "motar_tmp" / "s3c_20261106"
ATTEMPT_DIR = UNIT_DIR / "attempt01"
SMOKE_DIR = UNIT_DIR / "smoke01"
N_SCENES = 40
ARMS = (("iris", {"model": "navrl_iris", "delay_steps": 0},
         {"MPC_XY_VEL_P_ACC": 2.5, "MPC_XY_VEL_I_ACC": 0.1, "MPC_XY_VEL_D_ACC": 0.0}),
        ("delay1", {"model": "navrl_quad", "delay_steps": 1}, None))      # None: S3b's parameters
INVALID_ABORT_TOTAL = 12                # more than 12 INVALID episodes of the 80 stop the unit
OWN_SOURCES = ("tools/sitl_s3c.py", "tests/test_sitl_s3c_s4.py") + SF.OWN_SOURCES + (PREREG,)


def schedule(g8=SF.G8):
    envs = SF.eligible_envs(SF.read_jsonl(g8 / "zero" / (SF.CELL % 70) / "kroc2027_episodes.jsonl"))[:N_SCENES]
    return [{"arm": arm, "bars": 70, "env": e} for arm, _, _ in ARMS for e in envs]


def v0_with_params(V0, params):
    """The development runner module's namespace with PARAMS replaced (None keeps S3b's)."""
    if params is None:
        return V0
    ns = types.SimpleNamespace(**{k: getattr(V0, k) for k in dir(V0) if not k.startswith("__")})
    ns.PARAMS = dict(params)
    return ns


def check():
    out = {"input_problems": SF.input_problems(), "schedule": collections.Counter(s["arm"] for s in schedule()),
           "environment_problems": SF.environment_report()[1]}
    print(json.dumps(out, indent=1, default=str))
    return out


def smoke_problems(smoke_dir, head=None, sources=None):
    """attempt01 needs a COMPLETE smoke01 whose single episode is VALID (S3c/S4 review r1: a first flight of a new
    stack should not consume the single-use attempt), run at this HEAD with these source bytes (review r2 s2)."""
    f, e, a = smoke_dir / "final.json", smoke_dir / "episodes.jsonl", smoke_dir / "attempt.json"
    if not f.is_file() or not e.is_file() or not a.is_file():
        return ["no development smoke record in %s" % smoke_dir]
    att = json.loads(a.read_text())
    if head is not None and (att.get("git") or {}).get("head") != head:
        return ["development smoke ran at %s, not HEAD %s" % ((att.get("git") or {}).get("head"), head)]
    if sources is not None and att.get("sources") != sources:
        return ["development smoke ran other source bytes"]
    rows = SF.read_jsonl(e)
    final = json.loads(f.read_text())
    if final.get("status") != "COMPLETE" or len(rows) != 1 or not rows[0].get("valid"):
        return ["development smoke not COMPLETE and VALID: %s, %s" % (final.get("status"),
                                                                       [r.get("invalid_reason") for r in rows])]
    return []


def launch_unit(scope, unit_dir, attempt_dir, sources, sched_fn, attempt_extra, run_one_fn, extra_problems=(),
                invalid_abort=None, smoke_dir=None, smoke=False):
    """The S3b launch path for a unit whose schedule items carry an "arm": refusals before anything exists, then
    SF.run_unit (per-episode idle check, per-arm runtime binding, tree, fault and stop rules, termination recorded)
    and SF.write_final on every exit path."""
    import signal
    import sitl_dev08_attempt as AT
    import sitl_bridge as B
    signal.signal(signal.SIGTERM, SF._on_sigterm)
    gs = AT.git_state(ROOT)
    approval_path = unit_dir / "execution_approval.json"
    problems = SF.launch_problems(AT, B, gs, approval_path, scope=scope, sources=sources, attempt_dir=attempt_dir)
    problems += list(extra_problems)
    if not smoke and smoke_dir is not None:
        problems += smoke_problems(smoke_dir, gs["head"], {s: SF.sha256_file(ROOT / s) for s in sources})
    faults0 = None
    if not problems:
        try:
            faults0 = AT.kernel_faults()
        except Exception as e:                               # noqa: BLE001
            problems.append("kernel fault baseline unreadable: %s" % e)
    if problems:
        raise SystemExit("refused:\n  " + "\n  ".join(problems))
    attempt_dir.mkdir(parents=True)
    sched, done, terminated = None, 0, None
    try:
        sched = sched_fn()[:1] if smoke else sched_fn()
        ext, _ = AT.external_binding(SF.EXTERNAL_SOURCES)
        rec = {"scope": scope, "development_smoke": bool(smoke), "git": gs, "approval_sha256": SF.sha256_file(approval_path), "inputs": SF.INPUT_SHA256,
               "f_sha256": SF.F_SHA256, "sources": {s: SF.sha256_file(ROOT / s) for s in sources},
               "external_sources": ext, "environment": SF.environment_report()[0],
               "environment_keys": sorted(os.environ), "idle": AT.idle_check()[1], "schedule": sched,
               "kernel_faults_before": faults0, "started": time.time()}
        rec.update(attempt_extra)
        (attempt_dir / "attempt.json").write_text(json.dumps(rec, indent=1, default=str) + "\n")
        import sitl_owned_session as OS
        import sitl_s3b_closed_loop as V0
        full, zero = SF.G8 / "full" / (SF.CELL % 70), SF.G8 / "zero" / (SF.CELL % 70)
        specs = {e["env"]: e for e in V0.episodes_from_trace(full, 1000, 0, B.ROOT, zero)}
        B.setup(specs[sched[0]["env"]]["receipt_env"])
        policy = B.Policy()
        SF.run_unit.terminated = None
        status, reason, done = SF.run_unit(
            sched, specs, run_one_fn(B, OS, V0, policy), AT, attempt_dir / "episodes.jsonl", key="arm", primary=None,
            invalid_abort_a=invalid_abort if invalid_abort is not None else INVALID_ABORT_TOTAL)
        terminated = SF.run_unit.terminated
    except BaseException as e:                               # noqa: BLE001 - recorded, then re-raised
        status, reason = "ABORTED", "%s: %s" % (e.__class__.__name__, e)
        if not isinstance(e, Exception):
            terminated = e
    SF.write_final(attempt_dir, AT, status, reason, done, sched, faults0)
    if terminated is not None:
        raise terminated


def launch(smoke=False):
    import sitl_dev08_attempt as AT
    sources = sorted(set(OWN_SOURCES) | set(AT.SOURCES))
    out_dir = SMOKE_DIR if smoke else ATTEMPT_DIR

    def run_one_fn(B, OS, V0, policy):
        arms = {a: (c, v0_with_params(V0, p)) for a, c, p in ARMS}
        return lambda i, item, spec: SF.run_episode(B, OS, arms[item["arm"]][1], spec, out_dir / ("ep%03d" % i),
                                                    policy, condition=arms[item["arm"]][0])
    launch_unit(SCOPE, UNIT_DIR, out_dir, sources, schedule, {"protocol": PROTOCOL, "prereg": PREREG, "arms": ARMS},
                run_one_fn, smoke_dir=SMOKE_DIR, smoke=smoke)


def arm_integrity_problems(attempt, final, rows):
    """S3b's integrity rules for arm-keyed schedules: the schedule's prefix in order, all of it when COMPLETE, no
    fault increase, the tree unchanged."""
    if attempt is None:
        return ["no attempt record"]
    p = []
    sched = attempt.get("schedule", [])
    if [(r["arm"], r["env"]) for r in rows] != [(s["arm"], s["env"]) for s in sched[:len(rows)]]:
        p.append("ledger rows are not the schedule's prefix in order")
    if final.get("status") == "COMPLETE" and len(rows) != len(sched):
        p.append("COMPLETE with %d of %d scheduled episodes" % (len(rows), len(sched)))
    if final.get("kernel_faults_after") != attempt.get("kernel_faults_before"):
        p.append("kernel faults %r -> %r" % (attempt.get("kernel_faults_before"), final.get("kernel_faults_after")))
    ga, g0 = final.get("git_after") or {}, attempt.get("git") or {}
    if ga.get("head") != g0.get("head") or ga.get("dirty"):
        p.append("tree changed during the run")
    return p


def s3b_reference(s3b_attempt=SF.ATTEMPT_DIR):
    """S3b attempt01's VALID block-A outcomes, bound by SHA-256 and its reading (S3c/S4 review r1)."""
    led, rd = pathlib.Path(s3b_attempt) / "episodes.jsonl", pathlib.Path(s3b_attempt) / "reading.json"
    if not led.is_file():
        return {}, {"status": "S3B_NOT_AVAILABLE", "path": str(led)}
    rows = {r["env"]: r for r in SF.read_jsonl(led) if r["block"] == "A" and r["valid"]}
    ref = {"status": "BOUND", "path": str(led), "sha256": SF.sha256_file(led),
           "reading": json.loads(rd.read_text()).get("reading") if rd.is_file() else "NO_READING"}
    return rows, ref


def matched(ar, s3b):
    paired = [(s3b[r["env"]]["outcome"], r["outcome"]) for r in ar if r["valid"] and r["env"] in s3b]
    return {"pairs": len(paired), "same_outcome": sum(1 for a, b in paired if a == b),
            "s3b_crash": sum(1 for a, _ in paired if a == "crash"), "arm_crash": sum(1 for _, b in paired if b == "crash"),
            "s3b_capture": sum(1 for a, _ in paired if a == "capture"),
            "arm_capture": sum(1 for _, b in paired if b == "capture")}


def load_attempt(attempt_dir):
    """The attempt's records; a development smoke is refused, it is never a result (review r2 s3)."""
    attempt_dir = pathlib.Path(attempt_dir)
    load = lambda name, default: (json.loads((attempt_dir / name).read_text())          # noqa: E731
                                  if (attempt_dir / name).is_file() else default)
    if (load("attempt.json", None) or {}).get("development_smoke"):
        raise SystemExit("%s is a development smoke: it has no reading" % attempt_dir)
    return (load("attempt.json", None), load("final.json", {"status": "NO_FINAL_RECORD", "reason": "final.json missing"}),
            SF.read_jsonl(attempt_dir / "episodes.jsonl") if (attempt_dir / "episodes.jsonl").is_file() else [])


IRIS_HALF_SPAN_M = 0.235              # iris's arm reach from its centre (S3c/S4 review r1)
CLEARANCE_COL = 20                    # steps.npy: box_hit clearance (centre to the nearest bar box, in the bars' span)


def iris_footprint(attempt_dir, ar):
    """Ungated: VALID iris episodes whose centre came within iris's own half span of a bar (MOTAR's crash rule uses
    0.14 m), from the per-step clearance in steps.npy."""
    import numpy as np
    n, near, missing = 0, 0, 0
    for r in ar:
        if not r["valid"]:
            continue
        f = pathlib.Path(attempt_dir) / ("ep%03d" % r["index"]) / "steps.npy"
        if not f.is_file():
            missing += 1
            continue
        s = np.load(f)
        n += 1
        if len(s) and float(s[:, CLEARANCE_COL].min()) < IRIS_HALF_SPAN_M:
            near += 1
    return {"half_span_m": IRIS_HALF_SPAN_M, "episodes": n, "within_half_span": near, "steps_missing": missing}


def analyze(attempt_dir=ATTEMPT_DIR, s3b_attempt=SF.ATTEMPT_DIR):
    attempt, final, rows = load_attempt(attempt_dir)
    isaac = SF.isaac_outcomes(SF.G8, 70)
    s3b, ref = s3b_reference(s3b_attempt)
    out = {"protocol": PROTOCOL, "attempt_status": final["status"], "attempt_reason": final.get("reason"),
           "integrity_problems": arm_integrity_problems(attempt, final, rows), "s3b_reference": ref, "arms": {},
           "timing_and_state": SF.timing_summary([dict(r, block=r["arm"]) for r in rows])}
    for arm, _, _ in ARMS:
        ar = [r for r in rows if r["arm"] == arm]
        summ = SF.block_summary(ar, isaac) if ar else None
        if summ is not None:
            summ["vs_s3b_same_scenes"] = matched(ar, s3b)
            if arm == "iris":
                summ["iris_footprint_reported"] = iris_footprint(attempt_dir, ar)
        out["arms"][arm] = summ
    (pathlib.Path(attempt_dir) / "reading.json").write_text(json.dumps(out, indent=1, sort_keys=True, default=str) + "\n")
    print(json.dumps({"status": out["attempt_status"], "integrity": out["integrity_problems"]}), flush=True)
    return out


def main(argv=None):
    ap = argparse.ArgumentParser()
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--check", action="store_true")
    g.add_argument("--smoke", action="store_true")
    g.add_argument("--launch", action="store_true")
    g.add_argument("--analyze", nargs="?", const=str(ATTEMPT_DIR))
    a = ap.parse_args(argv)
    if a.check:
        check()
    elif a.smoke:
        launch(smoke=True)
    elif a.launch:
        launch()
    else:
        analyze(a.analyze)


if __name__ == "__main__":
    main()
