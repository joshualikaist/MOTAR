"""dev08 observed-prefix runner: integration of the runtime stack (amendment1; motar-5md).

Composes tools/sitl_owned_session.py (owned PX4/Gazebo session, fresh rootfs), tools/sitl_dev08_link.py
(parameter echo/readback, telemetry gate), tools/sitl_dev08_step.py (historical transform, prefix stopping) and
tools/sitl_dev08_attempt.py (fixed 42-member schedule, append-only ledger).

run_episode() is one member: owned session -> world (the historical writer's text) -> direct launch -> MAVLink
receiver -> heartbeat and 50 Hz streams -> settle -> parameters (booted values, airframe freshness, echo, readback)
-> the historical start phase (hold, offboard, arm, hold, three truth-offset holds, start check) -> gate alignment
-> checkpoints 0..k with a command only on CONTINUE -> the historical mode command outside the scored prefix. Any
exception makes the member operationally invalid (the attempt then stops); a valid member ends in capture, crash
or CENSORED_TRACE_END. Timings default to the historical runner's; tests shorten them.

The default CLI refuses. --execute requires a recorded scope/clean-commit approval, the fixed 0.15 s bound and
an external fresh attempt directory. It runs the unchanged two-smoke +40-main schedule in the main thread.
Live verification is reported in attempt evidence, never inferred from CLI availability.
"""
import hashlib
import json
import math
import pathlib
import threading
import time

STATUS = {"component": "dev08_prefix_runner", "live_verified": False, "execution_supported": True,
          "launch_ready": False}

ODE_ITERS = 100
SPAWN = (1.01, 0.98)              # tools/sitl_s3b_closed_loop.py SPAWN; sitl_run.sh spawn pose


def _rot(yaw, x, y):
    c, s = math.cos(yaw), math.sin(yaw)
    return c * x - s * y, s * x + c * y


def world_text(base, centers, halves, quats, p0, yaw0, ode_iters=ODE_ITERS):
    """The historical write_world text (tools/sitl_s3b_closed_loop.py) for one episode, returned instead of written
    into the PX4 tree. Bars are static boxes, transformed so that the spawn point and Gazebo yaw 0 are the episode's
    start position and yaw. The historical writer ignores bar orientation, so every bar quaternion must be the
    identity (true for the fixed twenty dev08 scenes; LAYOUT_NOT_QUALIFIED otherwise)."""
    for i, q in enumerate(quats):
        if max(abs(float(q[0])), abs(float(q[1])), abs(float(q[2])), abs(abs(float(q[3])) - 1.0)) > 1e-6:
            raise ValueError("LAYOUT_NOT_QUALIFIED: bar %d quaternion %s is not the identity" % (i, list(q)))
    if base.count("<iters>10</iters>") != 1:
        raise ValueError("empty.world: expected one <iters>10</iters>")
    base = base.replace("<iters>10</iters>", "<iters>%d</iters>" % ode_iters)
    models = []
    for i, (c, h) in enumerate(zip(centers, halves)):
        gx, gy = _rot(-yaw0, c[0] - p0[0], c[1] - p0[1])
        gx, gy = gx + SPAWN[0], gy + SPAWN[1]
        size = "%.4f %.4f %.4f" % (2 * h[0], 2 * h[1], 2 * h[2])
        models.append(
            "<model name='bar_%d'><static>true</static><pose>%.4f %.4f %.4f 0 0 %.6f</pose><link name='l'>"
            "<collision name='c'><geometry><box><size>%s</size></box></geometry></collision>"
            "<visual name='v'><geometry><box><size>%s</size></box></geometry></visual></link></model>"
            % (i, gx, gy, c[2], -yaw0, size, size))
    return base.replace("</world>", "\n".join(models) + "\n</world>", 1)


# ---------------------------------------------------------------- episode driver

TIMING = {"boot_wait_s": 3.0, "heartbeat_s": 120.0, "streams_s": 90.0, "settle_s": 10.0, "hold_pre_s": 2.0,
          "hold_mode_s": 1.0, "hold_arm_s": 10.0, "hold_at_s": 3.0, "hold_at_rounds": 3, "after_s": 1.0}
MSG_INTERVAL_US = 20000
MSG_IDS = {"LOCAL_POSITION_NED": 32, "ATTITUDE": 30, "HIL_STATE_QUATERNION": 115}
MAV_FRAME_LOCAL_NED = 1
MAV_CMD_SET_MESSAGE_INTERVAL, MAV_CMD_DO_SET_MODE, MAV_CMD_COMPONENT_ARM_DISARM = 511, 176, 400
DT, ALT = 0.1, 1.0


class HarnessInvalid(RuntimeError):
    pass


class Commander:
    """The historical Link.cmd / Link.setpoint sends."""

    def __init__(self, conn, target=(1, 1), lock=None):
        self.conn, self.target = conn, tuple(target)
        self.lock = lock or threading.Lock()
        self.sent = 0

    def cmd(self, command, *p):
        p = list(p) + [0] * (7 - len(p))
        with self.lock:
            self.conn.mav.command_long_send(self.target[0], self.target[1], command, 0, *p)

    def setpoint(self, mask, x=0.0, y=0.0, z=0.0, vx=0.0, vy=0.0, yaw_rate=0.0):
        with self.lock:
            self.conn.mav.set_position_target_local_ned_send(0, self.target[0], self.target[1], MAV_FRAME_LOCAL_NED,
                                                             mask, x, y, z, vx, vy, 0, 0, 0, 0, 0, yaw_rate)
            self.sent += 1


def latest(inbox, typ, src=(1, 1)):
    """The newest message of a type from the autopilot, unvalidated (the historical runner's L.pos/L.truth)."""
    items, _ = inbox.since(0)
    for m in reversed(items):
        if m["type"] == typ and tuple(m["src"]) == tuple(src):
            return m["fields"]
    return None


def latest_heartbeat(inbox, src=(1, 1)):
    return latest(inbox, "HEARTBEAT", src)


def _finite(*tuples):
    return all(math.isfinite(float(x)) for t in tuples for x in t)


def run_episode(member, spec, recorded_frames, arm_params, *, deps, B, policy, episode_dir, timing=None,
                gate_opts=None):
    """One scheduled member. Returns the ledger record (operational_status VALID or not, never raises for a
    member failure). deps: session(episode_dir), empty_world(), plan(session, world_path, name), binding(plan),
    launch(session, plan), connect(), fresh_defaults(), and the modules link, step."""
    T = dict(TIMING, **(timing or {}))
    LK, ST = deps["link"], deps["step"]
    H = ST.historical()
    rec = {"phase": member["phase"], "bars": member["bars"], "env": member["env"], "arm": member["arm"],
           "episode_dir": str(episode_dir), "recorded_frames": recorded_frames, "timing": T,
           "gate_opts": dict(gate_opts or {})}
    rows = []
    session = None
    try:
        with deps["session"](episode_dir) as session:
            name = "motar_dev08_%s" % member["name"]
            text = world_text(deps["empty_world"](), spec["centers"], spec["halves"], spec["quats"], spec["p0"],
                              spec["yaw0"])
            world = session.write_world(name, text)
            plan = deps["plan"](session, world, name)
            rec["runtime_binding"] = deps["binding"](plan)
            deps["launch"](session, plan)
            session.sleep(T["boot_wait_s"])
            conn = deps["connect"]()
            inbox = LK.Inbox()
            transport = LK.PymavlinkTransport(conn, inbox)
            rx = threading.Thread(target=transport.rx_loop, name="dev08-mavlink-rx", daemon=True)
            rx.start()
            try:
                _fly(member, spec, recorded_frames, arm_params, T, session, conn, inbox, transport, deps, B, policy,
                     rows, LK, ST, H, out=rec, gate_opts=dict(gate_opts or {}))   # fills rec as it goes
            finally:
                transport.stop.set()
                rx.join(timeout=5.0)
                rec["rx_thread_alive_after_join"] = rx.is_alive()
                getattr(conn, "close", lambda: None)()
            if rec["rx_thread_alive_after_join"]:
                raise RuntimeError("RX_THREAD_STUCK: the MAVLink receiver did not stop")
    except Exception as e:                                        # noqa: BLE001 - every failure is recorded
        rec["operational_status"] = "HARNESS_INVALID" if isinstance(e, HarnessInvalid) else "ERROR"
        rec["error"] = "%s: %s" % (e.__class__.__name__, e)
        rec["error_code"] = getattr(e, "code", None)
    finally:                                                      # also when a termination request passes through
        if session is not None and session.episode_dir is not None:
            receipt = session.episode_dir / "session.json"
            rec["session_receipt"] = ({"path": str(receipt), "sha256": hashlib.sha256(receipt.read_bytes()).hexdigest()}
                                      if receipt.is_file() else None)
            with open(session.episode_dir / "checkpoints.jsonl", "x") as fh:
                for r in rows:
                    fh.write(json.dumps(r, default=float) + "\n")
    return rec


def _fly(member, spec, recorded_frames, arm_params, T, session, conn, inbox, transport, deps, B, policy, rows,
         LK, ST, H, out, gate_opts):
    cmdr = Commander(conn, lock=transport.lock)                   # one lock for every send on conn
    hb = inbox.wait_for(lambda m: m["type"] == "HEARTBEAT" and tuple(m["src"]) == (1, 1), 0, T["heartbeat_s"])
    if hb is None:
        raise HarnessInvalid("HARNESS_INVALID no heartbeat")
    for mid in MSG_IDS.values():
        cmdr.cmd(MAV_CMD_SET_MESSAGE_INTERVAL, mid, MSG_INTERVAL_US)
    end = time.monotonic() + T["streams_s"]
    while not all(latest(inbox, typ) is not None for typ in MSG_IDS):
        if time.monotonic() > end:
            raise HarnessInvalid("HARNESS_INVALID telemetry streams missing")
        session.sleep(0.2)
    session.sleep(T["settle_s"])                                  # estimator settles (S1/S2: about 10 s)
    out["parameters"] = LK.apply_and_verify(inbox, transport, arm_params, fresh_defaults=deps["fresh_defaults"]())
    pn = ST.pos_tuple(latest(inbox, "LOCAL_POSITION_NED"))
    tr0 = ST.truth_tuple(latest(inbox, "HIL_STATE_QUATERNION"))
    if not _finite(pn, tr0):
        raise HarnessInvalid("HARNESS_INVALID non-finite state before takeoff")
    x0n, y0e = pn[1], pn[2]
    origin = (tr0[1], tr0[2], tr0[3])                              # truth origin: the vehicle on the ground at spawn

    def truth_local():
        tr = ST.truth_tuple(latest(inbox, "HIL_STATE_QUATERNION"))
        if not _finite(tr):
            raise HarnessInvalid("HARNESS_INVALID non-finite truth during the start phase")
        dn = math.radians(tr[1] - origin[0]) * ST.R_E
        de = math.radians(tr[2] - origin[1]) * ST.R_E * math.cos(math.radians(origin[0]))
        return de, dn, tr[3] - origin[2]

    def hold(seconds, n_ned, e_ned):
        for _ in range(int(round(seconds / DT))):
            cmdr.setpoint(H.TM_POS_XYZ, n_ned, e_ned, -ALT)
            session.sleep(DT)
            session.check(required=("gzserver", "px4"))

    hold(T["hold_pre_s"], x0n, y0e)
    cmdr.cmd(MAV_CMD_DO_SET_MODE, 1, 6)
    hold(T["hold_mode_s"], x0n, y0e)
    cmdr.cmd(MAV_CMD_COMPONENT_ARM_DISARM, 1)
    hold(T["hold_arm_s"], x0n, y0e)
    for _ in range(T["hold_at_rounds"]):                           # put the TRUE vehicle at the spawn point
        tx, ty, _ = truth_local()
        pos = ST.pos_tuple(latest(inbox, "LOCAL_POSITION_NED"))
        if not _finite(pos):
            raise HarnessInvalid("HARNESS_INVALID non-finite position during the start phase")
        hold(T["hold_at_s"], pos[1] - ty, pos[2] - tx)
    hb = latest_heartbeat(inbox) or {}
    armed = bool(hb.get("base_mode", 0) & 128)
    offboard = ((hb.get("custom_mode", 0) >> 16) & 0xFF) == 6
    tx, ty, tz = truth_local()
    pos = ST.pos_tuple(latest(inbox, "LOCAL_POSITION_NED"))
    if not _finite(pos):
        raise HarnessInvalid("HARNESS_INVALID non-finite position at the start check")
    out["start_check"] = {"armed": armed, "offboard": offboard, "truth_alt_m": tz, "ekf_alt_m": -pos[3],
                          "ekf_minus_truth_xy_m": math.hypot(pos[2] - tx, pos[1] - ty),
                          "truth_from_spawn_xy_m": math.hypot(tx, ty), "command_yaw": "truth"}
    if not (armed and offboard and abs(-pos[3] - ALT) < 0.3 and 0.3 < tz < 2.5 and math.hypot(tx, ty) < 0.25):
        raise HarnessInvalid("HARNESS_INVALID start state %s" % out["start_check"])
    # The bridge episode is built before the start is fixed, as in the historical runner (its construction takes
    # tens of milliseconds and must not delay checkpoint 0).
    ep = ST.PrefixEpisode(B, policy, spec, recorded_frames, origin)
    # the gate judges the stream from one second before alignment on (boot and takeoff are not scored)
    gate = LK.TelemetryGate(inbox, start_cursor=inbox.cursor_since(inbox.now() - 1.0), **gate_opts)
    out["alignment"] = gate.align()
    wall0 = time.monotonic()
    for k in range(recorded_frames):
        session.check(required=("gzserver", "px4"))
        snap = gate.checkpoint(k)
        compute0 = time.monotonic()
        r = ep.step(k, snap)
        r.update(pos_seq=snap["pos"]["seq"], att_seq=snap["att"]["seq"], truth_seq=snap["truth"]["seq"],
                 pos_lateness_s=snap["pos_lateness_s"], att_skew_s=snap["att_skew_s"],
                 truth_recv_skew_s=snap["truth_recv_skew_s"], wall_s=time.monotonic() - wall0)
        rows.append(r)
        if r["command"] is None:
            r["compute_ms"] = 1000.0 * (time.monotonic() - compute0)
            break
        cmdr.setpoint(*r["command"])
        r["compute_ms"] = 1000.0 * (time.monotonic() - compute0)
    c = ep.counts()
    if c["outcome"] is None:
        raise HarnessInvalid("HARNESS_INVALID prefix ended without a decision")
    k = c["final_frame_index"]
    wall = time.monotonic() - wall0
    out.update(operational_status="VALID", outcome=c["outcome"], final_frame_index=k,
               observation_checks=c["observation_checks"], commands_issued=c["commands_issued"],
               start_sim_time_s=gate.start, final_sim_time_s=rows[-1]["sim_s"],
               scheduled_elapsed_s=round(k * DT, 10), min_dist_m=c["min_dist_m"],
               crash_reason=rows[-1].get("crash_reason"),
               compute_ms_max=max(r["compute_ms"] for r in rows),
               real_time_factor=(rows[-1]["sim_s"] - gate.start) / wall if wall > 0 else None,
               ignored_foreign=dict(gate.ignored))
    cmdr.cmd(MAV_CMD_DO_SET_MODE, 1, 4, 6)                        # outside the scored prefix, as the historical runner
    session.sleep(T["after_s"])
    return out


def warm_up(B, policy, spec):
    """One observe and two policy calls on a scratch episode, so first-call costs (Warp kernels, torch) are paid
    before any member's clock starts. Policy F's deterministic forward has no state (rnn_states None)."""
    import numpy as np
    ep = B.Episode(spec["centers"], spec["halves"], spec["quats"])
    p = np.asarray(spec["p0"], float)
    yaw = float(spec["yaw0"])
    q = np.array([0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)])
    s = {"pos": p, "vel": np.zeros(3), "vquat": q, "fquat": q, "angvel": np.zeros(3),
         "tpos": np.asarray(spec["tpos"][0], float), "tquat": None, "prev_action": np.zeros(4, np.float32),
         "bounds_min": np.asarray(spec["bounds_min"], float), "bounds_max": np.asarray(spec["bounds_max"], float)}
    t0 = time.monotonic()
    obs, _, lidar = ep.observe(s)
    act = policy.act(obs)[0].numpy()
    ep.command(act, lidar)
    policy.act(obs)
    return {"warm_up_s": time.monotonic() - t0}


def historical_masks():
    """(TM_POS_XYZ, TM_POS_Z_VEL_XY_YAWRATE) of the historical runner."""
    import importlib.util
    spec = importlib.util.spec_from_file_location("dev08_masks_step", str(pathlib.Path(__file__).with_name(
        "sitl_dev08_step.py")))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    H = mod.historical()
    return H.TM_POS_XYZ, H.TM_POS_Z_VEL_XY_YAWRATE


# ---------------------------------------------------------------- the unit (not run in this repository unit)

TOOLS = pathlib.Path(__file__).resolve().parent
ROOT = TOOLS.parent
PYLINK = "/home/fair/motar_tmp/sitl_plan_20261104/pylink"       # pymavlink of the px4 environment (historical)
APPROVAL_KEYS = ("approved_by", "approved_at", "scope", "commit", "skip_max_s")
SCOPE = "dev08 observed-prefix-v1: 2 native smoke + 40 main episodes (amendment1 schedule)"


def _module(name, rel):
    import importlib.util
    import sys
    saved = list(sys.path)
    try:
        spec = importlib.util.spec_from_file_location(name, str(ROOT / rel))
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
    finally:
        sys.path[:] = saved
    return mod


def scene_specs(root=ROOT):
    """The twenty fixed scenes from the verified inputs: observed-prefix input verification (34 pinned files), then
    the historical trace reader for env0..9/episode0 with the zero-action target. Each target trajectory must have
    exactly the admitted prefix length (208 for 70 bars env5, else 600)."""
    P = _module("dev08_prefix_contract_run", "tools/sitl_dev08_prefix_contract.py")
    H = _module("dev08_runner_specs", "tools/sitl_s3b_closed_loop.py")
    prep = P.prepare(root, protocol="observed-prefix-v1")
    windows = {(w["bars"], w["env"]): w for w in prep["windows"]}
    inventory = json.loads((pathlib.Path(root) / P.INVENTORY).read_text())
    specs = {}
    for bars in (70, 205):
        full = pathlib.Path(inventory["cells"][str(bars)]["full"]["directory"])
        zero = pathlib.Path(inventory["cells"][str(bars)]["zero"]["directory"])
        for spec in H.episodes_from_trace(full, 10, 0, pathlib.Path(root), zero):
            frames = windows[(bars, spec["env"])]["recorded_frames"]
            if spec["target_source"] != "zero_action_cell" or len(spec["tpos"]) != frames:
                raise ValueError("scene %d/env%d target differs from the verified window" % (bars, spec["env"]))
            specs[(bars, spec["env"])] = spec
    if set(specs) != set(windows):
        raise ValueError("scene set differs from the fixed twenty")
    return specs, windows, prep


def real_deps():
    import os
    import sys
    OS = _module("dev08_owned_session_run", "tools/sitl_owned_session.py")
    LK = _module("dev08_link_run", "tools/sitl_dev08_link.py")
    ST = _module("dev08_step_run", "tools/sitl_dev08_step.py")
    empty = OS.PX4_ROOT / "Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/empty.world"
    airframe = OS.BUILD / "etc/init.d-posix/airframes" / OS.AIRFRAME
    base_env = dict(os.environ, CUDA_VISIBLE_DEVICES="", PYTHONNOUSERSITE="1")

    def plan(session, world, name):
        pl = OS.direct_launch_plan(session.episode_dir, name, base_env)
        if pathlib.Path(pl["world_path"]) != pathlib.Path(world):
            raise RuntimeError("plan world differs from the written world")
        return pl

    def binding(pl):
        b = OS.runtime_binding(pl)
        b["files"]["empty_world_template"] = {"path": str(empty), "sha256": OS.sha256_file(empty)}
        return b

    def connect():
        if PYLINK not in sys.path:
            sys.path.append(PYLINK)
        from pymavlink import mavutil
        return mavutil.mavlink_connection("udpin:0.0.0.0:14540", source_system=245)

    return {"session": lambda d: OS.OwnedSession(d), "empty_world": empty.read_text, "plan": plan,
            "binding": binding, "launch": OS.launch_px4_gazebo, "connect": connect,
            "fresh_defaults": lambda: LK.fresh_expectations(airframe.read_text(),
                                                            (OS.BUILD / "parameters.json").read_text()),
            "link": LK, "step": ST}


class RunnerTerminated(BaseException):
    """SIGTERM/SIGHUP to the runner: not an Exception, so a member wrapper never turns it into a member outcome."""


def _binding_core(binding):
    """The runtime binding without the per-episode world: must be identical for every member."""
    files = dict((binding or {}).get("files", {}))
    files.pop("world", None)
    return {k: v["sha256"] for k, v in files.items()}


def package_manifest(name):
    """Version, location and a SHA-256 over the package's .py files (pymavlink, rl_games: code outside this tree)."""
    import sys
    mod = sys.modules.get(name)
    if mod is None:
        return {"status": "not imported"}
    root = pathlib.Path(mod.__file__).resolve().parent
    h = hashlib.sha256()
    for f in sorted(root.rglob("*.py")):
        h.update(str(f.relative_to(root)).encode() + b"\0" + f.read_bytes())
    return {"path": str(root), "version": getattr(mod, "__version__", None), "py_files_sha256": h.hexdigest()}


def smoke_problems(record):
    """Amendment5's pre-main live timing checks; telemetry/parameters/cleanup retain their existing guards."""
    problems = []
    rtf = record.get("real_time_factor")
    if not isinstance(rtf, (int, float)) or isinstance(rtf, bool) or not math.isfinite(rtf) or not 0.95 <= rtf <= 1.05:
        problems.append("smoke real_time_factor must be in [0.95, 1.05]")
    compute = record.get("compute_ms_max")
    if not isinstance(compute, (int, float)) or isinstance(compute, bool) or not math.isfinite(compute) or not 0 <= compute < 100:
        problems.append("smoke compute_ms_max must be in [0, 100)")
    return problems


def execute(attempt_dir, approval, *, deps=None, specs=None, B=None, policy=None, run=None, root=ROOT,
            make_recorder=None, git_state=None, environment=None):
    """The fixed 42-member unit under a recorded approval. Not called by main in this repository unit.
    make_recorder / git_state / run / deps / specs / B / environment are injection points for the CPU fixtures."""
    import signal
    import sys
    AT = _module("dev08_attempt_run", "tools/sitl_dev08_attempt.py")
    P = _module("dev08_prefix_contract_exec", "tools/sitl_dev08_prefix_contract.py")
    if not isinstance(approval, dict) or any(approval.get(k) in (None, "") for k in APPROVAL_KEYS):
        raise PermissionError("no recorded approval (%s)" % ", ".join(APPROVAL_KEYS))
    if approval["scope"] != SCOPE:
        raise PermissionError("approval scope differs: %r" % approval["scope"])
    skip = approval["skip_max_s"]
    if type(skip) is not float or not 0.02 <= skip <= 0.5:
        raise PermissionError("approval skip_max_s must be a float in [0.02, 0.5], got %r" % (skip,))
    git_state = git_state or AT.git_state
    git0 = git_state(root)
    if approval["commit"] != git0["head"] or git0["dirty"]:
        raise PermissionError("approval names %s; tree is %s%s" % (approval["commit"], git0["head"],
                                                                   " (dirty)" if git0["dirty"] else ""))
    env_problems = AT.environment_problems(*(environment or (AT.process_environment(), AT.CORES)))
    if env_problems:                                              # before anything touches Warp, torch or CUDA
        raise PermissionError("process environment: %s" % env_problems)

    def on_signal(signum, frame):
        raise RunnerTerminated("runner received signal %d" % signum)

    saved = {}
    if threading.current_thread() is threading.main_thread():     # before anything long; restored on every path
        for sig in (signal.SIGTERM, signal.SIGHUP):
            saved[sig] = signal.signal(sig, on_signal)
    rec = None
    try:
        cfg = _module("dev08_cfg_exec", "tools/sitl_s3b_dev08.py")
        params = {arm: cfg.prepare(root, arm=arm)["selected_params"] for arm in ("native", "candidate")}
        prep_summary = None
        if specs is None:
            specs, windows, prep = scene_specs(root)
            prep_summary = {k: prep[k] for k in ("status", "verified_inventory_files", "inventory_sha256")}
        else:
            specs, windows = specs
        if B is None:
            sys.path.insert(0, str(TOOLS))
            import sitl_bridge as B                               # noqa: N811
            B.setup(specs[(70, 0)]["receipt_env"])                 # one setup for both densities (review r1 n2)
            resolved = pathlib.Path(list(sys.modules["aerial_gym"].__path__)[0]).resolve()
            if not str(resolved).startswith(str(pathlib.Path(root).resolve()) + "/"):
                raise PermissionError("aerial_gym resolves outside this tree: %s" % resolved)
            policy = B.Policy()
            import torch
            torch.set_num_threads(1)                              # amendment4: one compute thread
            if torch.get_num_threads() != 1:
                raise PermissionError("torch uses %d threads; amendment4 requires 1" % torch.get_num_threads())
            if PYLINK not in sys.path:
                sys.path.append(PYLINK)
            import pymavlink  # noqa: F401
        warm = warm_up(B, policy, specs[(70, 0)])
        extra = {"inputs": prep_summary, "warm_up": warm, "skip_max_s": skip,
                 "packages": {n: package_manifest(n) for n in ("pymavlink", "rl_games")},
                 "bridge_setup": "one setup with the 70-bar receipt's NAVRL_* snapshot for both densities"}
        rec = (make_recorder or (lambda d, scope: AT.AttemptRecorder(d, scope=scope, root=root,
                                                                     f_checkpoint=pathlib.Path(B.F_CKPT),
                                                                     f_sha=P.F_SHA, extra_header=extra)))(
            pathlib.Path(attempt_dir), {"approval": dict(approval), "scope": SCOPE})
        rec.open()
        deps = deps or real_deps()
        run = run or run_episode
        binding_ref = None
        current = None
        for member in AT.schedule():
            rec.check_budget()
            host = rec.check_host()
            if git_state(root) != git0:
                rec.abort("SOURCE_CHANGED")
                break
            key = (member["bars"], member["env"])
            current = member
            r = run(member, specs[key], windows[key]["recorded_frames"], params[member["arm"]], deps=deps, B=B,
                    policy=policy, episode_dir=pathlib.Path(attempt_dir) / member["name"],
                    gate_opts={"skip_max_s": skip})
            after_ok, after = rec.host_check()                    # recorded, not a stop: the member has ended
            r = dict(r, host_idle_before=host, host_idle_after=dict(after, ok=after_ok))
            core = _binding_core(r.get("runtime_binding"))
            if r.get("operational_status") == "VALID" and core:
                binding_ref = binding_ref or core
                if core != binding_ref:
                    r = dict(r, operational_status="BINDING_CHANGED")
            if git_state(root) != git0:                              # a change during this member, the last too
                r = dict(r, operational_status="SOURCE_CHANGED")
            if member["phase"] == "smoke" and r.get("operational_status") == "VALID":
                problems = smoke_problems(r)
                r = dict(r, smoke_timing_problems=problems)
                if problems:
                    r = dict(r, operational_status="HARNESS_INVALID", error="; ".join(problems))
            rec.append(r)
            current = None
            if rec.status is not None:
                break
        rec.finish()
    except BaseException as e:                                    # noqa: BLE001 - recorded, then re-raised
        if rec is not None and rec.t0 is not None:
            cur = locals().get("current")
            rec.abort("RUNNER_EXCEPTION", error="%s: %s" % (e.__class__.__name__, e),   # ran, not recorded: in unrun
                      interrupted_member=cur["name"] if cur is not None and rec.next == cur["index"] else None)
        raise
    finally:
        for sig, h in saved.items():
            signal.signal(sig, h)
    return rec.status


def main(argv=None):
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true", help="run the recorded two-smoke +40-main approval")
    parser.add_argument("--approval-file", type=pathlib.Path)
    parser.add_argument("--attempt-dir", type=pathlib.Path)
    args = parser.parse_args(argv)
    if not args.execute:
        print(json.dumps(dict(STATUS, refused="--execute and a scoped approval are required", scope=SCOPE)))
        return 2
    if args.approval_file is None or args.attempt_dir is None:
        parser.error("--execute requires --approval-file and --attempt-dir")
    try:
        if threading.current_thread() is not threading.main_thread():
            raise PermissionError("execute must run in the main thread")
        raw = args.approval_file.read_bytes()
        approval = json.loads(raw)
        if not isinstance(approval, dict) or type(approval.get("skip_max_s")) is not float or approval["skip_max_s"] != 0.15:
            raise PermissionError("D-2026-11-05-2 fixes skip_max_s at 0.15 s")
        approval = dict(approval, approval_file={"path": str(args.approval_file.resolve()),
                                               "sha256": hashlib.sha256(raw).hexdigest()})
        status = execute(args.attempt_dir, approval)
    except (Exception, RunnerTerminated, KeyboardInterrupt) as e:
        print(json.dumps({"status": "REFUSED_OR_ABORTED", "error": "%s: %s" % (type(e).__name__, e),
                          "attempt_dir": str(args.attempt_dir)}))
        return 1
    print(json.dumps({"status": status, "attempt_dir": str(args.attempt_dir), "scope": SCOPE}))
    return 0 if status == "COMPLETE" else 1


if __name__ == "__main__":
    raise SystemExit(main())
