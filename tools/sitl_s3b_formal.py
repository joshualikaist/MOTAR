"""S3b formal v1: Frozen Policy F closing the loop through PX4 v1.14.3 SITL, on the recorded G8 r2 scenes.

Preregistration: docs/plans/prereg_sitl_s3b_formal_v1_2026-11-06.md (plan stage S3b, docs/plans/sitl_validation_plan_v1.md).

The episode loop is the development runner's (tools/sitl_s3b_closed_loop.py, dev07: 20/20 outcomes equal to Isaac),
unchanged in what it computes. This file adds what a formal run needs and the development runner lacked:
  - an owned PX4 + Gazebo session per episode with a fresh rootfs (tools/sitl_owned_session.py), so no parameter
    persists between episodes or from earlier runs;
  - a read-back of every PX4 parameter the runner sets;
  - the lateness of every control step against its 0.1 s schedule (validity bound 0.15 s, D-2026-11-05-2);
  - a fixed scene schedule from pinned G8 r2 inputs, an approval bound to the commit, kernel-fault checks;
  - a fixed analyzer (--analyze) that writes the preregistered reading.

Usage (px4 conda Python is not needed; the aerialgym Python runs the bridge):
  tools/sitl_s3b_formal.py --check                 # inputs, schedule, environment; starts nothing
  tools/sitl_s3b_formal.py --launch                # needs UNIT_DIR/execution_approval.json; single use
  tools/sitl_s3b_formal.py --analyze [ATTEMPT_DIR] # writes ATTEMPT_DIR/reading.json
"""
import argparse
import collections
import hashlib
import json
import math
import os
import pathlib
import signal
import struct
import sys
import threading
import time

TOOLS = pathlib.Path(__file__).resolve().parent
ROOT = TOOLS.parent
sys.path.insert(0, str(TOOLS))

PROTOCOL = "s3b-formal-v1"
PREREG = "docs/plans/prereg_sitl_s3b_formal_v1_2026-11-06.md"
SCOPE = ("S3b formal v1: Frozen Policy F through PX4 v1.14.3 SITL on navrl_quad, G8 r2 scenes seed 4501; "
         "block A 70 bars all eligible envs, block B 205 bars first 30 eligible envs, block C 70 bars first 10 "
         "eligible envs repeated; episode lateness bound 0.15 s; one attempt, no retries")
APPROVAL_KEYS = ("approved_by", "approved_at", "scope", "commit")
UNIT_DIR = pathlib.Path.home() / "motar_tmp" / "s3b_formal_20261106"
ATTEMPT_DIR = UNIT_DIR / "attempt01"
G8 = pathlib.Path("/home/fair/workspaces/aerial_gym_ws/src/aerial_gym_simulator/results/sitl_ref_v1_r2/g8")
CELL = "E2_obstacle_aware__%dbars__seed4501"
TRACE_FILES = ("episodes.jsonl", "kroc2027_episodes.jsonl", "receipt.json", "occlusion_frames.bin",
               "occlusion_layouts.npz", "occlusion_trace_summary.json")
INPUT_SHA256 = {
    "full/70/episodes.jsonl": "36c419189a591c1a829e0419db94eda4382c24db75a5638a81e67b07be0c652f",
    "full/70/kroc2027_episodes.jsonl": "2a25857bef40c27001a761995b3ebcdd3e9a18eaabd586b17d6f4a9f64b47220",
    "full/70/receipt.json": "9de38dae0cceb1cf1eef64c51bba1aa0682e3bfb712caa566d358f12e9d6123b",
    "full/70/occlusion_frames.bin": "74504f9e93519775955fe9c888c4dd8b5658c43cb5660ac7fa7325d8b7eb833b",
    "full/70/occlusion_layouts.npz": "df8de9f05a8c5b85a6d5743e9c34258e9726f035a23ff4c4b585122b90442ea4",
    "full/70/occlusion_trace_summary.json": "06cc2840cda6c74202bdda4ee214f7b14787b5322ae0aa1e6fc14e6b9a59245c",
    "full/205/episodes.jsonl": "5877c1b15898e2fb89989672b5f83382730903f0ac9880796f0f41b96d84310c",
    "full/205/kroc2027_episodes.jsonl": "34c616cc00dae11d8cad492b2a4a0f7ab5adab1b973d4f89acd5293ab45985d2",
    "full/205/receipt.json": "d2e977b419102c4c2d3d858495ce7be79959046637aaf3f91dd7c0bd14aee6c8",
    "full/205/occlusion_frames.bin": "8ca095c2447c83839498faf00e5a1cf20e02bf4ac57a21fd69c7b72fb4fc035b",
    "full/205/occlusion_layouts.npz": "a07792b84a3b426bebf9a8ddd5dbd7efa10851e5b0715353d9139fbe0f95b6a4",
    "full/205/occlusion_trace_summary.json": "735ae1b523874a60a8eccf51a058326862d3446207269e9fa61f5fb662c465e1",
    "zero/70/episodes.jsonl": "091c5bd5d34ec39f5de127308cee26700a83271ea9a7ceafdca5a1eb59b0f3ae",
    "zero/70/kroc2027_episodes.jsonl": "737b9ba3e49247053afa2f63693c45e03027f787541cdf5027137b46e59bb747",
    "zero/70/receipt.json": "7b3c8504582ab92066e2a3f89ddd57af11b63b4bc1ac8ae3dbd6cb6244322a44",
    "zero/70/occlusion_frames.bin": "093ab91ce8ed5c626af5b23e1e1b5518e4315a925356282d1b8d60c060e81fe3",
    "zero/70/occlusion_layouts.npz": "0b137bb0b26eeedd6caa98aa8dca4f19d12d353e986418565a9baaa5564d44b9",
    "zero/70/occlusion_trace_summary.json": "7628257a2c3159fdd3b51831625a27528f97fbecb97f6ea3a562e7225f8e53a2",
    "zero/205/episodes.jsonl": "0b6f1bcc70383c9738a09509ae18251f66ad6335f0dcf6436233320b8231037f",
    "zero/205/kroc2027_episodes.jsonl": "3aee0b8f8e37cfbcb0fd97807a2d4764bb0451e2729e9f27bf2ab77234909b58",
    "zero/205/receipt.json": "41c8cd4b14204a26b68bd61988612523c63e7bbc9b0534114fad085a622ad9e5",
    "zero/205/occlusion_frames.bin": "00a89a0ec93213791c2b822bd75fcf6569ec94635b1104df9eb27f005015f222",
    "zero/205/occlusion_layouts.npz": "6379cdffc80d9f309b9df2ea3e2f64e80c3d62d616c381c62a536e86bb067b92",
    "zero/205/occlusion_trace_summary.json": "286d92e7738b246231d07d4c65f15e4082c2143b0ad672837eeb0e62c28695d3",
}
FULL_HORIZON = 600                      # a scene is eligible when its zero-action target ran the whole 600 actions
BLOCKS = (("A", 70, None, False), ("B", 205, 30, False), ("C", 70, 10, True))   # (block, bars, first n, repeat)
LATE_MAX_S = 0.15                       # D-2026-11-05-2 (the user's dev08 bound), applied to every control step
PARAM_TOL = 1e-6
INVALID_ABORT_A = 12                    # more than 10 % of block A invalid: stop, the reading would be VOID
VALID_MIN_A = 110                       # at least 90 % of the 122 block-A scenes must be VALID
MARGIN_PP = 5.0                         # plan S3b: feasible if SITL crash <= Isaac crash + 5 pp at 70 bars
CONDITION = {"model": "navrl_quad", "delay_steps": 0}
STEP_WALL_MAX_S = 2.0                   # PX4 simulation time must reach each step within this wall time (review r1 M1)
CHECK_EVERY_S = 0.1                     # the owned session's process/watchdog check while waiting for a step
F_SHA256 = "f702213936601860995cf61dcc570247e72543b1976e3716055cd8ec5593ad40"   # Frozen Policy F (ep25000 riskcap)
OWN_SOURCES = ("tools/sitl_s3b_formal.py", "tests/test_sitl_s3b_formal.py", "tools/sitl_s3b_closed_loop.py",
               "tools/sitl_owned_session.py", "tools/sitl_dev08_attempt.py", "tools/sitl_bridge.py",
               "motar_eval/occlusion_trace.py", PREREG)
EXTERNAL_SOURCES = (("/home/fair/workspaces/aerial_gym_ws/src/MOTAR-cplanner", "cplanner/motar/replay.py"),)
AIRFRAMES = {"navrl_quad": "10090_gazebo-classic_navrl_quad", "navrl_iris": "10091_gazebo-classic_navrl_iris",
             "navrl_quad_s4": "10092_gazebo-classic_navrl_quad_s4"}
CONSECUTIVE_INVALID_STOP = 3           # a harness failing repeatedly (review r2 LOW): stop the unit
HOST_BUSY_WAIT_S = 30.0                # one wait before an episode whose cores are not idle is recorded HOST_BUSY
ENV_REFUSE_PREFIXES = ("NAVRL_", "PX4_", "S3B_", "GAZEBO_")     # review r2 MINOR-3: no inherited treatment changes
MAV_PARAM_TYPE_INT32 = 6


class Terminated(BaseException):
    """SIGTERM (the outer timeout, a kill) outside an owned session: ends the unit through its recorded path."""


def _on_sigterm(signum, frame):
    raise Terminated("signal %d" % signum)


def sha256_file(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def input_problems(g8=G8, pins=INPUT_SHA256):
    p = []
    for key, want in sorted(pins.items()):
        kind, bars, name = key.split("/")
        path = g8 / kind / (CELL % int(bars)) / name
        if not path.is_file():
            p.append("missing input %s" % path)
        elif sha256_file(path) != want:
            p.append("input SHA-256 differs: %s" % path)
    return p


def eligible_envs(zero_rows):
    """Envs whose first zero-action episode ran all 600 actions (the target trajectory covers a full SITL episode).
    Fixed from the zero-action ledger alone, before any SITL outcome exists."""
    return sorted(r["env_index"] for r in zero_rows if r["env_episode_index"] == 0 and r["actions"] >= FULL_HORIZON)


def read_jsonl(path):
    with open(path) as f:
        return [json.loads(line) for line in f if line.strip()]


def schedule(g8=G8, blocks=BLOCKS):
    """The fixed episode list: [(block, bars, env, repeat)], blocks in order, envs ascending."""
    out = []
    for block, bars, first_n, repeat in blocks:
        envs = eligible_envs(read_jsonl(g8 / "zero" / (CELL % bars) / "kroc2027_episodes.jsonl"))
        if first_n is not None:
            envs = envs[:first_n]
        out.extend({"block": block, "bars": bars, "env": e, "repeat": repeat} for e in envs)
    return out


def isaac_outcomes(g8=G8, bars=70):
    rows = read_jsonl(g8 / "full" / (CELL % bars) / "episodes.jsonl")
    return {r["env_index"]: r["outcome"] for r in rows if r["env_episode_index"] == 0}


def approval_problems(approval, head, scope=SCOPE):
    p = []
    if not isinstance(approval, dict):
        return ["approval is not a JSON object"]
    for k in APPROVAL_KEYS:
        if not isinstance(approval.get(k), str) or not approval[k].strip():
            p.append("approval key %s missing or empty" % k)
    if approval.get("scope") != scope:
        p.append("approval scope differs from the code's SCOPE")
    if approval.get("commit") not in (head, head[:7]) or not head:
        p.append("approval commit %r is not HEAD %s" % (approval.get("commit"), head))
    return p


def world_text(base, centers, halves, p0, yaw0, ode_iters, spawn, rot):
    """The development runner's world (empty.world, ODE iterations raised, the episode's bars as static boxes in the
    spawn frame), returned as text instead of written into the PX4 tree."""
    if base.count("<iters>10</iters>") != 1:
        raise ValueError("empty.world: expected one <iters>10</iters>")
    base = base.replace("<iters>10</iters>", "<iters>%d</iters>" % ode_iters)
    models = []
    for i, (c, h) in enumerate(zip(centers, halves)):
        gx, gy = rot(-yaw0, c[0] - p0[0], c[1] - p0[1])
        gx, gy = gx + spawn[0], gy + spawn[1]
        size = "%.4f %.4f %.4f" % (2 * h[0], 2 * h[1], 2 * h[2])
        models.append(
            "<model name='bar_%d'><static>true</static><pose>%.4f %.4f %.4f 0 0 %.6f</pose><link name='l'>"
            "<collision name='c'><geometry><box><size>%s</size></box></geometry></collision>"
            "<visual name='v'><geometry><box><size>%s</size></box></geometry></visual></link></model>"
            % (i, gx, gy, c[2], -yaw0, size, size))
    return base.replace("</world>", "\n".join(models) + "\n</world>", 1)


def params_problems(readback, wanted, tol=PARAM_TOL):
    p = []
    for k, v in wanted.items():
        got = readback.get(k)
        if got is None:
            p.append("param %s not read back" % k)
        elif abs(float(got) - float(v)) > tol * max(1.0, abs(float(v))):
            p.append("param %s read back %r, set %r" % (k, got, v))
    return p


class CommandDelay:
    """Holds the governed command for `steps` control steps (S3c's injected delay). steps=0 passes every command
    through unchanged, in the same step, which is the development runner's behaviour."""

    def __init__(self, steps):
        self.steps = int(steps)
        self.q = collections.deque()

    def __call__(self, cmd):
        if self.steps == 0:
            return cmd
        self.q.append(cmd)
        if len(self.q) <= self.steps:
            return (0.0, 0.0, 0.0)                     # nothing commanded yet: hover in place
        return self.q.popleft()


def binding_core(binding):
    """The runtime binding without the per-episode world, as dev08: must not change within a condition."""
    files = dict((binding or {}).get("files", {}))
    files.pop("world", None)
    return {k: v["sha256"] for k, v in files.items()}


def env_problems(environ=None):
    environ = os.environ if environ is None else environ
    bad = sorted(k for k in environ if k.startswith(ENV_REFUSE_PREFIXES))
    return ["inherited environment variable %s would reach the bridge or PX4" % k for k in bad]


def episode_valid(res, late_max_s=LATE_MAX_S):
    if res.get("outcome") not in ("capture", "crash", "timeout"):
        return False, res.get("outcome") or "no outcome"
    if res.get("param_problems"):
        return False, "PARAM_MISMATCH"
    if res.get("late_max_s") is None or res["late_max_s"] > late_max_s:
        return False, "LATE"
    return True, None


# ----------------------------------------------------------------------------------------------- live episode


def wait_for_step(L, session, target_s, wall_max_s=None, check_every_s=CHECK_EVERY_S):
    """Wait until PX4's simulation time reaches target_s. Raises when it does not within wall_max_s (PX4 or Gazebo
    stalled or died), and checks the owned session's processes and watchdog meanwhile (review r1 M1)."""
    wall_max_s = STEP_WALL_MAX_S if wall_max_s is None else wall_max_s
    t0 = time.time()
    next_check = t0
    while L.pos[0] < target_s:
        now = time.time()
        if now - t0 > wall_max_s:
            raise RuntimeError("PX4_TIME_STALLED: sim time %.3f s did not reach %.3f s in %.1f s"
                               % (L.pos[0], target_s, wall_max_s))
        if now >= next_check:
            session.check(required=("gzserver", "px4"))
            next_check = now + check_every_s
        time.sleep(0.002)


def run_episode(B, OS, V0, spec, ep_dir, policy, condition=CONDITION, link_factory=None):
    """One episode in an owned session. The steps between take-off and termination are the development runner's.
    The MAVLink receiver is stopped, joined and its socket closed before the session ends (review r1 B1)."""
    import numpy as np
    link_factory = link_factory or ParamLink
    centers, halves = spec["centers"], spec["halves"]
    p0, yaw0, ttraj = spec["p0"], spec["yaw0"], spec["tpos"]
    res = {"env": spec["env"], "isaac_outcome": spec["isaac_outcome"], "target_frames": int(len(ttraj)),
           "condition": dict(condition)}
    steps_log, lateness = [], []
    L = None
    base_env = dict(os.environ)
    try:
        with OS.OwnedSession(ep_dir) as session:
            name = "motar_s3b_formal"
            base = (V0.WORLDS / "empty.world").read_text()
            session.write_world(name, world_text(base, centers, halves, p0, yaw0, V0.ODE_ITERS, V0.SPAWN, V0.rot))
            plan = OS.direct_launch_plan(session.episode_dir, name, base_env, model=condition["model"])
            res["runtime_binding"] = OS.runtime_binding(plan, airframe=AIRFRAMES[condition["model"]])
            OS.launch_px4_gazebo(session, plan)
            session.sleep(3)
            L = link_factory()
            try:
                if L.m.wait_heartbeat(timeout=120) is None:
                    raise RuntimeError("no heartbeat")
                th = threading.Thread(target=L.rx, daemon=True)
                th.start()
                try:
                    _fly(B, V0, L, session, spec, ep_dir, policy, condition, res, steps_log, lateness)
                finally:
                    L.stop = True
                    th.join(timeout=5.0)
                    res["rx_thread_alive_after_join"] = th.is_alive()
                if res["rx_thread_alive_after_join"]:
                    raise RuntimeError("RX_THREAD_STUCK: the MAVLink receiver did not stop")
            finally:
                L.stop = True
                getattr(L.m, "close", lambda: None)()
                res["link_closed"] = True
    except Exception as e:                                               # noqa: BLE001
        res["outcome"] = "HARNESS_INVALID" if "HARNESS_INVALID" in str(e) else "ERROR"
        res["error"] = "%s: %s" % (e.__class__.__name__, e)
    finally:
        if L is not None:
            L.stop = True
        ep_dir = pathlib.Path(ep_dir)
        if ep_dir.is_dir():
            np.save(ep_dir / "steps.npy", np.array(steps_log, dtype=np.float64))
            receipt = ep_dir / "session.json"
            res["session_receipt_sha256"] = sha256_file(receipt) if receipt.is_file() else None
    return res


def _fly(B, V0, L, session, spec, ep_dir, policy, condition, res, steps_log, lateness):
    """Set-up, take-off, start check and the control loop: the development runner's steps, in order."""
    import numpy as np
    centers, halves, quats = spec["centers"], spec["halves"], spec["quats"]
    p0, yaw0, ttraj = spec["p0"], spec["yaw0"], spec["tpos"]
    bmin, bmax = np.asarray(spec["bounds_min"], float), np.asarray(spec["bounds_max"], float)
    mu = L.mu.mavlink
    for mid in (mu.MAVLINK_MSG_ID_LOCAL_POSITION_NED, mu.MAVLINK_MSG_ID_ATTITUDE,
                mu.MAVLINK_MSG_ID_HIL_STATE_QUATERNION):
        L.cmd(mu.MAV_CMD_SET_MESSAGE_INTERVAL, mid, 20000)
    t0 = time.time()
    while time.time() - t0 < 90 and (L.pos is None or L.att is None or L.truth is None):
        session.sleep(0.2)
    session.sleep(10)
    for k, v in V0.PARAMS.items():
        L.send(L.m.mav.param_set_send, L.m.target_system, L.m.target_component, k.encode(), float(v),
               mu.MAV_PARAM_TYPE_REAL32)
        time.sleep(0.05)
    res["params_readback"] = L.read_params(list(V0.PARAMS) + ["MC_YAWRATE_P", "MC_YAWRATE_I",
                                                              "EKF2_HGT_REF", "MPC_XY_VEL_MAX"])
    res["param_problems"] = params_problems(res["params_readback"], V0.PARAMS)
    x0n, y0e = L.pos[1], L.pos[2]
    lat0, lon0, alt0 = L.truth[1], L.truth[2], L.truth[3]
    R_E = 6371000.0

    def truth_local():
        tr = L.truth
        dn = math.radians(tr[1] - lat0) * R_E
        de = math.radians(tr[2] - lon0) * R_E * math.cos(math.radians(lat0))
        w, x, y, z = tr[4], tr[5], tr[6], tr[7]
        return de, dn, tr[3] - alt0, math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

    def hold(seconds):
        for _ in range(int(round(seconds / V0.DT))):
            L.setpoint(V0.TM_POS_XYZ, x0n, y0e, -V0.ALT)
            time.sleep(V0.DT)
    hold(2.0)
    L.cmd(mu.MAV_CMD_DO_SET_MODE, 1, 6)
    hold(1.0)
    L.cmd(mu.MAV_CMD_COMPONENT_ARM_DISARM, 1)
    hold(10.0)
    for _ in range(3):
        tx, ty, _, _ = truth_local()
        sp_n, sp_e = L.pos[1] - ty, L.pos[2] - tx
        for _ in range(int(round(3.0 / V0.DT))):
            L.setpoint(V0.TM_POS_XYZ, sp_n, sp_e, -V0.ALT)
            time.sleep(V0.DT)
    armed = bool(L.hb and (L.hb[0] & 128))
    offboard = bool(L.hb and ((L.hb[1] >> 16) & 0xFF) == 6)
    tx, ty, tz, _ = truth_local()
    res["start_check"] = {"armed": armed, "offboard": offboard, "truth_alt_m": tz, "ekf_alt_m": -L.pos[3],
                          "ekf_minus_truth_xy_m": float(math.hypot(L.pos[2] - tx, L.pos[1] - ty)),
                          "truth_from_spawn_xy_m": float(math.hypot(tx, ty)), "command_yaw": V0.COMMAND_YAW}
    if not (armed and offboard and abs(-L.pos[3] - V0.ALT) < 0.3 and 0.3 < tz < 2.5 and math.hypot(tx, ty) < 0.25):
        raise RuntimeError("HARNESS_INVALID start state %s" % res["start_check"])
    ep = B.Episode(centers, halves, quats)
    delay = CommandDelay(condition["delay_steps"])
    prev = np.zeros(4, np.float32)
    outcome, prev_rel = "timeout", None
    min_dist = float("inf")
    truth_age = []
    sim_start = L.pos[0]
    wall_start = time.time()
    for k in range(V0.MAX_STEPS):
        wait_for_step(L, session, sim_start + k * V0.DT)
        pn = L.pos
        at = L.att
        lateness.append(pn[0] - (sim_start + k * V0.DT))
        truth_age.append(time.time() - L.truth[0])
        gx, gy = pn[2], pn[1]
        wx, wy = V0.rot(yaw0, gx, gy)
        p = np.array([p0[0] + wx, p0[1] + wy, -pn[3]])
        vwx, vwy = V0.rot(yaw0, pn[5], pn[4])
        v = np.array([vwx, vwy, -pn[6]])
        yaw = yaw0 + (math.pi / 2 - at[3])
        vq = np.array([0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)])
        tgx, tgy, tgz, tyaw_ned = truth_local()
        twx, twy = V0.rot(yaw0, tgx, tgy)
        p_true = np.array([p0[0] + twx, p0[1] + twy, tgz])
        yaw_true = yaw0 + (math.pi / 2 - tyaw_ned)
        vq_true = np.array([0.0, 0.0, math.sin(yaw_true / 2), math.cos(yaw_true / 2)])
        tpos = np.asarray(ttraj[min(k, len(ttraj) - 1)], float)
        s = {"pos": p, "vel": v, "vquat": vq, "fquat": vq, "angvel": np.array([0.0, 0.0, -at[6]]),
             "tpos": tpos, "tquat": None, "prev_action": prev, "bounds_min": bmin, "bounds_max": bmax,
             "render_pos": p_true, "render_vquat": vq_true, "render_fquat": vq_true}
        c0 = time.time()
        obs, diag, lidar = ep.observe(s)
        act = policy.act(obs)[0].numpy()
        gov, wz, prev_t = ep.command(act, lidar)
        prev = prev_t.numpy()
        cyaw = yaw_true if V0.COMMAND_YAW == "truth" else yaw
        vwx2, vwy2 = V0.rot(cyaw, float(gov[0]), float(gov[1]))
        gex, gey = V0.rot(-yaw0, vwx2, vwy2)
        cex, cey, cwz = delay((gex, gey, float(wz)))
        L.setpoint(V0.TM_POS_Z_VEL_XY_YAWRATE, 0, 0, -V0.ALT, cey, cex, -cwz)
        compute_ms = 1e3 * (time.time() - c0)
        rel = p_true - tpos
        dist = float(np.linalg.norm(rel))
        sweep = V0.seg_min_dist(prev_rel, rel) if prev_rel is not None else dist
        min_dist = min(min_dist, sweep)
        prev_rel = rel
        hit, clr = V0.box_hit(p_true, centers, halves)
        steps_log.append([k, pn[0], *p.tolist(), *v.tolist(), yaw, *act.tolist(), float(gov[0]),
                          float(gov[1]), float(wz), *tpos.tolist(), dist, clr, compute_ms,
                          bool(diag["visible"][0]), *p_true.tolist(), yaw_true])
        if sweep < V0.CAPTURE_M:
            outcome = "capture"
            break
        oob = (p_true[:2] < bmin[:2] - V0.OOB_MARGIN).any() or (p_true[:2] > bmax[:2] + V0.OOB_MARGIN).any()
        if hit or p_true[2] < 0.1 or p_true[2] > 4.0 or oob:
            outcome = "crash"
            res["crash_reason"] = "bar" if hit else ("altitude" if (p_true[2] < 0.1 or p_true[2] > 4.0)
                                                     else "bounds")
            break
    res["real_time_factor"] = (L.pos[0] - sim_start) / max(1e-9, time.time() - wall_start)
    est = [math.hypot(r[2] - r[23], r[3] - r[24]) for r in steps_log]
    res["ekf_vs_truth_xy_err_m"] = {"median": float(np.median(est)), "max": float(max(est))} if est else None
    res.update(outcome=outcome, steps=len(steps_log), min_dist_m=min_dist,
               compute_ms_median=float(np.median([r[21] for r in steps_log])) if steps_log else None,
               compute_ms_max=float(max(r[21] for r in steps_log)) if steps_log else None,
               late_max_s=float(max(lateness)) if lateness else None,
               late_p99_s=float(np.percentile(lateness, 99)) if lateness else None,
               truth_age_max_s=float(max(truth_age)) if truth_age else None)
    L.cmd(mu.MAV_CMD_DO_SET_MODE, 1, 4, 6)
    time.sleep(1)


def decode_param(msg):
    """PX4 sends INT32 parameters byte-cast into the float field (review r2 l2)."""
    v = float(msg.param_value)
    if getattr(msg, "param_type", None) == MAV_PARAM_TYPE_INT32:
        return float(struct.unpack("<i", struct.pack("<f", v))[0])
    return v


def _param_link_class():
    import sitl_s3b_closed_loop as V0

    class _ParamLink(V0.Link):
        """The development runner's MAVLink link, which also keeps PARAM_VALUE messages for the read-back."""

        def __init__(self):
            super().__init__()
            self.params = {}

        def rx(self):
            while not self.stop:
                msg = self.m.recv_match(blocking=True, timeout=0.5)
                if msg is None:
                    continue
                k = msg.get_type()
                if k == "LOCAL_POSITION_NED":
                    self.pos = (msg.time_boot_ms * 1e-3, msg.x, msg.y, msg.z, msg.vx, msg.vy, msg.vz)
                elif k == "HIL_STATE_QUATERNION":
                    q = msg.attitude_quaternion
                    self.truth = (time.time(), msg.lat * 1e-7, msg.lon * 1e-7, msg.alt * 1e-3, q[0], q[1], q[2], q[3],
                                  msg.vx * 1e-2, msg.vy * 1e-2, msg.vz * 1e-2)
                elif k == "HEARTBEAT" and msg.get_srcSystem() == self.m.target_system:
                    self.hb = (msg.base_mode, msg.custom_mode)
                elif k == "ATTITUDE":
                    self.att = (msg.time_boot_ms * 1e-3, msg.roll, msg.pitch, msg.yaw, msg.rollspeed, msg.pitchspeed,
                                msg.yawspeed)
                elif k == "PARAM_VALUE":
                    pid = msg.param_id
                    pid = pid.decode() if isinstance(pid, bytes) else pid
                    self.params[pid.rstrip("\x00")] = decode_param(msg)

        def read_params(self, names, timeout=8.0):
            for n in names:
                self.params.pop(n, None)
                self.send(self.m.mav.param_request_read_send, self.m.target_system, self.m.target_component,
                          n.encode(), -1)
                time.sleep(0.05)
            t0 = time.time()
            while time.time() - t0 < timeout and any(n not in self.params for n in names):
                time.sleep(0.1)
            return {n: self.params.get(n) for n in names}

    return _ParamLink


def ParamLink():
    return _param_link_class()()


# ----------------------------------------------------------------------------------------------- unit


def environment_report():
    import sitl_dev08_attempt as AT
    env = AT.process_environment()
    return env, AT.environment_problems(env)


def check(verbose=True):
    problems = input_problems()
    sched = schedule() if not problems else []
    counts = collections.Counter(s["block"] for s in sched)
    env, env_p = environment_report()
    out = {"input_problems": problems, "schedule_counts": dict(counts), "episodes": len(sched),
           "environment": env, "environment_problems": env_p}
    if verbose:
        print(json.dumps(out, indent=1))
    return out


def write_line(path, record):
    with open(path, "a") as f:
        f.write(json.dumps(record, default=str) + "\n")
        f.flush()
        os.fsync(f.fileno())


def launch_problems(AT, B, gs, approval_path, scope=SCOPE, sources=None, attempt_dir=None):
    """Every refusal reason before anything is created (review r1 M2, m1, m5)."""
    sources = sources if sources is not None else sorted(set(OWN_SOURCES) | set(AT.SOURCES))
    attempt_dir = attempt_dir or ATTEMPT_DIR
    problems = []
    if not approval_path.is_file():
        problems.append("no approval file %s" % approval_path)
    else:
        problems += approval_problems(json.loads(approval_path.read_text()), gs["head"], scope=scope)
    if gs["dirty"]:
        problems.append("tracked tree is not clean")
    problems += AT.source_problems(ROOT, sources)
    if pathlib.Path(B.ROOT).resolve() != ROOT.resolve():     # the bridge loads production code from B.ROOT by path
        problems.append("the bridge loads production code from %s, not this worktree" % B.ROOT)
    if not pathlib.Path(B.F_CKPT).is_file() or sha256_file(B.F_CKPT) != F_SHA256:
        problems.append("Policy F checkpoint missing or not %s" % F_SHA256[:12])
    _, ext_p = AT.external_binding(EXTERNAL_SOURCES)
    problems += ext_p
    problems += input_problems()
    problems += environment_report()[1]
    problems += env_problems()
    import sitl_s3b_closed_loop as V0
    if V0.COMMAND_YAW != "truth":
        problems.append("command yaw is %r, not truth" % V0.COMMAND_YAW)
    idle_ok, idle = AT.idle_check()
    if not idle_ok:
        problems.append("runner cores not idle: %s" % idle)
    if attempt_dir.exists():
        problems.append("attempt directory exists (single use): %s" % attempt_dir)
    return problems


def run_unit(sched, specs, run_one, AT, ledger, invalid_abort_a=INVALID_ABORT_A, key="block", primary="A",
             consecutive_stop=CONSECUTIVE_INVALID_STOP, busy_wait_s=None):
    """Run the schedule in order with the stop rules. run_one(i, item, spec) -> episode record.
    Returns (status, reason, episodes recorded). Status is COMPLETE; ABORTED (a stop in the primary block, a fault
    rise, a tree change, an exception, or a termination request kept in run_unit.terminated); or STOPPED_AFTER_A
    (a later block stopped once the primary block was complete). Per episode: an idle check before it (if still busy
    after one wait: recorded INVALID HOST_BUSY without running), the runtime binding against the first VALID episode
    of its condition (BINDING_CHANGED), and the tracked tree (unchanged, or the unit stops)."""
    busy_wait_s = HOST_BUSY_WAIT_S if busy_wait_s is None else busy_wait_s
    done, invalid_a, consecutive = 0, 0, 0
    refs = {}
    try:
        faults0 = AT.kernel_faults()
        git0 = AT.git_state(ROOT)
        for i, item in enumerate(sched):
            skey = (item["bars"], item["env"])
            spec = specs[skey] if skey in specs else specs[item["env"]]
            if spec["target_source"] != "zero_action_cell" or len(spec["tpos"]) < FULL_HORIZON:
                return "ABORTED", "scene %s has no full-horizon target trajectory" % item, done
            idle_ok, idle = AT.idle_check()
            if not idle_ok and busy_wait_s > 0:
                time.sleep(busy_wait_s)
                idle_ok, idle = AT.idle_check()
            t0 = time.time()
            if idle_ok:
                res = run_one(i, item, spec)
                ok, why = episode_valid(res)
                core = binding_core(res.get("runtime_binding"))
                cond = item.get("arm", "main")
                if ok and core:
                    refs.setdefault(cond, core)
                    if core != refs[cond]:
                        ok, why = False, "BINDING_CHANGED"
            else:
                res, ok, why = {"outcome": "NOT_RUN"}, False, "HOST_BUSY"
            faults = AT.kernel_faults()
            rec = dict(item, index=i, wall_s=time.time() - t0, kernel_faults=faults, valid=ok, invalid_reason=why,
                       host_idle_before=dict(idle, ok=idle_ok), **res)
            write_line(ledger, rec)
            done += 1
            print(json.dumps({k: rec.get(k) for k in ("index", key, "bars", "env", "outcome", "isaac_outcome",
                                                       "valid", "invalid_reason", "late_max_s", "wall_s")}),
                  flush=True)
            if faults != faults0:
                return "ABORTED", "kernel fault count rose %d -> %d" % (faults0, faults), done
            if AT.git_state(ROOT) != git0:
                return "ABORTED", "the tracked tree changed during episode %d" % i, done
            if i == 0 and not ok:
                return "ABORTED", "first episode invalid (%s): the harness does not work" % why, done
            consecutive = 0 if ok else consecutive + 1
            in_primary = primary is None or item[key] == primary
            if in_primary and not ok:
                invalid_a += 1
                if invalid_a > invalid_abort_a:
                    return "ABORTED", "more than %d invalid %s episodes" % (invalid_abort_a, primary), done
            if consecutive >= consecutive_stop:
                stop = "%d consecutive invalid episodes (last: %s)" % (consecutive, why)
                return ("ABORTED" if in_primary else "STOPPED_AFTER_A"), stop, done
        return "COMPLETE", None, done
    except BaseException as e:                                       # noqa: BLE001 - recorded, then re-raised
        if not isinstance(e, Exception):
            run_unit.terminated = e
        return "ABORTED", "%s: %s" % (e.__class__.__name__, e), done


run_unit.terminated = None


def write_final(attempt_dir, AT, status, reason, done, sched, faults0):
    try:
        faults1 = AT.kernel_faults()
    except Exception as e:                                   # noqa: BLE001
        faults1 = "unreadable: %s" % e
    try:
        git_after = AT.git_state(ROOT)
    except Exception as e:                                   # noqa: BLE001
        git_after = {"error": str(e)}
    final = {"status": status, "reason": reason, "episodes_recorded": done,
             "episodes_scheduled": len(sched) if sched is not None else None, "kernel_faults_before": faults0,
             "kernel_faults_after": faults1, "finished": time.time(), "git_after": git_after}
    (pathlib.Path(attempt_dir) / "final.json").write_text(json.dumps(final, indent=1, default=str) + "\n")
    print(json.dumps(final, default=str), flush=True)
    return final


def launch():
    signal.signal(signal.SIGTERM, _on_sigterm)               # review r2 MINOR-2: a kill is recorded, not silent
    import sitl_dev08_attempt as AT
    import sitl_bridge as B
    gs = AT.git_state(ROOT)
    approval_path = UNIT_DIR / "execution_approval.json"
    problems = launch_problems(AT, B, gs, approval_path)
    faults0 = None
    if not problems:
        try:
            faults0 = AT.kernel_faults()                     # read before the single-use directory exists (m1)
        except Exception as e:                               # noqa: BLE001
            problems.append("kernel fault baseline unreadable: %s" % e)
    if problems:
        raise SystemExit("refused:\n  " + "\n  ".join(problems))
    ATTEMPT_DIR.mkdir(parents=True)
    sched, done, terminated = None, 0, None
    try:
        sched = schedule()
        ext, _ = AT.external_binding(EXTERNAL_SOURCES)
        attempt = {"protocol": PROTOCOL, "prereg": PREREG, "scope": SCOPE, "git": gs, "condition": CONDITION,
                   "approval_sha256": sha256_file(approval_path), "inputs": INPUT_SHA256, "f_sha256": F_SHA256,
                   "sources": {s: sha256_file(ROOT / s) for s in sorted(set(OWN_SOURCES) | set(AT.SOURCES))},
                   "external_sources": ext, "environment": environment_report()[0],
                   "environment_keys": sorted(os.environ), "idle": AT.idle_check()[1],
                   "schedule": sched, "kernel_faults_before": faults0, "started": time.time()}
        (ATTEMPT_DIR / "attempt.json").write_text(json.dumps(attempt, indent=1, default=str) + "\n")
        import sitl_owned_session as OS
        import sitl_s3b_closed_loop as V0
        specs = {}
        for bars in sorted({s["bars"] for s in sched}):
            full, zero = G8 / "full" / (CELL % bars), G8 / "zero" / (CELL % bars)
            for e in V0.episodes_from_trace(full, 1000, 0, B.ROOT, zero):
                specs[(bars, e["env"])] = e
        B.setup(specs[(sched[0]["bars"], sched[0]["env"])]["receipt_env"])
        policy = B.Policy()
        run_unit.terminated = None
        status, reason, done = run_unit(
            sched, specs, lambda i, item, spec: run_episode(B, OS, V0, spec, ATTEMPT_DIR / ("ep%03d" % i), policy),
            AT, ATTEMPT_DIR / "episodes.jsonl")
        terminated = run_unit.terminated
    except BaseException as e:                               # noqa: BLE001 - recorded, then re-raised
        status, reason = "ABORTED", "%s: %s" % (e.__class__.__name__, e)
        if not isinstance(e, Exception):
            terminated = e
    write_final(ATTEMPT_DIR, AT, status, reason, done, sched, faults0)
    if terminated is not None:
        raise terminated


# ----------------------------------------------------------------------------------------------- analysis


def block_summary(rows, isaac):
    valid = [r for r in rows if r["valid"]]
    n = len(valid)
    sitl = collections.Counter(r["outcome"] for r in valid)
    isa = collections.Counter(isaac[r["env"]] for r in valid)
    agree = sum(1 for r in valid if r["outcome"] == isaac[r["env"]])
    rate = lambda c, k: (100.0 * c[k] / n) if n else None          # noqa: E731
    out = {"scheduled": len(rows), "valid": n, "invalid": collections.Counter(r["invalid_reason"] for r in rows
                                                                              if not r["valid"]),
           "sitl": dict(sitl), "isaac_same_envs": dict(isa), "outcome_agreement": agree,
           "sitl_crash_pct": rate(sitl, "crash"), "isaac_crash_pct": rate(isa, "crash"),
           "sitl_capture_pct": rate(sitl, "capture"), "isaac_capture_pct": rate(isa, "capture")}
    if n:
        out["crash_diff_pp"] = out["sitl_crash_pct"] - out["isaac_crash_pct"]
        out["capture_diff_pp"] = out["sitl_capture_pct"] - out["isaac_capture_pct"]
    return out


def integrity_problems(attempt, final, rows):
    """The ledger must be the schedule's prefix in order, all of it when COMPLETE, with no fault increase and the
    tree unchanged at the end (review r1 m4)."""
    p = []
    sched = attempt.get("schedule", [])
    got = [(r["block"], r["env"]) for r in rows]
    want = [(s["block"], s["env"]) for s in sched[:len(rows)]]
    if got != want:
        p.append("ledger rows are not the schedule's prefix in order")
    if final.get("status") == "COMPLETE" and len(rows) != len(sched):
        p.append("COMPLETE with %d of %d scheduled episodes" % (len(rows), len(sched)))
    n_a = sum(1 for s in sched if s["block"] == "A")
    if final.get("status") == "STOPPED_AFTER_A" and sum(1 for r in rows if r["block"] == "A") != n_a:
        p.append("STOPPED_AFTER_A before block A completed")
    if final.get("kernel_faults_after") != attempt.get("kernel_faults_before"):
        p.append("kernel faults %r -> %r" % (attempt.get("kernel_faults_before"), final.get("kernel_faults_after")))
    ga, g0 = final.get("git_after") or {}, attempt.get("git") or {}
    if ga.get("head") != g0.get("head") or ga.get("dirty"):
        p.append("tree changed during the run")
    return p


def timing_summary(rows):
    """Reported per block: lateness, truth age, compute, real-time factor, EKF error, parameter problems."""
    out = {}
    for b in sorted({r["block"] for r in rows}):
        v = [r for r in rows if r["block"] == b and r["valid"]]
        g = lambda k: [r[k] for r in v if r.get(k) is not None]                       # noqa: E731
        ek = [r["ekf_vs_truth_xy_err_m"]["max"] for r in v if r.get("ekf_vs_truth_xy_err_m")]
        out[b] = {"late_max_s": max(g("late_max_s"), default=None),
                  "truth_age_max_s": max(g("truth_age_max_s"), default=None),
                  "compute_ms_max": max(g("compute_ms_max"), default=None),
                  "rtf_min": min(g("real_time_factor"), default=None),
                  "rtf_max": max(g("real_time_factor"), default=None),
                  "ekf_xy_err_max_m": max(ek, default=None),
                  "param_problem_episodes": sum(1 for r in rows if r["block"] == b and r.get("param_problems"))}
    return out


def clopper_pearson(k, n, alpha=0.05):
    if not n:
        return None
    try:
        from scipy.stats import beta
    except ImportError:                                          # reported only; absent scipy leaves it unset
        return None
    lo = 0.0 if k == 0 else float(beta.ppf(alpha / 2, k, n - k + 1))
    hi = 1.0 if k == n else float(beta.ppf(1 - alpha / 2, k + 1, n - k))
    return [100.0 * lo, 100.0 * hi]


def reading(final, rows, isaac_by_bars, attempt=None):
    by = collections.defaultdict(list)
    for r in rows:
        by[r["block"]].append(r)
    blocks = {b: block_summary(by[b], isaac_by_bars[by[b][0]["bars"]]) for b in sorted(by) if by[b]}
    a = blocks.get("A")
    integ = integrity_problems(attempt, final, rows) if attempt is not None else ["no attempt record"]
    if final.get("status") not in ("COMPLETE", "STOPPED_AFTER_A"):
        label, why = "VOID", "attempt %s: %s" % (final.get("status"), final.get("reason"))
    elif integ:
        label, why = "VOID", "; ".join(integ)
    elif a is None or a["valid"] < VALID_MIN_A:
        label, why = "VOID", "fewer than %d valid block-A episodes" % VALID_MIN_A
    elif a["crash_diff_pp"] <= MARGIN_PP:
        label, why = "FEASIBLE_IN_SITL", None
    else:
        label, why = "NOT_FEASIBLE_IN_SITL", None
    out = {"protocol": PROTOCOL, "reading": label, "void_reason": why, "margin_pp": MARGIN_PP, "blocks": blocks,
           "integrity_problems": integ, "timing_and_state": timing_summary(rows)}
    if a is not None:
        isaac_a = isaac_by_bars[70]
        ar = by["A"]
        worst = sum(1 for r in ar if (not r["valid"]) or r["outcome"] == "crash")
        out["block_A_reported"] = {
            "sitl_crash_pct_ci95": clopper_pearson(a["sitl"].get("crash", 0), a["valid"]),
            "worst_case_invalid_as_crash": {
                "n": len(ar), "sitl_crash_pct": 100.0 * worst / len(ar),
                "isaac_crash_pct": 100.0 * sum(1 for r in ar if isaac_a[r["env"]] == "crash") / len(ar)}}
        w = out["block_A_reported"]["worst_case_invalid_as_crash"]
        w["crash_diff_pp"] = w["sitl_crash_pct"] - w["isaac_crash_pct"]
    first = {r["env"]: r for r in by.get("A", []) if r["valid"]}
    rep = [(first[r["env"]]["outcome"], r["outcome"]) for r in by.get("C", []) if r["valid"] and r["env"] in first]
    out["repeat_C"] = {"pairs": len(rep), "same_outcome": sum(1 for a_, b_ in rep if a_ == b_),
                       "pairs_list": rep}
    return out


def analyze(attempt_dir=ATTEMPT_DIR):
    attempt_dir = pathlib.Path(attempt_dir)
    load = lambda name, default: (json.loads((attempt_dir / name).read_text())          # noqa: E731
                                  if (attempt_dir / name).is_file() else default)
    attempt = load("attempt.json", None)
    final = load("final.json", {"status": "NO_FINAL_RECORD", "reason": "final.json missing"})
    rows = read_jsonl(attempt_dir / "episodes.jsonl") if (attempt_dir / "episodes.jsonl").is_file() else []
    isaac = {bars: isaac_outcomes(G8, bars) for bars in (70, 205)}
    out = reading(final, rows, isaac, attempt)
    out["input_problems"] = input_problems()
    (attempt_dir / "reading.json").write_text(json.dumps(out, indent=1, sort_keys=True) + "\n")
    print(json.dumps({k: out[k] for k in ("reading", "void_reason")}), flush=True)
    return out


def main(argv=None):
    ap = argparse.ArgumentParser()
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--check", action="store_true")
    g.add_argument("--launch", action="store_true")
    g.add_argument("--analyze", nargs="?", const=str(ATTEMPT_DIR))
    a = ap.parse_args(argv)
    if a.check:
        check()
    elif a.launch:
        launch()
    else:
        analyze(a.analyze)


if __name__ == "__main__":
    main()
