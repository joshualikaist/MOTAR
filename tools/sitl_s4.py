"""S4 v1: the S3b loop with Gazebo's own sensors (plan stage S4). Preregistration: docs/plans/prereg_sitl_s4_v1_2026-11-06.md.

The episode is tools/sitl_s3b_formal.run_episode's, step for step, except where the sensors come from:
  - the vehicle is navrl_quad_s4 (navrl_quad, airframe 10090's body, plus gpu_ray LiDAR, RGB and depth cameras; see
    tools/sitl_s4_model.py), and the world adds the red target sphere and the plugin tools/sitl_s4/motar_s4_bridge.cc;
  - each step the replayed target position is sent to Gazebo one step ahead (it is known from the recording), and the
    newest LiDAR, RGB and depth frames are converted (tools/sitl_s4_gazebo.py) and given to the same perception,
    policy and governor in place of MOTAR's ray casts;
  - arm "raw" passes Gazebo's frames as delivered; arm "ground" first removes returns that hit the ground plane
    (MOTAR's sensors never see a ground; Gazebo's and real ones do);
  - MOTAR's own render at the same pose is computed beside it as a shadow, for the record only.
Reported, not gated (the plan's S4: "reported; detector recall in Gazebo").

  tools/sitl_s4.py --check | --launch | --analyze [ATTEMPT_DIR]
"""
import argparse
import collections
import json
import math
import os
import pathlib
import sys
import threading
import time

TOOLS = pathlib.Path(__file__).resolve().parent
ROOT = TOOLS.parent
sys.path.insert(0, str(TOOLS))
import sitl_s3b_formal as SF                                    # noqa: E402
import sitl_s4_gazebo as G                                      # noqa: E402

PROTOCOL = "s4-v1"
PREREG = "docs/plans/prereg_sitl_s4_v1_2026-11-06.md"
SCOPE = ("S4 v1: Frozen Policy F through PX4 v1.14.3 SITL on navrl_quad_s4 (visuals removed) with Gazebo gpu_ray "
         "LiDAR, RGB and depth cameras (OpenGL rendering on display :1, no CUDA), G8 r2 seed 4501 70-bar scenes; arm "
         "ground (first 40 eligible) then arm raw (first 20 eligible); development smoke01 (the first scheduled "
         "episode once, not read) then attempt01; episode lateness bound 0.15 s including frame age; no retries")
UNIT_DIR = pathlib.Path.home() / "motar_tmp" / "s4_20261106"
ATTEMPT_DIR = UNIT_DIR / "attempt01"
SMOKE_DIR = UNIT_DIR / "smoke01"
PLUGIN_SRC = ROOT / "tools/sitl_s4/motar_s4_bridge.cc"
PLUGIN_DIR = pathlib.Path.home() / "motar_tmp" / "s4_gazebo_20261106" / "build"
ARMS = (("ground", 40), ("raw", 20))
MODEL = "navrl_quad_s4"
GPU_ALLOWED = ("/usr/share/rustdesk/rustdesk",)   # the user's remote desktop (screen encoding), not a compute job
HOVER_GUARD_M = 0.35                  # any LiDAR return or depth pixel nearer than this at the start: HARNESS_INVALID
INVALID_ABORT_TOTAL = 10                # more than 10 INVALID episodes of the 60 stop the unit
OWN_SOURCES = ("tools/sitl_s4.py", "tools/sitl_s4_gazebo.py", "tools/sitl_s4_model.py", "tools/sitl_s4/motar_s4_bridge.cc",
               "tools/sitl_s4/CMakeLists.txt", "tools/sitl_s3c.py", "tests/test_sitl_s3c_s4.py") + SF.OWN_SOURCES + (PREREG,)


def schedule(g8=SF.G8):
    envs = SF.eligible_envs(SF.read_jsonl(g8 / "zero" / (SF.CELL % 70) / "kroc2027_episodes.jsonl"))
    return [{"arm": arm, "bars": 70, "env": e} for arm, n in ARMS for e in envs[:n]]


def colour_px(rgb):
    import numpy as np
    R, Gc, B = rgb[0]
    return int(((1.0 / (1.0 + np.exp(-(3 * R - 2 * Gc - 2 * B - 0.9)))) >= 0.55).sum())


def run_episode(B, OS, V0, spec, ep_dir, policy, arm, link_factory=None, client_factory=None):
    """tools/sitl_s3b_formal.run_episode with Gazebo's sensors; see the module docstring for the only differences.
    The MAVLink receiver and the plugin connection are stopped and closed on every exit path."""
    import numpy as np
    link_factory = link_factory or SF.ParamLink
    client_factory = client_factory or G.S4Client
    centers, halves = spec["centers"], spec["halves"]
    p0, yaw0, ttraj = spec["p0"], spec["yaw0"], spec["tpos"]
    res = {"env": spec["env"], "isaac_outcome": spec["isaac_outcome"], "target_frames": int(len(ttraj)), "arm": arm,
           "model": MODEL}
    steps_log, lateness, sens = [], [], []
    L = None
    base_env = dict(os.environ)
    base_env["GAZEBO_PLUGIN_PATH"] = str(PLUGIN_DIR) + ((":" + base_env["GAZEBO_PLUGIN_PATH"])
                                                        if base_env.get("GAZEBO_PLUGIN_PATH") else "")
    try:
        with OS.OwnedSession(ep_dir) as session:
            name = "motar_s4"
            base = (V0.WORLDS / "empty.world").read_text()
            session.write_world(name, G.world_text_s4(SF.world_text(base, centers, halves, p0, yaw0, V0.ODE_ITERS,
                                                                     V0.SPAWN, V0.rot)))
            plan = OS.direct_launch_plan(session.episode_dir, name, base_env, model=MODEL)
            res["runtime_binding"] = OS.runtime_binding(plan, airframe=SF.AIRFRAMES[MODEL])
            so = PLUGIN_DIR / "libmotar_s4_bridge.so"
            res["runtime_binding"]["files"]["s4_plugin"] = {"path": str(so), "sha256": SF.sha256_file(so)
                                                             if so.is_file() else None}
            OS.launch_px4_gazebo(session, plan)
            session.sleep(3)
            L = link_factory()
            C = None
            try:
                if L.m.wait_heartbeat(timeout=120) is None:
                    raise RuntimeError("no heartbeat")
                th = threading.Thread(target=L.rx, daemon=True)
                th.start()
                try:
                    for _ in range(60):
                        try:
                            C = client_factory()
                            break
                        except OSError:
                            session.sleep(0.5)
                    if C is None:
                        raise RuntimeError("HARNESS_INVALID no S4 plugin connection")
                    _fly(B, V0, L, C, session, spec, policy, arm, res, steps_log, lateness, sens)
                finally:
                    L.stop = True
                    th.join(timeout=5.0)
                    res["rx_thread_alive_after_join"] = th.is_alive()
                if res["rx_thread_alive_after_join"]:
                    raise RuntimeError("RX_THREAD_STUCK: the MAVLink receiver did not stop")
            finally:
                if C is not None:
                    C.close()
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
            np.save(ep_dir / "sensor_steps.npy", np.array(sens, dtype=np.float64))
            receipt = ep_dir / "session.json"
            res["session_receipt_sha256"] = SF.sha256_file(receipt) if receipt.is_file() else None
            res["dataman_removed_bytes"] = SF.remove_dataman(ep_dir)
    return res


def _fly(B, V0, L, C, session, spec, policy, arm, res, steps_log, lateness, sens):
    """The S3b set-up, take-off, start check and control loop, with the sensor substitution."""
    import numpy as np
    centers, halves, quats = spec["centers"], spec["halves"], spec["quats"]
    p0, yaw0, ttraj = spec["p0"], spec["yaw0"], spec["tpos"]
    bmin, bmax = np.asarray(spec["bounds_min"], float), np.asarray(spec["bounds_max"], float)

    def tgt(k):
        t = np.asarray(ttraj[min(k, len(ttraj) - 1)], float)
        gx, gy = G.to_gazebo_xy(t[:2], p0, yaw0, V0.SPAWN, V0.rot)
        return C.set_target(gx, gy, float(t[2]) + SF_H0)     # truth z is above the resting origin
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
    res["param_problems"] = SF.params_problems(res["params_readback"], V0.PARAMS)
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
    Ep = G.gazebo_episode_class(B)
    ep = Ep(centers, halves, quats)
    prev = np.zeros(4, np.float32)
    outcome, prev_rel = "timeout", None
    min_dist = float("inf")
    tgt(0)
    hover = C.frames()
    hl = G.motar_lidar(hover["ranges"])
    _, hd = G.motar_rgbd(hover["rgb"], hover["depth"])
    res["hover_guard"] = {"lidar_min_m": float(hl.min()), "depth_min_m": float(hd.min())}
    if hl.min() < HOVER_GUARD_M or hd.min() < HOVER_GUARD_M:
        raise RuntimeError("HARNESS_INVALID hover guard: a return nearer than %.2f m at the start %s"
                           % (HOVER_GUARD_M, res["hover_guard"]))
    truth_age = []
    sim_start = L.pos[0]
    wall_start = time.time()
    for k in range(V0.MAX_STEPS):
        SF.wait_for_step(L, session, sim_start + k * V0.DT)
        pn = L.pos
        at = L.att
        f = C.frames()
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
        lidar_g = G.motar_lidar(f["ranges"])
        rgb_g, depth_g = G.motar_rgbd(f["rgb"], f["depth"])
        removed = {}
        if arm == "ground":
            lidar_g, depth_g, removed = G.remove_ground(lidar_g, depth_g, L.truth[4:8], tgz + SF_H0)
        obs, diag, lidar, shadow = ep.observe_with(s, rgb_g, depth_g, lidar_g)
        act = policy.act(obs)[0].numpy()
        gov, wz, prev_t = ep.command(act, lidar)
        prev = prev_t.numpy()
        cyaw = yaw_true if V0.COMMAND_YAW == "truth" else yaw
        vwx2, vwy2 = V0.rot(cyaw, float(gov[0]), float(gov[1]))
        gex, gey = V0.rot(-yaw0, vwx2, vwy2)
        L.setpoint(V0.TM_POS_Z_VEL_XY_YAWRATE, 0, 0, -V0.ALT, gey, gex, -float(wz))
        compute_ms = 1e3 * (time.time() - c0)
        tgt(k + 1)                                            # next step's target, rendered by then
        sens.append([k, f["lidar_t"], f["rgb_t"], f["depth_t"], f["target_t"], colour_px(rgb_g),
                     shadow["motar_target_px"], bool(diag["camera_visible"][0]), shadow["lidar_abs_diff_median_m"],
                     shadow["lidar_abs_diff_max_m"], shadow["depth_abs_diff_median_m"],
                     removed.get("lidar_ground_removed", -1), removed.get("depth_ground_removed", -1),
                     bool(diag["lidar_visible"][0]), f["sim_now"], pn[0]])
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
    S = np.array(sens, dtype=np.float64) if sens else np.zeros((0, 16))
    if len(S):
        # frame age: the step's PX4 sim time minus the oldest of the three Gazebo frames used at that step (both
        # are Gazebo's lockstep simulation time; the plugin's sim_now is logged per step for the audit)
        frame_age = S[:, 15] - S[:, 1:4].min(axis=1)
        shadow_vis = S[:, 6] >= 2
        res["sensor"] = {
            "frame_age_max_s": float(frame_age.max()), "frame_age_median_s": float(np.median(frame_age)),
            "steps_motar_visible": int(shadow_vis.sum()),
            "steps_camera_visible_when_motar_visible": int((S[:, 7].astype(bool) & shadow_vis).sum()),
            "steps_lidar_only_visible_when_motar_visible": int((~S[:, 7].astype(bool) & S[:, 13].astype(bool)
                                                                 & shadow_vis).sum()),
            "colour_px_median_when_motar_visible": float(np.median(S[shadow_vis, 5])) if shadow_vis.any() else None,
            "motar_px_median_when_motar_visible": float(np.median(S[shadow_vis, 6])) if shadow_vis.any() else None,
            "lidar_abs_diff_median_m": float(np.median(S[:, 8])), "depth_abs_diff_median_m": float(np.median(S[:, 10]))}
        # a stale frame is late: lateness at a step is the larger of the state sample's lateness and the frame age,
        # under the same 0.15 s bound (S3c/S4 review r1)
        lateness = [max(a_, float(b_)) for a_, b_ in zip(lateness, frame_age)]
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


SF_H0 = 0.09        # the settled navrl_quad body-centre height above the ground, the truth origin's offset (AGQ diagnosis)


def check():
    out = {"input_problems": SF.input_problems(), "schedule": collections.Counter(s["arm"] for s in schedule()),
           "s4_launch_problems": s4_extra_problems(),
           "plugin": str(PLUGIN_DIR / "libmotar_s4_bridge.so"),
           "plugin_present": (PLUGIN_DIR / "libmotar_s4_bridge.so").is_file(),
           "display": os.environ.get("DISPLAY"), "environment_problems": SF.environment_report()[1]}
    print(json.dumps(out, indent=1, default=str))
    return out


def s4_extra_problems():
    import socket
    import subprocess
    p = []
    so = PLUGIN_DIR / "libmotar_s4_bridge.so"
    if not so.is_file():
        p.append("plugin not built: %s" % PLUGIN_DIR)
    elif so.stat().st_mtime < PLUGIN_SRC.stat().st_mtime:
        p.append("plugin %s is older than its source %s: rebuild it from the committed source" % (so, PLUGIN_SRC))
    if not os.environ.get("DISPLAY"):
        p.append("no DISPLAY: Gazebo's cameras need an X display for OpenGL rendering")
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        s.bind(("127.0.0.1", G.PORT))
    except OSError:
        p.append("TCP 127.0.0.1:%d is in use" % G.PORT)
    finally:
        s.close()
    try:
        done = subprocess.run(["nvidia-smi", "--query-compute-apps=pid,process_name", "--format=csv,noheader"],
                              capture_output=True, text=True, timeout=20)
        if done.returncode != 0:
            p.append("nvidia-smi failed (rc %d): the GPU check would be vacuous" % done.returncode)
        out = done.stdout.strip()
        jobs = [ln for ln in out.splitlines() if ln.strip() and not ln.strip().endswith(GPU_ALLOWED)]
        if jobs:
            p.append("a CUDA job is running on the GPU: %s" % "; ".join(jobs)[:200])
    except (OSError, subprocess.TimeoutExpired) as e:
        p.append("nvidia-smi unavailable: %s" % e)
    return p


def launch(smoke=False):
    import sitl_dev08_attempt as AT
    import sitl_s3c as S3C
    sources = sorted(set(OWN_SOURCES) | set(AT.SOURCES))
    out_dir = SMOKE_DIR if smoke else ATTEMPT_DIR
    so = PLUGIN_DIR / "libmotar_s4_bridge.so"
    px4m = pathlib.Path("/home/fair/workspaces/px4_ws/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/"
                        "models") / MODEL / (MODEL + ".sdf")
    attempt_extra = {"protocol": PROTOCOL, "prereg": PREREG, "arms": ARMS, "display": os.environ.get("DISPLAY"),
                     "plugin_sha256": SF.sha256_file(so) if so.is_file() else None,
                     "plugin_source_sha256": SF.sha256_file(PLUGIN_SRC),
                     "model_sdf_sha256": SF.sha256_file(px4m) if px4m.is_file() else None}

    def run_one_fn(B, OS, V0, policy):
        return lambda i, item, spec: run_episode(B, OS, V0, spec, out_dir / ("ep%03d" % i), policy, item["arm"])
    S3C.launch_unit(SCOPE, UNIT_DIR, out_dir, sources, schedule, attempt_extra, run_one_fn,
                    extra_problems=s4_extra_problems(), invalid_abort=INVALID_ABORT_TOTAL, smoke_dir=SMOKE_DIR,
                    smoke=smoke)


def analyze(attempt_dir=ATTEMPT_DIR, s3b_attempt=SF.ATTEMPT_DIR):
    import sitl_s3c as S3C
    attempt, final, rows = S3C.load_attempt(attempt_dir)
    isaac = SF.isaac_outcomes(SF.G8, 70)
    s3b, ref = S3C.s3b_reference(s3b_attempt)
    out = {"protocol": PROTOCOL, "attempt_status": final["status"], "attempt_reason": final.get("reason"),
           "integrity_problems": S3C.arm_integrity_problems(attempt, final, rows), "s3b_reference": ref, "arms": {},
           "timing_and_state": SF.timing_summary([dict(r, block=r["arm"]) for r in rows])}
    for arm, _ in ARMS:
        ar = [r for r in rows if r["arm"] == arm]
        if not ar:
            out["arms"][arm] = None
            continue
        summ = SF.block_summary(ar, isaac)
        valid = [r for r in ar if r["valid"] and r.get("sensor")]
        mv = sum(r["sensor"]["steps_motar_visible"] for r in valid)
        cv = sum(r["sensor"]["steps_camera_visible_when_motar_visible"] for r in valid)
        lv = sum(r["sensor"]["steps_lidar_only_visible_when_motar_visible"] for r in valid)
        med = lambda k: (float(sorted(r["sensor"][k] for r in valid if r["sensor"][k] is not None)[  # noqa: E731
            len([1 for r in valid if r["sensor"][k] is not None]) // 2])
            if any(r["sensor"][k] is not None for r in valid) else None)
        summ["detector_recall_steps"] = {
            "denominator": "steps where MOTAR's shadow renderer (yaw-level camera) has >= 2 target pixels",
            "motar_visible_steps": mv, "camera_visible_of_those": cv, "lidar_only_visible_of_those": lv,
            "camera_recall": (cv / mv) if mv else None}
        summ["sensor_medians_over_episodes"] = {k: med(k) for k in (
            "colour_px_median_when_motar_visible", "motar_px_median_when_motar_visible", "frame_age_median_s",
            "lidar_abs_diff_median_m", "depth_abs_diff_median_m")}
        summ["frame_age_max_s"] = max((r["sensor"]["frame_age_max_s"] for r in valid), default=None)
        summ["hover_guard_failures"] = sum(1 for r in ar if "hover guard" in str(r.get("error", "")))
        summ["vs_s3b_same_scenes"] = S3C.matched(ar, s3b)
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
