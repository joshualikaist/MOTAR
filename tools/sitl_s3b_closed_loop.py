"""S3b: Policy F closing the loop through PX4 SITL (docs/plans/sitl_validation_plan_v1.md). Development runner (v0).

For each episode taken from a recorded Isaac cell trace (bar layout, the pursuer's start pose, the target's recorded
trajectory, Isaac's outcome), this:
  1. writes a Gazebo world with the episode's bars as static boxes, transformed so that the vehicle's spawn point
     (sitl_run.sh: 1.01, 0.98) and Gazebo yaw 0 are the episode's start position and yaw;
  2. starts PX4 v1.14.3 SITL + Gazebo headless with the navrl_quad airframe (10090) and the S2 parameter set, takes off
     to 1.0 m (MOTAR's flight altitude) under offboard position control and holds 3 s;
  3. runs the episode at 10 Hz of PX4 sim time: PX4 state (LOCAL_POSITION_NED, ATTITUDE) -> MOTAR world frame ->
     tools/sitl_bridge.py (MOTAR's sensors against the known world and the target sphere at its recorded position,
     production perception, Policy F, riskcap) -> governed vehicle-frame velocity and yaw rate -> PX4 offboard
     velocity setpoint (NED) with the altitude held by a z position setpoint;
  4. ends on MOTAR's rules: capture when the pursuer-target distance (swept segment, 3-D) falls below 0.5 m; crash
     when the vehicle footprint (0.14 m half width) meets a bar box, z < 0.1 m or > 4 m, or it leaves the arena;
     timeout at 600 steps.
Known approximations (v0, disclosed in every report): the target replays its Isaac trajectory and does not react to
the SITL pursuer; the bridge's sensors are MOTAR's, not Gazebo's; the LiDAR and camera are yaw-level (MOTAR's mount);
the vehicle state is PX4's estimate.
usage: tools/sitl_s3b_closed_loop.py TRACE_CELL_DIR OUT_DIR N_EPISODES [FIRST_ENV]
"""
import json
import math
import os
import pathlib
import signal
import subprocess
import sys
import threading
import time

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.append("/home/fair/motar_tmp/sitl_plan_20261104/pylink")      # pymavlink from the px4 env (Python 3.8)

PX4 = pathlib.Path("/home/fair/workspaces/px4_ws/PX4-Autopilot")
BUILD = PX4 / "build/px4_sitl_default"
WORLDS = PX4 / "Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds"
SPAWN = (1.01, 0.98)
ALT = 1.0
DT = 0.1
MAX_STEPS = 600
CAPTURE_M = 0.5
FOOT_HALF = 0.14
PARAMS = {"MC_ROLLRATE_P": 0.013, "MC_ROLLRATE_I": 0.017, "MC_ROLLRATE_D": 0.00026, "MC_PITCHRATE_P": 0.013,
          "MC_PITCHRATE_I": 0.017, "MC_PITCHRATE_D": 0.00026, "MPC_XY_VEL_P_ACC": 2.5, "MPC_XY_VEL_I_ACC": 0.1,
          "MPC_XY_VEL_D_ACC": 0.0, "THR_MDL_FAC": 1.0}
TM_POS_XYZ = (8 | 16 | 32 | 64 | 128 | 256 | 1024 | 2048)
TM_POS_Z_VEL_XY_YAWRATE = (1 | 2 | 32 | 64 | 128 | 256 | 1024)


def rot(yaw, x, y):
    c, s = math.cos(yaw), math.sin(yaw)
    return c * x - s * y, s * x + c * y


def yaw_of(q):
    x, y, z, w = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def write_world(name, centers, halves, p0, yaw0):
    base = (WORLDS / "empty.world").read_text()
    models = []
    for i, (c, h) in enumerate(zip(centers, halves)):
        gx, gy = rot(-yaw0, c[0] - p0[0], c[1] - p0[1])
        gx, gy = gx + SPAWN[0], gy + SPAWN[1]
        size = "%.4f %.4f %.4f" % (2 * h[0], 2 * h[1], 2 * h[2])
        models.append(
            "<model name='bar_%d'><static>true</static><pose>%.4f %.4f %.4f 0 0 %.6f</pose><link name='l'>"
            "<collision name='c'><geometry><box><size>%s</size></box></geometry></collision>"
            "<visual name='v'><geometry><box><size>%s</size></box></geometry></visual></link></model>"
            % (i, gx, gy, c[2], -yaw0, size, size))
    world = base.replace("</world>", "\n".join(models) + "\n</world>", 1)
    (WORLDS / (name + ".world")).write_text(world)


def start_sitl(world, log):
    env = dict(os.environ, HEADLESS="1", NO_PXH="1", CUDA_VISIBLE_DEVICES="")
    cmd = [str(PX4 / "Tools/simulation/gazebo-classic/sitl_run.sh"), str(BUILD / "bin/px4"), "none", "navrl_quad", world,
           str(PX4), str(BUILD)]
    return subprocess.Popen(cmd, cwd=str(BUILD / "src/modules/simulation/simulator_mavlink"), env=env, stdout=log,
                            stderr=subprocess.STDOUT, preexec_fn=os.setsid)


def stop_sitl(proc):
    try:
        os.killpg(proc.pid, signal.SIGINT)
        proc.wait(timeout=20)
    except Exception:                                                # noqa: BLE001
        try:
            os.killpg(proc.pid, signal.SIGKILL)
        except Exception:                                            # noqa: BLE001
            pass
    for pat in ("gzserver", "px4_sitl_default/bin/px4"):
        subprocess.run(["pkill", "-u", str(os.getuid()), "-f", "^[^ ]*" + pat])


class Link:
    def __init__(self):
        from pymavlink import mavutil
        self.mu = mavutil
        self.m = mavutil.mavlink_connection("udpin:0.0.0.0:14540", source_system=245)
        self.lock = threading.Lock()
        self.pos = None          # (sim_t, x, y, z, vx, vy, vz) NED
        self.att = None          # (sim_t, roll, pitch, yaw, p, q, r)
        self.truth = None        # (wall_t, lat_deg, lon_deg, alt_m, qw, qx, qy, qz, vn, ve, vd)  HIL_STATE_QUATERNION
        self.hb = None           # (base_mode, custom_mode)
        self.stop = False

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

    def send(self, fn, *a):
        with self.lock:
            fn(*a)

    def cmd(self, command, *p):
        p = list(p) + [0] * (7 - len(p))
        self.send(self.m.mav.command_long_send, self.m.target_system, self.m.target_component, command, 0, *p)

    def setpoint(self, mask, x=0.0, y=0.0, z=0.0, vx=0.0, vy=0.0, yaw_rate=0.0):
        self.send(self.m.mav.set_position_target_local_ned_send, 0, self.m.target_system, self.m.target_component,
                  self.mu.mavlink.MAV_FRAME_LOCAL_NED, mask, x, y, z, vx, vy, 0, 0, 0, 0, 0, yaw_rate)


def box_hit(p, centers, halves, r=FOOT_HALF):
    """Vehicle footprint (radius-r disc, conservative for the 0.28 m box at any yaw within r*sqrt2) against bar boxes
    spanning the vehicle's height."""
    span = np.abs(p[2] - centers[:, 2]) <= halves[:, 2]
    d = np.abs(p[None, :2] - centers[:, :2]) - halves[:, :2]
    dist = np.hypot(np.maximum(d[:, 0], 0), np.maximum(d[:, 1], 0))
    dist[~span] = np.inf
    return bool((dist < r).any()), float(dist.min())


def seg_min_dist(a, b):
    """Minimum norm on the segment a->b (relative positions), for the swept capture test."""
    ab = b - a
    t = float(np.clip(-np.dot(a, ab) / max(1e-12, np.dot(ab, ab)), 0.0, 1.0))
    return float(np.linalg.norm(a + t * ab))


def run_episode(B, ep_spec, out_dir, idx, policy):
    centers, halves, quats = ep_spec["centers"], ep_spec["halves"], ep_spec["quats"]
    p0, yaw0, ttraj = ep_spec["p0"], ep_spec["yaw0"], ep_spec["tpos"]
    bmin, bmax = np.asarray(ep_spec["bounds_min"], float), np.asarray(ep_spec["bounds_max"], float)
    world = "motar_s3b_%02d" % idx
    write_world(world, centers, halves, p0, yaw0)
    log = open(out_dir / ("ep%02d_sitl.log" % idx), "w")
    proc = start_sitl(world, log)
    res = {"episode": idx, "env": ep_spec["env"], "isaac_outcome": ep_spec["isaac_outcome"], "world": world}
    steps_log = []
    L = None
    try:
        time.sleep(3)
        L = Link()
        if L.m.wait_heartbeat(timeout=120) is None:
            raise RuntimeError("no heartbeat")
        th = threading.Thread(target=L.rx, daemon=True)
        th.start()
        mu = L.mu.mavlink
        for mid in (mu.MAVLINK_MSG_ID_LOCAL_POSITION_NED, mu.MAVLINK_MSG_ID_ATTITUDE, mu.MAVLINK_MSG_ID_HIL_STATE_QUATERNION):
            L.cmd(mu.MAV_CMD_SET_MESSAGE_INTERVAL, mid, 20000)
        t0 = time.time()
        while time.time() - t0 < 90 and (L.pos is None or L.att is None or L.truth is None):
            time.sleep(0.2)
        time.sleep(10)                                                  # estimator settles (S1/S2: about 10 s)
        for k, v in PARAMS.items():
            L.send(L.m.mav.param_set_send, L.m.target_system, L.m.target_component, k.encode(), float(v),
                   mu.MAV_PARAM_TYPE_REAL32)
            time.sleep(0.05)
        x0n, y0e = L.pos[1], L.pos[2]
        lat0, lon0, alt0 = L.truth[1], L.truth[2], L.truth[3]          # truth origin: the vehicle on the ground at spawn
        R_E = 6371000.0

        def truth_local():
            tr = L.truth
            dn = math.radians(tr[1] - lat0) * R_E
            de = math.radians(tr[2] - lon0) * R_E * math.cos(math.radians(lat0))
            w, x, y, z = tr[4], tr[5], tr[6], tr[7]
            yaw_ned = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
            return de, dn, tr[3] - alt0, yaw_ned                        # ENU-local x (east), y (north), up, NED yaw

        def hold(seconds):
            n = int(round(seconds / DT))
            for _ in range(n):
                L.setpoint(TM_POS_XYZ, x0n, y0e, -ALT)
                time.sleep(DT)
        hold(2.0)
        L.cmd(mu.MAV_CMD_DO_SET_MODE, 1, 6)
        hold(1.0)
        L.cmd(mu.MAV_CMD_COMPONENT_ARM_DISARM, 1)
        hold(10.0)
        armed = bool(L.hb and (L.hb[0] & 128))
        offboard = bool(L.hb and ((L.hb[1] >> 16) & 0xFF) == 6)
        tx, ty, tz, _ = truth_local()
        res["start_check"] = {"armed": armed, "offboard": offboard, "truth_alt_m": tz, "ekf_alt_m": -L.pos[3],
                              "ekf_minus_truth_xy_m": float(math.hypot(L.pos[2] - tx, L.pos[1] - ty))}
        # PX4 holds the altitude in its estimator's frame; the truth altitude is recorded (SITL baro/GPS bias)
        if not (armed and offboard and abs(-L.pos[3] - ALT) < 0.3 and 0.3 < tz < 2.5):
            raise RuntimeError("HARNESS_INVALID start state %s" % res["start_check"])
        ep = B.Episode(centers, halves, quats)
        prev = np.zeros(4, np.float32)
        outcome, prev_rel = "timeout", None
        min_dist = float("inf")
        sim_start = L.pos[0]
        wall_start = time.time()
        for k in range(MAX_STEPS):
            while L.pos[0] < sim_start + k * DT:                        # pace on PX4 sim time
                time.sleep(0.002)
            pn = L.pos
            at = L.att
            gx, gy = pn[2], pn[1]                                       # ENU-local from NED (x east = NED y)
            wx, wy = rot(yaw0, gx, gy)
            p = np.array([p0[0] + wx, p0[1] + wy, -pn[3]])
            vwx, vwy = rot(yaw0, pn[5], pn[4])
            v = np.array([vwx, vwy, -pn[6]])
            yaw = yaw0 + (math.pi / 2 - at[3])
            vq = np.array([0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)])
            tgx, tgy, tgz, tyaw_ned = truth_local()                    # simulator truth: sensors and termination
            twx, twy = rot(yaw0, tgx, tgy)
            p_true = np.array([p0[0] + twx, p0[1] + twy, tgz])
            yaw_true = yaw0 + (math.pi / 2 - tyaw_ned)
            vq_true = np.array([0.0, 0.0, math.sin(yaw_true / 2), math.cos(yaw_true / 2)])
            tpos = np.asarray(ttraj[min(k, len(ttraj) - 1)], float)
            s = {"pos": p, "vel": v, "vquat": vq, "fquat": vq, "angvel": np.array([0.0, 0.0, -at[6]]), "tpos": tpos,
                 "tquat": None, "prev_action": prev, "bounds_min": bmin, "bounds_max": bmax,
                 "render_pos": p_true, "render_vquat": vq_true, "render_fquat": vq_true}
            c0 = time.time()
            obs, diag, lidar = ep.observe(s)
            act = policy.act(obs)[0].numpy()
            gov, wz, prev_t = ep.command(act, lidar)
            prev = prev_t.numpy()
            vwx2, vwy2 = rot(yaw, float(gov[0]), float(gov[1]))         # vehicle -> MOTAR world
            gex, gey = rot(-yaw0, vwx2, vwy2)                           # world -> Gazebo ENU-local
            L.setpoint(TM_POS_Z_VEL_XY_YAWRATE, 0, 0, -ALT, gey, gex, -wz)
            compute_ms = 1e3 * (time.time() - c0)
            rel = p_true - tpos
            dist = float(np.linalg.norm(rel))
            sweep = seg_min_dist(prev_rel, rel) if prev_rel is not None else dist
            min_dist = min(min_dist, sweep)
            prev_rel = rel
            hit, clr = box_hit(p_true, centers, halves)
            steps_log.append([k, pn[0], *p.tolist(), *v.tolist(), yaw, *act.tolist(), float(gov[0]), float(gov[1]), wz,
                              *tpos.tolist(), dist, clr, compute_ms, bool(diag["visible"][0]), *p_true.tolist(), yaw_true])
            if sweep < CAPTURE_M:
                outcome = "capture"
                break
            pt = p_true
            if hit or pt[2] < 0.1 or pt[2] > 4.0 or (pt[:2] < bmin[:2]).any() or (pt[:2] > bmax[:2]).any():
                outcome = "crash"
                res["crash_reason"] = "bar" if hit else ("altitude" if (pt[2] < 0.1 or pt[2] > 4.0) else "bounds")
                break
        res["real_time_factor"] = (L.pos[0] - sim_start) / max(1e-9, time.time() - wall_start)
        est_err = [math.hypot(r[2] - r[23], r[3] - r[24]) for r in steps_log]
        res["ekf_vs_truth_xy_err_m"] = {"median": float(np.median(est_err)), "max": float(max(est_err))} if est_err else None
        res.update(outcome=outcome, steps=len(steps_log), min_dist_m=min_dist,
                   compute_ms_median=float(np.median([r[-3] for r in steps_log])) if steps_log else None,
                   compute_ms_max=float(max(r[-3] for r in steps_log)) if steps_log else None)
        L.cmd(mu.MAV_CMD_DO_SET_MODE, 1, 4, 6)
        time.sleep(1)
    except Exception as e:                                               # noqa: BLE001
        res["outcome"] = "HARNESS_INVALID" if "HARNESS_INVALID" in str(e) else "ERROR"
        res["error"] = "%s: %s" % (e.__class__.__name__, e)
    finally:
        if L is not None:
            L.stop = True
        stop_sitl(proc)
        log.close()
        try:
            (WORLDS / (world + ".world")).unlink()
        except OSError:
            pass
    np.save(out_dir / ("ep%02d_steps.npy" % idx), np.array(steps_log, dtype=np.float64))
    return res


def episodes_from_trace(cell_dir, n, first_env, tree):
    import importlib.util
    spec = importlib.util.spec_from_file_location("ot_s3b", str(tree / "motar_eval/occlusion_trace.py"))
    OT = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(OT)
    tr = OT.read_trace(cell_dir)
    by = OT.episode_frames(tr["frames"])
    look = OT.reset_lookup(tr["layouts"])
    outcome = {}
    with open(cell_dir / "episodes.jsonl") as f:
        for line in f:
            e = json.loads(line)
            outcome[(e["env_index"], e["env_episode_index"])] = e["outcome"]
    receipt = json.loads((cell_dir / "receipt.json").read_text())
    arena = float(receipt["env_snapshot_navrl"].get("NAVRL_ARENA_XY", "40"))
    out = []
    for env in range(first_env, first_env + 1000):
        if len(out) >= n:
            break
        key = (env, 0)
        if key not in by or key not in look:
            continue
        fr = by[key]
        c, h, q = OT.layout_arrays(tr["layouts"], look[key])
        out.append({"env": env, "centers": np.asarray(c, float), "halves": np.asarray(h, float),
                    "quats": np.asarray(q, float), "p0": np.asarray(fr[0]["p_pos"], float),
                    "yaw0": yaw_of(np.asarray(fr[0]["p_vehicle_quat"], float)),
                    "tpos": np.asarray(fr["t_pos"], float), "isaac_outcome": outcome.get(key),
                    "isaac_steps": int(len(fr)), "bounds_min": [0.0, 0.0, 0.0], "bounds_max": [arena, arena, 3.0],
                    "receipt_env": receipt["env_snapshot_navrl"]})
    return out


def main():
    cell = pathlib.Path(sys.argv[1]).resolve()
    out_dir = pathlib.Path(sys.argv[2]).resolve()
    n = int(sys.argv[3])
    first_env = int(sys.argv[4]) if len(sys.argv) > 4 else 0
    out_dir.mkdir(parents=True, exist_ok=False)
    import sitl_bridge as B
    eps = episodes_from_trace(cell, n, first_env, B.ROOT)
    B.setup(eps[0]["receipt_env"])
    policy = B.Policy()
    report = {"unit": "S3b v0", "cell": str(cell), "params": PARAMS, "episodes": []}
    for i, spec in enumerate(eps):
        r = run_episode(B, spec, out_dir, i, policy)
        r["isaac_steps"] = spec["isaac_steps"]
        report["episodes"].append(r)
        print(json.dumps(r), flush=True)
        (out_dir / "s3b_report.json").write_text(json.dumps(report, indent=1, default=str) + "\n")


if __name__ == "__main__":
    main()
