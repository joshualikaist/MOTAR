"""SITL S1/S2 with pymavlink (vehicle from env SITL_MODEL, default iris) (no MAVSDK): PX4 offboard baseline with the default iris (SITL_VALIDATION_PLAN_v2.md S1).

Starts PX4 v1.14.3 SITL + Gazebo Classic headless (the built binary through sitl_run.sh), talks MAVLink on
udp 14540 (PX4's offboard link), and runs:
  A0  stream an offboard position setpoint (hold x, y; z = -1.5 m NED), switch to OFFBOARD, arm, climb (no failsafe)
  A2  hover 60 s: altitude RMS error, tilt RMS and max
  A3  body-frame velocity steps at 10 Hz, as MOTAR's policy commands them (vehicle frame x forward, y left):
      0 -> 1.0 -> 2.5 -> 0 m/s, 3 s each, in x and then in y. The command is sent as it will be in the bridge:
      body (x, y) rotated by the measured yaw into NED (vx, vy), altitude held by a z position setpoint (MOTAR's
      policy z output is overwritten by an altitude loop as well). Rise time 10-90 %, overshoot, 5 % settling,
      onset latency (first measured change > 0.05 m/s toward the command).
  A4  yaw-rate steps +3.0 / -3.0 rad/s, 3 s each
  A5  aggressive command (x = y = 2.5 m/s body, 3 s): max tilt, max horizontal speed
  A6  MAVLink TIMESYNC round trip (100 samples) and the 10 Hz setpoint loop's period
then switches to AUTO.LAND. Writes one JSON (refuses to overwrite) with the PX4 commit, the parameters that bound
these responses and the host fault count before and after.
Run under taskset -c 6,7,12,13 nice -n 10, CUDA_VISIBLE_DEVICES="", with the px4 conda env's python.
usage: s1_offboard_mavlink.py OUT_JSON
"""
import json
import math
import os
import pathlib
import signal
import statistics
import subprocess
import sys
import threading
import time

from pymavlink import mavutil

PX4 = pathlib.Path("/home/fair/workspaces/px4_ws/PX4-Autopilot")
BUILD = PX4 / "build/px4_sitl_default"
OUT = pathlib.Path(sys.argv[1])
MODEL = os.environ.get("SITL_MODEL", "iris")
DT = 0.1
ALT = float(os.environ.get("SITL_ALT", "1.0"))   # MOTAR altitude hold target is 1.0 m
SITL_PARAMS = json.loads(os.environ.get("SITL_PARAMS", "{}"))
PARAMS = ("MPC_XY_VEL_MAX", "MPC_Z_VEL_MAX_UP", "MPC_ACC_HOR_MAX", "MPC_ACC_HOR", "MPC_JERK_MAX", "MPC_TILTMAX_AIR",
          "MPC_XY_VEL_P_ACC", "MPC_XY_VEL_I_ACC", "MPC_XY_VEL_D_ACC", "MPC_XY_P", "MPC_Z_P", "MC_YAWRATE_MAX",
          "COM_OF_LOSS_T", "MPC_THR_HOVER")
TM_POS_Z_VEL_XY_YAWRATE = (1 | 2 | 32 | 64 | 128 | 256 | 1024)     # ignore x, y, vz, accel, yaw; use z, vx, vy, yaw_rate
TM_POS_XYZ = (8 | 16 | 32 | 64 | 128 | 256 | 1024 | 2048)            # use x, y, z only


def host_faults():
    j = subprocess.run(["journalctl", "-k", "-b", "0", "--no-pager"], capture_output=True, text=True).stdout
    return sum(1 for line in j.splitlines() if any(s in line for s in ("Machine check", "segfault", "general protection")))


def start_sitl(log):
    env = dict(os.environ, HEADLESS="1", NO_PXH="1", CUDA_VISIBLE_DEVICES="")
    cwd = BUILD / "src/modules/simulation/simulator_mavlink"
    cmd = [str(PX4 / "Tools/simulation/gazebo-classic/sitl_run.sh"), str(BUILD / "bin/px4"), "none", MODEL, os.environ.get("SITL_WORLD", "none"),
           str(PX4), str(BUILD)]
    return subprocess.Popen(cmd, cwd=str(cwd), env=env, stdout=log, stderr=subprocess.STDOUT, preexec_fn=os.setsid)


def stop_sitl(proc):
    try:
        os.killpg(proc.pid, signal.SIGINT)
        proc.wait(timeout=20)
    except Exception:                                            # noqa: BLE001
        try:
            os.killpg(proc.pid, signal.SIGKILL)
        except Exception:                                        # noqa: BLE001
            pass
    for pat in ("gzserver", "px4_sitl_default/bin/px4"):
        subprocess.run(["pkill", "-u", str(os.getuid()), "-f", "^[^ ]*" + pat])


class Link:
    def __init__(self):
        self.m = mavutil.mavlink_connection("udpin:0.0.0.0:14540", source_system=245)
        self.lock = threading.Lock()
        self.pos, self.att, self.ts, self.acks, self.params, self.text = [], [], [], [], {}, []
        self.clock = []   # (wall monotonic, sim time) pairs from LOCAL_POSITION_NED
        self.stop = False
        self.t_boot0 = None

    def rx(self):
        while not self.stop:
            msg = self.m.recv_match(blocking=True, timeout=0.5)
            if msg is None:
                continue
            t = time.monotonic()
            k = msg.get_type()
            if k == "LOCAL_POSITION_NED":
                self.pos.append((msg.time_boot_ms * 1e-3, msg.x, msg.y, msg.z, msg.vx, msg.vy, msg.vz))
                self.clock.append((t, msg.time_boot_ms * 1e-3))
            elif k == "ATTITUDE":
                self.att.append((msg.time_boot_ms * 1e-3, msg.roll, msg.pitch, msg.yaw, msg.yawspeed))
            elif k == "TIMESYNC" and msg.tc1 != 0:
                self.ts.append((t, msg.ts1))
            elif k == "COMMAND_ACK":
                self.acks.append((t, msg.command, msg.result))
            elif k == "PARAM_VALUE":
                self.params[msg.param_id] = msg.param_value
            elif k == "STATUSTEXT":
                self.text.append((t, msg.text))

    def send(self, fn, *a):
        with self.lock:
            fn(*a)

    def cmd(self, command, *p):
        p = list(p) + [0] * (7 - len(p))
        self.send(self.m.mav.command_long_send, self.m.target_system, self.m.target_component, command, 0, *p)

    def interval(self, msg_id, hz):
        self.cmd(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, msg_id, int(1e6 / hz))

    def setpoint(self, mask, x=0.0, y=0.0, z=0.0, vx=0.0, vy=0.0, yaw_rate=0.0):
        self.send(self.m.mav.set_position_target_local_ned_send, 0, self.m.target_system, self.m.target_component,
                  mavutil.mavlink.MAV_FRAME_LOCAL_NED, mask, x, y, z, vx, vy, 0, 0, 0, 0, 0, yaw_rate)

    def sim_now(self):
        return self.clock[-1][1] if self.clock else 0.0

    def latest(self, arr):
        return arr[-1] if arr else None


def body_to_ned(vx_b, vy_left, yaw):
    """Vehicle frame (x forward, y left; yaw-only) to NED velocity."""
    vy_right = -vy_left
    return vx_b * math.cos(yaw) - vy_right * math.sin(yaw), vx_b * math.sin(yaw) + vy_right * math.cos(yaw)


def ned_to_body(vn, ve, yaw):
    vx = vn * math.cos(yaw) + ve * math.sin(yaw)
    vy_right = -vn * math.sin(yaw) + ve * math.cos(yaw)
    return vx, -vy_right


def nearest(arr, t):
    return min(arr, key=lambda r: abs(r[0] - t))


def step_metrics(samples, t0, v0, v1, T):
    s = [(t - t0, v) for t, v in samples if t0 <= t <= t0 + T]
    if not s or abs(v1 - v0) < 1e-6:
        return {}
    dv = v1 - v0
    frac = [(t, (v - v0) / dv) for t, v in s]
    t10 = next((t for t, f in frac if f >= 0.1), None)
    t90 = next((t for t, f in frac if f >= 0.9), None)
    onset = next((t for t, v in s if (v - v0) * math.copysign(1, dv) > 0.05), None)
    settle = next((frac[i][0] for i in range(len(frac)) if all(abs(f - 1) <= 0.05 for _, f in frac[i:])), None)
    return {"rise_10_90_s": (t90 - t10) if (t10 is not None and t90 is not None) else None, "onset_latency_s": onset,
            "overshoot_pct": max(0.0, (max(f for _, f in frac) - 1) * 100), "settling_5pct_s": settle,
            "final_mean": statistics.mean(v for _, v in s[-5:]), "samples": len(s)}


def run(L, result):
    hb = L.m.wait_heartbeat(timeout=120)
    if hb is None:
        raise RuntimeError("no heartbeat")
    th = threading.Thread(target=L.rx, daemon=True)
    th.start()
    for mid in (mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE):
        L.interval(mid, 50)
    t0 = time.monotonic()
    while time.monotonic() - t0 < 90:                          # estimator ready: local position flowing for 10 s
        if L.clock and L.clock[-1][1] - L.clock[0][1] > 10 and time.monotonic() - L.clock[-1][0] < 0.5:
            break
        time.sleep(0.5)
    result["ready_s"] = time.monotonic() - t0
    for p in PARAMS:
        L.send(L.m.mav.param_request_read_send, L.m.target_system, L.m.target_component, p.encode(), -1)
        time.sleep(0.05)
    for k, v in SITL_PARAMS.items():
        L.send(L.m.mav.param_set_send, L.m.target_system, L.m.target_component, k.encode(), float(v),
               mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        time.sleep(0.1)
    time.sleep(1.0)
    result["param_overrides"] = SITL_PARAMS
    x0, y0 = L.pos[-1][1], L.pos[-1][2]
    sends = []

    def hold(seconds, fn):
        n = int(round(seconds / DT))
        t_next = time.monotonic()
        for _ in range(n):
            fn()
            sends.append(time.monotonic())
            t_next += DT
            time.sleep(max(0.0, t_next - time.monotonic()))

    def pos_sp():
        L.setpoint(TM_POS_XYZ, x0, y0, -ALT)

    def vel_sp(vx_b, vy_l, wz):
        def f():
            yaw = L.att[-1][3]
            vn, ve = body_to_ned(vx_b, vy_l, yaw)
            L.setpoint(TM_POS_Z_VEL_XY_YAWRATE, 0, 0, -ALT, vn, ve, wz)
        return f
    # A0: stream, offboard, arm, climb
    hold(2.0, pos_sp)
    L.cmd(mavutil.mavlink.MAV_CMD_DO_SET_MODE, 1, 6)            # custom mode, PX4 main mode OFFBOARD
    hold(1.0, pos_sp)
    L.cmd(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
    hold(10.0, pos_sp)
    z = -L.pos[-1][3]
    result["A0"] = {"altitude_after_climb_m": z, "acks": [(c, r) for _, c, r in L.acks],
                    "ok": abs(z - ALT) < 0.3, "statustext": [s for _, s in L.text][-10:]}
    if not result["A0"]["ok"]:
        return
    # A2 hover 60 s (velocity zero, z held)
    hold(3.0, vel_sp(0, 0, 0))
    ta = L.sim_now()
    hold(60.0, vel_sp(0, 0, 0))
    zz = [-r[3] for r in L.pos if r[0] >= ta]
    tilt = [math.degrees(math.hypot(r[1], r[2])) for r in L.att if r[0] >= ta]
    result["A2"] = {"alt_mean_m": statistics.mean(zz), "alt_rms_err_m": math.sqrt(statistics.mean((v - ALT) ** 2 for v in zz)),
                    "tilt_rms_deg": math.sqrt(statistics.mean(v * v for v in tilt)), "tilt_max_deg": max(tilt),
                    "samples": len(zz)}
    # A3 velocity steps
    a3 = {}
    for axis in ("x", "y"):
        seq = [0.0, 1.0, 2.5, 0.0]
        marks = []
        for i in range(1, len(seq)):
            ts = L.sim_now()
            marks.append((ts, seq[i - 1], seq[i]))
            hold(3.0, vel_sp(seq[i] if axis == "x" else 0, seq[i] if axis == "y" else 0, 0))
        hold(3.0, vel_sp(0, 0, 0))
        steps = []
        for ts, v0, v1 in marks:
            smp = []
            for r in L.pos:
                if ts <= r[0] <= ts + 3.0:
                    yaw = nearest(L.att, r[0])[3]
                    bx, by = ned_to_body(r[4], r[5], yaw)
                    smp.append((r[0], bx if axis == "x" else by))
            steps.append({"from": v0, "to": v1, **step_metrics(smp, ts, v0, v1, 3.0)})
        a3[axis] = steps
    result["A3"] = a3
    zz = [-r[3] for r in L.pos if r[0] >= marks[0][0]]
    result["A3_alt_range_m"] = [min(zz), max(zz)]
    # A4 yaw rate
    a4 = []
    for wz in (3.0, -3.0):
        ts = L.sim_now()
        hold(3.0, vel_sp(0, 0, wz))
        smp = [(r[0], r[4]) for r in L.att if ts <= r[0] <= ts + 3.0]
        a4.append({"cmd_rad_s": wz, "achieved_mean_last1s": statistics.mean(v for t, v in smp if t >= ts + 2.0),
                   **step_metrics(smp, ts, 0.0, wz, 3.0)})
        hold(3.0, vel_sp(0, 0, 0))
    result["A4"] = a4
    # A5 aggressive diagonal
    ts = L.sim_now()
    hold(3.0, vel_sp(2.5, 2.5, 0))
    tilt = [math.degrees(math.hypot(r[1], r[2])) for r in L.att if ts <= r[0] <= ts + 3.0]
    hs = [(r[0], math.hypot(r[4], r[5])) for r in L.pos if ts <= r[0] <= ts + 3.0]
    result["A5"] = {"cmd_body": [2.5, 2.5], "cmd_norm_m_s": math.hypot(2.5, 2.5), "tilt_max_deg": max(tilt),
                    "hspeed_max_m_s": max(h for _, h in hs),
                    "hspeed_last1s_m_s": statistics.mean(h for t, h in hs if t >= ts + 2.0)}
    hold(4.0, vel_sp(0, 0, 0))
    # A6 TIMESYNC round trip, setpoints keep streaming
    rtt = []
    for _ in range(100):
        tsent = time.monotonic_ns()
        n0 = len(L.ts)
        L.send(L.m.mav.timesync_send, 0, tsent)
        vel_sp(0, 0, 0)()
        t_end = time.monotonic() + 0.1
        while time.monotonic() < t_end:
            got = [x for x in L.ts[n0:] if x[1] == tsent]
            if got:
                rtt.append(got[0][0] - tsent / 1e9)
                break
            time.sleep(0.001)
        time.sleep(max(0.0, t_end - time.monotonic()))
    per = [b - a for a, b in zip(sends, sends[1:])]
    onsets = [s["onset_latency_s"] for ax in a3.values() for s in ax if s.get("onset_latency_s") is not None]
    result["A6"] = {"timesync_rtt_n": len(rtt), "timesync_rtt_median_ms": 1e3 * statistics.median(rtt) if rtt else None,
                    "timesync_rtt_p95_ms": 1e3 * sorted(rtt)[int(0.95 * (len(rtt) - 1))] if rtt else None,
                    "setpoint_period_median_s": statistics.median(per),
                    "setpoint_period_p95_s": sorted(per)[int(0.95 * (len(per) - 1))], "setpoint_period_max_s": max(per),
                    "step_onset_latency_median_s": statistics.median(onsets) if onsets else None}
    L.cmd(mavutil.mavlink.MAV_CMD_DO_SET_MODE, 1, 4, 6)       # AUTO.LAND
    time.sleep(8)
    for p in list(SITL_PARAMS):
        L.send(L.m.mav.param_request_read_send, L.m.target_system, L.m.target_component, p.encode(), -1)
        time.sleep(0.05)
    time.sleep(1.0)
    result["params"] = {k: v for k, v in L.params.items()}
    c = L.clock
    result["real_time_factor"] = (c[-1][1] - c[0][1]) / max(1e-9, c[-1][0] - c[0][0]) if len(c) > 1 else None
    result["metrics_time_base"] = "PX4 time_boot_ms (sim time)"
    result["telemetry_counts"] = {"pos": len(L.pos), "att": len(L.att)}


def main():
    if OUT.exists():
        raise SystemExit("refusing to overwrite " + str(OUT))
    commit = subprocess.run(["git", "-C", str(PX4), "rev-parse", "HEAD"], capture_output=True, text=True).stdout.strip()
    result = {"stage": "S1", "client": "pymavlink", "vehicle": MODEL + (" (PX4 default parameters)" if MODEL == "iris" else " (airframe 10090, MOTAR training spec)"), "px4_commit": commit,
              "host_faults_before": host_faults(), "affinity": sorted(os.sched_getaffinity(0)), "started": time.time()}
    log = open(str(OUT) + ".sitl.log", "w")
    proc = start_sitl(log)
    L = None
    try:
        time.sleep(3)
        L = Link()
        run(L, result)
        result["status"] = "COMPLETE" if "A6" in result else "INCOMPLETE"
    except Exception as e:                                       # noqa: BLE001
        result["status"] = "ERROR"
        result["error"] = "%s: %s" % (e.__class__.__name__, e)
    finally:
        if L is not None:
            L.stop = True
        stop_sitl(proc)
        log.close()
    result["host_faults_after"] = host_faults()
    result["finished"] = time.time()
    if L is not None and os.environ.get("SITL_DUMP") == "1":
        import numpy as np
        np.savez_compressed(str(OUT) + ".raw.npz", pos=np.array(L.pos), att=np.array(L.att), clock=np.array(L.clock))
    OUT.write_text(json.dumps(result, indent=1, default=str) + "\n")
    print(json.dumps({k: result.get(k) for k in ("status", "error", "ready_s", "A0", "A2", "A5", "A6")}, indent=1, default=str))


if __name__ == "__main__":
    main()
