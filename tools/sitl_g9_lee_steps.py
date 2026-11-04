"""G9: MOTAR's own velocity response in Isaac Gym, the reference for the PX4 SITL tests S1/S2
(docs/plans/prereg_sitl_reference_g8g9_v1.md; SITL plan stage S1/S2 test A3/A4).

Builds Policy F's simulation exactly as the NavRL task does (sim `base_sim`, env `navrl_bars_env` with 10 physics
steps per 0.1 s action, robot `navrl_quad`, controller `lee_velocity_control_navrl`, gains at their midpoint, no
randomization), with one bar placed far from the test area. No policy and no governor: the controller gets the
same vehicle-frame command the task would send, held for 10 physics steps, with the task's altitude PI
(e = 1.0 - z, I += 0.1 e clamped to +/-2.5, vz = clamp(4 e + I, +/-2.5), per 0.1 s; navrl_task.py 4979-5002).

Sequence per env (as the SITL test A3/A4, 10 Hz commands, vehicle frame x forward, y left):
  hover 3 s; x: 0 -> 1.0 -> 2.5 -> 0 m/s, 3 s each; hover 3 s; y: the same; hover 3 s; yaw rate +3 then -3 rad/s,
  3 s each with 3 s hover after each; diagonal (2.5, 2.5) for 3 s.
Records the body-frame velocity and yaw rate at every physics step (0.01 s) and reports per step: 10-90 % rise,
onset latency (> 0.05 m/s toward the command), overshoot, 5 % settling, final mean; plus the diagonal's steady speed
and max tilt. Writes one JSON (refuses to overwrite).
usage: tools/sitl_g9_lee_steps.py OUT_JSON   (needs the GPU; run with PYTHONPATH = this tree)
"""
import json
import math
import os
import pathlib
import sys

OUT = pathlib.Path(sys.argv[1])
if OUT.exists():
    raise SystemExit("refusing to overwrite " + str(OUT))
# Policy F's contract values that shape the dynamics (receipts of the C2 cells)
os.environ.setdefault("NAVRL_MAX_VELOCITY", "2.5")
os.environ.setdefault("NAVRL_YAW_RATE_MAX", "3.0")
os.environ.setdefault("NAVRL_MAX_TILT_DEG", "45")
os.environ.setdefault("NAVRL_ARENA_XY", "40")
os.environ.setdefault("NAVRL_ARENA_Z", "3")
os.environ.setdefault("NAVRL_NUM_BARS", "1")
os.environ.setdefault("NAVRL_BAR_POOL", "bars_h3")
for k in ("NAVRL_GAIN_RAND", "NAVRL_GAIN_RAND_SCALE", "NAVRL_GAIN_RAND_SEED"):
    os.environ.pop(k, None)

from isaacgym import gymapi  # noqa: E402,F401  (Isaac Gym before torch)
import torch  # noqa: E402

import subprocess  # noqa: E402

import aerial_gym  # noqa: E402
from aerial_gym.sim.sim_builder import SimBuilder  # noqa: E402

TREE = pathlib.Path(__file__).resolve().parents[1]
if not str(pathlib.Path(aerial_gym.__file__).resolve()).startswith(str(TREE) + os.sep):
    raise SystemExit("aerial_gym resolves outside this tree: %s (set PYTHONPATH=%s)" % (aerial_gym.__file__, TREE))


def host_faults():
    j = subprocess.run(["journalctl", "-k", "-b", "0", "--no-pager"], capture_output=True, text=True).stdout
    return sum(1 for line in j.splitlines() if any(k in line for k in ("Machine check", "segfault", "general protection")))

DT_ACTION = 0.1
ALT = 1.0
NUM_ENVS = 4


def yaw_of(q):
    x, y, z, w = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    return torch.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def tilt_deg(q):
    x, y, z, w = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    # angle between body z and world z
    bz = 1 - 2 * (x * x + y * y)
    return torch.rad2deg(torch.arccos(bz.clamp(-1, 1)))


def schedule():
    seq = [("hover", 0, 0, 0, 3.0)]
    for ax in ("x", "y"):
        for v in (1.0, 2.5, 0.0):
            seq.append(("step_" + ax, v if ax == "x" else 0.0, v if ax == "y" else 0.0, 0.0, 3.0))
        seq.append(("hover", 0, 0, 0, 3.0))
    for wz in (3.0, -3.0):
        seq.append(("yaw", 0, 0, wz, 3.0))
        seq.append(("hover", 0, 0, 0, 3.0))
    seq.append(("diag", 2.5, 2.5, 0.0, 3.0))
    return seq


def metrics(ts, vs, t0, v0, v1, T):
    s = [(t - t0, v) for t, v in zip(ts, vs) if t0 <= t <= t0 + T]
    if not s or abs(v1 - v0) < 1e-9:
        return {}
    dv = v1 - v0
    fr = [(t, (v - v0) / dv) for t, v in s]
    t10 = next((t for t, f in fr if f >= 0.1), None)
    t90 = next((t for t, f in fr if f >= 0.9), None)
    onset = next((t for t, v in s if (v - v0) * math.copysign(1, dv) > 0.05), None)
    settle = next((fr[i][0] for i in range(len(fr)) if all(abs(f - 1) <= 0.05 for _, f in fr[i:])), None)
    return {"rise_10_90_s": (t90 - t10) if (t10 is not None and t90 is not None) else None, "onset_latency_s": onset,
            "overshoot_pct": max(0.0, (max(f for _, f in fr) - 1) * 100), "settling_5pct_s": settle,
            "final_mean": sum(v for _, v in s[-50:]) / len(s[-50:])}


def main():
    faults0 = host_faults()
    git = lambda *a: subprocess.run(["git", "-C", str(TREE), *a], capture_output=True, text=True).stdout.strip()  # noqa: E731
    head, dirty = git("rev-parse", "HEAD"), bool(git("status", "--porcelain"))
    em = SimBuilder().build_env(sim_name="base_sim", env_name="navrl_bars_env", robot_name="navrl_quad",
                                controller_name="lee_velocity_control_navrl", args=None, device="cuda:0",
                                num_envs=NUM_ENVS, headless=True, use_warp=True)
    td = em.global_tensor_dict
    em.reset()
    phys_dt = float(em.sim_config.sim.dt)
    n_sub = int(round(DT_ACTION / phys_dt))
    cmd = torch.zeros((NUM_ENVS, 4), device="cuda:0")
    integ = torch.zeros(NUM_ENVS, device="cuda:0")
    rec = {"t": [], "vx": [], "vy": [], "wz": [], "z": [], "tilt": [], "speed": []}   # per env: lists of [NUM_ENVS]
    t = 0.0
    marks = []
    for name, vx, vy, wz, dur in schedule():
        marks.append((name, t, vx, vy, wz, dur))
        for _ in range(int(round(dur / DT_ACTION))):
            z = td["robot_position"][:, 2]
            e = ALT - z
            integ = (integ + e * DT_ACTION).clamp(-2.5, 2.5)
            vz = (4.0 * e + integ).clamp(-2.5, 2.5)
            cmd[:, 0], cmd[:, 1], cmd[:, 2], cmd[:, 3] = vx, vy, vz, wz
            for _ in range(n_sub):
                em.simulate(cmd, None)
                em.compute_observations()
                t += phys_dt
                q = td["robot_vehicle_orientation"]
                yaw = yaw_of(q)
                v = td["robot_linvel"]
                bx = v[:, 0] * torch.cos(yaw) + v[:, 1] * torch.sin(yaw)
                by = -v[:, 0] * torch.sin(yaw) + v[:, 1] * torch.cos(yaw)
                rec["t"].append(t)
                rec["vx"].append(bx.detach().cpu().tolist())
                rec["vy"].append(by.detach().cpu().tolist())
                rec["wz"].append(td["robot_body_angvel"][:, 2].detach().cpu().tolist())
                rec["z"].append(td["robot_position"][:, 2].detach().cpu().tolist())
                rec["tilt"].append(tilt_deg(td["robot_orientation"]).detach().cpu().tolist())
                rec["speed"].append(torch.hypot(v[:, 0], v[:, 1]).detach().cpu().tolist())
    def col(key, i):
        return [row[i] for row in rec[key]]

    def med(dicts):
        keys = set().union(*dicts) if dicts else set()
        res = {}
        for k in keys:
            vals = sorted(d[k] for d in dicts if d.get(k) is not None)
            res[k] = vals[len(vals) // 2] if vals else None
        res["envs"] = len(dicts)
        return res

    out = {"unit": "G9", "commit": head, "dirty": dirty, "aerial_gym_origin": str(aerial_gym.__file__),
           "host_faults_before": faults0, "phys_dt_s": phys_dt, "substeps_per_action": n_sub, "num_envs": NUM_ENVS,
           "controller": "lee_velocity_control_navrl", "robot": "navrl_quad", "alt_target_m": ALT, "A3": {}, "A4": []}
    prev = {"x": 0.0, "y": 0.0}
    for name, t0, vx, vy, wz, dur in marks:
        if name.startswith("step_"):
            ax = name[-1]
            v1 = vx if ax == "x" else vy
            per_env = [metrics(rec["t"], col("vx" if ax == "x" else "vy", i), t0, prev[ax], v1, dur)
                       for i in range(NUM_ENVS)]
            out["A3"].setdefault(ax, []).append({"from": prev[ax], "to": v1, "median_over_envs": med(per_env),
                                                 "per_env": per_env})
            prev[ax] = v1
        elif name == "yaw":
            per_env = [metrics(rec["t"], col("wz", i), t0, 0.0, wz, dur) for i in range(NUM_ENVS)]
            out["A4"].append({"cmd_rad_s": wz, "median_over_envs": med(per_env), "per_env": per_env})
        elif name == "diag":
            sel = [i for i, tt in enumerate(rec["t"]) if t0 <= tt <= t0 + dur]
            out["A5"] = {"cmd_body": [vx, vy],
                         "hspeed_last1s_m_s_per_env": [sum(rec["speed"][j][i] for j in sel[-100:]) / 100 for i in range(NUM_ENVS)],
                         "tilt_max_deg_per_env": [max(rec["tilt"][j][i] for j in sel) for i in range(NUM_ENVS)]}
    hov = [i for i, tt in enumerate(rec["t"]) if 1.0 <= tt <= 3.0]
    out["hover_alt_mean_m_per_env"] = [sum(rec["z"][j][i] for j in hov) / len(hov) for i in range(NUM_ENVS)]
    out["host_faults_after"] = host_faults()
    OUT.write_text(json.dumps(out, indent=1) + "\n")
    print(json.dumps({"A3": {ax: [(st["from"], st["to"], st["median_over_envs"]) for st in v] for ax, v in out["A3"].items()},
                      "A4": [(a["cmd_rad_s"], a["median_over_envs"]) for a in out["A4"]], "A5": out["A5"]}, indent=1))


if __name__ == "__main__":
    main()
