"""dev08 policy/command step (amendment1 runtime-stack row "Policy/command step"; motar-5md).

One observed-prefix checkpoint k, given a checked telemetry snapshot (tools/sitl_dev08_link.TelemetryGate):
  1. the state the historical runner built (tools/sitl_s3b_closed_loop.py run_episode, the 10 Hz loop body): PX4's
     estimate for the policy's pose, the simulator truth for the rendered sensors and the termination, the recorded
     target position target[k] (no clamp: k must be inside the admitted prefix);
  2. the unchanged physical terminal predicates of that runner (swept 3-D capture < 0.5 m before the bar footprint,
     altitude and arena crash), evaluated on this state;
  3. tools/sitl_dev08_prefix_contract.checkpoint_decision: capture > crash > CENSORED_TRACE_END on the last admitted
     index > CONTINUE;
  4. on CONTINUE only, the historical observation -> Policy F -> governor -> PX4 velocity setpoint (truth command
     yaw). A terminal or censor checkpoint issues no command.
The historical runner sends the command of a step before testing that step's state, so its last command (at the
terminal step) has no counterpart here; up to that step the two are the same (tests/test_sitl_dev08_step.py runs
the historical loop and this step on the same streams). The geometry and constants are the historical runner's own
functions, loaded from the pinned file. Arms differ only in PX4 parameters, never here.

Library only: importing it loads nothing. Not verified live.
"""
import hashlib
import importlib.util
import math
import pathlib

STATUS = {"component": "policy_command_step", "contract": "amendment1 runtime-stack: Policy/command step",
          "live_verified": False, "execution_supported": False, "launch_ready": False}

ROOT = pathlib.Path(__file__).resolve().parents[1]
RUNNER = ROOT / "tools/sitl_s3b_closed_loop.py"
RUNNER_SHA = "d170b0271f73822280d8c92af68ab2bb56e350bbdf598702c353d8b79f0284b4"   # tools/sitl_s3b_dev08.py PINS
PREFIX = ROOT / "tools/sitl_dev08_prefix_contract.py"
R_E = 6371000.0
_MODS = {}


def _load(name, path, sha=None):
    if name not in _MODS:
        data = path.read_bytes()
        if sha is not None and hashlib.sha256(data).hexdigest() != sha:
            raise RuntimeError("pinned source changed: %s" % path)
        import sys
        saved = list(sys.path)                    # the historical runner extends sys.path at import
        try:
            spec = importlib.util.spec_from_file_location(name, str(path))
            mod = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(mod)
        finally:
            sys.path[:] = saved
        _MODS[name] = mod
    return _MODS[name]


def historical():
    return _load("dev08_historical_runner", RUNNER, RUNNER_SHA)


def prefix_contract():
    return _load("dev08_prefix_contract", PREFIX)


# The historical Link.rx conversions of the three MAVLink messages.
def pos_tuple(f):
    return (f["time_boot_ms"] * 1e-3, f["x"], f["y"], f["z"], f["vx"], f["vy"], f["vz"])


def att_tuple(f):
    return (f["time_boot_ms"] * 1e-3, f["roll"], f["pitch"], f["yaw"], f["rollspeed"], f["pitchspeed"], f["yawspeed"])


def truth_tuple(f, wall=0.0):
    q = f["attitude_quaternion"]
    return (wall, f["lat"] * 1e-7, f["lon"] * 1e-7, f["alt"] * 1e-3, q[0], q[1], q[2], q[3], f["vx"] * 1e-2,
            f["vy"] * 1e-2, f["vz"] * 1e-2)


class PrefixEpisode:
    """Holds one episode's bridge state across checkpoints. B: tools/sitl_bridge (set up), policy: B.Policy()."""

    def __init__(self, B, policy, spec, recorded_frames, truth_origin):
        import numpy as np
        self.np = np
        self.H = historical()
        self.P = prefix_contract()
        self.window = self.P.frame_window(recorded_frames)
        self.recorded_frames = recorded_frames
        self.spec = spec
        self.centers, self.halves, self.quats = spec["centers"], spec["halves"], spec["quats"]
        self.p0, self.yaw0, self.ttraj = spec["p0"], spec["yaw0"], spec["tpos"]
        if len(self.ttraj) < recorded_frames:
            raise ValueError("target trajectory shorter than the admitted prefix")
        self.bmin = np.asarray(spec["bounds_min"], float)
        self.bmax = np.asarray(spec["bounds_max"], float)
        self.lat0, self.lon0, self.alt0 = truth_origin
        self.ep = B.Episode(self.centers, self.halves, self.quats)
        self.policy = policy
        self.prev = np.zeros(4, np.float32)
        self.prev_rel = None
        self.min_dist = float("inf")
        self.next_k = 0
        self.done = None
        self.commands = []

    def truth_local(self, tr):
        dn = math.radians(tr[1] - self.lat0) * R_E
        de = math.radians(tr[2] - self.lon0) * R_E * math.cos(math.radians(self.lat0))
        w, x, y, z = tr[4], tr[5], tr[6], tr[7]
        yaw_ned = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        return de, dn, tr[3] - self.alt0, yaw_ned

    def state(self, k, pn, at, tr):
        np, rot = self.np, self.H.rot
        gx, gy = pn[2], pn[1]
        wx, wy = rot(self.yaw0, gx, gy)
        p = np.array([self.p0[0] + wx, self.p0[1] + wy, -pn[3]])
        vwx, vwy = rot(self.yaw0, pn[5], pn[4])
        v = np.array([vwx, vwy, -pn[6]])
        yaw = self.yaw0 + (math.pi / 2 - at[3])
        vq = np.array([0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)])
        tgx, tgy, tgz, tyaw_ned = self.truth_local(tr)
        twx, twy = rot(self.yaw0, tgx, tgy)
        p_true = np.array([self.p0[0] + twx, self.p0[1] + twy, tgz])
        yaw_true = self.yaw0 + (math.pi / 2 - tyaw_ned)
        vq_true = np.array([0.0, 0.0, math.sin(yaw_true / 2), math.cos(yaw_true / 2)])
        tpos = np.asarray(self.ttraj[k], float)
        s = {"pos": p, "vel": v, "vquat": vq, "fquat": vq, "angvel": np.array([0.0, 0.0, -at[6]]), "tpos": tpos,
             "tquat": None, "prev_action": self.prev, "bounds_min": self.bmin, "bounds_max": self.bmax,
             "render_pos": p_true, "render_vquat": vq_true, "render_fquat": vq_true}
        return {"s": s, "p": p, "v": v, "yaw": yaw, "p_true": p_true, "yaw_true": yaw_true, "tpos": tpos}

    def step(self, k, snapshot):
        """snapshot: TelemetryGate.checkpoint(k). Returns the checkpoint record; 'command' is the setpoint tuple
        (mask, x, y, z, vx, vy, yaw_rate) sent to PX4 on CONTINUE, else None."""
        if self.done is not None:
            raise RuntimeError("episode already ended: %s" % self.done)
        if k != self.next_k or snapshot["k"] != k:
            raise ValueError("checkpoint %d out of order (expected %d)" % (k, self.next_k))
        if not 0 <= k <= self.window["last_frame_index"]:
            raise ValueError("checkpoint %d outside the admitted prefix" % k)
        H, np = self.H, self.np
        pn = pos_tuple(snapshot["pos"]["fields"])
        at = att_tuple(snapshot["att"]["fields"])
        tr = truth_tuple(snapshot["truth"]["fields"])
        st = self.state(k, pn, at, tr)
        rel = st["p_true"] - st["tpos"]
        dist = float(np.linalg.norm(rel))
        sweep = H.seg_min_dist(self.prev_rel, rel) if self.prev_rel is not None else dist
        self.min_dist = min(self.min_dist, sweep)
        self.prev_rel = rel
        pt = st["p_true"]
        hit, clr = H.box_hit(pt, self.centers, self.halves)
        oob = bool((pt[:2] < self.bmin[:2] - H.OOB_MARGIN).any() or (pt[:2] > self.bmax[:2] + H.OOB_MARGIN).any())
        alt_bad = bool(pt[2] < 0.1 or pt[2] > 4.0)
        capture = bool(sweep < H.CAPTURE_M)
        crash = bool(hit or alt_bad or oob)
        decision = self.P.checkpoint_decision(k, self.recorded_frames, capture=capture, crash=crash)
        rec = {"k": k, "decision": decision, "sim_s": pn[0], "scheduled_sim_s": snapshot["scheduled_sim_s"],
               "dist_m": dist, "sweep_m": sweep, "clearance_m": clr, "p": st["p"].tolist(),
               "p_true": pt.tolist(), "yaw": st["yaw"], "yaw_true": st["yaw_true"], "tpos": st["tpos"].tolist(),
               "command": None}
        if decision == "crash":
            rec["crash_reason"] = "bar" if hit else ("altitude" if alt_bad else "bounds")
        if decision != "CONTINUE":
            self.done = decision
            self.next_k = k + 1
            return rec
        obs, diag, lidar = self.ep.observe(st["s"])
        act = self.policy.act(obs)[0].numpy()
        gov, wz, prev_t = self.ep.command(act, lidar)
        self.prev = prev_t.numpy()
        vwx2, vwy2 = H.rot(st["yaw_true"], float(gov[0]), float(gov[1]))      # truth command yaw (dev07/dev08)
        gex, gey = H.rot(-self.yaw0, vwx2, vwy2)
        command = (H.TM_POS_Z_VEL_XY_YAWRATE, 0, 0, -H.ALT, gey, gex, -wz)
        rec.update(command=command, action=[float(a) for a in act], governed=[float(gov[0]), float(gov[1])],
                   yaw_rate=float(wz), visible=bool(diag["visible"][0]))
        self.commands.append(command)
        self.next_k = k + 1
        return rec

    def counts(self):
        """final_frame_index, observation_checks, commands_issued (amendment1: k, k+1, k at a valid end)."""
        return {"final_frame_index": self.next_k - 1, "observation_checks": self.next_k,
                "commands_issued": len(self.commands), "outcome": self.done, "min_dist_m": self.min_dist}


if __name__ == "__main__":
    import json
    print(json.dumps(STATUS))
