"""C-Planner skeleton (M3, S8): predictive interception planner with the P0-R certificate as label (C-OFF) or as
execution filter (C-FILTER). One object per env and episode; pure Python/NumPy on CPU; never raises on values.

Interface (same inputs and action space as the B1-R classical arms, tools/b1r_hook.py Controller.act): own position,
velocity and yaw; the online tracker belief (position, velocity, active flag); the NORMALISED 72x4 LiDAR scan
(`depth_range_pixels`, before `_lidar_distance_m`). Output: normalised action [vx, vy, 0, yaw_rate] in the yaw-only
vehicle frame (x 2.5 m/s, x 3 rad/s), the altitude loop and governor stay in the task.

Per step:
1. The scan becomes a P0-R packet (cplanner/motar/adapter.py) and is ingested by the certifier core (evidence map).
2. No active belief: search exactly like the classical arms (no translation, yaw scan at 0.6 rad/s).
3. Intercept point: constant-velocity target, pursuer speed V_MAX, earliest tau in (0, TAU_MAX] with
   |r + v_t tau| = V_MAX tau; otherwise the target position at TAU_MAX.
4. Terminal: target within D_TERMINAL: pursue the target position predicted 0.1 s ahead (uncertified by design).
5. Candidates: headings at OFFSETS_DEG from the bearing to the intercept point, speeds SPEEDS. Each candidate's flown
   path over HORIZON_S is predicted with a first-order velocity response (time constant TAU_V) as a polyline.
6. Collision prediction (both modes): the obstacle points are the horizontal projections of every HIT return of all
   four rows (the classical arms' `lidar_points_xy`; B1-R bars are full height and the scene has no floor) plus a
   point at MIN_RANGE along every NEAR_CLIPPED beam, after excluding returns within EXCLUDE_M of the belief and, if
   given, the sensor-associated target-return mask (`last_target_like`, as the classical arms do). A candidate is
   admissible if its polyline keeps R_SAFE from every point; when the vehicle is already closer than R_SAFE, an
   escape candidate (never closer than now, and ending strictly farther) is admissible. Cost: time-to-intercept
   proxy plus a heading-change and a clearance term.
7. C-OFF executes the cheapest admissible candidate and records whether its polyline is STATIC_BAR_CERTIFIED.
   C-FILTER applies the same selection with one more condition: the executed polyline must be certified. Admissible
   candidates are certified lazily in cost order, at most MAX_FILTER_TRIES of them; if none is certified, it brakes
   (zero translation). C-FILTER - C-OFF is therefore the certificate's own effect. Neither mode claims safety:
   certificates have the P0-R scope and do not include tracking error (M4 adds it). The camera / target-like mask
   reaches only the planner's own collision prediction, never the certifier.
"""
import math
import pathlib
import sys

import numpy as np

_ROOT = pathlib.Path(__file__).resolve().parents[1]
for _p in (_ROOT / "src", _ROOT / "motar"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))
import adapter as AD     # noqa: E402
import core              # noqa: E402

V_MAX = 2.5
YAW_RATE_MAX = 3.0
YAW_GAIN = 2.0
SCAN_YAW_RATE = 0.6
TAU_MAX = 3.0
D_TERMINAL = 0.9
OFFSETS_DEG = (0.0, 15.0, -15.0, 30.0, -30.0, 50.0, -50.0, 75.0, -75.0, 100.0, -100.0, 180.0)
SPEEDS = (2.5, 1.25)
HORIZON_S = 1.0
N_POINTS = 5
TAU_V = 0.4
R_SAFE = 0.45
EXCLUDE_M = 0.5
MAX_FILTER_TRIES = 8
W_TURN = 0.3
W_CLEAR = 1.0
CLEAR_SOFT_M = 1.2
MODES = ("OFF", "FILTER")
ELEVATIONS = np.radians([20.0, 10.0, 0.0, -10.0])
AZIMUTHS = np.radians(180.0 - 355.0 * np.arange(72) / 71.0)


def intercept(p_r, p_t, v_t):
    r = np.asarray(p_t, float) - np.asarray(p_r, float)
    v = np.asarray(v_t, float)
    a = v @ v - V_MAX * V_MAX
    b = 2.0 * (r @ v)
    c = r @ r
    tau = None
    if abs(a) < 1e-12:
        if b < 0:
            tau = -c / b
    else:
        disc = b * b - 4 * a * c
        if disc >= 0:
            roots = sorted(((-b - math.sqrt(disc)) / (2 * a), (-b + math.sqrt(disc)) / (2 * a)))
            pos = [t for t in roots if t > 0]
            tau = pos[0] if pos else None
    if tau is None or tau > TAU_MAX:
        tau = TAU_MAX
    return np.asarray(p_t, float) + v * tau, tau


def flown_path(p0, v0, v_cmd):
    """Polyline (N_POINTS + 1, 2) of a first-order velocity response from v0 to v_cmd over HORIZON_S."""
    ts = np.linspace(0.0, HORIZON_S, N_POINTS + 1)
    e = np.exp(-ts / TAU_V)
    disp = np.outer(ts, v_cmd) + np.outer(TAU_V * (1.0 - e), np.asarray(v0) - np.asarray(v_cmd))
    return np.asarray(p0, float) + disp


def scan_points(norm, pos, yaw, target_like=None):
    """World XY obstacle points of a normalised (4, 72) scan: every HIT return of all rows projected horizontally,
    plus a point at MIN_RANGE along every NEAR_CLIPPED beam (value -1.0). Beams flagged in `target_like` are dropped."""
    v = np.asarray(norm, float).reshape(4, 72)
    finite = np.isfinite(v)
    hit = finite & (v > -1.0) & (v < 1.0)
    near = finite & ((v == -1.0) | (hit & (v * AD.MAX_RANGE < AD.MIN_RANGE)))
    r = np.where(near, AD.MIN_RANGE, v * AD.MAX_RANGE)
    keep = hit | near
    if target_like is not None:
        keep &= ~np.asarray(target_like, bool).reshape(4, 72)
    horiz = r * np.cos(ELEVATIONS)[:, None]
    az = AZIMUTHS[None, :] + yaw
    xs = pos[0] + horiz * np.cos(az)
    ys = pos[1] + horiz * np.sin(az)
    return np.stack([xs[keep], ys[keep]], axis=1)


def point_clearance(p, pts):
    if len(pts) == 0:
        return math.inf
    return float(np.sqrt(((pts - np.asarray(p, float)) ** 2).sum(1)).min())


def admissible(path, pts, clear_now):
    """(admissible, path clearance): keep R_SAFE, or, already inside R_SAFE, escape without getting closer."""
    clear = min_clearance(path, pts)
    if clear >= R_SAFE:
        return True, clear
    if clear_now < R_SAFE:
        return clear >= clear_now - 1e-9 and point_clearance(path[-1], pts) > clear_now + 1e-6, clear
    return False, clear


def min_clearance(path, pts):
    if len(pts) == 0:
        return math.inf
    a, b = path[:-1], path[1:]
    d = b - a
    den = np.maximum((d * d).sum(1), 1e-18)
    best = math.inf
    for i in range(len(a)):
        t = np.clip(((pts - a[i]) @ d[i]) / den[i], 0.0, 1.0)
        q = a[i] + t[:, None] * d[i]
        best = min(best, float(np.sqrt(((pts - q) ** 2).sum(1)).min()))
    return best


class CPlanner:
    def __init__(self, env_id, episode_id, mode="OFF", config="MOTAR-72x4-RUNTIME"):
        if mode not in MODES:
            raise ValueError("mode must be OFF or FILTER")
        self.mode = mode
        self.core = core.CertifierCore(config, int(env_id), int(episode_id))
        self.env, self.episode = int(env_id), int(episode_id)
        self.counts = {"steps": 0, "search": 0, "terminal": 0, "brake": 0, "executed_certified": 0,
                       "executed_uncertified": 0, "certify_calls": 0, "segments_certified_queries": 0,
                       "packet_refused": 0, "no_admissible": 0, "escape": 0, "filter_exhausted": 0}

    def _certified(self, path, step):
        segs = np.stack([path[:-1], path[1:]], axis=1).astype(np.float64)
        self.counts["certify_calls"] += 1
        self.counts["segments_certified_queries"] += len(segs)
        res = self.core.certify(segs, step)
        return all(r["static_bar_certified"] for r in res), [r["reason"] for r in res]

    def act(self, step, pos, vel, yaw, belief_pos, belief_vel, belief_active, norm_scan, target_like=None):
        """(action float64 [4] in [-1, 1], info dict)."""
        self.counts["steps"] += 1
        pos, vel = np.asarray(pos, float), np.asarray(vel, float)
        rep = self.core.step(*AD.packet(norm_scan, int(step), self.env, self.episode, pos, yaw))
        accepted = bool(rep.get("accepted"))
        for_step = rep["step"] if accepted else None
        if not accepted:
            self.counts["packet_refused"] += 1
        info = {"mode": self.mode, "packet": rep.get("reason", "ACCEPTED"), "certified": None, "terminal": False}
        act = np.zeros(4)
        if not belief_active:
            self.counts["search"] += 1
            act[3] = SCAN_YAW_RATE / YAW_RATE_MAX
            info["behaviour"] = "SEARCH"
            return act, info
        p_t, v_t = np.asarray(belief_pos, float)[:2], np.asarray(belief_vel, float)[:2]
        rel = p_t - pos[:2]
        bearing = math.atan2(rel[1], rel[0])
        yaw_err = math.atan2(math.sin(bearing - yaw), math.cos(bearing - yaw))
        act[3] = max(-1.0, min(1.0, YAW_GAIN * yaw_err / YAW_RATE_MAX))
        if float(np.hypot(*rel)) < D_TERMINAL:
            self.counts["terminal"] += 1
            aim = p_t + v_t * 0.1 - pos[:2]
            v_cmd = V_MAX * aim / max(float(np.hypot(*aim)), 1e-9)
            info.update(behaviour="TERMINAL", terminal=True, certified=False)
            return self._finish(act, v_cmd, yaw), info
        goal, tau = intercept(pos[:2], p_t, v_t)
        g_bear = math.atan2(goal[1] - pos[1], goal[0] - pos[0])
        pts = scan_points(norm_scan, pos, yaw, target_like)
        if len(pts):
            near_belief = np.hypot(pts[:, 0] - p_t[0], pts[:, 1] - p_t[1]) <= EXCLUDE_M
            pts = pts[~near_belief]
        clear_now = point_clearance(pos[:2], pts)
        cur_head = math.atan2(vel[1], vel[0]) if np.hypot(*vel[:2]) > 0.2 else g_bear
        cands = []
        for off in OFFSETS_DEG:
            h = g_bear + math.radians(off)
            for sp in SPEEDS:
                v_cmd = sp * np.array([math.cos(h), math.sin(h)])
                path = flown_path(pos[:2], vel[:2], v_cmd)
                ok, clear = admissible(path, pts, clear_now)
                turn = abs(math.atan2(math.sin(h - cur_head), math.cos(h - cur_head)))
                cost = (float(np.hypot(*(goal - path[-1]))) / V_MAX + W_TURN * turn
                        + W_CLEAR * max(0.0, CLEAR_SOFT_M - clear))
                cands.append((cost, ok, v_cmd, path, off, sp))
        cands.sort(key=lambda c: c[0])
        info.update(behaviour="NAVIGATE", tau=tau, clearance_now=clear_now)
        adm = [c for c in cands if c[1]]
        if not adm:
            self.counts["no_admissible"] += 1
            self.counts["brake"] += 1
            info.update(behaviour="BRAKE", certified=False)
            return self._finish(act, np.zeros(2), yaw), info
        if clear_now < R_SAFE:
            self.counts["escape"] += 1
            info["escape"] = True
        if self.mode == "OFF":
            c = adm[0]
            ok, reasons = self._certified(c[3], for_step) if accepted else (False, ["PACKET_REFUSED"])
            self.counts["executed_certified" if ok else "executed_uncertified"] += 1
            info.update(certified=ok, offset=c[4], speed=c[5], reasons=reasons[:1])
            return self._finish(act, c[2], yaw), info
        tried = 0
        for c in adm[:MAX_FILTER_TRIES]:                   # FILTER: same selection, certified lazily in cost order
            if not accepted:
                break
            tried += 1
            ok, reasons = self._certified(c[3], for_step)
            if ok:
                self.counts["executed_certified"] += 1
                info.update(certified=True, offset=c[4], speed=c[5], filter_tries=tried)
                return self._finish(act, c[2], yaw), info
        self.counts["filter_exhausted"] += 1
        self.counts["brake"] += 1
        info.update(behaviour="BRAKE", certified=False, filter_tries=tried)
        return self._finish(act, np.zeros(2), yaw), info

    @staticmethod
    def _finish(act, v_cmd_w, yaw):
        c, s = math.cos(yaw), math.sin(yaw)
        vx = c * v_cmd_w[0] + s * v_cmd_w[1]                # world -> yaw-only vehicle frame
        vy = -s * v_cmd_w[0] + c * v_cmd_w[1]
        n = max(1.0, math.hypot(vx, vy) / V_MAX)
        act[0], act[1] = vx / (V_MAX * n), vy / (V_MAX * n)
        return np.clip(act, -1.0, 1.0)
