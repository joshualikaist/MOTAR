"""P0-R certifier core: one owned object per env and episode; numeric arrays in, plain results out.

    core = CertifierCore("SYNTH-72x1", env_id=0, episode_id=0)
    report = core.step(ranges, status, header_i, header_f, camera=None, track=None)
    certs = core.certify(paths, for_step=report["step"])

`step` validates one acquisition packet (boundary.exact_copy), ingests it into the core's own evidence map and makes
it the CURRENT packet. `certify` evaluates world-frame straight segments against the current packet only and returns
one scoped result per segment; there is no SAFE flag. Nothing either function receives is a Python object the core
later trusts, and neither raises on an argument value (R-4): errors become refusals.

Certificate (STATIC_BAR_CERTIFIED) for segment s and packet P iff:
  1. P is complete (every manifest beam validated as HIT or NO_HIT; R-1 fail-closed);
  2. `coverage_b._coverage_B` (verbatim, AST-identical to v3-v7) certifies s with P's free rays and P's HIT points as
     surfaces (every HIT, whatever the camera says) for the catalog shapes at plan radius R; a configuration with
     coverage NATIVE_EXACT calls its exact C port instead (`coverage_native`, M3), which must return identical
     results; a missing native library refuses (NATIVE_COVERAGE_UNAVAILABLE);
  3. the evidence map has no veto cell near s (veto only);
  4. if the config declares an arena, the swept disc of s stays inside it;
  5. the config's range_error_bound is 0 (R-6).
The camera and label layer are not arguments of anything on this path.
"""
import collections
import functools
import math
import time
import warnings

import numpy as np

import boundary as B
import coverage_b as CB
import coverage_native as CN
import registry as REG
from evidence_map import EvidenceMap, WIDTH, TRACK_FRESH
from labels import TargetLabelLayer

CORE_VERSION = "p0r-core-r1"
TIME_BASIS = "PACKET_ACQUISITION_STEP"          # no caller clock; a certificate is valid for its packet's step
HEALTH_BASIS = "DECLARED_NOT_AUTHENTICATED"
K_MAX = 64
COORD_LIMIT = 1.0e4
ID_LIMIT = 2 ** 53

# raw status codes (declared by the adapter) and validated per-beam outcomes
NOT_DELIVERED, HIT, NO_HIT, SENSOR_ERROR, DROPOUT, NEAR_CLIPPED, DISTANCE_ONLY = 0, 1, 2, 3, 4, 5, 6
V_HIT, V_NO_HIT, V_MISSING, V_INVALID, V_AMBIGUOUS = 1, 2, 10, 11, 12
OUTCOME_NAMES = {V_HIT: "HIT", V_NO_HIT: "NO_HIT", V_MISSING: "MISSING", V_INVALID: "INVALID",
                 V_AMBIGUOUS: "AMBIGUOUS"}
HEALTH_UNKNOWN, HEALTH_RUNTIME, HEALTH_SYNTHETIC = 0, 1, 2

SCOPE = {
    "static_catalog_bars": "CERTIFIED_UNDER_DECLARED_PRIOR",
    "dynamic_objects": "UNVERIFIED",
    "non_catalog_objects": "UNVERIFIED",
    "sensor_provenance": HEALTH_BASIS,
    "pose_error": "UNVERIFIED",
    "tracking_error": "UNVERIFIED",
    "terminal_approach": "UNCERTIFIED",
    "timing": "UNVERIFIED",
}


_USE_CURRENT = object()


def _rz(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


class CertifierCore:
    def __init__(self, config_name, env_id, episode_id):
        # construction errors are programming errors and raise (v6 table); nothing here is per-step input
        if type(config_name) is not str or config_name not in REG.CONFIGS:
            raise ValueError("unregistered configuration")
        if not (type(env_id) is int and type(episode_id) is int and 0 <= env_id < ID_LIMIT
                and 0 <= episode_id < ID_LIMIT):
            raise ValueError("env and episode ids must be non-negative ints")
        self.cfg = REG.CONFIGS[config_name]
        self.layout = REG.LAYOUTS[self.cfg.layout]
        self.shape = (len(self.layout.rows_deg), self.layout.n_az)
        self.dirs = REG.directions(self.cfg.layout)
        self.env_id, self.episode_id = env_id, episode_id
        self.map = EvidenceMap(self.cfg.map_bounds)
        self._labels = TargetLabelLayer(self.cfg.map_bounds, self.cfg.max_range)
        self._current = None        # the current packet's snapshot (a tuple; see _step)
        self._last_step = None
        self.exceptions = collections.deque(maxlen=1000)

    # ------------------------------------------------------------ step
    def step(self, ranges, status, header_i, header_f, camera=None, track=None):
        with np.errstate(all="ignore"), warnings.catch_warnings():
            warnings.simplefilter("ignore")
            try:
                return self._step(ranges, status, header_i, header_f, camera, track)
            except Exception as e:      # reachable only before the map mutation (plan/apply split)
                self.exceptions.append(("step", type(e).__name__, str(e)[:200]))
                self._current = None
                return {"accepted": False, "reason": "INTERNAL_ERROR_" + type(e).__name__}

    def _refuse(self, reason):
        self._current = None        # R-5: a refused step leaves no current packet
        return {"accepted": False, "reason": reason}

    def _step(self, ranges, status, header_i, header_f, camera, track):
        r = B.exact_copy(ranges, B.F64, self.shape)
        s = B.exact_copy(status, B.I8, self.shape)
        hi = B.exact_copy(header_i, B.I64, (5,))
        hf = B.exact_copy(header_f, B.F64, (5,))
        for name, v in (("ranges", r), ("status", s), ("header_i", hi), ("header_f", hf)):
            if v is None:
                return self._refuse("MALFORMED_ARRAY_" + name)
        env, ep, step, health, noise = (int(x) for x in hi)
        if (env, ep) != (self.env_id, self.episode_id):
            return self._refuse("ENV_OR_EPISODE_MISMATCH")
        if not 0 <= step < ID_LIMIT or health not in (0, 1, 2) or noise not in (0, 1):
            return self._refuse("MALFORMED_HEADER")
        if self._last_step is not None and step <= self._last_step:
            return self._refuse("STEP_NOT_INCREASING")
        if not np.isfinite(hf).all():
            return self._refuse("NONFINITE_HEADER")
        t, x, y, z, yaw = (float(v) for v in hf)
        if max(abs(x), abs(y), abs(z)) > COORD_LIMIT or abs(yaw) > 2 * math.pi:
            return self._refuse("POSE_OUT_OF_RANGE")
        if abs(t - step * self.cfg.dt) > 1e-9 * max(1.0, abs(t)):
            return self._refuse("TIMESTAMP_NOT_ON_STEP_SCHEDULE")
        cam = None if camera is None else B.exact_copy(camera, B.F64, (14,))
        trk = None if track is None else B.exact_copy(track, B.F64, (5,))

        # ---- per-beam validation (declared status + range consistency); range alone never decides HIT/NO_HIT
        cfg = self.cfg
        v = np.full(self.shape, V_INVALID, np.int8)
        finite = np.isfinite(r)
        v[s == NOT_DELIVERED] = V_MISSING
        v[(s == NEAR_CLIPPED) | (s == DISTANCE_ONLY)] = V_AMBIGUOUS
        hit_ok = (s == HIT) & finite & (r >= cfg.min_range) & (r <= cfg.max_range)
        v[(s == HIT) & finite & ~hit_ok] = V_AMBIGUOUS
        v[hit_ok] = V_HIT
        miss_ok = (s == NO_HIT) & finite & (r == cfg.no_hit_fill)
        v[(s == NO_HIT) & finite & ~miss_ok] = V_AMBIGUOUS
        v[miss_ok] = V_NO_HIT
        health_ok = health == HEALTH_RUNTIME or (health == HEALTH_SYNTHETIC and cfg.allow_synthetic_health)
        if not health_ok or noise == 1:             # R-6: noise without a qualified error bound
            v[(v == V_HIT) | (v == V_NO_HIT)] = V_AMBIGUOUS
        v.flags.writeable = False
        complete = bool(((v == V_HIT) | (v == V_NO_HIT)).all())

        # ---- geometry in the world frame (declared pose; yaw-level sensor frame)
        origin = np.array([x, y, z])
        wd = self.dirs.reshape(-1, 3) @ _rz(yaw).T
        rr, vv = r.reshape(-1), v.reshape(-1)
        f0 = 0.0 if cfg.nearest_hit_assured else cfg.min_range
        f1 = np.where(vv == V_HIT, rr - cfg.range_error_bound - 1e-7, cfg.max_range - cfg.range_error_bound)
        lo, hi_ = self._extent(z, wd[:, 2])
        a_t = np.maximum(f0, lo)
        free, free_map, occl, withdrawn, hits2, hits3 = [], [], [], [], [], []
        for k in range(len(vv)):
            d = wd[k]
            if vv[k] in (V_HIT, V_NO_HIT) and complete:
                b_t = min(f1[k], hi_[k])
                if b_t > a_t[k]:
                    free.append(((x + d[0] * a_t[k], y + d[1] * a_t[k]), (x + d[0] * b_t, y + d[1] * b_t)))
                bm = min(f1[k] - WIDTH, hi_[k])
                if bm > a_t[k]:
                    free_map.append(((x + d[0] * a_t[k], y + d[1] * a_t[k]), (x + d[0] * bm, y + d[1] * bm)))
            if vv[k] == V_HIT:
                hits2.append((x + d[0] * rr[k], y + d[1] * rr[k]))
                hits3.append((x + d[0] * rr[k], y + d[1] * rr[k], z + d[2] * rr[k]))
                e = min(cfg.max_range, hi_[k])
                if e > rr[k] + WIDTH:
                    occl.append(((x + d[0] * (rr[k] + WIDTH), y + d[1] * (rr[k] + WIDTH)), (x + d[0] * e, y + d[1] * e)))
            elif not complete:      # fail-closed: withdraw visibility along every non-HIT beam of an incomplete packet
                e = min(cfg.max_range, hi_[k])
                if e > lo[k]:
                    withdrawn.append(((x + d[0] * lo[k], y + d[1] * lo[k]), (x + d[0] * e, y + d[1] * e)))

        # ---- track (liveness/conservatism only): fresh, active, finite
        track_xyr = None
        if trk is not None and np.isfinite(trk).all() and trk[0] == 1.0 and 0.0 <= t - trk[1] <= TRACK_FRESH \
                and trk[4] >= 0.0:
            track_xyr = (float(trk[2]), float(trk[3]), float(trk[4]))

        arr = lambda L, n: np.asarray(L, dtype=np.float64).reshape(-1, *n)   # noqa: E731
        plan = self.map.plan(step, t, complete, arr(free_map, (2, 2)), arr(hits2, (2,)), arr(occl, (2, 2)),
                             arr(withdrawn, (2, 2)), track_xyr)
        # ---- mutation (cannot fail on caller data)
        self.map.apply(plan)
        self._last_step = step
        self._current = (step, t, complete, tuple(free), tuple(hits2), health, (x, y, z, yaw))
        # ---- label layer last, isolated: nothing it does or raises can change the map, the current packet or
        # the step's acceptance
        if camera is not None and cam is None:
            label_report = {"camera": "IGNORED_MALFORMED_ARRAY", "labelled_returns": 0, "target_estimate_world": None}
        else:
            try:
                label_report = self._labels.update(step, t, (x, y, z), yaw, hits3, cam)
            except Exception as e:
                self.exceptions.append(("labels", type(e).__name__, str(e)[:200]))
                label_report = {"camera": "LABEL_LAYER_ERROR", "labelled_returns": 0, "target_estimate_world": None}
        counts = {OUTCOME_NAMES[c]: int((v == c).sum()) for c in OUTCOME_NAMES}
        return {"accepted": True, "step": step, "complete": complete, "beam_outcomes": counts,
                "free_rays": len(free), "hits": len(hits2), "camera": label_report["camera"],
                "target_estimate_world": label_report["target_estimate_world"],
                "labelled_returns": label_report["labelled_returns"],
                "track": "USED_FOR_STATIC_RULE" if track_xyr else ("ABSENT" if track is None else "IGNORED"),
                "evidence_label": self._label(health), "health_basis": HEALTH_BASIS, "time_basis": TIME_BASIS}

    def _extent(self, z, dz):
        """Free evidence exists strictly inside the qualified bar-height interval for every layout."""
        n = len(dz)
        zlo, zhi = REG.BAR_Z_EXTENT
        eps = self.cfg.direction_error_m
        if eps:     # every ray within the declared direction bound stays inside the bar-height interval
            zlo, zhi = zlo + eps, zhi - eps
        lo, hi = np.zeros(n), np.full(n, np.inf)
        if not zlo < z < zhi:
            return lo, np.zeros(n)              # boundary grazing is unqualified: no 2-D free evidence
        if self.layout.planar:
            return lo, hi
        up, down = dz > 0, dz < 0
        hi[up] = (zhi - z) / dz[up]
        hi[down] = (zlo - z) / dz[down]
        return lo, hi

    def _label(self, health):
        if health == HEALTH_RUNTIME:
            return "RUNTIME_DECLARED"
        return "SYNTHETIC_ASSUMED_HEALTH" if health == HEALTH_SYNTHETIC else "NONE"

    # ------------------------------------------------------------ certify
    def certify(self, paths, for_step, deadline_s=None):
        with np.errstate(all="ignore"), warnings.catch_warnings():
            warnings.simplefilter("ignore")
            cur = self._current
            try:
                return self._certify(paths, for_step, deadline_s)
            except Exception as e:
                self.exceptions.append(("certify", type(e).__name__, str(e)[:200]))
                return [self._result(None, "INTERNAL_ERROR_" + type(e).__name__, "UNKNOWN", packet=cur)]

    def _result(self, k, reason, category, packet=_USE_CURRENT, **extra):
        cur = self._current if packet is _USE_CURRENT else packet
        out = {"index": k, "static_bar_certified": reason == "ALL_COLLIDING_CENTER_CELLS_EXCLUDED",
               "result": "STATIC_BAR_CERTIFIED" if reason == "ALL_COLLIDING_CENTER_CELLS_EXCLUDED" else "REFUSED",
               "reason": reason, "category": category, "valid_for_step": None if cur is None else cur[0],
               "scope": dict(SCOPE, arena_boundary=("CHECKED_DECLARED_BOUNDS" if self.cfg.arena else "UNVERIFIED")),
               "evidence_label": "NONE" if cur is None else self._label(cur[5]), "health_basis": HEALTH_BASIS,
               "time_basis": TIME_BASIS, "config": self.cfg.name, "core_version": CORE_VERSION}
        out.update(extra)
        return out

    def _certify(self, paths, for_step, deadline_s):
        t0 = time.perf_counter()
        cur = self._current
        result = functools.partial(self._result, packet=cur)
        p = B.exact_paths(paths, K_MAX)
        if p is None:
            return [result(None, "MALFORMED_ARRAY_paths", "INVALID_INPUT")]
        if deadline_s is not None and not (type(deadline_s) is float and math.isfinite(deadline_s) and deadline_s > 0):
            return [result(None, "MALFORMED_DEADLINE", "INVALID_INPUT")]
        if cur is None:
            return [result(k, "NO_CURRENT_PACKET", "UNKNOWN") for k in range(len(p))]
        if type(for_step) is not int or for_step != cur[0]:
            return [result(k, "PACKET_NOT_CURRENT_FOR_STEP", "UNKNOWN") for k in range(len(p))]
        if not cur[2]:
            return [result(k, "INCOMPLETE_ACQUISITION_PACKET", "UNKNOWN") for k in range(len(p))]
        if self.cfg.range_error_bound > 0:
            return [result(k, "RANGE_ERROR_BOUND_NOT_QUALIFIED", "SAFE_REJECT") for k in range(len(p))]
        if self.cfg.orientation_prior != "AXIS_ALIGNED_CATALOG":
            return [result(k, "ORIENTATION_PRIOR_NOT_QUALIFIED", "SAFE_REJECT") for k in range(len(p))]
        out = []
        for k in range(len(p)):
            if deadline_s is not None and time.perf_counter() - t0 > deadline_s:
                out.append(result(k, "DEADLINE_UNKNOWN", "UNKNOWN"))
                continue
            try:                    # one result per requested segment, in order, even on an internal error
                one = self._certify_one(k, p[k], cur)
                # Decision expiry does not interrupt computation; a late result grants no certificate.
                if deadline_s is not None and time.perf_counter() - t0 > deadline_s:
                    one = result(k, "DEADLINE_UNKNOWN", "UNKNOWN")
                out.append(one)
            except Exception as e:
                self.exceptions.append(("certify_one", type(e).__name__, str(e)[:200]))
                out.append(result(k, "INTERNAL_ERROR_" + type(e).__name__, "UNKNOWN"))
        return out

    def _certify_one(self, k, seg, cur):
        result = functools.partial(self._result, packet=cur)
        R = self.cfg.plan_radius
        a, b = (float(seg[0, 0]), float(seg[0, 1])), (float(seg[1, 0]), float(seg[1, 1]))
        if not all(math.isfinite(c) and abs(c) <= COORD_LIMIT for c in a + b):
            return result(k, "INVALID_PATH", "SAFE_REJECT")
        if self.cfg.arena is not None:
            x0, x1, y0, y1 = self.cfg.arena
            if not all(x0 + R <= q[0] <= x1 - R and y0 + R <= q[1] <= y1 - R for q in (a, b)):
                return result(k, "ARENA_BOUNDARY", "SAFE_REJECT")
        veto, n = self.map.veto_near(np.array(a), np.array(b), R, cur[0])
        if veto is not None:
            return result(k, "MAP_VETO_" + veto, "OCCUPIED", map_veto_cells=n)
        coverage = {"VERBATIM_PY": CB._coverage_B, "NATIVE_EXACT": CN.coverage_B_native}.get(self.cfg.coverage)
        if coverage is None:
            return result(k, "COVERAGE_IMPL_NOT_QUALIFIED", "SAFE_REJECT")
        shapes, radius = REG.BAR_HALF_SIZES, R
        eps = self.cfg.direction_error_m
        if eps:     # direction bound: a free point within eps of the cast ray proves only the footprint shrunk by eps
            shapes = tuple((hx - eps, hy - eps) for hx, hy in REG.BAR_HALF_SIZES)
            radius = R + 2.0 * eps
        r = coverage((a, b), list(cur[3]), list(cur[4]), shapes, radius, self.cfg.resolution, self.cfg.max_nodes)
        cat = ("CERTIFIED_STATIC_BARS" if r["certified"] else "OCCUPIED"
               if r["reason"] == "OBSERVED_SURFACE_INTERSECTION" else "UNKNOWN"
               if r["reason"] == "NO_VALID_RAYS" else "SAFE_REJECT")
        return result(k, r["reason"], cat, nodes=r["nodes"])

    # ------------------------------------------------------------ reporting (planner-facing; not used by certify)
    def map_states(self, points, now_t):
        """Reported state of the cells containing (N, 2) float64 points: the map's state, with a fresh camera label
        shown as MOVING_TARGET_CANDIDATE on an unresolved occupied cell. Diagnostic labels, never certificates."""
        with np.errstate(all="ignore"), warnings.catch_warnings():
            warnings.simplefilter("ignore")
            try:
                return self._map_states(points, now_t)
            except Exception as e:
                self.exceptions.append(("map_states", type(e).__name__, str(e)[:200]))
                return None

    def _map_states(self, points, now_t):
        if not (type(points) is np.ndarray and type(points.dtype) is B.F64 and points.ndim == 2
                and points.shape[1] == 2 and type(now_t) is float and math.isfinite(now_t)):
            return None
        pts = B.exact_copy(points, B.F64, points.shape)
        if pts is None or not np.isfinite(pts).all() or (np.abs(pts) > COORD_LIMIT).any():
            return None
        out = []
        for q in pts:
            ids, _ = self.map._ids(q.reshape(1, 2))
            if not len(ids):
                out.append("UNKNOWN")
                continue
            c = int(ids[0])
            st = self.map.derive_ids([c])[0]
            if st == "OBSERVED_HIT" and self._labels.labelled(c, int(self.map.occ_last.flat[c]), now_t):
                st = "MOVING_TARGET_CANDIDATE"
            out.append(st)
        return out


def certify_step(core, ranges, status, header_i, header_f, paths, camera=None, track=None, deadline_s=None):
    """One call per control step: ingest the packet, then certify the candidate segments for that same step."""
    rep = core.step(ranges, status, header_i, header_f, camera=camera, track=track)
    step = rep.get("step") if rep.get("accepted") else -1
    return rep, core.certify(paths, step, deadline_s=deadline_s)
