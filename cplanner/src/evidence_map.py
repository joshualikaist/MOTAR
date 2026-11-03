"""P0-R occupancy evidence map: world-anchored 2-D grid, LiDAR evidence only, veto-only towards the certificate.

Each cell keeps EVIDENCE; its state is derived from that evidence on every query, and nothing changes because time
passes (v2-v7 ledger rule, now on a world grid with the packet's declared pose instead of a fixed-frame beam ledger).

Evidence rules (fixed before the R2 runs):
- Free evidence comes only from validated free rays of COMPLETE packets. A cell is traversed if a sample of the ray
  (spacing WIDTH/4) lies in the central half of the cell, so a corner graze is not traversal; the last WIDTH before
  a HIT is never free evidence.
- Every validated HIT, of a complete or an incomplete packet, is occupancy evidence of its cell. The camera cannot
  reach this module: there is no camera argument anywhere in it.
- An occupancy episode resolves only after K_FREE distinct later packets add free evidence through the cell. Time
  alone never resolves anything.
- STATIC_OCCUPANCY (kept from v7: >= N_STATIC distinct packets spanning >= T_STATIC s, each hit made while a fresh,
  active track placed the moving target >= D_SEP beyond its radius from the hit) is permanent: free evidence after
  it makes the cell CONFLICT, which is permanent too. A track can therefore only make a cell MORE conservative.
- An incomplete packet adds no free evidence and withdraws the current visibility of every cell along each beam that
  did not validate (and along validated NO_HIT beams of that packet).

Towards the certificate the map is a VETO ONLY: `veto_near` reports cells in a veto state near a segment; it never
grants anything, and UNKNOWN never vetoes (the certificate's own coverage proof decides). Occupancy whose episode
started in the certificate's own packet is skipped there because that packet's HIT points are checked exactly by
the certificate's surface test.
"""
import math

import numpy as np

WIDTH = 0.05
K_FREE = 2
N_STATIC = 3
T_STATIC = 2.0
D_SEP = 1.0
TRACK_FRESH = 0.2

VIS_NONE, VIS_FREE, VIS_HIT, VIS_OCCLUDED, VIS_WITHDRAWN = 0, 1, 2, 3, 4
VALIDATED_FREE, OBSERVED_HIT, STATIC_OCCUPANCY, UNKNOWN, CONFLICT = (
    "VALIDATED_FREE", "OBSERVED_HIT", "STATIC_OCCUPANCY", "UNKNOWN", "CONFLICT")


class EvidenceMap:
    def __init__(self, bounds):
        lo, hi = bounds
        self.lo = float(lo)
        self.n = int(round((hi - lo) / WIDTH))
        shape = (self.n, self.n)
        self.occ_first = np.full(shape, -1, np.int64)   # step that started the current occupancy episode
        self.occ_last = np.full(shape, -1, np.int64)    # step of the latest occupancy evidence
        self.free_since = np.zeros(shape, np.int32)      # distinct packets with free evidence after occ_last
        self.free_any = np.zeros(shape, bool)
        self.static = np.zeros(shape, bool)
        self.conflict = np.zeros(shape, bool)
        self.vis = np.zeros(shape, np.int8)
        self.pending = {}           # flat cell -> [(step, t, track_ok, sep)] hits since the cell's last free evidence
        self.outside = 0            # evidence points that fell outside the grid (never recorded)

    # ------------------------------------------------------------ geometry helpers (pure)
    def _ids(self, pts):
        """Flat cell ids of the in-grid points of (M, 2) `pts`, and how many were outside."""
        if len(pts) == 0:
            return np.empty(0, np.int64), 0
        ij = np.floor((pts - self.lo) / WIDTH).astype(np.int64)
        ok = (ij >= 0).all(1) & (ij < self.n).all(1)
        return ij[ok, 0] * self.n + ij[ok, 1], int((~ok).sum())

    def _traversed(self, segs, inner):
        """Unique cells traversed by (S, 2, 2) segments; `inner`: only samples in the central half of a cell."""
        if len(segs) == 0:
            return np.empty(0, np.int64)
        a, b = segs[:, 0], segs[:, 1]
        k = np.maximum(1, np.ceil(np.hypot(*(b - a).T) / (WIDTH / 4)).astype(np.int64)) + 1
        rep = np.repeat(np.arange(len(segs)), k)
        t = np.concatenate([np.linspace(0.0, 1.0, int(m)) for m in k])
        p = a[rep] + (b - a)[rep] * t[:, None]
        if inner:
            f = (p - self.lo) / WIDTH
            p = p[(np.abs(f - np.floor(f) - 0.5) <= 0.25).all(1)]
        return np.unique(self._ids(p)[0])

    # ------------------------------------------------------------ ingestion
    def plan(self, step, t, complete, free_segs, hit_pts, occluded_segs, withdrawn_segs, track):
        """Everything one packet will change, computed before any mutation (atomic ingest). `track` is None or the
        core's validated (x, y, radius) of a fresh, active track."""
        free_ids = self._traversed(free_segs, True) if complete else np.empty(0, np.int64)
        hit_ids, out = self._ids(hit_pts)
        hit_keep = ((np.floor((hit_pts - self.lo) / WIDTH) >= 0).all(1)
                    & (np.floor((hit_pts - self.lo) / WIDTH) < self.n).all(1)) if len(hit_pts) else np.zeros(0, bool)
        track_ok = track is not None
        seps = []
        for p in hit_pts[hit_keep]:
            seps.append(math.hypot(p[0] - track[0], p[1] - track[1]) - track[2] if track_ok else None)
        return dict(step=step, t=t, free=free_ids, hits=hit_ids, seps=seps, track_ok=track_ok, outside=out,
                    occluded=self._traversed(occluded_segs, False), withdrawn=self._traversed(withdrawn_segs, False))

    def apply(self, pl):
        """Mutation for one planned packet (nothing below depends on caller data)."""
        step, t = pl["step"], pl["t"]
        self.vis.flat[pl["occluded"]] = VIS_OCCLUDED
        f = pl["free"]
        if len(f):
            occ = self.occ_last.flat[f] >= 0
            self.free_since.flat[f[occ]] += 1
            self.free_any.flat[f] = True
            self.vis.flat[f] = VIS_FREE
            st = self.static.flat[f]
            self.conflict.flat[f[st]] = True
            for c in set(f.tolist()) & set(self.pending):
                del self.pending[c]
        self.vis.flat[pl["withdrawn"]] = VIS_WITHDRAWN
        for c, sep in zip(pl["hits"].tolist(), pl["seps"]):
            if self.occ_last.flat[c] < 0 or self.free_since.flat[c] >= K_FREE:
                self.occ_first.flat[c] = step                   # a new occupancy episode
            self.occ_last.flat[c] = step
            self.free_since.flat[c] = 0
            self.vis.flat[c] = VIS_HIT
            h = self.pending.setdefault(c, [])
            h.append((step, t, pl["track_ok"], sep))
            if not self.static.flat[c]:
                self._maybe_confirm_static(c, h)
        self.outside += pl["outside"]

    def _maybe_confirm_static(self, c, hits):
        if len({h[0] for h in hits}) < N_STATIC:
            return
        if max(h[1] for h in hits) - min(h[1] for h in hits) < T_STATIC:
            return
        if not all(h[2] and h[3] is not None and h[3] >= D_SEP for h in hits):
            return
        self.static.flat[c] = True

    # ------------------------------------------------------------ derivation
    def derive_ids(self, ids):
        """Map-only state of each flat cell id (labels are an overlay of the core, never read here)."""
        out = []
        for c in ids:
            if self.conflict.flat[c]:
                out.append(CONFLICT)
            elif self.static.flat[c]:
                out.append(STATIC_OCCUPANCY)
            elif self.occ_last.flat[c] >= 0 and self.free_since.flat[c] < K_FREE:
                out.append(OBSERVED_HIT)
            elif self.free_any.flat[c] and self.vis.flat[c] == VIS_FREE:
                out.append(VALIDATED_FREE)
            else:
                out.append(UNKNOWN)
        return out

    def veto_near(self, a, b, radius, current_step):
        """(state, n_cells) of the strongest veto among cells within radius + half a cell diagonal of segment a-b,
        or (None, 0). Veto states: CONFLICT, STATIC_OCCUPANCY, and an unresolved occupancy episode that started
        before `current_step`."""
        reach = radius + WIDTH * math.sqrt(2.0) / 2.0
        lo = np.floor((np.minimum(a, b) - reach - self.lo) / WIDTH).astype(np.int64)
        hi = np.floor((np.maximum(a, b) + reach - self.lo) / WIDTH).astype(np.int64)
        lo = np.clip(lo, 0, self.n)
        hi = np.clip(hi + 1, 0, self.n)
        if (hi <= lo).any():
            return None, 0
        sl = (slice(lo[0], hi[0]), slice(lo[1], hi[1]))
        cx = self.lo + (np.arange(lo[0], hi[0]) + 0.5) * WIDTH
        cy = self.lo + (np.arange(lo[1], hi[1]) + 0.5) * WIDTH
        px, py = np.meshgrid(cx, cy, indexing="ij")
        dx, dy = b[0] - a[0], b[1] - a[1]
        den = dx * dx + dy * dy
        u = np.clip(((px - a[0]) * dx + (py - a[1]) * dy) / den, 0.0, 1.0) if den else np.zeros_like(px)
        near = np.hypot(px - a[0] - u * dx, py - a[1] - u * dy) <= reach
        conflict = self.conflict[sl] & near
        static = self.static[sl] & near
        occ = (self.occ_last[sl] >= 0) & (self.free_since[sl] < K_FREE) & (self.occ_first[sl] < current_step) & near
        for state, m in ((CONFLICT, conflict), (STATIC_OCCUPANCY, static), (OBSERVED_HIT, occ)):
            if m.any():
                return state, int(m.sum())
        return None, 0
