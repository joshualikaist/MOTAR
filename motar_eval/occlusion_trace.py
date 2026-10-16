"""Compact occlusion telemetry (R-P1a-v2): per-frame UAV states and sensor flags, per-reset layouts.

Evaluation only. Nothing here reaches an observation, a reward or a termination, and nothing draws
randomness. Three files per cell, all created exclusively (never overwritten):

* ``occlusion_frames.bin`` -- fixed-size little-endian records (``FRAME_DTYPE``), one per consumed
  observation and environment, in consumption order. Frame ``k`` is the state after ``k`` completed
  actions (the reset observation is frame 0); it is the state the observation was rendered from.
* ``occlusion_layouts.npz`` -- every distinct obstacle layout once (keyed by scene hash) plus one
  row per reset mapping (env, episode) to its layout.
* ``occlusion_trace_summary.json`` -- dtype, counts, SHA-256 of both files, camera and timing
  contract, written at finalize.

Sensor capture time is not recorded. ``render_alignment`` says whether the detector delivered the
same frame it rendered (zero configured latency, no latency steps, no empirical injector); only then
may camera flags and current geometry be compared frame by frame.
"""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
from typing import Dict, List

import numpy as np

SCHEMA = "motar.occlusion-trace.v1"
FRAMES_FILE = "occlusion_frames.bin"
LAYOUTS_FILE = "occlusion_layouts.npz"
SUMMARY_FILE = "occlusion_trace_summary.json"
NOT_RECORDED = "NOT_RECORDED"

# Bit flags of ``FRAME_DTYPE['flags']``.
CAMERA_VALID, FUSED_VALID, TRACK_ACTIVE, GEO_IN_FOV, GEO_VISIBLE, LIDAR_VALID, ACQUIRED_BEFORE = (
    1, 2, 4, 8, 16, 32, 64)

FRAME_DTYPE = np.dtype([
    ("env", "<u2"), ("frame", "<u2"), ("episode", "<u4"),
    ("p_pos", "<f4", (3,)), ("p_vel", "<f4", (3,)), ("p_quat", "<f4", (4,)), ("p_vehicle_quat", "<f4", (4,)),
    ("t_pos", "<f4", (3,)), ("t_vel", "<f4", (3,)),
    ("flags", "u1"),
    # R-P1a probe, when it ran: -1 unknown/not run, 0 clear, 1 blocked; pixels -1 when unknown.
    ("probe_center_blocked", "i1"), ("probe_visible_pixels", "<i4"),
])
RESET_DTYPE = np.dtype([("env", "<u2"), ("episode", "<u4"), ("layout", "<u4")])


def pack_flags(camera, fused, track, in_fov, geo_visible, lidar, acquired_before):
    """[n] bool arrays -> [n] uint8 bit field."""
    out = np.zeros(len(camera), dtype=np.uint8)
    for bit, values in ((CAMERA_VALID, camera), (FUSED_VALID, fused), (TRACK_ACTIVE, track),
                        (GEO_IN_FOV, in_fov), (GEO_VISIBLE, geo_visible), (LIDAR_VALID, lidar),
                        (ACQUIRED_BEFORE, acquired_before)):
        if values is not None:
            out |= np.where(np.asarray(values, dtype=bool), bit, 0).astype(np.uint8)
    return out


def flag(frames, bit):
    return (frames["flags"] & bit) != 0


def sha256_file(path) -> str:
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for block in iter(lambda: handle.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


class TraceWriter:
    """Append-only writer. Layouts are kept in memory (deduplicated) and written at ``close``."""

    def __init__(self, directory, metadata: Dict, flush_every: int = 64):
        self.directory = Path(directory)
        for name in (FRAMES_FILE, LAYOUTS_FILE, SUMMARY_FILE):
            if (self.directory / name).exists():
                raise FileExistsError("occlusion trace needs a fresh cell directory: %s exists" % name)
        self.metadata = dict(metadata)
        self._frames = open(self.directory / FRAMES_FILE, "xb")
        self.frames_written = 0
        self._pending = 0
        self.flush_every = int(flush_every)
        self._layout_index: Dict[str, int] = {}
        self._layouts: List[Dict] = []
        self._resets: List[tuple] = []
        self.closed = False

    def add_reset(self, env: int, episode: int, scene_hash: str, center, half, quat=None):
        """Record the layout of one reset (deduplicated by ``scene_hash``)."""
        index = self._layout_index.get(scene_hash)
        if index is None:
            center = np.asarray(center, dtype=np.float32)
            half = np.asarray(half, dtype=np.float32)
            if center.shape != half.shape or center.ndim != 2 or center.shape[1] != 3:
                raise ValueError("layout centre/half must be [B, 3]")
            quat = (np.tile(np.array([0, 0, 0, 1], dtype=np.float32), (len(center), 1)) if quat is None
                    else np.asarray(quat, dtype=np.float32))
            index = len(self._layouts)
            self._layout_index[scene_hash] = index
            self._layouts.append({"hash": scene_hash, "center": center, "half": half, "quat": quat})
        self._resets.append((int(env), int(episode), index))

    def write_frames(self, records: np.ndarray):
        if records.dtype != FRAME_DTYPE:
            raise ValueError("frame records must use FRAME_DTYPE")
        self._frames.write(records.tobytes())
        self.frames_written += len(records)
        self._pending += 1
        if self._pending >= self.flush_every:
            self._frames.flush()
            self._pending = 0

    def close(self, reason: str) -> Dict:
        if self.closed:
            return {}
        self._frames.close()
        sizes = np.array([len(l["center"]) for l in self._layouts], dtype=np.int64)
        offsets = np.concatenate([[0], np.cumsum(sizes)]).astype(np.int64)
        stack = lambda key, width: (np.concatenate([l[key] for l in self._layouts]) if self._layouts  # noqa: E731
                                    else np.zeros((0, width), dtype=np.float32))
        resets = np.array(self._resets, dtype=RESET_DTYPE) if self._resets else np.zeros(0, dtype=RESET_DTYPE)
        with open(self.directory / LAYOUTS_FILE, "xb") as handle:
            np.savez_compressed(handle, center=stack("center", 3), half=stack("half", 3), quat=stack("quat", 4),
                                offsets=offsets, hashes=np.array([l["hash"] for l in self._layouts]),
                                resets=resets)
        summary = {"schema": SCHEMA, "reason": reason, "metadata": self.metadata,
                   "frame_dtype": FRAME_DTYPE.descr, "frame_itemsize": FRAME_DTYPE.itemsize,
                   "frames": self.frames_written, "layouts": len(self._layouts), "resets": len(self._resets),
                   "files": {FRAMES_FILE: sha256_file(self.directory / FRAMES_FILE),
                             LAYOUTS_FILE: sha256_file(self.directory / LAYOUTS_FILE)},
                   "bytes": {FRAMES_FILE: (self.directory / FRAMES_FILE).stat().st_size,
                             LAYOUTS_FILE: (self.directory / LAYOUTS_FILE).stat().st_size}}
        with open(self.directory / SUMMARY_FILE, "x", encoding="utf-8") as handle:
            handle.write(json.dumps(summary, indent=1, sort_keys=True) + "\n")
        self.closed = True
        return summary


def read_trace(directory) -> Dict:
    """Frames, layouts and resets of one cell, after checking both SHA-256 values."""
    directory = Path(directory)
    summary = json.loads((directory / SUMMARY_FILE).read_text())
    if summary.get("schema") != SCHEMA:
        raise ValueError("unsupported occlusion trace schema")
    if summary.get("frame_itemsize") != FRAME_DTYPE.itemsize:
        raise ValueError("frame record size differs from this reader")
    for name, recorded in summary["files"].items():
        if sha256_file(directory / name) != recorded:
            raise ValueError("%s SHA-256 differs from its summary" % name)
    frames = np.fromfile(directory / FRAMES_FILE, dtype=FRAME_DTYPE)
    if len(frames) != summary["frames"]:
        raise ValueError("frame count differs from the summary")
    with np.load(directory / LAYOUTS_FILE) as data:
        layouts = {k: data[k] for k in data.files}
    return {"summary": summary, "frames": frames, "layouts": layouts}


def layout_arrays(layouts: Dict, index: int):
    """(center [B, 3], half [B, 3], quat [B, 4]) of one stored layout."""
    a, b = int(layouts["offsets"][index]), int(layouts["offsets"][index + 1])
    return layouts["center"][a:b], layouts["half"][a:b], layouts["quat"][a:b]


def reset_lookup(layouts: Dict) -> Dict[tuple, int]:
    """(env, episode) -> layout index; a duplicate key is an error."""
    out = {}
    for row in layouts["resets"]:
        key = (int(row["env"]), int(row["episode"]))
        if key in out:
            raise ValueError("two resets recorded for env %d episode %d" % key)
        out[key] = int(row["layout"])
    return out


def estimate_bytes(frames: int, layouts: int, obstacles_per_layout: int, compression: float = 0.85) -> int:
    """Upper-bound storage for a cell: raw frames plus compressed layouts (float32 x 10 per box)."""
    return int(frames * FRAME_DTYPE.itemsize + layouts * obstacles_per_layout * 40 * compression)


def episode_frames(frames: np.ndarray) -> Dict[tuple, np.ndarray]:
    """Group frame records by (env, episode), each sorted by frame index (consumption order kept)."""
    order = np.lexsort((frames["frame"], frames["episode"], frames["env"]))
    sorted_frames = frames[order]
    keys = np.stack([sorted_frames["env"].astype(np.int64), sorted_frames["episode"].astype(np.int64)], axis=1)
    if len(keys) == 0:
        return {}
    change = np.nonzero(np.any(np.diff(keys, axis=0) != 0, axis=1))[0] + 1
    bounds = np.concatenate([[0], change, [len(keys)]])
    return {(int(keys[a, 0]), int(keys[a, 1])): sorted_frames[a:b] for a, b in zip(bounds[:-1], bounds[1:])}


__all__ = ["SCHEMA", "FRAMES_FILE", "LAYOUTS_FILE", "SUMMARY_FILE", "FRAME_DTYPE", "RESET_DTYPE",
           "CAMERA_VALID", "FUSED_VALID", "TRACK_ACTIVE", "GEO_IN_FOV", "GEO_VISIBLE", "LIDAR_VALID",
           "ACQUIRED_BEFORE", "pack_flags", "flag", "TraceWriter", "read_trace", "layout_arrays",
           "reset_lookup", "estimate_bytes", "episode_frames", "sha256_file", "NOT_RECORDED"]
