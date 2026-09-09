"""Verify the extracted ETH ds5 cam0 video against its published timestamps and the calibration candidate.

Checks (all diagnostics; none of them validates lens/crop calibration or drone identity):
  1. container stream metadata (codec, size, rate, declared frame count)
  2. container packet timestamps for EVERY frame (no decode) and a full decode count
  3. published project-time frame timestamps versus container timestamps under two hypotheses:
     already-synchronised (published = scale*pts + shift) or raw camera time (published = pts)
  4. calibration candidate: resolution/FPS equality and whatever camera-model tags the file carries
"""
import argparse
from fractions import Fraction
import json
from pathlib import Path
import platform
import statistics
import subprocess
import sys

from prepare_eth_ds5 import COMMIT, digests, numeric_table, write_json


def probe(ffprobe, video, *extra):
    return json.loads(subprocess.check_output([ffprobe, "-v", "error", "-of", "json"] + list(extra) + [str(video)], text=True))


def packet_times(ffprobe, video):
    out = subprocess.check_output([ffprobe, "-v", "error", "-select_streams", "v:0", "-show_entries",
                                   "packet=pts_time", "-of", "csv=p=0", str(video)], text=True)
    times = [float(line.strip().rstrip(",")) for line in out.splitlines() if line.strip()]
    if len(times) < 2:
        raise ValueError("too few packets")
    return sorted(times)  # decode order differs from presentation order for B-frames


def decoded_frame_count(ffmpeg, video):
    out = subprocess.run([ffmpeg, "-v", "info", "-nostats", "-i", str(video), "-map", "0:v:0", "-vsync", "passthrough",
                          "-f", "null", "-"], capture_output=True, text=True, check=True).stderr
    counts = [int(t.split("frame=")[1].split()[0]) for t in out.splitlines() if "frame=" in t and "fps=" in t]
    if not counts:
        raise ValueError("ffmpeg did not report a decoded frame count")
    return counts[-1]


def published_fit(frame_ids, published, period, sync):
    """Least-squares line of published stamps against frame id, compared with every camera's scale.

    The published file is treated as opaque data: the fitted slope tells which scale generated it,
    and the intercept is expressed in frames relative to cam0's published shift.
    """
    n = len(frame_ids)
    mean_x, mean_y = sum(frame_ids) / n, sum(published) / n
    sxx = sum((x - mean_x) ** 2 for x in frame_ids)
    slope = sum((x - mean_x) * (y - mean_y) for x, y in zip(frame_ids, published)) / sxx
    intercept = mean_y - slope * mean_x
    residual = max(abs(y - (slope * x + intercept)) for x, y in zip(frame_ids, published))
    ratio = slope / period
    return {"slope_s_per_frame": slope, "intercept_s": intercept, "max_abs_residual_s": residual,
            "slope_over_nominal_period": ratio,
            "closest_camera_scale": min(sync, key=lambda cam: abs(sync[cam][0] - ratio)),
            "camera_scales": {str(cam): sync[cam][0] for cam in sync},
            "intercept_minus_cam0_shift_frames": (intercept - sync[0][1]) / slope}


def timestamp_consistency(pts, published, scale, shift, period, alignments=(0, 1)):
    """Residuals of published project-time stamps against container stamps.

    Every alignment (published row i <-> container packet i+k) and both models are reported so
    that the caller cannot silently pick the flattering one. Only jitter and drift after removing
    the first-row offset are tested; the offset (relative to the container start) is reported, not judged.
    """
    result = {"row_count_published": len(published), "packet_count": len(pts), "alignments": {}}
    for k in alignments:
        if k + len(published) > len(pts):
            continue
        window = pts[k:k + len(published)]
        base = pts[0]  # container start for every alignment, so offsets differ by exactly k frames
        models = {"synchronised_affine": [scale * (p - base) + shift for p in window],
                  "raw_camera_time": [p - base for p in window]}
        entry = {}
        for name, model in models.items():
            residual = [t - m for t, m in zip(published, model)]
            centred = [r - residual[0] for r in residual]
            entry[name] = {"offset_first_row_s": residual[0], "offset_first_row_frames": residual[0] / period,
                           "drift_last_minus_first_s": centred[-1],
                           "max_abs_jitter_after_offset_s": max(abs(c) for c in centred),
                           "within_half_frame": max(abs(c) for c in centred) < 0.5 * period}
        result["alignments"]["packet_offset_%d" % k] = entry
    steps = [b - a for a, b in zip(published, published[1:])]
    result["published_step_s"] = {"min": min(steps), "max": max(steps), "median": statistics.median(steps),
                                  "expected_cam0_scaled_period_s": scale * period}
    return result


def edit_list_offsets(video, timescale=30000):
    """Media-time offsets of every `elst` entry in the MP4 (raw container parse; ffprobe hides edit lists).

    A nonzero video edit offset means decoders that ignore edit lists (e.g. some MATLAB backends) report
    presentation times shifted by that amount relative to ffmpeg's zero-based timeline.
    """
    import struct
    offsets = []
    with open(video, "rb") as handle:
        handle.seek(0, 2)
        size = handle.tell()
        position = 0
        while position < size:
            handle.seek(position)
            header = handle.read(16)
            if len(header) < 8:
                break
            atom_size, kind = struct.unpack(">I", header[:4])[0], header[4:8]
            if atom_size == 1:
                atom_size = struct.unpack(">Q", header[8:16])[0]
            if kind == b"moov":
                handle.seek(position)
                moov = handle.read(atom_size)
                index = moov.find(b"elst")
                while index >= 0:
                    body = moov[index + 4:]
                    version, count = body[0], struct.unpack(">I", body[4:8])[0]
                    entry_size, fmt = (20, ">qqhh") if version else (12, ">Iihh")
                    for i in range(count):
                        entry = struct.unpack(fmt, body[8 + i * entry_size:8 + (i + 1) * entry_size])
                        offsets.append({"segment_duration": entry[0], "media_time": entry[1],
                                        "media_time_frames_at_%d" % timescale: entry[1] / (timescale * 1001 / 30000)})
                    index = moov.find(b"elst", index + 4)
                break
            if atom_size <= 0:
                break
            position += atom_size
    return offsets


def calibration_candidate_check(stream, tags, calibration):
    width, height = int(stream["width"]), int(stream["height"])
    fps = float(Fraction(stream["avg_frame_rate"]))
    model_tags = {k: v for k, v in tags.items() if any(s in k.lower() for s in ("model", "make", "device", "lens", "encoder", "comment"))}
    return {"resolution": [width, height], "resolution_matches": [width, height] == list(calibration["resolution"]),
            "fps": fps, "fps_matches": abs(fps - calibration["fps"]) < 1e-3,
            "camera_model_tags": model_tags,
            "lens_zoom_crop_verified": False,
            "calibration_validated": False,
            "note": "Equality of resolution/FPS is necessary, not sufficient. Lens, zoom, crop mode and the"
                    " calibration session are unknown; reprojection of reviewed drone0 boxes is required."}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--video", type=Path, required=True)
    parser.add_argument("--extraction-receipt", type=Path, required=True)
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--calibration", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--ffprobe", required=True)
    parser.add_argument("--ffmpeg", required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise ValueError("refusing existing output; preserve previous verification")
    extraction = json.loads(args.extraction_receipt.read_text())
    if extraction["source_commit"] != COMMIT:
        raise ValueError("source commit mismatch")
    video_sha = digests(args.video)[1]
    if video_sha != extraction["video_sha256"] or args.video.stat().st_size != extraction["video_bytes"]:
        raise ValueError("video does not match extraction receipt")
    calibration = json.loads(args.calibration.read_text())
    frames = numeric_table(args.dataset / "dataset5/videos/cam0/cam0_frame_ts.txt", 2)
    sync = {int(r[0]): (r[1], r[2]) for r in numeric_table(args.dataset / "dataset5/raw-data/sync_coefficients_cam2pc.txt", 3)}
    scale, shift = sync[0]
    info = probe(args.ffprobe, args.video, "-show_streams", "-show_format")
    streams = [s for s in info["streams"] if s.get("codec_type") == "video"]
    if len(streams) != 1:
        raise ValueError("expected exactly one video stream")
    stream = streams[0]
    tags = dict(info["format"].get("tags", {}), **stream.get("tags", {}))
    pts = packet_times(args.ffprobe, args.video)
    decoded = decoded_frame_count(args.ffmpeg, args.video)
    published = [r[1] for r in frames]
    period = 1.0 / float(Fraction(stream["avg_frame_rate"]))
    counts = {"published_timestamp_rows": len(published), "container_packets": len(pts),
              "declared_nb_frames": int(stream.get("nb_frames", -1)), "decoded_frames": decoded}
    container_consistent = counts["container_packets"] == counts["declared_nb_frames"] == counts["decoded_frames"]
    if not container_consistent:
        raise ValueError("container/decoder frame count disagreement: " + json.dumps(counts))
    count_match = counts["container_packets"] == counts["published_timestamp_rows"]
    consistency = timestamp_consistency(pts, published, scale, shift, period)
    consistency["mp4_edit_lists"] = edit_list_offsets(args.video)
    fit = published_fit([r[0] for r in frames], published, period, sync)
    blockers = []
    if not count_match:
        blockers.append("published timestamp rows (%d) != container frames (%d); frame_id<->container index"
                        " alignment unresolved by one frame" % (len(published), len(pts)))
    if fit["closest_camera_scale"] != 0:
        blockers.append("published stamp slope matches cam%d Time_scale, not cam0; provenance of the"
                        " published stamps unresolved" % fit["closest_camera_scale"])
    status = "VIDEO_INTEGRITY_VERIFIED_CALIBRATION_AND_IDENTITY_PENDING" if not blockers else \
        "VIDEO_DECODES_TIMESTAMP_MAPPING_UNRESOLVED"
    result = {"status": status, "blockers": blockers, "source_commit": COMMIT,
              "video_sha256": video_sha, "frame_counts": counts, "frame_count_matches_published": count_match,
              "stream": {k: stream.get(k) for k in ("codec_name", "profile", "width", "height", "pix_fmt", "r_frame_rate",
                                                    "avg_frame_rate", "time_base", "duration", "nb_frames", "bit_rate")},
              "format_tags": tags, "sync_cam0": {"scale": scale, "shift_s": shift},
              "timestamp_consistency": consistency, "published_timestamp_fit": fit,
              "calibration_candidate": calibration_candidate_check(stream, tags, calibration),
              "extraction_receipt_sha256": digests(args.extraction_receipt)[1],
              "tool_sha256": digests(Path(__file__))[1],
              "runtime": {"python": platform.python_version(), "executable": sys.executable,
                          "ffprobe": subprocess.check_output([args.ffprobe, "-version"], text=True).splitlines()[0]}}
    write_json(args.output / "video_verification.json", result)
    print(json.dumps({k: v for k, v in result.items() if k not in ("format_tags",)}, indent=2))


if __name__ == "__main__":
    main()
