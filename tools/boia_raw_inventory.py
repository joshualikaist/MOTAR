"""Read-only representative raw field inventory: ZIP/NPY headers and bounded JSON only."""
import argparse
import ast
import hashlib
import json
from pathlib import Path
import shutil
import struct
import zipfile

ROOT = Path(__file__).resolve().parents[1]


def npz_headers(path):
    """Do not extract members, instantiate arrays, load pickles or read their payloads."""
    path = Path(path)
    if not path.is_file():
        return {"path": str(path), "status": "MISSING", "arrays": {}}
    arrays = {}
    with zipfile.ZipFile(path) as archive:
        if len(archive.infolist()) > 1024:
            raise ValueError("representative NPZ has unexpectedly many members")
        for member in archive.infolist():
            if not member.filename.endswith(".npy"):
                raise ValueError("unexpected non-NPY archive member")
            with archive.open(member) as handle:
                if handle.read(6) != b"\x93NUMPY":
                    raise ValueError("invalid NPY magic")
                version = tuple(handle.read(2))
                if version not in ((1, 0), (2, 0), (3, 0)):
                    raise ValueError("unsupported NPY version")
                count = 2 if version == (1, 0) else 4
                length = struct.unpack("<H" if count == 2 else "<I", handle.read(count))[0]
                if length > 65536:
                    raise ValueError("NPY header exceeds declared limit")
                raw = handle.read(length)
                if len(raw) != length:
                    raise ValueError("truncated NPY header")
                header = ast.literal_eval(raw.decode("utf8" if version == (3, 0) else "latin1"))
                arrays[member.filename[:-4]] = {"shape": list(header["shape"]),
                    "dtype": header["descr"], "fortran_order": header["fortran_order"],
                    "header_sha256": hashlib.sha256(raw).hexdigest()}
    return {"path": str(path), "status": "HEADER_ONLY", "bytes": path.stat().st_size,
            "arrays": arrays, "array_payload_read": False}


def small_json(path):
    path = Path(path)
    if not path.is_file():
        return {"status": "MISSING", "path": str(path)}
    if path.stat().st_size > 2_000_000:
        raise ValueError("metadata exceeds the declared 2 MB limit")
    return json.loads(path.read_text())


def build_inventory(raw_root=None):
    base = Path(raw_root) if raw_root else ROOT.parent / "aerial_gym_simulator/results"
    p03 = base / "h_r3_p03_qualification_v1/HR3-P03-Q01/long_B0"
    names = ["initial"] + ["step%04d" % k for k in (0, 1, 2, 3, 4, 38, 39, 40, 41, 42, 598, 599)]
    dense = npz_headers(p03 / "h_r3_p03_trace.npz")
    snapshots = [npz_headers(p03 / "snapshots" / (name + ".npz")) for name in names]
    occ = []
    for run, folder, seed in (("OCC-P2", "occlusion_confirmatory_v1/OCC-P2", 4301),
                              ("OCC-P1", "occlusion_pilot_v1/OCC-P1_rerun1", 4201)):
        for motion, arm, model in (("moving", "A3", "E2_obstacle_aware"), ("static", "A1", "E0_static")):
            cell = base / folder / motion / arm / ("%s__160bars__seed%d" % (model, seed))
            summary = small_json(cell / "occlusion_trace_summary.json")
            frames = cell / "occlusion_frames.bin"
            occ.append({"run": run, "motion": motion, "cell": str(cell), "frame_summary": summary,
                "frames_stat_bytes": frames.stat().st_size if frames.is_file() else None,
                "frame_payload_read": False, "layouts": npz_headers(cell / "occlusion_layouts.npz"),
                "field_scope": {"scan_state_cov_action": "NOT_IN_FRAME_SCHEMA",
                    "camera_validity": "flags bit1; motar_eval/occlusion_trace.py:35",
                    "own_pose_velocity": "p_pos/p_vel/p_quat/p_vehicle_quat in frame_dtype",
                    "termination": "NOT_IN_FRAME_SCHEMA; episode files not read"}
                    if summary.get("schema") == "motar.occlusion-trace.v1"
                    else {"status": "MISSING_OR_UNRECOGNIZED_SUMMARY"}})
    fields = dense["arrays"]
    sparse = set().union(*(record["arrays"] for record in snapshots))
    aliases = {"lidar_scan": ("lidar",), "tracker_state": ("tracker_state",),
        "tracker_covariance": ("tracker_cov",), "camera_flag": ("camera",),
        "own_position": ("before_pos", "actual_pos", "robot_pos"), "own_velocity": ("pre_vel", "actual_vel"),
        "own_orientation": ("vehicle_quat", "robot_orientation"), "policy_action": ("action",),
        "terminal_flags": ("reset", "captured", "crashed", "timeout")}
    coverage = {key: {"dense_fields": [name for name in values if name in fields],
                       "snapshot_fields": [name for name in values if name in sparse]}
                for key, values in aliases.items()}
    coverage["own_orientation"]["snapshot_container_fields"] = [name for name in ("root_state",) if name in sparse]
    return {"schema": "motar.boia.raw-field-inventory.v1", "scope": "REPRESENTATIVE_HEADERS_ONLY",
        "raw_root": str(base), "p03_trace": dense, "p03_snapshot_names": names,
        "p03_snapshots": snapshots, "p03_metadata": small_json(p03 / "h_r3_p03_meta.json"),
        "coverage": coverage, "occ_representatives": occ, "payload_integrity_verified": False,
        "field_time_alignment": "NOT_TESTED; pre/post step fields must not be silently interchanged",
        "dense_orientation_reconstruction": "NOT_TESTED; snapshot root_state is only a container",
        "conclusions": ["Presence is reported by the actual header coverage/status; missing representatives cannot establish absence elsewhere.",
          "Digests, belief_pos and actor_obs do not reconstruct omitted online inputs.",
          "OCC flags expose camera/track validity; their bitfield is not a recorded scan or tracker covariance.",
          "Root actor pose can be derived only in a future separately authorized raw-calibration step.",
          "No NEES/NIS/reliability/false-alarm or performance conclusion is computed."]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--raw-root", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    result = build_inventory(args.raw_root)
    text = json.dumps(result, indent=2, ensure_ascii=False) + "\n"
    if args.output is None:
        print(text, end="")
        return
    output = args.output.resolve()
    canonical = ROOT / "docs/research/occlusion_algorithm_v1/raw_inventory.json"
    if output != canonical and Path("/tmp/codex_occ") not in output.parents:
        parser.error("output must be the new canonical inventory or under /tmp/codex_occ/")
    if shutil.disk_usage(ROOT).free < 3_000_000_000:
        raise RuntimeError("free disk is below the user-approved 3 GB floor")
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("x") as handle:
        handle.write(text)


if __name__ == "__main__":
    main()
