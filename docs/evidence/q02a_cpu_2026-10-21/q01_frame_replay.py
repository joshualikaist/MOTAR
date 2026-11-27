"""Q02-A smoke replay over the saved Q01-V2 sensor snapshots (CPU, read-only).

Usage: python -B docs/evidence/q02a_cpu_2026-10-21/q01_frame_replay.py <Q01 run root> <out.json>
Runs motar_eval.q02.evidence.estimate on every saved full packet, de-duplicated by content hash,
with the recorded camera detection flag and, separately, with that flag cleared to exercise the
tracker branch. No labels exist: the Q01 recorder keeps target truth only as hashes, so this is
SMOKE_NOT_VALIDATION. Nothing under the run root is written.
"""
import hashlib, json, sys, time
from dataclasses import asdict
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
from motar_eval.q02.contracts import Clock, EvidenceInput
from motar_eval.q02.evidence import estimate


def main(run_root, out):
    run_root, rows, seen = Path(run_root), [], {}
    for stage in ("S0", "S1_OFF", "S1_ON"):
        sensors = [json.loads(l) for l in (run_root / stage / "sensors.jsonl").read_text().splitlines()]
        for i, npz in enumerate(sorted((run_root / stage).glob("sensor_*.npz"))):
            values = dict(np.load(npz, allow_pickle=False))
            digest = hashlib.sha256(b"".join(values[k].tobytes() for k in sorted(values))).hexdigest()
            meta = sensors[i]  # snapshots are taken at the first sampled packets, in order
            ck = lambda t: Clock(t["capture_sim_s"], t["delivery_sim_s"], t["provenance"], t["frame"], t["depth_convention"])
            if digest in seen:
                seen[digest]["duplicates"].append("%s/%s" % (stage, npz.name)); continue
            out_row = dict(frame="%s/%s" % (stage, npz.name), key=meta["key"], content_sha256=digest, duplicates=[])
            for label, flag in (("recorded_flag", None), ("camera_flag_cleared", False)):
                v = dict(values)
                if flag is not None:
                    v["camera"] = np.zeros_like(v["camera"])
                inp = EvidenceInput.from_arrays(v, ck(meta["camera_time"]), ck(meta["lidar_time"]),
                                                meta["camera_time"]["decision_sim_s"], tuple(meta["calibration"]))
                t0 = time.perf_counter(); rec = estimate(inp); ms = (time.perf_counter() - t0) * 1000
                out_row[label] = dict(asdict(rec), cpu_ms=ms)
            seen[digest] = out_row; rows.append(out_row)
    result = dict(scope="SMOKE_NOT_VALIDATION: no occlusion labels; camera flag is oracle-grade in this profile",
                  run_root=str(run_root), unique_frames=len(rows), frames=rows)
    Path(out).write_text(json.dumps(result, indent=1, allow_nan=True) + "\n")
    for r in rows:
        print(r["frame"], r["key"]["action"], r["recorded_flag"]["hypothesis"], "| cleared:", r["camera_flag_cleared"]["hypothesis"],
              r["camera_flag_cleared"]["reasons"], "dups", len(r["duplicates"]))


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
