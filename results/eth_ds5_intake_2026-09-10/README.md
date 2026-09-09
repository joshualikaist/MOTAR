# ETH ds5 cam0 intake, E1–E2 (2026-09-10)

Status: **E2 material ready, gates NOT passed.** drone0 identity, frame-time alignment and the
Sony5100 calibration candidate remain unverified; E3 (error modelling) has not started.

| Receipt | Content | Result |
|---|---|---|
| `receipt.json`, `video_receipt.json` | E1 pinned files, Git blob SHA-1 + SHA-256, 43 split-archive parts (4,471,880,421 B) | verified |
| `extraction_receipt.json` | cam0.mp4 4,520,030,301 B, SHA-256 `d3e6727e…c1ec4`, 7-Zip CRC OK | extracted outside Git: `datasets/eth_ds5_cam0_extracted/` |
| `video_verification/video_verification.json` | h264 1920x1080 30000/1001, 20970 packets = 20970 decoded = nb_frames | **20969 published timestamp rows: off by one** |
| `review/pilot_frames_receipt.json` | 36 clean PNGs (stride 150 inside pose overlap), hash-pinned | rendered with `--allow-count-mismatch` |
| `review/review_material_receipt.json` | per-frame GT range/bearing/elevation/speed, motion-cue hints, panels, contact sheet | identity unverified |
| `review/review_template.csv` | empty reviewer table | pending human review |

## Findings that block measurement

1. **Frame count**: container 20970 frames, `cam0_frame_ts.txt` 20969 rows. Upstream generated the
   file with MATLAB `VideoReader` (`readFrame` loop, `CurrentTime` after each read, 1-based ids;
   `drone-tracking-toolkits/codes/postprocess/signal/video_ts.m`). The MP4 video track carries an
   edit list with a one-frame media offset (1001/30000 s), which ffmpeg applies and MATLAB backends
   may not. Which container frame row *k* denotes is therefore uncertain by 1–3 frames (≤ 100 ms).
2. **Scale provenance**: published stamps are exactly linear in frame id (residual 3e-8 s) with slope
   1.0000310876 x (1001/30000 s). That equals cam1's current `Time_scale` (1.000031087), not cam0's
   (1.000004517); the sync file at the time the stamps were committed (2021-03-08) held rounded 1.000.
   The coefficients were revised upstream on 2022-02-07 without regenerating the stamps. Drift between
   the two scales is 18.6 ms over the full video and ≤ 4.7 ms inside the pose overlap (≤ 187.6 s).
   First-row offset relative to cam0's current shift: 2.95 frames (alignment 0) or 1.95 (alignment +1).
3. **Calibration candidate**: resolution 1920x1080 and 29.97 fps match `calibration/sony5100/sony5100.json`;
   the file carries no camera-model, lens or zoom tag (XAVC brand only). ds5 has no per-camera calibration
   folder upstream. Compatibility is testable only by reprojecting reviewed drone0 boxes
   (`tools/check_eth_ds5_reprojection.py`, thresholds fixed before any box exists: pixel RMS < 4 px,
   PnP centre within 2 m of the surveyed cam0 position, ≥ 6 boxes, alignment shifts −3…+3 frames).
4. **Identity**: three drones flew; no 2D labels exist upstream for ds5. GT is drone0 (Pixhawk) only.
   GT says drone0 sat 5.8 m from cam0 on the ground until ~30 s and then flew 5–108 m away.
5. **TrackingStatus** (from the upstream toolkit and thesis): Leica total-station status, 0 fine,
   1 warning (accuracy may be reduced by fast motion), 2 lost (already filtered out). Not a visibility flag.
   Attitude: ArduPilot EKF roll/pitch/yaw, body → local NED, while positions are ENU. Uninterpreted here.

## Local environment notes

Extraction used p7zip 16.02 from Ubuntu `p7zip-full_16.02+dfsg-7build1_amd64.deb`
(SHA-256 `efc2d2795fe6c707183a4b7f4146477fc410c478131bf451914d534246d06896`, binary
`9e912f90…67ada7`) unpacked under `/tmp` without installation. To make room (4.0 GB free before),
only the conda package tarball cache and the pip cache were purged; no dataset, result or run file
was deleted. ffprobe/ffmpeg 7.1 and OpenCV 4.13 come from the `aerialgym` conda environment.

## Next (E2 completion, not E3)

Human review of the 36 panels → filled `review_template.csv` → `check_eth_ds5_reprojection.py`.
Only if that reports `CALIBRATION_CANDIDATE_CONSISTENT` for some alignment shift may dense segments
be registered for E3, and the chosen shift must then be preregistered, not tuned.
