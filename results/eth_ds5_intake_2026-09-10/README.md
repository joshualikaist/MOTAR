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

## Root causes of the two timestamp blockers (2026-09-10, investigated)

### Why 20969 published rows for 20970 container frames

The MP4 video track carries an edit list `media_time = 1001` at a 30000 timescale, i.e. exactly one
frame period, while the audio and metadata tracks carry `media_time = 0`. The edit therefore says
presentation begins one frame into the media, so a reader honouring it presents 20969 frames. Upstream
generated the file with a MATLAB `VideoReader` `readFrame` loop, which honours edit lists, and got
20969 rows.

ffmpeg and OpenCV do **not** drop that frame: full decodes return 20970 both by default and with
`-ignore_editlist 1`, and OpenCV's first three frames are byte-identical to ffmpeg's default output.
So the missing row is at one end of the container, but three candidate mappings remain, and the
timestamps cannot separate them because each is a whole-frame relabelling:

| candidate | container index of `frame_id` | evidence |
|---|---|---|
| edit list skips the first frame | `frame_id` | container edit list, quantitatively exact |
| reader dropped the last frame | `frame_id - 1` | documented MATLAB `readFrame`/`NumFrames` discrepancy, no container evidence |
| `CurrentTime` is the next frame's time | `frame_id + 1` | MathWorks semantics with the fitted origin; predicts one row too few, kept only as a bound |

The pilot images were rendered with `frame_id - 1`. One frame of timing error displaces drone0 in the
image by a median of 2.2 px, a 95th percentile of 7.6 px and at most 20.3 px, so the mapping is
immaterial for deciding visibility and material for measurement.

### Why the stamps carry a scale that is not cam0's

The published stamps are exactly affine in frame id (maximum residual 33 ns):
`t(j) = 0.033367703956 * j + 10.551275280`. Writing this as `scale * (j + origin) * period + shift`
and testing every row of `sync_coefficients_cam2pc.txt`:

- the slope matches **cam1's** `Time_scale` (1.000031087) to 5.9e-10; every other camera is off by at
  least 2.1e-6;
- the intercept only yields a near-integer frame origin for **cam0's** `Time_shift`: origin 1.9486,
  i.e. 0.051 frames from 2. Every other row lands 0.25 to 0.46 frames from an integer.
- at origin 2 the implied shift is 10.48454 s, which is 1.7 ms from cam0's current 10.48625553 s.

The stamps were committed on 2021-03-08; the coefficient table was revised on 2022-02-07 and the stamps
were never regenerated. The pre-revision table displayed rounded values `1.000` and `10.48`, which are
consistent with the pair the stamps actually used (1.000031087, 10.48454) and inconsistent with origins
1 or 3 (which would need 10.518 or 10.451). So the file predates the revision rather than being
internally corrupt.

Practical size of the discrepancy, recomputing the stamps from cam0's current coefficients:

| assumed origin | change over the pose overlap |
|---|---|
| 0 | −69.7 to −65.0 ms (−2.09 to −1.95 frames) |
| 1 | −36.4 to −31.7 ms (−1.09 to −0.95 frames) |
| **2 (favoured)** | **−3.0 to +1.7 ms (−0.09 to +0.05 frames)** |
| 3 | +30.4 to +35.1 ms (+0.91 to +1.05 frames) |

The scale difference alone contributes at most 4.7 ms across the whole pose overlap. So the coefficient
revision is harmless once the integer origin is fixed; the residual uncertainty is the integer, worth
at most about two frames. The reprojection check searches whole-frame shifts of −3…+3, which covers
every combination above, and its 4 px RMS gate is below the 7.6 px that one frame of error produces.

## Findings that still block measurement

1. **Frame/time alignment**: three candidate mappings, unresolved without reviewed boxes (above).
2. **Identity**: three drones flew and upstream publishes no 2D labels for ds5. GT is drone0 (Pixhawk)
   only. GT places drone0 5.8 m from cam0 on the ground until about 30 s, then 5–108 m away.
3. **Calibration candidate**: resolution 1920x1080 and 29.97 fps match `calibration/sony5100/sony5100.json`;
   the file carries no camera-model, lens or zoom tag (XAVC brand only) and ds5 publishes no per-camera
   calibration. Compatibility is testable only by reprojecting reviewed drone0 boxes
   (`tools/check_eth_ds5_reprojection.py`; thresholds fixed before any box existed: pixel RMS < 4 px,
   PnP centre within 2 m of the surveyed cam0 position, at least 6 boxes, shifts −3…+3 frames).
4. **TrackingStatus** is the Leica total-station status (0 fine, 1 warning that fast motion may reduce
   accuracy, 2 lost and already filtered upstream), not a visibility flag. Attitude is the ArduPilot
   EKF roll/pitch/yaw, body to local NED, while positions are ENU. Both stay uninterpreted here.

## Local environment notes

Extraction used p7zip 16.02 from Ubuntu `p7zip-full_16.02+dfsg-7build1_amd64.deb`
(SHA-256 `efc2d2795fe6c707183a4b7f4146477fc410c478131bf451914d534246d06896`) unpacked under `/tmp`
without installation. ffprobe/ffmpeg 7.1 and OpenCV 4.13 come from the `aerialgym` conda environment.

A previous session had already extracted the same video to `datasets/eth_ds5_extracted_cam0/`; this
session extracted it again to `datasets/eth_ds5_cam0_extracted/`. Both files are byte-identical
(SHA-256 `d3e6727e…c1ec4`, confirmed by each directory's own receipt), so one of the two directories is
a redundant 4.3 GB copy. Nothing was deleted. To free room for the extraction only the conda package
tarball cache and the pip cache were purged; no dataset, result or run file was removed.

## Next (E2 completion, not E3)

Human review of the 36 panels → filled `review_template.csv` → `check_eth_ds5_reprojection.py`.
Only if that reports `CALIBRATION_CANDIDATE_CONSISTENT` for some alignment shift may dense segments
be registered for E3, and the chosen shift must then be preregistered, not tuned.
