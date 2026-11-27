# Outdoor hand-held depth recordings of 2026-11-26: obstacle-scan analysis

Offline CPU analysis (2026-11-27) of the vehicle team's eight hand-held OAK-D outdoor recordings (trees, a pole,
a target drone, sky). It replays the onboard camera runtime's projection (CAM-S2) and the S3-W sector rule
(k = 24) on recorded depth. No policy ran and nothing flew.

- [`REPORT.md`](REPORT.md): findings and caveats, as saved on 2026-11-27 (source tree `7a2e9c8`).
- [`sky_scan_analysis.json`](sky_scan_analysis.json): all rates by ground-filter height, ceiling and threshold.
- `attitude_check.json`, `shift_test.json`: hand-held attitude and sub-bin yaw-shift checks.
- `source_sha256.txt`: hashes of the analysed sources. Raw recordings stay outside Git
  (`~/motar_tmp/jetson_in_20261127_outdoor/`).

Later state: REPORT.md says the runtime had no hook for the sector rule. RT-S3W (`4f6b709`, review r1 GO for a
shadow test only, [`../rt_s3w/REVIEW_r1.md`](../rt_s3w/REVIEW_r1.md)) added it after this analysis. Two visual
odometry jumps in the recordings are treated as measurement error (user, 2026-11-27). Ground handling and sky
false-near returns remain open.
