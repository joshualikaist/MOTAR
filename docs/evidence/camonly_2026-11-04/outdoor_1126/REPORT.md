# Outdoor 11-26 recordings: obstacle scan with S2b-FT native (a) vs S3-W sector k=24 (b) — 2026-11-27
Analysis by a delegated Claude subagent (CPU, read-only, sources = HEAD 7a2e9c8, hashes in source_sha256.txt).
Numbers: sky_scan_analysis.json. Saved by the main session from the subagent's final report (its write was refused).

- Runtime (CAM-S2): v2 min-valid projection with estimator roll/pitch; invalid (0 or < 0.315 m) and > 20 m -> no return;
  floor filter ON (0.10 m above asserted floor); ceiling filter OFF by default; no sky rule (outdoor profile refused for
  CAM-S2); column value >= 10 m -> 12 m no return. Valid fake sky depth passes through; only the 10 m cut removes it.
  The runtime has no hook for the cams3 sector rule (S3-W flight path NOT_IMPLEMENTED).
- Main finding: ground. Floor filter at assumed 1.5 m: 38.7 % (a) / 36.8 % (b) of frame-bins read < 5 m only because of
  ground; 97 of 239 frames have all 13 bins < 5 m. By assumed height: 1.3 m 18.5 %, 1.4 m 28.0 %, 1.6 m 49.2 %,
  1.7 m 54.8 %. Handheld tilt median 4.5 deg, p95 8.3 deg (roll likely carries an offset, NOT_MEASURED).
- Sky phantoms (heuristic sky mask; 3,107 frame-bins): default floor filter T3/T5/T7 1.29/2.67/2.45 % (a) vs
  1.45/2.90/3.15 % (b); ground suppressed T5/T7 3.25/3.83 % (a) vs 3.54/4.70 % (b). Backlit pole/treeB 9-11 % at T5.
  Median phantom value 3.3-4.2 m, elevation +22 deg. Visual check of 6: 1 stadium roof mislabelled sky, 4 thin branches
  against bright sky, 1 glare. Ceiling 3.0 / 2.5 m cuts T5 phantoms by ~25 / ~50 % but also removes real canopy.
- Taped stops (ground suppressed): trunks/pole read within ~0.3 m of their own depth (tape vs depth gap at 1-2 m, cause
  NOT_MEASURED). Sub-bin yaw shift test: trunk depth in the centre bin 96.6 % (a) vs 100 % (b) over 10 stops.
- Drone target: thin drone is seen through: 3 m -> 4.5/4.4 m, 5 m -> 7.5/7.0 m ((a)/(b)).
- Caveats: hand-built contract, assumed 1.5 m camera height, accelerometer attitude, yaw by image shift, heuristic sky
  mask. NOT_MEASURED: policy output, true camera height, roll offset, treeB distances, closed-loop effect.
