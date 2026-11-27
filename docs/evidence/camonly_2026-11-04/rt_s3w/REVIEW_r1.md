# RT-S3W review r1 — 4f6b709 (base ab39d18), camonly/s2b

Reviewer: Claude Opus 5.5 (claude-opus-5-5), fresh context, read-only, CPU only (cores 6,7,12,13, nice 10).

## Probes (this folder)
- probe_parity.py / .out: 7 independent analytic scenes x 12 steps (target at left/right FOV edge, target occluding a
  bar, <MinZ bar + >MaxZ wall, 12 m wall, 3 cm speckle sphere + bars at +/-33 deg, empty sky), runtime-only holes
  (10-row top band + 5 % random) mirrored into the training frame as navrl_cams3.apply_measured does.
  Full 898-D obs runtime vs training path: max |diff| <= 4.3e-7 in every scene; previous-action mismatches 0; yaw err 0;
  numpy sector rule vs training sector: 0.0 on 982 target-free bins; S2b column rule differs (1.0) in 6/6 non-empty
  scenes, 0.0 in empty sky (control); runtime vs training sector no-return class: no mismatch.
- probe_failclosed.py / .out: control accepted (k 24, 1.5708); inherited NAVRL_DEPTH_MEAS=0,
  NAVRL_DEPTH_MEAS_TOP_ROWS, NAVRL_CAMS3_SECTOR_K=1, cams3.sector_k edit, measured_depth_on_vehicle true, yaw 1.0472
  edit: all REFUSED.
- tests_related.log: cams3w 15 OK (no skips), cams2 27 + CudaV2Equivalence GPU_REQUIRED error (as recorded),
  contract 8, runtime 7, cams3 20, fixes 20, fixes_r2 22, unit_b 28, repair_r1 31, outdoor_r1 86 (10 xfail).
- Full-suite claim: failing sets of tests_full.log (4648) and tests_full_amend1.log (4633) identical, 9 F / 8 E.
- Bundle: pth 2626a4b2 = run checkpoint = training checkpoint_sha256*.txt; npz == checkpoint
  _orig_mod.running_mean_std mean/var/count exactly, eps 1e-5, clip 5; receipt 3ae14489 = training receipt;
  checkpoint env_state cfg_yaw_rate_max 1.5708. Exporter re-run on the CAM-S2a pth reproduces 77478ece byte for byte.
- navrl_cams3.py sha 96012f0a = receipt cams3_sha256; all 31 sources of contract_s3w_host.json match HEAD.
- Install order: runtime = cams2 apply_config -> ablation.install_perception(no_lidar) -> install_sector(k) — same
  relative order and parameters as realveh_entry (ablation -> cams3_sector).

## Findings
P1: none. P2: none.
- P3-1 handoff README §2/§4: the trained invalid pattern includes a 10-row top band (100 % invalid in the recordings,
  tools/cams3_build_mask_library.py:26). Vehicle-off is justified only if the Jetson's OAK-D depth config reproduces
  it; nothing checks this. Log the trained-grid top-10-row invalid share and the overall invalid share in the shadow test.
- P3-2 README §4: say that cams3_sector_ranges_m >= 10 m means no return (20 m = sensor invalid, 12 m = target carve).
- P3-3 README / Jetson: CAM-S2b contracts built before 4f6b709 now fail --check (bound sources changed); say they must
  be rebuilt from this tree.
- P3-4 camera_policy_runtime.py _setup_cams3 measured-depth env checks are unreachable behind the inherited-env
  collision check (harmless defence in depth).
- P3-5 parity (test and this probe) builds the "training" perception from the runtime's own task config, so it does
  not test realveh_entry's camtrain/cams2 config against the runtime's (pre-existing CAM-S2 P2-6 scope; recorded as open).

## Verdict
GO for a Jetson desk / hand-held shadow test (no flight); P3-1..3 are README additions worth doing before it.
