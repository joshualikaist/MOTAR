# CAM-S2a amendment 1 (2026-11-23 ~23:10 KST, before the run; the one smoke so far failed at task build)

Smoke `~/motar_tmp/cams2a_20261123/train_s1_smoke_220814/` (HEAD aafcb21) stopped while building the task:
`RuntimeError: D8 dynamic-mesh treatment does not implement detect-resolution decoupling; set NAVRL_DETECT_WIDTH/HEIGHT
equal to the camera resolution` (navrl_detector.attach_dynamic_mesh_treatment). No epoch ran.

Change: the camera render and the detection both run at **312 x 234** (`NAVRL_CAMERA_WIDTH/HEIGHT` =
`NAVRL_DETECT_WIDTH/HEIGHT`), the real 4:3 profile at half resolution, instead of detection at 624 x 468 over a 160 x 90
render. Reason: the configuration's measured cost (~1 ms/Mpx, ~84 B/px downstream of the ray-cast) puts 624 x 468 x 128
environments at ~3.1 GB of extra GPU memory on the 8 GB RTX 3070, an out-of-memory risk for an unattended overnight
run; 312 x 234 is ~0.8 GB. The 4:3 render also matches the 66.7 x 52.6 deg field of view (square pixels), which the
16:9 160 x 90 render did not. At 312 px over 66.7 deg (fx ~236 px) the 0.28 m quadrotor spans ~3.3 px at 20 m.
Full 624 x 468 moves to CAM-S2b (limit). Nothing else changes: settings, stop rule, labels and scope stay as
preregistered. The smoke is repeated (development only).
