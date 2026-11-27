# Camera runtime independent review r1

Reviewer: fresh-context Codex agent `/root/camera_independent_review`, GPT-6 family; exact deployment identifier not exposed. Date: 2026-11-11. Usage state: USAGE_UNKNOWN, inherited root session a2a54346-d85b-405e-b76c-4cd51608e939. Worktree: `/home/fair/workspaces/aerial_gym_ws/src/MOTAR-camonly`, branch `camonly/dev`, HEAD/base `eefb832e1c4fc6de8e597e990aa9cd58f2228079`.

**Initial verdict: AMEND_FIRST (CPU source preparation only).** Two required timestamp guards were missing from the reviewed snapshot. No simulator, GPU, sensor, FC or flight execution occurred. This report records initial findings; any repair review appears in an addendum and does not replace original evidence.

## Required findings

1. **P1 — Stale estimator telemetry can publish the first command.** `tools/sitl_camera.py:83-104` reads only heartbeat monotonic receipt time. It does not require or check position/attitude receipt times. `_packet_times` at lines118-120 compares internally matching sample clocks, which cannot show that every sample stopped advancing. A fresh heartbeat alongside position/attitude received90 seconds ago and an equally old cached RGB-D packet passes all current guards; `run_control` publishes one command. Independent `probe_controls.py` runs the immutable initial source snapshot and reproduces one accepted positive and this stale negative, each with one command. Required fix: validate monotonic receive freshness for both estimator messages, in the atomic snapshot when available and before/post inference; reject missing, old and future receipts. Tests must preserve fresh positives and ensure no policy call/command for stale inputs. Boot timestamps and a fresh heartbeat are insufficient receipt-age evidence.
2. **P1 — Attitude capture skew bypasses the25ms limit.** `tools/sitl_camera.py:103` allows position/attitude difference up to150ms, and `_packet_times` lines123-124 checks only RGB/depth/position. Attitude is then removed from the runtime state. A100ms-old attitude next to matching fresh position/RGB/depth is accepted and publishes one command; independent probe reproduces this. Required fix: include translated attitude capture timestamp in the25ms bound; retain valid tilted-attitude controls.

## Independent validation

All commands used hidden CUDA, PYTHONNOUSERSITE=1, numerical thread count1, nice10, cores12,13,16GiB address-space cap, timeout300s plus10s kill, `/home/fair/miniconda3/envs/aerialgym/bin/python -B`. All outputs remain in this audit directory.

- `test_camera*.py`:22 tests PASS, including separate fresh-process inference for both real pinned trained seeds, explicit no-LiDAR socket packets, legacy default positive/negative cases, calibration/invalid-mask geometry and timing rejections.
- `test_sitl_camera.py`:6 tests PASS. These original tests do not detect the two findings above.
- `test_depth_validity.py`:21 tests PASS.
- `probe_controls.py`: fresh positive1 command; each known-bad case above incorrectly1 command on the preserved initial source. Full NED/FRD->ENU/FLU quaternion conversion independently matched matrix composition at3 nonzero roll/pitch/yaw orientations; largest absolute element error2.220446049250313e-16.
- `probe_trained_governor.py`: AST-extracted the actual unchanged production `NavRLTask._apply_sensor_speed_governor` method and wrapped it with the actual trained `no_lidar` governor adapter. Its executed XY output matched runtime exactly for4 nonvacuous actions (forward near obstacle, outside-FOV lateral, reverse diagonal and zero), and previous-action values matched within5e-8. Actual runtime history stride5, step_dt0.1, robot history5. A nonfinite actor output after perception advance raises, and a subsequent call stays latched until reset. This probe uses the real seed1 checkpoint on CPU but substitutes explicit action outputs to exercise otherwise uncontrolled branches.

## Scope and limitations

The source explicitly selects RGB-D-only readiness while legacy defaults still require LiDAR. The model generator removes the LiDAR sensor in that explicit mode and the bridge accepts an absent LiDAR element; wire-v1 absence retains zero dimensions, empty payload and-1 timestamp. Live C++ compilation/plugin operation remain NOT_TESTED. Frozen F actor/checkpoint/default sensor mode are not replaced.

The runtime binds training report/checkpoint/manifest hashes for two distinct CAM-K2v3 seed candidates; no superior-seed claim is supported. The898-D observation/no_lidar storage,10Hz step,66x54deg camera intrinsics,5-step/0.5s history and trained governor/action feedback paths were inspected. Runtime inputs are RGB, optical-z depth with explicit invalid mask and estimator fields; no actual LiDAR, target truth, simulator construction or synthetic depth noise is used. Invalid/disoccluded pixels remain explicit and dense-invalid handling calls the shared production kernel. Body-to-yaw projection is geometric, with a measured-point depth buffer for tilted/translating cameras.

CPU finite-output, source binding and fake-interface checks are source preparation evidence. They do not establish physical OAK-D calibration/alignment/timing, target appearance robustness,10Hz compute budget on a companion, body-to-yaw sensor equivalence, closed-loop continuous control, FC safety behavior or flight readiness. Caller-owned startup, clock-binding evidence, transport serialization/cleanup and stop response remain deployment responsibilities. No live claim is made.

## Evidence and traceability

`source_sha256.json` binds12 inspected source/test snapshots under `source_snapshot/`; `source_diff.patch` preserves the actual initial scoped diff and `git_status.txt` records pre-existing dirty paths. `kernel_before.json` records regex count0 before CPU validation. Final host counter and evidence SHA manifest are recorded at closeout.

Git Traceability: branch camonly/dev; base/HEAD eefb832e1c4fc6de8e597e990aa9cd58f2228079; new commit NONE (read-only review, explicit prohibition); repo changed files NONE by reviewer; all repository dirty/untracked paths belong to existing/root work and are preserved in git_status.txt; tests as above; working tree pre-existing dirty; Remote Push NO; Main Merge NO; raw/checkpoint bytes read only and not modified. Reviewer writes only this own external audit directory. No owned live jobs remain after commands complete.
