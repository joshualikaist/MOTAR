# v2 re-review after full-suite failures

Reviewer: Codex `/root/camera_independent_review`, GPT-6 family; exact deployment identifier unavailable. Same camonly/dev worktree/base/USAGE_UNKNOWN as prior reports. **v2 source repair review: PASS_WITH_LIMITS — CPU source preparation only, required repairs remaining0 in this scope.** Broad acceptance remains conditional on the parent's fresh full PUBLIC_CPU result for these v2 bytes. This reviewer does not claim that uncompleted suite PASS.

## Preserved v1 failure and scope correction

The parent's actual PUBLIC_CPU v1 log reports4010 tests,2 failures,0 errors,10 skipped and5 exclusions. I independently read the failing traces: (1) the real-vehicle isolation test's historical three-entry allowlist excluded the new explicit camera runtime, and (2) changing the byte-pinned historical S4 plugin violated the unchanged S4F1 SHA contract. The parent's broad suite found these failures; my earlier focused source review did not check that historical plugin byte pin. Initial source reviews, v1 tests/artifacts and failure logs retain their original bytes and scope; none are relabeled as passing full-suite evidence. public_cpu_v1_binding.json binds the original log by path/SHA. Parent source-bound artifacts_v1 must remain old-source evidence; v2 artifacts need fresh bindings.

## Actual repairs independently reviewed

- `tools/sitl_s4/motar_s4_bridge.cc` is byte-for-byte equal to HEAD and the historical pin SHA256661a4d71623aba293cb21feee569837bd5866c7f2357125f432b759196f8e5fa. No historical pin was rewritten.
- New `motar_rgbd_bridge.cc` contains the optional-LiDAR variant under distinct MotarRgbdBridge class, destructor, event/thread bindings, log prefix and registration. I compared its complete diff with the HEAD historical source: substantive sensor changes are the optional SDF lidar lookup and guard against connecting an absent lidar name; wire layout, timestamps, sensor copying and target handling remain the same. Existing wire-v1 absence tests still exercise zero dimensions/empty payload/-1 time.
- CMake retains the original motar_s4_bridge target/source/link rule and adds a separate motar_rgbd_bridge target. World generation selects libmotar_rgbd_bridge.so only for explicit rgbd mode; the default selects libmotar_s4_bridge.so and includes LiDAR. An independent AST-extracted HEAD function control proves default generated world bytes exactly match HEAD, paired with the distinct RGB-D variant positive. Original default model-generator byte comparison remains valid.
- `tests/test_realveh.py` adds only camera_policy_runtime.py as the fourth explicit ablation importer, positively asserts its discovery, and leaves vehicle-randomization/depth-model importer restrictions intact. Its fresh-process probe confirms importing camera runtime is inert (no aerial_gym/torch/Isaac Gym/Warp imports or environment changes), and prior production-package imports are refused before torch/wrappers.
- The policy contract now binds25 source files, including both plugin sources and CMake. A contract prepared for v1 must not silently validate after this change; new source-bound v2 contracts are required.

## Independent bounded CPU validation

Same controls as earlier: hidden CUDA, PYTHONNOUSERSITE=1, numerical threads1, nice10, cores12,13,16GiB address-space cap, timeout300s+kill10s, aerialgym Python-B.

- `test_realveh.TestIsolation` plus `test_sitl_s4f1.TestPluginPins`:6/6 PASS (`test_fullsuite_repairs_v2.log`).
- Camera tests30/30 PASS (`test_camera_v2.log`), including the unchanged real-checkpoint/reset/composition controls and9 sensor-mode wire/model tests with both plugin filenames.
- `probe_plugin_v2.json`: original plugin exact HEAD/pin, default world exact HEAD, separate RGB-D class/target all PASS.
- Parent-built v2 RGB-D shared library was inspected read-only with nm: RegisterPlugin and MotarRgbdBridge::Load/Connect symbols exist. v2_plugin_build_binding.json binds this artifact. I did not compile it, load it or execute it; successful build/symbol presence is not Gazebo runtime evidence.
- Kernel regex count remains0 (`kernel_v2_final.json`); no reviewer jobs remain running.

## Final v2 bindings and limits

`v2_source_sha256.json`/`v2_snapshot/` bind25 contract sources and10 relevant test files; manifest SHA256e6cf3d1a5205fc6b7f761ebd52b823d45dc4abc32f34c00603b24d43ea1fc507. These are the source bytes reviewed for the next full suite. Earlier final_snapshot/ remains immutable historical v1-reviewed evidence. Parent's separate session-helper usage-fit fix is outside this neural/sensor source review.

All physical limitations remain: actual SDK API execution/device access is untested; reported isolated SDK download timeouts are not a successful installation or hardware test. No measured camera configuration/alignment/clock binding is provided. SDK discovery/calibration/startup has no proven absolute deadline and must finish before OFFBOARD. Bootstrap red detector, full-resolution timing, physical sensor equivalence, real-loop control, target capture, ground handling, safety behavior and flight remain unqualified. No SITL/GPU/hardware/device load occurred in this review.

Git Traceability: camonly/dev; base/HEAD eefb832e1c4fc6de8e597e990aa9cd58f2228079; reviewer repo changes NONE; new commit NONE; full dirty inventory preserved in v2_git_status.txt; tests above; Remote Push NO; Main Merge NO; raw/checkpoints read only and not modified. Reviewer writes confined to this external audit directory.
