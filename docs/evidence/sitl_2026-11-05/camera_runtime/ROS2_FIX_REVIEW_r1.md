# Independent pre-commit review r1: camera-runtime deployment fixes r1 + r2 (2026-11-13)

Reviewer: Claude Code subagent on **Opus 5.5** (`claude-opus-5-5`), fresh context, read-only on the repository.
Worktree `/home/fair/workspaces/aerial_gym_ws/src/MOTAR-camonly`, branch `camonly/dev`, base `fc8aa4d`. Under review:
`git diff fc8aa4d` (16 tracked files) plus untracked `tests/test_camera_runtime_fixes.py` and
`tests/test_camera_runtime_fixes_r2.py`. Inputs: `ros2_runtime_verify_20261112/REPORT.md` (H1-H5), the builders'
`FIX_r1.md` and `FIX_r2.md` (treated as claims).

Nothing in the repository was edited, staged, committed or stashed. `git status --short` gave the same 18 paths
before and after (`git_status_before.txt`). Every Python run used `-B`, CPU only (`CUDA_VISIBLE_DEVICES=`),
`PYTHONNOUSERSITE=1`, `nice -n 10`. No GPU, simulator, ROS launch or hardware. The host fault count
(`journalctl -k -b 0 | grep -cE "Machine check|segfault|general protection"`) was 0 before and after; cpu4-5 were
offline. Usage: USAGE_UNKNOWN. All probes and outputs are in this directory (`probes/`). The base tree was
extracted with `git archive fc8aa4d ... | tar -x` into `base_fc8aa4d/`; the repository was not changed.

## Verdict: PASS_WITH_FIXES

The diff does what H1-H5 asked for in the cases its tests cover:
- the tilt phantom wall is gone;
- +inf, beyond-MaxZ and undistortion borders are no-return;
- textured and textureless floors and ceilings are removed;
- the goal range ignores substituted pixels;
- the ROS pose is slerped to the RGB time;
- the gateway clamps against a measured box and latches breaches;
- the ROS session waits outside the height band.

Nothing under `aerial_gym/` changed. The untilted no-floor observation is **bit-identical to the real fc8aa4d
runtime**. I checked 8 scenes with the floor filter on and off, plus a 0.5° z-buffer case.

Four P1 findings must be fixed, or explicitly accepted by the user with tests that pin the behaviour, before
commit:
- **F1:** the default floor filter erases a real obstacle closer than MinZ that does not reach camera height. This is
  a regression against R3.
- **F2:** the geofence's stopping guarantee fails on a diagonal approach to a corner.
- **F3:** a CameraInfo coverage mask can erase valid depth. This fails open.
- **F4:** the z-buffer's "any valid point wins" rule hides a speckled near surface in every non-level frame. It was
  pre-existing, but it now carries R1/R3 alone.

None is P0. The commit is local and the runtime is NOT_TESTED on hardware.

## Requirement check

| Req | Result | Basis |
|---|---|---|
| R1 tilt | PASS for clean invalid blobs. LIMITED by F4 (speckle under any tilt) | tests 1-4; probe C |
| R2 invalid kinds | PASS: +inf, z > MaxZ, OAK border and ROS CameraInfo border are no-return; -inf, NaN, z < MinZ stay sensor-invalid. F3: coverage also overrides valid depth | tests; probe B |
| R3 floor/ceiling | FAIL in part (F1): textured and textureless planes are removed, and side walls, poles and full-height near slabs are kept, but a near obstacle below the horizon band is erased. The ceiling has no default height: PASS | probe A, `compare_box.txt` |
| R4 goal range | PASS for the range. Side effect F5: a 0.4 m obstacle appears at the goal's bearing | `compare_rt.txt` |
| R5 pose time | PASS: slerp takes the short arc across ±π, refuses extrapolation and brackets over 0.15 s, and uses an exact sample as is | `probe_slerp.out.json`; tests |
| R6 geofence | PASS for the formula as specified (per component, inward and parallel unchanged, latch, reset, fail closed). FAIL for the stopping guarantee on diagonals with norm-limited braking (F2) | `probe_geofence.out.json` |
| R7 altitude gating | PASS: waits without a fault or a history advance, starts fresh, and stale streams, period, clock pause and malformed input still fault | r2 J tests (real seed-1 runtime) |
| R8 no other change | PASS: `git diff fc8aa4d -- aerial_gym` is empty, and the untilted no-floor observation is bit-identical to fc8aa4d (see *Train/deploy consistency*) | `compare_rt.txt` |

## Findings

### F1 — P1 — The plane rule erases real obstacles closer than MinZ that do not reach the horizon band
`tools/camera_policy_runtime.py:296-303` (floor), `:310-316` (ceiling); FIX_r1 decision 1.

**Mechanism.** A sensor-invalid pixel is "explained" by the floor whenever the floor would lie at optical z in
[MinZ, MaxZ] along its row. The content of the pixel plays no part.
- The lowest trained ray has v = 0.51, so `floor_z = h/v >= 2.0 m` at h = 1.03 m. The MinZ bound therefore never
  binds above a camera height of about 0.2 m.
- Every sensor-invalid pixel in rows with v ≥ h/MaxZ is removed: rows 50-89 at h = 1.03 m.
- A surface closer than MinZ survives only if it reaches the few rows just below the horizon. At 0.3 m it must
  come within about 1.5 cm of camera height.
- With the ceiling filter on, the same applies above the horizon.

**Failing input.** I ran `probes/probe_scan.py` (section A, production `preprocess`) and `probes/compare_box.txt`
(the real CAM-K2-v3 seed-1 runtime of fc8aa4d against the new one). The scene:
- a full-width box face 0.3 m ahead (optical), reported invalid as the OAK-D does below MinZ;
- a wall at 10 m behind it;
- a textured floor;
- body height 1.0 m, camera height 1.03 m.

| box top | path | base fc8aa4d | new, ground off | new, default contract (ground on) |
|---|---|---|---|---|
| 0.98 m | level (trained-equal) | 4800 near px; token 0.408 m @ −5°; cmd FLU (1.107, 1.666) | identical to base | **0 near px; column min 9.90 m; tokens 9.9 m; cmd (−1.284, 1.534)** |
| 0.98 m | OAK-like, pitch 1° (z-buffer) | 0.406 m, token 0.408 | identical | **0 near, 9.90 m** |
| 0.90 m | OAK-like, pitch 3° | 0.436 m, token 0.438 | 0.436 m | **0 near, 9.90 m** |
| 1.00 / 0.95 m | all three sources | 0.402-0.415 m on 160/160 columns | same | **0/160 columns < 1 m** |
| 1.20 m (control, crosses the band) | OAK-like, pitch 1° | 0.400 m | 0.400 m | 0.400 m (kept) |

**Why it matters.** This is exactly the regime blob_near exists for (`navrl_depth_validity.py:4-7`): the obstacle is
seen while it is farther than 0.4 m and vanishes in the last 0.4 m. Examples are a table edge, a rail or a box
whose top is a few cm below the camera. The vehicle body extends about ±0.1 m around the camera. No test covers it:
the near-slab positive control is full height.

**Suggested fix.** Row geometry alone cannot separate "textureless floor" from "near object below camera height".
Options:
- (a) Explain an invalid pixel by a plane only with local valid-plane support. For example, require removed-valid
  floor pixels in the same column within k rows, or a minimum share of the neighbourhood. A box face that hides
  the floor has none.
- (b) Put the invalid-pixel explanation behind its own contract flag, default OFF until a bench recording shows that
  textureless floors occur. The OAK-D Pro's IR projector usually gives floors texture. Valid-pixel floor removal stays
  ON.
- (c) Either way, record the user's choice of failure direction.

Add tests: box tops 1.00, 0.98 and 0.90 m at 0.3 m must read ≤ 0.45 m with the default contract. Keep the
textureless-floor case as the documented counterpart.

### F2 — P1 — The geofence stop formula assumes deceleration *a* on each axis at once; a diagonal corner approach overshoots
`tools/camera_ros2_px4.py:63-69` (`stop_speed`), `:126-147` (`Geofence.limit`). The docstring at `:8-17` claims "it
stops within d". The default at `:49` is the trained governor's brake, which is a norm along the command.

**Failing input** (`probes/probe_geofence.py`, production `Geofence.limit`): a 10 m box, latency 0.2 s, decel 2.961.
The vehicle is 1.5 m from both x_max and y_max and commands (2.5, 2.5).
- The published command is (2.4465, 2.4465), norm 3.46 m/s, which is inside the encoder's √2·2.5 envelope.
- It holds that command for τ, then brakes at |a| = 2.961 opposite its velocity. That is what a tilt-limited
  multirotor does: PX4's horizontal acceleration limit is a norm.
- It crosses **both walls by 0.419 m**. The formula's per-axis model gives 0.000.

Other cases:

| latency τ | distance d | overshoot |
|---|---|---|
| 0.2 s | 0.5 m | 0.105 m |
| 0.4 s | 1.5 m | 0.286 m |
| 0.4 s | 0.5 m | 0.057 m |

Single-axis control: 0 in both models.

The r2 tests (`stop_overshoot`, `test_narrow_box...`) are 1-D and use the formula's own dynamics, so they cannot see
this.

**Fix.**
- Scale the XY vector by one factor s ∈ [0, 1] such that, for every component toward a wall,
  `s|v_i|τ + s²|v_i||v|/(2a) ≤ d_i`. Solve the quadratic per axis and take the minimum. This keeps the policy's
  direction.
- Or, conservatively, use a/√2 per axis.
- Document `decel_mps2` as the horizontal deceleration norm.
- Add a 2-D corner test with norm-limited braking and a single-axis control.

### F3 — P1 — Coverage overrides valid depth, so a mismatched coverage mask silently erases real returns (fails open)
`tools/camera_policy_runtime.py:105-106`: `no_return = ~covered | ...`, documented as "Coverage wins over the
explicit mask". ROS source of coverage: `tools/camera_ros2_adapter.py:206-258` and `:389`.

On the OAK device path, coverage False implies depth 0 by construction, because both come from the same remap. On the
ROS path, coverage is inferred from CameraInfo K, D and P. It is correct only if the delivered image is exactly that
remap's output. Mismatches that break it:
- a driver that publishes an already undistorted image with a CameraInfo that still carries D;
- a different alpha or P;
- depth aligned before rather than after undistortion;
- a raw image declared rectified in the binding.

In each case every valid return inside the computed border becomes no-return.

**Failing input** (`probe_scan.py`, section B): a valid pole at 0.96 m in trained columns 11-25, with coverage False on
the left 26 columns. The column minimum goes from 0.956 m to **20.0 m** (2340 no-return pixels), with the floor
filter on or off.

**Fix.** Only an uncovered pixel without a valid return is no-return. A valid return under coverage False contradicts
the producer contract. Refuse the frame with an InputError naming a CameraInfo/producer mismatch, or at least keep the
pixel valid and count it in diagnostics. Add the probe-B scene as a test. Its positive control is the existing
k1 = 0.45 border with depth 0, which must stay no-return.

### F4 — P1, pre-existing in fc8aa4d — The z-buffer rule "valid if any valid point lands" lets sparse speckle hide a near surface in every tilted frame
`tools/camera_policy_runtime.py:196-204`; the rule is now explicit in the docstring at `:127-135`.

About 53 OAK source pixels land on each trained pixel, and one valid sample makes it valid. The inverse-nearest path
classifies each trained pixel by one sample. It is taken only when the attitude is level within 1e-8
(`:168`), so in flight every frame uses the z-buffer path.

**Failing input** (`probe_scan.py`, section C): a slab 0.3 m ahead, invalid, with a share of spurious valid speckle at
6 m (stereo matchers produce such matches below MinZ).

| pitch | speckle | near px | slab column min |
|---|---|---|---|
| 0° | 5 % | 13700 | 0.400 m on 154/154 columns |
| 0° | 30 % | 9973 | 0.400 m |
| 0.5° | 5 % | **0** | **6.0 m** |
| 3° | 5-30 % | **0** | **6.0 m** |

Legacy behaves the same at 0.5°. At 3°, legacy showed 0.44 m only through the phantom tilt band that r1 removed.

**Real-world relevance.** The OAK-D's real near-range output is NOT_TESTED, but R1/R3 in flight rest on this rule.

**Fix.** Classify each trained pixel by the share of sensor-invalid rays among all samples that land on it, for example
sensor-invalid if `n_invalid ≥ n_valid`, otherwise valid with the nearest valid z, and no-return when nothing lands.
In expectation this is the inverse-nearest semantics, and iid 2 % holes stay non-dense.

Tests:
- 5-30 % speckle at 0.5° and 3° reads near;
- 2 % holes do not;
- the tilt band stays no-return.

### F5 — P2 — Near pixels on the goal now become a 0.4 m obstacle at the goal's bearing on the level path; FIX_r1 decision 6 misdescribes this
`tools/camera_policy_runtime.py:481` (`blank = invalid | no_return | near`).

Decision 6 says the pixels "stay a near obstacle blob in the scan". On the inverse-nearest path at fc8aa4d they were
red, therefore part of the target mask, therefore carved out of the obstacle map. They did not stay.

**Evidence** (`probes/compare_rt.txt`, real seed-1 runtime, base against new):

| scene | base tokens / tracker x | new tokens / tracker x | governed cmd base → new |
|---|---|---|---|
| half-invalid red sphere at 5 m, trained-equal, level | 9.908 m @ 0° / **3.849** (biased) | **0.4 m @ 0°** / 5.048 (correct) | (1.560, −1.251) → (1.614, −1.181) |
| same, OAK-like | 9.908 @ 0 / 3.481 | 0.4 @ 0 / 5.054 | — |
| half-invalid sphere at 1.5 m | no near token | 0.4 m @ 0° | (1.668, −1.103) → (1.879, −0.684) |
| half-invalid sphere at 1.5 m, OAK-like, pitch 0.5° (z-buffer) | 0.400 m centre | **identical to base** | — |

In flight (z-buffer path) nothing changes against base. The training semantics, however, was "target pixels are
carved out, even when near". The governor's `target_return_mask` exemption no longer covers these pixels.

**Fix options.**
- For near pixels that the production segmenter scores as target on the raw RGB, set depth to MaxZ and RGB to 0.
  They are then neither a range input nor an obstacle: the training carve-out, without the range bias.
- Or keep the current behaviour, correct decision 6, and add a test that pins the scan at the goal's bearing.

### F6 — P2 — The floor filter is ON by default, but its height reference is never checked
`tools/camera_policy_contract.py:24` (`GROUND_DEFAULT`); `height_reference: "floor"` is only a declaration.

Every hit height is the EKF ENU z + 0.03. Ways the EKF origin can sit off the floor: a VIO started at camera
height, a mocap origin, or a take-off from a platform.
- **Origin above the floor:** obstacle bases up to offset + 0.1 m are erased, and the invalid-floor rows move.
- **Origin below the floor:** the floor survives as a phantom wall.

The height observation (z/3) shares this assumption, which was flagged in the verification report.

**Fix.** Bind a measured floor offset or a pre-flight check in the configuration, for example landed z within
±0.05 m of 0 and, if available, PX4 `dist_bottom` agreement.

### F7 — P2 — The ROS 2 package README and package.xml are stale; one gap blocks deployment
- **`ros2/motar_camera_policy/README.md`.** It does not mention:
  - the required `geofence` block or `~/reset_geofence`;
  - `WAITING_ALTITUDE`;
  - the `ground`/`ceiling` contract fields and CLI;
  - CameraInfo D coverage and the OpenCV requirement;
  - REP 117 depth.

  Its list of conditions that "stop publication and latch the session" now has an undocumented exception:
  - altitude gating stops proposals without a policy fault;
  - the gateway then revokes on a stale command.
- **`package.xml`.** It lacks `<exec_depend>python3-opencv</exec_depend>`. `camera_ros2_adapter.py:243` imports cv2
  for every nonzero D, which image_proc and depthai CameraInfo normally carry. Without OpenCV every frame raises and
  the session faults. That is fail-closed, but it blocks deployment.
- **`docs/evidence/sitl_2026-11-05/camera_runtime/README.md:109-110`** still describes the pre-fix runtime. Do not
  edit it; add a dated note in this unit's evidence folder.

### F8 — P2 — Offboard publication is allowed with the geofence disabled
`tools/camera_ros2_px4_node.py:99-109`; FIX_r2 open item 4.

The policy never trained with walls. Recommendation: refuse `enable_offboard_publication: true` with a disabled fence
unless a hash-bound waiver is configured, in the same style as `flight_checks_provenance`. This is a user decision.

### F9 — P2 — Test gaps
- None of the failing inputs of F1-F5 is a test.
- The geofence stop check is 1-D and uses the formula's own model.
- The "bit-identical to legacy" claim is tested against a re-implementation (`legacy_project_rgbd`). I confirmed it
  against the real fc8aa4d code. Pinning golden observation hashes from that run would make it permanent.

### F10 — P2 — The SITL camera path changes quietly
`tools/sitl_camera.py:231` calls `runtime.step`, so Gazebo frames now get the default floor filter. Meanwhile
`gazebo_frame` (`camera_policy_runtime.py:325-334`) still flags +inf (sky) and z ≤ 0 as sensor-invalid. SITL camera
results from before and after this commit are therefore not comparable; record this. The MAVLink path has no pose
interpolation (disclosed).

### P3 notes
- **F11.** `camera_ros2_adapter.py` and `camera_ros2_session.py` now define observation semantics (REP 117, coverage,
  pose interpolation), but they are not in the contract's `SOURCE_PATHS` (`camera_policy_contract.py:37-51`). Neither
  are the PX4 node files. A contract cannot detect adapter drift.
- **F12. Geofence configuration.**
  - The box is axis-aligned in `px4_local_enu`, while rooms rarely are. Document that the operator inscribes it.
  - A disabled block silently ignores box values. The ceiling block refuses a height when disabled; do the same here.
  - There is no sanity upper bound on `decel_mps2` or `latency_s`.
- **F13.** The session watchdogs apply from the first gated tick. On the ground, any stream gap over 0.15 s latches a
  fault before take-off and needs a reset. This is conservative; document the operator procedure.
- **F14.** The inverse-nearest path is only for an attitude equal to identity within 1e-8. The tests at exactly 0°
  exercise a path that never flies. Bit-identity also holds at 0.5° pitch (z-buffer), as verified.
- **F15.** A target within the floor or ceiling margin, such as a landed or very low target drone, is removed from the
  detector: depth MaxZ, RGB 0.
- **F16.** The ceiling `height_m` is "measured" by the operator's word. No provenance is bound to it.
- **F17.** A breach leaves PX4 in OFFBOARD with a zero-velocity hold until reset or pilot takeover. The design (G3)
  accepts this; the README should say so.

## Train/deploy consistency

**Bit identity.** I ran the actual fc8aa4d runtime (`base_fc8aa4d/`) and the new runtime with the real seed-1
checkpoint on identical frames (`probes/compare_rt.txt`). The 898-D observation is **bit-identical** (IEEE bytes) with
the ground filter both OFF and ON in all of these scenes:
- level wall, trained-equal and OAK-like sources;
- level pole + target;
- level 2 % holes, with and without target;
- full valid sphere, at 5 m and 1.5 m;
- pitch 0.5° wall and pole + target (z-buffer path, full coverage).

The governed commands are identical too. The claim in FIX_r1 holds, and it holds against the real base, not only
against the re-implementation.

**Where the deployed observation now differs from training:**

| # | Difference | Intended? | Judgement |
|---|---|---|---|
| 1 | Tilt-uncovered pixels, +inf, z > MaxZ and undistortion borders: no-return (MaxZ) instead of dense near | Yes, restores training (H1/H5a) | Correct |
| 2 | Floor filter on valid pixels: hit_z < 0.10 m → MaxZ (default ON). The bottom 10 cm of every obstacle is removed | Yes (training had no floor) | Correct. Column minima are unaffected for vertical bars; depends on F6 |
| 3 | Invalid pixels below the horizon band → no-return (floor explanation); above it too with the ceiling on | Partly | **F1:** also erases real near obstacles |
| 4 | Ceiling filter (off unless a measured height is given) | Yes | Correct |
| 5 | RGB 0 on sensor-invalid, no-return and near pixels; near pixels on the target leave the target mask and become obstacles | Range: yes (R4). Obstacle effect: not stated correctly | **F5** |
| 6 | Pose interpolated to rgb_t (ROS path) | Yes, restores synchronous state | Correct |
| 7 | History reset after altitude gating | Yes | Correct (training had no such gap) |
| 8 | Geofence clamp: executed velocity ≠ `previous_action` | Inherent to a post-policy limiter (disclosed) | Accept; unmeasured |
| 9 | z-buffer sensor-invalid rays are rotation only (translation ignored) | Disclosed | Accept; mm-to-cm effect |
| 10 | z-buffer "any valid wins" (pre-existing) | Not intended as training semantics | **F4** |

## Decisions outside the spec

| Decision | Judgement |
|---|---|
| r1-1: invalid-pixel plane rule bounded to [MinZ, MaxZ] | **Reject as is (F1).** MinZ never binds above about 0.2 m camera height, so all invalid rows below the band are removed regardless of content; the "conservative" description is wrong for low near obstacles |
| r1-2: MinZ/MaxZ evaluated on level optical z | Accept |
| r1-3: `ground` (default ON, 0.10) and `ceiling` (default OFF, null) contract fields, structural validation | Accept. The JSON-boolean and null-height rules are good. F6 for the unverified height reference |
| r1-4: runtime refuses outside the band | Accept. The ROS path now waits (J); the direct runtime and replay still refuse |
| r1-5: interpolation does not relax acceptance; `MAX_POSE_BRACKET_S = 0.15` | Accept. Verified: refusals and the ±π slerp |
| r1-6: blanking moves near target pixels into the obstacle map | Accept the range part. **F5:** the scan consequence is misdescribed and untested |
| r1-7: `depth_near_pixels` alias; `invalid_pixels` dropped from the projection report | Accept. No in-repo reader: `camk2v3e_run.py` reads depth-validity telemetry, not this report |
| G1: each component limited against the wall it points at | Accept. This is the natural reading of R6, but see **F2** (simultaneous per-axis deceleration) |
| G2: a breach also zeroes the yaw rate | Accept (conservative) |
| G3: new latched `geofence_fault`, streams stay up; other faults still revoke | Accept. A stale policy during a breach still revokes (verified by reading `update()`); F17 |
| G4: reset needs a fresh decode and an inside position; the boundary counts as inside; refused when not latched or not primed/active | Accept |
| G5: a breach at prime latches | Accept |
| G6: enabled values must be numbers, latency > 0; `geofence=` is a required kwarg; a disabled block ignores values | Accept, except that silently ignoring values is weak (F12) |
| G7: frame literal `px4_local_enu` = decoded ENU position | Accept. Verified: `decode_estimator` gives (E, N, U), and `encode_setpoint` swaps the clamped ENU velocity back to NED |
| H1: CameraInfo D: missing D refused; zero D means all covered; plumb_bob/rational_polynomial only; lazy cv2 | Accept the D handling. **F3** (coverage overrides valid depth), **F7** (OpenCV dependency) |
| I1: finite 32FC1 z ≤ 0 stays sensor-invalid | Accept |
| J1: a gated tick counts for the output watchdog; watchdogs apply from the first tick | Accept; F13 |
| J2: shared `height_band` predicate; contract invalidated again | Accept. A base-built contract is refused with 7 problems (`probes/old_contract_validation.txt`) |

## Test quality
- **Focused suites, run by me.**
  - `-m unittest discover -s tests -p 'test_camera_*.py'`: `Ran 130 tests in 18.775s`, `OK`, EXIT 0
    (`focused_camera.log`; verbose `focused_camera_verbose.log`). This includes both real-checkpoint classes of the
    new files, `ActualPolicyCPU` and the ROS integration.
  - `-p 'test_oakd*.py'`: `Ran 11 tests`, `OK` (`focused_oakd.log`).
- **Full suite: not rerun.** The builder's `r2/public_cpu_r2.log` ends `Ran 4114 tests`, `OK (skipped=10)`, EXIT 0.
  Its launch-time SHA-256 manifest still matches the working tree (`sha256sum -c` exit 0). Kernel counts in its
  records are 0.
- **Production code is exercised.** Fixed paths call the production `project_rgbd`, `remove_floor_and_ceiling`,
  `preprocess`, `apply_mask`, `build_frame`, `RosPolicySession`, `Px4OffboardGateway` and the real seed-1 runtime.
  Negative controls use `legacy_project_rgbd`, a faithful re-implementation; I confirmed it against the real
  fc8aa4d.
- **Controls.** Positive and negative controls with non-zero counts are present throughout. Gaps: F9.
- **Existing assertions were not weakened.**
  - The 104 union is split exactly into 103 + 1, and disjointness is added.
  - The tilt test's old "uncovered band is invalid" was the H1 bug; it is now "no-return > 0 and sensor-invalid
    == 0".
  - The REP 117 +inf row changed meaning by design, with the reason in the test, and new -inf/0 rows were added.
  - The other changes only add the `geofence=` kwarg, `d=[]`, and a fake `height_band`.

## Process and records
- **Contract invalidation.** Correct and fail-closed: an old contract fails on schema, ground, ceiling and source
  hashes. The rebuild instruction (`--output NEW.json --seed 1 [--ceiling-height-m H]`, then update
  `policy_contract_sha256`) is right. The adapter and session are unbound (F11).
- **README and package.xml.** Stale (F7). This is misleading by omission, especially about OpenCV, the required
  geofence block and WAITING_ALTITUDE.
- **Builder reports.** Mostly accurate and well cross-referenced. Inaccurate or overstated:
  - decision 6 (F5);
  - r1-1 "conservative direction" (F1);
  - the docstring "it stops within d" (F2).
- **Before commit:** a WORKLOG entry, an EXPERIMENTS.md family entry if an evidence folder is added, and this review
  committed as `REVIEW_r1.md` in the unit's evidence folder.

## Verified by running vs by reading
- **By running (production code):**
  - the focused tests;
  - fc8aa4d against the new runtime with the real seed-1 checkpoint: bit-identity, F1 and F5;
  - production `preprocess`: F1, F3 and F4;
  - production `Geofence.limit` with 2-D rollouts: F2;
  - `interpolate_state` at the ±π yaw wrap and its refusals;
  - the base-contract refusal.
- **By reading only:**
  - the absence of any unfenced publish path. The payload of `_accept_command` is discarded; only `prime` →
    `_neutral` and `update` → `_neutral`/`_geofenced` are published. Every position path decodes with freshness
    first;
  - the session gating and watchdog flow beyond the builders' tests;
  - the ENU/NED handling in the clamp;
  - the CameraInfo K/D/R/P use (`R` must be identity, `P[:, :3]` as the new camera matrix, nearest interpolation,
    zero border);
  - the README and package.xml staleness;
  - the SITL path effect (F10);
  - F6, F11-F17.

Reproduce: `probes/probe_scan.py <worktree>`, `probes/gen_frames.py`, `probes/run_rt.py <npz> base|off|on <out>`
(base with `PYTHONPATH=base_fc8aa4d/tools`), `probes/gen_box_frames.py`, `probes/probe_geofence.py` and
`probes/probe_slerp.py`, each with `PYTHONPATH` set to the worktree's `tools` (and `tests`).

Git Traceability: Branch `camonly/dev`; Base SHA `fc8aa4d`; New Commit SHA none (read-only review); Changed Files none
in the repository; Tests: focused camera 130 OK and oakd 11 OK, run by the reviewer; Working Tree: 18 uncommitted
paths before and after, all from the builders; Remote Push NO; Main Merge NO; Raw/Checkpoint Integrity: checkpoint
only read (contract pin `888e8056…` verified by `build_contract` at each runtime construction), `results/` untouched.
