# BOIA G4 online-path source audit — 2026-10-20

Base: `13d5f195399bf1a7f1cdf9ddeccd3338d8e34c0f`, branch `codex/mt-p1`.
Scope: all functions in `access`, `belief`, `visibility`, `candidates`, `cost`,
`arbitration`, `fallback`, `terminal`, `planner`, `trace`, and the package initializer.
Geometry, safety and dynamics receive separate audits. Existing source, experiments,
raw, checkpoints, main WIP and stash were not changed by this audit.

**Result: no additional P0 from this delegated online-path audit.** Geometry-owner
issue **B02**, a P0 camera-LOS origin mismatch in the base `visibility.py`, is counted
and fixed in the coordinated geometry audit, not duplicated here. Its new
`_camera_origin_xy` helper and both forecast/BPOD call sites were independently read:
they use the world optical centre `p_xy + R_yaw camera_offset_xy` consistently.
No additional critical violation of the approved CPU contract was demonstrated;
this does not qualify
the sensor/model/controller for online use. There are **10 P1 qualification or
interpretation risks and 2 P2 trace/comparator limitations**, with distinct IDs below.
The existing target-first-hit/known-free issue `motar-3p3.13` remains blocked and is
not counted again here. Several risks are intentionally prescribed design limits,
not defects that may be silently corrected in G4.

Risk-table file:line citations refer to the base13d5 source. The JSON records both
base and reviewed working-file SHA-256 and line ranges. The coordinated B02 patch
adds one helper and shifts later visibility lines; no other owned source was edited.

## Independent CPU evidence

Command, using the pinned environment and no visible GPU:

```bash
env PYTHONNOUSERSITE=1 CUDA_VISIBLE_DEVICES='' OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 \
  /home/fair/miniconda3/envs/aerialgym/bin/python -B -m unittest discover \
  -s tests -p test_boia_p0_online.py -v
```

**9 tests, 0 failures, 0 errors, 0.329 s** in the final focused run after the coordinated
B02 camera-origin patch. Passing
characterizations deliberately establish the existence of limitations; they must
not be read as passing the corresponding qualification requirement. The independent
oracles include scalar Joseph-update arithmetic, SHA-256 via `hashlib`, and an
explicit pinhole ray/sphere discriminant at pixel centres. No simulator module,
raw array, detector implementation or external algorithm was executed. The existing
PUBLIC suite was not repeated by this sub-audit.

## Risk register

| ID / severity | File:line and claim | Counterexample or precise scope limit | Independent check / fix status |
| --- | --- | --- | --- |
| ON-01 / P1 | `planner.py:94–97`; `access.py:37–38`: only live rows are evaluated/logged, but the full batch is copied and validated first. | With terminal mask `[True,False]`, NaN in env0 position or belief aborts env1 with `ValueError`. All-terminal bypass is correct. | `test_o01_mixed_terminal_nan_blocks_unrelated_live_row`. Reproduced, **NOT_FIXED**. This is fail-closed batch availability and reset-isolation failure, not unsafe returned control. Later accessor masking must preserve whitelist and live-row validation. |
| ON-02 / P1 | `planner.py:97,109`; `belief.py:151–157`: one local generator draws a tensor whose leading size is `N_live`. | Masking env0 changes env1's Monte Carlo path despite identical env1 input digest and generator seed. Identical local seed is insufficient for paired exogenous randomness under asynchronous terminations. | `test_o02_other_terminal_mask_reassigns_live_env_samples`. Reproduced, **NOT_FIXED**. All candidates within one call still share paths correctly; no global RNG is used. Per-env streams/counters and matched-reset tests require an approved integration design. |
| ON-03 / P1 | `access.py:102–118`; `visibility.py:56–59`: camera intrinsics are copied, while primary detection uses `cam.fx`, `params.r_T`, `params.min_pixels`. | Adding a 320×180 DETECT path leaves BOIA's copied/digested calibration unchanged. An independent 18 m on-axis sphere has 0 pixels at160×90 and4 at320×180, crossing the real2-pixel gate; primary area using the old `fx` remains below2. Separately, copied `cam.min_pixels=100` still yields primary probability1 at6m with default parameter threshold2. | `test_o03_detect_resolution_is_not_part_of_current_accessor`, `test_o03_primary_gate_ignores_copied_camera_pixel_threshold`. Production source independently distinguishes camera and DETECT intrinsics at `navrl_perception.py:765–783`, thresholds at1608. Defaults have equal resolutions; no claim that frozen P03 used a decoupled setting. **NOT_FIXED**; changing the explicitly approved whitelist/priors requires review. |
| ON-04 / P1 | `access.py:75–79`; `arbitration.py:58–73`; `planner.py:109,115`: active/age/camera flag contain no confirmed identity or measurement timestamp. | Active fused track with age0 and camera miss streak20 is LOST_PREDICTABLE; it does not establish identity or a fresh camera measurement. An inactive/stale mean is still sampled/scored when choosing a safety alternative, although it cannot trigger gain intervention. | `test_o04_active_and_fused_age_are_not_identity_or_timestamp` plus source inspection. Tracker LiDAR corrections can reset fused age (`navrl_perception.py:2077–2080`); separate identity bookkeeping exists at2274–2279. **NOT_FIXED / NOT_CALIBRATED**. No forbidden diagnostic/GT was added. Future alignment/identity contract must be explicit. |
| ON-05 / P1 | `candidates.py:28,83–87`: the family named “yaw-only” includes translation. | Candidate19 requests0.6m/s along current velocity heading, or yaw heading when stopped. It is not a translation-free yaw baseline and cannot isolate pure yaw contribution. | `test_o05_candidate19_translates_and_intercept_never_decline_gain`. Independent norm is0.6m/s. **SPEC_CONFORMING**: user B5 and07§8 explicitly require this command. No automatic change. |
| ON-06 / P1 | `arbitration.py:67,73,115–127`: INTERCEPT and NEVER_ACQUIRED disable gain intervention. | Even when another feasible candidate has cost−10 vs F cost10, these modes return feasible F. d_int=3m hands the final approach back to F; never-acquired starts are not BOIA search trials. | Same `test_o05...` direct counterfactual costs. **SPEC_CONFORMING** under user B6/07§2. Do not claim close-range recovery improvement or initial search capability without separate evidence/design. |
| ON-07 / P1 | `belief.py:71–79,97–99,127–164,195–205`; `planner.py:90–114`: Q is mirrored CV noise and R is an explicit fixed world-diagonal prior. | Scalar Joseph arithmetic is correct, but neither correct algebra nor Gaussian samples calibrate E2 manoeuvres, detector error, cross-correlation, identity switches or latency. R is not the live range/pixel-dependent measurement covariance. | `test_expected_update_matches_independent_scalar_and_does_not_calibrate`: position variance0.68 for P=1,p=.4,R=.25; unobserved velocity variance stays1; p=0 is exact; singular entropy fails closed. **NEES/NIS, reliability, coverage and s_Q/R calibration NOT_TESTED**. G5 only. |
| ON-08 / P1 | `visibility.py:38–63`; `planner.py:99–106`: one fixed horizontal sensed point set/free context is reused throughout H. | No obstacle velocity or occupancy timestamp is supplied. A moving obstacle entering a formerly free ray is outside the model; geometric camera silhouette/partial support is also not represented by the centre-line forecast. | Source/interface absence, not an observed simulation incident. Independent moving-obstacle/partial-silhouette validation **NOT_TESTED**; no test is mislabeled PASS. Existing static vertical-bar scope is explicit. |
| ON-09 / P1 | `terminal.py:7–17`; `planner.py:60–133`; `fallback.py:5–17`: outcomes are externally flagged and fallback requests zero translation. | Synthetic static-observation replay does not produce actual capture, collision, timeout, safe-abort or hand-back actor-history outcomes. Zero command cannot establish collision-free stopping when all candidate stopping tests already failed. | Source/interface inspection. **NOT_TESTED end-to-end**; no capture/abort declaration exists, which correctly avoids GT/outcome leakage. Mechanics/plant/sensor validation and independently recorded simulator outcomes remain required before a Recovery claim. |
| ON-10 / P1 | `cost.py:46`; controller interpretation07§6.1/user B4: all4 normalized actions enter smoothness although a_z is overwritten by altitude PI. | Identical modeled physical trajectories with commands differing only by a_z=.9 vs0 have smoothness difference .2025 and default weighted J difference .010125. This is utility variation without differing physical effort. | `test_o10_overwritten_vertical_action_changes_smoothness_cost`. **SPEC_CONFORMING / NOT_FIXED**, because07§9 explicitly defines the four-component norm. Review the utility/advantage interpretation before calibration; do not silently alter the objective. |
| ON-11 / P2 | `trace.py:76–115`: digest binds copied online inputs, not complete controller execution state. | Same input digest can accompany different RNG state, BOIA dwell/history, parameters or source version. The trace omits complete planner state and local generator state. First8 SHA bytes are a64-bit fingerprint, not a collision-proof archival identifier. | SHA independent vectors and trace copy mutation pass (`test_sha_standard_vectors_and_trace_copy_remain_isolated`). **REQUIRED OUT-OF-BAND PROVENANCE**, not a failed input-digest implementation. Generic helper schema growth also needs an explicit serialization-version review. |
| ON-12 / P2 | `visibility.py:59,67–70,83–122`: stressed detector and analytic comparator retain declared approximations. | Within-path product assumes conditionally independent detector misses. BPOD uses one active linearized FOV boundary and a product across obstacle events, omits unknown-space probability, and can be biased in either direction under correlation. | Source/formula inspection; prior focused unit evidence only, no new universal calibration claim. **LIMITED_VALIDITY**. These comparison utilities are not calibrated online detection probabilities. |

## Function and contract coverage

The companion `online_audit.json` enumerates **44 base functions plus the coordinated
B02 helper,45 reviewed working functions in11 modules**, including
private validators/helpers and the initializer's lack of runtime code. Every function
was read in its actual source; not every function has a new independent runtime test.
Inspection, new runtime evidence, existing test references, and unavailable empirical
qualification are distinguished. Scope covers input/output shapes, metres vs m/s vs
radians, W/yaw-only V frames, explicit dt, per-env batch operations, finite/PSD/PD
checks, probability boundaries, cost/arbitration precedence and terminal/reset handling.

The approved online call tree uses only own state, the measured LiDAR ranges, copied
tracker belief and the camera flag/calibration. No actual forbidden task/GT lookup or
simulator import was found. Frozen dataclasses contain tensors, so caller-owned tensors
remain mutable objects; the audited steps clone/update without writing the source task,
tracker, incoming BOIA state or trace-linked action. The passed local generator is the
documented exception: it advances.

`arbitration` uses camera-only miss counts, INTERCEPT precedence, five completed engaged
decisions before gain-only release, immediate feasible-F handback in safe modes, safety
override irrespective of dwell, and finite alternatives/+inf costs without NaN advantage.
The fallback actually returned is stored in `a_prev`. Terminal flags clear only BOIA
state; live rows are reclassified, so TERMINAL does not latch across episodes. These
mechanics are source-consistent; their research usefulness is not established by this audit.

## Disposition

No production or BOIA source patch was made. Preserve the current negative/limited
evidence and request model/interface review before G5/online evaluation. New tests
characterize the base behavior; they do not relax thresholds or substitute synthetic
scenes for recorded calibration. Cross-audit counts must deduplicate shared geometry,
safety, dynamics and existing issue13 risks rather than sum filenames.

## Git Traceability

- Branch: `codex/mt-p1`
- Base SHA: `13d5f195399bf1a7f1cdf9ddeccd3338d8e34c0f`
- New Commit SHA: parent integration pending; this delegated audit makes no commit
- Changed Files: this report, `online_audit.json`, `tests/test_boia_p0_online.py`
- Tests:9 focused characterizations PASS,0 failures/errors;0.329s
- Working Tree: new audit artifacts only for this scope; other agents' work is separate
- Remote Push: NO
- Main Merge: NO
- Raw/Checkpoint Integrity: no raw arrays or checkpoint bytes opened/changed; no stash/WIP operations
