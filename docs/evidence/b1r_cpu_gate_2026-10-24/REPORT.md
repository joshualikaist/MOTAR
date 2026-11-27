# B1-R CPU entry gate — BLOCKED

Date: 2026-10-24 (client date). Source: `2acc2d5f4365e83e036f45e8fa6570029e859eb5`.
This commit changes only preregistration and WORKLOG relative to `fde21cd`.

## Result

PUBLIC_CPU: 3,177 tests, 0 failures, **4 errors**, 10 skipped, 5 profile exclusions;
252.701 seconds; exit 1. Full receipt and machine-readable summary are adjacent.
No B1-R source has been installed, so these errors predate the proposed implementation.
A fresh-process isolated detect-resolution suite passes all 34 tests (exit 0).

All four errors are `test_navrl_detect_resolution.DetectReduction` methods:
`test_bounding_box_and_empty_convention`, `test_count_centroid_and_range`,
`test_frame_carries_the_rendered_target_colour_and_far_plane`,
`test_the_frame_carries_no_image_sized_tensor`.

The stack reaches `navrl_detector.py:698` real `wp.launch`; Warp cannot convert
`target_half_extents` containing None to vec3f. The intended stub in
`tests/test_navrl_detect_resolution.py:70` uses `sys.modules.setdefault`, which
does not replace an already imported real Warp module. This is consistent with
existing issue `motar-6nn.5` (P03 isolation/detection test process interference).
The present check confirms full-suite versus isolated divergence; it does not
newly prove which earlier import is the sole cause. No production defect is
established by this test-harness error.

## Decision and resume point

The explicit zero-failure/zero-error CPU requirement is unmet. STOP before B1-R
implementation and GPU Stage A. G0/G1/G2/G3 and C1/C2 are NOT_TESTED, not failed
experimental outcomes. G5 HOLD. No retry, GPU, PPO, push or merge performed.

User has approved all four GPU stages conditional on gates; that approval persists.
The Freeze completion obligation is removed; stage caps remain. No new GPU approval
is needed after the gates pass. The current handoff restricts implementation to six
new files: repairing existing tests is outside that scope. Proposed next approval:
allow the minimal test-only isolation repair for `motar-6nn.5`, preserving all test
assertions and production bytes, then rerun PUBLIC_CPU and resume the authorized B1-R
sequence. Do not mask this error with test exclusions or ordering changes.

Remaining work: six-file implementation (including late sidecar-path resolution and
runner supervision), focused and full CPU checks, source commit, gated A/B/C-PN/C-PP,
selection evidence and final analysis. No process remains owned/running at handoff.

## Preservation

Main HEAD/status and the three WIP file hashes match the pre-work receipt; protected
untracked prereg draft hash and stash identities match. Policy F SHA matches
`f702213936601860995cf61dcc570247e72543b1976e3716055cd8ec5593ad40`.
No existing source, raw or checkpoint was intentionally edited; historical raw was
not exhaustively rehashed. No other worktree was edited. Large raw is not staged.
Logs include synthetic runner fixture output; those are CPU tests, not GPU runs.
