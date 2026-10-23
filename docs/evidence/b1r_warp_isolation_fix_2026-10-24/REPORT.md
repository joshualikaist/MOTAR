# motar-6nn.5 — scoped Warp isolation repair

Only runtime test change: `tests/test_navrl_detect_resolution.py` imports unittest.mock
and temporarily patches `_DET.wp` to the existing stub within `DetectReduction._run`.
No shared `warp.launch` or `sys.modules["warp"]` mutation is added. The context manager
restores the detector reference on normal return and exception. All 34 test function
ASTs are unchanged; no assertion, skip, xfail, ordering, environment or production edit.

Cause: module-level `setdefault("warp", stub)` cannot override an earlier real Warp
import. The reduction-only fixture has no real ray buffers, so real launch fails
packing `target_half_extents=None`. Before repair, fresh real-Warp-first execution
reproduced 4 errors. Afterwards: standalone 34 PASS; real-Warp-first 34 PASS, with
module/launch/detector object restoration assertions PASS; PUBLIC_CPU 3,177 tests,
0 failures, 0 errors, 10 skipped, 5 existing profile exclusions.

Commands: pinned aerialgym Python, PYTHONNOUSERSITE=1, -B. PUBLIC_CPU uses
`tools/run_tests.py` unchanged; isolated probes set CUDA_VISIBLE_DEVICES empty.
The real-Warp probe imports warp first, then loads this test module and runs all
34 tests through unittest, asserting the original objects remain after execution.
Full logs, SHA bindings and exact diff are adjacent. Previous failed gate evidence
in `../b1r_cpu_gate_2026-10-24/` remains unchanged.

CPU gate reopened; resume previously approved B1-R implementation, re-run focused
and full CPU validation before GPU source freeze. G5 HOLD remains; this repair
does not qualify simulator behaviour or expand GPU authorization. No GPU run here.
