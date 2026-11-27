# Review r1: CAM-S2 projection latency port (b3b025a + uncommitted diff), camonly/s2b

Reviewer: independent, read-only. Worktree `/home/fair/workspaces/aerial_gym_ws/src/MOTAR-camonly-s2b`, HEAD b3b025a
plus `git diff HEAD` (tools/camera_policy_runtime.py, tools/run_tests.py, tests/test_camera_runtime_cams2.py).
Line numbers refer to the working tree. Reviewer scripts: session scratchpad chk1-chk4.py (not in the repo).

## Runs (this host: x86, numpy 1.23 / OpenBLAS64, torch 2.4.1+cu121, RTX 3070)
- `python -B -m unittest test_camera_runtime_cams2` (CPU, CUDA hidden): 25 tests OK; the one error is the expected
  `CudaV2Equivalence.setUpClass` refusal without CUDA (the PUBLIC_CPU profile excludes that class, see Q4).
- `... test_camera_runtime_cams2.CudaV2Equivalence` (GPU): 2/2 OK.
- `run_tests.py --list` (PUBLIC_CPU): 25 cams2 ids, 0 CudaV2 ids; `--profile GPU_REQUIRED --list` lists both CudaV2 tests.
- Golden: `check_golden.py --self-test --source-root <worktree>` against the new default (v2): reference PASS on all 3
  cases (0 class mismatch, 0 depth/RGB error); attitude-blind mutant FAIL. SELF_TEST PASS.

## Q1. `_first_min_per_pixel` (cpr.py:652-666) and v1_fast (669-770)
- Equivalent to "first entry per index after `np.lexsort((keys, indices))`". lexsort is stable, so the first entry per
  index is the min key, and among equal keys the smallest position. The function takes per-index min (`minimum.at`),
  then the smallest position whose key == that min. `==` treats -0.0 == 0.0, the same way the sort compares them. Empty
  input returns two empty int64 arrays (658-659). Precondition: keys contain no NaN. It holds: zs are finite in
  [MinZ, MaxZ] and the ray offsets are finite squares. Checked: 300 random cases (float keys, signed zeros, many ties,
  n=0..3000), 0 mismatches.
- v1_fast == v1: the arithmetic is the same expression on the same operands. `u.reshape(-1)[good]` uses the same
  row-major order as `u[good_mask]`. `rgb_flat[good[selected[level_valid[pos]]]]` is `rgb[good][selected][level_valid][first]`.
  The invalid-ray colour (754-757) first keeps only rays whose pixel is out_invalid, then picks. Rays are kept or
  dropped per whole pixel, so each pixel's pick is unchanged (v1 picks first, then masks with `take`). Refusal text is
  identical. Checked: 40 extra frames, including sigma 0.35 rad tilts, coverage, ties and trained-resolution refusals.
  Every output, every report and every refusal were identical. Q1: YES.

## Q2. v2 CPU (cpr.py:781-940) vs v1
- Semantics match line by line:
  - Validation and source classes are in `_projection_inputs` (781-818).
  - Points: `(R0*z + R1*(-u z)) + R2*(-v z) + off` (_v2_rows 831-837).
  - Splat: the same `floor(x+.5)` and keep bounds. The offset² is `dx*dx+dy*dy`, the same as numpy `**2`.
  - Level-camera MinZ/MaxZ filter, first-min selection, hit/out_z/colours.
  - Invalid rays, `imaged` (the same `ahead` threshold and coverage gather), direction coverage (all pixels when
    there is no coverage mask, flatnonzero(cov) otherwise), `enclosed`, hole refusal (same message, `_holes_error`).
  - out_invalid / no_return, cosine and ranges, report dict (`_v2_finish` 845-859; level is computed on the host
    rotation, as in v1).
- Where v2 may differ from v1: only where v1's BLAS (N,3)@(3,3) product rounds differently from the fixed elementwise
  order. That can move a sample across a `floor(x+.5)` border, the MinZ/MaxZ edge, the `ahead`/`imaged` thresholds or a
  min-z tie. In principle a hole refusal can also flip, and the test then fails on `a[0]==b[0]`, which would be a
  spurious failure.
- Measured on this host: v1 points are bit-identical to the v2 rows (0 of 34 frames differ), so v2 == v1 exactly here.
- How sensitive are the outputs to rounding? Summing the rows in another order changes 13 % of the point coordinates
  by about 1 ulp (719,119 of 5.46 M), yet gives 0 differing output pixels over 34 frames. Border flips are rare.
- The <= 4 px bound (tests:921) is therefore not exercised by rounding. It is still a non-vacuous semantics guard:
  - Mutant "offset + 1 mm": 43,409-69,641 px per frame, all 21 accepted frames > 4.
  - Mutant "last-tie": up to 13,124 px, 3/21 frames > 4.
  - Neither mutant is in the suite (see P3-6).
  - The bound is justified as a loose cross-host allowance.

## Q3. CUDA bit-identity (CudaProjector, cpr.py:943-1067)
- Shared inputs:
  - `R` and `offset` are host floats (807-809). The u/v grids are copies of the cached CPU `_pixel_rays`, and tu/tv of
    `_trained_grid` (956-959, 997-998). The cosine is used only in the CPU `_v2_finish`.
  - Done on the GPU: neg, mul, tensor/tensor div, add/sub, floor, compares, where, nonzero, boolean-mask gather,
    scatter_reduce(amin), cummax.
- Rounding hazards checked:
  - No `tensor / python-scalar` on the GPU. Torch's CUDA true-div would multiply by the reciprocal (div_true_kernel_cuda
    `is_cpu_scalar(2)` path). Every division is tensor/tensor: `-a1/a0` (966-967) and `-s1/den` (1026-1027).
  - Scalar multiplies are exact IEEE muls in float64.
  - CUDA add/sub is `a + alpha*b`. nvcc may contract it to fma(alpha, b, a), but with alpha = ±1 that equals
    round(a ± b): no change.
  - Eager mode launches one kernel per op, so nothing fuses across ops. No addcmul, matmul or pow.
  - All tensors are float64. Every Python constant meets a float64 tensor (zz is cast with `.to(float64)` at 1003),
    so nothing is promoted to float32. CUDA float64 has no FTZ and correctly rounded div.
  - scatter_reduce amin and the int64 `first` are order-independent. Under -0.0/0.0 the `best` value may come out
    either way, but it is only used with `==`, and outputs come from `zs[pos]`. index_put targets unique pixels (or
    writes True everywhere).
  - nonzero is ascending, like flatnonzero. cummax on a uint8 0/1 tensor == `np.maximum.accumulate`. Index dtypes
    are int64.
  - `torch.where` uses tensors on both sides (`one*-10.`).
- Checked:
  - 40 frames with sigma 0.4 rad tilts, float64 depth input, a 0.3 m mount offset and coverage: 33 accepted and
    7 refused, 0 mismatches.
  - Intermediates (rows, xf/yf/index/offset², xs) for 50 random rotations × 200k points: 0 bitwise differences in
    33.2 M values.
  - Same result under `torch.use_deterministic_algorithms(True)` and `set_flush_denormal(True)`.
- Q3 answer: yes, the CPU and CUDA paths are bit-identical by construction on one host, given eager torch and IEEE
  float64 CUDA. Limits:
  - The claim relies on torch internals (alpha = 1, no scalar division). Its regression test checks final outputs
    only, and final outputs barely react to ulp drift (see Q2). That is P2-1.
  - Bit-identity across hosts is not provided; see P3-1.

## Q4. Runtime integration
- `CameraPolicyRuntime(contract, torch_threads=None, projection_device=None)` (1707). None means no thread change:
  `pin_cpu_threads` with n=None only reads values (1681-1700). The projection is CPU v2 (1734). Instances built
  without `__init__` fall back to CPU v2 through `getattr` (2018).
- Default arithmetic changed from v1 to v2 for every caller, including the old semantic tests at tests:112-237 and the
  golden checker. Outputs are identical on this host (tests + golden). See P2-2 for traceability.
- Callers:
  - No test passes `torch_threads`.
  - The ROS2 node pins 1 thread only after the `check_only` return (camera_ros2_node.py:237-244). Tests call it with
    `check_only=True` or in subprocesses.
  - The replay CLI now defaults `--torch-threads 1` (2283-2289), but no test calls the runtime's `main`.
  - ThreadPinning uses a FakeTorch.
  - No torch thread state leaks into the shared process.
- GPU_REQUIRED (run_tests.py:32-33): the class id matches via `startswith(entry+'.')`. PUBLIC_CPU excludes both tests,
  GPU_REQUIRED and FULL_RESEARCH include them, and the stale-entry check passes.
- FULL_RESEARCH: the cams2 CUDA class creates a CUDA context in the runner process. The later GPU tests start workers
  with `subprocess.run`, not fork, so this is harmless.

## Q5. Tests
- Non-vacuous where it matters:
  - `test_random_scenes...`: asserts tilted_ok > 10, refused > 0 and ties > 5.
  - first-min unit test: covers n=0 and n=1.
  - Last-tie mutant negative control: shows colours change.
  - CanonicalV2: asserts ok > 10.
  - CudaV2: asserts ok > 10 and refused > 0, with refusal text compared.
  - Fixed-order rows/cols test: exact.
- Gaps:
  - No negative control for the <= 4 bound.
  - No intermediate-level CPU/GPU bitwise test (P2-1).

## Findings

### P1: none.

### P2
- **P2-1. The CUDA bit-identity guard cannot see ulp drift.**
  - Problem: `CudaV2Equivalence.test_bit_identical_to_cpu_v2` (tests:944-953) compares only final outputs. A 1-ulp
    change in 13 % of the coordinates gave 0 output-pixel differences over 34 frames. A future change such as
    `(x - c)/f` on the GPU, an addcmul, or `torch.compile` would most likely still pass, and would surface on the
    vehicle as rare CPU/GPU border flips.
  - Fix: add a bitwise CPU/GPU test of the intermediates (`_v2_rows` output, `_splat` xf/yf/index/offset², xs/ys) on
    random rotations, with a mutant control (e.g. GPU `/scalar`) that must fail it.
- **P2-2. Logs cannot tell which projection produced a frame.**
  - Problem: the runtime default silently switched to v2. The report dict still says
    `'projection': 'measured-point-min-valid'` (`_v2_finish` 853), the same as v1, with no device. The replay report
    adds `threads` (2292) but no projection device, and the replay CLI has no `--projection-device`.
  - Fix: add `'projection_arithmetic': 'v2'` and `'projection_device'` to the report (and to the replay output).

### P3
- **P3-1. The "between machines" claim is overstated.** The v2 header comment (773-779) and the CanonicalV2
  docstring imply the result is reproducible across machines. But these inputs are still computed with host BLAS or
  libm, then shared with both backends:
  - `rel = yaw_r.T @ body_r`, `rotation = rel @ optical` and `offset = rel @ mount` (805-809), via numpy BLAS;
  - `atan2`, `cos` and `sin` (`validate_state`, 798).

  So CPU == GPU holds on one host, while x86 golden vs Orin can still differ in rare border cases. `check_golden`
  demands 0 class mismatch. Fix: word the comment "same host", or make the 3x3 products elementwise too.
- **P3-2. New classes sit after `unittest.main()`.** They follow `if __name__ == '__main__': unittest.main()`
  (tests:729), so `python tests/test_camera_runtime_cams2.py` never defines them. `-m unittest` and run_tests are
  unaffected.
- **P3-3. The CUDA path raises where the CPU path does not.** Non-native-endian depth raises `ValueError` (from
  `torch.from_numpy`) where CPU v2 accepts it. Read-only input arrays (e.g. `np.frombuffer` from ROS) trigger a torch
  UserWarning. The ROS nodes catch `Exception`, so it fails closed. Fix: use `np.ascontiguousarray(..., dtype=native)`.
- **P3-4. Constructor order.** `projection_device` is validated (1735-1737) after `pin_cpu_threads` already changed
  global torch state. `warm_up_projection` (2033) raises AttributeError on instances built without `__init__`.
- **P3-5. Unused v1_fast.** `project_rgbd_min_valid_v1_fast` is now reachable only from tests. Keep it as a
  documented reference or drop it later.
- **P3-6. Mutant controls are only in the reviewer scripts.** Consider adding one semantic mutant (e.g.
  offset + 1 mm) to CanonicalV2 to pin that the <= 4 bound would fail on a real regression.

VERDICT: GO_WITH_FIXES (no P1; P2-1 and P2-2 are small and should land before the vehicle side relies on CPU/GPU interchangeability in logs)
