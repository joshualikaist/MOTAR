# Independent Fable5.1 focused read-only review

Reviewed result: be4514b8d7bf99aaa168b1e4bd15b296154c5677; source9dabd3c. Reviewer inspected source/report only, did not run tests or inspect/rehash raw NPZ. Its result follows verbatim; subsequent root-snapshot check is separately marked in REVIEW_RESPONSE.md.

**Verdict: narrowly scoped GO_MECHANICS_QUALIFIED is supported.** No BLOCKER or HIGH gap found in the supplied source, tests, or reported evidence. H-R3 stays PROVISIONAL / CONFIRMATION HOLD; no P03.

What I inspected versus what I only read as reported: I read the wrapper, test, launcher, analysis, plan, and report text, plus two production files. I did not run tests, open NPZ frames, or rehash manifests. Every "PASS exact" below is the analysis output as recorded in analysis.json, cross-checked only against the analysis code that would have produced it.

**1. Oracle and actual Warp CPU BVH: valid.**
The float64 matrix in `oracle_transform` expands algebraically to the same expression as the production cross-product form, including for non-unit quaternions, so the non-unit contract test is a legitimate independent check rather than a tautology. The Möller-Trumbore oracle is written from scratch and tests every triangle with no BVH. The stale-BVH negative test shows a real Warp CPU mesh returning a miss until `refit`, then matching the oracle, which demonstrates the refit dependency is physical, not simulated. Production builds each Warp mesh from a slice of the updated vertex tensor, as shown at `warp_env_manager.py:153-166`, so the wrapper's pointer-aliasing precondition matches production. I count 11 test methods, consistent with the CPU report.

**2. Step 2 force 127, no natural reset: supported.**
Reset chronology is triangulated three ways in `_events`: the task-level reset wrapper, the Warp geometry event env ids, and the indexed root-flush ids must all agree per frame. All five processes show reset ids of exactly [127] or [], and natural resets empty at step 2. The fixed pair is bitwise equal on all 127 untouched environments for all 41 frames, including vertices and derived kinematic caches.

**3. Separate numeric and SHA sensor comparison, evaluator ON/OFF: supported.**
The launcher stores full RGB, depth, and LiDAR arrays plus per-env SHA rows. `_validate_frame` rejects any frame where SHA and numeric disagree, and `compare` tests both with byte equality. Evaluator ON versus OFF is equal on all 128 environments with nothing excluded.

**4. N16 blindspot: eliminated for this profile.**
The legacy negative shows 126 of 127 untouched environments changing in all three modalities at step 2, so the N128 design has detection power that a small-N fixture would have lacked. I can only assess this from the supplied material.

**5. Selected reset parity, mappings, refit, firewall, FrozenF: supported.**
Env 127 at step 2 is bitwise equal between legacy and selected modes on every field including vertices, so the per-env transform path produced identical bytes to the production global path. Mapping, topology, aliasing, mesh id, and lifetime guards are fail-closed. The launcher hooks only observe and the forced truncation is the intended intervention. Checkpoint SHA, RMS digest, 898-dim observations, eval mode, and 0.1 s timestep are asserted at both ends of each process.

**6. Preservation:** production `reset_idx` is unchanged and still shows the global rewrite; the wrapper's `close` restores the original. Manifest, disk, and preservation.json claims are report-only.

**Minor observations (LOW, non-blocking):**
- `WarpResetIsolation.__init__` resolves `tf_apply` from the bound method's module globals and otherwise falls back to `transform_xyzw`. Process metadata does not record which was bound. The env 127 bitwise parity makes this moot for ISO-Q02. Smallest remedy: record the transform's qualified name in `process_meta.json`.
- The launcher's in-step overlap guard in `post()` only sees collision and truncation flags before the original step. Any other termination source would still be caught by the frame-level reset cross-check, which passed. Remedy: none needed for this result.
- The `max_abs_delta` reported for SHA fields is meaningless noise; harmless.

**Remaining validity limits:** one seed, one density, 40 steps, static obstacle meshes only. The legacy-side onset magnitudes are consistent with quaternion renormalization, but the report's attribution to that specific root change is inferred, not separately measured. The old P02 14 failures remain unexplained retrospectively. External backup is not established.
