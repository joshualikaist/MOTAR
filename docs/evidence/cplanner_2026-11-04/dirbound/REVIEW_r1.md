# Independent review: C-Planner direction-bound repair (D0-D3) and amendment 1 (D2-A1)

- **Reviewer:** Claude Opus 5.5 (`claude-opus-5-5`), fresh context, read-only. I wrote only inside `review/audit/`.
- **Date:** 2026-11-04.
- **Reviewed:**
  - the frozen copy of `8e2b77d` (`frozen/`) and of `f5b4802` (`frozen_a1/`);
  - `git diff 2e5ffd7 8e2b77d` and `8e2b77d f5b4802`;
  - the preregistration, amendment 1 and charter §3-§7;
  - `run_copy/` and `run_a1_copy/` (bound by `SHA256_MANIFEST.json`);
  - the M2 raw G-alpha output `~/motar_tmp/cplanner_m2_20261104/run/galpha.json`, for comparison only.
- **Not used as evidence:** the author's WORKLOG, the summaries, or `docs/evidence/`.

## Verdict: **PASS**

I found no blocking finding.

- **Q1.** The eps construction is sound under the stated bound. It covers every place where sensor geometry enters a certificate. I checked it analytically (Q1 below), and my own full-eps adversary agrees (probe P1):
  - with the repaired configurations: 0 FALSE_CERT in 14,584 segments, 10,904 of them aimed into the band (R − 1.5 mm, R + 0.3 mm);
  - with the unrepaired eps = 0 configurations on the same scans: 1,577 FALSE_CERT.
- **eps = 0.** The eps = 0 configurations are unchanged, byte for byte in their outputs (probe P5).
- **The gates** D0, D1 and D3 (first run) and D2-A1 (amendment run) stand as computed, as do:
  - the carry-over (Q8);
  - the host, input and determinism records.
- **One important finding (I1).** D1 has essentially no power at the scale of the bound, and its candidate segments are not built as preregistered. D1's PASS therefore must not be cited as empirical evidence that the repair is needed or sufficient at the eps scale. That evidence comes from the analytic argument and from P1.
- **Minor findings and limitations** are listed below. None of them can produce a FALSE_CERT under the bound or change D3's reading band.

## Findings

### Blocking

None.

### Important

**I1. D1 is not adversarial at the scale of eps and does not discriminate repaired from unrepaired. Its aimed candidates are not the preregistered "R ± 5 cm" segments.**

- **The adversary is a cone from the declared origin**, half-angle `ALPHA_IN = 0.9 eps / 12 m` (`check_direction_bound.py:42,56-83`).
  - Bars sit 1-7 m from the vehicle (`:93`), where the cone's lateral offset is only 0.075-0.53 mm, i.e. ≤ 0.53 eps.
  - It has no origin offset and no parallel or pivoting rays, although the preregistered model is a tube of radius eps at every range (prereg l.18-20).
  - At a height-clip point at range t it moves the ray by 0.9 eps × t / 12 m (0.07-0.6 mm), never by the full eps.
  - Vehicle heights are U[0.3, 2.7] m (`:88`), so the sensor-height eligibility shrink (`core.py:213-218`) is never exercised.
- **The aimed segments miss the band they were meant to fill.**
  - The prereg (l.72) says "8 aimed to pass a bar at R ± 5 cm".
  - The code aims at a point `max(h) + R ± 5 cm` from the bar centre, perpendicular to the sight line, with length ≤ 2.5 m (`:110-119`). The resulting clearance depends on the bar's extent in that direction and on whether the segment reaches the bar.
  - Probe P3 recomputed D1's 2,000 scenes without casting any ray. Only 1,280 of 16,000 aimed segments have clearance within ±5 cm of R. Only 12 aimed and 3 random segments of 32,000 lie in (R − 1 mm, R], the band where an eps-scale unsoundness would show.
- **No discrimination.** Part C (eps = 0, `MOTAR-72x4-RUNTIME-NATIVE`, at alpha_in) gave **0 FALSE_CERT** (`run_copy/dirbound.json` stats.C: 21,665 certified, 0 FALSE_CERT). So D1 cannot tell the repaired certifier from the unrepaired one.
- **The power control is far from the bound.** Part B (×100) displaces rays by 7.5-52 mm at bar range. Its 46 FALSE_CERT have clearance − R from −0.4 to −19 mm. It shows only that gross violations are detected.
- **Why this is not blocking.** It hides no case: the construction is sound (Q1), and P1 shows the repaired configuration with 0 FALSE_CERT under a full-eps adversary. P1 also has power at that scale: the eps = 0 configuration gives 1,577 FALSE_CERT, including 42 at ordinary heights in [0.3, 2.7] m (recorded examples at clearance R − 0.04 mm and R − 0.70 mm).
- **Required:** disclose the deviation of D1's candidates from the prereg, and do not use "D1 A: 0 FALSE_CERT" as evidence of eps-scale soundness.

### Minor

**M1. The near-range band also excuses NO_HIT beams (D2 and D2-A1).**

- `v1v2_kernelray.py:81-83` and `v1v2a_kernelray.py:61-63` (both inherited from V1, `v1_hitbit.py:124,129-132`) apply `band` both at 12 m and at 0.2 m.
- So a NO_HIT whose oracle first hit is within 0.1 mm of 0.2 m is not counted at all. Yet the certifier's free ray [0.2, 12] would cross that whole object.
- It cannot arise from float32 rounding. It would need the kernel to miss an object at 0.2 m outright. It was never observed (only 54-183 NEAR beams per 1.7 M).
- Fix: apply only the 12 m band to NO_HIT.

**M2. The permissive excuse measures depth only in the oracle's first-hit object.**

- `v1v2_kernelray.py:146-148` and `v1v2a_kernelray.py:158-160` use only the first-hit object.
- A kernel NO_HIT that grazes object A (excused) and misses a deeper object B behind it would pass.
- This needs a kernel defect, which would also show on beams that see B alone. Use the maximum depth over all objects within 12 m.

**M3. The D2-A1 rule controls are largely tautological** (`v1v2a_kernelray.py:113-130`).

- The permissive control selects beams with depth > 1 cm. Its flag condition (`dep > DELTA_GEOM`) is implied by that selection, so the control tests only `classify(1.0, ·)`.
- The conservative control tests `classify(0.25, inf)` and a surface distance at 3 m.
- Neither goes through the main loop's accounting (`:150-182`), the manifest path, or an excuse near its threshold.
- They are non-vacuous only in the weak sense. The main path was exercised by real disagreements:
  - first-run sample: 4 kernel-ray and 6 manifest conservative cases, 2 manifest permissive cases;
  - fresh sample: 4 kernel-ray and 5 manifest conservative cases.

**M4. The cost of the repair is not reported beside D3, as the prereg asks (l.99).** It is also non-monotone.

- `dirbound_summary.json` has no M2 comparison.
- From the raw files, on the same sample (168 episodes, 12,636 frames, 6,089 decision steps):

  | | `-DIR-NATIVE` (D3) | `MOTAR-72x4-RUNTIME` (M2) |
  | --- | --- | --- |
  | certifiable steps | 4,970 (0.81623) | 4,969 (0.81606) |
  | certified segments | 49,969 | 49,999 |
  | UNRESOLVED | 468 | 446 |
  | MAP_VETO | 46,211 | 46,209 |
  | surface refusals | 559 | 553 |

- So the repaired configuration certifies **one more** decision step (in B-PP:205) although it certifies 30 fewer segments.
- Probe P4 replayed D3's B-PP:205 episodes step by step. The eps = 0 side ran on `MOTAR-72x4-RUNTIME-NATIVE`, the exact port of M2's configuration. The group counts are reproduced exactly: 1,967/2,391 for M2 and 1,968/2,391 for D3.
  - The extra step is `205bars__seed4302`, env 111, episode 6, frame 88, segment 6. It is refused at eps = 0 by `MAP_VETO_OBSERVED_HIT` and certified by `-DIR-NATIVE`. Its true clearance is 1.12 m.
  - All 22 segments certified only by `-DIR` come from an eps = 0 map veto.
  - The 31 certified only at eps = 0 are 19 map vetoes and 12 `UNRESOLVED_CENTER_CELL` under the shrink.
  - Neither side has a FALSE_CERT. The smallest clearance among the differing segments is 0.462 m.
- The gain therefore comes from the map. With eps the height-shrunk free rays are slightly shorter, which changes the map's sampled traversal and so which OBSERVED_HIT cells are resolved. The map is veto-only, so this is benign for soundness. D3 also checks every certificate against the layout.

**M5. D2-A1 is stricter than the model in the permissive direction.** This is a false-FAIL risk, not a soundness issue.

- "An overshoot is never allowed" and the adapter-range check (`v1v2a_kernelray.py:67-68,146,174`) fail on a HIT range overshoot of more than 0.5 mm along the ray, even when the incidence is grazing and the perpendicular depth is about a micrometre.
- The fresh sample already contains the conservative mirror image: trace index 23442 of the 70-bar cell, 0.71 mm range error, 1.25e-6 m from the surface.
- The gate passed here, but a rerun on another sample could FAIL with no soundness impact.

**M6. The amendment's monotonicity sentence is right only within scope** (amendment l.31-32).

- "A conservative declaration ... cannot make a certificate unsound" holds for catalogue bars. Certificates come only from the current packet's coverage proof (`core.py:298-313`), and a conservative declaration's free points are a subset of the true ones.
- But the certified set is not monotone in the declarations:
  - a shortened or phantom HIT removes the true surface point from the surface rule;
  - it changes the map's sampled traversal (`evidence_map.py:66-78`);
  - the 1e-9 midpoint test is not monotone in free-ray length (`coverage_b.py:29-30`).
- For out-of-scope objects (the target, non-catalogue objects) a conservative HIT can therefore remove a refusal. State the claim as scoped.

**M7. Provenance gaps (small).**

- Not bound by the input hashes: the Warp kernel cache (`~/.cache/warp`), site-packages (warp 1.0.0, numpy, torch) and gcc.
- `galpha_dir.json` records neither the coverage library path nor its hash. They are bound only through `inputs_before/after` and the driver environment.
- If the library had failed to load, D3 would have read FILTER_PAUSED rather than failing. This did not happen: 49,969 certificates and no `NATIVE_COVERAGE_UNAVAILABLE`.

**M8. An origin inside a box is treated as a hit at the exit** (`v1_hitbit.py:60`, `t = tmax`).

- This mirrors Warp's back-face hit, so D2 would not flag a sensor inside a bar.
- Physically that is a crash state. Segments starting there are refused by the surface rule, because the nearest exit face is ≤ 0.39 m < R.

### Limitations (recorded, not findings against PASS)

- **L1. GPU float32 was not measured** (preregistered).
  - Warp 1.0.0 compiles with `fast_math=False` (`warp/context.py:1315`), but nvcc contracts FMAs by default, so CUDA ray directions and intersections can differ from CPU at the ulp level.
  - Margins: the measured lateral offset is 2.96e-6 m against eps/10 = 1e-4 m (34×) and eps = 1e-3 m (340×).
- **L2. D2's samples are 6,000 frames of the six seed-4301 cells.**
  - Grazing events occur about once per 6,000 frames.
  - No permissive disagreement occurred in the gated fresh sample: `k_PERMISSIVE = m_PERMISSIVE = 0`. The permissive excuse was exercised only on the first-run sample (manifest, 2 cases, 4.7e-7 m).
  - D3's own FALSE_CERT check is what protects the D3 sample.
- **L3. The D2 rule was relaxed after its data were seen.**
  - The relaxation follows the soundness argument (Q7).
  - It used the charter's single repair round and a fresh sample (seed 20,261,110).
  - The first-run FAIL stays recorded.
- **L4. Exact boundary grazing is excluded by the charter.** An example is a clipped endpoint at exactly z = 0 or 3 m. The open-interval argument in Q1 covers every point the proof actually uses.
- **L5. The guarantee is conditional on the bound itself.** It is established only for CPU replays of sampled frames, and statistically.

## Answers to the questions

### Q1. Soundness of the repair

**Assumptions.** Let o be the declared origin and d the manifest direction (world frame, yaw-rotated). Assume:

- for every range t ≤ 12, the probed point p(t) satisfies |p(t) − (o + t d)| ≤ eps;
- every declared status and range is true along the probed ray.

**Free rays** (`core.py:153-166`). Free evidence is the 2-D projection of o + t d for t in [a, b]:

- a = 0.2 (no near-hit assurance);
- b = min(rr − 1e-7, hi) for HIT and min(12, hi) for NO_HIT;
- only for complete packets, in which every beam is HIT or NO_HIT.

For t < rr the probed point p(t) is free. Its 2-D projection is within eps of the declared point. eps is not needed at the start or the end of the ray, because the model compares points at equal range.

**Height** (`core.py:209-224`). With eps > 0:

- eligibility requires z ∈ (eps, 3 − eps);
- non-horizontal rays are clipped where o + t d reaches eps or 3 − eps.

The coverage test uses the midpoint of a strictly interior chord, so the free point it relies on has t strictly inside (a, b). Hence o + t d has height strictly inside (eps, 3 − eps), and p(t) has height strictly inside (0, 3). Bars are full height, so the projection of p(t) lies outside every footprint.

This shrink is necessary. Without it, a 1 mm vertical error at a 20° or 10° beam's clip point moves the 2-D endpoint 2.7-5.7 mm into a footprint, and a sensor within eps of 0 or 3 m sees over or under the bars (the B1-R scene has no ground plane). P1 confirms this: of the 1,577 eps = 0 FALSE_CERT, 1,535 have the sensor within 5 cm of 0 or 3 m, 1,502 of them within eps.

**Coverage proof** (`core.py:304-309` → `coverage_b.py:42-73`, `coverage_b.c:103-166`; both take the shapes from the caller, `coverage_native.py:57`). With eps > 0 the proof receives shapes h − eps and radius R + 2 eps.

- **Initial box and pruning.** Let F be a true footprint within R of the segment. Then its shrunk footprint F' is within R + √2·eps of it. So F's centre lies in the shrunk problem's initial box (`:52-53`) and is never pruned (`:59`). Pruning is sound for the shrunk problem.
- **Covered test** (`:62-63`).
  - The common core of the shrunk shapes is the true core shrunk by eps on each side.
  - A declared free point strictly inside it (margin 1e-9) has its probed point within eps in 2-D, hence within eps per axis. That probed point is strictly inside the true core, which lies inside every candidate footprint.
  - That contradicts freeness, so no bar centre lies in the cell.
  - The registry assertion (`registry.py:130-131`, h > 10 eps) keeps the shapes positive.
- **Surface rule** (`:45`). It uses R + 2 eps. It is refusal-only and not needed for bars.

**Other paths.**

- The map veto (`core.py:298-300`, `evidence_map.py:148-173`) is veto-only.
- The arena check (`core.py:294-297`) involves no sensor geometry.
- Neither needs eps; both keep R.
- The label layer and the camera are not on the certify path.

I found no path where eps is needed and not applied.

**eps = 0.**

- The only changes are under `if eps:`, and 0.0 is falsy.
- `shapes, radius` are the identical objects `REG.BAR_HALF_SIZES, R`.
- Of `src/`, only `core.py` and `registry.py` differ from `2e5ffd7`.
- Probe P5 ran the pre-repair and the repaired core in separate processes over 60 scenes × 5 steps × 5 eps = 0 configurations: SYNTH-72x1, SYNTH-72x4, SYNTH-72x4-RUNTIME, MOTAR-72x4-RUNTIME and MOTAR-72x4-RUNTIME-NATIVE.
  - Inputs: exact and perturbed declarations; the map is exercised (1,598 MAP_VETO, 320 incomplete).
  - Result: 1,500 step records and 24,000 certify results, **byte-identical** (sha256 `1da1c952…`).
- D0's seven suites also pass on the tree.

### Q2. D1

- **Construction.**
  - The "most permissive within the cone" is built correctly for a cone: the maximum nearest-hit distance over the manifest ray and 2 × 8 cone rays at alpha and alpha/2, and NO_HIT if any ray misses.
  - The basis and the tangent construction are correct.
- **Ground truth** (`harness.classify` → `snapshot/c_review/indep.clearance`) is independent of `src/`.
- **Checks.** A, B, D and E are non-vacuous:
  - A certified 21,636, all 2,000 steps accepted;
  - E compared 32,000 segments with 0 mismatches.
- **The outputs support the preregistered PASS.**
- **Adequacy.** D1 is not adversarial enough and its power control is not meaningful at the bound's scale (I1).

### Q3. D2 and D2-A1

- **Reconstruction (probe P2).** I made an instrumented copy of the MOTAR kernel that only adds a store of `ray_direction_world`, with the Warp cache redirected into `audit/`. On 64 frames, including first-run trace index 14571, 160-bar 5575, 70-bar 23442 and the M2 V1 frame 10752:
  - its distances are bit-identical to the original kernel's;
  - its rays are bit-identical to `v1v2_kernelray.kernel_rays()`;
  - frame 14571 reproduces 4 kernel hits that the float64 oracle along those rays does not see within 12 m.
- **Oracle.**
  - `v1_hitbit.oracle` is independent float64 numpy code (slab test plus sphere); it shares only the layout inputs.
  - `depth_inside` samples 65 points of a concave depth profile, so it underestimates the depth by at most about 3 %.
- **Thresholds.**
  - DELTA_GEOM = 1e-5 m: float32 vertices at ≤ 40 m round by ≤ 1.9e-6 m per coordinate, and the measured excuses are ≤ 1.25e-6 m.
  - K2 = eps/10: measured 2.96e-6 m (first run) and 2.70e-6 m (fresh); origin rounding is 0 because `p_pos` is float32.
  - K3 = eps: this matches the model exactly. Together with the height clip, 3-D depths up to eps in the top or bottom slab are never used as free evidence.
  - Range tolerance 0.5 mm: half the adapter margin.
  - All of these are justified.
- **What D2-A1 establishes for the sampled frames:**
  - kernel ray within 3e-6 m of the manifest ray;
  - no permissive disagreement along the kernel ray;
  - conservative disagreements only where grazing (≤ 1.25e-6 m from a surface);
  - no unexcused manifest disagreement;
  - permissive overshoot ≤ 0.18 mm;
  - adapter mapping exact.
  - So the effective bound on these scans is about 1e-5 m, far below eps.
- **Not covered:**
  - GPU float32 (L1);
  - frames outside the sample (L2);
  - the band and owner holes (M1, M2);
  - the Warp mesh's behaviour on configurations never sampled.

### Q4. D3

- **Computed as preregistered.** `galpha_replay.py` ran with `CONFIG` from the environment, which equals `MOTAR-72x4-RUNTIME-DIR-NATIVE` in both runs.
  - Sample: seed 20,261,105, 6 episodes per cell, 28 cells.
  - Decision steps: even frames, accepted packet, target > 0.9 m.
  - FALSE_CERT: exact 2-D segment-rectangle distance ≤ 0.45 m to every recorded active bar (`:140-150`), with an identity-orientation guard (`:109-110`).
  - Reading thresholds as in the charter (`:219-229`).
- **Result:** 0 FALSE_CERT, rate 0.81623, FILTER_ARM_RETAINED, 21.6 pp above the 60 % threshold. The duplicate is identical.
- **Comparison with M2** on the same sample: see M4 and P4.

### Q5. Driver and provenance

- **Frozen trees.**
  - `frozen/` equals `git show 8e2b77d` on 113 of 113 files; `frozen_a1/` equals `f5b4802` on 116 of 116 (`FROZEN_COMMIT` excluded).
  - `meta.json` points at these trees.
- **Native library.**
  - Built inside the run directory from the frozen `coverage_b.c` (src sha256 `d559bce4…`). The library's sha256 `afbd3e68…` is the same in `build.json`, in the copy and in `run/`.
  - Bound in `inputs_before/after` and passed through `CPLANNER_COVERAGE_SO` (`run_dirbound.py:42-48,86-96`).
- **Host.** Every record has `host_valid` true, fault count 0 before and after, the same boot id, and online CPUs 0-3,6-23 unchanged.
- **Inputs.** `inputs_before == inputs_after` (177 and 178 entries). I re-hashed every bound file now: all match.
- **Duplicates.** All agree after timing removal: D1, D2 (FAIL in both runs), D3 and A1.
- **Verdicts are fail-closed.**
  - First run: FAIL, because D2 FAILed (`run_dirbound.py:144-153`).
  - A1: CONDITIONAL only with carry-over, both A1 runs PASS, a1_d2 REPORTED, unchanged inputs and no host fault (`run_dirbound_a1.py:108-111`).
- **Manifest.** `SHA256_MANIFEST.json` covers all 66 files in the two run copies, all match, and there are no unlisted files.
- **The outputs support both summaries.**
- Gaps: M7.

### Q6. Other things a gate reading would hide

- D1's PASS carries no eps-scale information (I1).
- D3's rate does not measure the cost of the repair. The +1 step comes from map-veto sampling, and the shrink alone costs 12 refusals in B-PP:205 (M4).
- D2-A1's gated sample never exercised the permissive excuse (L2).
- D2-A1 can falsely FAIL on grazing overshoot (M5).
- The eps model is stronger than what the certifier needs. Grazing range errors larger than 1 mm along a ray are harmless under the shrink but violate "ranges true along the probed ray". So the gate rules can be stricter than soundness requires, never looser, except for M1 and M2.

### Q7. Amendment 1

**The distinction is correct for this certifier**, for catalogue bars. A certificate requires:

- a complete packet;
- the coverage proof on that packet's own free rays and surfaces;
- no map veto;
- the arena check.

A conservative declaration (NEAR_CLIPPED, a shortened HIT, a phantom HIT) leaves only free points that are truly free along the probed ray, or makes the packet incomplete. So it cannot create an unsound certificate:

- not through the map, which is veto-only and never supplies evidence to the proof;
- not through the label layer, which is outside the certify path;
- not through occlusion rays, which are map-only;
- not through the next packet, whose certificates rest on its own evidence.

It can change which segments are certified (M6), but not soundness.

**The excuses are sound.**

- The permissive excuse is a depth limit, and it is within the eps budget.
- The conservative excuse (surface distance) is not needed for soundness; it detects phantom objects and replay defects.
- The rule controls are weak (M3).

**Changing the rule after the run** is acceptable: it was the charter's single repair round, it relaxes only the direction that soundness does not need, it used a fresh sample, and the first-run FAIL stays recorded (L3).

**The fresh-sample design is adequate** for detecting systematic defects. It has limited power for events rarer than about 1e-4 per frame (L2).

### Q8. Carry-over

- `src/`, `native/` and `motar/` are byte-identical between `frozen/` and `frozen_a1/`: 12 files, checked with `diff -r` and by the driver (`carry_differences: []`).
- The whole trees differ only by `FROZEN_COMMIT`, `README.md` and the two new files plus the amendment. The tests behind D0, D1 and D3 are therefore identical too.
- D0, D1 and D3 of the first run apply.

## What I verified and how (probes in `audit/probes/`, outputs in `audit/out/`)

| Probe | What | Result |
| --- | --- | --- |
| P1 `p1_full_eps.py` | Own vectorised caster and exact 2-D distance. Adversary of 81 straight rays per beam with origin offset and tilt, ≤ 0.99 eps at every t ≤ 12 m (max offset 0.99000 mm, checked). Sensor heights include the regions near 0 and 3 m. 12 of 16 candidates aimed at clearance R − U(−0.3, 1.5) mm. 2 × 400 scenes native and 2 × 60 verbatim. | Repaired (`-DIR-NATIVE` / `-DIR`): **0 FALSE_CERT** in 14,584 segments; 1,984 certified, so non-vacuous. eps = 0 (`-NATIVE` / `MOTAR-72x4-RUNTIME`) on the same scans: **1,577 FALSE_CERT** (1,345 native + 232 verbatim), 42 of them at heights in [0.3, 2.7] m. Exact scans: 0 FALSE_CERT for both. |
| P2 `p2_kernel_rays.py` | Instrumented kernel copy vs the original kernel and `kernel_rays()`, 64 frames. | Distances and rays bit-identical on all 64 frames. |
| P3 `p3_d1_clearance.py` | Clearance of D1's candidates (2,000 scenes), recomputed. | 1,280 of 16,000 aimed within ±5 cm of R; 15 of 32,000 in (R − 1 mm, R]. |
| P4 `p4b_galpha_diff.py` | D3 sample, B-PP:205 (42 episodes), per step eps = 0 (`-NATIVE`) vs `-DIR-NATIVE`. The first version, with the verbatim eps = 0 side (`p4_galpha_diff.py`), hit its 20-minute timeout and is not used. | Group counts reproduced exactly. The +1 step comes from a map-veto difference (M4). 0 FALSE_CERT on both sides. |
| P5 `p5_eps0_diff.py` | eps = 0 configurations, `2e5ffd7` vs `8e2b77d` core. | 1,500 records and 24,000 results byte-identical. |

Also checked:

- the run-copy manifest;
- the frozen trees against git;
- re-hashing of all bound inputs;
- library hashes;
- every record's host fields;
- every D0 suite's verdict key;
- the D1, D2, D2-A1 and D3 JSON statistics against the summaries;
- the M2 raw G-alpha output.

Kernel fault count (`journalctl -k -b 0 | grep -cE "Machine check|segfault|general protection"`): **0 before** (12:06) and **0 after** my probes (see `kernel_fault_count.txt`). At most 2 probe processes ran at once, CPU only, `taskset -c 0-3,8-11,14-23`.
