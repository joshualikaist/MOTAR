# S4F-1 v1 — results review r1

- **Reviewer:** Claude Opus 5.5 (fresh context), 2026-11-08 ~18:15–18:40 KST. I read the run outputs and the repository
  and wrote nothing to the repository. CPU probes ran under `nice -n 19 taskset -c 6,7`, stdlib only. No GPU, Gazebo or
  PX4 was used.
- **Inputs:** prereg `docs/plans/prereg_sitl_s4f1_v1_2026-11-08.md` at `29717ba`, runner `tools/sitl_s4f1.py`,
  worktree MOTAR-camonly (HEAD `ab2eeb5`, branch camonly/dev, clean). Run outputs are under
  `~/motar_tmp/s4f1_20261108/`. The references are S4 attempt01's ledger (`522f716e…`, as pinned) and S3b attempt01's
  ledger (`97b94dfd…`, as pinned).
- **Probes in this directory:**
  - `recompute.py` → `recompute.json`: per-cell numbers, Clopper–Pearson intervals, paired effects, Holm, S4 and S3b
    tables.
  - `extra.py` → `extra.json`: agreement κ, bootstrap intervals, run position, timeouts.
  - `position.py` → `position.json`: the run-position permutation test.
  - `ulog_check.py` → `ulog_check.json`: decompresses every `.gz` and checks it against its record.

## Verdict: **PASS_WITH_LIMITS**

- The execution is intact.
- The REPORTED label follows the prereg's rules.
- My own code reproduces every analyzer number I checked, to the digit.
- **Neither primary contrast reached significance after Holm.**
- One MAJOR interpretive limit must lead the results record. Outcomes do not reproduce scene by scene from one boot to
  the next: lit_rendered's agreement with S4 attempt01 on the same 39 scenes is exactly what chance predicts (κ ≈ 0).
  This does not invalidate the tests, but it limits what the record may say about any single scene.

## 1. Execution integrity

| Check | Result |
| --- | --- |
| Approval | `execution_approval.json` (SHA-256 `9f418b61…`, the one bound in `attempt.json`) has a `scope` byte-equal to the runner's `SCOPE` (I parsed it with `ast`) and `commit` = `ab2eeb5` = HEAD at every launch. The approval is a blanket same-day approval (12:07 KST, quoted) that predates the prereg commit (15:42); see LOW-2. |
| Source binding | The runner's SHA-256 `4b079db9…` equals the prereg pin and its bytes at `29717ba`. `component_smoke.py` is `14925afb…` = pin. Between `29717ba` and `ab2eeb5`, 24 files changed (the CAM-K2 prereg and the CAM-K1-E record). None is on the import path of `sitl_s4f1`, `sitl_s4`, `sitl_s3c`, `sitl_s3b_formal` or `sitl_s4_gazebo`; the diff on those is 0 lines. |
| Component smoke | `result.json` (SHA-256 `42281419…` = bound) reads PASS with C1–C9 all true, using the preregistered thresholds. It was produced by the pinned script on the runner's current bytes at `ab2eeb5`. Measured values: <br>• C1: emissive pass fraction 32/34 = 0.941 in both emissive cells (≥ 0.80). <br>• C2: lit 4/34 = 0.118 (≤ 0.50). <br>• C3: mask 34 px; centroid offset about 0.04 px; 0 passes outside the dilated mask. <br>• C4: 0 LiDAR and 0 depth returns removed outside the bar feet when unrendered. <br>• C5: 68 LiDAR and 6160 depth returns removed when rendered. <br>• C6: 2.800 / 1.800 / 2.700 m. <br>• C7: rest z 0.089 m (both grounds). <br>• C8: 0.000. <br>• C9: 0.000. <br>It took 36 s, against the roughly 15 min predicted; it was not affected. |
| Cell smokes | All four `smoke01_<cell>` are COMPLETE with 1 of 1 episode, VALID, of their own cell, at HEAD `ab2eeb5` and clean. Kernel faults were 0 → 0. Start-check EKF − truth altitude was +0.009, +0.019, +0.010 and +0.026 m (≤ 0.05). |
| Attempt01 | COMPLETE with 160 of 160 recorded and **160 VALID**, no INVALID reason and no exception. Kernel faults were 0 → 0, `git_after` is clean at `ab2eeb5`, and the chain ended with CHAIN DONE (rc = 0 at every step). |
| Latin square | I rebuilt the order from scratch: 40 scenes in ascending env, scene *i* follows row *i* mod 4 of ((0,1,3,2),(1,2,0,3),(2,3,1,0),(3,0,2,1)). The ledger's (env, cell, position, square_row) sequence matches it exactly. Indices are contiguous, and each cell takes each position 10 times. |
| Host | Chain STATE lines show faults = 0 before and after every step and an empty GPU list throughout. Free disk went from 2.17 to 1.56 GB. The kernel journal from 15:40 to 18:12 has no MCE, segfault, oops or hardware error. CPUs 4–5 are offline (known hazard), and the runner used 6, 7, 12 and 13. Every episode has `host_gpu_before.gpu_ok` with no job (GPU_BUSY and GPU_UNCHECKED are 0 in every cell) and `host_idle_before.ok`. No episode has a parameter problem, and there were 0 hover-guard failures. The maximum lateness in any cell was 0.136 s (emissive_rendered), under the 0.15 s bound. |
| ULog | My independent decompression covered 160 files, with 0 mismatches against their records (`.gz` SHA-256 and size, decompressed SHA-256 and size). Every file has `verified` and `original_removed` true, and no stray or leftover `.ulg` remains. Size went from 1,644,581,884 to 547,435,406 B (0.333). This matches the analyzer's `ulog` block. |
| Replication block | `SAME_STACK_AND_WORLDS` with `stack_compared`, `models_compared`, 39 worlds compared and 39 equal. I separately confirmed that the bound world SHA-256 of lit_rendered equals S4's on all 39 pairs. S4's env 37 was INVALID in S4. |
| Tests | `tests_s4f1_alone` (34 tests), `first` (86) and `last` (86) all read OK. |
| Label | REPORTED. The attempt is COMPLETE, the fault count did not rise, `integrity_problems` is `[]`, and 40 of 40 scenes are VALID in all four cells (≥ 30). The label is correct under the prereg's VOID / INSUFFICIENT / REPORTED rules. |

## 2. Recomputed numbers (own code; all match `reading.json`)

**Per cell** (n = 40 each, Clopper–Pearson 95 %):

| Cell | Capture | Crash | Timeout | Camera recall | Colour/shadow px ratio | Median steps | Start EKF − truth alt (m) |
| --- | --- | --- | --- | --- | --- | --- | --- |
| lit_rendered | 22 (55.0 %, 38.5–70.7) | 14 (35.0 %, 20.6–51.7) | 4 (10.0 %, 2.8–23.7) | 0.530 | 0.148 | 112 | −0.006 to +0.022 |
| emissive_rendered | 24 (60.0 %, 43.3–75.1) | 16 (40.0 %, 24.9–56.7) | 0 (0.0 %, 0–8.8) | 0.805 | 0.915 | 66 | −0.007 to +0.025 |
| lit_unrendered | 27 (67.5 %, 50.9–81.4) | 11 (27.5 %, 14.6–43.9) | 2 (5.0 %, 0.6–16.9) | 0.521 | 0.153 | 105 | +0.001 to +0.022 |
| emissive_unrendered | 30 (75.0 %, 58.8–87.3) | 10 (25.0 %, 12.7–41.2) | 0 (0.0 %, 0–8.8) | 0.849 | 0.907 | 49 | −0.006 to +0.025 |

Isaac (S3b) captured all 40 scenes, so a cell's agreement with Isaac equals its capture count.

**Paired effects on capture** (40 complete scenes; exact two-sided sign tests on scenes with a non-zero contrast):

| Contrast | Mean (pp) | + / − / 0 scenes | Raw p | Holm p | Status |
| --- | --- | --- | --- | --- | --- |
| **Main A** (emissive − lit), primary | +6.25 | 16 / 11 / 13 | 0.442 | 0.442 | not reached |
| **Main G** (unrendered − rendered), primary | +13.75 | 17 / 6 / 17 | 0.0347 | **0.0694** | **not reached** |
| A × G, secondary | +2.5 | 12 / 12 / 16 | 1.000 | — | descriptive |
| A given rendered (McNemar) | +5.0 | 11 vs 9 | 0.824 | — | descriptive |
| A given unrendered | +7.5 | 11 vs 8 | 0.648 | — | descriptive |
| G given lit | +12.5 | 11 vs 6 | 0.332 | — | descriptive |
| G given emissive | +15.0 | 10 vs 4 | 0.180 | — | descriptive |

**Crash contrasts** (all secondary):
- Main A: +1.25 pp (12/12/16), p 1.000.
- Main G: −11.25 pp (5/14/21), p 0.064.
- A × G: −7.5 pp (9/11/20), p 0.824.

**Descriptive only, not preregistered:**
- Scene-bootstrap 95 % intervals (20,000 draws): main G +1.2 to +26.2 pp; main A −10.0 to +22.5 pp.
- emissive_unrendered − lit_rendered: +20.0 pp (−2.5 to +40.0); discordant pairs 14 vs 6, McNemar p 0.115.
- Timeouts: lit 6/80, emissive 0/80.

**lit_rendered against S4 attempt01's `ground` arm** (39 scenes VALID in both):
- Same outcome on **17 of 39**. If the two runs were independent, given their margins, chance alone would predict
  17.08 (κ = −0.004).
- On capture alone, the runs agree on 20 of 39, against 19.5 expected by chance (κ = 0.02).
- Captures: S4 20, S4F-1 21. Capture-discordant scenes: 9 captured only in S4, 10 only here.
- Cross-table (S4 → S4F-1):

| S4 → S4F-1 | Scenes |
| --- | --- |
| capture → capture | 11 |
| capture → crash | 8 |
| capture → timeout | 1 |
| crash → capture | 10 |
| crash → crash | 5 |
| crash → timeout | 2 |
| timeout → crash | 1 |
| timeout → timeout | 1 |

**Between the cells of S4F-1:** capture agreement κ ranges from −0.15 to +0.22 across the six pairs; same-outcome
counts range from 19 to 26 of 40.

## 3. What may and may not be said

**May:**
- The attempt was COMPLETE with 160/160 VALID, the label is REPORTED, and S4's ground arm was replicated on the same
  stack and worlds (39 of 39).
- **Neither primary contrast reached significance after Holm.**
  - Main G on capture: +13.75 pp, raw p 0.035, Holm p 0.069.
  - Main A: +6.25 pp, p 0.44.
- Descriptively, the G direction was the same in both simple effects (+12.5 and +15.0 pp), and crashes fell under G
  (−11.25 pp, p 0.064, secondary). That is a description, not a finding.
- Both manipulation checks show the factors took effect:
  - **A:** the colour-rule to shadow pixel ratio was 0.91–0.92 emissive against 0.15 lit. Camera recall was 0.80–0.85
    against 0.52–0.53.
  - **G:** at hover (step 0), ground LiDAR removals had a median of 62.5–63 beams with the ground rendered, against
    0–0.5 unrendered. Per step the medians were 46–47 against 1 beam, and depth medians were 2561–3455 against 51–56
    px.
- Per-cell rates with their intervals, on these 40 scenes and one boot per cell per scene.
- The best cell (emissive_unrendered, 75.0 %, CP 58.8–87.3 %) remains below Isaac and S3b (40/40). Its upper bound is
  below 100 %, so both changes together leave a gap of at least about 13 pp, and the point gap is 25 pp. The gap
  includes at least prereg item 8's list, **including run-to-run variation**, and is not attributed to any one item.

**May not:**
- "G improved capture", "G significant", "trend toward significance", or using the raw p or the bootstrap interval to
  override Holm.
- "No effect of A" (a non-significant result is not evidence of no effect). Under Holm, power at +25 pp was about
  75 %. The data are compatible with an A effect from about −10 to +22 pp (descriptive).
- Attributing G to the filter margin, or A to blanking (the prereg's compound-factor list applies verbatim).
- Anything about real-camera detection, an onboard ground filter, mount, FOV, timing or depth-range effects, S3b
  acceptance or flight readiness.
- "The two factors explain 20 of the 45 pp gap". The combined cell moved capture by +20 pp descriptively (McNemar p
  0.115, not preregistered), and attribution or additivity is not supported.
- **Any per-scene reading** (see MAJOR-1). This rules out statements such as "scene X fails under lit but not
  emissive" and "the scenes G rescues". It also covers the matched-scene tables against Isaac, S3b and S4 read scene
  by scene.

## 4. Findings

**MAJOR-1: outcomes do not reproduce scene by scene across boots; the record must lead with this limit.**
- **Evidence.** S4F-1 ran the same stack and the same world bytes as S4 attempt01 (SAME_STACK_AND_WORLDS). Its
  lit_rendered outcomes agree with S4's on 17 of 39 scenes, against 17.08 predicted by chance (κ ≈ 0), and on capture
  alone 20 vs 19.5 (κ 0.02). The capture *rate* replicated (20/39 against 21/39). Agreement between cells is also
  near chance (κ −0.15 to +0.22).
- **What this means.** In Gazebo-sensor SITL near a 50–75 % capture rate, a scene's outcome is effectively a fresh draw
  on every boot. S3b's block C (10/10 repeats, MOTAR renderer, 100 % capture) does not bound this variation and must
  not be cited for it.
- **Consequences.**
  - (a) The sign and McNemar tests remain valid. Under H0, boot noise keeps the cells exchangeable within a scene, and
    the Latin square balances position. The pairing, however, buys little variance reduction, so the prereg's power
    model (independent Bernoulli, no scene heterogeneity) is roughly the right one.
  - (b) Every per-scene table (against Isaac, S3b and S4, the per-scene contrast counts, the discordant pairs) is boot
    noise at the scene level and is reported only as counts.
  - (c) Each cell rate is one 40-boot sample; with ±15 pp intervals, differences of 5–10 pp between cells are within
    run-to-run noise.
  - (d) The same limit applies backwards to S4's single-boot per-scene readings, such as "11 of 39 ground episodes kept
    the ground's LiDAR signature at hover" and the per-scene mechanism notes. The S4 results record should get a
    cross-reference.
  - (e) The cause is not identified. Candidates such as PX4/EKF and Gazebo timing nondeterminism are NOT_TESTED. "Run
    to run" here includes a different day and a different position in the sequence, not only the boot.
- **Required.** State this in the results record's summary, not only in its body.

**MINOR-1: wording around main G.**
- Raw p 0.035 sits below α while Holm p 0.069 does not. The record must use the prereg's "reached" only for Holm, say
  "not reached", and give raw and Holm p together.
- The consistent direction (both G simple effects and crash G) may be described, labelled secondary and descriptive.
- The bootstrap interval and the EU − LR contrast must be marked as not preregistered.

**MINOR-2: the G manipulation in flight is "consistent with bar feet only", not verified as such.**
- The unrendered cells still removed some ground LiDAR returns at hover: in 17 of 40 (lit) and 20 of 40 (emissive)
  episodes, at most 5 and 3 beams.
- Per step, depth removals had medians of 51/56 px and maxima of 2587/1512 px.
- The flight path has no bar-foot mask; the component smoke checked the mask only in its static rig. These removals
  are plausibly bar feet and tilted bar bases, two orders of magnitude below the rendered cells.
- The record should give the numbers and say "consistent with", not "only bar feet".

**MINOR-3: per-step manipulation and recall summaries are taken over different step populations.**
- Emissive episodes end sooner: median 66/49 steps, against 112/105 for lit. Shadow-visible steps were 1712/1309,
  against 2470/2309.
- So medians of removals, the colour/shadow ratio and recall are not computed over matched phases of flight. This is
  descriptive and should be disclosed.

**LOW-1: capture rose with within-scene run position, post hoc.**
- Positions 0–3 gave 22, 23, 28 and 30 of 40. A permutation stratified by cell gives p ≈ 0.04, uncorrected and one of
  several looks.
- Each cell held each position 10 times, so the cell contrasts are not biased.
- No logged covariate differs by position: median lateness, frame age, RTF, load and spawn offset.
- Probably chance. Disclose it as exploratory, and as further reason not to read scenes individually.

**LOW-2: provenance to disclose.**
- The prereg is at `29717ba`, and every launch ran at `ab2eeb5`. The two commits in between change only CAM-K2 and
  CAM-K1-E files, and the runner's bytes equal the prereg pin.
- The approval is the user's blanket same-day approval (12:07 KST), quoted in `execution_approval.json`, and it
  predates the prereg commit. The file names this runner's SCOPE (byte-equal) and the run HEAD.

**LOW-3: 0/80 timeouts with emissive against 6/80 with lit.**
- This is not a preregistered contrast and was not tested. It may be described alongside the shorter episodes.

**LOW-4: `wall_s` includes the gzip** (as preregistered). attempt01 took 2 h 22 min, inside the 2.2–2.7 h estimate.

## 5. Disclosures the results record must carry

1. **Label and status.** REPORTED; COMPLETE with 160/160 VALID; kernel faults 0 → 0. The prereg is at `29717ba` and
   the runs were at `ab2eeb5` (runner bytes pinned). The approval is the user's blanket same-day approval, quoted.
2. **Primary contrasts.** Raw and Holm p for both, the words "neither reached", and "a non-significant result is not
   evidence of no effect", with the Holm power figures (about 75 % at +25 pp).
3. **Secondary results.** All of them are labelled descriptive: the interaction, the simple effects and the crash
   contrasts.
4. **Per-cell table.** Counts, rates and Clopper–Pearson intervals; recall; start altitude; lateness maximum.
5. **Manipulation checks.** The numbers, the "consistent with bar feet" wording (MINOR-2) and the
   step-population caveat (MINOR-3).
6. **Replication.** SAME_STACK_AND_WORLDS on 39/39, with outcome agreement 17/39 at chance level (κ ≈ 0), the
   capture-rate agreement, and MAJOR-1's consequences for every per-scene table, including S4's.
7. **The compound-factor "may not" list,** the red-sphere colour rule (the real target is a drone; the YOLOv5s lineage
   is not connected), the unrendered ground as a simulator-only condition, and no S3b-acceptance or flight claim.
8. **The remaining gap** to Isaac and S3b: 25 pp in the best cell (CP upper bound 87.3 %), with prereg item 8's
   non-exclusive list including run-to-run variation, not attributed to any one item.
9. **Host and data.**
   - GPU and idle checks were clean in every episode; CPUs 4–5 offline; no journal faults.
   - ULog: 160/160 gzipped and decompression-verified (and independently re-verified by this review), 1.64 → 0.55 GB;
     `wall_s` includes the gzip.
10. **Exploratory observations,** flagged as such: the run-position trend (LOW-1) and the timeouts (LOW-3).
