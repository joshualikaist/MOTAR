# S4F-1 v1 — SITL 2×2 of target appearance × ground on S4's ground arm (2026-11-08)

**Status:** ran 2026-11-08 at `ab2eeb5`, 15:44–18:10 KST, reading **REPORTED**.
- The component smoke passed C1–C9.
- Four cell smokes were VALID, and attempt01 was COMPLETE with 160 of 160 VALID.
- Host faults 0 → 0.
- **Neither primary contrast reached significance after Holm.**
- Results review r1 (Claude Opus 5.5): **PASS_WITH_LIMITS**.

Read the [preregistration](../../../plans/prereg_sitl_s4f1_v1_2026-11-08.md) first.

- [`REVIEW_design_r1.md`](REVIEW_design_r1.md) and [`REVIEW_design_r2.md`](REVIEW_design_r2.md): the design reviews
  (Claude Opus 5.5, fresh context). Their probes stay in `~/motar_tmp/s4f1_20261108/review_design_r{1,2}/`.
- Code: `tools/sitl_s4f1.py`, `tests/test_sitl_s4f1.py` (54 tests, CPU).
- Outside the repository, bound by SHA-256 in the prereg: `~/motar_tmp/s4f1_20261108/component_smoke.py` and the
  chain `run_s4f1.sh`. The implementer's notes are in `IMPLEMENTATION.md` there.

## Results (attempt01, 2026-11-08)

**Records** in this folder:
- `reading_attempt01.json`: the runner's `--analyze` output, a byte copy.
- `attempt01.json`: the attempt record (bindings, order, host).
- `component_smoke_result.json`: the component smoke.
- `SHA256_MANIFEST.json`: 2,021 raw files under `~/motar_tmp/s4f1_20261108/` (episodes, gzipped ULogs, smokes, chain logs,
  approval).
- [`RESULTS_REVIEW_r1.md`](RESULTS_REVIEW_r1.md): the results review, with its own recomputation and probes in
  `~/motar_tmp/s4f1_20261108/review_results_r1/`.

**Component smoke** (gzserver only, fixed thresholds):
- emissive target pass fraction 0.941 (needed ≥ 0.80), lit 0.118 (needed ≤ 0.50);
- no floor returns outside the bar feet with the ground unrendered;
- resting height 0.089 m in both worlds.

**Per cell** (n = 40 each; Clopper–Pearson 95 %):

| Cell | Capture | Crash | Timeout |
| --- | --- | --- | --- |
| `lit_rendered` (S4's `ground` arm) | 22 (55.0 %, 38.5–70.7) | 14 | 4 |
| `emissive_rendered` | 24 (60.0 %, 43.3–75.1) | 16 | 0 |
| `lit_unrendered` | 27 (67.5 %, 50.9–81.4) | 11 | 2 |
| `emissive_unrendered` | 30 (75.0 %, 58.8–87.3) | 10 | 0 |

**Primary contrasts** (capture; exact two-sided sign tests on the 40 paired scenes, Holm over the two):

| Contrast | Effect | Scenes + / − / 0 | p raw | p Holm | Result |
| --- | --- | --- | --- | --- | --- |
| Ground: unrendered − rendered | +13.75 pp | 17 / 6 / 17 | 0.035 | 0.069 | not reached |
| Appearance: emissive − lit | +6.25 pp | 16 / 11 / 13 | 0.44 | 0.44 | not reached |

**Manipulation checks** (both factors took effect):
- **Appearance:** colour-rule to rendered target pixels was 0.91–0.92 emissive against 0.15 lit. Camera recall was
  0.80–0.85 against 0.52–0.53.
- **Ground:** LiDAR ground returns removed at hover had a median of 62.5–63 rendered against 0–0.5 unrendered.

**What may be said:**
- In this SITL setup with Gazebo's own sensors, neither not rendering the floor nor an emissive target produced a
  significant change in capture on 40 paired scenes after Holm.
- The ground effect's direction (+13.75 pp, raw p 0.035) is secondary only.
- The best cell captured 75 % (upper bound 87.3 %), still about 25 pp below Isaac and S3b. The remaining gap is not
  attributed to any one cause; candidates include the tilting mount, the FOV and pixels, frame timing and the depth
  range.

**Disclosed, from the results review:**
- **MAJOR-1, scene outcomes do not repeat across runs.**
  - Per scene, the replication cell agrees with S4 attempt01 on 17 of 39 scenes, where chance predicts 17.1 (κ ≈ 0).
  - Agreement between cells is also near chance (κ −0.15 to +0.22).
  - The capture rate replicated: 21 here against S4's 20 on the matched scenes.
  - So no per-scene table may be read scene by scene: not this one, not S4's, not one against Isaac or S3b. The paired
    design gains little.
  - S3b's block C (10/10 second boots) does not cover Gazebo-sensor runs. The cause is NOT_TESTED.
- **MINOR-1:** the ground factor was not reached (raw p 0.035, Holm p 0.069). The bootstrap interval (+1.2 to
  +26.2 pp) and the `emissive_unrendered` − `lit_rendered` contrast (+20 pp, p 0.115) are not preregistered.
- **MINOR-2:** in flight, unrendered cells still removed up to 5 LiDAR beams at hover (17 and 20 of 40 episodes) and
  up to 2,587 depth pixels in a step. The ground manipulation is "consistent with bar feet", not verified as such.
- **MINOR-3:** the per-step medians cover different step populations. Emissive episodes were shorter: a median of 66
  and 49 steps against 112 and 105.
- **LOW-1:** capture rose with run position within a scene (22, 23, 28, 30 of 40; post hoc p ≈ 0.04). The Latin
  square balances it.
- **LOW-2:** provenance. The prereg is at `29717ba`; the runs were at `ab2eeb5`, whose added commits touch none of
  S4F-1's sources. The approval is the user's same-day approval of 12:07 KST.
- **LOW-3:** timeouts were 0 of 80 emissive against 6 of 80 lit. This is descriptive only.
- **LOW-4:** attempt01 took 2 h 22 min. Gzip took 1.64 GB of ULogs down to 0.55 GB.
