# S-L1B independent review and report addendum — 2026-10-18

This file is dated after the `3ac6272` result commit. [`REPORT.md`](REPORT.md) is hashed in
[`index.json`](index.json) and is not edited. Where this addendum disagrees with the report, the
addendum supersedes it.

## Independent review

- **Reviewer:** a separate Claude subagent running the model `claude-fable-5-1` (Fable 5.1). It was
  started read-only and given no instructions to edit, commit or run GPU code. It re-ran the CPU
  tests and recomputed several quantities from the raw SL1B-C01 frames with its own code.
- **Verdict:** **GO-WITH-NOTES; no blocker.**
- **Checks it reported as passing:**
  - every formula matches the preregistration and amendment 1, as checked against
    `warp_lidar.py` and `lidar_bin_bearings`;
  - LiDAR row order: 74,298 of 74,304 delivered beams agree with the oracle beam truth;
  - oracle independence, including its own carve recomputation: 0 of 6×43×288 cells mismatched;
  - no gate was relaxed or made vacuous;
  - the off, s_l1a and s_l1b step logs match EL0-R1, SL1A-C01 and P1/P2 by SHA-256;
  - the GT firewall holds and the actor is 898-D;
  - the analysis re-run is byte-identical;
  - the raw manifest checks 138/138.

## Corrections and qualifications

1. **Target-return retention.** The report's "1–8 mm" came from the scratch mechanics smoke. It is
   not in `s_l1b_analysis.json`. On the official SL1B-C01 frames, 96 of the 102 target beams have
   a supported carve.
   - **Below s_L:** 78 of them, by 0.41–24.3 mm (median 1.89 mm).
   - **Carved:** 18, so 84 of 102 are retained as obstacles.
   - **Cause:** this is a structural property of the rule. The camera anchor is the minimum over
     pixel rays, and no pixel ray passes exactly through the sphere's nearest point, so the anchor
     is biased high. The bias grows with range. With zero tolerance below s_L, most target-centre
     returns stay. Amendment 1 removed none of these.
   - **Status:** retention was preregistered as reported-only, so no gate changes. Any fix, for
     example an analytic pixel-pitch bound, needs its own preregistered amendment. This is an
     inherited Known limitation of `s_l1b_v1`.
2. **"Legacy misses 1–2 bins in 18 of 21 passes" is mostly definitional.**
   - **Missing bins:** 21 of the 24 have a bin-centre bearing outside the bar's azimuth extent; the
     other 3 already held the bar through LiDAR.
   - **Genuine evidence:** only the B_TWO extra bin is real A1 evidence on the GPU. The GPU half of
     B-A1A2 largely repeats B-ENVELOPE.
   - **Where A1/A2 is shown:** the decisive evidence is the CPU fixtures in
     `tests/test_s_l1b_runtime.py`. A 0.1 m bar at 20°/6 m lands in different bins under legacy
     than in truth, and the on-axis front-plane error is −0.0998 m. The reviewer re-ran these tests.
3. **The three attitude passes are identical inputs.** Camera and LiDAR are yaw-only, so the depth
   and LiDAR of the level, roll15 and pitch15 passes are bitwise equal. The unique counts are a
   third of the reported totals:

   | Quantity | Reported | Unique |
   | --- | --- | --- |
   | B-POINT pixels | 137,862 | 45,954 |
   | Robust bins | 148,608 | 49,536 |
   | Bar beams | 843 | 281 |
   | Target beams | 102 | 34 |

   Gates with 0 mismatches are unaffected. Rotation R ≠ I is exercised only by CPU fixture F4.
4. **GPU B-VERTICAL and amendment 1 are CPU evidence in practice.**
   - **Bins:** on the GPU scenes, strict bins equal clamped bins in every cell. The scenes contain
     only vertical bars, and their out-of-band points are always farther than in-band ones.
   - **Amendment 1:** it changed 0 GPU carve cells.
   - **Where these are shown:** CPU F3 and B_TIGHT are the evidence for both.
5. **Reviewer item not confirmed.** The reviewer said the carve's azimuth window is centred on a
   camera-frame bearing, `atan(−(u−cx)/fx)`. That expression is in `_record_detector_profile`, a
   diagnostic. The bearing that reaches `_fuse_static_and_extract_obstacles` comes from
   `navrl_perception.py:1733`: `bearing = atan2(m_y, m_x)` with `m = camera_offset + ray·centre_range`.
   That is a body-origin bearing that includes the camera translation. The remaining error comes only
   from the A7 centre-range bias, which is radial; with an on-axis 0.10 m offset it changes the
   bearing negligibly. This disagreement is recorded and is not adopted as a finding.
6. **Minor points.**
   - **Elevation span:** the amendment-1 eligibility uses the 0.15 m visual silhouette span, which is
     narrower than the 0.20 m proxy. This is stricter, as recorded.
   - **Mis-attribution:** the report credits amendment 1 for S-L1A carving the B_TIGHT 2.77 m bar.
     That carve is the A2 frame offset; amendment 1 concerns the ±10°/+20° rows.
   - **"Legacy" carve row:** it is legacy rules replayed on S-L1A depth inside the s_l1b process.
   - **Dropped beams:** the analysis drops oracle/semantic-disagreeing beams without reporting the
     fraction. The reviewer measured 6 of 951 returns, all S09 at 9.9 m with a 1.2e−4 m float32
     offset. Future analyses report this fraction.
   - **Cost:** S-L1B fusion is about 4× slower (17.2 ms against 4.4 ms for 43 envs).

## Decision

**S-L1B: GO.** All preregistered gates pass with independent CPU and GPU evidence. The
lower-lineage regression passes, original integrity passes, and the source SHA is fixed at
`a1f97e9`. There is no unresolved blocker. The S-L1C preregistration carries items 1 and 3 as
inherited limitations, and no S-L1C gate depends on the S-L1B carve.
