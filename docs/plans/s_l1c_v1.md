# S-L1C v1 — target identity and range-only covariance (preregistration)

Status: preregistered after the recorded S-L1B GO decision (`2389a41`) and before any S-L1C runtime
code or measurement. The machine-readable contract is
[`s_l1c_v1.json`](s_l1c_v1.json), and it governs if the two texts differ. No gate may be relaxed
to reach a pass.

## Problem

Three EQ findings motivate this lineage:

- **Identity (EQ-1A A6, EQ-1D D2-N2).** A track primed where a target would stand in front of a
  bar accepts that bar's LiDAR return as the target. This happened in **7 of 7** rendered
  bar-only fixtures. Each acceptance also resets the track clock (EQ-1C T3), so the track stays
  alive with no identity evidence.
- **Covariance (EQ-1C T4).** Policy F's LiDAR update applies a diagonal world-axis covariance to a
  point whose bearing and height are the track's own prediction. One range return cut the
  tangential variance from 1.0 to 0.022 and the vertical variance from 1.0 to 0.038.
- **Radius (EQ-1A A4).** The association adds the camera's 0.15 m to a return from the LiDAR's
  0.20 m proxy sphere, a −0.050 m centre bias.

## Correction (cumulative `s_l1c_v1` over S-L1A + S-L1B)

1. **Camera vs LiDAR measurements.** A camera measurement carries identity, from appearance plus
   depth. A LiDAR association carries range only and no identity.
2. **Radius (C1).** The association uses the LiDAR proxy radius of 0.20 m in both directions.
   The camera centre-range bias (A7) is unchanged and stays a Known FAIL.
3. **Row eligibility (C2).** A row can measure the target only if its elevation lies within
   asin(0.20/predicted range) of the predicted target elevation. The measurement uses only the
   eligible rows of the column nearest the predicted bearing. If no row is eligible, there is no
   association.
4. **Shape consistency (C3).** This uses only LiDAR ranges and the target's declared size:
   - **Rule.** Reject the association when a vertically adjacent row in the same column returns a
     horizontal range within 0.10 m of the matched return. Beyond 1.152 m, a vertical surface can
     do that and a 0.20 m sphere cannot.
   - **Short range.** Below 1.152 m the test is inconclusive and the association is rejected.
   - **Scope.** Real target returns are therefore still accepted. LiDAR target measurement is
     not disabled.
5. **Range-only model (C4).**
   - **Measurement point:** the origin plus (surface + 0.20)·u, where u points to the predicted
     centre.
   - **Noise:** R = 0.08²·uuᵀ + 50²·(I − uuᵀ), the exact EKF with H = uᵀ.
6. **Identity clock (C5).** The tracker's fused-recency age and its 5 s expiry are unchanged. A new
   diagnostic `identity_age` resets only on camera updates. A per-step `update_source` records
   none, camera or lidar_range. Both are evaluator-facing only and are not actor inputs.
7. **Reacquisition.** An identity-confirmed reacquisition is a camera update after
   identity_age > 0.5 s. A LiDAR-only update is fused-valid but is not reacquisition.

## Fixtures

**Negatives.** The 7 D2-N2 bar-only scenes (N1, N2, N3, N5, N6, N7, N8), primed exactly as in
EQ-1D.

**Positives.** Every target scene where the column nearest the target has an oracle row that
hits the unoccluded 0.20 m proxy, and the predicted range is ≥ 1.152 m. Each is primed at the true
centre and at ±0.15 m range offsets, with the camera lost.

**CPU exact cases** (scans from the float64 oracle):

- tall bars at 2–11 m;
- targets at 1.5–11.5 m, including one 0.3 m in front of a bar and one between bars;
- covariance at bearings 0/45/90° and elevations 0/8°.

## Gates

| Gate | Requirement |
| --- | --- |
| C-NEG | 0 of 7 bar-only negatives accepted, on CPU and GPU. Legacy's 7/7 is reported. |
| C-POS | 100% recall on the preregistered eligible positives, on CPU and GPU. Centre error ≤ 1e−3 m on beam-centred GPU targets. |
| C-COV | After one range-only update, orthogonal variances stay ≥ 0.99 of the prior and the variance along u decreases. |
| C-PROV | Camera updates reset identity_age; lidar_range updates never do. The fused-age expiry contract is unchanged. |
| C-RADIUS | Analytic centre error ≤ 1e−4 m on CPU; ≤ 1e−3 m on GPU beam-centred targets. |
| C-REGRESSION | CPU: off, s_l1a and s_l1b are bitwise equal to `a1f97e9`. GPU: off = EL0-R1, s_l1a = SL1A-C01, s_l1b = SL1B-C01, on all 14 digest keys. |
| C-EL0 | The s_l1c E-L0 P1/P2 digests are identical. |
| C-FIREWALL | Association code reads only LiDAR, the camera flag, its own track state and declared constants. GT is used only by the evaluator and for fixture priming. The actor stays at 898 dimensions. |
| C-INTEGRITY | Checkpoint, RMS, import origins, frozen tree and manifests are valid, with no NaN/Inf. |

**QUALIFIED** requires every gate above. A metric that improves while another gate fails is
NO-GO. Rejected false matches and legitimate recall are reported for every mode. Closed-loop
update-source counts and ON changes are reported with no performance claim.

## Inherited limitations (from the S-L1B review)

- s_l1b_v1's carve keeps about 82% of target-centre LiDAR returns as obstacles. The pixel-sampled
  anchor is biased high and there is zero tolerance below s_L. No S-L1C gate depends on the carve.
- GPU attitude passes are identical yaw-only inputs. Unique counts, a third of the totals, are
  reported next to totals.
- The camera centre-range bias (A7) is unchanged.

## Known limits (declared)

- A 3 m bar (vehicle frame z −1…+2 m at the 1 m flight altitude) whose front face is beyond about
  11.35 m returns only the 0° row before the 12 m LiDAR limit and can look the same as the target. The shape test cannot reject such a bar, so that residual false-association
  band is reported, not gated.
- Below 1.152 m, LiDAR association is refused, which costs some recall there.
- The test assumes vertical-faced bars, which is how the environment builds obstacles.

## Execution

The canary is `SL1C-C01`. It runs:

- three sensor processes (legacy, s_l1b, s_l1c) on the 43 S-L1B scenes;
- ten closed-loop processes covering static/moving × {off, s_l1a, s_l1b, s_l1c P1, s_l1c P2}.

The watchdogs, launch-time GPU cap, 8 GB disk floor, no-retry rule and stop on non-interference
failure are the same as for S-L1B.
