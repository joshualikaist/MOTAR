# S3a-P1 — policy precision control, CPU preparation and preflight, 2026-11-06

**Status:** CPU implementation, CPU preflight and independent reviews. **The GPU conditions are NOT_RUN.** They need
the user's scoped approval bound to the clean commit that adds this folder. S3a's historical verdict stays
NOT_EQUIVALENT; no tolerance, weight or formal gate changes.

Read the [preregistration](../../../../../plans/prereg_sitl_s3a_policy_precision_v1_2026-11-06.md) first.

## CPU preflight

The CPU condition and pairing controls ran on the stored G8 r2 grid: 84 calls × 128 environments × 4 action
dimensions per density, in batches of one call's 128 environments.
- Command: `tools/sitl_s3a_policy_precision.py --cpu-only`.
- Environment: CUDA hidden, cores 8–11, `nice -n 10`, one thread.
- Record: [`cpu_preflight_v4.json`](cpu_preflight_v4.json), a byte-identical copy of the external
  `cpu_only_v4/result.json`, which ran the committed tool bytes (hash in [`validation.json`](validation.json)).

| Density | CPU vs recorded GPU, max | at (call, env, dim) | components > 1e-4 | shifted-pairing control, max |
| --- | ---: | --- | ---: | ---: |
| 70 bars | 1.7202e-3 | (79, 119, 1) | 8453 of 43008 | 1.93 |
| 205 bars | 1.7297e-3 | (28, 34, 1) | 8468 of 43008 | 1.93 |

- The known residual is present on the full grid. The largest error is in action dimension 1, the lateral
  component with F's 0.4 mu scale.
- The positive control (true pairing < 1e-2) and the negative control (shifted pairing ≥ 1e-2) both pass.
- The kernel fault count was 0 before and after.
- The earlier preflights gave the same action npz bytes (`83c26c21…`):
  - `cpu_only_v1` (review-r1 bytes);
  - `cpu_only_v2` (review-r3 bytes);
  - `cpu_only_v3` (review-r4 bytes).
- The prior audit's smaller figures came from envs 0–7 before their first reset.
- These numbers do not decide the preregistered reading. That needs the GPU_ON and GPU_OFF conditions.

## Reviews (Claude Opus 5.5, fresh context each round)

- [r1](REVIEW_r1.md): FAIL, 1 MAJOR. The GPU run was not single use, the tool kept no fault-count record, and
  `--out` could point inside the worktree.
- [r2](REVIEW_r2.md): PASS_WITH_LIMITS. Its MINORs were repaired before commit:
  - the attempt record;
  - checks before the run directory is created;
  - the MIXED reading;
  - preflight provenance;
  - no `.pyc` written by test children.
- [r3](REVIEW_r3.md): PASS_WITH_LIMITS. Its MINOR N12 (per-density labels when no cause is assigned) and LOW N11/N14 were repaired before commit.
- [r4](REVIEW_r4.md): PASS_WITH_LIMITS. This round was shared with the geometry unit. LOW B4-l1 to B4-l3 were
  repaired before commit:
  - an ASCII GPU index;
  - Policy F's pin checked before the run directory;
  - refusal tests that assert their reason.
- [r5](REVIEW_r5.md): PASS_WITH_LIMITS, 0 MINOR. The LOWs N13 (wording), N14 and B5-l1 (an unreachable git failure
  after `mkdir`) remain open as stated.
