# S3c v1 and S4 v1 — amendment 1 (2026-11-07, before either unit ran)

Amends [S3c v1](prereg_sitl_s3c_v1_2026-11-06.md) and [S4 v1](prereg_sitl_s4_v1_2026-11-06.md). Neither unit has
run. The repair was reviewed (Claude Opus 5.5, review of 2026-11-06: PASS), and its record is in
`docs/evidence/camonly_2026-11-04/camk1/REVIEW_r1.md` (part 1).

## What changes and why

1. **PX4's `rootfs/dataman` is deleted after each episode** (`remove_dataman`, in `tools/sitl_s3b_formal.py`, used by
   S3c, and in `tools/sitl_s4.py`).
   - **What it is:** PX4's mission and fence storage, 7,866,640 bytes per boot, not flight data. Nothing hashes or
     cites it: not the session record, not the runtime binding, not any preregistration.
   - **When:** it is removed after the owned session has ended. The bytes removed are recorded per episode as
     `dataman_removed_bytes`.
   - **Why:** the disk had 3.2–4.7 GB free for about 140 more episodes.
   - S3b's attempt01 ran before this change and kept its dataman files.
2. **S4's CUDA-job refusal allows exactly `/usr/share/rustdesk/rustdesk`.**
   - That is the user's remote desktop, which appears in `nvidia-smi --query-compute-apps` because it encodes the
     screen on the GPU.
   - Any other listed process still refuses the launch.
   - A failing `nvidia-smi` (non-zero exit) also refuses, so the check cannot pass vacuously.
3. **Dates:** both units run on 2026-11-07, after S3b's attempt01 (2026-11-06). The approvals are re-issued for the
   commit that holds this amendment.

Nothing else changes: scenes, arms, validity, stop rules and the reported quantities are as preregistered.
