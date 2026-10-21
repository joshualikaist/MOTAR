# G4-R7 handoff

> **Update 2026-10-21 (supervisor fix, before any execution).** An independent review found that
> `verify` read only worker receipts, so a stage the supervisor had stopped (e.g.
> `DEVICE_VRAM_CEILING`) could still give PASS. Fixed in `6c9bc81` and hardened in `453bd12`
> (final source): every stage needs a valid COMPLETED supervisor receipt; otherwise
> `STOPPED_INTEGRITY_NOT_EVALUATED`, with no SUPPORTED claim. Fable delta and micro-reviews:
> PASS_WITH_LIMITATIONS, no P0/P1/P2 (3 P3 notes). Current receipt:
> [validation_supervisor_fix.json](validation_supervisor_fix.json); `validation.json` below is
> superseded for the verifier. The rest of this handoff still applies.

2026-10-21 KST, `codex/mt-p1`. Local commits only; nothing pushed or merged. GPU NOT_RUN. G5 HOLD.

## State

- Source: `1efbcf8` (S1_NATIVE baseline, shared host trace, CPU `verify`, Q01-V2 plan) and
  `e6db0f1` (follow-up to the Fable re-review: seed attestation, receipt row counts, explicit
  mismatch tags, terminal-coverage claim, `sim_steps`, real-SIGTERM test) and `a0840c2`
  (delta-review NOTE fixes; final source). The evidence commit containing this file sits on top
  and changes no code.
- Receipt: [validation.json](validation.json). Report: [IMPLEMENTATION_REPORT.md](IMPLEMENTATION_REPORT.md).
  Git: [GIT_CLEANUP_REPORT.md](GIT_CLEANUP_REPORT.md). Commands: [GPU_SHADOW_RUNBOOK.md](GPU_SHADOW_RUNBOOK.md).
- Fable 5.1: full re-review of `1efbcf8` PASS_WITH_LIMITATIONS (no P0/P1);
  delta review of `1efbcf8..e6db0f1` PASS (no P0/P1/P2; seed 4901 confirmed); micro-review of
  `e6db0f1..a0840c2` PASS. The final source has a completed independent review.
- Q01 v1 is withdrawn before execution; its files are unchanged and its output root never existed.

## To run Q01-V2 (only after explicit GPU approval of the exact HEAD)

Follow the runbook: preflight on the approved HEAD, then `campaign` (or the four stages and
`verify`). Caps: 20/200/200/200 actions, 180/300/300/300 s, 1080 s total, 256 MB output, stop
below 3 GB free. No retry; an existing RunID path is never reused. No tracked edit during the run.

## Open items

- `motar-3p3.22` Q01-V2 GPU run — awaits user approval.
- `motar-6nn.5` PUBLIC_CPU's 4 pre-existing `DetectReduction` errors (aggregate FAIL).
- `motar-3p3.13`, `motar-3p3.15` BOIA P0 model/sensor blockers; G5 HOLD regardless of Q01-V2.
- User decisions, not touched: main's WIP `.gitignore` change (`*.pyc` → `c*.pyc`, looks
  accidental), the 78 presentation files and `tests/test_professor_presentation.py` in main,
  and opening the research window from `MOTAR-mt-p1-research.code-workspace`.
- `~/.cache/uv` grew 785 MB today from the claude-mem plugin; not removed.

No owned process is left running.
