# Project state

A short index of where MOTAR stands now. It is not a log; history lives in [`HISTORY.md`](HISTORY.md),
[`WORKLOG.md`](../WORKLOG.md) and the evidence folders. Established results and their limits live in
[`EVIDENCE.md`](EVIDENCE.md); what the project is lives in [`PROJECT.md`](../PROJECT.md). Every status
below names its record. Where this file and a record disagree, the record is right.

**Last verified:** 2026-10-19, on `codex/mt-p1` at `2e77e57`. The check covered Git state, the evidence
folders listed below and the Beads issue list. No simulator or GPU run was part of this check.

## In one line

MOTAR is a simulation study of an aerial pursuer that tracks a moving target with RGB-D camera +
LiDAR fusion, using one frozen policy (Policy F). It asks what the perception, safety and
target-motion conditions do to capture, and which corrections help.

## What is working and verified

Each item is verified only within the scope stated in its record.

| Area | Status | Record |
| --- | --- | --- |
| Sensor contract | RGB-D camera + LiDAR fusion is the official contract | [`SENSOR_CONTRACT.md`](SENSOR_CONTRACT.md) |
| Environment qualification | EQ-0 → EQ-1D → E-L0 complete; the environment verdict is LIMITED_VALIDITY | [`evidence/eq1d_qualification_2026-10-18/`](evidence/eq1d_qualification_2026-10-18/REPORT.md) |
| Occlusion study | OCC-P2 complete; RQ1/RQ3 supported, RQ2 inconclusive | [`evidence/occ_p2_confirmatory_2026-10-17/`](evidence/occ_p2_confirmatory_2026-10-17/REPORT.md) |
| Sensor-fusion corrections | S-L1A/B/C/D v1 GO; C-v2 GO in a limited scope | [`evidence/s_l1c_v2_2026-10-18/`](evidence/s_l1c_v2_2026-10-18/QUALIFICATION.md) |
| H-R3 mechanics | P03-Q01 GO for qualification only | [`evidence/h_r3_p03_preparation_2026-10-19/`](evidence/h_r3_p03_preparation_2026-10-19/REPORT.md) |
| KROC2027 study | H-A supported; H-A2/I_A and H-B inconclusive | [`evidence/mt_precision_review_2026-10-16/`](evidence/mt_precision_review_2026-10-16/README.md) |

## Current branch and phase

- **Work branch:** `codex/mt-p1`. It is a short-lived worktree branch and is not yet merged into
  `main`. A merge needs user review (`motar-lp5.2`).
- **Phase:** KROC2027 study. The Simulation Freeze is 2026-10-28. The H-R3 line runs in parallel and
  stays PROVISIONAL.

## Known problems and blockers

- **H-R3 attribution is INCONCLUSIVE.** Confirmation is on HOLD until the estimand is chosen
  (`motar-6nn.1`).
- **Stale docs.** `PROJECT.md` (phase dated 2026-09-26), `AGENTS.md` and `OPERATIONS.md` still say no
  GPU run is authorised. They predate the October approved runs (`motar-lp5.4`).
- **TF32 caveat missing from public claims.** Historical moving-target cells are TF32-exposed, and the
  public target-motion claims do not say so yet (`motar-lp5.3`).
- **No physical-safety claim.** P-L1 and G-L1 are not done (`motar-tbx`).
- **No external backup.** Raw results are local only; the user declined an external backup (see
  [`DECISIONS.md`](DECISIONS.md)).

## Recent decisions

Full list with reasons: [`DECISIONS.md`](DECISIONS.md).

- **2026-10-19:** Beads is the only tracker for actionable TODOs; Claude-Mem is reference memory.
- **2026-10-19:** BACKUP_DECLINED_BY_USER. Raw stays local; no deletion follows from this.
- **2026-10-19:** H-R3 P03-Q01 is GO for qualification only; no automatic pilot.
- **2026-10-18:** the sensor contract is RGB-D + LiDAR fusion.
- **2026-10-14:** Policy F is the only policy; no new PPO.

## Next priorities

Run `bd ready` for the live list. At the time of writing, the top items were:

- `motar-d43.1`: write the KROC2027 paper draft (internal deadline 2026-11-13).
- `motar-6nn.1`: the user's choice of H-R3 estimand.
- `motar-lp5.1`: push the unpublished `codex/mt-p1` commits (needs push approval).
- `motar-lp5.4`: sync the stale canonical docs.
- `motar-lp5.3`: add the TF32 qualification to the public target-motion claims.

## Where things are

- **Commands, test profiles and run rules:** [`OPERATIONS.md`](OPERATIONS.md). The default check is
  `python -B tools/run_tests.py`.
- **Plans:** [`plans/`](plans/). **Evidence:** [`evidence/`](evidence/). **Long-term direction:**
  [`ROADMAP.md`](ROADMAP.md).
- **Tasks:** Beads (`bd ready`, `bd show <id>`). The database is untracked and local, in the main
  checkout's `.beads/`.
